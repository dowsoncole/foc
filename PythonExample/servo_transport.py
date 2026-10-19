"""RS485 V2 multi-mode waveform transport.

Implements the variable-length protocol documented by the firmware in
``MDK-ARM/docs/pc_waveform_protocol.md``.  Frames carry one to four channel
IDs, so decoded values always retain their identity and engineering unit.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Sequence

FRAME_HEADER = b"\xA5\x5A"
PROTOCOL_VERSION = 2
MIN_CHANNELS = 1
MAX_CHANNELS = 4
CRC8_POLYNOMIAL = 0x07
DEFAULT_BAUDRATE = 921600
SAMPLE_RATE_HZ = 1000
ADC_SAMPLE_RATE_HZ = 20_000
ADC_SAMPLES_PER_FRAME = ADC_SAMPLE_RATE_HZ // SAMPLE_RATE_HZ
SERIAL_READ_TIMEOUT_S = 0.05
SERIAL_READ_CHUNK = 4096

MODE_NAMES = {
    0: "关闭",
    1: "电压开环",
    2: "电流开环",
    3: "自定义",
}


@dataclass(frozen=True)
class ChannelDefinition:
    channel_id: int
    name: str
    unit: str


CHANNEL_DEFINITIONS = {
    item.channel_id: item
    for item in (
        ChannelDefinition(1, "U相电流", "A"),
        ChannelDefinition(2, "V相电流", "A"),
        ChannelDefinition(3, "W相电流", "A"),
        ChannelDefinition(4, "D电流指令", "A"),
        ChannelDefinition(5, "D电流反馈", "A"),
        ChannelDefinition(6, "Q电流指令", "A"),
        ChannelDefinition(7, "Q电流反馈", "A"),
        ChannelDefinition(8, "D电压输出", "pu"),
        ChannelDefinition(9, "Q电压输出", "pu"),
        ChannelDefinition(10, "U相占空比", ""),
        ChannelDefinition(11, "V相占空比", ""),
        ChannelDefinition(12, "W相占空比", ""),
        ChannelDefinition(13, "电角度", "rad"),
        ChannelDefinition(14, "直流母线电压", "V"),
    )
}


class TransportError(RuntimeError):
    """Raised when a waveform source cannot be opened or configured."""


def frame_length(channel_count: int) -> int:
    if not MIN_CHANNELS <= channel_count <= MAX_CHANNELS:
        raise ValueError(f"通道数量必须为 {MIN_CHANNELS}～{MAX_CHANNELS}")
    return 12 + 5 * channel_count


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (((crc << 1) ^ CRC8_POLYNOMIAL) if crc & 0x80 else crc << 1) & 0xFF
    return crc


def build_frame(
    mode: int,
    frame_sequence: int,
    adc_sequence: int,
    channels: Sequence[tuple[int, float]],
) -> bytes:
    """Build a V2 frame from ``(channel_id, engineering_value)`` pairs."""
    if mode not in MODE_NAMES:
        raise ValueError(f"未知波形模式：{mode}")
    count = len(channels)
    frame_length(count)
    ids = bytes(channel_id for channel_id, _ in channels)
    if any(channel_id not in CHANNEL_DEFINITIONS for channel_id, _ in channels):
        raise ValueError("包含未知通道 ID")
    raw_values = [int(round(value * 1000.0)) for _, value in channels]
    body = struct.pack(
        "<2sBBBHI",
        FRAME_HEADER,
        PROTOCOL_VERSION,
        mode,
        count,
        frame_sequence & 0xFFFF,
        adc_sequence & 0xFFFFFFFF,
    ) + ids + struct.pack("<" + "i" * count, *raw_values)
    return body + bytes((crc8(body),))


@dataclass(frozen=True)
class WaveformChannel:
    channel_id: int
    value: float

    @property
    def definition(self) -> ChannelDefinition:
        return CHANNEL_DEFINITIONS[self.channel_id]

    @property
    def name(self) -> str:
        return self.definition.name

    @property
    def unit(self) -> str:
        return self.definition.unit


@dataclass(frozen=True)
class WaveformFrame:
    mode: int
    frame_sequence: int
    adc_sequence: int
    channels: tuple[WaveformChannel, ...]

    @property
    def mode_name(self) -> str:
        return MODE_NAMES[self.mode]

    @property
    def channel_ids(self) -> tuple[int, ...]:
        return tuple(channel.channel_id for channel in self.channels)

    @property
    def values(self) -> tuple[float, ...]:
        return tuple(channel.value for channel in self.channels)


@dataclass
class SourceStats:
    frames: int = 0
    crc_errors: int = 0
    protocol_errors: int = 0
    resync_bytes: int = 0
    sequence_gaps: int = 0
    adc_discontinuities: int = 0
    dropped_samples: int = 0
    last_sequence: int | None = field(default=None)
    last_adc_sequence: int | None = field(default=None)
    last_mode: int | None = field(default=None)

    def reset(self) -> None:
        for name in vars(self):
            setattr(self, name, None if name.startswith("last_") else 0)

    def summary(self) -> str:
        mode = "--" if self.last_mode is None else MODE_NAMES.get(self.last_mode, str(self.last_mode))
        return (
            f"帧 {self.frames}   模式 {mode}   CRC错 {self.crc_errors}   "
            f"协议错 {self.protocol_errors}   重同步 {self.resync_bytes}B   "
            f"跳帧 {self.sequence_gaps}   ADC异常 {self.adc_discontinuities}   "
            f"丢弃 {self.dropped_samples}   seq {self.last_sequence}   adc {self.last_adc_sequence}"
        )

    def compact_summary(self) -> str:
        mode = "--" if self.last_mode is None else MODE_NAMES.get(self.last_mode, str(self.last_mode))
        return (
            f"帧 {self.frames} | {mode} | CRC {self.crc_errors} | 协议 {self.protocol_errors} | "
            f"跳 {self.sequence_gaps} | ADC {self.adc_discontinuities} | 丢 {self.dropped_samples}"
        )


class WaveformParser:
    """Incrementally decode V2 variable frames and recover one byte at a time."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = SourceStats()
        self._last_sequence: int | None = None
        self._last_adc_sequence: int | None = None

    def feed(self, chunk: bytes) -> list[WaveformFrame]:
        if chunk:
            self._buffer.extend(chunk)
        frames: list[WaveformFrame] = []
        while True:
            header_index = self._buffer.find(FRAME_HEADER)
            if header_index < 0:
                self._discard_without_header()
                break
            if header_index:
                self.stats.resync_bytes += header_index
                del self._buffer[:header_index]
            if len(self._buffer) < 5:
                break
            version, mode, count = self._buffer[2:5]
            if (
                version != PROTOCOL_VERSION
                or mode not in MODE_NAMES
                or not MIN_CHANNELS <= count <= MAX_CHANNELS
            ):
                self._reject_protocol_byte()
                continue
            length = frame_length(count)
            if len(self._buffer) < length:
                break
            candidate = bytes(self._buffer[:length])
            if crc8(candidate[:-1]) != candidate[-1]:
                self.stats.crc_errors += 1
                self.stats.resync_bytes += 1
                del self._buffer[0]
                continue
            channel_ids = tuple(candidate[11:11 + count])
            if any(channel_id not in CHANNEL_DEFINITIONS for channel_id in channel_ids):
                self._reject_protocol_byte()
                continue
            frame_sequence = struct.unpack_from("<H", candidate, 5)[0]
            adc_sequence = struct.unpack_from("<I", candidate, 7)[0]
            values = struct.unpack_from("<" + "i" * count, candidate, 11 + count)
            del self._buffer[:length]
            frame = WaveformFrame(
                mode,
                frame_sequence,
                adc_sequence,
                tuple(
                    WaveformChannel(channel_id, raw_value / 1000.0)
                    for channel_id, raw_value in zip(channel_ids, values)
                ),
            )
            self._update_stats(frame)
            frames.append(frame)
        return frames

    def _reject_protocol_byte(self) -> None:
        self.stats.protocol_errors += 1
        self.stats.resync_bytes += 1
        del self._buffer[0]

    def _discard_without_header(self) -> None:
        keep = 1 if self._buffer[-1:] == FRAME_HEADER[:1] else 0
        discarded = len(self._buffer) - keep
        if discarded:
            self.stats.resync_bytes += discarded
            del self._buffer[:discarded]

    def _update_stats(self, frame: WaveformFrame) -> None:
        self.stats.frames += 1
        if self._last_sequence is not None:
            if ((frame.frame_sequence - self._last_sequence) & 0xFFFF) != 1:
                self.stats.sequence_gaps += 1
        if self._last_adc_sequence is not None:
            if ((frame.adc_sequence - self._last_adc_sequence) & 0xFFFFFFFF) != ADC_SAMPLES_PER_FRAME:
                self.stats.adc_discontinuities += 1
        self._last_sequence = frame.frame_sequence
        self._last_adc_sequence = frame.adc_sequence
        self.stats.last_sequence = frame.frame_sequence
        self.stats.last_adc_sequence = frame.adc_sequence
        self.stats.last_mode = frame.mode

    def reset(self) -> None:
        self._buffer.clear()
        self.stats.reset()
        self._last_sequence = None
        self._last_adc_sequence = None


def parse_baudrate(text: str) -> int:
    value = text.strip().lower()
    for suffix, multiplier in (("mbps", 1_000_000), ("kbps", 1_000), ("bps", 1)):
        if value.endswith(suffix):
            try:
                return int(float(value[: -len(suffix)].strip()) * multiplier)
            except ValueError as exc:
                raise TransportError(f"无法解析波特率：{text}") from exc
    raise TransportError(f"无法解析波特率：{text}")


def list_serial_ports() -> list[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [
        f"{port.device} - {port.description}" if port.description else port.device
        for port in sorted(list_ports.comports(), key=lambda item: item.device)
    ]


class SerialWaveformSource:
    BUFFER_SAMPLES = 4000

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE) -> None:
        self.port, self.baudrate, self.rate_hz = port, baudrate, SAMPLE_RATE_HZ
        self._serial = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._samples: deque[WaveformFrame] = deque(maxlen=self.BUFFER_SAMPLES)
        self._lock = threading.Lock()
        self._parser = WaveformParser()
        self._dropped = 0

    @property
    def stats(self) -> SourceStats:
        with self._lock:
            snapshot = SourceStats(**vars(self._parser.stats))
            snapshot.dropped_samples = self._dropped
            return snapshot

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise TransportError("缺少 pyserial 依赖，请先执行：python -m pip install pyserial") from exc
        try:
            self._serial = serial.Serial(
                port=self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_S,
            )
        except Exception as exc:
            self._serial = None
            raise TransportError(f"无法打开串口 {self.port}：{exc}") from exc
        self._parser.reset()
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, name="rs485-waveform", daemon=True)
        self._thread.start()

    def _read_loop(self) -> None:
        serial_port = self._serial
        while not self._stop.is_set() and serial_port is not None:
            try:
                chunk = serial_port.read(SERIAL_READ_CHUNK)
            except Exception:
                break
            if not chunk:
                continue
            frames = self._parser.feed(chunk)
            if frames:
                with self._lock:
                    room = self.BUFFER_SAMPLES - len(self._samples)
                    if room < len(frames):
                        self._dropped += len(frames) - max(room, 0)
                    self._samples.extend(frames)

    def drain(self) -> list[WaveformFrame]:
        with self._lock:
            frames = list(self._samples)
            self._samples.clear()
        return frames

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


class SineWaveformSource:
    """Balanced three-phase V2 demo source using production encode/decode."""

    def __init__(self, rate_hz: int = SAMPLE_RATE_HZ, amplitude_a: float = 10.0) -> None:
        self.rate_hz, self.amplitude_a = rate_hz, amplitude_a
        self.port, self.baudrate = "SIMULATED", DEFAULT_BAUDRATE
        self._parser = WaveformParser()
        self._pending: deque[WaveformFrame] = deque(maxlen=4000)
        self._dropped = 0
        self._running = False
        self._emitted = 0
        self._last_sample_at = 0.0

    @property
    def stats(self) -> SourceStats:
        snapshot = SourceStats(**vars(self._parser.stats))
        snapshot.dropped_samples = self._dropped
        return snapshot

    def open(self) -> None:
        self._parser.reset()
        self._pending.clear()
        self._dropped = self._emitted = 0
        self._last_sample_at = time.perf_counter()
        self._running = True

    def emit(self, count: int) -> list[WaveformFrame]:
        if count <= 0:
            return []
        frames: list[WaveformFrame] = []
        for _ in range(count):
            phase = (self._emitted % self.rate_hz) / self.rate_hz * math.tau
            iu = self.amplitude_a * math.sin(phase)
            iv = self.amplitude_a * math.sin(phase - math.tau / 3.0)
            iw = -(iu + iv)
            encoded = build_frame(
                1,
                self._emitted,
                self._emitted * ADC_SAMPLES_PER_FRAME,
                ((1, iu), (2, iv), (3, iw)),
            )
            frames.extend(self._parser.feed(encoded))
            self._emitted += 1
        room = self._pending.maxlen - len(self._pending)
        if room < len(frames):
            self._dropped += len(frames) - max(room, 0)
        self._pending.extend(frames)
        if self._running:
            self._last_sample_at = time.perf_counter()
        return frames

    def drain(self) -> list[WaveformFrame]:
        if not self._running:
            return []
        now = time.perf_counter()
        due = int((now - self._last_sample_at) * self.rate_hz)
        if due > 0:
            self.emit(min(due, self._pending.maxlen))
        frames = list(self._pending)
        self._pending.clear()
        return frames

    def close(self) -> None:
        self._running = False
        self._pending.clear()


def estimate_frequency(values: Sequence[float], rate_hz: float) -> float:
    if len(values) < 3 or rate_hz <= 0:
        return 0.0
    crossings = [i for i in range(1, len(values)) if values[i - 1] < 0.0 <= values[i]]
    if len(crossings) < 2:
        return 0.0
    span = crossings[-1] - crossings[0]
    return 0.0 if span <= 0 else (len(crossings) - 1) * rate_hz / span


def to_scope_sample(frame: WaveformFrame) -> tuple[float, float, float, float]:
    values = list(frame.values)
    values.extend([0.0] * (MAX_CHANNELS - len(values)))
    return tuple(values[:MAX_CHANNELS])  # type: ignore[return-value]


def to_scope_samples(frames: Iterable[WaveformFrame]) -> list[tuple[float, float, float, float]]:
    return [to_scope_sample(frame) for frame in frames]
