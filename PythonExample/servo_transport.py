"""Real RS485 waveform transport for the servo workstation.

The ``servo_v1`` firmware streams a binary waveform protocol on USART1
(RS485 half duplex, 921600 8-N-1). Each frame is eight bytes::

    A5 5A <channel> <sequence> <sample_lo> <sample_hi> <reserved> <crc8>

* ``crc8`` covers the first seven bytes with polynomial ``0x07``.
* ``sample`` is a signed 16-bit value. The firmware emits 1 kHz samples and
  completes exactly one sine cycle per second.

Every serial detail lives here so the Qt pages stay free of hardware code, as
the project README requires. Two interchangeable sources are provided:
``SerialWaveformSource`` reads a real COM port on a background thread, while
``SineWaveformSource`` synthesises the same frames so the display path can be
verified without hardware attached.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Sequence

FRAME_LENGTH = 8
FRAME_HEADER = (0xA5, 0x5A)
CRC8_POLYNOMIAL = 0x07
DEFAULT_BAUDRATE = 921600
SAMPLE_RATE_HZ = 1000
SERIAL_READ_TIMEOUT_S = 0.05
SERIAL_READ_CHUNK = 512
FULL_SCALE = 32768.0


class TransportError(RuntimeError):
    """Raised when a waveform source cannot be opened or configured."""


def crc8(data: bytes) -> int:
    """CRC-8 with polynomial 0x07, matching ``WaveformGenerator_Crc8()``."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLYNOMIAL) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(sequence: int, sample: int, channel: int = 1) -> bytes:
    """Assemble one firmware-format frame. Used by tests and the simulator."""
    sample &= 0xFFFF
    body = bytes((
        FRAME_HEADER[0],
        FRAME_HEADER[1],
        channel & 0xFF,
        sequence & 0xFF,
        sample & 0xFF,
        (sample >> 8) & 0xFF,
        0x00,
    ))
    return body + bytes((crc8(body),))


@dataclass(frozen=True)
class WaveformFrame:
    """One decoded waveform sample."""

    channel: int
    sequence: int
    sample: int

    @property
    def normalized(self) -> float:
        """Map the signed 16-bit sample onto the scope's +/-100 scale."""
        return self.sample / FULL_SCALE * 100.0


@dataclass
class SourceStats:
    """Counters shared by every waveform source."""

    frames: int = 0
    crc_errors: int = 0
    resync_bytes: int = 0
    sequence_gaps: int = 0
    dropped_samples: int = 0
    last_sequence: int | None = field(default=None)

    def reset(self) -> None:
        self.frames = 0
        self.crc_errors = 0
        self.resync_bytes = 0
        self.sequence_gaps = 0
        self.dropped_samples = 0
        self.last_sequence = None

    def summary(self) -> str:
        gaps = "" if self.last_sequence is None else f"  seq {self.last_sequence}"
        return (
            f"帧 {self.frames}   CRC错 {self.crc_errors}   "
            f"重同步 {self.resync_bytes}B   跳帧 {self.sequence_gaps}   "
            f"丢弃 {self.dropped_samples}{gaps}"
        )


class WaveformParser:
    """Incremental frame parser that resynchronises after any corruption.

    Feed it arbitrary byte chunks; it returns whole validated frames. A bad CRC
    discards a single byte and rescans, so one corrupted frame cannot desync the
    rest of the stream.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = SourceStats()
        self._last_sequence: int | None = None

    def feed(self, chunk: bytes) -> list[WaveformFrame]:
        if chunk:
            self._buffer.extend(chunk)
        frames: list[WaveformFrame] = []
        while True:
            header = self._find_header()
            if header < 0:
                break
            if len(self._buffer) - header < FRAME_LENGTH:
                break
            candidate = bytes(self._buffer[header:header + FRAME_LENGTH])
            if crc8(candidate[:7]) != candidate[7]:
                self.stats.crc_errors += 1
                del self._buffer[header]
                continue
            del self._buffer[header:header + FRAME_LENGTH]
            frame = WaveformFrame(
                channel=candidate[2],
                sequence=candidate[3],
                sample=int.from_bytes(candidate[4:6], "little", signed=True),
            )
            self.stats.frames += 1
            if self._last_sequence is not None:
                expected = (self._last_sequence + 1) & 0xFF
                if frame.sequence != expected:
                    self.stats.sequence_gaps += 1
            self._last_sequence = frame.sequence
            self.stats.last_sequence = frame.sequence
            frames.append(frame)
        return frames

    def _find_header(self) -> int:
        """Return the offset of the next header, discarding leading garbage."""
        length = len(self._buffer)
        # A complete header needs two bytes, so the last index to test is
        # length - 2. Testing length - 1 would read one byte past the buffer.
        index = 0
        limit = length - 2
        while index <= limit:
            if self._buffer[index] == FRAME_HEADER[0] and self._buffer[index + 1] == FRAME_HEADER[1]:
                if index:
                    self.stats.resync_bytes += index
                    del self._buffer[:index]
                return 0
            index += 1
        # Keep a trailing 0xA5: it may be the first half of a split header.
        if length and self._buffer[-1] == FRAME_HEADER[0]:
            if length > 1:
                self.stats.resync_bytes += length - 1
                del self._buffer[:-1]
            return -1
        if length:
            self.stats.resync_bytes += length
            self._buffer.clear()
        return -1

    def reset(self) -> None:
        self._buffer.clear()
        self.stats.reset()
        self._last_sequence = None


def parse_baudrate(text: str) -> int:
    """Convert UI labels such as ``921600 bps`` or ``1 Mbps`` into an integer."""
    value = text.strip().lower()
    for suffix, multiplier in (("mbps", 1_000_000), ("kbps", 1_000), ("bps", 1)):
        if value.endswith(suffix):
            number = value[: -len(suffix)].strip()
            try:
                return int(float(number) * multiplier)
            except ValueError as exc:
                raise TransportError(f"无法解析波特率：{text}") from exc
    raise TransportError(f"无法解析波特率：{text}")


def list_serial_ports() -> list[str]:
    """Return ``COM3 - USB Serial Port`` style entries, or [] without pyserial."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    entries = []
    for port in sorted(list_ports.comports(), key=lambda item: item.device):
        entries.append(f"{port.device} - {port.description}" if port.description else port.device)
    return entries


class SerialWaveformSource:
    """Reads the firmware waveform from a real COM port on a worker thread.

    The worker only parses and appends; the Qt thread drains the buffer from
    ``drain()``. ``deque`` append/popleft are atomic under the GIL and the lock
    keeps the statistics consistent.
    """

    BUFFER_SAMPLES = 4000

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE) -> None:
        self.port = port
        self.baudrate = baudrate
        self.rate_hz = SAMPLE_RATE_HZ
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
            raise TransportError(
                "缺少 pyserial 依赖，请先执行：python -m pip install pyserial"
            ) from exc
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_S,
            )
        except Exception as exc:
            self._serial = None
            raise TransportError(f"无法打开串口 {self.port}：{exc}") from exc
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._read_loop, name="rs485-waveform", daemon=True
        )
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
            if not frames:
                continue
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
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None


class SineWaveformSource:
    """Generates the firmware waveform locally, for hardware-free verification.

    It reproduces the firmware timing: 1 kHz samples, one sine cycle per second,
    an 8-bit rolling sequence number and valid CRC8 framing.
    """

    def __init__(self, rate_hz: int = SAMPLE_RATE_HZ, amplitude: int = 32767) -> None:
        self.rate_hz = rate_hz
        self.amplitude = amplitude
        self.port = "SIMULATED"
        self.baudrate = DEFAULT_BAUDRATE
        self._parser = WaveformParser()
        self._pending: deque[WaveformFrame] = deque(maxlen=4000)
        self._dropped = 0
        self._running = False
        self._clock = 0.0
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
        self._dropped = 0
        self._emitted = 0
        self._clock = time.perf_counter()
        self._last_sample_at = self._clock
        self._running = True

    def emit(self, count: int) -> list[WaveformFrame]:
        """Generate ``count`` frames deterministically, without a clock.

        Phase comes from the emit counter, so exactly one sine cycle spans
        ``rate_hz`` frames regardless of how fast the caller runs. ``drain()``
        uses this after converting elapsed time into a frame count, and tests
        call it directly.
        """
        if count <= 0:
            return []
        frames: list[WaveformFrame] = []
        for _ in range(count):
            phase = (self._emitted % self.rate_hz) / self.rate_hz * math.tau
            sample = int(self.amplitude * math.sin(phase))
            frames.append(
                WaveformFrame(1, self._emitted & 0xFF, max(-32768, min(32767, sample)))
            )
            self._emitted += 1
        room = self._pending.maxlen - len(self._pending)
        if room < len(frames):
            self._dropped += len(frames) - max(room, 0)
        self._pending.extend(frames)
        self._parser.stats.frames += len(frames)
        self._parser.stats.last_sequence = frames[-1].sequence
        return frames

    def drain(self) -> list[WaveformFrame]:
        if not self._running:
            return []
        now = time.perf_counter()
        period = 1.0 / self.rate_hz
        due = int((now - self._last_sample_at) / period)
        if due > 0:
            self._last_sample_at += due * period
            # A long stall must not produce a burst that floods the buffer.
            if now - self._last_sample_at > 1.0:
                self._last_sample_at = now
            self.emit(due)
        frames = list(self._pending)
        self._pending.clear()
        return frames

    def close(self) -> None:
        self._running = False
        self._pending.clear()


def estimate_frequency(values: Sequence[float], rate_hz: float) -> float:
    """Estimate signal frequency in Hz from rising zero crossings.

    Counting crossings over the whole window under-reads low frequencies, so
    the span between the first and last crossing is measured instead. Fewer
    than two crossings means the window holds no complete cycle and the
    frequency cannot be determined, so 0.0 is returned rather than a guess.
    """
    if len(values) < 3 or rate_hz <= 0:
        return 0.0
    crossings = [
        index for index in range(1, len(values))
        if values[index - 1] < 0.0 <= values[index]
    ]
    if len(crossings) < 2:
        return 0.0
    span_samples = crossings[-1] - crossings[0]
    if span_samples <= 0:
        return 0.0
    return (len(crossings) - 1) * rate_hz / span_samples


def to_scope_samples(frames: Iterable[WaveformFrame]) -> list[tuple[float, float, float, float]]:
    """Convert frames into the four-channel tuples ``ScopeChart`` expects.

    The firmware currently streams one channel, so it lands on CH1 and the
    remaining channels stay flat at zero.
    """
    samples = []
    for frame in frames:
        value = max(-100.0, min(100.0, frame.normalized))
        samples.append((value, 0.0, 0.0, 0.0))
    return samples
