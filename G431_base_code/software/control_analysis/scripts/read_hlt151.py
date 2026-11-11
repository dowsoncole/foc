"""Read-only HLT-151 torque/speed/power capture over Modbus RTU.

The HLT-151 is a sensor; this script cannot command dynamometer load or servo
motion. Raw 32-bit values are retained because the device's power scale needs
to be cross-checked against torque * speed on a rotating shaft.
"""

import argparse
import csv
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path

import serial


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def read_registers(port: serial.Serial, slave: int, address: int, count: int) -> bytes:
    payload = bytes((slave, 3, address >> 8, address & 255, count >> 8, count & 255))
    request = payload + crc16(payload).to_bytes(2, "little")
    port.reset_input_buffer()
    port.write(request)
    header = port.read(3)
    if len(header) != 3 or header[0] != slave or header[1] != 3:
        raise IOError(f"Unexpected Modbus header: {header.hex(' ')}")
    if header[2] != 2 * count:
        raise IOError(f"Unexpected byte count: {header[2]}")
    tail = port.read(header[2] + 2)
    if len(tail) != header[2] + 2:
        raise IOError(f"Short Modbus response: {len(tail)} bytes")
    response = header + tail
    if crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
        raise IOError(f"CRC mismatch: {response.hex(' ')}")
    return tail[:-2]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM11")
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--slave", type=int, default=1)
    ap.add_argument("--stopbits", type=int, choices=(1, 2), default=2)
    ap.add_argument("--gear-ratio", type=float, default=202.0,
                    help="motor turns per output-shaft turn")
    ap.add_argument("--rpm-scale", type=float, default=1.0,
                    help="physical shaft rpm per raw register count; verify on each installation")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = one sample")
    ap.add_argument("--interval", type=float, default=0.1)
    ap.add_argument("--csv", type=Path, help="optional raw and decoded log path")
    ap.add_argument("--quiet", action="store_true", help="write CSV without per-sample output")
    args = ap.parse_args()
    if (not 1 <= args.slave <= 247 or args.duration < 0 or
            args.interval <= 0 or not math.isfinite(args.gear_ratio) or
            args.gear_ratio <= 0 or not math.isfinite(args.rpm_scale) or
            args.rpm_scale <= 0):
        ap.error("Invalid slave, duration, interval, ratio or RPM scale")
    out = None
    writer = None
    count = 0
    errors = 0
    consecutive_errors = 0
    try:
        with serial.Serial(args.port, args.baud, bytesize=8, parity="N",
                           stopbits=args.stopbits, timeout=0.4, write_timeout=0.4) as port:
            # The 0x08 parameter is a 32-bit decimal-place count, default 2.
            decimals = int.from_bytes(read_registers(port, args.slave, 0x08, 2), "big")
            if decimals > 6:
                raise ValueError(f"Unreasonable torque decimals: {decimals}")
            start = time.monotonic()
            deadline = start + args.duration
            while True:
                timestamp = dt.datetime.now(dt.timezone.utc).isoformat()
                try:
                    # Three adjacent 32-bit values: torque (0x00), rpm (0x02),
                    # indicated power (0x04). Preserve the indicated power raw value.
                    data = read_registers(port, args.slave, 0, 6)
                except (IOError, serial.SerialException) as exc:
                    errors += 1
                    consecutive_errors += 1
                    print(f"HLT-151 read error {errors}: {exc}", file=sys.stderr)
                    if consecutive_errors >= 5:
                        raise
                else:
                    consecutive_errors = 0
                    torque_raw, rpm_raw, power_raw = (
                        int.from_bytes(data[offset:offset + 4], "big", signed=True)
                        for offset in (0, 4, 8)
                    )
                    torque_nm = torque_raw / 10 ** decimals
                    rpm = rpm_raw * args.rpm_scale
                    mechanical_w = torque_nm * rpm * 2 * math.pi / 60
                    row = dict(time_utc=timestamp, elapsed_s=time.monotonic() - start,
                               torque_raw=torque_raw, torque_decimals=decimals,
                               torque_nm=torque_nm, output_rpm_raw=rpm_raw,
                               rpm_scale=args.rpm_scale, output_rpm=rpm,
                               estimated_motor_rpm=rpm * args.gear_ratio,
                               indicated_power_raw=power_raw,
                               output_mechanical_w=mechanical_w)
                    if args.csv and out is None:
                        args.csv.parent.mkdir(parents=True, exist_ok=True)
                        out = args.csv.open("w", newline="", encoding="utf-8-sig")
                        writer = csv.DictWriter(out, fieldnames=row)
                        writer.writeheader()
                    if writer:
                        writer.writerow(row)
                        out.flush()
                    count += 1
                    if not args.quiet:
                        print(json.dumps(row, ensure_ascii=False), flush=True)
                if time.monotonic() >= deadline:
                    break
                time.sleep(args.interval)
    finally:
        if out:
            out.close()
        print(f"HLT-151: samples={count}, read_errors={errors}", file=sys.stderr)


if __name__ == "__main__":
    main()
