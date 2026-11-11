"""Read-only AXF/HEX/BIN consistency check, including compressed RW load data.

Usage: python verify_encoder_artifacts.py firmware.axf firmware.hex firmware.bin
This checks files, not a board or Flash readback.
"""
from pathlib import Path
import struct
import sys


def load_image(path):
    data = Path(path).read_bytes()
    if data[:6] != b'\x7fELF\x01\x01':
        raise ValueError('Expected ELF32 little endian')
    offset = struct.unpack_from('<I', data, 28)[0]
    size, count = struct.unpack_from('<HH', data, 42)
    if size < 32:
        raise ValueError('Invalid ELF program header size')
    image = {}
    for index in range(count):
        kind, start, _, physical, length, _, _, _ = struct.unpack_from(
            '<8I', data, offset + index * size)
        if kind != 1 or not length:
            continue
        if not 0x08000000 <= physical < physical + length <= 0x08020000:
            raise ValueError('Load segment outside STM32G431VB Flash')
        segment = data[start:start + length]
        if len(segment) != length:
            raise ValueError('Truncated ELF segment')
        for delta, value in enumerate(segment):
            address = physical + delta
            if address in image and image[address] != value:
                raise ValueError('Conflicting ELF load segments')
            image[address] = value
    if not image or min(image) != 0x08000000:
        raise ValueError('Missing Flash load image')
    return image


def hex_image(path):
    image, upper, ended = {}, 0, False
    for line in Path(path).read_text(encoding='ascii').splitlines():
        if ended or not line.startswith(':'):
            raise ValueError('Invalid Intel HEX framing')
        record = bytes.fromhex(line[1:])
        if len(record) < 5 or len(record) != record[0] + 5 or sum(record) & 0xff:
            raise ValueError('Invalid Intel HEX length/checksum')
        length, address, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
        payload = record[4:4 + length]
        if kind == 0:
            for delta, value in enumerate(payload):
                absolute = upper + address + delta
                if absolute in image and image[absolute] != value:
                    raise ValueError('Conflicting HEX records')
                image[absolute] = value
        elif kind == 1:
            ended = True
        elif kind == 4:
            if length != 2:
                raise ValueError('Invalid HEX extended address')
            upper = int.from_bytes(payload, 'big') << 16
        elif kind != 5:
            raise ValueError(f'Unsupported HEX record type {kind}')
    if not ended:
        raise ValueError('Missing HEX EOF')
    return image


def verify(axf, hex_path, bin_path):
    expected = load_image(axf)
    base, end = min(expected), max(expected) + 1
    if len(expected) != end - base:
        raise ValueError('BIN check requires contiguous Flash load segments')
    actual = Path(bin_path).read_bytes()
    if actual != bytes(expected[address] for address in range(base, end)):
        raise ValueError('BIN differs from ELF load segment(s)')
    if hex_image(hex_path) != expected:
        raise ValueError('HEX differs from ELF load segment(s)')
    print(f'PASS: AXF/HEX/BIN match all {len(expected)} Flash load bytes '
          f'at 0x{base:08X}..0x{end - 1:08X}, including RW initialization')
    print('Artifact comparison only; no board download or Flash readback performed.')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    verify(*sys.argv[1:])
