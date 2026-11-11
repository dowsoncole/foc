"""Compare a GDB flash dump against ER_IROM1 in an ELF32 little-endian AXF.

Read-only verification; no debugger, download or motor control is performed.
Usage: py -3 verify_flash_image.py image.axf flash_readback.bin
"""
import struct
import sys
from pathlib import Path


def verify(image_path, readback_path):
    image = Path(image_path).read_bytes()
    assert image[:6] == b"\x7fELF\x01\x01", "Expected ELF32 little endian"
    section_offset = struct.unpack_from("<I", image, 32)[0]
    entry_size, count, string_index = struct.unpack_from("<HHH", image, 46)
    assert entry_size >= 40 and string_index < count
    sections = [struct.unpack_from("<10I", image, section_offset + i * entry_size)
                for i in range(count)]
    string_section = sections[string_index]
    strings = image[string_section[4]:string_section[4] + string_section[5]]
    matches = [s for s in sections
               if strings[s[0]:].split(b"\0", 1)[0] == b"ER_IROM1"]
    assert len(matches) == 1, "Missing or ambiguous flash section"
    section = matches[0]
    assert section[3] == 0x08000000, "Unexpected flash base address"
    expected = image[section[4]:section[4] + section[5]]
    actual = Path(readback_path).read_bytes()
    print(f"Flash base 0x{section[3]:08x}, expected {len(expected)} bytes, "
          f"readback {len(actual)} bytes")
    print(f"Different overlapping bytes: {sum(x != y for x, y in zip(expected, actual))}")
    assert expected == actual, "Flash readback differs from AXF"
    print("PASS: entire ER_IROM1 section matches")


if __name__ == "__main__":
    assert len(sys.argv) == 3, "Provide AXF path and flash dump path"
    verify(sys.argv[1], sys.argv[2])
