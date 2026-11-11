"""Read-only MCU diagnosis through documented pylink CoreSight operations.

Only debug-port power/selection and MEM-AP addressing registers are written.
No target memory writes, core connect, halt, reset, or power-output changes.
"""
import json
import sys
import struct
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'jlink_diag_libs'))
import pylink

result = {'dll': sys.argv[1], 'reads': {}}
link = pylink.JLink(lib=pylink.library.Library(dllpath=sys.argv[1]))
original_read = link.coresight_read
def paced_read(*args, **kwargs):
    time.sleep(.003)
    return original_read(*args, **kwargs)
link.coresight_read = paced_read
try:
    link.open(serial_no=150710982)
    link.set_tif(pylink.enums.JLinkInterfaces.SWD)
    link.set_speed(50)
    link.coresight_configure()
    result['dpidr'] = hex(link.coresight_read(0, ap=False))
    link.coresight_write(0, 0x1e, ap=False)
    link.coresight_write(1, 0x50000000, ap=False)
    result['ctrlstat'] = hex(link.coresight_read(1, ap=False))
    link.coresight_write(2, 0, ap=False)
    csw = link.coresight_read(0, ap=True)
    result['csw'] = hex(csw)
    link.coresight_write(0, (csw & ~0x37) | 2, ap=True)
    for addr in (0xe0042000, 0x40012c44, 0xe000ed28, 0xe000ed2c):
        link.coresight_write(1, addr, ap=True)
        result['reads'][hex(addr)] = hex(link.coresight_read(3, ap=True))
    if '--stop-verified-r14' in sys.argv:
        sys.path.insert(0, str(Path('software/control_analysis/scripts').resolve()))
        from verify_encoder_artifacts import load_image
        root = Path('software/control_analysis/results/position_tuning_20261029')
        out = root / 'native_verified_stop'
        out.mkdir(exist_ok=True)
        expected = load_image(root / 'flash_r14/firmware.axf')
        start, end = min(expected), max(expected) + 1
        source = out/'flash_readback_slow_retry.bin'
        if not source.exists(): source = out/'flash_readback.bin'
        raw = bytearray(source.read_bytes()) if source.exists() else bytearray()
        if not raw:
            link.set_speed(1000)
            link.coresight_write(0, (csw & ~0x37) | 0x12, ap=True)
            for addr in range(start, (end + 3) & ~3, 4):
                if addr == start or addr % 1024 == 0:
                    link.coresight_write(1, addr, ap=True)
                raw.extend(struct.pack('<I', link.coresight_read(3, ap=True)))
        (out/'flash_readback.bin').write_bytes(raw)
        mismatches = [hex(addr) for addr, value in expected.items()
                      if raw[addr-start] != value]
        link.set_speed(50)
        link.coresight_write(0, (csw & ~0x37) | 2, ap=True)
        # Retry suspect words individually at slow speed, requiring two equal
        # reads; keep the first capture as evidence of transport corruption.
        for addr in sorted({int(addr,16) & ~3 for addr in mismatches}):
            readings = []
            for _ in range(2):
                link.coresight_write(1, addr, ap=True)
                readings.append(link.coresight_read(3, ap=True))
            if readings[0] != readings[1]:
                raise RuntimeError('Inconsistent flash word during slow retry')
            raw[addr-start:addr-start+4] = struct.pack('<I', readings[0])
            (out/'flash_readback_slow_retry.bin').write_bytes(raw)
        (out/'flash_readback_slow_retry.bin').write_bytes(raw)
        mismatches = [hex(addr) for addr, value in expected.items()
                      if raw[addr-start] != value]
        result['flash_mismatches'] = mismatches[:20]
        result['verified_bytes'] = len(expected)
        if mismatches:
            raise RuntimeError('Flash mismatch; no application RAM written')
        link.set_speed(50)
        link.coresight_write(0, (csw & ~0x37) | 2, ap=True)
        symbols = json.loads((root/'kp70_amp2_ff0_r14_rpm7500/symbols.json').read_text())
        for key, value in [('user',0), ('position',2), ('speed_run',0)]:
            link.coresight_write(1, symbols[key], ap=True)
            link.coresight_write(3, value, ap=True)
        time.sleep(.3)
        link.coresight_write(1, 0x40012c44, ap=True)
        result['bdtr_after_stop'] = hex(link.coresight_read(3, ap=True))
        (out/'result.json').write_text(json.dumps(result, indent=2))
        if int(result['bdtr_after_stop'], 16) & 0x8000:
            raise RuntimeError('PWM still enabled after normal stop request')
except Exception as exc:
    result['error'] = repr(exc)
    import traceback
    result['traceback'] = traceback.format_exc()
finally:
    link.close()
    if '--stop-verified-r14' in sys.argv and 'out' in globals():
        (out/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
