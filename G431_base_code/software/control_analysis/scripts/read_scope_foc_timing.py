"""Capture one MDO34 frame, measure CH1 high pulses, restore acquisition.

CH1 = PA0 ADC/FOC, CH2 = PD2 TIM1 update, CH3 = PB7 DMA (active low).
Requires pyvisa and numpy. No MCU or output-enable writes.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import pyvisa

p = argparse.ArgumentParser()
p.add_argument("output", type=Path)
p.add_argument("--resource", default="USB::0x0699::0x052C::C064843::INSTR")
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
s = pyvisa.ResourceManager().open_resource(a.resource)
s.timeout = 20000
state = int(s.query("ACQUIRE:STATE?"))
mode = s.query("ACQUIRE:STOPAFTER?").strip()
scale = float(s.query("HORIZONTAL:SCALE?"))
try:
    s.write("ACQUIRE:STATE STOP")
    s.write("HORIZONTAL:SCALE 200E-6")
    s.write("ACQUIRE:STOPAFTER SEQUENCE")
    s.write("ACQUIRE:STATE RUN")
    deadline = time.monotonic() + 10
    while int(s.query("ACQUIRE:STATE?")):
        if time.monotonic() > deadline:
            raise TimeoutError("No completed triggered frame")
        time.sleep(.05)
    count = int(s.query("HORIZONTAL:RECORDLENGTH?"))
    s.write("DATA:ENCDG RIBINARY")
    s.write("DATA:WIDTH 1")
    s.write("DATA:START 1")
    s.write(f"DATA:STOP {count}")
    data = {}
    for ch in (1, 2, 3):
        s.write(f"DATA:SOURCE CH{ch}")
        dx = float(s.query("WFMOUTPRE:XINCR?"))
        ym = float(s.query("WFMOUTPRE:YMULT?"))
        yo = float(s.query("WFMOUTPRE:YOFF?"))
        yz = float(s.query("WFMOUTPRE:YZERO?"))
        raw = s.query_binary_values("CURVE?", datatype="b", container=np.array)
        data[f"ch{ch}"] = (raw.astype(float) - yo) * ym + yz
    data["x_increment_s"] = np.array(dx)
    np.savez_compressed(a.output / "scope_frame.npz", **data)
    high = data["ch1"] > 1.5
    rises = np.flatnonzero(~high[:-1] & high[1:]) + 1
    falls = np.flatnonzero(high[:-1] & ~high[1:]) + 1
    widths = []
    for rise in rises:
        following = falls[falls > rise]
        if len(following):
            widths.append(float((following[0] - rise) * dx * 1e6))
    result = {"idn": s.query("*IDN?").strip(), "samples": count,
              "x_increment_s": dx, "pa0_high_us": widths,
              "period_us": (np.diff(rises) * dx * 1e6).tolist()}
    (a.output / "scope_timing.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
finally:
    s.write("ACQUIRE:STATE STOP")
    s.write(f"HORIZONTAL:SCALE {scale}")
    s.write(f"ACQUIRE:STOPAFTER {mode}")
    s.write(f"ACQUIRE:STATE {state}")
    s.close()
