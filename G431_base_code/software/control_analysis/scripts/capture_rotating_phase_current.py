"""Capture CH4 phase current during rotating open-current operation.

CH4 must be a current probe whose waveform unit is ampere.  The script does
not control the MCU or motor output; it only acquires one scope record,
extracts the fundamental at the requested electrical frequency, saves the
waveform/evidence, and restores the scope settings it changed.
"""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import pyvisa


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--frequency-hz", type=float, required=True)
    parser.add_argument("--command-peak-a", type=float, required=True)
    parser.add_argument("--horizontal-scale-s", type=float, default=0.2)
    parser.add_argument(
        "--resource", default="USB::0x0699::0x052C::C064843::INSTR"
    )
    args = parser.parse_args()
    if args.frequency_hz < 0.0 or args.command_peak_a <= 0.0:
        parser.error("frequency must be non-negative and command peak positive")

    args.output.mkdir(parents=True, exist_ok=True)
    scope = pyvisa.ResourceManager().open_resource(args.resource)
    scope.timeout = 30000
    scope.chunk_size = 1024 * 1024
    state = {
        "acquire": int(scope.query("ACQUIRE:STATE?")),
        "stop_after": scope.query("ACQUIRE:STOPAFTER?").strip(),
        "horizontal_scale": float(scope.query("HORIZONTAL:SCALE?")),
        "trigger_mode": scope.query("TRIGGER:A:MODE?").strip(),
    }
    try:
        scope.write("ACQUIRE:STATE STOP")
        scope.write(f"HORIZONTAL:SCALE {args.horizontal_scale_s}")
        scope.write("TRIGGER:A:MODE AUTO")
        scope.write("ACQUIRE:STOPAFTER SEQUENCE")
        scope.write("ACQUIRE:STATE RUN")
        deadline = time.monotonic() + 15.0
        while int(scope.query("ACQUIRE:STATE?")):
            if time.monotonic() > deadline:
                raise TimeoutError("scope did not complete the sequence")
            time.sleep(0.05)

        count = int(scope.query("HORIZONTAL:RECORDLENGTH?"))
        scope.write("DATA:SOURCE CH4")
        scope.write("DATA:ENCDG RIBINARY")
        scope.write("DATA:WIDTH 1")
        scope.write("DATA:START 1")
        scope.write(f"DATA:STOP {count}")
        dt = float(scope.query("WFMOUTPRE:XINCR?"))
        ymult = float(scope.query("WFMOUTPRE:YMULT?"))
        yoff = float(scope.query("WFMOUTPRE:YOFF?"))
        yzero = float(scope.query("WFMOUTPRE:YZERO?"))
        unit = scope.query("WFMOUTPRE:YUNIT?").strip().strip('"')
        raw = scope.query_binary_values("CURVE?", datatype="b", container=np.array)
        current = (raw.astype(float) - yoff) * ymult + yzero
        t = np.arange(current.size, dtype=float) * dt

        if args.frequency_hz > 0.0:
            omega_t = 2.0 * math.pi * args.frequency_hz * t
            design = np.column_stack((np.ones_like(t), np.sin(omega_t), np.cos(omega_t)))
            offset, sin_coeff, cos_coeff = np.linalg.lstsq(design, current, rcond=None)[0]
            fitted = design @ np.array((offset, sin_coeff, cos_coeff))
            fundamental_peak = float(math.hypot(sin_coeff, cos_coeff))
            fit_phase = float(math.atan2(cos_coeff, sin_coeff))
        else:
            offset = float(current.mean())
            fitted = np.full_like(current, offset)
            fundamental_peak = 0.0
            fit_phase = None
        residual_rms = float(np.sqrt(np.mean((current - fitted) ** 2)))
        command = args.command_peak_a
        result = {
            "scope": scope.query("*IDN?").strip(),
            "probe_id": scope.query("CH4:PROBE:ID?").strip(),
            "unit": unit,
            "samples": int(current.size),
            "sample_interval_s": dt,
            "record_duration_s": float(current.size * dt),
            "electrical_frequency_hz": args.frequency_hz,
            "command_peak_a": command,
            "ch4_mean_a": float(current.mean()),
            "ch4_rms_a": float(np.sqrt(np.mean(current ** 2))),
            "fundamental_peak_a": fundamental_peak,
            "fundamental_rms_a": fundamental_peak / math.sqrt(2.0),
            "fundamental_to_command": (
                fundamental_peak / command if args.frequency_hz > 0.0 else None
            ),
            "fit_phase_rad": fit_phase,
            "residual_rms_a": residual_rms,
            "minimum_a": float(current.min()),
            "maximum_a": float(current.max()),
        }
        np.savez_compressed(
            args.output / "ch4_current.npz",
            current_a=current,
            sample_interval_s=np.array(dt),
            fitted_a=fitted,
        )
        (args.output / "analysis.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        scope.write("ACQUIRE:STATE STOP")
        scope.write(f"HORIZONTAL:SCALE {state['horizontal_scale']}")
        scope.write(f"TRIGGER:A:MODE {state['trigger_mode']}")
        scope.write(f"ACQUIRE:STOPAFTER {state['stop_after']}")
        scope.write(f"ACQUIRE:STATE {state['acquire']}")
        scope.close()


if __name__ == "__main__":
    main()
