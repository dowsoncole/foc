"""Sample rotating-frame current and speed from a running target via SWD.

The target is not halted.  Addresses are defaults for the current servo_v1
image and must be re-resolved from the AXF if the data layout changes.
"""

import argparse
import json
import re
import struct
import subprocess

import numpy as np


def word_to_float(word):
    return struct.unpack("<f", struct.pack("<I", int(word, 16)))[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--interval-ms", type=int, default=10)
    parser.add_argument("--status-address", default="0x20001484")
    parser.add_argument(
        "--pyocd",
        default=(
            r"C:\Users\penghao\.vscode\extensions\arm.vscode-cmsis-debugger-"
            r"1.10.0-win32-x64\tools\pyocd\pyocd.exe"
        ),
    )
    parser.add_argument(
        "--pack",
        default=r"C:\Users\penghao\AppData\Local\Arm\Packs\Keil\STM32G4xx_DFP\2.2.0",
    )
    args = parser.parse_args()
    if args.samples <= 0 or args.interval_ms < 0:
        parser.error("invalid sample count or interval")

    commands = []
    for _ in range(args.samples):
        commands.append(f"read32 {args.status_address} 16")
        if args.interval_ms:
            commands.append(f"sleep {args.interval_ms}")
    commands.append("exit")
    completed = subprocess.run(
        [
            args.pyocd,
            "commander",
            "-t",
            "stm32g431cbtx",
            "--pack",
            args.pack,
            "-u",
            "E1007200D0D2139393740544",
            "-M",
            "attach",
            "-W",
        ],
        input="\n".join(commands) + "\n",
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    )
    address = args.status_address.lower().replace("0x", "")
    pattern = re.compile(
        rf"{address}:\s+([0-9a-fA-F]{{8}})\s+([0-9a-fA-F]{{8}})\s+"
        rf"([0-9a-fA-F]{{8}})\s+([0-9a-fA-F]{{8}})"
    )
    rows = np.array(
        [[word_to_float(word) for word in match] for match in pattern.findall(completed.stdout)],
        dtype=float,
    )
    if rows.shape != (args.samples, 4):
        raise RuntimeError(f"expected {args.samples} samples, parsed {len(rows)}")
    names = ("actual_rpm", "actual_position_deg", "id_a", "iq_a")
    result = {"samples": args.samples, "interval_ms": args.interval_ms}
    for column, name in enumerate(names):
        values = rows[:, column]
        result[name] = {
            "mean": float(values.mean()),
            "std": float(values.std()),
            "minimum": float(values.min()),
            "maximum": float(values.max()),
        }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
