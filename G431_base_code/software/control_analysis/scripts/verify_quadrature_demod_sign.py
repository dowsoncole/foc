"""Verify the sign convention of the single-frequency quadrature demodulator
implemented in output_position.c (OutputPosition_ReadSweepFrd).

Author : 千问工作助理
Date   : 2026-10-28
Reason : A causal closed-loop position response must have NEGATIVE phase
         (lag). The first implementation produced +90 deg for a pure
         y = sin(wt) response to x = cos(wt), i.e. a LEAD, which is
         physically impossible. This script settles the correct formula
         numerically before the firmware is corrected.

Pure standard library, no hardware access.
"""
import cmath
import math

N = 4000
F = 20.0
W = 2.0 * math.pi * F
DT = 1.0 / 1000.0            # 1 kHz position loop (after this round's change)


def demod(x_fn, y_fn, n=N, dt=DT, w=W):
    """Accumulate exactly as the firmware does, then form H both ways."""
    xc = xs = yc = ys = 0.0
    for i in range(n):
        t = i * dt
        c, s = math.cos(w * t), math.sin(w * t)
        x, y = x_fn(t), y_fn(t)
        xc += x * c
        xs += x * s
        yc += y * c
        ys += y * s
    sxx = xc * xc + xs * xs
    syy = yc * yc + ys * ys
    # Variant A: the ORIGINAL firmware implementation.
    #   re = (yc*xc + ys*xs)/sxx ; im = (ys*xc - yc*xs)/sxx
    re_a = (yc * xc + ys * xs) / sxx
    im_a = (ys * xc - yc * xs) / sxx
    # Variant B: the CORRECTED firmware implementation -- only the sign of the
    #   imaginary part is flipped. Derived below from H = Y/X with conj-DFT bins:
    #     Y = sum(y*conj(e)) = yc - j*ys ;  X = sum(x*conj(e)) = xc - j*xs
    #     H = (yc - j*ys)(xc + j*xs)/sxx
    #       = [yc*xc + ys*xs + j*(yc*xs - ys*xc)] / sxx
    #   so im must be (yc*xs - ys*xc)/sxx, i.e. the NEGATIVE of variant A.
    re_b = (yc * xc + ys * xs) / sxx
    im_b = (yc * xs - ys * xc) / sxx
    # Variant C: explicit complex ratio, independent cross-check of B.
    Y = complex(yc, -ys)
    X = complex(xc, -xs)
    Hc = Y / X
    coh = (abs(Hc) ** 2 * sxx) / syy if syy > 0 else 0.0
    return dict(A=complex(re_a, im_a), B=complex(re_b, im_b), C=Hc,
                n=n, sxx=sxx, coherence=coh)


def report(label, x_fn, y_fn, expect_deg, expect_mag=1.0):
    r = demod(x_fn, y_fn)
    print(f"\n--- {label} ---")
    print(f"  expected: |H|={expect_mag:.4f}, phase={expect_deg:+.2f} deg")
    for k in ("A", "B", "C"):
        h = r[k]
        mag = abs(h)
        ph = math.degrees(cmath.phase(h))
        ok = abs(ph - expect_deg) < 1.0 and abs(mag - expect_mag) < 0.02
        print(f"  variant {k}: |H|={mag:.4f}  phase={ph:+8.2f} deg   "
              f"[{'OK' if ok else 'WRONG'}]")
    print(f"  coherence gamma^2 = {r['coherence']:.6f} (expect ~1.0)")
    return r


print("=" * 74)
print("Quadrature demodulator sign verification (single frequency, 20 Hz)")
print("=" * 74)

# Case 1: pure 90 deg LAG. x = cos(wt), y = sin(wt) = cos(wt - 90deg)
#   True H = y/x = -j  ->  phase = -90 deg
report("case 1: pure -90 deg lag (y = sin wt, x = cos wt)",
       lambda t: math.cos(W * t), lambda t: math.sin(W * t), -90.0)

# Case 2: unity gain, zero phase. x = y = cos(wt) -> H = 1
report("case 2: unity, zero phase",
       lambda t: math.cos(W * t), lambda t: math.cos(W * t), 0.0)

# Case 3: gain 0.5, phase -45 deg (typical closed-loop point below crossover)
report("case 3: |H|=0.5, phase=-45 deg",
       lambda t: math.cos(W * t),
       lambda t: 0.5 * math.cos(W * t - math.radians(45.0)), -45.0, 0.5)

# Case 4: the DESIGN POINT from position_loop_bandwidth_design.py
#   20 Hz closed-loop gain -0.35 dB (=0.9605), phase -118.7 deg
mag = 10.0 ** (-0.35 / 20.0)
report(f"case 4: design point |H|={mag:.4f} (-0.35 dB), phase=-118.7 deg",
       lambda t: math.cos(W * t),
       lambda t: mag * math.cos(W * t - math.radians(118.7)), -118.7, mag)

# Case 5: non-integer number of cycles in the window (leakage check).
#   1 kHz sampling of 20 Hz over 0.1875 s = 3.75 cycles, still exact because
#   20 Hz divides 1 kHz evenly; use an off-grid 17.3 Hz to show leakage.
F2 = 17.3
W2 = 2.0 * math.pi * F2
r = demod(lambda t: math.cos(W2 * t),
          lambda t: 0.7 * math.cos(W2 * t - math.radians(60.0)),
          n=4000, dt=DT, w=W2)
h = r["C"]
print("\n--- case 5: off-grid 17.3 Hz, |H|=0.7, phase=-60 deg (leakage check) ---")
print(f"  variant C: |H|={abs(h):.4f}  phase={math.degrees(cmath.phase(h)):+8.2f} deg")
print(f"  coherence gamma^2 = {r['coherence']:.6f}")
print("  NOTE: window must contain an integer number of cycles, otherwise")
print("        spectral leakage biases both gain and phase. The firmware must")
print("        therefore accumulate over an integer cycle count (see fix note).")

print("\n" + "=" * 74)
print("CONCLUSION")
print("=" * 74)
r1 = demod(lambda t: math.cos(W * t), lambda t: math.sin(W * t))
print(f"  Variant A (original firmware) on case 1 gives "
      f"phase {math.degrees(cmath.phase(r1['A'])):+.1f} deg -- WRONG SIGN "
      f"(a causal loop cannot lead).")
print(f"  Variant C (H = sum(y*conj(e)) / sum(x*conj(e))) gives "
      f"phase {math.degrees(cmath.phase(r1['C'])):+.1f} deg -- CORRECT.")
print("  => Firmware must use Y = (yc, -ys) and X = (xc, -xs), H = Y/X.")
print("     Equivalently: im = (ys*xc - yc*xs)/sxx is correct ONLY if the")
print("     accumulator pairs are interpreted as conj-DFT bins. The safe")
print("     formulation is the explicit complex ratio above.")
