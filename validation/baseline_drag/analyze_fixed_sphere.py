#!/usr/bin/env python3
"""
analyze_fixed_sphere.py <case> --Re <Re> [--t-start T]

Evaluates steady drag coefficient C_D of a fixed sphere against:
1. Schiller & Naumann (1933): C_D = (24/Re) * (1 + 0.15 * Re^0.687)
2. Clift, Grace & Weber (1978): standard correlation for 20 <= Re <= 260
"""
import argparse, glob, os, re, sys
import numpy as np

D = 0.05
RHO = 1.204
NU = 1.5e-5
A = np.pi * (D / 2.0)**2

def schiller_naumann(Re):
    return (24.0 / Re) * (1.0 + 0.15 * (Re**0.687))

def clift_grace_weber(Re):
    w = np.log10(Re)
    log_cd = 1.6435 - 1.1242 * w + 0.1558 * (w**2)
    return 10.0**log_cd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("case", help="Path to sphere case")
    ap.add_argument("--Re", type=float, required=True, help="Reynolds number")
    ap.add_argument("--t-start", type=float, default=None, help="Start time for averaging (s)")
    args = ap.parse_args()

    motion_file = os.path.join(args.case, "postProcessing", "fallingMotion", "motion.dat")
    if not os.path.exists(motion_file):
        print(f"Error: {motion_file} not found.")
        sys.exit(1)

    data = np.loadtxt(motion_file)
    t = data[:, 0]
    Vy = np.abs(data[:, 3])
    Fy = data[:, 27]

    V = args.Re * NU / D
    q = 0.5 * RHO * (V**2) * A

    # Evaluation window: last 40% of run or after t-start
    if args.t_start is not None:
        mask = t >= args.t_start
    else:
        mask = t >= t[-1] * 0.6

    t_eval = t[mask]
    Fy_eval = Fy[mask]
    Cd_eval = Fy_eval / q

    Cd_mean = np.mean(Cd_eval)
    Cd_min = np.min(Cd_eval)
    Cd_max = np.max(Cd_eval)
    drift = (Cd_eval[-1] - Cd_eval[0]) / (t_eval[-1] - t_eval[0]) if (t_eval[-1] - t_eval[0]) > 0 else 0

    Cd_sn = schiller_naumann(args.Re)
    Cd_cgw = clift_grace_weber(args.Re)
    err_sn = (Cd_mean - Cd_sn) / Cd_sn * 100.0
    err_cgw = (Cd_mean - Cd_cgw) / Cd_cgw * 100.0

    print("=" * 80)
    print(f"SPHERE BENCHMARK RESULTS: Re = {args.Re:.1f} (V = {V:.4f} m/s, D = {D:.3f} m)")
    print("=" * 80)
    print(f"Case directory      : {args.case}")
    print(f"Evaluation window   : t = {t_eval[0]:.3f} s to {t_eval[-1]:.3f} s ({len(t_eval)} samples)")
    print(f"Computed C_D (mean) : {Cd_mean:.4f}  [min: {Cd_min:.4f}, max: {Cd_max:.4f}]")
    print(f"Linear drift        : {drift:+.4e} /s (relative drift: {drift/Cd_mean*100:+.2f}% /s)")
    print("-" * 80)
    print(f"Reference Correlations:")
    print(f"  Schiller-Naumann (1933) : C_D = {Cd_sn:.4f}  |  Error: {err_sn:+.2f}%")
    print(f"  Clift et al. (1978)     : C_D = {Cd_cgw:.4f}  |  Error: {err_cgw:+.2f}%")
    print("=" * 80)

if __name__ == "__main__":
    main()
