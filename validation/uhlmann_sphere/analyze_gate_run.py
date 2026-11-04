#!/usr/bin/env python3
"""
analyze_gate_run.py <case_dir> [--label LABEL]

Evaluates steady drag coefficient C_D of a fixed sphere at Re = 185.08 against:
1. Spectral Reference (Uhlmann & Dusek 2014, Jenny et al. 2004): C_D = 0.8075
2. Schiller & Naumann (1933): C_D = 0.832
3. Clift & Gauvin (1971): C_D = 0.836
"""

import argparse
import os
import sys
import numpy as np

D = 1.0
RHO = 1.0
NU = 0.0049104638
V = 0.908632
A = np.pi * (D / 2.0)**2
Q = 0.5 * RHO * (V**2) * A

CD_REF_SPECTRAL = 0.80748   # Exact Uhlmann & Dusek 2014 Table 2 Case AL
CD_SCHILLER_NAUMANN = 0.832
CD_CLIFT_GAUVIN = 0.836

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("case", help="Path to OpenFOAM case directory")
    parser.add_argument("--label", default="Gate Run", help="Case label")
    parser.add_argument("--window-fraction", type=float, default=0.4, help="Fraction of end time for averaging")
    args = parser.parse_args()

    motion_file = os.path.join(args.case, "postProcessing", "fallingMotion", "motion.dat")
    force_file = os.path.join(args.case, "postProcessing", "sphereForces", "0", "force.dat")

    if os.path.exists(motion_file):
        data = np.loadtxt(motion_file)
        t = data[:, 0]
        Fy = data[:, 27]       # aeroForce Y
        Fy_p = data[:, 21]     # pressureForce Y
        Fy_v = data[:, 24]     # viscousForce Y
    elif os.path.exists(force_file):
        rows = []
        for line in open(force_file):
            if line.startswith("#") or not line.strip():
                continue
            cleaned = line.replace("(", " ").replace(")", " ")
            rows.append([float(x) for x in cleaned.split()[:10]])
        data = np.array(rows)
        t = data[:, 0]
        Fy = data[:, 2]
        Fy_p = data[:, 5]
        Fy_v = data[:, 8]
    else:
        print(f"Error: Neither {motion_file} nor {force_file} found in {args.case}")
        sys.exit(1)

    t_end = t[-1]
    t_start = t_end * (1.0 - args.window_fraction)
    mask = t >= t_start

    t_eval = t[mask]
    Fy_eval = Fy[mask]
    Fyp_eval = Fy_p[mask]
    Fyv_eval = Fy_v[mask]

    Cd_eval = Fy_eval / Q
    Cd_p_eval = Fyp_eval / Q
    Cd_v_eval = Fyv_eval / Q

    Cd_mean = np.mean(Cd_eval)
    Cd_p_mean = np.mean(Cd_p_eval)
    Cd_v_mean = np.mean(Cd_v_eval)

    dt_span = t_eval[-1] - t_eval[0]
    drift = (Cd_eval[-1] - Cd_eval[0]) / dt_span if dt_span > 0 else 0.0
    rel_drift_pct = (drift / Cd_mean * 100.0) if Cd_mean > 0 else 0.0

    err_spectral = (Cd_mean - CD_REF_SPECTRAL) / CD_REF_SPECTRAL * 100.0
    err_sn = (Cd_mean - CD_SCHILLER_NAUMANN) / CD_SCHILLER_NAUMANN * 100.0
    err_cg = (Cd_mean - CD_CLIFT_GAUVIN) / CD_CLIFT_GAUVIN * 100.0

    print("=" * 80)
    print(f"UHLMANN BENCHMARK GATE CHECK: {args.label}")
    print(f"Case directory : {args.case}")
    print(f"Flow conditions: Re = 185.08, V = {V:.6f} m/s, D = {D:.2f} m, nu = {NU:.8f}")
    print(f"Dynamic force scale q = 0.5 * rho * V^2 * A = {Q:.6f} N")
    print("=" * 80)
    print(f"Evaluation window   : t = {t_eval[0]:.2f} s to {t_eval[-1]:.2f} s ({len(t_eval)} samples)")
    print(f"Computed C_D (total): {Cd_mean:.4f}  [min: {np.min(Cd_eval):.4f}, max: {np.max(Cd_eval):.4f}]")
    print(f"  - Pressure drag   : {Cd_p_mean:.4f} ({Cd_p_mean/Cd_mean*100:.1f}%)")
    print(f"  - Friction drag   : {Cd_v_mean:.4f} ({Cd_v_mean/Cd_mean*100:.1f}%)")
    print(f"Linear drift        : {drift:+.4e} /s (relative drift: {rel_drift_pct:+.2f}% /s)")
    print("-" * 80)
    print(f"Comparison to Benchmark Reference & Correlations:")
    print(f"  * Spectral Reference (Uhlmann & Dusek 2014): C_D = {CD_REF_SPECTRAL:.4f} | Error: {err_spectral:+.2f}%")
    print(f"    Schiller-Naumann (1933)                  : C_D = {CD_SCHILLER_NAUMANN:.4f} | Error: {err_sn:+.2f}%")
    print(f"    Clift-Gauvin (1971)                      : C_D = {CD_CLIFT_GAUVIN:.4f} | Error: {err_cg:+.2f}%")
    print("-" * 80)
    if abs(err_spectral) <= 2.0:
        print("GATE STATUS: PASSED (Error <= 2.0% of spectral benchmark reference)")
    elif abs(err_spectral) <= 5.0:
        print("GATE STATUS: ACCEPTABLE (Error <= 5.0%, matches IBM code yardstick)")
    else:
        print("GATE STATUS: FAILED (Exceeds acceptance criteria)")
    print("=" * 80)

if __name__ == "__main__":
    main()
