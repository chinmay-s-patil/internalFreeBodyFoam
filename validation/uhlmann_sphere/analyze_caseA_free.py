#!/usr/bin/env python3
"""
analyze_caseA_free.py <case_dir>

Evaluates free settling dynamics for Uhlmann & Dusek (2014) Case A (G = 144):
1. Initial acceleration check at release: a0 = -0.25 m/s^2 (-0.25 g)
2. Terminal settling velocity u_pV against spectral reference -0.908632 m/s (-1.285 u_ref)
3. Steady Reynolds number against reference Re = 185.08
"""

import argparse
import os
import sys
import numpy as np

D = 1.0
RHO_F = 1.0
RHO_P = 1.5
G = 1.0
NU = 0.0049104638
U_REF = np.sqrt(0.5 * G * D)  # 0.7071068 m/s

UPV_REF = -1.285 * U_REF      # -0.908632 m/s
RE_REF = 185.08
A0_ANALYTIC = -0.25 * G       # -0.25 m/s^2

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("case", help="Path to Case A case directory")
    args = parser.parse_args()

    motion_file = os.path.join(args.case, "postProcessing", "fallingMotion", "motion.dat")
    if not os.path.exists(motion_file):
        print(f"Error: {motion_file} not found.")
        sys.exit(1)

    data = np.loadtxt(motion_file)
    t = data[:, 0]
    Vy = data[:, 3]    # frame Vy
    Ay = data[:, 6]    # frame Ay
    Faero_y = data[:, 27]

    # 1. Initial acceleration check at first time step
    a0_sim = Ay[0]
    err_a0 = (a0_sim - A0_ANALYTIC) / abs(A0_ANALYTIC) * 100.0

    # 2. Terminal settling velocity check (last 30% of run)
    mask = t >= t[-1] * 0.70
    t_eval = t[mask]
    Vy_eval = Vy[mask]
    Ay_eval = Ay[mask]

    Vy_mean = np.mean(Vy_eval)
    Vy_min = np.min(Vy_eval)
    Vy_max = np.max(Vy_eval)
    Ay_mean = np.mean(Ay_eval)

    Re_sim = abs(Vy_mean) * D / NU
    err_upv = (Vy_mean - UPV_REF) / abs(UPV_REF) * 100.0
    err_re = (Re_sim - RE_REF) / RE_REF * 100.0

    dt_span = t_eval[-1] - t_eval[0]
    vel_drift = (Vy_eval[-1] - Vy_eval[0]) / dt_span if dt_span > 0 else 0.0

    print("=" * 80)
    print("UHLMANN & DUSEK (2014) CASE A (G = 144) FREE SETTLING BENCHMARK RESULTS")
    print(f"Case directory : {args.case}")
    print("=" * 80)
    print(f"1. ANALYTIC INITIAL ACCELERATION AT RELEASE:")
    print(f"   Theoretical a0 = g*(rho_r - 1)/(rho_r + 0.5) : {A0_ANALYTIC:.6f} m/s^2 (-0.2500 g)")
    print(f"   Simulated initial acceleration               : {a0_sim:.6f} m/s^2 ({a0_sim/G:+.4f} g)")
    print(f"   Error on initial release acceleration        : {err_a0:+.3f}%")
    if abs(err_a0) <= 2.0:
        print("   -> Analytic added-mass and buoyancy coupling: VERIFIED")
    else:
        print("   -> Analytic added-mass check: DEVIATION DETECTED")
    print("-" * 80)
    print(f"2. TERMINAL SETTLING VELOCITY & REYNOLDS NUMBER:")
    print(f"   Evaluation window                     : t = {t_eval[0]:.2f} s to {t_eval[-1]:.2f} s ({len(t_eval)} samples)")
    print(f"   Mean settling velocity u_pV           : {Vy_mean:.6f} m/s (min: {Vy_min:.6f}, max: {Vy_max:.6f})")
    print(f"   Exact Spectral Reference (Case AL)   : {UPV_REF:.6f} m/s (-1.285 u_ref)")
    print(f"   Velocity Error vs Spectral Reference  : {err_upv:+.2f}%")
    print(f"   Simulated Reynolds number Re          : {Re_sim:.2f}")
    print(f"   Exact Reference Reynolds number Re    : {RE_REF:.2f} (Error: {err_re:+.2f}%)")
    print(f"   Residual acceleration / velocity drift: {vel_drift:+.4e} m/s^2")
    print("-" * 80)
    if abs(err_upv) <= 2.0:
        print("CASE A BENCHMARK STATUS: PASSED (Settling velocity error <= 2.0%)")
    elif abs(err_upv) <= 5.0:
        print("CASE A BENCHMARK STATUS: ACCEPTABLE (Settling velocity error <= 5.0%, matches paper's IBM code)")
    else:
        print("CASE A BENCHMARK STATUS: FAILED (Exceeds acceptance criteria)")
    print("=" * 80)

if __name__ == "__main__":
    main()
