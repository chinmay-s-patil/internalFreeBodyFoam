#!/usr/bin/env python3
"""
plot_validation.py — Post-processing & Experimental Validation for Thin Disk Falling in Air
Reference: Tinklenberg, Guala, Coletti (2023), J. Fluid Mech. (Vol 972, A17)

Compares OpenFOAM freeBody6DoFPimpleFoam simulation results against experimental benchmarks:
  - Terminal descent velocity: V_t = 1.23 ± 0.31 m/s
  - Mean drag coefficient:     C_D = 0.74
  - Tumbling frequency:        f_t ≈ 36.4 Hz (|omega_t| ≈ 228.7 rad/s)
  - Drift inclination angle:   |psi| ≈ 10.5 deg

Skips initial numerical startup transients to ensure clean, publication-grade figures.
"""

import os
import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FALLING_COLS_38 = [
    "Time", "Altitude", "Vx", "Vy", "Vz", "Ax", "Ay", "Az",
    "gx", "gy", "gz", "rho", "a_sound", "Mach",
    "Umean_x", "Umean_y", "Umean_z", "Urel_x", "Urel_y", "Urel_z",
    "Fpress_x", "Fpress_y", "Fpress_z", "Fvisc_x", "Fvisc_y", "Fvisc_z",
    "Faero_x", "Faero_y", "Faero_z", "Fgrav_x", "Fgrav_y", "Fgrav_z",
    "Fnet_x", "Fnet_y", "Fnet_z", "Fext_x", "Fext_y", "Fext_z"
]

def load_data(case_dir, skip_time=0.002, skip_rows=50):
    post_dir = os.path.join(case_dir, "postProcessing")
    tumble_file = os.path.join(post_dir, "tumbleMotion", "motion.dat")
    falling_file = os.path.join(post_dir, "fallingMotion", "motion.dat")
    
    if not os.path.exists(tumble_file):
        print(f"[Error] Tumble log not found at: {tumble_file}")
        sys.exit(1)
        
    # Read tumble data
    with open(tumble_file, "r") as f:
        first_line = f.readline().strip()
    headers = [h for h in first_line.lstrip("#").strip().split() if h]
    df_tumble = pd.read_csv(tumble_file, sep=r"\s+", comment="#", names=headers)
    
    col_map = {}
    for c in df_tumble.columns:
        clean = c.split("(")[0].replace("#", "").strip()
        col_map[c] = clean
    df_tumble.rename(columns=col_map, inplace=True)
    
    # Filter initial startup points (both by min row count and physical time)
    if len(df_tumble) > skip_rows:
        df_tumble = df_tumble.iloc[skip_rows:].copy()
    if "Time" in df_tumble.columns:
        df_tumble = df_tumble[df_tumble["Time"] >= skip_time].copy()
        
    df_falling = None
    if os.path.exists(falling_file) and os.path.getsize(falling_file) > 0:
        try:
            # Check if file has header
            with open(falling_file, "r") as f:
                first_line_f = f.readline().strip()
            if first_line_f.startswith("#"):
                headers_f = [h for h in first_line_f.lstrip("#").strip().split() if h]
                df_falling = pd.read_csv(falling_file, sep=r"\s+", comment="#", names=headers_f)
            else:
                df_falling = pd.read_csv(falling_file, sep=r"\s+", names=FALLING_COLS_38[:len(first_line_f.split())])
                
            if len(df_falling) > skip_rows:
                df_falling = df_falling.iloc[skip_rows:].copy()
            if "Time" in df_falling.columns:
                df_falling = df_falling[df_falling["Time"] >= skip_time].copy()
        except Exception as e:
            print(f"[Warning] Error parsing falling file: {e}")
            
    return df_tumble, df_falling

def main():
    parser = argparse.ArgumentParser(description="Plot Thin Disk Validation vs Experiment")
    parser.add_argument("--skip-time", type=float, default=0.002, help="Ignore data before this time (s) to remove startup impulses")
    parser.add_argument("--skip-rows", type=int, default=30, help="Minimum number of initial rows to discard")
    parser.add_argument("--output", type=str, default="validation_comparison.png", help="Output PNG path")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    case_dir = os.path.join(script_dir, "case")
    
    df_tumble, df_falling = load_data(case_dir, skip_time=args.skip_time, skip_rows=args.skip_rows)
    
    # Experimental benchmarks (2 mm disk, Tinklenberg et al. 2023)
    exp_D = 2.02e-3       # m
    exp_h = 50e-6         # m
    exp_mass = 2.2113e-7  # kg
    exp_rho_f = 1.200     # kg/m^3
    exp_area = np.pi * (exp_D / 2.0)**2  # m^2
    exp_g = 9.81          # m/s^2
    
    exp_Vt_mean = 1.23    # m/s
    exp_Vt_std = 0.31     # m/s
    exp_CD = 0.74
    exp_ft = 36.4         # Hz
    exp_omega_t = 228.7   # rad/s
    exp_psi = 10.5        # deg
    
    # Analyze simulated metrics
    t_sim = df_tumble["Time"].values
    omega_x = df_tumble["Omega_x"].values if "Omega_x" in df_tumble.columns else np.zeros_like(t_sim)
    angle_x = df_tumble["Angle_x"].values if "Angle_x" in df_tumble.columns else np.zeros_like(t_sim)
    
    # Falling velocity
    if df_falling is not None and "Vy" in df_falling.columns:
        t_fall = df_falling["Time"].values
        vy = -df_falling["Vy"].values  # downward speed positive
        sim_Vt = np.mean(vy[-max(1, len(vy)//4):])
    else:
        t_fall = t_sim
        vy = np.zeros_like(t_sim)
        sim_Vt = 0.0

    # Tumbling frequency estimation from angle zero crossings or mean omega
    if len(t_sim) > 20 and (t_sim[-1] - t_sim[0]) > 0.01:
        w_sub = omega_x - np.mean(omega_x)
        zero_crossings = np.where(np.diff(np.sign(w_sub)))[0]
        if len(zero_crossings) >= 4:
            t_cross = t_sim[zero_crossings]
            half_periods = np.diff(t_cross)
            period = 2.0 * np.mean(half_periods)
            sim_ft = 1.0 / period if period > 0 else 0.0
        else:
            sim_ft = np.abs(np.mean(omega_x[-max(1, len(omega_x)//4):])) / (2.0 * np.pi)
    else:
        sim_ft = np.abs(omega_x[-1]) / (2.0 * np.pi) if len(omega_x) > 0 else 0.0

    sim_omega = np.abs(omega_x[-1]) if len(omega_x) > 0 else 0.0
    sim_CD = (2.0 * exp_mass * exp_g) / (exp_rho_f * exp_area * (sim_Vt**2)) if sim_Vt > 1e-3 else 0.0

    print("=" * 65)
    print("      EXPERIMENTAL VALIDATION SUMMARY (Tinklenberg et al. 2023)    ")
    print(f"      (Ignored initial startup transient t < {args.skip_time*1e3:.1f} ms)      ")
    print("=" * 65)
    print(f"{'Parameter':<28} | {'Exp (Paper)':<16} | {'Simulation':<16}")
    print("-" * 65)
    print(f"{'Disk Diameter D (mm)':<28} | {exp_D*1e3:<16.2f} | {exp_D*1e3:<16.2f}")
    print(f"{'Disk Thickness h (um)':<28} | {exp_h*1e6:<16.1f} | {exp_h*1e6:<16.1f}")
    print(f"{'Disk Mass (mg)':<28} | {exp_mass*1e6:<16.3f} | {exp_mass*1e6:<16.3f}")
    print(f"{'Descent Velocity Vy (m/s)':<28} | {exp_Vt_mean:<7.2f} ± {exp_Vt_std:<6.2f} | {sim_Vt:<16.3f}")
    print(f"{'Tumbling Freq f_t (Hz)':<28} | {exp_ft:<16.1f} | {sim_ft:<16.1f}")
    print(f"{'Tumbling Omega |w_t| (rad/s)':<28} | {exp_omega_t:<16.1f} | {sim_omega:<16.1f}")
    print(f"{'Drag Coefficient C_D':<28} | {exp_CD:<16.2f} | {sim_CD:<16.2f}")
    print("=" * 65)
    
    # Generate 4-panel publication plot
    fig, axes = plt.subplots(2, 2, figsize=(13, 9.5))
    fig.patch.set_facecolor("#FAFAFA")
    
    # 1. Fall Velocity vs Time
    ax = axes[0, 0]
    if df_falling is not None and len(t_fall) > 1:
        ax.plot(t_fall * 1e3, vy, color="#1f77b4", lw=2.2, label=r"freeBody6DoF CFD $V_y(t)$")
        ax.axhline(exp_Vt_mean, color="#d62728", linestyle="--", lw=1.8, label=f"Exp Mean: {exp_Vt_mean} m/s")
        ax.axhspan(exp_Vt_mean - exp_Vt_std, exp_Vt_mean + exp_Vt_std, color="#d62728", alpha=0.15, label="Exp ±1σ Range")
        ax.legend(loc="lower right", frameon=True)
    ax.set_title("Descent Fall Speed $V_y(t)$", fontsize=13, fontweight="bold")
    ax.set_xlabel("Time (ms)", fontsize=11)
    ax.set_ylabel("Fall Speed $V_y$ (m/s)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    
    # 2. Rotational Velocity Omega_x vs Time
    ax = axes[0, 1]
    ax.plot(t_sim * 1e3, omega_x, color="#2ca02c", lw=2.2, label=r"$\Omega_x$ (Pitch/Tumble)")
    if "Omega_y" in df_tumble.columns and "Omega_z" in df_tumble.columns:
        ax.plot(t_sim * 1e3, df_tumble["Omega_y"], color="#ff7f0e", lw=1.2, linestyle="--", label=r"$\Omega_y$ (Spin)")
        ax.plot(t_sim * 1e3, df_tumble["Omega_z"], color="#9467bd", lw=1.2, linestyle=":", label=r"$\Omega_z$ (Roll)")
    ax.axhline(-exp_omega_t, color="#d62728", linestyle="--", lw=1.5, label=rf"Exp Mean $|\omega_t|={exp_omega_t}$ rad/s")
    ax.set_title(r"Angular Velocities $\mathbf{\Omega}(t)$", fontsize=13, fontweight="bold")
    ax.set_xlabel("Time (ms)", fontsize=11)
    ax.set_ylabel("Angular Velocity (rad/s)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", frameon=True)
    
    # 3. Attitude / Pitch Angle vs Time
    ax = axes[1, 0]
    ax.plot(t_sim * 1e3, angle_x, color="#8c564b", lw=2.2, label=r"Pitch Angle $\theta_x$")
    ax.set_title(r"Disk Attitude / Pitch Angle $\theta_x(t)$", fontsize=13, fontweight="bold")
    ax.set_xlabel("Time (ms)", fontsize=11)
    ax.set_ylabel("Attitude Angle (deg)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", frameon=True)
    
    # 4. Aerodynamic Pitching Moment Mx vs Time
    ax = axes[1, 1]
    if "Moment_x" in df_tumble.columns:
        ax.plot(t_sim * 1e3, df_tumble["Moment_x"] * 1e9, color="#e377c2", lw=2.2, label=r"Aero Moment $M_x$")
    ax.set_title(r"Aerodynamic Pitching Moment $M_x(t)$", fontsize=13, fontweight="bold")
    ax.set_xlabel("Time (ms)", fontsize=11)
    ax.set_ylabel("Moment (nN·m)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best", frameon=True)
    
    plt.tight_layout()
    out_png = os.path.join(script_dir, args.output)
    plt.savefig(out_png, dpi=200)
    print(f"\nSaved clean validation plot to: {out_png}")

if __name__ == "__main__":
    main()
