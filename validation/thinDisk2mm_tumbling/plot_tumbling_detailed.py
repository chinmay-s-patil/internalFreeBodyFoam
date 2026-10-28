#!/usr/bin/env python3
"""
plot_tumbling_detailed.py — High-resolution 6-panel validation analysis for thin disk tumbling.
Compares steady-state continuous autorotation against Tinklenberg et al. (2023, JFM 962, A3).
Explicitly separates settling-velocity-derived C_D from instantaneous surface-integrated force coefficients.
"""

import os
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def main():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    case_dir = os.path.join(script_dir, "case")
    post_dir = os.path.join(case_dir, "postProcessing")
    
    tumble_file = os.path.join(post_dir, "tumbleMotion", "motion.dat")
    falling_file = os.path.join(post_dir, "fallingMotion", "motion.dat")
    
    if not os.path.exists(tumble_file) or not os.path.exists(falling_file):
        print(f"[Error] Motion files not found in {post_dir}")
        sys.exit(1)
        
    with open(tumble_file) as f:
        headers_t = f.readline().lstrip("#").strip().split()
    df_t = pd.read_csv(tumble_file, sep=r"\s+", comment="#", names=headers_t)
    df_t.columns = [c.split("(")[0].replace("#","").strip() for c in df_t.columns]

    FALLING_COLS = [
        "Time", "Altitude", "Vx", "Vy", "Vz", "Ax", "Ay", "Az",
        "gx", "gy", "gz", "rho", "a_sound", "Mach",
        "Umean_x", "Umean_y", "Umean_z", "Urel_x", "Urel_y", "Urel_z",
        "Fpress_x", "Fpress_y", "Fpress_z", "Fvisc_x", "Fvisc_y", "Fvisc_z",
        "Faero_x", "Faero_y", "Faero_z", "Fgrav_x", "Fgrav_y", "Fgrav_z",
        "Fnet_x", "Fnet_y", "Fnet_z", "Fext_x", "Fext_y", "Fext_z"
    ]
    df_f = pd.read_csv(falling_file, sep=r"\s+", names=FALLING_COLS)

    # Discard initial 30 startup rows
    df_t = df_t.iloc[30:].copy()
    df_f = df_f.iloc[30:].copy()

    df = pd.merge_asof(df_t.sort_values("Time"), df_f.sort_values("Time"), on="Time", direction="nearest")

    t_ms = df["Time"].values * 1000.0  # ms
    t_s = df["Time"].values
    vy = np.abs(df["Vy"].values)
    vx = df["Vx"].values
    vz = df["Vz"].values
    v_horiz = np.sqrt(vx**2 + vz**2)
    v_tot = np.sqrt(vx**2 + vy**2 + vz**2)

    omega_x = df["Omega_x"].values
    omega_y = df["Omega_y"].values
    omega_z = df["Omega_z"].values
    angle_x = df["Angle_x"].values
    moment_x = df["Moment_x"].values * 1e9  # nN*m

    mass = 2.2113e-7
    g = 9.81
    W = mass * g
    rho = 1.200
    D = 2.02e-3
    A = np.pi * (D / 2.0)**2

    faero = np.sqrt(df["Faero_x"]**2 + df["Faero_y"]**2 + df["Faero_z"]**2).values
    
    # 1. Surface-integrated instantaneous aerodynamic force coefficient
    cd_aero = faero / (0.5 * rho * v_tot**2 * A)
    
    # 2. Paper's terminal settling formula: C_D = 2 W / (rho A V_y^2)
    # Evaluated once fall speed develops (Vy > 0.4 m/s)
    valid_vy = vy > 0.4
    cd_paper = np.full_like(vy, np.nan)
    cd_paper[valid_vy] = (2.0 * W) / (rho * A * vy[valid_vy]**2)

    # Unwrapped angle (revolutions)
    unwrapped_turns = np.unwrap(np.radians(angle_x)) / (2.0 * np.pi)
    continuous_deg = np.degrees(np.unwrap(np.radians(angle_x)))

    # Cycle extraction for like-for-like cycle-averaged comparisons
    revs = []
    target_rev = 1
    for i in range(1, len(continuous_deg)):
        target_deg = -360.0 * target_rev
        if continuous_deg[i] <= target_deg and continuous_deg[i-1] > target_deg:
            frac = (target_deg - continuous_deg[i-1]) / (continuous_deg[i] - continuous_deg[i-1])
            t_cross = t_s[i-1] + frac * (t_s[i] - t_s[i-1])
            t_prev = 0.0 if target_rev == 1 else revs[-1]['t_end']
            revs.append({
                'rev': target_rev,
                't_start': t_prev,
                't_end': t_cross
            })
            target_rev += 1

    trapz_func = np.trapezoid if hasattr(np, 'trapezoid') else np.trapz
    for r in revs:
        t0, t1 = r['t_start'], r['t_end']
        w0 = np.interp(t0, t_s, omega_x)
        w1 = np.interp(t1, t_s, omega_x)
        mask = (t_s > t0) & (t_s < t1)
        sub_t = np.concatenate(([t0], t_s[mask], [t1]))
        sub_w = np.concatenate(([w0], omega_x[mask], [w1]))
        r['om_mean'] = trapz_func(sub_w, sub_t) / (t1 - t0)

    # Trajectory integration
    dt_arr = np.gradient(t_s)
    y_fall = np.cumsum(vy * dt_arr) * 1000.0  # mm
    z_drift = np.cumsum(vz * dt_arr) * 1000.0  # mm

    # Instantaneous trajectory inclination angle psi(t)
    psi_deg = np.degrees(np.arctan2(v_horiz, vy))

    fig, axes = plt.subplots(3, 2, figsize=(14, 13))
    fig.patch.set_facecolor("#F8F9FA")

    # 1. Fall velocity
    ax = axes[0, 0]
    ax.plot(t_ms, vy, color="#1565C0", lw=2.2, label=r"freeBody6DoF CFD $V_y(t)$")
    ax.axhline(1.23, color="#C62828", ls="--", lw=1.8, label=r"Exp Mean $V_t = 1.23$ m/s (Table 3)")
    ax.axhspan(1.23 - 0.31, 1.23 + 0.31, color="#FFCDD2", alpha=0.5, label=r"Exp $1\sigma$ Spread ($0.92 - 1.54$ m/s)")
    ax.set_title(r"Descent Fall Speed $V_y(t)$", fontweight="bold", fontsize=12)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Fall Speed (m/s)")
    ax.legend(loc="lower right")
    ax.grid(True, ls=":", alpha=0.6)

    # 2. Angular velocity (Instantaneous & Cycle-Averaged Overlay)
    ax = axes[0, 1]
    ax.plot(t_ms, omega_x, color="#4CAF50", lw=1.2, alpha=0.7, label=r"Instantaneous $\Omega_x(t)$ (Pitch/Tumble)")
    ax.plot(t_ms, omega_y, color="#F57C00", lw=1.1, ls="--", label=r"$\Omega_y$ (Spin, out-of-plane)")
    ax.plot(t_ms, omega_z, color="#7B1FA2", lw=1.1, ls=":", label=r"$\Omega_z$ (Roll, out-of-plane)")

    for idx, r in enumerate(revs):
        t0_ms = r['t_start'] * 1000.0
        t1_ms = r['t_end'] * 1000.0
        w_m = r['om_mean']
        lbl = r"Sim Cycle-Avg $\langle\Omega_x\rangle$ (Like-for-Like)" if idx == 0 else ""
        ax.plot([t0_ms, t1_ms], [w_m, w_m], color="#1B5E20", lw=3.2, zorder=5, label=lbl)
        ax.plot(0.5 * (t0_ms + t1_ms), w_m, marker="o", color="#1B5E20", markersize=5, zorder=6)
        ax.annotate(f"R{r['rev']}", xy=(0.5 * (t0_ms + t1_ms), w_m), xytext=(0, 6), textcoords="offset points", ha="center", fontsize=8, fontweight="bold", color="#1B5E20")

    ax.axhline(-200.0, color="#C62828", ls="--", lw=1.6, label=r"Exp Observed Tumbling Rate $\approx 200$ rad/s (Fig. 18)")
    ax.axhspan(-228.7, -188.5, color="#FFCDD2", alpha=0.35, label=r"Exp Peak Freq Band ($2\pi f_t \approx 188 - 229$ rad/s, Fig. 20a)")
    ax.set_title(r"Angular Velocities $\mathbf{\Omega}(t)$ & Cycle-Averages vs. Exp Data", fontweight="bold", fontsize=12)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Angular Velocity (rad/s)")
    ax.legend(loc="lower left", fontsize=8.5)
    ax.grid(True, ls=":", alpha=0.6)

    # 3. Cumulative Revolutions
    ax = axes[1, 0]
    ax.plot(t_ms, np.abs(unwrapped_turns), color="#00838F", lw=2.2, label=r"Completed Turns ($N_\mathrm{rev}$)")
    ax.set_title("Cumulative Tumbling Revolutions (Monotonic Autorotation)", fontweight="bold", fontsize=12)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Revolutions (Turns)")
    ax.grid(True, ls=":", alpha=0.6)
    ax.legend(loc="upper left")

    # 4. Aerodynamic Pitching Moment
    ax = axes[1, 1]
    ax.plot(t_ms, moment_x, color="#AD1457", lw=2.2, label=r"Aero Pitch Moment $M_x(t)$")
    ax.axhline(0, color="gray", ls="-", lw=0.8)
    ax.set_title(r"Aerodynamic Pitching Moment $M_x(t)$", fontweight="bold", fontsize=12)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Pitching Moment (nN·m)")
    ax.grid(True, ls=":", alpha=0.6)
    ax.legend(loc="lower left")

    # 5. Trajectory Y vs Z & Instantaneous Inclination
    ax = axes[2, 0]
    ax.plot(z_drift, -y_fall, color="#4527A0", lw=2.2, label=r"Flight Path (Glide in $Y-Z$)")
    ax.set_title(r"2D Descent Trajectory (Flight in $Y-Z$ Plane)", fontweight="bold", fontsize=12)
    ax.set_xlabel("Lateral Drift Z (mm)")
    ax.set_ylabel("Descent Distance Y (mm)")
    ax.grid(True, ls=":", alpha=0.6)
    ax.legend(loc="lower left")

    # 6. Aerodynamic Force & Drag Coefficients (Explicit Definitions & Masked Startup)
    ax = axes[2, 1]
    ax.plot(t_ms, cd_aero, color="#D84315", lw=1.8, alpha=0.85, label=r"Resultant Force Coeff $C_{F,\mathrm{mag}}(t) = |\mathbf{F}_\mathrm{aero}|/(\frac{1}{2}\rho V^2 A)$")

    t_settle_start_ms = revs[3]['t_start'] * 1000.0 if len(revs) >= 4 else 111.3
    mask_settle = (t_ms >= t_settle_start_ms) & valid_vy

    ax.axvspan(0, t_settle_start_ms, color="#E0E0E0", alpha=0.5, label=r"Startup Transient ($V_y < V_t$; Settling Formula Not Physical)")
    ax.plot(t_ms[mask_settle], cd_paper[mask_settle], color="#0277BD", lw=2.2, label=r"Settling Force Equiv $C_{D,\mathrm{paper}}(t) = 2W/(\rho A V_y^2)$ ($t \geq 111$ ms)")
    ax.axhline(0.746, color="#C62828", ls="--", lw=1.8, label=r"Exp Mean $C_{D,\mathrm{paper}} = 0.746$ (from Mean $V_t = 1.23$ m/s)")
    ax.axhspan(0.47, 1.33, color="#FFCDD2", alpha=0.35, label=r"Transformed $V_t \pm 1\sigma$ Envelope $[0.47, 1.33]$")
    ax.set_title(r"Aerodynamic Coefficients (Settling Formula vs. Resultant Force)", fontweight="bold", fontsize=12)
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel(r"Aerodynamic Coefficient ($C_D$ / $C_{F,\mathrm{mag}}$)")
    ax.set_ylim([0, 2.0])
    ax.grid(True, ls=":", alpha=0.6)
    ax.legend(loc="upper right", fontsize=8.0)

    plt.tight_layout()
    out_file = os.path.join(script_dir, "tumbling_validation_detailed.png")
    plt.savefig(out_file, dpi=200)
    print(f"[Done] Detailed validation plot saved to: {out_file}")

if __name__ == "__main__":
    main()
