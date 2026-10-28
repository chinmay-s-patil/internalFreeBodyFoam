#!/usr/bin/env python3
"""
plot_rotation.py - Analyze and plot propeller rotation metrics from sixDoFRotatingMotion.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator

def main():
    case_dir = os.path.dirname(os.path.abspath(__file__))
    motion_file = os.path.join(case_dir, "postProcessing", "sixDoFRotatingMotion", "motion.dat")

    if not os.path.exists(motion_file):
        print(f"Error: {motion_file} not found!")
        sys.exit(1)

    # Read data
    data = []
    with open(motion_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 5:
                try:
                    data.append([float(p) for p in parts[:5]])
                except ValueError:
                    continue

    if not data:
        print("No valid data points found in motion.dat!")
        sys.exit(1)

    data = np.array(data)
    time_s = data[:, 0]
    time_ms = time_s * 1000.0  # ms
    torque = data[:, 1]        # N.m
    omega = data[:, 2]         # rad/s
    rpm = data[:, 3]           # RPM
    angle_deg = data[:, 4]     # deg
    revs = np.abs(angle_deg) / 360.0  # completed revolutions

    # Propeller physical properties
    I_yy = 6.5e-5  # kg.m^2
    mass = 0.012   # kg (12 g)
    E_rot_mJ = 0.5 * I_yy * (omega**2) * 1000.0  # mJ
    power_W = np.abs(torque * omega)             # Watts
    alpha_rad_s2 = torque / I_yy                 # rad/s^2

    # Latest values
    t_end_ms = time_ms[-1]
    t_end_s = time_s[-1]
    curr_rpm = np.abs(rpm[-1])
    curr_omega = np.abs(omega[-1])
    curr_angle = np.abs(angle_deg[-1])
    curr_revs = revs[-1]
    curr_torque = np.abs(torque[-1])
    curr_alpha = np.abs(alpha_rad_s2[-1])
    curr_Erot = E_rot_mJ[-1]
    curr_power = power_W[-1]

    # Print summary
    print("=" * 65)
    print("       PROPELLER DYNAMIC ROTATION ANALYSIS SUMMARY       ")
    print("=" * 65)
    print(f"Total Flow Time Simulated : {t_end_s:.6f} s  ({t_end_ms:.3f} ms)")
    print(f"Total Time Steps Logged   : {len(time_s)}")
    print(f"Current Rotational Speed  : {curr_rpm:.2f} RPM  ({curr_omega:.3f} rad/s)")
    print(f"Total Accumulated Rotation: {curr_angle:.4f}°  ({curr_revs:.5f} revolutions)")
    print(f"Current Aerodynamic Torque: {curr_torque:.4f} N.m")
    print(f"Angular Acceleration      : {curr_alpha:.1f} rad/s²")
    print(f"Rotational Kinetic Energy : {curr_Erot:.3f} mJ")
    print(f"Aerodynamic Spin Power    : {curr_power:.3f} W")
    
    # Extrapolations based on current acceleration
    if curr_alpha > 0:
        time_to_1st_rev_ms = np.sqrt(2.0 * (2.0 * np.pi) / curr_alpha) * 1000.0
        print(f"Est. Time to complete 1 full turn (360°): ~{time_to_1st_rev_ms:.2f} ms")
    print("=" * 65)

    # Plot styling
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(2, 2, figsize=(14, 9.5))
    fig.patch.set_facecolor('#0d1117')

    palette = {
        'bg': '#161b22',
        'grid': '#30363d',
        'text': '#c9d1d9',
        'blue': '#58a6ff',
        'teal': '#39d353',
        'orange': '#f0883e',
        'purple': '#bc8cff',
        'red': '#ff7b72'
    }

    for ax in axes.flat:
        ax.set_facecolor(palette['bg'])
        ax.tick_params(colors=palette['text'], which='both', labelsize=10)
        ax.grid(True, linestyle='--', alpha=0.35, color=palette['grid'])
        for spine in ax.spines.values():
            spine.set_color(palette['grid'])

    # --- Plot 1: Angular Velocity & RPM ---
    ax1 = axes[0, 0]
    line1 = ax1.plot(time_ms, np.abs(omega), color=palette['blue'], lw=2.2, label=r'$|\omega|$ [rad/s]')
    ax1.set_xlabel("Physical Flow Time [ms]", color=palette['text'], fontsize=11, fontweight='bold')
    ax1.set_ylabel(r"Angular Velocity $|\omega|$ [rad/s]", color=palette['blue'], fontsize=11, fontweight='bold')
    ax1.set_title(f"Propeller Spin-Up Speed (Current: {curr_rpm:.1f} RPM)", color=palette['text'], fontsize=12, fontweight='bold')

    ax1_twin = ax1.twinx()
    ax1_twin.set_ylabel("Rotational Speed [RPM]", color=palette['teal'], fontsize=11, fontweight='bold')
    ax1_twin.plot(time_ms, np.abs(rpm), color=palette['teal'], lw=2.0, linestyle=':', alpha=0.85, label='RPM')
    ax1_twin.tick_params(colors=palette['teal'], labelsize=10)
    ax1_twin.grid(False)

    ax1.annotate(f"{curr_rpm:.1f} RPM\n({curr_omega:.1f} rad/s)",
                 xy=(t_end_ms, curr_omega),
                 xytext=(t_end_ms * 0.70, curr_omega * 0.85),
                 color=palette['teal'], fontweight='bold', fontsize=10,
                 arrowprops=dict(facecolor=palette['teal'], edgecolor='none', shrink=0.08, width=1.5, headwidth=6))

    # --- Plot 2: Total Accumulated Rotation Angle & Revolutions ---
    ax2 = axes[0, 1]
    ax2.plot(time_ms, curr_angle * (time_ms / t_end_ms)**2 if curr_angle == 0 else np.abs(angle_deg),
             color=palette['orange'], lw=2.2, label=r'Angle $|\theta|$ [deg]')
    ax2.set_xlabel("Physical Flow Time [ms]", color=palette['text'], fontsize=11, fontweight='bold')
    ax2.set_ylabel(r"Accumulated Rotation $|\theta|$ [deg]", color=palette['orange'], fontsize=11, fontweight='bold')
    ax2.set_title(f"Accumulated Rotation Angle (Current: {curr_angle:.3f}°)", color=palette['text'], fontsize=12, fontweight='bold')

    ax2_twin = ax2.twinx()
    ax2_twin.set_ylabel("Revolutions [turns]", color=palette['purple'], fontsize=11, fontweight='bold')
    ax2_twin.plot(time_ms, revs, color=palette['purple'], lw=1.8, linestyle='--', alpha=0.75)
    ax2_twin.tick_params(colors=palette['purple'], labelsize=10)
    ax2_twin.grid(False)

    ax2.annotate(f"{curr_angle:.3f}°\n({curr_revs:.4f} rev)",
                 xy=(t_end_ms, curr_angle),
                 xytext=(t_end_ms * 0.65, curr_angle * 0.70),
                 color=palette['orange'], fontweight='bold', fontsize=10,
                 arrowprops=dict(facecolor=palette['orange'], edgecolor='none', shrink=0.08, width=1.5, headwidth=6))

    # --- Plot 3: Aerodynamic Driving Torque ---
    ax3 = axes[1, 0]
    # Filter startup impulse for clear visualization if needed
    t_plot = time_ms[1:] if len(time_ms) > 1 else time_ms
    q_plot = np.abs(torque[1:]) if len(torque) > 1 else np.abs(torque)
    ax3.plot(t_plot, q_plot, color=palette['red'], lw=2.0, label=r'$|Q_y|$ [N$\cdot$m]')
    ax3.set_xlabel("Physical Flow Time [ms]", color=palette['text'], fontsize=11, fontweight='bold')
    ax3.set_ylabel(r"Aerodynamic Torque $|Q_y|$ [N$\cdot$m]", color=palette['red'], fontsize=11, fontweight='bold')
    ax3.set_title(f"Aerodynamic Driving Torque (Current: {curr_torque:.4f} N.m)", color=palette['text'], fontsize=12, fontweight='bold')
    ax3.set_ylim([0, max(0.5, np.max(q_plot[len(q_plot)//5:]) * 1.3)])

    ax3.axhline(curr_torque, color='#8b949e', linestyle=':', alpha=0.6)
    ax3.annotate(f"Steady driving torque\n~ {curr_torque:.3f} N.m",
                 xy=(t_end_ms, curr_torque),
                 xytext=(t_end_ms * 0.50, curr_torque * 1.15),
                 color=palette['red'], fontweight='bold', fontsize=10,
                 arrowprops=dict(facecolor=palette['red'], edgecolor='none', shrink=0.08, width=1.5, headwidth=6))

    # --- Plot 4: Kinetic Energy & Aerodynamic Power ---
    ax4 = axes[1, 1]
    ax4.plot(time_ms, E_rot_mJ, color=palette['teal'], lw=2.2, label=r'$E_{rot} = \frac{1}{2}I\omega^2$ [mJ]')
    ax4.set_xlabel("Physical Flow Time [ms]", color=palette['text'], fontsize=11, fontweight='bold')
    ax4.set_ylabel(r"Rotational Kinetic Energy [mJ]", color=palette['teal'], fontsize=11, fontweight='bold')
    ax4.set_title(f"Energy & Spin Power (Current Power: {curr_power:.2f} W)", color=palette['text'], fontsize=12, fontweight='bold')

    ax4_twin = ax4.twinx()
    ax4_twin.set_ylabel(r"Aero Spin Power $P = |Q\cdot\omega|$ [W]", color='#e3b341', fontsize=11, fontweight='bold')
    t_pwr = time_ms[2:] if len(time_ms) > 2 else time_ms
    pwr_plot = power_W[2:] if len(power_W) > 2 else power_W
    ax4_twin.plot(t_pwr, pwr_plot, color='#e3b341', lw=1.8, linestyle='-.', alpha=0.85)
    if len(pwr_plot) > 10:
        ax4_twin.set_ylim([0, max(3.0, np.max(pwr_plot[5:]) * 1.35)])
    ax4_twin.tick_params(colors='#e3b341', labelsize=10)
    ax4_twin.grid(False)

    fig.tight_layout(rect=[0, 0.02, 1, 0.93])
    fig.suptitle(f"Propeller Free-Fall Spin-Up Dynamics (Inflow U = 70 m/s, Iyy = {I_yy*1e6:.0f} g·cm²)\nSimulated Flow Time: {t_end_ms:.2f} ms ({len(time_s)} Time Steps)",
                 color=palette['text'], fontsize=14, fontweight='bold', y=0.985)

    # Save to case dir
    out_case = os.path.join(case_dir, "rotation_plot.png")
    plt.savefig(out_case, dpi=200, bbox_inches='tight', facecolor=fig.get_facecolor(), edgecolor='none')
    print(f"Plot saved to: {out_case}")

    # Copy to artifacts directory
    artifact_dir = "/home/lavender/.gemini/antigravity-ide/brain/1a886228-ca85-4b7c-8ec1-3d683a36a1fb"
    if os.path.exists(artifact_dir):
        out_art = os.path.join(artifact_dir, "rotation_plot.png")
        plt.savefig(out_art, dpi=200, bbox_inches='tight', facecolor=fig.get_facecolor(), edgecolor='none')
        print(f"Plot copied to artifact: {out_art}")

    plt.close()

if __name__ == "__main__":
    main()
