#!/usr/bin/env python3
"""
plot_motion.py - Post-processing and trajectory analysis for rodRoller tutorial.
Parses freeBody6DoFPimpleFoam log file and plots 2D/6DoF kinematics:
- Fall velocity V_y(t)
- Spin velocity Omega_z(t) [RPM and rad/s]
- Forces: Aero Drag vs Gravity
- Aero Moment M_z(t)
"""

import re
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def parse_log(logfile_path):
    times = []
    vels = []
    accs = []
    faeros = []
    fgravs = []
    fnets = []
    omegas = []
    angles = []
    moments = []

    time_re = re.compile(r"^Time = ([\d\.\+eE-]+)")
    vel_re = re.compile(r"^V = ([\d\.\+eE-]+) m/s")
    acc_re = re.compile(r"^a = ([\d\.\+eE-]+) m/s2")
    fgrav_re = re.compile(r"^Fgravity = ([\d\.\+eE-]+) N")
    faero_re = re.compile(r"^Faero = ([\d\.\+eE-]+) N")
    fnet_re = re.compile(r"^Fnet = \(([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\) N")
    motion_re = re.compile(r"freeBody6DoF Motion:\s+Omega = \(([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\) rad/s.*?Angles = \(([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\) deg")
    moment_re = re.compile(r"Aero Moment = \(([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\) N\.m")

    cur_time = None
    cur_v = None
    cur_a = None
    cur_fg = None
    cur_fa = None
    cur_fn = (0.0, 0.0, 0.0)
    cur_om = (0.0, 0.0, 0.0)
    cur_ang = (0.0, 0.0, 0.0)
    cur_mom = (0.0, 0.0, 0.0)

    with open(logfile_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            tm = time_re.match(line)
            if tm:
                if cur_time is not None and cur_v is not None:
                    times.append(cur_time)
                    vels.append(cur_v)
                    accs.append(cur_a if cur_a is not None else 0.0)
                    faeros.append(cur_fa if cur_fa is not None else 0.0)
                    fgravs.append(cur_fg if cur_fg is not None else 0.0)
                    fnets.append(cur_fn)
                    omegas.append(cur_om)
                    angles.append(cur_ang)
                    moments.append(cur_mom)
                cur_time = float(tm.group(1))
                continue

            vm = vel_re.match(line)
            if vm:
                cur_v = float(vm.group(1))
                continue

            am = acc_re.match(line)
            if am:
                cur_a = float(am.group(1))
                continue

            fgm = fgrav_re.match(line)
            if fgm:
                cur_fg = float(fgm.group(1))
                continue

            fam = faero_re.match(line)
            if fam:
                cur_fa = float(fam.group(1))
                continue

            fnm = fnet_re.match(line)
            if fnm:
                cur_fn = (float(fnm.group(1)), float(fnm.group(2)), float(fnm.group(3)))
                continue

            motm = motion_re.search(line)
            if motm:
                cur_om = (float(motm.group(1)), float(motm.group(2)), float(motm.group(3)))
                cur_ang = (float(motm.group(4)), float(motm.group(5)), float(motm.group(6)))
                continue

            mom = moment_re.search(line)
            if mom:
                cur_mom = (float(mom.group(1)), float(mom.group(2)), float(mom.group(3)))
                continue

        # Append last step
        if cur_time is not None and cur_v is not None:
            times.append(cur_time)
            vels.append(cur_v)
            accs.append(cur_a if cur_a is not None else 0.0)
            faeros.append(cur_fa if cur_fa is not None else 0.0)
            fgravs.append(cur_fg if cur_fg is not None else 0.0)
            fnets.append(cur_fn)
            omegas.append(cur_om)
            angles.append(cur_ang)
            moments.append(cur_mom)

    return {
        "time": np.array(times),
        "V": np.array(vels),
        "a": np.array(accs),
        "Faero": np.array(faeros),
        "Fgravity": np.array(fgravs),
        "Fnet": np.array(fnets),
        "omega": np.array(omegas),
        "angles": np.array(angles),
        "moment": np.array(moments)
    }

def main():
    case_dir = Path(__file__).resolve().parent
    logfile = case_dir / "log.freeBody6DoFPimpleFoam"

    if not logfile.exists():
        print(f"Log file not found at {logfile}")
        sys.exit(1)

    data = parse_log(logfile)
    t = data["time"]
    if len(t) == 0:
        print("No time steps found in log file.")
        sys.exit(1)

    print(f"Parsed {len(t)} time steps from t = {t[0]:.4f} to {t[-1]:.4f} s")
    print(f"Final V_y: {data['V'][-1]:.4f} m/s")
    print(f"Final Omega_z: {data['omega'][-1, 2]:.4f} rad/s ({(data['omega'][-1, 2]*60/(2*np.pi)):.2f} RPM)")
    print(f"Final Angle_z: {data['angles'][-1, 2]:.2f} deg")

    fig, axs = plt.subplots(2, 2, figsize=(12, 9))
    plt.subplots_adjust(hspace=0.28, wspace=0.25)

    # 1. Fall velocity V(t)
    ax = axs[0, 0]
    ax.plot(t, data["V"], "b-", lw=2, label=r"$V_y(t)$ (fall speed)")
    ax.set_title("Vertical Fall Velocity", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 2. Angular Velocity Omega_z(t)
    ax = axs[0, 1]
    ax.plot(t, data["omega"][:, 2], "r-", lw=2, label=r"$\omega_z$ [rad/s]")
    ax_rpm = ax.twinx()
    rpm = data["omega"][:, 2] * 60.0 / (2.0 * np.pi)
    ax_rpm.plot(t, rpm, "r--", alpha=0.5, label="RPM")
    ax_rpm.set_ylabel("Spin [RPM]", color="r")
    ax.set_title(r"Rod Axial Rotation $\omega_z(t)$", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Angular Velocity [rad/s]")
    ax.grid(True, alpha=0.3)

    # 3. Dynamic Forces
    ax = axs[1, 0]
    ax.plot(t, data["Faero"], "g-", lw=2, label=r"$F_{\mathrm{aero}}$ (drag)")
    ax.plot(t, np.abs(data["Fgravity"]), "k--", lw=1.5, label=r"$|F_{\mathrm{grav}}|$ ($mg$)")
    ax.set_title("Force Balance", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Force [N]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 4. Aerodynamic Moment
    ax = axs[1, 1]
    ax.plot(t, data["moment"][:, 2], "m-", lw=2, label=r"$M_z$ (aerodynamic torque)")
    ax.set_title(r"Aerodynamic Torque $M_z(t)$", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Torque [$N\cdot m$]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    out_png = case_dir / "rodRoller_kinematics.png"
    plt.savefig(out_png, dpi=200, bbox_inches="tight")
    print(f"Plot saved to {out_png}")

if __name__ == "__main__":
    main()
