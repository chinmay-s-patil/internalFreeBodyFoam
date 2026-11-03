#!/usr/bin/env python3
"""
track_motion.py - Live tracking and publication plotting for steelBallRun.

Usage:
    python3 track_motion.py --watch      # Live dashboard in terminal
    python3 track_motion.py --plot       # Generate 4-panel publication plot
    python3 track_motion.py              # Parse and print summary
"""

import argparse
import os
import re
import sys
import time
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

    if not os.path.exists(logfile_path):
        return None

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

    if len(times) == 0:
        return None

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

def watch_dashboard(logfile):
    print("\033[2J\033[H", end="")
    print("=== steelBallRun 3D 6DoF Real-Time Monitor ===")
    print("Press Ctrl+C to exit.\n")
    try:
        while True:
            data = parse_log(logfile)
            if data and len(data["time"]) > 0:
                t = data["time"][-1]
                v = data["V"][-1]
                a = data["a"][-1]
                om = data["omega"][-1]
                ang = data["angles"][-1]
                fa = data["Faero"][-1]
                fg = data["Fgravity"][-1]
                fn = data["Fnet"][-1]
                mom = data["moment"][-1]
                rpm = om * 60.0 / (2.0 * np.pi)

                print("\033[3;1H", end="")
                print(f"Time:         {t:10.4f} s    (Total Timesteps: {len(data['time'])})")
                print(f"Fall Speed:   {v:10.4f} m/s  | Acceleration: {a:10.4f} m/s^2")
                print("-" * 65)
                print(f"3D Spin Rate: Omega = ({om[0]:8.2f}, {om[1]:8.2f}, {om[2]:8.2f}) rad/s")
                print(f"              RPM   = ({rpm[0]:8.1f}, {rpm[1]:8.1f}, {rpm[2]:8.1f}) RPM")
                print(f"Attitude:     Euler = ({ang[0]:8.2f}, {ang[1]:8.2f}, {ang[2]:8.2f}) deg")
                print("-" * 65)
                print(f"Forces:       F_drag = {fa:9.4f} N  | F_grav = {fg:9.4f} N")
                print(f"              F_net  = ({fn[0]:8.4f}, {fn[1]:8.4f}, {fn[2]:8.4f}) N")
                print(f"Aero Torque:  M_aero = ({mom[0]:9.2e}, {mom[1]:9.2e}, {mom[2]:9.2e}) N.m")
                print("-" * 65)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting monitor.")

def generate_plots(data, out_path):
    t = data["time"]
    fig, axs = plt.subplots(2, 2, figsize=(13, 9.5))
    plt.subplots_adjust(hspace=0.28, wspace=0.25)

    # 1. Fall Velocity V(t)
    ax = axs[0, 0]
    ax.plot(t, data["V"], "b-", lw=2, label=r"$V_y(t)$ (fall speed)")
    ax.set_title("Vertical Fall Velocity $V_y(t)$", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 2. 3D Angular Velocity Vector Omega(t)
    ax = axs[0, 1]
    ax.plot(t, data["omega"][:, 0], "r-", lw=1.8, label=r"$\omega_x$ (roll)")
    ax.plot(t, data["omega"][:, 1], "g-", lw=1.8, label=r"$\omega_y$ (pitch)")
    ax.plot(t, data["omega"][:, 2], "b-", lw=1.8, label=r"$\omega_z$ (yaw)")
    ax.set_title(r"3D Rotational Velocity $\vec{\omega}(t)$", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Angular Velocity [rad/s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 3. Dynamic Force Balance
    ax = axs[1, 0]
    ax.plot(t, data["Faero"], "g-", lw=2, label=r"$F_{\mathrm{aero}}$ (drag)")
    ax.plot(t, np.abs(data["Fgravity"]), "k--", lw=1.5, label=r"$|F_{\mathrm{grav}}|$ ($mg = 5.01\text{ N}$)")
    ax.set_title("Dynamic Force Balance", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Force [N]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # 4. 3D Aerodynamic Torque Vector
    ax = axs[1, 1]
    ax.plot(t, data["moment"][:, 0], "r-", lw=1.8, label=r"$M_x$")
    ax.plot(t, data["moment"][:, 1], "g-", lw=1.8, label=r"$M_y$")
    ax.plot(t, data["moment"][:, 2], "b-", lw=1.8, label=r"$M_z$")
    ax.set_title(r"3D Hydrodynamic Torque $\vec{M}_{\mathrm{aero}}(t)$", fontsize=12, fontweight="bold")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel(r"Torque [$N\cdot m$]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"Publication kinematics plot saved to {out_path}")

def main():
    case_dir = Path(__file__).resolve().parent
    logfile = case_dir / "log.freeBody6DoFPimpleFoam"

    parser = argparse.ArgumentParser(description="Track and analyze steelBallRun 6DoF kinematics.")
    parser.add_argument("--watch", action="store_true", help="Live monitor dashboard in terminal")
    parser.add_argument("--plot", action="store_true", help="Generate publication kinematics plots")
    args = parser.parse_args()

    if args.watch:
        watch_dashboard(logfile)
        return

    data = parse_log(logfile)
    if not data or len(data["time"]) == 0:
        print(f"No trajectory data found in {logfile}. Run the simulation first.")
        sys.exit(1)

    print(f"Parsed {len(data['time'])} time steps from t = {data['time'][0]:.4f} to {data['time'][-1]:.4f} s")
    print(f"Final V: {data['V'][-1]:.4f} m/s")
    print(f"Final Omega: {data['omega'][-1]} rad/s")
    print(f"Final Angles: {data['angles'][-1]} deg")

    if args.plot or True:
        out_png = case_dir / "steelBallRun_kinematics.png"
        generate_plots(data, out_png)

if __name__ == "__main__":
    main()
