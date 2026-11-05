#!/usr/bin/env python3
"""
track_motion.py — 6DoF Motion & Tumble Dynamics Tracker for OpenFOAM propTumble

Tracks and analyzes:
  1. Rotational speed: Omega (rad/s), RPM (roll, pitch, yaw) and total magnitude
  2. Rotation occurred: Euler angles (deg), accumulated revolutions (revs), attitude quaternion
  3. Aerodynamic moments: Mx, My, Mz (N.m) and total torque
  4. Translational free-fall: Descent velocity (m/s), acceleration (m/s^2), altitude fallen (m),
     aerodynamic drag vs gravity force balance, and Mach number.

Usage:
  python3 track_motion.py                  # Display current motion status summary
  python3 track_motion.py --watch          # Live terminal dashboard (updates in real time)
  python3 track_motion.py --plot           # Generate 4-panel publication-quality plots (PNG)
  python3 track_motion.py --export out.csv # Export combined 6DoF data to CSV
  python3 track_motion.py --case <path>    # Target specific case directory
"""

import os
import sys
import time
import re
import glob
import argparse
import warnings

# Suppress harmless 3D matplotlib warnings
warnings.filterwarnings("ignore", category=UserWarning)

import numpy as np
import pandas as pd

# Matplotlib headless backend
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


FALLING_COLS_35 = [
    "Time", "Altitude", "Vx", "Vy", "Vz", "Ax", "Ay", "Az",
    "gx", "gy", "gz", "rho", "a_sound", "Mach",
    "Umean_x", "Umean_y", "Umean_z", "Urel_x", "Urel_y", "Urel_z",
    "Fpress_x", "Fpress_y", "Fpress_z", "Fvisc_x", "Fvisc_y", "Fvisc_z",
    "Faero_x", "Faero_y", "Faero_z", "Fgrav_x", "Fgrav_y", "Fgrav_z",
    "Fnet_x", "Fnet_y", "Fnet_z"
]

FALLING_COLS_38 = FALLING_COLS_35 + [
    "Fext_x", "Fext_y", "Fext_z"
]

FALLING_COLS = FALLING_COLS_38


def find_files(case_dir):
    """Locate tumbleMotion/motion.dat and fallingMotion/motion.dat with multi-path resolution."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        case_dir,
        os.path.join(case_dir, "case"),
        script_dir,
        os.path.join(script_dir, "case"),
        os.path.join(case_dir, "validation", "thinDisk2mm", "case"),
        os.path.join(case_dir, "cases", "propTumble6DoF"),
    ]
    
    tumble_file = None
    falling_file = None
    
    # 1. Search candidates for tumbleMotion
    for base in candidates:
        if not os.path.isdir(base):
            continue
        p = os.path.join(base, "postProcessing", "tumbleMotion", "motion.dat")
        if os.path.exists(p) and os.path.getsize(p) > 0:
            tumble_file = p
            break
        p_proc = os.path.join(base, "processor0", "postProcessing", "tumbleMotion", "motion.dat")
        if os.path.exists(p_proc) and os.path.getsize(p_proc) > 0:
            tumble_file = p_proc
            break
            
    # 2. Search candidates for fallingMotion
    for base in candidates:
        if not os.path.isdir(base):
            continue
        p = os.path.join(base, "postProcessing", "fallingMotion", "motion.dat")
        if os.path.exists(p) and os.path.getsize(p) > 0:
            falling_file = p
            break
        p_proc = os.path.join(base, "processor0", "postProcessing", "fallingMotion", "motion.dat")
        if os.path.exists(p_proc) and os.path.getsize(p_proc) > 0:
            falling_file = p_proc
            break

    # 3. Recursive glob fallback if still not found
    if not tumble_file or not os.path.exists(tumble_file):
        matches = glob.glob(os.path.join(case_dir, "**", "tumbleMotion", "motion.dat"), recursive=True)
        if matches:
            tumble_file = matches[0]
            
    if not falling_file or not os.path.exists(falling_file):
        matches = glob.glob(os.path.join(case_dir, "**", "fallingMotion", "motion.dat"), recursive=True)
        if matches:
            falling_file = matches[0]

    # Fallback paths if files do not exist yet
    if not tumble_file:
        tumble_file = os.path.join(case_dir, "postProcessing", "tumbleMotion", "motion.dat")
    if not falling_file:
        falling_file = os.path.join(case_dir, "postProcessing", "fallingMotion", "motion.dat")

    return tumble_file, falling_file


def load_tumble_data(tumble_path):
    """Load rotational 6DoF dynamics from tumbleMotion/motion.dat."""
    if not os.path.exists(tumble_path) or os.path.getsize(tumble_path) == 0:
        return None

    try:
        # Read header from line starting with '#'
        with open(tumble_path, "r") as f:
            first_line = f.readline().strip()
        
        if first_line.startswith("#"):
            headers = [h for h in first_line.lstrip("#").strip().split() if h]
            df = pd.read_csv(tumble_path, sep=r"\s+", comment="#", names=headers)
        else:
            df = pd.read_csv(tumble_path, sep=r"\s+")

        # Normalize column names
        col_map = {}
        for c in df.columns:
            clean = c.split("(")[0].replace("#", "").strip()
            col_map[c] = clean
        df.rename(columns=col_map, inplace=True)

        # Compute total angular velocity and RPM
        if {"Omega_x", "Omega_y", "Omega_z"}.issubset(df.columns):
            df["Omega_mag"] = np.sqrt(df["Omega_x"]**2 + df["Omega_y"]**2 + df["Omega_z"]**2)
        if {"RPM_x", "RPM_y", "RPM_z"}.issubset(df.columns):
            df["RPM_mag"] = np.sqrt(df["RPM_x"]**2 + df["RPM_y"]**2 + df["RPM_z"]**2)
        if {"Moment_x", "Moment_y", "Moment_z"}.issubset(df.columns):
            df["Moment_mag"] = np.sqrt(df["Moment_x"]**2 + df["Moment_y"]**2 + df["Moment_z"]**2)

        # Compute accumulated revolutions by numerical integration: int(RPM / 60 dt)
        if "Time" in df.columns and "RPM_x" in df.columns:
            t = df["Time"].values
            dt = np.diff(t, prepend=t[0])
            df["Revs_x"] = np.cumsum(df["RPM_x"].values * dt / 60.0)
            df["Revs_y"] = np.cumsum(df["RPM_y"].values * dt / 60.0)
            df["Revs_z"] = np.cumsum(df["RPM_z"].values * dt / 60.0)
            df["Revs_total"] = np.cumsum(df["RPM_mag"].values * dt / 60.0)

        # Total 3D angle from initial orientation (from quaternion qw)
        if "Quat_w" in df.columns:
            qw = np.clip(np.abs(df["Quat_w"].values), -1.0, 1.0)
            df["Angle_3D_deg"] = 2.0 * np.arccos(qw) * (180.0 / np.pi)

        return df
    except Exception as e:
        print(f"[Warning] Failed to parse tumble data: {e}", file=sys.stderr)
        return None


def load_falling_data(falling_path):
    """Load translational free-fall kinematics from fallingMotion/motion.dat."""
    if not os.path.exists(falling_path) or os.path.getsize(falling_path) == 0:
        return None

    try:
        with open(falling_path, "r") as f:
            first_line = f.readline().strip()

        df = None

        if first_line.startswith("#"):
            headers = [h for h in first_line.lstrip("#").strip().split() if h]
            try:
                df = pd.read_csv(falling_path, sep=r"\s+", comment="#", names=headers)
            except Exception:
                df = None
        else:
            first_cols = len(first_line.split())
            cols = FALLING_COLS_38 if first_cols >= 38 else FALLING_COLS_35
            try:
                df = pd.read_csv(falling_path, sep=r"\s+", names=cols)
            except Exception:
                df = None

        # Fallback for mixed column files (e.g. 35 columns before restart, 38 after restart)
        if df is None:
            with open(falling_path, "r") as f:
                lines = f.readlines()

            data_35, data_38 = [], []
            for line in lines:
                s = line.strip()
                if not s or s.startswith("#"):
                    continue
                parts = s.split()
                if len(parts) == 35:
                    data_35.append(parts)
                elif len(parts) >= 38:
                    data_38.append(parts[:38])

            arr35 = np.array(data_35, dtype=np.float64) if data_35 else np.empty((0, 35))
            arr38 = np.array(data_38, dtype=np.float64) if data_38 else np.empty((0, 38))

            if len(arr35) > 0 and len(arr38) > 0:
                pad35 = np.hstack([arr35, np.zeros((len(arr35), 3))])
                all_arr = np.vstack([pad35, arr38])
            elif len(arr38) > 0:
                all_arr = arr38
            elif len(arr35) > 0:
                all_arr = np.hstack([arr35, np.zeros((len(arr35), 3))])
            else:
                return None

            df = pd.DataFrame(all_arr, columns=FALLING_COLS_38)

        # Ensure Fext columns exist
        for col in ["Fext_x", "Fext_y", "Fext_z"]:
            if col not in df.columns:
                df[col] = 0.0

        if "Vx" in df.columns and "Vy" in df.columns and "Vz" in df.columns:
            df["V_mag"] = np.sqrt(df["Vx"]**2 + df["Vy"]**2 + df["Vz"]**2)
            df["V_horiz"] = np.sqrt(df["Vx"]**2 + df["Vz"]**2)
        if "Ax" in df.columns and "Ay" in df.columns and "Az" in df.columns:
            df["A_mag"] = np.sqrt(df["Ax"]**2 + df["Ay"]**2 + df["Az"]**2)
            df["A_horiz"] = np.sqrt(df["Ax"]**2 + df["Az"]**2)
        if "Faero_x" in df.columns and "Faero_y" in df.columns and "Faero_z" in df.columns:
            df["Faero_mag"] = np.sqrt(df["Faero_x"]**2 + df["Faero_y"]**2 + df["Faero_z"]**2)
            df["Faero_horiz"] = np.sqrt(df["Faero_x"]**2 + df["Faero_z"]**2)
        if "Fext_x" in df.columns and "Fext_y" in df.columns and "Fext_z" in df.columns:
            df["Fext_mag"] = np.sqrt(df["Fext_x"]**2 + df["Fext_y"]**2 + df["Fext_z"]**2)
            df["Fext_horiz"] = np.sqrt(df["Fext_x"]**2 + df["Fext_z"]**2)
        if "Fnet_x" in df.columns and "Fnet_y" in df.columns and "Fnet_z" in df.columns:
            df["Fnet_mag"] = np.sqrt(df["Fnet_x"]**2 + df["Fnet_y"]**2 + df["Fnet_z"]**2)
            df["Fnet_horiz"] = np.sqrt(df["Fnet_x"]**2 + df["Fnet_z"]**2)

        # Numerical integration for 3D trajectory and horizontal drift displacements
        if "Time" in df.columns and len(df) > 0:
            t = df["Time"].values
            dt = np.diff(t, prepend=t[0])
            if len(dt) > 1 and dt[0] == 0:
                dt[0] = dt[1]

            if "Vx" in df.columns:
                vx = df["Vx"].values
                vx_mid = 0.5 * (vx + np.roll(vx, 1))
                vx_mid[0] = vx[0]
                df["Disp_x"] = np.cumsum(vx_mid * dt)
            else:
                df["Disp_x"] = 0.0

            if "Vz" in df.columns:
                vz = df["Vz"].values
                vz_mid = 0.5 * (vz + np.roll(vz, 1))
                vz_mid[0] = vz[0]
                df["Disp_z"] = np.cumsum(vz_mid * dt)
            else:
                df["Disp_z"] = 0.0

            if "Altitude" in df.columns:
                df["Disp_y"] = df["Altitude"].values
            elif "Vy" in df.columns:
                vy = df["Vy"].values
                vy_mid = 0.5 * (vy + np.roll(vy, 1))
                vy_mid[0] = vy[0]
                df["Disp_y"] = np.cumsum(vy_mid * dt)
            else:
                df["Disp_y"] = 0.0

            df["Drift_horiz"] = np.sqrt(df["Disp_x"]**2 + df["Disp_z"]**2)
            df["Disp_3D"] = np.sqrt(df["Disp_x"]**2 + df["Disp_y"]**2 + df["Disp_z"]**2)

            if "V_horiz" in df.columns and "Vy" in df.columns:
                vy_safe = np.maximum(np.abs(df["Vy"].values), 1e-9)
                df["Drift_angle_deg"] = np.arctan2(df["V_horiz"].values, vy_safe) * (180.0 / np.pi)
            else:
                df["Drift_angle_deg"] = 0.0

        return df
    except Exception as e:
        print(f"[Warning] Failed to parse falling data: {e}", file=sys.stderr)
        return None


def get_start_times_from_logs(case_dir):
    """
    Scan case directory for OpenFOAM solver log files (e.g. log*, log.*, log*.freeBody*)
    and extract all simulation start and restart times.
    """
    log_files = sorted(glob.glob(os.path.join(case_dir, "log*")))
    start_times = []

    for log_path in log_files:
        if not os.path.isfile(log_path):
            continue
        try:
            with open(log_path, "r", errors="ignore") as f:
                for line in f:
                    if "Create mesh for time =" in line:
                        m = re.search(r"Create mesh for time =\s*([\d\.eE\+-]+)", line)
                        if m:
                            start_times.append(float(m.group(1)))
                            break
                    elif line.startswith("Time = ") and not start_times:
                        m = re.search(r"Time =\s*([\d\.eE\+-]+)", line)
                        if m:
                            start_times.append(float(m.group(1)))
                            break
        except Exception:
            pass

    return sorted(list(set(start_times)))


def filter_restart_transients(df, log_start_times=None, skip_points=20, skip_time=0.002, skip_initial=True):
    """
    Filter out startup and restart numerical transient data points.

    When OpenFOAM restarts a run with sliding AMI / 6DoF coupling, the first ~5
    iterations experience a numerical pressure-velocity relaxation transient
    (e.g. initial residual spike before AMI continuity converges), causing
    large artificial spikes in acceleration and force balance.

    This function ignores:
      1. Initial simulation start (first `skip_points` rows after t_start)
      2. In-file time rewinds / duplicate timestamps (t_i <= t_{i-1})
      3. First `skip_points` data points immediately following any log-detected restart time.

    Returns:
      cleaned_df (pd.DataFrame): DataFrame with transient points removed and clean index.
      summary_events (list): List of detected restart event dictionaries.
    """
    if df is None or df.empty or "Time" not in df.columns or skip_points <= 0:
        return df, []

    t = df["Time"].values
    n = len(t)
    drop_indices = set()
    summary_events = []

    # 1. Initial simulation start (skip points and skip physical time)
    if skip_initial and n > 0:
        drop_range = list(range(0, min(skip_points, n)))
        for k in drop_range:
            drop_indices.add(df.index[k])
        if skip_time > 0:
            time_drop = np.where(t < skip_time)[0]
            for k in time_drop:
                drop_indices.add(df.index[k])
        summary_events.append({
            "type": "Initial start",
            "time": float(t[0]),
            "points": len(drop_indices)
        })

    # 2. Timestamp rewinds or duplicates in data file (t_i <= t_{i-1})
    rewind_locs = np.where(np.diff(t) <= 0)[0]
    for r_idx in rewind_locs:
        if t[r_idx] == t[r_idx + 1]:
            drop_indices.add(df.index[r_idx])
        drop_range = list(range(r_idx + 1, min(r_idx + 1 + skip_points, n)))
        for k in drop_range:
            drop_indices.add(df.index[k])
        summary_events.append({
            "type": "Data time rewind",
            "time": float(t[r_idx + 1]),
            "points": len(drop_range)
        })

    # 3. Known restart start times from OpenFOAM logs
    if log_start_times:
        for st in log_start_times:
            if abs(st - t[0]) < 1e-5 and skip_initial:
                continue
            locs = np.where(t > st)[0]
            if len(locs) > 0:
                first_step_idx = locs[0]
                if abs(t[first_step_idx] - st) < 0.05:
                    drop_range = list(range(first_step_idx, min(first_step_idx + skip_points, n)))
                    for k in drop_range:
                        drop_indices.add(df.index[k])
                    summary_events.append({
                        "type": f"Log restart at {st:.4f}s",
                        "time": float(t[first_step_idx]),
                        "points": len(drop_range)
                    })

    cleaned_df = df.drop(index=list(drop_indices)).reset_index(drop=True)
    return cleaned_df, summary_events


def print_dashboard(df_rot, df_trans, clear=False, status_line=None):
    """Render a clean ANSI terminal dashboard of the latest motion state."""
    if clear:
        os.system("clear" if os.name == "posix" else "cls")

    print("=" * 80)
    print("         6DoF PROPELLER TUMBLE & ROTATION DYNAMICS TRACKER")
    if status_line:
        print(f" {status_line}")
    print("=" * 80)

    if df_rot is not None and not df_rot.empty:
        last_rot = df_rot.iloc[-1]
        t = last_rot.get("Time", 0.0)
        print(f" SIMULATION TIME: {t:.6f} s   |   SAMPLES: {len(df_rot):,}")
        print("-" * 80)

        # 1. Rotational Speed
        ox, oy, oz = last_rot.get("Omega_x", 0.0), last_rot.get("Omega_y", 0.0), last_rot.get("Omega_z", 0.0)
        rx, ry, rz = last_rot.get("RPM_x", 0.0), last_rot.get("RPM_y", 0.0), last_rot.get("RPM_z", 0.0)
        omag = last_rot.get("Omega_mag", np.sqrt(ox**2 + oy**2 + oz**2))
        rmag = last_rot.get("RPM_mag", np.sqrt(rx**2 + ry**2 + rz**2))

        print("[1] ROTATIONAL SPEED (ANGULAR VELOCITY):")
        print(f"  • Total Speed : {omag:12.3f} rad/s  ({rmag:12.2f} RPM)")
        print(f"  • Pitch (X)   : {ox:12.3f} rad/s  ({rx:12.2f} RPM)")
        print(f"  • Spin/Yaw (Y): {oy:12.3f} rad/s  ({ry:12.2f} RPM)")
        print(f"  • Span/Roll(Z): {oz:12.3f} rad/s  ({rz:12.2f} RPM)")
        print()

        # 2. Rotation Occurred
        ax, ay, az = last_rot.get("Angle_x", 0.0), last_rot.get("Angle_y", 0.0), last_rot.get("Angle_z", 0.0)
        rev_x = last_rot.get("Revs_x", 0.0)
        rev_y = last_rot.get("Revs_y", 0.0)
        rev_z = last_rot.get("Revs_z", 0.0)
        rev_tot = last_rot.get("Revs_total", 0.0)
        ang_3d = last_rot.get("Angle_3D_deg", 0.0)

        print("[2] ROTATION OCCURRED & ORIENTATION:")
        print(f"  • Total 3D Tilt Angle : {ang_3d:10.3f}° from initial orientation")
        print(f"  • Total Accumulated   : {rev_tot:10.4f} revolutions completed")
        print(f"  • Pitch Axis (X)      : {ax:10.3f}°  |  Revs: {rev_x:+9.4f} revs")
        print(f"  • Spin Axis (Y)       : {ay:10.3f}°  |  Revs: {rev_y:+9.4f} revs")
        print(f"  • Span Axis (Z)       : {az:10.3f}°  |  Revs: {rev_z:+9.4f} revs")

        qw = last_rot.get("Quat_w", 1.0)
        qx = last_rot.get("Quat_x", 0.0)
        qy = last_rot.get("Quat_y", 0.0)
        qz = last_rot.get("Quat_z", 0.0)
        print(f"  • Attitude Quaternion : ({qw:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})")
        print()

        # 3. Aerodynamic Moments
        mx, my, mz = last_rot.get("Moment_x", 0.0), last_rot.get("Moment_y", 0.0), last_rot.get("Moment_z", 0.0)
        mmag = last_rot.get("Moment_mag", np.sqrt(mx**2 + my**2 + mz**2))
        print("[3] AERODYNAMIC TORQUES & MOMENTS:")
        print(f"  • Total Torque : {mmag:12.6e} N·m")
        print(f"  • Pitch (Mx)   : {mx:12.6e} N·m")
        print(f"  • Spin  (My)   : {my:12.6e} N·m")
        print(f"  • Span  (Mz)   : {mz:12.6e} N·m")
        print()

    else:
        print("[Notice] No rotational tumbleMotion/motion.dat found yet.")

    # 4. Translational Free Fall & Lateral Drift (3 DOFs)
    if df_trans is not None and not df_trans.empty:
        last_tr = df_trans.iloc[-1]
        alt = last_tr.get("Altitude", 0.0)
        vx = last_tr.get("Vx", 0.0)
        vy = last_tr.get("Vy", 0.0)
        vz = last_tr.get("Vz", 0.0)
        vmag = last_tr.get("V_mag", np.sqrt(vx**2 + vy**2 + vz**2))
        vhoriz = last_tr.get("V_horiz", np.sqrt(vx**2 + vz**2))
        mach = last_tr.get("Mach", 0.0)
        drift_ang = last_tr.get("Drift_angle_deg", np.arctan2(vhoriz, max(abs(vy), 1e-9)) * 180.0 / np.pi)

        ax = last_tr.get("Ax", 0.0)
        ay = last_tr.get("Ay", -9.81)
        az = last_tr.get("Az", 0.0)
        amag = last_tr.get("A_mag", np.sqrt(ax**2 + ay**2 + az**2))
        ahoriz = last_tr.get("A_horiz", np.sqrt(ax**2 + az**2))

        disp_x = last_tr.get("Disp_x", 0.0)
        disp_y = last_tr.get("Disp_y", alt)
        disp_z = last_tr.get("Disp_z", 0.0)
        drift_h = last_tr.get("Drift_horiz", np.sqrt(disp_x**2 + disp_z**2))
        disp_3d = last_tr.get("Disp_3D", np.sqrt(disp_x**2 + disp_y**2 + disp_z**2))

        faero_x = last_tr.get("Faero_x", 0.0)
        faero_y = last_tr.get("Faero_y", 0.0)
        faero_z = last_tr.get("Faero_z", 0.0)
        faero_h = last_tr.get("Faero_horiz", np.sqrt(faero_x**2 + faero_z**2))

        fgrav_x = last_tr.get("Fgrav_x", 0.0)
        fgrav_y = last_tr.get("Fgrav_y", -0.1129)
        fgrav_z = last_tr.get("Fgrav_z", 0.0)

        fext_x = last_tr.get("Fext_x", 0.0)
        fext_y = last_tr.get("Fext_y", 0.0)
        fext_z = last_tr.get("Fext_z", 0.0)

        fnet_x = last_tr.get("Fnet_x", faero_x + fgrav_x + fext_x)
        fnet_y = last_tr.get("Fnet_y", faero_y + fgrav_y + fext_y)
        fnet_z = last_tr.get("Fnet_z", faero_z + fgrav_z + fext_z)
        fnet_mag = last_tr.get("Fnet_mag", np.sqrt(fnet_x**2 + fnet_y**2 + fnet_z**2))

        print("-" * 80)
        print("[4] TRANSLATIONAL FREE-FALL & LATERAL DRIFT KINEMATICS (3 DOFs):")
        print(f"  • Velocity (3D)        : |V| = {vmag:10.4f} m/s  (Mach {mach:.4f})")
        print(f"    - Vertical (Vy)      : {vy:12.4f} m/s  (Descent speed: {abs(vy):.4f} m/s)")
        print(f"    - Lateral X (Vx)     : {vx:12.4f} m/s  (Side drift)")
        print(f"    - Lateral Z (Vz)     : {vz:12.4f} m/s  (Longitudinal drift)")
        print(f"    - Horiz. Drift Speed : {vhoriz:12.4f} m/s  | Drift Angle: {drift_ang:6.2f}° from vertical")
        print()
        print(f"  • Acceleration (3D)    : |A| = {amag:10.4f} m/s² ({amag/9.81:.3f} g)")
        print(f"    - Vertical (Ay)      : {ay:12.4f} m/s² ({ay/9.81:+.3f} g)")
        print(f"    - Lateral X (Ax)     : {ax:12.4f} m/s² ({ax/9.81:+.3f} g)")
        print(f"    - Lateral Z (Az)     : {az:12.4f} m/s² ({az/9.81:+.3f} g)")
        print(f"    - Horiz. Accel (Ah)  : {ahoriz:12.4f} m/s² ({ahoriz/9.81:.3f} g)")
        print()
        print(f"  • Displacements (3D)   : Total 3D Distance = {disp_3d:10.4f} m")
        print(f"    - Altitude Fallen (Y): {abs(disp_y):12.4f} m")
        print(f"    - Lateral Drift (X)  : {disp_x:+12.4f} m")
        print(f"    - Lateral Drift (Z)  : {disp_z:+12.4f} m")
        print(f"    - Horiz. Drift Dist  : {drift_h:12.4f} m (Ground track offset)")
        print()
        print("  • Aerodynamic Force Balance (3D):")
        print(f"    - Vertical (Y)       : Gravity = {fgrav_y:+.5f} N | Drag = {faero_y:+.5f} N | Net = {fnet_y:+.5f} N")
        print(f"    - Lateral X          : Aero    = {faero_x:+.5f} N | Ext  = {fext_x:+.5f} N | Net = {fnet_x:+.5f} N")
        print(f"    - Lateral Z          : Aero    = {faero_z:+.5f} N | Ext  = {fext_z:+.5f} N | Net = {fnet_z:+.5f} N")
        print(f"    - Horiz. Aero Force  : {faero_h:12.5f} N  | Total Net Force: {fnet_mag:.5f} N")
    print("=" * 80)


def generate_plots(df_rot, df_trans, output_path="render/motion_tracking_report.png", silent=False, filter_note=None):
    """
    Generate two high-resolution 4-panel publication-quality motion plots:
      1. Rotational & Aerodynamic Loads (_rotational.png)
      2. 3D Translational Kinematics & Trajectory (_translational.png)
    """
    if df_rot is None and df_trans is None:
        if not silent:
            print("[Error] No data available to plot.", file=sys.stderr)
        return []

    root, ext = os.path.splitext(output_path)
    if not ext:
        ext = ".png"
    out_rot = f"{root}_rotational{ext}"
    out_trans = f"{root}_translational{ext}"
    os.makedirs(os.path.dirname(os.path.abspath(out_rot)), exist_ok=True)
    saved_files = []

    # ====================================================
    # FIGURE 1: Rotational Dynamics & Aerodynamic Loads (2x2)
    # ====================================================
    fig1, axes1 = plt.subplots(2, 2, figsize=(14, 11), dpi=150)
    fig1.suptitle("6DoF Propeller Dynamics — Rotational & Aerodynamic Loads", fontsize=15, fontweight="bold", y=0.985)
    if filter_note:
        fig1.text(0.5, 0.958, f"Note: {filter_note}", ha="center", fontsize=8.5, fontstyle="italic", color="#555555")

    # [1] Rotational Speed (RPM) — Total removed
    ax1 = axes1[0, 0]
    if df_rot is not None and "Time" in df_rot.columns:
        t = df_rot["Time"]
        ax1.plot(t, df_rot["RPM_x"], label="Pitch (RPM_x)", color="#d9534f", lw=1.8)
        ax1.plot(t, df_rot["RPM_y"], label="Spin (RPM_y)", color="#0275d8", lw=1.8)
        ax1.plot(t, df_rot["RPM_z"], label="Span (RPM_z)", color="#5cb85c", lw=1.8)
        ax1.set_title("[1] Rotational Speed (RPM)", fontweight="bold")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Rotational Speed (RPM)")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="best", frameon=True, fontsize=8.5)
    else:
        ax1.text(0.5, 0.5, "Rotational data not available", ha="center", va="center", color="gray")

    # [2] Orientation & Angles (deg) — Total removed
    ax2 = axes1[0, 1]
    if df_rot is not None and "Time" in df_rot.columns:
        t = df_rot["Time"]
        ax2.plot(t, df_rot["Angle_x"], label="Pitch Angle (θx)", color="#d9534f", lw=1.8)
        ax2.plot(t, df_rot["Angle_y"], label="Spin Angle (θy)", color="#0275d8", lw=1.8)
        ax2.plot(t, df_rot["Angle_z"], label="Span Angle (θz)", color="#5cb85c", lw=1.8)
        ax2.set_title("[2] Orientation & Angles (deg)", fontweight="bold")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Angle (degrees)")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="best", frameon=True, fontsize=8.5)
    else:
        ax2.text(0.5, 0.5, "Orientation data not available", ha="center", va="center", color="gray")

    # [3] Aerodynamic Torques / Moments (N.m) — Total removed
    ax3 = axes1[1, 0]
    if df_rot is not None and "Moment_x" in df_rot.columns:
        t = df_rot["Time"]
        ax3.plot(t, df_rot["Moment_x"], label="Mx (Pitch)", color="#d9534f", lw=1.8)
        ax3.plot(t, df_rot["Moment_y"], label="My (Spin)", color="#0275d8", lw=1.8)
        ax3.plot(t, df_rot["Moment_z"], label="Mz (Span)", color="#5cb85c", lw=1.8)
        ax3.set_title("[3] Aerodynamic Moments & Torques (N·m)", fontweight="bold")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Moment (N·m)")
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc="best", frameon=True, fontsize=8.5)
    else:
        ax3.text(0.5, 0.5, "Moment data not available", ha="center", va="center", color="gray")

    # [4] 3D Aerodynamic Forces & Balance (N)
    ax4 = axes1[1, 1]
    if df_trans is not None and "Faero_y" in df_trans.columns:
        t_tr = df_trans["Time"]
        ax4.axhline(0, color="gray", lw=0.8, ls="-", alpha=0.6)
        ax4.plot(t_tr, df_trans["Faero_y"], label="Vertical Drag (Faero_y)", color="#0275d8", lw=2.0)
        if "Faero_x" in df_trans.columns:
            ax4.plot(t_tr, df_trans["Faero_x"], label="Lateral Force X (Faero_x)", color="#f0ad4e", lw=1.6)
        if "Faero_z" in df_trans.columns:
            ax4.plot(t_tr, df_trans["Faero_z"], label="Lateral Force Z (Faero_z)", color="#6f42c1", lw=1.6)
        if "Faero_horiz" in df_trans.columns:
            ax4.plot(t_tr, df_trans["Faero_horiz"], label="Total Horiz. Force", color="#20c997", lw=1.5, ls="--")
        if "Fgrav_y" in df_trans.columns:
            ax4.plot(t_tr, df_trans["Fgrav_y"], label="Gravity Force (Fgrav_y)", color="#d9534f", lw=1.4, ls=":")
        if "Fnet_y" in df_trans.columns:
            ax4.plot(t_tr, df_trans["Fnet_y"], label="Net Vertical (Fnet_y)", color="#5cb85c", lw=1.4, ls="-.")
        ax4.set_title("[4] 3D Aerodynamic & Net Forces (N)", fontweight="bold")
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Force (N)")
        ax4.grid(True, alpha=0.3)
        ax4.legend(loc="best", frameon=True, fontsize=8.5)
    else:
        ax4.text(0.5, 0.5, "Force data not available", ha="center", va="center", color="gray")

    fig1.tight_layout()
    fig1.savefig(out_rot, dpi=200)
    plt.close(fig1)
    saved_files.append(out_rot)

    # ====================================================
    # FIGURE 2: 3D Translational Kinematics & Trajectory (2x2)
    # ====================================================
    fig2, axes2 = plt.subplots(2, 2, figsize=(15, 11), dpi=150)
    fig2.suptitle("6DoF Propeller Dynamics — 3D Translation & Lateral Drift", fontsize=15, fontweight="bold", y=0.985)
    if filter_note:
        fig2.text(0.5, 0.958, f"Note: {filter_note}", ha="center", fontsize=8.5, fontstyle="italic", color="#555555")

    # [5] 3D Translational Velocities (m/s) — Separate scales for Vertical (Y) and Lateral (X, Z)
    ax5 = axes2[0, 0]
    if df_trans is not None and "Vy" in df_trans.columns:
        t_tr = df_trans["Time"]
        ax5.axhline(0, color="#0275d8", lw=0.8, ls="--", alpha=0.4)
        ax5.plot(t_tr, df_trans["Vy"], label="Descent Speed Vy", color="#0275d8", lw=2.0)
        if "V_mag" in df_trans.columns:
            ax5.plot(t_tr, -df_trans["V_mag"], label="Total Speed -|V|", color="#292b2c", lw=1.3, ls=":")
        ax5.set_title("[5] 3D Translational Velocities", fontweight="bold")
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("Vertical Velocity Vy (m/s)", color="#0275d8")
        ax5.tick_params(axis='y', labelcolor="#0275d8")
        ax5.grid(True, alpha=0.3)

        # Secondary axis for Lateral X & Z velocities
        ax5_lat = ax5.twinx()
        has_lat5 = False
        ax5_lat.axhline(0, color="gray", lw=0.7, ls=":", alpha=0.5)
        if "Vx" in df_trans.columns:
            ax5_lat.plot(t_tr, df_trans["Vx"], label="Lateral Speed Vx", color="#f0ad4e", lw=1.6)
            has_lat5 = True
        if "Vz" in df_trans.columns:
            ax5_lat.plot(t_tr, df_trans["Vz"], label="Lateral Speed Vz", color="#6f42c1", lw=1.6)
            has_lat5 = True
        if "V_horiz" in df_trans.columns:
            ax5_lat.plot(t_tr, df_trans["V_horiz"], label="Horiz. Speed Vh", color="#20c997", lw=1.5, ls="--")
            has_lat5 = True

        if has_lat5:
            ax5_lat.set_ylabel("Lateral Velocity Vx, Vz (m/s)", color="#333333")
            ax5_lat.tick_params(axis='y', labelcolor="#333333")
            ax5_lat.grid(False)
            lines_1, labels_1 = ax5.get_legend_handles_labels()
            lines_2, labels_2 = ax5_lat.get_legend_handles_labels()
            ax5_lat.legend(lines_1 + lines_2, labels_1 + labels_2, loc="lower left", frameon=True, fontsize=8.5)
        else:
            ax5_lat.remove()
            ax5.legend(loc="best", frameon=True, fontsize=8.5)
    else:
        ax5.text(0.5, 0.5, "Velocity data not available", ha="center", va="center", color="gray")

    # [6] 3D Linear Accelerations & G-Force (m/s²) — Total removed
    ax6 = axes2[0, 1]
    if df_trans is not None and "Ay" in df_trans.columns:
        t_tr = df_trans["Time"]
        ax6.axhline(0, color="gray", lw=0.8, ls="--", alpha=0.5, label="Equilibrium (0 m/s²)")
        ax6.axhline(-9.81, color="#d9534f", lw=0.8, ls=":", alpha=0.7, label="Free-Fall g (-9.81 m/s²)")
        ax6.plot(t_tr, df_trans["Ay"], label="Vertical Accel Ay", color="#0275d8", lw=2.0)
        if "Ax" in df_trans.columns:
            ax6.plot(t_tr, df_trans["Ax"], label="Lateral Accel Ax", color="#f0ad4e", lw=1.6)
        if "Az" in df_trans.columns:
            ax6.plot(t_tr, df_trans["Az"], label="Lateral Accel Az", color="#6f42c1", lw=1.6)
        if "A_horiz" in df_trans.columns:
            ax6.plot(t_tr, df_trans["A_horiz"], label="Horiz. Accel Ah", color="#20c997", lw=1.5, ls="--")
        ax6.set_title("[6] 3D Linear Accelerations & G-Force", fontweight="bold")
        ax6.set_xlabel("Time (s)")
        ax6.set_ylabel("Acceleration (m/s²)", color="#0275d8")
        ax6.tick_params(axis='y', labelcolor="#0275d8")
        ax6.grid(True, alpha=0.3)

        # Secondary axis for G-force
        ax6_g = ax6.twinx()
        ax6_g.set_ylim(ax6.get_ylim()[0] / 9.81, ax6.get_ylim()[1] / 9.81)
        ax6_g.set_ylabel("G-Force (g)", color="#333333")
        ax6_g.tick_params(axis='y', labelcolor="#333333")
        ax6.legend(loc="lower right", frameon=True, fontsize=8.5)
    else:
        ax6.text(0.5, 0.5, "Acceleration data not available", ha="center", va="center", color="gray")

    # [7] 3D Displacements vs Time (m) — Separate scales for Vertical (Y) and Lateral (X, Z)
    ax7 = axes2[1, 0]
    if df_trans is not None and "Disp_y" in df_trans.columns:
        t_tr = df_trans["Time"]
        ax7.plot(t_tr, df_trans["Disp_y"], label="Altitude Y (Fall Distance)", color="#0275d8", lw=2.0)
        ax7.set_title("[7] 3D Displacements & Trajectory vs Time", fontweight="bold")
        ax7.set_xlabel("Time (s)")
        ax7.set_ylabel("Vertical Displacement Y (m)", color="#0275d8")
        ax7.tick_params(axis='y', labelcolor="#0275d8")
        ax7.grid(True, alpha=0.3)

        # Secondary axis for Lateral X & Z displacements
        ax7_lat = ax7.twinx()
        has_lat7 = False
        ax7_lat.axhline(0, color="gray", lw=0.7, ls=":", alpha=0.5)
        if "Disp_x" in df_trans.columns:
            ax7_lat.plot(t_tr, df_trans["Disp_x"], label="Lateral Drift X (Δx)", color="#f0ad4e", lw=1.6)
            has_lat7 = True
        if "Disp_z" in df_trans.columns:
            ax7_lat.plot(t_tr, df_trans["Disp_z"], label="Lateral Drift Z (Δz)", color="#6f42c1", lw=1.6)
            has_lat7 = True

        if has_lat7:
            ax7_lat.set_ylabel("Lateral Displacement X, Z (m)", color="#333333")
            ax7_lat.tick_params(axis='y', labelcolor="#333333")
            ax7_lat.grid(False)
            lines_1, labels_1 = ax7.get_legend_handles_labels()
            lines_2, labels_2 = ax7_lat.get_legend_handles_labels()
            ax7_lat.legend(lines_1 + lines_2, labels_1 + labels_2, loc="lower left", frameon=True, fontsize=8.5)
        else:
            ax7_lat.remove()
            ax7.legend(loc="best", frameon=True, fontsize=8.5)
    else:
        ax7.text(0.5, 0.5, "Displacement data not available", ha="center", va="center", color="gray")

    # [8] Horizontal Ground Track / Lateral Drift (X vs Z)
    ax8 = axes2[1, 1]
    if df_trans is not None and "Disp_x" in df_trans.columns and "Disp_z" in df_trans.columns:
        dx = df_trans["Disp_x"].values
        dz = df_trans["Disp_z"].values
        t_tr = df_trans["Time"].values

        sc = ax8.scatter(dx, dz, c=t_tr, cmap="plasma", s=18, alpha=0.85, edgecolors="none")
        cbar = plt.colorbar(sc, ax=ax8, fraction=0.046, pad=0.04)
        cbar.set_label("Simulation Time (s)", fontsize=8)
        cbar.ax.tick_params(labelsize=8)

        ax8.plot(dx, dz, color="#555555", lw=0.8, alpha=0.5, zorder=1)
        ax8.plot(dx[0], dz[0], marker="o", markersize=8, color="#5cb85c", label="Start (0, 0)", zorder=5)
        ax8.plot(dx[-1], dz[-1], marker="s", markersize=8, color="#d9534f", label=f"Latest ({dx[-1]:+.2e}, {dz[-1]:+.2e})", zorder=5)

        ax8.axvline(0, color="gray", lw=0.8, ls="--", alpha=0.5)
        ax8.axhline(0, color="gray", lw=0.8, ls="--", alpha=0.5)

        max_h = np.max(df_trans.get("Drift_horiz", np.sqrt(dx**2 + dz**2)))
        final_h = np.sqrt(dx[-1]**2 + dz[-1]**2)
        final_ang = np.arctan2(dz[-1], dx[-1]) * 180.0 / np.pi
        stat_txt = f"Max Horiz Drift: {max_h:.4e} m\nFinal Drift: {final_h:.4e} m\nDrift Heading: {final_ang:+.1f}°"
        ax8.text(0.04, 0.94, stat_txt, transform=ax8.transAxes, fontsize=8,
                 verticalalignment='top', bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.8, edgecolor='#cccccc'))

        ax8.set_title("[8] Horizontal Flight Path & Lateral Drift (X vs Z)", fontweight="bold")
        ax8.set_xlabel("Lateral Displacement X (m)")
        ax8.set_ylabel("Lateral Displacement Z (m)")
        ax8.grid(True, alpha=0.3)
        ax8.legend(loc="lower right", frameon=True, fontsize=8.5)
    else:
        ax8.text(0.5, 0.5, "Horizontal drift data not available", ha="center", va="center", color="gray")

    fig2.tight_layout(w_pad=3.0)
    fig2.savefig(out_trans, dpi=200)
    plt.close(fig2)
    saved_files.append(out_trans)

    if not silent:
        print(f"\n[Success] Saved 4-panel Rotational Dynamics report to:    {out_rot}")
        print(f"[Success] Saved 4-panel 3D Translation & Drift report to: {out_trans}")

    return saved_files


def export_csv(df_rot, df_trans, out_csv):
    """Merge rotational and translational metrics into a single unified CSV."""
    if df_rot is not None and df_trans is not None:
        merged = pd.merge_asof(
            df_rot.sort_values("Time"),
            df_trans.sort_values("Time"),
            on="Time",
            direction="nearest",
            suffixes=("_rot", "_trans")
        )
        merged.to_csv(out_csv, index=False)
    elif df_rot is not None:
        df_rot.to_csv(out_csv, index=False)
    elif df_trans is not None:
        df_trans.to_csv(out_csv, index=False)
    else:
        print("[Error] No data available to export.", file=sys.stderr)
        return
    print(f"[Success] Combined 6DoF data exported to: {out_csv}")


def main():
    parser = argparse.ArgumentParser(
        description="6DoF Propeller Tumbling & Motion Dynamics Tracker",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--case", type=str, default=".", help="Case directory (default: current directory)")
    parser.add_argument("-w", "--watch", action="store_true", help="Live watch mode with automatic refresh")
    parser.add_argument("-p", "--plot", action="store_true", help="Generate publication-quality kinematic plots")
    parser.add_argument("-o", "--output", type=str, default="render/motion_tracking_report.png", help="Plot output filename or path (default: render/motion_tracking_report.png)")
    parser.add_argument("-e", "--export", type=str, default=None, help="Export merged dataset to CSV (default directory: render/)")
    parser.add_argument("--interval", type=float, default=1.0, help="Terminal dashboard refresh interval in seconds (default: 1.0s)")
    parser.add_argument("--plot-interval", type=float, default=10.0, help="Plot refresh interval in seconds in watch mode (default: 10.0s)")
    parser.add_argument("--no-plot", action="store_true", help="Disable plot generation during --watch mode")
    parser.add_argument("--skip-points", type=int, default=30, help="Number of startup points to ignore after start and each restart (default: 30)")
    parser.add_argument("--skip-time", type=float, default=0.002, help="Ignore data points with Time < skip_time (default: 0.002s) to eliminate startup impulses")
    parser.add_argument("--no-filter", action="store_true", help="Disable filtering of startup/restart numerical transients")
    parser.add_argument("--keep-initial", action="store_true", help="Keep the initial 5 points at t=0 (only filter subsequent restarts)")
    parser.add_argument("--verbose-filter", action="store_true", help="Print details of filtered restart events")
    args = parser.parse_args()

    case_dir = os.path.abspath(args.case)
    tumble_file, falling_file = find_files(case_dir)
    log_start_times = get_start_times_from_logs(case_dir)

    if tumble_file and os.path.exists(tumble_file):
        actual_case_dir = os.path.abspath(os.path.dirname(os.path.dirname(os.path.dirname(tumble_file))))
    else:
        actual_case_dir = case_dir

    def resolve_output_path(path_arg):
        if not path_arg:
            path_arg = "render/motion_tracking_report.png"
        if not os.path.isabs(path_arg):
            path_arg = os.path.join(actual_case_dir, path_arg)
        return path_arg

    if args.watch:
        print(f"Monitoring 6DoF dynamics in: {case_dir}")
        if not args.no_filter and log_start_times:
            st_str = ", ".join([f"{st:.3f}s" for st in log_start_times])
            print(f"[Filtering] Ignoring {args.skip_points} points after start & restarts ({st_str}) for plotting.")
        print("Press Ctrl+C to exit.\n")
        last_plot_time = 0.0
        last_plot_samples = -1
        plot_status = "Waiting for data..."
        try:
            while True:
                df_rot = load_tumble_data(tumble_file)
                df_trans = load_falling_data(falling_file)

                # Update plots at regular, slower interval to save CPU resources
                now = time.time()
                if not args.no_plot and (now - last_plot_time >= args.plot_interval):
                    cur_samples = len(df_rot) if df_rot is not None else 0
                    if cur_samples > 0 and cur_samples != last_plot_samples:
                        out_plot = resolve_output_path(args.output)
                        if not args.no_filter:
                            df_rot_plot, _ = filter_restart_transients(df_rot, log_start_times, skip_points=args.skip_points, skip_initial=not args.keep_initial)
                            df_trans_plot, _ = filter_restart_transients(df_trans, log_start_times, skip_points=args.skip_points, skip_initial=not args.keep_initial)
                            f_note = f"Filtered initial {args.skip_points} points after start/restarts"
                        else:
                            df_rot_plot, df_trans_plot = df_rot, df_trans
                            f_note = None
                        generate_plots(df_rot_plot, df_trans_plot, output_path=out_plot, silent=True, filter_note=f_note)
                        last_plot_time = now
                        last_plot_samples = cur_samples
                        plot_status = f"Saved reports at {time.strftime('%H:%M:%S')} ({cur_samples} samples)"
                    elif cur_samples == 0:
                        plot_status = "Waiting for simulation samples..."
                elif args.no_plot:
                    plot_status = "Plotting disabled (--no-plot)"

                status_line = f"LIVE WATCH: Refresh {args.interval}s | PLOT: {plot_status} (interval {args.plot_interval}s)"
                print_dashboard(df_rot, df_trans, clear=True, status_line=status_line)
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped by user.")
            if not args.no_plot and df_rot is not None and len(df_rot) > 0:
                out_plot = resolve_output_path(args.output)
                if not args.no_filter:
                    df_rot_plot, _ = filter_restart_transients(df_rot, log_start_times, skip_points=args.skip_points, skip_initial=not args.keep_initial)
                    df_trans_plot, _ = filter_restart_transients(df_trans, log_start_times, skip_points=args.skip_points, skip_initial=not args.keep_initial)
                    f_note = f"Filtered initial {args.skip_points} points after start/restarts"
                else:
                    df_rot_plot, df_trans_plot = df_rot, df_trans
                    f_note = None
                generate_plots(df_rot_plot, df_trans_plot, output_path=out_plot, silent=False, filter_note=f_note)
            return

    # One-shot execution
    df_rot = load_tumble_data(tumble_file)
    df_trans = load_falling_data(falling_file)

    if not args.no_filter:
        df_rot_filt, ev_rot = filter_restart_transients(df_rot, log_start_times, skip_points=args.skip_points, skip_time=args.skip_time, skip_initial=not args.keep_initial)
        df_trans_filt, ev_trans = filter_restart_transients(df_trans, log_start_times, skip_points=args.skip_points, skip_time=args.skip_time, skip_initial=not args.keep_initial)
        f_note = f"Filtered {args.skip_points} points & t < {args.skip_time:.3f}s to eliminate numerical transients"
    else:
        df_rot_filt, df_trans_filt = df_rot, df_trans
        ev_rot, ev_trans = [], []
        f_note = None

    print_dashboard(df_rot, df_trans, clear=False)

    if not args.no_filter and (ev_rot or ev_trans):
        st_desc = ", ".join([f"{st:.3f}s" for st in log_start_times]) if log_start_times else "detected points"
        print(f"\n[Filtering] Ignored initial {args.skip_points} data points after start & restarts (times: {st_desc}) to eliminate numerical transients.")
        if args.verbose_filter:
            for ev in ev_trans:
                print(f"  • Falling motion: {ev['type']} (t={ev['time']:.5f}s): dropped {ev['points']} transient points")
            for ev in ev_rot:
                print(f"  • Tumble motion:  {ev['type']} (t={ev['time']:.5f}s): dropped {ev['points']} transient points")

    if args.plot:
        out_plot = resolve_output_path(args.output)
        generate_plots(df_rot_filt, df_trans_filt, output_path=out_plot, filter_note=f_note)

    if args.export:
        export_arg = args.export
        if not os.path.dirname(export_arg):
            export_arg = os.path.join("render", export_arg)
        out_csv = os.path.join(case_dir, export_arg)
        os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
        export_csv(df_rot_filt, df_trans_filt, out_csv)


if __name__ == "__main__":
    main()
