#!/usr/bin/env python3
"""
animate_drift.py — 6DoF Thin Disk Motion & Drift Visualizer with Floor Grid & Corner Radar HUD
Validation Case: 2.02 mm PET Thin Disk (50 µm thickness) falling in air
Reference: Tinklenberg, Guala, Coletti (2023), J. Fluid Mech. 962, A3

Combines 3D PyVista simulation rendering with:
  1. Micro-scale Floor Reference Grid & Laser Plumb-Bob (Ground Track):
     - Metric floor grid & concentric range rings below disk (Y = -3.5 mm).
     - 3D ground track ribbon showing cumulative horizontal translation (Dx, Dz).
     - Glowing vertical laser dropline connecting disk center to instantaneous ground track.
     - 3D instantaneous velocity and lateral aerodynamic force vectors at disk hub.
     - Streamlined upward relative airflow streaks representing free-fall wind.
  2. Corner Radar Compass HUD & Avionics Telemetry:
     - Top-right circular radar scope showing top-down horizontal drift (X vs Z in mm).
     - Concentric range rings (0.5 mm, 1.0 mm, 1.5 mm, 2.0 mm) and cardinal crosshairs.
     - Real-time ground track trail and pulsing position blip.
     - Dynamic velocity vector and lateral aerodynamic force vector arrows.
     - Integrated telemetry card: Horiz Drift, Bearing, Horiz Speed, Horiz Accel, Lat Force, Descent Alt/Speed.
     - Altitude tape and Vertical Speed Indicator (VSI).

Hardware Acceleration & Multi-Processing:
  - OpenGL off-screen rendering via PyVista.
  - Multi-core parallel chunk rendering (-j / --workers N).
  - Lossless FFmpeg stream concatenation.
"""

import os
import sys
import glob
import argparse
import time
import math
import shutil
import tempfile
import subprocess
import multiprocessing as mp
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R, Slerp
from PIL import Image, ImageDraw, ImageFont

import warnings
warnings.filterwarnings("ignore")

import pyvista as pv
import imageio

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

def locate_disk_stl(case_dir):
    """Search common case and meshing directories for the disk STL geometry."""
    candidates = [
        os.path.join(case_dir, "meshing/triSurface/disk.stl"),
        os.path.join(case_dir, "case/constant/triSurface/disk.stl"),
        os.path.join(case_dir, "constant/triSurface/disk.stl"),
        os.path.join(case_dir, "../meshing/triSurface/disk.stl"),
        os.path.join(case_dir, "../../meshing/triSurface/disk.stl"),
        os.path.join(case_dir, "meshing/rotatingMesh/constant/triSurface/disk.stl"),
    ]
    for c in candidates:
        abs_p = os.path.abspath(c)
        if os.path.exists(abs_p):
            return abs_p
    matches = glob.glob(os.path.join(case_dir, "**", "disk.stl"), recursive=True)
    if matches:
        return os.path.abspath(matches[0])
    return None

def locate_telemetry_files(case_dir):
    """Locate tumbleMotion/motion.dat and fallingMotion/motion.dat."""
    candidates = [
        case_dir,
        os.path.join(case_dir, "case"),
        os.path.join(case_dir, "validation", "thinDisk2mm", "case"),
    ]
    tumble_p = None
    falling_p = None
    
    for base in candidates:
        if not os.path.isdir(base):
            continue
        p = os.path.join(base, "postProcessing", "tumbleMotion", "motion.dat")
        if os.path.exists(p) and os.path.getsize(p) > 0:
            tumble_p = p
            break
        p_proc = os.path.join(base, "processor0", "postProcessing", "tumbleMotion", "motion.dat")
        if os.path.exists(p_proc) and os.path.getsize(p_proc) > 0:
            tumble_p = p_proc
            break

    for base in candidates:
        if not os.path.isdir(base):
            continue
        p = os.path.join(base, "postProcessing", "fallingMotion", "motion.dat")
        if os.path.exists(p) and os.path.getsize(p) > 0:
            falling_p = p
            break
        p_proc = os.path.join(base, "processor0", "postProcessing", "fallingMotion", "motion.dat")
        if os.path.exists(p_proc) and os.path.getsize(p_proc) > 0:
            falling_p = p_proc
            break

    if not tumble_p:
        matches = glob.glob(os.path.join(case_dir, "**", "tumbleMotion", "motion.dat"), recursive=True)
        if matches:
            tumble_p = matches[0]
    if not falling_p:
        matches = glob.glob(os.path.join(case_dir, "**", "fallingMotion", "motion.dat"), recursive=True)
        if matches:
            falling_p = matches[0]

    return tumble_p, falling_p

def load_and_filter_tumble(tumble_path, skip_points=30, skip_time=0.002):
    """Load attitude quaternions and rotational telemetry from tumbleMotion/motion.dat."""
    if not os.path.exists(tumble_path):
        raise FileNotFoundError(f"Tumble motion file not found: {tumble_path}")

    with open(tumble_path, "r") as f:
        first_line = f.readline().strip()

    if first_line.startswith("#"):
        raw_headers = [h for h in first_line.lstrip("#").strip().split() if h]
        headers = [h.split("(")[0].strip() for h in raw_headers]
        df = pd.read_csv(tumble_path, sep=r"\s+", comment="#", names=headers)
    else:
        df = pd.read_csv(tumble_path, sep=r"\s+")

    col_map = {c: c.split("(")[0].replace("#", "").strip() for c in df.columns}
    df.rename(columns=col_map, inplace=True)

    times = df["Time"].values
    n = len(times)
    diffs = np.diff(times)
    rewinds = np.where(diffs <= 0)[0]

    drop_indices = set()
    if skip_points > 0 and n > 0:
        for k in range(min(skip_points, n)):
            drop_indices.add(df.index[k])
    if skip_time > 0 and n > 0:
        for k in np.where(times < skip_time)[0]:
            drop_indices.add(df.index[k])

    for r_idx in rewinds:
        if times[r_idx] == times[r_idx + 1]:
            drop_indices.add(df.index[r_idx])
        for k in range(r_idx + 1, min(r_idx + 1 + skip_points, n)):
            drop_indices.add(df.index[k])

    df_clean = df.drop(index=list(drop_indices)).reset_index(drop=True)
    return df_clean

def load_and_filter_falling(falling_path, skip_points=30, skip_time=0.002):
    """Load translational free-fall kinematics and lateral forces from fallingMotion/motion.dat."""
    if not os.path.exists(falling_path):
        raise FileNotFoundError(f"Falling motion file not found: {falling_path}")

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

    if df is None:
        with open(falling_path, "r") as f:
            lines = f.readlines()
        data_38 = []
        for line in lines:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) >= 38:
                data_38.append(parts[:38])
            elif len(parts) == 35:
                data_38.append(parts + ["0.0", "0.0", "0.0"])
        df = pd.DataFrame(np.array(data_38, dtype=np.float64), columns=FALLING_COLS_38)

    times = df["Time"].values
    n = len(times)
    diffs = np.diff(times)
    rewinds = np.where(diffs <= 0)[0]

    drop_indices = set()
    if skip_points > 0 and n > 0:
        for k in range(min(skip_points, n)):
            drop_indices.add(df.index[k])
    if skip_time > 0 and n > 0:
        for k in np.where(times < skip_time)[0]:
            drop_indices.add(df.index[k])

    for r_idx in rewinds:
        if times[r_idx] == times[r_idx + 1]:
            drop_indices.add(df.index[r_idx])
        for k in range(r_idx + 1, min(r_idx + 1 + skip_points, n)):
            drop_indices.add(df.index[k])

    df_clean = df.drop(index=list(drop_indices)).reset_index(drop=True)

    dt = np.diff(df_clean["Time"].values, prepend=df_clean["Time"].values[0])
    df_clean["Dx"] = np.cumsum(df_clean["Vx"].values * dt)
    df_clean["Dz"] = np.cumsum(df_clean["Vz"].values * dt)
    df_clean["Dhoriz"] = np.sqrt(df_clean["Dx"].values**2 + df_clean["Dz"].values**2)
    df_clean["Vhoriz"] = np.sqrt(df_clean["Vx"].values**2 + df_clean["Vz"].values**2)
    df_clean["Ahoriz"] = np.sqrt(df_clean["Ax"].values**2 + df_clean["Az"].values**2)
    df_clean["Faero_horiz"] = np.sqrt(df_clean["Faero_x"].values**2 + df_clean["Faero_z"].values**2)

    return df_clean

def build_synchronized_interpolators(df_tumble, df_falling):
    t_t = df_tumble["Time"].values
    t_f = df_falling["Time"].values

    diffs_t = np.diff(t_t, prepend=t_t[0] - 1.0)
    strict_idx_t = np.where(diffs_t > 1e-9)[0]
    t_t = t_t[strict_idx_t]

    diffs_f = np.diff(t_f, prepend=t_f[0] - 1.0)
    strict_idx_f = np.where(diffs_f > 1e-9)[0]
    t_f = t_f[strict_idx_f]

    t_start = max(t_t[0], t_f[0])
    t_end = min(t_t[-1], t_f[-1])

    qw = df_tumble["Quat_w"].values[strict_idx_t]
    qx = df_tumble["Quat_x"].values[strict_idx_t]
    qy = df_tumble["Quat_y"].values[strict_idx_t]
    qz = df_tumble["Quat_z"].values[strict_idx_t]

    quats_xyzw = np.column_stack([qx, qy, qz, qw])
    norms = np.linalg.norm(quats_xyzw, axis=1, keepdims=True)
    quats_xyzw /= np.maximum(norms, 1e-12)

    for i in range(1, len(quats_xyzw)):
        if np.dot(quats_xyzw[i], quats_xyzw[i - 1]) < 0.0:
            quats_xyzw[i] = -quats_xyzw[i]

    rotations = R.from_quat(quats_xyzw)
    slerp = Slerp(t_t, rotations)

    rpm_mag_t = np.sqrt(df_tumble["RPM_x"]**2 + df_tumble["RPM_y"]**2 + df_tumble["RPM_z"]**2).values[strict_idx_t]
    omega_x_t = df_tumble["Omega_x"].values[strict_idx_t] if "Omega_x" in df_tumble.columns else np.zeros_like(t_t)
    angle_x_t = df_tumble["Angle_x"].values[strict_idx_t] if "Angle_x" in df_tumble.columns else np.zeros_like(t_t)
    moment_x_t = df_tumble["Moment_x"].values[strict_idx_t] if "Moment_x" in df_tumble.columns else np.zeros_like(t_t)
    qw_clean = np.clip(np.abs(qw), -1.0, 1.0)
    tilt_3d_t = 2.0 * np.arccos(qw_clean) * (180.0 / np.pi)

    falling_interp_data = {
        "t": t_f,
        "dx": df_falling["Dx"].values[strict_idx_f],
        "dz": df_falling["Dz"].values[strict_idx_f],
        "dhoriz": df_falling["Dhoriz"].values[strict_idx_f],
        "vx": df_falling["Vx"].values[strict_idx_f],
        "vy": df_falling["Vy"].values[strict_idx_f],
        "vz": df_falling["Vz"].values[strict_idx_f],
        "vhoriz": df_falling["Vhoriz"].values[strict_idx_f],
        "ax": df_falling["Ax"].values[strict_idx_f],
        "ay": df_falling["Ay"].values[strict_idx_f],
        "az": df_falling["Az"].values[strict_idx_f],
        "ahoriz": df_falling["Ahoriz"].values[strict_idx_f],
        "faero_x": df_falling["Faero_x"].values[strict_idx_f],
        "faero_y": df_falling["Faero_y"].values[strict_idx_f],
        "faero_z": df_falling["Faero_z"].values[strict_idx_f],
        "faero_horiz": df_falling["Faero_horiz"].values[strict_idx_f],
        "altitude": df_falling["Altitude"].values[strict_idx_f],
    }

    tumble_interp_data = {
        "t": t_t,
        "slerp": slerp,
        "rpm": rpm_mag_t,
        "omega_x": omega_x_t,
        "angle_x": angle_x_t,
        "moment_x": moment_x_t,
        "tilt": tilt_3d_t
    }

    return t_start, t_end, tumble_interp_data, falling_interp_data

def evaluate_telemetry_frames(eval_times, tumble_data, falling_data):
    eval_rotations = tumble_data["slerp"](eval_times)
    eval_rot_matrices = eval_rotations.as_matrix()
    eval_rpm = np.interp(eval_times, tumble_data["t"], tumble_data["rpm"])
    eval_omega_x = np.interp(eval_times, tumble_data["t"], tumble_data["omega_x"])
    eval_angle_x = np.interp(eval_times, tumble_data["t"], tumble_data["angle_x"])
    eval_moment_x = np.interp(eval_times, tumble_data["t"], tumble_data["moment_x"])
    eval_tilt = np.interp(eval_times, tumble_data["t"], tumble_data["tilt"])

    tf = falling_data["t"]
    return {
        "times": eval_times,
        "rot_matrices": eval_rot_matrices,
        "rpm": eval_rpm,
        "omega_x": eval_omega_x,
        "angle_x": eval_angle_x,
        "moment_x": eval_moment_x,
        "tilt": eval_tilt,
        "dx": np.interp(eval_times, tf, falling_data["dx"]),
        "dz": np.interp(eval_times, tf, falling_data["dz"]),
        "dhoriz": np.interp(eval_times, tf, falling_data["dhoriz"]),
        "vx": np.interp(eval_times, tf, falling_data["vx"]),
        "vy": np.interp(eval_times, tf, falling_data["vy"]),
        "vz": np.interp(eval_times, tf, falling_data["vz"]),
        "vhoriz": np.interp(eval_times, tf, falling_data["vhoriz"]),
        "ax": np.interp(eval_times, tf, falling_data["ax"]),
        "ay": np.interp(eval_times, tf, falling_data["ay"]),
        "az": np.interp(eval_times, tf, falling_data["az"]),
        "ahoriz": np.interp(eval_times, tf, falling_data["ahoriz"]),
        "fx": np.interp(eval_times, tf, falling_data["faero_x"]),
        "fy": np.interp(eval_times, tf, falling_data["faero_y"]),
        "fz": np.interp(eval_times, tf, falling_data["faero_z"]),
        "fhoriz": np.interp(eval_times, tf, falling_data["faero_horiz"]),
        "altitude": np.interp(eval_times, tf, falling_data["altitude"]),
    }

# ==============================================================================
# Micro Floor Grid & Target Geometry
# ==============================================================================

def create_floor_grid_polydata(y_floor=-0.0035, x_span=0.0035, z_span=0.0035, step=0.0005):
    pts = []
    lines = []
    pt_id = 0
    for x in np.arange(-x_span, x_span + 1e-6, step):
        if abs(x) < 1e-6:
            continue
        pts.append([x, y_floor, -z_span])
        pts.append([x, y_floor, z_span])
        lines.extend([2, pt_id, pt_id + 1])
        pt_id += 2
    for z in np.arange(-z_span, z_span + 1e-6, step):
        if abs(z) < 1e-6:
            continue
        pts.append([-x_span, y_floor, z])
        pts.append([x_span, y_floor, z])
        lines.extend([2, pt_id, pt_id + 1])
        pt_id += 2
    return pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))

def create_floor_axes_polydata(y_floor=-0.0035, x_span=0.0035, z_span=0.0035):
    pts = [
        [-x_span, y_floor, 0.0], [x_span, y_floor, 0.0],
        [0.0, y_floor, -z_span], [0.0, y_floor, z_span]
    ]
    lines = [2, 0, 1, 2, 2, 3]
    return pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))

def create_floor_rings_polydata(y_floor=-0.0035, radii=(0.0005, 0.0010, 0.0015, 0.0020, 0.0025), n_pts=64):
    theta = np.linspace(0, 2.0 * np.pi, n_pts, endpoint=False)
    pts = []
    lines = []
    pt_id = 0
    for r in radii:
        start_id = pt_id
        for k in range(n_pts):
            pts.append([r * np.sin(theta[k]), y_floor, r * np.cos(theta[k])])
            pt_id += 1
        lines.append(n_pts + 1)
        lines.extend(list(range(start_id, pt_id)))
        lines.append(start_id)
    return pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))

def create_target_marker_polydata(center=(0.0, -0.0035, 0.0), radius=0.00025, n_pts=32):
    cx, cy, cz = center
    theta = np.linspace(0, 2.0 * np.pi, n_pts, endpoint=False)
    pts = []
    lines = []
    for k in range(n_pts):
        pts.append([cx + radius * np.sin(theta[k]), cy, cz + radius * np.cos(theta[k])])
    lines.append(n_pts + 1)
    lines.extend(list(range(n_pts)))
    lines.append(0)

    p_start = n_pts
    pts.append([cx - radius * 1.8, cy, cz])
    pts.append([cx + radius * 1.8, cy, cz])
    lines.extend([2, p_start, p_start + 1])

    pts.append([cx, cy, cz - radius * 1.8])
    pts.append([cx, cy, cz + radius * 1.8])
    lines.extend([2, p_start + 2, p_start + 3])
    return pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))

# ==============================================================================
# 2D HUD Compositor (Pillow)
# ==============================================================================

def get_hud_font(size=12, bold=False):
    font_paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf" if bold else "/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    ]
    for p in font_paths:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()

def render_radar_hud_overlay(img_rgba, frame_idx, total_frames, fps, telem, max_drift_mm=2.5):
    draw = ImageDraw.Draw(img_rgba, "RGBA")

    font_banner_title = get_hud_font(13, bold=True)
    font_banner_body = get_hud_font(11, bold=False)
    font_title = get_hud_font(11, bold=True)
    font_card = get_hud_font(11, bold=False)
    font_card_b = get_hud_font(11, bold=True)
    font_tiny = get_hud_font(9, bold=False)

    sim_t = telem["times"][frame_idx]
    rpm_val = telem["rpm"][frame_idx]
    omega_x = telem["omega_x"][frame_idx]
    angle_x = telem["angle_x"][frame_idx]
    alt_m = telem["altitude"][frame_idx]
    vy_m = telem["vy"][frame_idx]
    v_descent = abs(vy_m)
    v_horiz = telem["vhoriz"][frame_idx]
    ahoriz = telem["ahoriz"][frame_idx]
    f_horiz = telem["fhoriz"][frame_idx]
    f_drag_uN = telem["fy"][frame_idx] * 1e6  # microNewtons
    moment_nNm = telem["moment_x"][frame_idx] * 1e9  # nanoN.m

    curr_dx = telem["dx"][frame_idx]
    curr_dz = telem["dz"][frame_idx]
    dx_mm = curr_dx * 1000.0
    dz_mm = curr_dz * 1000.0
    dhoriz_mm = telem["dhoriz"][frame_idx] * 1000.0

    bearing_deg = (math.atan2(curr_dx, curr_dz) * 180.0 / math.pi) % 360.0

    # 1. TOP-LEFT MISSION BANNER
    bx1, by1, bx2, by2 = 18, 16, 550, 155
    draw.rounded_rectangle([bx1, by1, bx2, by2], radius=10, fill=(10, 16, 26, 230), outline=(48, 54, 61, 255), width=1)
    draw.text((bx1 + 14, by1 + 10), "THIN DISK 6DoF MOTION & DRIFT VISUALIZER", fill=(0, 242, 254, 255), font=font_banner_title)
    draw.text((bx1 + 14, by1 + 30), "Benchmark: Tinklenberg, Guala, Coletti (2023) JFM | D = 2.02 mm, h = 50 µm", fill=(139, 148, 158, 220), font=font_tiny)

    line1 = f"TIME: {sim_t*1e3:6.1f} ms  |  STEP: {frame_idx+1:04d}/{total_frames:04d}  |  FRAME: {frame_idx+1:04d}"
    line2 = f"PITCH: {angle_x:+6.1f}°  |  PITCH RATE: {omega_x:+6.1f} rad/s ({abs(rpm_val):5.0f} RPM)"
    line3 = f"DESCENT: {v_descent:5.3f} m/s  |  FALLEN: {abs(alt_m)*100:5.2f} cm  |  DRAG: {f_drag_uN:+5.2f} µN"
    line4 = f"Z-DRIFT: {dz_mm:+6.2f} mm  |  LATERAL SPEED: {v_horiz*1000:5.1f} mm/s  |  BEARING: {bearing_deg:5.1f}°"
    draw.text((bx1 + 14, by1 + 50), line1, fill=(201, 209, 217, 255), font=font_banner_body)
    draw.text((bx1 + 14, by1 + 71), line2, fill=(57, 255, 20, 255), font=font_banner_body)
    draw.text((bx1 + 14, by1 + 92), line3, fill=(255, 215, 0, 255), font=font_banner_body)
    draw.text((bx1 + 14, by1 + 113), line4, fill=(0, 242, 254, 255), font=font_banner_body)

    # 2. TOP-RIGHT RADAR COMPASS HUD (X vs Z in mm)
    cx, cy = 1110, 150
    r_radar = 85
    px_per_m = r_radar / (max_drift_mm * 1e-3)

    bg_box = [cx - r_radar - 14, cy - r_radar - 42, cx + r_radar + 14, cy + r_radar + 155]
    draw.rounded_rectangle(bg_box, radius=12, fill=(10, 16, 26, 230), outline=(48, 54, 61, 255), width=1)
    draw.text((cx - r_radar, cy - r_radar - 28), "6DoF HORIZ DRIFT RADAR", fill=(88, 166, 255, 255), font=font_title)

    draw.ellipse([cx - r_radar, cy - r_radar, cx + r_radar, cy + r_radar], fill=(6, 11, 20, 245), outline=(56, 139, 253, 230), width=2)

    radii_mm = [0.5, 1.0, 1.5, 2.0]
    for r_mm in radii_mm:
        r_px = (r_mm * 1e-3) * px_per_m
        if r_px <= r_radar:
            color = (33, 53, 85, 180) if r_mm != 2.0 else (56, 139, 253, 140)
            draw.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px], outline=color, width=1)
            draw.text((cx + 3, cy - r_px - 8), f"{r_mm:.1f}", fill=(100, 130, 170, 160), font=font_tiny)

    draw.line([cx - r_radar, cy, cx + r_radar, cy], fill=(30, 45, 70, 200), width=1)
    draw.line([cx, cy - r_radar, cx, cy + r_radar], fill=(30, 45, 70, 200), width=1)

    draw.text((cx - 10, cy - r_radar - 14), "+Z", fill=(88, 166, 255, 230), font=font_tiny)
    draw.text((cx + r_radar - 14, cy - 14), "+X", fill=(88, 166, 255, 230), font=font_tiny)
    draw.text((cx - 10, cy + r_radar + 3), "-Z", fill=(100, 120, 150, 180), font=font_tiny)
    draw.text((cx - r_radar - 12, cy - 14), "-X", fill=(100, 120, 150, 180), font=font_tiny)

    # Origin marker
    draw.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(88, 166, 255, 180), outline=(255, 255, 255, 220), width=1)

    # Historical Ground Track on Radar
    all_dx = telem["dx"][:frame_idx + 1]
    all_dz = telem["dz"][:frame_idx + 1]
    if len(all_dx) > 1:
        track_pixels = []
        for i in range(len(all_dx)):
            px = cx + all_dx[i] * px_per_m
            py = cy - all_dz[i] * px_per_m
            track_pixels.append((px, py))
        draw.line(track_pixels, fill=(0, 242, 254, 210), width=2)

    # Current Position Blip
    curr_px = cx + curr_dx * px_per_m
    curr_py = cy - curr_dz * px_per_m
    halo_r = 6 + int(2 * math.sin(sim_t * 30.0))
    draw.ellipse([curr_px - halo_r, curr_py - halo_r, curr_px + halo_r, curr_py + halo_r], outline=(57, 255, 20, 120), width=1)
    draw.ellipse([curr_px - 3, curr_py - 3, curr_px + 3, curr_py + 3], fill=(57, 255, 20, 255))

    # Dynamic Velocity Vector Arrow
    vx = telem["vx"][frame_idx]
    vz = telem["vz"][frame_idx]
    if v_horiz > 1e-5:
        v_arrow_len = min(35.0, v_horiz * 800.0)
        v_dir_x = (vx / v_horiz) * v_arrow_len
        v_dir_y = -(vz / v_horiz) * v_arrow_len
        draw.line([curr_px, curr_py, curr_px + v_dir_x, curr_py + v_dir_y], fill=(57, 255, 20, 240), width=2)

    # Telemetry Card
    card_top = cy + r_radar + 12
    lines = [
        ("H-DRIFT", f"{dhoriz_mm:5.2f} mm", (0, 242, 254, 255)),
        ("DX / DZ", f"{dx_mm:+4.2f}/{dz_mm:+4.2f} mm", (140, 180, 230, 240)),
        ("BEARING", f"{bearing_deg:5.1f}°", (200, 215, 235, 240)),
        ("H-SPEED", f"{v_horiz*1000:5.2f} mm/s", (57, 255, 20, 255)),
        ("PITCH Mx", f"{moment_nNm:+5.2f} nN·m", (255, 110, 50, 255)),
        ("DESCENT", f"{abs(alt_m)*100:4.1f}cm @ {v_descent:4.2f}m/s", (180, 195, 210, 220)),
    ]
    curr_y = card_top
    for label, val, val_col in lines:
        draw.text((cx - r_radar, curr_y), label, fill=(139, 148, 158, 220), font=font_card)
        draw.text((cx - 8, curr_y), val, fill=val_col, font=font_card_b)
        curr_y += 18

    return img_rgba

# ==============================================================================
# PyVista Worker Rendering Function
# ==============================================================================

def _render_drift_chunk_worker(worker_id, start_idx, end_idx, total_frames,
                               stl_path, telem,
                               fps, width, height, out_chunk_path,
                               shared_counter, counter_lock,
                               thickness_scale=2.5):
    try:
        mesh = pv.read(stl_path)
        # Apply visual thickness boost along Y axis (normal) to prevent sub-pixel thinning edge-on
        pts = mesh.points.copy()
        pts[:, 1] *= thickness_scale
        mesh.points = pts

        # Compute dual-tone cell colors (Cyan top face, Orange bottom face, White rim)
        polys = mesh.compute_normals(cell_normals=True, point_normals=False)
        normals = polys.cell_data['Normals']
        colors = np.zeros((mesh.n_cells, 3), dtype=np.uint8)
        for i in range(mesh.n_cells):
            ny = normals[i, 1]
            if ny > 0.3:
                colors[i] = [0, 229, 255]     # Vibrant cyan (top face)
            elif ny < -0.3:
                colors[i] = [255, 109, 0]     # Vibrant orange (bottom face)
            else:
                colors[i] = [240, 240, 240]   # Crisp white rim
        mesh.cell_data['RGB'] = colors

        edges = mesh.extract_feature_edges(feature_angle=25, boundary_edges=True, feature_edges=True, manifold_edges=False)
        base_points_mesh = mesh.points.copy()
        base_points_edges = edges.points.copy()

        y_floor = -0.0018  # 1.8 mm below center

        pv.OFF_SCREEN = True
        plotter = pv.Plotter(off_screen=True, window_size=[width, height])
        plotter.set_background("#0d1117")

        plotter.add_axes(interactive=False, line_width=2.0)

        # 1. Floor Reference Grid, Rings & Axes (calibrated to millimeter scale)
        grid_poly = create_floor_grid_polydata(y_floor=y_floor, x_span=0.0030, z_span=0.0040, step=0.0005)
        rings_poly = create_floor_rings_polydata(y_floor=y_floor, radii=(0.0005, 0.0010, 0.0015, 0.0020, 0.0025, 0.0030))
        axes_poly = create_floor_axes_polydata(y_floor=y_floor, x_span=0.0030, z_span=0.0040)
        plotter.add_mesh(grid_poly, color="#1f2937", line_width=1.0)
        plotter.add_mesh(rings_poly, color="#374151", line_width=1.5)
        plotter.add_mesh(axes_poly, color="#4b5563", line_width=1.8)

        # Distance ticks along Z axis at every 0.5 mm
        z_min_telem = float(np.min(telem["dz"]))
        z_max_telem = float(np.max(telem["dz"]))
        z_ticks_pts = []
        z_ticks_lines = []
        t_k = 0
        tick_min = math.floor(min(z_min_telem, -0.001) / 0.0005) * 0.0005
        tick_max = math.ceil(max(z_max_telem, 0.001) / 0.0005) * 0.0005 + 0.0005
        for z_val in np.arange(tick_min, tick_max, 0.0005):
            z_ticks_pts.extend([[-0.0002, y_floor, z_val], [0.0002, y_floor, z_val]])
            z_ticks_lines.extend([2, t_k, t_k + 1])
            t_k += 2
        z_ticks_poly = pv.PolyData(np.array(z_ticks_pts, dtype=np.float32), lines=np.array(z_ticks_lines, dtype=np.int32))
        plotter.add_mesh(z_ticks_poly, color="#4b5563", line_width=2.0)

        # Origin release target marker
        origin_marker = create_target_marker_polydata(center=(0.0, y_floor, 0.0), radius=0.00020)
        plotter.add_mesh(origin_marker, color="#388bfd", line_width=2.5)

        # 2. Ground Track Ribbon on Floor
        track_pts = np.zeros((total_frames, 3), dtype=np.float32)
        track_lines = []
        for i in range(max(0, total_frames - 1)):
            track_lines.extend([2, i, i + 1])
        track_poly = pv.PolyData(track_pts, lines=np.array(track_lines, dtype=np.int32))
        track_actor = plotter.add_mesh(track_poly, color="#00f2fe", line_width=4.0)

        # Vertical Trajectory Curtain (Translucent ribbon connecting 3D flight path to floor)
        curtain_pts = np.zeros((total_frames * 2, 3), dtype=np.float32)
        curtain_faces = []
        for i in range(max(0, total_frames - 1)):
            curtain_faces.extend([4, 2*i, 2*i+2, 2*i+3, 2*i+1])
        curtain_poly = pv.PolyData(curtain_pts, faces=np.array(curtain_faces, dtype=np.int32))
        curtain_actor = plotter.add_mesh(curtain_poly, color="#00f2fe", opacity=0.18)

        # 3. Laser Plumb-Bob Dropline
        plumb_pts = np.array([[0.0, 0.0, 0.0], [0.0, y_floor, 0.0]], dtype=np.float32)
        plumb_poly = pv.PolyData(plumb_pts, lines=np.array([2, 0, 1], dtype=np.int32))
        plumb_actor = plotter.add_mesh(plumb_poly, color="#39ff14", line_width=3.5)

        # Ground target blip
        ground_blip_poly = create_target_marker_polydata(center=(0.0, y_floor, 0.0), radius=0.00020)
        blip_actor = plotter.add_mesh(ground_blip_poly, color="#39ff14", line_width=2.5)
        blip_base_pts = ground_blip_poly.points.copy()

        # 4. 3D Hub Velocity Vector (Green) & Lateral Force Vector (Coral)
        vel_line_pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        vel_poly = pv.PolyData(vel_line_pts, lines=np.array([2, 0, 1], dtype=np.int32))
        vel_actor = plotter.add_mesh(vel_poly, color="#39ff14", line_width=4.5)

        force_line_pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        force_poly = pv.PolyData(force_line_pts, lines=np.array([2, 0, 1], dtype=np.int32))
        force_actor = plotter.add_mesh(force_poly, color="#ff5500", line_width=4.5)

        # 5. Upward Airflow Streaks
        n_streaks = 50
        np.random.seed(42)
        r_streaks = np.random.uniform(0.0012, 0.0035, n_streaks)
        theta_streaks = np.random.uniform(0, 2 * np.pi, n_streaks)
        wx = (r_streaks * np.cos(theta_streaks)).astype(np.float32)
        wz = (r_streaks * np.sin(theta_streaks)).astype(np.float32)
        wy0 = np.random.uniform(-0.0025, 0.0025, n_streaks).astype(np.float32)
        base_lens = np.random.uniform(0.0006, 0.0012, n_streaks).astype(np.float32)
        y_min_span, y_max_span = -0.0025, 0.0025
        y_span = y_max_span - y_min_span

        wind_pts = np.zeros((n_streaks * 2, 3), dtype=np.float32)
        wind_lines = []
        for i in range(n_streaks):
            wind_pts[2 * i] = [wx[i], wy0[i], wz[i]]
            wind_pts[2 * i + 1] = [wx[i], wy0[i] + base_lens[i], wz[i]]
            wind_lines.extend([2, 2 * i, 2 * i + 1])
        wind_poly = pv.PolyData(wind_pts, lines=np.array(wind_lines, dtype=np.int32))
        wind_actor = plotter.add_mesh(wind_poly, color="#58a6ff", line_width=1.5, opacity=0.40)

        # 6. Disk Solid Mesh (Dual-Tone RGB) & Highlight Edges
        mesh_actor = plotter.add_mesh(
            mesh.copy(),
            scalars='RGB',
            rgb=True,
            opacity=0.98,
            smooth_shading=True,
            specular=0.5,
            ambient=0.35
        )
        edges_actor = plotter.add_mesh(
            edges.copy(),
            color="#ffffff",
            line_width=2.2,
            render_lines_as_tubes=False
        )

        max_d_mm = max(2.5, float(np.max(telem["dhoriz"]) * 1000.0))

        os.makedirs(os.path.dirname(os.path.abspath(out_chunk_path)), exist_ok=True)
        writer = imageio.get_writer(out_chunk_path, fps=fps, quality=8, codec="libx264")

        # Pre-populate track and curtain points up to start_idx
        for k in range(start_idx + 1):
            track_pts[k] = [telem["dx"][k], y_floor + 0.00002, telem["dz"][k]]
            curtain_pts[2*k] = [telem["dx"][k], 0.0, telem["dz"][k]]
            curtain_pts[2*k + 1] = [telem["dx"][k], y_floor, telem["dz"][k]]
        if total_frames > 1:
            track_actor.mapper.dataset.points = track_pts
            track_actor.mapper.dataset.Modified()
            curtain_actor.mapper.dataset.points = curtain_pts
            curtain_actor.mapper.dataset.Modified()

        # Fixed flight-corridor camera perspective (scales down disk to ~0.6 and makes full Z glide visible)
        z_center = 0.5 * (z_min_telem + z_max_telem)
        focal_pt = [0.0000, -0.0006, z_center]
        cam_pos = [0.0055, 0.0032, z_center + 0.0028]
        plotter.camera_position = [cam_pos, focal_pt, [0.0, 1.0, 0.0]]

        for idx in range(start_idx, end_idx):
            rot_mat = telem["rot_matrices"][idx]
            dx = telem["dx"][idx]
            dz = telem["dz"][idx]
            vx = telem["vx"][idx]
            vy = telem["vy"][idx]
            vz = telem["vz"][idx]
            fx = telem["fx"][idx]
            fz = telem["fz"][idx]

            hub_pos = np.array([dx, 0.0, dz], dtype=np.float32)

            # Rotate and translate disk mesh
            transformed_pts = (base_points_mesh @ rot_mat.T) + hub_pos
            mesh_actor.mapper.dataset.points = transformed_pts
            mesh_actor.mapper.dataset.Modified()

            transformed_edges = (base_points_edges @ rot_mat.T) + hub_pos
            edges_actor.mapper.dataset.points = transformed_edges
            edges_actor.mapper.dataset.Modified()

            # Ground track & trajectory curtain update
            if total_frames > 1:
                track_pts[idx] = [dx, y_floor + 0.00002, dz]
                track_actor.mapper.dataset.points = track_pts
                track_actor.mapper.dataset.Modified()

                curtain_pts[2*idx] = [dx, 0.0, dz]
                curtain_pts[2*idx + 1] = [dx, y_floor, dz]
                curtain_actor.mapper.dataset.points = curtain_pts
                curtain_actor.mapper.dataset.Modified()

            # Plumb-bob line
            plumb_actor.mapper.dataset.points = np.array([hub_pos, [dx, y_floor, dz]], dtype=np.float32)
            plumb_actor.mapper.dataset.Modified()

            # Ground target blip
            blip_actor.mapper.dataset.points = blip_base_pts + np.array([dx, 0.0, dz], dtype=np.float32)
            blip_actor.mapper.dataset.Modified()

            # Velocity vector arrow
            v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
            v_scale = 0.0025 / max(v_mag, 1.0)
            vel_actor.mapper.dataset.points = np.array([
                hub_pos,
                hub_pos + np.array([vx, vy, vz], dtype=np.float32) * v_scale
            ], dtype=np.float32)
            vel_actor.mapper.dataset.Modified()

            # Lateral force vector arrow
            f_mag = math.sqrt(fx**2 + fz**2)
            f_scale = 0.0020 / max(f_mag, 1e-6)
            force_actor.mapper.dataset.points = np.array([
                hub_pos,
                hub_pos + np.array([fx, 0.0, fz], dtype=np.float32) * f_scale
            ], dtype=np.float32)
            force_actor.mapper.dataset.Modified()

            # Airflow streaks flowing upwards relative to falling disk
            flow_speed = max(0.5, abs(vy))
            t_curr = telem["times"][idx]
            flow_shift = (t_curr * flow_speed * 0.1) % y_span
            y_curr = wy0 + flow_shift
            y_curr = np.where(y_curr > y_max_span, y_curr - y_span, y_curr)
            w_pts_updated = wind_pts.copy()
            for i in range(n_streaks):
                w_pts_updated[2 * i, 1] = y_curr[i]
                w_pts_updated[2 * i + 1, 1] = y_curr[i] + base_lens[i]
            wind_actor.mapper.dataset.points = w_pts_updated
            wind_actor.mapper.dataset.Modified()

            # Force OpenGL to render updated mesh and camera tracking
            plotter.render()
            img_np = plotter.screenshot(return_img=True)
            img_pil = Image.fromarray(img_np)

            # Render 2D Avionics HUD
            img_with_hud = render_radar_hud_overlay(img_pil, idx, total_frames, fps, telem, max_drift_mm=max_d_mm)
            writer.append_data(np.asarray(img_with_hud))

            with counter_lock:
                shared_counter.value += 1

        writer.close()
        plotter.close()
        return True

    except Exception as e:
        import traceback
        traceback.print_exc()
        return False

# ==============================================================================
# Main Orchestrator
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="Animate 6DoF Thin Disk Motion & Drift with Floor Grid & Radar HUD")
    parser.add_argument("--case", type=str, default=".", help="Case or validation directory (default: current directory)")
    parser.add_argument("--slowdown", type=float, default=20.0, help="Slow-motion replay factor (default: 20x)")
    parser.add_argument("--fps", type=int, default=60, help="Video frame rate in FPS (default: 60)")
    parser.add_argument("--stride", type=int, default=1, help="Frame subsampling stride (default: 1)")
    parser.add_argument("--max-frames", type=int, default=None, help="Maximum frames to render")
    parser.add_argument("-j", "--workers", type=int, default=None, help="Parallel worker threads (default: CPU cores / 2)")
    parser.add_argument("-o", "--output", type=str, default="render/disk_drift_animation.mp4", help="Output MP4 filename")
    parser.add_argument("--preview", action="store_true", help="Generate a single-frame preview screenshot and exit")
    parser.add_argument("--skip-time", type=float, default=0.002, help="Ignore startup transient before this time (s)")
    parser.add_argument("--skip-points", type=int, default=30, help="Ignore startup transient points count")
    parser.add_argument("--thickness-scale", type=float, default=2.0, help="Visual thickness scale factor for micro-disk (default: 2.0x)")
    args = parser.parse_args()

    case_dir = os.path.abspath(args.case)
    script_dir = os.path.dirname(os.path.abspath(__file__))

    stl_path = locate_disk_stl(case_dir) or locate_disk_stl(script_dir)
    if not stl_path:
        print("[Error] Could not find disk.stl!", file=sys.stderr)
        sys.exit(1)

    tumble_p, falling_p = locate_telemetry_files(case_dir)
    if not tumble_p or not falling_p:
        tumble_p, falling_p = locate_telemetry_files(script_dir)

    if not tumble_p or not falling_p:
        print("[Error] Telemetry files (motion.dat) not found in case directory!", file=sys.stderr)
        sys.exit(1)

    print("=" * 80)
    print("      THIN DISK 6DoF MOTION & DRIFT ANIMATOR (PyVista + Avionics HUD)")
    print("=" * 80)
    print(f"  • Disk STL  : {stl_path}")
    print(f"  • Tumble Log: {tumble_p}")
    print(f"  • Fall Log  : {falling_p}")

    df_tumble = load_and_filter_tumble(tumble_p, skip_points=args.skip_points, skip_time=args.skip_time)
    df_falling = load_and_filter_falling(falling_p, skip_points=args.skip_points, skip_time=args.skip_time)

    t_start, t_end, tumble_data, falling_data = build_synchronized_interpolators(df_tumble, df_falling)
    sim_duration = t_end - t_start

    print(f"  • Valid Sim Window: {t_start*1e3:.1f} ms -> {t_end*1e3:.1f} ms (duration {sim_duration*1e3:.1f} ms)")

    # Output path resolution
    out_path = args.output
    if not os.path.isabs(out_path):
        out_path = os.path.join(script_dir, out_path)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)

    # 1. Preview Mode
    if args.preview:
        preview_png = os.path.splitext(out_path)[0] + "_preview.png"
        print(f"\n[Preview] Generating single-frame preview screenshot: {preview_png} ...")
        n_preview = 120
        eval_times = np.linspace(t_start, t_end, n_preview)
        telem = evaluate_telemetry_frames(eval_times, tumble_data, falling_data)
        
        tmp_mp4 = tempfile.mktemp(suffix=".mp4")
        counter = mp.Value("i", 0)
        lock = mp.Lock()
        success = _render_drift_chunk_worker(
            0, n_preview - 1, n_preview, n_preview,
            stl_path, telem, 30, 1280, 720,
            tmp_mp4, counter, lock,
            thickness_scale=args.thickness_scale
        )
        
        if success and os.path.exists(tmp_mp4):
            reader = imageio.get_reader(tmp_mp4)
            frame = reader.get_data(0)
            reader.close()
            Image.fromarray(frame).save(preview_png)
            try:
                os.remove(tmp_mp4)
            except Exception:
                pass
            print(f"[Success] Saved preview screenshot to: {preview_png}")
            return

    # 2. Full Video Rendering
    video_duration = sim_duration * args.slowdown
    total_frames = int(round(video_duration * args.fps))
    if args.max_frames and args.max_frames > 0:
        total_frames = min(total_frames, args.max_frames)

    eval_times = np.linspace(t_start, t_end, total_frames)
    telem = evaluate_telemetry_frames(eval_times, tumble_data, falling_data)

    print(f"  • Video Specs: {total_frames} frames @ {args.fps} FPS ({args.slowdown}x slow-mo, duration {total_frames/args.fps:.2f}s)")
    print(f"  • Target MP4 : {out_path}")

    num_workers = args.workers if args.workers else max(1, mp.cpu_count() // 2)
    print(f"  • Parallel Rendering on {num_workers} CPU workers...")

    chunk_size = math.ceil(total_frames / num_workers)
    temp_dir = tempfile.mkdtemp(prefix="disk_drift_render_")
    chunk_files = []
    processes = []

    shared_counter = mp.Value("i", 0)
    counter_lock = mp.Lock()

    for w_id in range(num_workers):
        s_idx = w_id * chunk_size
        e_idx = min(total_frames, s_idx + chunk_size)
        if s_idx >= total_frames:
            break
        c_path = os.path.join(temp_dir, f"chunk_{w_id:03d}.mp4")
        chunk_files.append(c_path)
        p = mp.Process(target=_render_drift_chunk_worker, args=(
            w_id, s_idx, e_idx, total_frames,
            stl_path, telem,
            args.fps, 1280, 720, c_path,
            shared_counter, counter_lock,
            args.thickness_scale
        ))
        processes.append(p)
        p.start()

    # Progress monitor
    start_wall = time.time()
    while any(p.is_alive() for p in processes):
        with counter_lock:
            done = shared_counter.value
        elapsed = time.time() - start_wall
        fps_render = done / max(elapsed, 0.001)
        eta = (total_frames - done) / max(fps_render, 0.001) if fps_render > 0 else 0.0
        pct = (done / total_frames) * 100.0
        sys.stdout.write(f"\r  [Rendering] {done}/{total_frames} frames ({pct:5.1f}%) | Speed: {fps_render:5.1f} fps | ETA: {eta:4.0f}s  ")
        sys.stdout.flush()
        time.sleep(0.5)

    for p in processes:
        p.join()
    print("\n  [Rendering Complete] Concatenating parallel video streams...")

    # Concatenate chunks via FFmpeg
    concat_list = os.path.join(temp_dir, "concat.txt")
    with open(concat_list, "w") as f:
        for c in chunk_files:
            f.write(f"file '{c}'\n")

    cmd = [
        "ffmpeg", "-y", "-f", "concat", "-safe", "0",
        "-i", concat_list,
        "-c", "copy",
        out_path
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)

    try:
        shutil.rmtree(temp_dir)
    except Exception:
        pass

    total_time = time.time() - start_wall
    print(f"\n[Success] Generated 6DoF Thin Disk Motion & Drift Video!")
    print(f"  • Video Output: {out_path}")
    print(f"  • Render Time : {total_time:.1f} s")
    print("=" * 80)

if __name__ == "__main__":
    main()
