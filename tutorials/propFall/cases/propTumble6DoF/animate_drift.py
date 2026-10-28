#!/usr/bin/env python3
"""
animate_drift.py — 6DoF Propeller Motion & Drift Visualizer with Floor Grid & Corner Radar HUD

Combines 3D PyVista simulation rendering with:
  1. Floor Reference Grid & Laser Plumb-Bob (Ground Track):
     - Reference coordinate grid & concentric range rings at Y = -0.16m (below propeller).
     - 3D ground track ribbon showing cumulative horizontal translation (Dx, Dz).
     - Glowing vertical laser dropline connecting propeller hub to instantaneous ground track.
     - 3D instantaneous velocity and lateral aerodynamic force vectors at the hub.
  2. Corner Radar Compass HUD & Avionics Telemetry:
     - Top-right circular radar scope showing top-down horizontal drift (X vs Z).
     - Concentric range rings (10mm, 20mm, 30mm, 40mm) and cardinal crosshairs (+Z Span, +X Normal).
     - Real-time ground track trail and pulsing green position blip.
     - Dynamic velocity vector and lateral aerodynamic force vector arrows.
     - Integrated telemetry card: Horiz Drift, Bearing, Horiz Speed, Horiz Accel, Lat Force, Descent Alt/Speed.

Hardware Acceleration & Multi-Processing:
  - OpenGL GPU off-screen rendering via PyVista.
  - Multi-core parallel chunk rendering (-j / --workers N).
  - Lossless FFmpeg stream concatenation.

Usage Examples:
  # Default settings (20x slow-mo, 60 FPS, output to slowdown20-Radar/Grid.mp4):
  python3 animate_drift.py

  # Explicit CLI options as requested:
  python3 animate_drift.py --slowdown 20 --fps 60 --o slowdown20-Radar/Grid.mp4

  # Quick single-frame preview screenshot:
  python3 animate_drift.py --preview

  # Fast preview test (first 300 frames on 6 workers):
  python3 animate_drift.py --slowdown 20 --fps 60 --max-frames 300 -j 6
"""

import os
import sys
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

# Suppress harmless Matplotlib / 3D warnings
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


def locate_propeller_stl(case_dir):
    """Search common case and meshing directories for the propeller STL geometry."""
    candidates = [
        os.path.join(case_dir, "../../meshing/rotatingMesh/constant/triSurface/patch_2_1.stl"),
        os.path.join(case_dir, "../../meshing/rotatingMesh/constant/triSurface/MeshRefinement.stl"),
        os.path.join(case_dir, "constant/triSurface/propeller.stl"),
        os.path.join(case_dir, "constant/triSurface/patch_2_1.stl"),
    ]
    for c in candidates:
        abs_p = os.path.abspath(c)
        if os.path.exists(abs_p):
            return abs_p
    return None


def locate_telemetry_files(case_dir):
    """Locate tumbleMotion/motion.dat and fallingMotion/motion.dat."""
    post_dir = os.path.join(case_dir, "postProcessing")
    tumble_p = os.path.join(post_dir, "tumbleMotion", "motion.dat")
    falling_p = os.path.join(post_dir, "fallingMotion", "motion.dat")

    if not os.path.exists(tumble_p):
        tumble_p = os.path.join(case_dir, "processor0/postProcessing/tumbleMotion/motion.dat")
    if not os.path.exists(falling_p):
        falling_p = os.path.join(case_dir, "processor0/postProcessing/fallingMotion/motion.dat")

    return tumble_p if os.path.exists(tumble_p) else None, falling_p if os.path.exists(falling_p) else None


def load_and_filter_tumble(tumble_path, skip_points=5):
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

    for r_idx in rewinds:
        if times[r_idx] == times[r_idx + 1]:
            drop_indices.add(df.index[r_idx])
        for k in range(r_idx + 1, min(r_idx + 1 + skip_points, n)):
            drop_indices.add(df.index[k])

    df_clean = df.drop(index=list(drop_indices)).reset_index(drop=True)
    return df_clean


def load_and_filter_falling(falling_path, skip_points=5):
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
            raise ValueError(f"Could not parse valid data from: {falling_path}")

        df = pd.DataFrame(all_arr, columns=FALLING_COLS_38)

    # Clean duplicates / rewinds
    times = df["Time"].values
    n = len(times)
    diffs = np.diff(times)
    rewinds = np.where(diffs <= 0)[0]

    drop_indices = set()
    if skip_points > 0 and n > 0:
        for k in range(min(skip_points, n)):
            drop_indices.add(df.index[k])

    for r_idx in rewinds:
        if times[r_idx] == times[r_idx + 1]:
            drop_indices.add(df.index[r_idx])
        for k in range(r_idx + 1, min(r_idx + 1 + skip_points, n)):
            drop_indices.add(df.index[k])

    df_clean = df.drop(index=list(drop_indices)).reset_index(drop=True)

    # Compute cumulative horizontal displacements (Dx, Dz)
    dt = np.diff(df_clean["Time"].values, prepend=df_clean["Time"].values[0])
    df_clean["Dx"] = np.cumsum(df_clean["Vx"].values * dt)
    df_clean["Dz"] = np.cumsum(df_clean["Vz"].values * dt)
    df_clean["Dhoriz"] = np.sqrt(df_clean["Dx"].values**2 + df_clean["Dz"].values**2)
    df_clean["Vhoriz"] = np.sqrt(df_clean["Vx"].values**2 + df_clean["Vz"].values**2)
    df_clean["Ahoriz"] = np.sqrt(df_clean["Ax"].values**2 + df_clean["Az"].values**2)
    df_clean["Faero_horiz"] = np.sqrt(df_clean["Faero_x"].values**2 + df_clean["Faero_z"].values**2)

    return df_clean


def build_synchronized_interpolators(df_tumble, df_falling):
    """
    Construct continuous Slerp attitude and translational spline interpolators
    over the shared simulation time window.
    """
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

    # Slerp for attitude
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
    qw_clean = np.clip(np.abs(qw), -1.0, 1.0)
    tilt_3d_t = 2.0 * np.arccos(qw_clean) * (180.0 / np.pi)

    # Store falling telemetry
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
        "faero_z": df_falling["Faero_z"].values[strict_idx_f],
        "faero_horiz": df_falling["Faero_horiz"].values[strict_idx_f],
        "altitude": df_falling["Altitude"].values[strict_idx_f],
    }

    tumble_interp_data = {
        "t": t_t,
        "slerp": slerp,
        "rpm": rpm_mag_t,
        "tilt": tilt_3d_t
    }

    return t_start, t_end, tumble_interp_data, falling_interp_data


def evaluate_telemetry_frames(eval_times, tumble_data, falling_data):
    """Interpolate all 6DoF kinematic and dynamic telemetry for the given evaluation times."""
    # Attitude
    eval_rotations = tumble_data["slerp"](eval_times)
    eval_rot_matrices = eval_rotations.as_matrix()
    eval_rpm = np.interp(eval_times, tumble_data["t"], tumble_data["rpm"])
    eval_tilt = np.interp(eval_times, tumble_data["t"], tumble_data["tilt"])

    # Translations
    tf = falling_data["t"]
    eval_dx = np.interp(eval_times, tf, falling_data["dx"])
    eval_dz = np.interp(eval_times, tf, falling_data["dz"])
    eval_dhoriz = np.interp(eval_times, tf, falling_data["dhoriz"])
    eval_vx = np.interp(eval_times, tf, falling_data["vx"])
    eval_vy = np.interp(eval_times, tf, falling_data["vy"])
    eval_vz = np.interp(eval_times, tf, falling_data["vz"])
    eval_vhoriz = np.interp(eval_times, tf, falling_data["vhoriz"])
    eval_ax = np.interp(eval_times, tf, falling_data["ax"])
    eval_ay = np.interp(eval_times, tf, falling_data["ay"])
    eval_az = np.interp(eval_times, tf, falling_data["az"])
    eval_ahoriz = np.interp(eval_times, tf, falling_data["ahoriz"])
    eval_fx = np.interp(eval_times, tf, falling_data["faero_x"])
    eval_fz = np.interp(eval_times, tf, falling_data["faero_z"])
    eval_fhoriz = np.interp(eval_times, tf, falling_data["faero_horiz"])
    eval_alt = np.interp(eval_times, tf, falling_data["altitude"])

    return {
        "times": eval_times,
        "rot_matrices": eval_rot_matrices,
        "rpm": eval_rpm,
        "tilt": eval_tilt,
        "dx": eval_dx,
        "dz": eval_dz,
        "dhoriz": eval_dhoriz,
        "vx": eval_vx,
        "vy": eval_vy,
        "vz": eval_vz,
        "vhoriz": eval_vhoriz,
        "ax": eval_ax,
        "ay": eval_ay,
        "az": eval_az,
        "ahoriz": eval_ahoriz,
        "fx": eval_fx,
        "fz": eval_fz,
        "fhoriz": eval_fhoriz,
        "altitude": eval_alt,
    }


def extract_raw_step_telemetry(df_tumble, df_falling, stride=1, max_frames=None):
    """
    Extract raw recorded simulation steps as video frames without interpolation.
    1 recorded simulation step = 1 video frame (per-sample / raw frame mode).
    """
    times_t = df_tumble["Time"].values
    times_f = df_falling["Time"].values

    t_start = max(times_t[0], times_f[0])
    t_end = min(times_t[-1], times_f[-1])

    mask_t = (times_t >= t_start) & (times_t <= t_end)
    df_t_sub = df_tumble.iloc[mask_t].iloc[::stride]

    if max_frames is not None and max_frames > 0:
        df_t_sub = df_t_sub.iloc[:max_frames]

    eval_times = df_t_sub["Time"].values

    # Attitude from raw quaternions
    qw = df_t_sub["Quat_w"].values
    qx = df_t_sub["Quat_x"].values
    qy = df_t_sub["Quat_y"].values
    qz = df_t_sub["Quat_z"].values

    quats_xyzw = np.column_stack([qx, qy, qz, qw])
    norms = np.linalg.norm(quats_xyzw, axis=1, keepdims=True)
    quats_xyzw /= np.maximum(norms, 1e-12)

    for i in range(1, len(quats_xyzw)):
        if np.dot(quats_xyzw[i], quats_xyzw[i - 1]) < 0.0:
            quats_xyzw[i] = -quats_xyzw[i]

    rotations = R.from_quat(quats_xyzw)
    eval_rot_matrices = rotations.as_matrix()

    eval_rpm = np.sqrt(df_t_sub["RPM_x"]**2 + df_t_sub["RPM_y"]**2 + df_t_sub["RPM_z"]**2).values
    qw_clean = np.clip(np.abs(qw), -1.0, 1.0)
    eval_tilt = 2.0 * np.arccos(qw_clean) * (180.0 / np.pi)

    # Align falling kinematics at eval_times
    tf = df_falling["Time"].values
    eval_dx = np.interp(eval_times, tf, df_falling["Dx"].values)
    eval_dz = np.interp(eval_times, tf, df_falling["Dz"].values)
    eval_dhoriz = np.interp(eval_times, tf, df_falling["Dhoriz"].values)
    eval_vx = np.interp(eval_times, tf, df_falling["Vx"].values)
    eval_vy = np.interp(eval_times, tf, df_falling["Vy"].values)
    eval_vz = np.interp(eval_times, tf, df_falling["Vz"].values)
    eval_vhoriz = np.interp(eval_times, tf, df_falling["Vhoriz"].values)
    eval_ax = np.interp(eval_times, tf, df_falling["Ax"].values)
    eval_ay = np.interp(eval_times, tf, df_falling["Ay"].values)
    eval_az = np.interp(eval_times, tf, df_falling["Az"].values)
    eval_ahoriz = np.interp(eval_times, tf, df_falling["Ahoriz"].values)
    eval_fx = np.interp(eval_times, tf, df_falling["Faero_x"].values)
    eval_fz = np.interp(eval_times, tf, df_falling["Faero_z"].values)
    eval_fhoriz = np.interp(eval_times, tf, df_falling["Faero_horiz"].values)
    eval_alt = np.interp(eval_times, tf, df_falling["Altitude"].values)

    return {
        "times": eval_times,
        "rot_matrices": eval_rot_matrices,
        "rpm": eval_rpm,
        "tilt": eval_tilt,
        "dx": eval_dx,
        "dz": eval_dz,
        "dhoriz": eval_dhoriz,
        "vx": eval_vx,
        "vy": eval_vy,
        "vz": eval_vz,
        "vhoriz": eval_vhoriz,
        "ax": eval_ax,
        "ay": eval_ay,
        "az": eval_az,
        "ahoriz": eval_ahoriz,
        "fx": eval_fx,
        "fz": eval_fz,
        "fhoriz": eval_fhoriz,
        "altitude": eval_alt,
    }


# ==============================================================================
# 3D Floor Grid & Plumb-Bob Helpers
# ==============================================================================

def create_floor_grid_polydata(y_floor=-0.16, x_span=0.08, z_span=0.09, step=0.01):
    """Generate PolyData for floor grid lines."""
    pts = []
    lines = []
    pt_id = 0

    # Grid lines parallel to Z
    for x in np.arange(-x_span, x_span + 1e-5, step):
        if abs(x) < 1e-4:
            continue
        pts.append([x, y_floor, -z_span])
        pts.append([x, y_floor, z_span])
        lines.extend([2, pt_id, pt_id + 1])
        pt_id += 2

    # Grid lines parallel to X
    for z in np.arange(-z_span, z_span + 1e-5, step):
        if abs(z) < 1e-4:
            continue
        pts.append([-x_span, y_floor, z])
        pts.append([x_span, y_floor, z])
        lines.extend([2, pt_id, pt_id + 1])
        pt_id += 2

    poly = pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))
    return poly


def create_floor_axes_polydata(y_floor=-0.16, x_span=0.08, z_span=0.09):
    """Primary coordinate crosshairs on the floor."""
    pts = [
        [-x_span, y_floor, 0.0], [x_span, y_floor, 0.0],
        [0.0, y_floor, -z_span], [0.0, y_floor, z_span]
    ]
    lines = [2, 0, 1, 2, 2, 3]
    poly = pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))
    return poly


def create_floor_rings_polydata(y_floor=-0.16, radii=(0.01, 0.02, 0.03, 0.04, 0.05), n_pts=72):
    """Generate concentric range circles on the floor with smooth closed line loops."""
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

    poly = pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))
    return poly


def create_target_marker_polydata(center=(0.0, -0.16, 0.0), radius=0.0035, n_pts=36):
    """Create a small crosshair/target ring at a specified floor position."""
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
    pts.append([cx - radius * 1.6, cy, cz])
    pts.append([cx + radius * 1.6, cy, cz])
    lines.extend([2, p_start, p_start + 1])

    pts.append([cx, cy, cz - radius * 1.6])
    pts.append([cx, cy, cz + radius * 1.6])
    lines.extend([2, p_start + 2, p_start + 3])

    poly = pv.PolyData(np.array(pts, dtype=np.float32), lines=np.array(lines, dtype=np.int32))
    return poly


# ==============================================================================
# Corner Radar HUD Inset (Pillow 2D Compositor)
# ==============================================================================

def get_hud_font(size=12, bold=False):
    """Load system monospace font or fall back gracefully."""
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


def render_radar_hud_overlay(img_rgba, frame_idx, total_frames, fps, mode_tag, telem, max_drift_mm=40.0):
    """
    Renders both:
      1. Top-Left Mission Avionics Banner (time, tilt, rpm, descent, drift summary)
      2. Top-Right Aerospace Radar Compass HUD & digital telemetry card
    """
    draw = ImageDraw.Draw(img_rgba, "RGBA")

    # Font sizes
    font_banner_title = get_hud_font(12, bold=True)
    font_banner_body = get_hud_font(11, bold=False)
    font_title = get_hud_font(11, bold=True)
    font_card = get_hud_font(11, bold=False)
    font_card_b = get_hud_font(11, bold=True)
    font_tiny = get_hud_font(9, bold=False)

    sim_t = telem["times"][frame_idx]
    tilt_val = telem["tilt"][frame_idx]
    rpm_val = telem["rpm"][frame_idx]
    curr_dx = telem["dx"][frame_idx]
    curr_dz = telem["dz"][frame_idx]
    dx_mm = curr_dx * 1000.0
    dz_mm = curr_dz * 1000.0
    dhoriz_mm = telem["dhoriz"][frame_idx] * 1000.0
    v_horiz = telem["vhoriz"][frame_idx]
    ahoriz = telem["ahoriz"][frame_idx]
    bearing_deg = (math.atan2(curr_dx, curr_dz) * 180.0 / math.pi) % 360.0
    alt_m = telem["altitude"][frame_idx]
    vy_m = abs(telem["vy"][frame_idx])
    f_horiz = telem["fhoriz"][frame_idx]

    # ----------------------------------------------------
    # 1. TOP-LEFT AVIONICS STATUS BANNER
    # ----------------------------------------------------
    banner_box = [20, 16, 640, 92]
    draw.rounded_rectangle(banner_box, radius=8, fill=(10, 16, 26, 230), outline=(48, 54, 61, 255), width=1)

    # Line 1: Timing & Playback
    draw.text((32, 22), "SIM TIME:", fill=(139, 148, 158, 240), font=font_banner_body)
    time_fmt = f"{sim_t:.5f}s" if ("SAMPLE" in mode_tag or "RAW" in mode_tag) else f"{sim_t:.4f}s"
    draw.text((105, 21), time_fmt, fill=(255, 255, 255, 255), font=font_banner_title)
    draw.text((185, 22), f"({mode_tag})", fill=(88, 166, 255, 240), font=font_banner_body)
    step_label = "STEP:" if ("SAMPLE" in mode_tag or "RAW" in mode_tag) else "FRAME:"
    draw.text((345, 22), step_label, fill=(139, 148, 158, 240), font=font_banner_body)
    draw.text((395, 21), f"{frame_idx+1:,}/{total_frames:,}", fill=(201, 209, 217, 255), font=font_banner_title)
    draw.text((505, 22), f"({fps} FPS)", fill=(110, 118, 129, 220), font=font_banner_body)

    # Line 2: Attitude & Descent
    draw.text((32, 44), "ATTITUDE:", fill=(139, 148, 158, 240), font=font_banner_body)
    draw.text((105, 43), f"TILT {tilt_val:4.1f}°  |  SPEED {rpm_val:5.0f} RPM", fill=(57, 255, 20, 255), font=font_card_b)
    draw.text((385, 43), f"DESCENT: {alt_m:4.2f}m ({vy_m:3.1f}m/s)", fill=(165, 214, 255, 230), font=font_card_b)

    # Line 3: 6DoF Horizontal Drift
    draw.text((32, 66), "6DoF DRIFT:", fill=(139, 148, 158, 240), font=font_banner_body)
    draw.text((115, 65), f"{dhoriz_mm:5.1f} mm", fill=(0, 242, 254, 255), font=font_card_b)
    draw.text((205, 66), f"(ΔX: {dx_mm:+4.1f}mm, ΔZ: {dz_mm:+4.1f}mm)", fill=(139, 148, 158, 230), font=font_banner_body)
    draw.text((435, 65), f"V_horiz: {v_horiz:5.3f} m/s", fill=(255, 215, 0, 240), font=font_card_b)

    # ----------------------------------------------------
    # 2. TOP-RIGHT RADAR COMPASS HUD & TELEMETRY
    # ----------------------------------------------------
    cx, cy = 1005, 145
    r_radar = 76
    max_range_m = max(0.040, max_drift_mm * 1e-3)
    px_per_m = r_radar / max_range_m

    # Radar Glass Background
    bg_box = [cx - r_radar - 12, cy - r_radar - 32, cx + r_radar + 12, cy + r_radar + 185]
    draw.rounded_rectangle(bg_box, radius=12, fill=(10, 16, 26, 230), outline=(48, 54, 61, 255), width=1)

    # Title Banner over Radar (with ample top margin)
    draw.text((cx - r_radar, cy - r_radar - 26), "6DoF HORIZ RADAR", fill=(88, 166, 255, 255), font=font_title)

    # Scope Circle Background
    draw.ellipse([cx - r_radar, cy - r_radar, cx + r_radar, cy + r_radar], fill=(6, 11, 20, 245), outline=(56, 139, 253, 230), width=2)

    # Concentric Range Rings
    radii_mm = [10.0, 20.0, 30.0, 40.0]
    for r_mm in radii_mm:
        r_px = (r_mm * 1e-3) * px_per_m
        if r_px <= r_radar:
            color = (33, 53, 85, 180) if r_mm != 40.0 else (56, 139, 253, 140)
            draw.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px], outline=color, width=1)
            draw.text((cx + 3, cy - r_px - 8), f"{int(r_mm)}", fill=(100, 130, 170, 160), font=font_tiny)

    # Crosshairs (+Z Span is UP, +X Normal is RIGHT)
    draw.line([cx - r_radar, cy, cx + r_radar, cy], fill=(30, 45, 70, 200), width=1)
    draw.line([cx, cy - r_radar, cx, cy + r_radar], fill=(30, 45, 70, 200), width=1)

    # Axis Labels
    draw.text((cx - 16, cy - r_radar - 14), "+Z (Span)", fill=(88, 166, 255, 230), font=font_tiny)
    draw.text((cx + r_radar - 14, cy - 14), "+X", fill=(88, 166, 255, 230), font=font_tiny)
    draw.text((cx - 16, cy + r_radar + 3), "-Z", fill=(100, 120, 150, 180), font=font_tiny)
    draw.text((cx - r_radar - 12, cy - 14), "-X", fill=(100, 120, 150, 180), font=font_tiny)

    # Radar Sweep Line
    sweep_angle = (sim_t * 6.0) % (2.0 * math.pi)
    sw_x = cx + r_radar * math.cos(sweep_angle)
    sw_y = cy + r_radar * math.sin(sweep_angle)
    draw.line([cx, cy, sw_x, sw_y], fill=(0, 242, 254, 70), width=1)

    # Origin Drop Marker (0, 0)
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

    halo_r = 6 + int(2 * math.sin(sim_t * 20.0))
    draw.ellipse([curr_px - halo_r, curr_py - halo_r, curr_px + halo_r, curr_py + halo_r], outline=(57, 255, 20, 120), width=1)
    draw.ellipse([curr_px - 3, curr_py - 3, curr_px + 3, curr_py + 3], fill=(57, 255, 20, 255))

    # Dynamic Velocity Vector Arrow on Radar (Green)
    vx = telem["vx"][frame_idx]
    vz = telem["vz"][frame_idx]
    if v_horiz > 1e-4:
        v_arrow_len = min(35.0, v_horiz * 280.0)
        v_dir_x = (vx / v_horiz) * v_arrow_len
        v_dir_y = -(vz / v_horiz) * v_arrow_len
        v_tip_x = curr_px + v_dir_x
        v_tip_y = curr_py + v_dir_y
        draw.line([curr_px, curr_py, v_tip_x, v_tip_y], fill=(57, 255, 20, 240), width=2)
        draw.ellipse([v_tip_x - 2, v_tip_y - 2, v_tip_x + 2, v_tip_y + 2], fill=(57, 255, 20, 255))

    # Lateral Force Vector Arrow on Radar (Coral / Red)
    fx = telem["fx"][frame_idx]
    fz = telem["fz"][frame_idx]
    if f_horiz > 1e-5:
        f_arrow_len = min(30.0, f_horiz * 2200.0)
        f_dir_x = (fx / f_horiz) * f_arrow_len
        f_dir_y = -(fz / f_horiz) * f_arrow_len
        f_tip_x = curr_px + f_dir_x
        f_tip_y = curr_py + f_dir_y
        draw.line([curr_px, curr_py, f_tip_x, f_tip_y], fill=(255, 85, 0, 230), width=2)
        draw.ellipse([f_tip_x - 2, f_tip_y - 2, f_tip_x + 2, f_tip_y + 2], fill=(255, 85, 0, 255))

    # Telemetry Card Readouts
    card_top = cy + r_radar + 14
    lines = [
        ("H-DRIFT", f"{dhoriz_mm:5.1f} mm", (0, 242, 254, 255)),
        ("DX / DZ", f"{dx_mm:+4.1f}/{dz_mm:+4.1f}", (140, 180, 230, 240)),
        ("BEARING", f"{bearing_deg:5.1f}°", (200, 215, 235, 240)),
        ("H-SPEED", f"{v_horiz:5.3f} m/s", (57, 255, 20, 255)),
        ("H-ACCEL", f"{ahoriz:5.2f} m/s²", (255, 215, 0, 240)),
        ("LAT FORCE", f"{f_horiz*1000:5.2f} mN", (255, 110, 50, 255)),
        ("DESCENT", f"{alt_m:5.2f}m @ {vy_m:4.1f}m/s", (180, 195, 210, 220)),
    ]

    curr_y = card_top
    for label, val, val_col in lines:
        draw.text((cx - r_radar, curr_y), label, fill=(139, 148, 158, 220), font=font_card)
        draw.text((cx - 8, curr_y), val, fill=val_col, font=font_card_b)
        curr_y += 18

    # ----------------------------------------------------
    # 3. RIGHT-EDGE GLASS COCKPIT ALTITUDE TAPE & VSI
    # ----------------------------------------------------
    font_alt_box = get_hud_font(13, bold=True)

    # Altitude Tape
    x_tape_l, x_tape_r = 1135, 1222
    y_tape_t, y_tape_b = 45, 675
    y_mid = 360
    px_per_m = 150.0

    draw.rounded_rectangle([x_tape_l, y_tape_t, x_tape_r, y_tape_b], radius=8, fill=(10, 16, 26, 235), outline=(48, 54, 61, 255), width=1)
    draw.text((x_tape_l + 8, y_tape_t + 8), "ALT (m)", fill=(88, 166, 255, 255), font=font_title)
    draw.text((x_tape_l + 8, y_tape_t + 24), "REL 0.00", fill=(247, 37, 133, 255), font=font_tiny)

    h_span = (y_tape_b - y_tape_t) / (2.0 * px_per_m)
    h_min = alt_m - h_span
    h_max = alt_m + h_span

    for h_step in np.arange(np.floor(h_min * 10) / 10.0, np.ceil(h_max * 10) / 10.0 + 0.05, 0.1):
        h_val = round(h_step, 2)
        y_pos = y_mid - (h_val - alt_m) * px_per_m
        if y_tape_t + 40 <= y_pos <= y_tape_b - 25:
            is_major = (abs(round(h_val * 2) - h_val * 2) < 1e-4)
            is_meter = (abs(round(h_val) - h_val) < 1e-4)
            t_len = 16 if is_major else 8
            col = (255, 255, 255, 240) if is_major else (110, 125, 145, 180)
            draw.line([x_tape_l, y_pos, x_tape_l + t_len, y_pos], fill=col, width=2 if is_meter else 1)
            if is_major:
                draw.text((x_tape_l + t_len + 4, y_pos - 6), f"{h_val:+.1f}", fill=(201, 209, 217, 240), font=font_card)

    # Center Readout Box (Rolling Altitude Box with left pointer chevron)
    box_top = y_mid - 16
    box_bot = y_mid + 16
    draw.polygon([
        (x_tape_l - 12, y_mid),
        (x_tape_l, box_top),
        (x_tape_r, box_top),
        (x_tape_r, box_bot),
        (x_tape_l, box_bot)
    ], fill=(6, 11, 20, 255), outline=(247, 37, 133, 255), width=2)
    draw.text((x_tape_l + 4, y_mid - 8), f"{alt_m:+.2f}m", fill=(255, 255, 255, 255), font=font_alt_box)
    draw.text((x_tape_l + 10, y_tape_b - 20), "GCS BARO", fill=(57, 255, 20, 240), font=font_tiny)

    # Vertical Speed Indicator (VSI) Tape
    x_vsi_l, x_vsi_r = 1227, 1262
    y_vsi_t, y_vsi_b = y_tape_t + 100, y_tape_b - 100
    draw.rounded_rectangle([x_vsi_l, y_vsi_t, x_vsi_r, y_vsi_b], radius=6, fill=(10, 16, 26, 235), outline=(48, 54, 61, 255), width=1)
    draw.text((x_vsi_l + 8, y_vsi_t + 6), "VSI", fill=(88, 166, 255, 240), font=font_tiny)

    vsi_scale = 32.0  # px per m/s
    vsi_ticks = [(2, "+2"), (1, "+1"), (0, "0"), (-1, "-1"), (-2, "-2"), (-4, "-4"), (-6, "-6")]
    for v_val, v_lbl in vsi_ticks:
        y_tick = y_mid - v_val * vsi_scale
        draw.line([x_vsi_l, y_tick, x_vsi_l + 8, y_tick], fill=(139, 148, 158, 220), width=1)
        if v_val in [0, -2, -4, -6]:
            draw.text((x_vsi_l + 12, y_tick - 5), str(abs(v_val)), fill=(139, 148, 158, 220), font=font_tiny)

    # Dynamic VSI Pointer (points to vertical speed vy)
    vy_actual = telem["vy"][frame_idx]  # negative for descent
    y_ptr = np.clip(y_mid - vy_actual * vsi_scale, y_vsi_t + 15, y_vsi_b - 15)
    draw.polygon([
        (x_vsi_l - 2, y_ptr),
        (x_vsi_l + 10, y_ptr - 8),
        (x_vsi_r - 2, y_ptr - 8),
        (x_vsi_r - 2, y_ptr + 8),
        (x_vsi_l + 10, y_ptr + 8)
    ], fill=(255, 85, 0, 240), outline=(255, 255, 255, 255), width=1)
    draw.text((x_vsi_l + 6, y_ptr - 6), f"{abs(vy_actual):.1f}", fill=(255, 255, 255, 255), font=font_card_b)


# ==============================================================================
# Worker Rendering Function
# ==============================================================================

def _render_drift_chunk_worker(worker_id, start_idx, end_idx, total_frames,
                               stl_path, telem,
                               fps, style, mode_tag,
                               width, height, out_chunk_path,
                               shared_counter, counter_lock):
    """
    Subprocess worker: renders an allocated slice of frames with 3D floor grid,
    laser plumb-bob, dynamic vectors, and corner radar HUD.
    """
    try:
        # Load local mesh and feature edges
        mesh = pv.read(stl_path)
        edges = mesh.extract_feature_edges(feature_angle=25, boundary_edges=True, feature_edges=True, manifold_edges=False)
        base_points_mesh = mesh.points.copy()
        base_points_edges = edges.points.copy()

        y_floor = -0.16  # 3.3 cm below 10-inch propeller span envelope

        pv.OFF_SCREEN = True
        plotter = pv.Plotter(off_screen=True, window_size=[width, height])
        plotter.set_background("#0d1117")

        # Camera positioned to clearly view the prop, plumb line, and floor grid
        plotter.camera_position = [(0.28, 0.18, 0.28), (0.005, -0.06, 0.015), (0.0, 1.0, 0.0)]
        plotter.camera.zoom(1.18)
        plotter.add_axes(interactive=False, line_width=2.0)

        # 1. Floor Reference Grid, Rings, & Coordinate Axes
        grid_poly = create_floor_grid_polydata(y_floor=y_floor, x_span=0.08, z_span=0.09, step=0.01)
        rings_poly = create_floor_rings_polydata(y_floor=y_floor, radii=(0.01, 0.02, 0.03, 0.04, 0.05))
        axes_poly = create_floor_axes_polydata(y_floor=y_floor, x_span=0.08, z_span=0.09)
        plotter.add_mesh(grid_poly, color="#1f2937", line_width=1.0)
        plotter.add_mesh(rings_poly, color="#374151", line_width=1.5)
        plotter.add_mesh(axes_poly, color="#4b5563", line_width=1.8)

        # Origin release target marker on floor
        origin_marker = create_target_marker_polydata(center=(0.0, y_floor, 0.0), radius=0.0035)
        plotter.add_mesh(origin_marker, color="#388bfd", line_width=2.0)

        # 2. Ground Track on Floor (Pre-allocated PolyData for maximum speed)
        track_pts = np.full((total_frames, 3), [telem["dx"][0], y_floor + 0.0005, telem["dz"][0]], dtype=np.float32)
        track_lines = []
        for i in range(total_frames - 1):
            track_lines.extend([2, i, i + 1])
        track_poly = pv.PolyData(track_pts, lines=np.array(track_lines, dtype=np.int32))
        track_actor = plotter.add_mesh(track_poly, color="#00f2fe", line_width=3.5)

        # 3. Laser Plumb-Bob (Dropline from Hub to Floor)
        plumb_pts = np.array([[0.0, 0.0, 0.0], [0.0, y_floor, 0.0]], dtype=np.float32)
        plumb_poly = pv.PolyData(plumb_pts, lines=np.array([2, 0, 1], dtype=np.int32))
        plumb_actor = plotter.add_mesh(plumb_poly, color="#39ff14", line_width=3.5)

        # Ground target blip under plumb-bob
        ground_blip_poly = create_target_marker_polydata(center=(0.0, y_floor, 0.0), radius=0.0035)
        blip_actor = plotter.add_mesh(ground_blip_poly, color="#39ff14", line_width=2.5)
        blip_base_pts = ground_blip_poly.points.copy()

        # 4. 3D Hub Velocity Vector (Green) & Lateral Force Vector (Coral)
        vel_line_pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        vel_poly = pv.PolyData(vel_line_pts, lines=np.array([2, 0, 1], dtype=np.int32))
        vel_actor = plotter.add_mesh(vel_poly, color="#39ff14", line_width=4.5)

        force_line_pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        force_poly = pv.PolyData(force_line_pts, lines=np.array([2, 0, 1], dtype=np.int32))
        force_actor = plotter.add_mesh(force_poly, color="#ff5500", line_width=4.5)

        # 5. Relative Airflow Wind Streaks (Upward-streaming relative to falling body)
        n_streaks = 75
        np.random.seed(42)
        r_streaks = np.random.uniform(0.035, 0.22, n_streaks)
        theta_streaks = np.random.uniform(0, 2 * np.pi, n_streaks)
        wx = (r_streaks * np.cos(theta_streaks)).astype(np.float32)
        wz = (r_streaks * np.sin(theta_streaks)).astype(np.float32)
        wy0 = np.random.uniform(-0.22, 0.22, n_streaks).astype(np.float32)
        base_lens = np.random.uniform(0.04, 0.07, n_streaks).astype(np.float32)
        y_min_span, y_max_span = -0.22, 0.22
        y_span = y_max_span - y_min_span

        wind_pts = np.zeros((n_streaks * 2, 3), dtype=np.float32)
        wind_lines = []
        for i in range(n_streaks):
            wind_pts[2 * i] = [wx[i], wy0[i], wz[i]]
            wind_pts[2 * i + 1] = [wx[i], wy0[i] + base_lens[i], wz[i]]
            wind_lines.extend([2, 2 * i, 2 * i + 1])

        wind_poly = pv.PolyData(wind_pts, lines=np.array(wind_lines, dtype=np.int32))
        wind_actor = plotter.add_mesh(wind_poly, color="#58a6ff", line_width=1.8, opacity=0.45)

        # 6. Propeller Body & Edges
        mesh_actor = None
        if style in ["solid", "both"]:
            mesh_actor = plotter.add_mesh(
                mesh.copy(),
                color="#388bfd",
                opacity=1.0,
                smooth_shading=True,
                specular=0.45,
                ambient=0.30
            )

        edges_actor = plotter.add_mesh(
            edges.copy(),
            color="#ffffff",
            line_width=2.5,
            render_lines_as_tubes=False
        )

        max_d_mm = max(40.0, float(np.max(telem["dhoriz"]) * 1000.0))

        # Open video writer via imageio
        os.makedirs(os.path.dirname(os.path.abspath(out_chunk_path)), exist_ok=True)
        writer = imageio.get_writer(out_chunk_path, fps=fps, quality=8, codec="libx264")

        # Initial render to set up OpenGL pipeline
        plotter.show(auto_close=False)

        for idx in range(start_idx, end_idx):
            rot_mat = telem["rot_matrices"][idx]
            dx = telem["dx"][idx]
            dz = telem["dz"][idx]
            vx = telem["vx"][idx]
            vz = telem["vz"][idx]
            fx = telem["fx"][idx]
            fz = telem["fz"][idx]

            hub_pos = np.array([dx, 0.0, dz], dtype=np.float32)

            # Update propeller position & orientation
            if mesh_actor is not None:
                mesh_actor.mapper.dataset.points = (base_points_mesh @ rot_mat.T) + hub_pos
            edges_actor.mapper.dataset.points = (base_points_edges @ rot_mat.T) + hub_pos

            # Update Laser Plumb-Bob
            plumb_actor.mapper.dataset.points = np.array([
                hub_pos,
                [dx, y_floor, dz]
            ], dtype=np.float32)

            # Update Ground Target Blip
            blip_actor.mapper.dataset.points = blip_base_pts + np.array([dx, 0.0, dz], dtype=np.float32)

            # Update Cumulative Ground Track
            track_pts[:idx + 1, 0] = telem["dx"][:idx + 1]
            track_pts[:idx + 1, 1] = y_floor + 0.0005
            track_pts[:idx + 1, 2] = telem["dz"][:idx + 1]
            if idx + 1 < total_frames:
                track_pts[idx + 1:, 0] = dx
                track_pts[idx + 1:, 1] = y_floor + 0.0005
                track_pts[idx + 1:, 2] = dz
            track_actor.mapper.dataset.points = track_pts

            # Update 3D Velocity Arrow at Hub
            v_scale = 0.35  # 0.1 m/s -> 3.5 cm arrow
            vel_actor.mapper.dataset.points = np.array([
                hub_pos,
                hub_pos + np.array([vx * v_scale, 0.0, vz * v_scale], dtype=np.float32)
            ], dtype=np.float32)

            # Update 3D Aerodynamic Force Arrow at Hub
            f_scale = 3.5   # 0.01 N -> 3.5 cm arrow
            force_actor.mapper.dataset.points = np.array([
                hub_pos,
                hub_pos + np.array([fx * f_scale, 0.0, fz * f_scale], dtype=np.float32)
            ], dtype=np.float32)

            # Update Upward Relative Wind Streaks
            d_fall = -telem["altitude"][idx]   # fallen depth >= 0
            vy_mag = abs(telem["vy"][idx])     # descent speed m/s
            curr_wind_pts = np.zeros((n_streaks * 2, 3), dtype=np.float32)
            for s_i in range(n_streaks):
                y_base = y_min_span + ((wy0[s_i] + d_fall) - y_min_span) % y_span
                len_i = base_lens[s_i] * (1.0 + 1.2 * min(1.0, vy_mag / 5.5))
                y_top = y_base + len_i
                curr_wind_pts[2 * s_i] = [dx + wx[s_i], y_base, dz + wz[s_i]]
                curr_wind_pts[2 * s_i + 1] = [dx + wx[s_i], y_top, dz + wz[s_i]]
            wind_actor.mapper.dataset.points = curr_wind_pts

            # Render 3D Scene and extract RGB image
            plotter.render()
            frame_rgb = plotter.screenshot(return_img=True)

            # 2D Corner Radar HUD & Mission Banner Overlay via Pillow
            img_pil = Image.fromarray(frame_rgb)
            render_radar_hud_overlay(img_pil, idx, total_frames, fps, mode_tag, telem, max_drift_mm=max_d_mm)
            annotated_frame = np.array(img_pil)

            writer.append_data(annotated_frame)

            if shared_counter is not None and counter_lock is not None:
                with counter_lock:
                    shared_counter.value += 1

        writer.close()
        plotter.close()

    except Exception as e:
        print(f"\n[Worker {worker_id} Error] {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        raise e


# ==============================================================================
# Main Animation Driver
# ==============================================================================

def render_drift_animation(stl_path, tumble_path, falling_path,
                           output_path="slowdown20-Radar/Grid.mp4",
                           fps=60, real_time=False, slowdown=20.0,
                           every_input=False, stride=1,
                           workers=4, max_frames=None, style="solid",
                           preview=False, width=1280, height=720, skip_points=5):
    """
    Renders 3D propeller tumbling with 6DoF lateral drift, floor grid track,
    and corner radar avionics HUD across multi-process workers.

    Playback Modes:
      1. Real-Time (--real-time):
         Treats 1 second of simulation time as exactly 1 second in video (slowdown = 1.0).
      2. Time-Scaled (--slowdown X):
         User defines how many video seconds correspond to 1 second of simulation time.
      3. Per-Sample / Raw Frames (--every-input / --per-sample):
         Treats every recorded simulation step in motion.dat as 1 video frame without interpolation.
    """
    print("=" * 76)
    print("  6DoF Propeller Motion & Drift Visualizer (Floor Grid + Corner Radar HUD)")
    print("=" * 76)

    # 1. Hardware & Files Info
    gpu_info = pv.GPUInfo()
    print(f"  • Hardware GPU    : {gpu_info.renderer} ({gpu_info.version})")
    print(f"  • Geometry STL    : {os.path.basename(stl_path)}")
    print(f"  • Tumble File     : {os.path.relpath(tumble_path)}")
    print(f"  • Falling File    : {os.path.relpath(falling_path)}")

    # 2. Load Telemetry & Configure Playback Mode
    df_tumble = load_and_filter_tumble(tumble_path, skip_points=skip_points)
    df_falling = load_and_filter_falling(falling_path, skip_points=skip_points)

    if every_input:
        telem = extract_raw_step_telemetry(df_tumble, df_falling, stride=stride, max_frames=max_frames)
        eval_times = telem["times"]
        total_frames = len(eval_times)
        duration_sim = eval_times[-1] - eval_times[0] if total_frames > 1 else 0.0
        duration_video = total_frames / float(fps)
        mode_tag = f"PER-SAMPLE (s={stride})"
        mode_label = f"STEP-BY-STEP RAW FRAMES (--every-input, 1 sample = 1 frame, stride={stride})"
    else:
        if real_time:
            time_scale = 1.0
            mode_tag = "REAL-TIME (1.0x)"
            mode_label = "REAL-TIME (1s sim time = 1s video)"
        else:
            time_scale = float(slowdown)
            mode_tag = f"{time_scale:.1f}x SLOW-MO"
            mode_label = f"TIME-SCALED ({time_scale:.1f}x SLOWDOWN, {time_scale:.1f} video sec per 1s sim time)"

        t_start, t_end, tumble_data, falling_data = build_synchronized_interpolators(df_tumble, df_falling)
        duration_sim = t_end - t_start
        duration_video = max(duration_sim * time_scale, 1.0 / fps)
        total_frames = max(1, int(np.round(duration_video * fps)))

        if max_frames is not None and max_frames > 0:
            total_frames = min(total_frames, max_frames)
            duration_video = total_frames / float(fps)
            t_span_eval = total_frames / (fps * time_scale)
            eval_times = np.linspace(t_start, min(t_start + t_span_eval, t_end), total_frames)
        else:
            eval_times = np.linspace(t_start, t_end, total_frames)

        telem = evaluate_telemetry_frames(eval_times, tumble_data, falling_data)

    print(f"  • Playback Mode   : {mode_label}")
    print(f"  • Simulation Time : {eval_times[0]:.4f}s to {eval_times[-1]:.4f}s (ΔT = {eval_times[-1]-eval_times[0]:.4f}s)")
    print(f"  • Max Lateral Drift: {np.max(telem['dhoriz'])*1000.0:5.1f} mm (Z: {np.max(telem['dz'])*1000.0:5.1f} mm, X: {np.max(telem['dx'])*1000.0:5.1f} mm)")
    print(f"  • Target FPS      : {fps} FPS")
    print(f"  • Total Frames    : {total_frames:,} frames")
    print(f"  • Video Duration  : {duration_video:.2f} s ({duration_video/60.0:.2f} min)")
    print(f"  • Output File     : {output_path}")

    # Ensure parent output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    # ----------------------------------------------------
    # PREVIEW MODE: Save single high-res still image
    # ----------------------------------------------------
    if preview:
        print("\nGenerating high-resolution still preview screenshot ...")
        preview_path = "propeller_drift_preview.png"
        mid_idx = min(total_frames - 1, total_frames // 2 if total_frames > 1 else 0)

        # Render 1 frame sequentially
        _render_drift_chunk_worker(
            worker_id=0,
            start_idx=mid_idx,
            end_idx=mid_idx + 1,
            total_frames=total_frames,
            stl_path=stl_path,
            telem=telem,
            fps=fps,
            style=style,
            mode_tag=mode_tag,
            width=width,
            height=height,
            out_chunk_path="/tmp/pv_preview_temp.mp4",
            shared_counter=None,
            counter_lock=None
        )

        # Extract first frame from temp video and save as PNG
        reader = imageio.get_reader("/tmp/pv_preview_temp.mp4")
        frame_img = reader.get_data(0)
        reader.close()
        if os.path.exists("/tmp/pv_preview_temp.mp4"):
            os.remove("/tmp/pv_preview_temp.mp4")

        Image.fromarray(frame_img).save(preview_path)
        print(f"[Success] Still preview image rendered to: {os.path.abspath(preview_path)}")
        return

    # ----------------------------------------------------
    # MULTI-PROCESS PARALLEL RENDERING
    # ----------------------------------------------------
    num_workers = max(1, min(int(workers), total_frames))

    if num_workers <= 1:
        print(f"  • Workers         : 1 (Sequential mode)")
        print(f"\nEncoding video to: {output_path} ...")
        t0 = time.time()
        _render_drift_chunk_worker(
            worker_id=0,
            start_idx=0,
            end_idx=total_frames,
            total_frames=total_frames,
            stl_path=stl_path,
            telem=telem,
            fps=fps,
            style=style,
            mode_tag=mode_tag,
            width=width,
            height=height,
            out_chunk_path=output_path,
            shared_counter=None,
            counter_lock=None
        )
        dt = time.time() - t0
        print(f"\n[Success] Animation rendered in {dt:.1f}s ({total_frames/max(dt,1e-3):.1f} FPS)!")
        print(f"Video saved: {os.path.abspath(output_path)} ({os.path.getsize(output_path)/1024/1024:.2f} MB)")
        return

    print(f"  • Parallel Workers: {num_workers} processes (Concurrent GPU rendering)")
    temp_dir = tempfile.mkdtemp(prefix="pv_drift_parts_")

    try:
        splits = np.array_split(np.arange(total_frames), num_workers)
        chunk_tasks = []
        chunk_files = []

        for cid, c_range in enumerate(splits):
            if len(c_range) == 0:
                continue
            chunk_file = os.path.join(temp_dir, f"part_{cid:03d}.mp4")
            chunk_files.append(chunk_file)
            chunk_tasks.append((cid, c_range[0], c_range[-1] + 1, chunk_file))

        active_workers = len(chunk_tasks)
        print(f"\nDispatching {total_frames:,} frames across {active_workers} parallel workers ...")

        shared_counter = mp.Value('i', 0)
        counter_lock = mp.Lock()

        processes = []
        for (cid, s_idx, e_idx, c_file) in chunk_tasks:
            p = mp.Process(
                target=_render_drift_chunk_worker,
                args=(
                    cid, s_idx, e_idx, total_frames,
                    stl_path, telem,
                    fps, style, mode_tag,
                    width, height, c_file,
                    shared_counter, counter_lock
                )
            )
            processes.append(p)

        t0_render = time.time()
        for p in processes:
            p.start()

        # Monitor progress across workers
        last_print = 0
        while any(p.is_alive() for p in processes):
            now = time.time()
            with counter_lock:
                current = shared_counter.value

            if now - last_print >= 0.5 or current == total_frames:
                pct = (current / total_frames) * 100.0
                elapsed = now - t0_render
                fps_render = current / max(elapsed, 1e-3)
                eta = (total_frames - current) / max(fps_render, 1e-3)
                sys.stdout.write(
                    f"\r  Rendered {current:,}/{total_frames:,} frames ({pct:5.1f}%) | "
                    f"Speed: {fps_render:5.1f} FPS | ETA: {eta:4.1f}s [{active_workers} workers]"
                )
                sys.stdout.flush()
                last_print = now
            time.sleep(0.1)

        for p in processes:
            p.join()
            if p.exitcode != 0:
                raise RuntimeError(f"Worker process failed with exit code {p.exitcode}")

        dt_render = time.time() - t0_render
        print(f"\n\nAll {active_workers} chunks completed in {dt_render:.1f}s ({total_frames/max(dt_render,1e-3):.1f} FPS overall)!")

        # Lossless Stitching with FFmpeg
        print("Stitching video chunks losslessly with FFmpeg stream copy ...")
        concat_list_file = os.path.join(temp_dir, "concat_list.txt")
        with open(concat_list_file, "w") as f:
            for c_path in chunk_files:
                f.write(f"file '{c_path}'\n")

        ffmpeg_cmd = [
            "ffmpeg", "-y", "-v", "error",
            "-f", "concat", "-safe", "0",
            "-i", concat_list_file,
            "-c", "copy",
            output_path
        ]
        subprocess.run(ffmpeg_cmd, check=True)

        total_time = time.time() - t0_render
        print(f"[Success] Video saved: {os.path.abspath(output_path)} ({os.path.getsize(output_path)/1024/1024:.2f} MB)")
        print(f"Total time elapsed: {total_time:.1f}s (Effective speed: {total_frames/max(total_time,1e-3):.1f} FPS)")

    finally:
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir, ignore_errors=True)


# ==============================================================================
# CLI Entrypoint
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="6DoF Propeller Motion & Drift Visualizer (Floor Grid + Corner Radar HUD)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Playback Modes:
  1. Time-Scaled Slowdown (default):
     python3 animate_drift.py --slowdown 20 --fps 60 --o slowdown20-Radar/Grid.mp4

  2. Real-Time Playback:
     python3 animate_drift.py --real-time --fps 60 -o realtime-Radar/Grid.mp4

  3. Per-Sample / Raw Step as Frame:
     python3 animate_drift.py --every-input --fps 60 -o persample-Radar/Grid.mp4

  4. Per-Sample with Stride (every 5th step):
     python3 animate_drift.py --per-sample --stride 5 --fps 60 -o persample_s5.mp4

  5. Single-Frame Preview Screenshot:
     python3 animate_drift.py --preview
"""
    )
    parser.add_argument("--case", type=str, default=".", help="Case directory (default: current directory)")

    # Output filename options (supports -o, --o, and --output)
    parser.add_argument(
        "-o", "--o", "--output",
        type=str,
        default="slowdown20-Radar/Grid.mp4",
        dest="output",
        help="Output MP4 filename (default: slowdown20-Radar/Grid.mp4)"
    )

    # Playback Mode Arguments
    parser.add_argument(
        "--real-time",
        action="store_true",
        dest="real_time",
        help="Treat 1 second of simulation time as 1 second in the video (real-time playback, slowdown = 1.0)"
    )
    parser.add_argument(
        "--slowdown", "--video-sec-per-sim-sec", "--time-scale", "--sec-per-sec",
        type=float,
        default=None,
        dest="slowdown",
        help="Slowdown factor: video seconds per 1s simulation time (default: 20.0 unless --real-time or --every-input is specified)"
    )
    parser.add_argument(
        "--every-input", "--per-sample", "--raw-frames", "--every-step",
        action="store_true",
        dest="every_input",
        help="Treat every recorded simulation step/sample in motion.dat as 1 video frame (no interpolation). Total video length is controlled via --fps and --stride."
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=1,
        help="Step stride in per-sample (--every-input) mode (e.g. 1 = every step, 2 = every 2nd step, 10 = every 10th step). Default: 1"
    )

    # Frame rate
    parser.add_argument(
        "--fps",
        type=int,
        default=60,
        help="Output video framerate (default: 60 FPS)"
    )

    # Multi-processing workers
    parser.add_argument(
        "-j", "--workers", "--parallel",
        type=int,
        default=4,
        dest="workers",
        help="Number of parallel worker processes (default: 4)"
    )

    # Max frames for testing
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Maximum frames to render (useful for quick testing)"
    )

    # Style and Preview
    parser.add_argument(
        "--style",
        type=str,
        default="solid",
        choices=["solid", "outline"],
        help="Render style: 'solid' (colored body + outline) or 'outline' (perimeter wireframe)"
    )
    parser.add_argument("--preview", action="store_true", help="Generate a still preview image instead of full video")
    parser.add_argument("--skip-restart-points", type=int, default=5, help="Number of restart transient points to filter (default: 5)")

    args = parser.parse_args()

    # Resolve playback mode and slowdown
    if args.every_input:
        slowdown = 1.0
    elif args.real_time:
        slowdown = 1.0
    elif args.slowdown is not None:
        slowdown = args.slowdown
    else:
        slowdown = 20.0  # default

    case_dir = os.path.abspath(args.case)
    stl_path = locate_propeller_stl(case_dir)
    tumble_path, falling_path = locate_telemetry_files(case_dir)

    if not stl_path:
        print(f"[Error] Could not locate propeller STL geometry in {case_dir}", file=sys.stderr)
        sys.exit(1)

    if not tumble_path or not falling_path:
        print(f"[Error] Missing telemetry files in {case_dir}:", file=sys.stderr)
        print(f"  • tumbleMotion/motion.dat : {'FOUND' if tumble_path else 'MISSING'}", file=sys.stderr)
        print(f"  • fallingMotion/motion.dat: {'FOUND' if falling_path else 'MISSING'}", file=sys.stderr)
        sys.exit(1)

    # Handle output path
    out_file = args.output
    if not os.path.isabs(out_file):
        out_file = os.path.join(case_dir, out_file)

    render_drift_animation(
        stl_path=stl_path,
        tumble_path=tumble_path,
        falling_path=falling_path,
        output_path=out_file,
        fps=args.fps,
        real_time=args.real_time,
        slowdown=slowdown,
        every_input=args.every_input,
        stride=args.stride,
        workers=args.workers,
        max_frames=args.max_frames,
        style=args.style,
        preview=args.preview,
        skip_points=args.skip_restart_points
    )


if __name__ == "__main__":
    main()
