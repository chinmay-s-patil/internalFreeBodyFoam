#!/usr/bin/env python3
"""
animate_motion.py - 3D 6DoF Visualizer for steelBallRun with golf-ball dot markers.

Visualizes the 3D rotating steel ball using PyVista:
- True solid sphere (metallic steel)
- 32 high-contrast visual surface markers ("golf ball dots")
- Instant visual recognition of 3D rotation (roll, pitch, yaw) in a flash
- Live telemetry HUD (time, velocity, spin RPM, attitude angles)
- MP4 export / interactive 3D window

Usage:
    python3 animate_motion.py                     # Generates preview animation/video
    python3 animate_motion.py --video ball.mp4    # Exports high-res MP4 video
    python3 animate_motion.py --interactive       # Opens interactive 3D PyVista window
"""

import argparse
import json
import os
import re
import sys
from pathlib import Path
import numpy as np
import pyvista as pv

def parse_trajectory(logfile_path):
    times = []
    vels = []
    alts = []
    omegas = []
    angles = []

    time_re = re.compile(r"^Time = ([\d\.\+eE-]+)")
    vel_re = re.compile(r"^V = ([\d\.\+eE-]+) m/s")
    alt_re = re.compile(r"^Altitude = ([\d\.\+eE-]+) m")
    motion_re = re.compile(r"freeBody6DoF Motion:\s+Omega = \(([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\) rad/s.*?Angles = \(([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\s+([\d\.\+eE-]+)\) deg")

    cur_time = None
    cur_v = 0.0
    cur_alt = 0.0
    cur_om = (0.0, 0.0, 0.0)
    cur_ang = (0.0, 0.0, 0.0)

    if not os.path.exists(logfile_path):
        return None

    with open(logfile_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            tm = time_re.match(line)
            if tm:
                if cur_time is not None:
                    times.append(cur_time)
                    vels.append(cur_v)
                    alts.append(cur_alt)
                    omegas.append(cur_om)
                    angles.append(cur_ang)
                cur_time = float(tm.group(1))
                continue

            vm = vel_re.match(line)
            if vm:
                cur_v = float(vm.group(1))
                continue

            am = alt_re.match(line)
            if am:
                cur_alt = float(am.group(1))
                continue

            motm = motion_re.search(line)
            if motm:
                cur_om = (float(motm.group(1)), float(motm.group(2)), float(motm.group(3)))
                cur_ang = (float(motm.group(4)), float(motm.group(5)), float(motm.group(6)))
                continue

        if cur_time is not None:
            times.append(cur_time)
            vels.append(cur_v)
            alts.append(cur_alt)
            omegas.append(cur_om)
            angles.append(cur_ang)

    if len(times) == 0:
        return None

    return {
        "time": np.array(times),
        "V": np.array(vels),
        "altitude": np.array(alts),
        "omega": np.array(omegas),
        "angles": np.array(angles)
    }

def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """Compute 3D rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    rx = np.radians(roll_deg)
    ry = np.radians(pitch_deg)
    rz = np.radians(yaw_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx),  np.cos(rx)]
    ])

    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz),  np.cos(rz), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

def create_dots_polydata(base_dots, R_mat, dot_radius=0.0018):
    """Create a combined PolyData of small spheres for all 32 rotated surface dots."""
    rotated_dots = (R_mat @ base_dots.T).T
    spheres = []
    for pt in rotated_dots:
        s = pv.Sphere(radius=dot_radius, center=pt, theta_resolution=12, phi_resolution=12)
        spheres.append(s)
    
    combined = spheres[0]
    for s in spheres[1:]:
        combined = combined.merge(s)
    return combined

def render_animation(traj, dots_info, output_video, interactive=False, max_frames=200):
    times = traj["time"]
    n_total = len(times)
    
    # Subsample frames if trajectory is long
    step = max(1, n_total // max_frames)
    frame_indices = list(range(0, n_total, step))
    if frame_indices[-1] != n_total - 1:
        frame_indices.append(n_total - 1)

    print(f"Rendering {len(frame_indices)} frames for video...")

    ball_radius = dots_info["radius"]
    base_dots = np.array(dots_info["dots"])

    plotter = pv.Plotter(off_screen=not interactive, window_size=[1280, 720])
    plotter.set_background("#1e222a")

    # Central Steel Ball Sphere
    ball_mesh = pv.Sphere(radius=ball_radius, theta_resolution=48, phi_resolution=48)
    plotter.add_mesh(
        ball_mesh,
        color="#c8d6e5",
        specular=0.9,
        specular_power=30,
        smooth_shading=True,
        ambient=0.3,
        name="steel_ball"
    )

    # Orientation coordinate triad
    axes = pv.Axes(show_actor=True, actor_scale=0.04)
    plotter.add_actor(axes.actor)

    # Initial Dots
    R0 = euler_to_rotation_matrix(traj["angles"][0, 0], traj["angles"][0, 1], traj["angles"][0, 2])
    dots_mesh = create_dots_polydata(base_dots, R0, dot_radius=ball_radius * 0.08)
    plotter.add_mesh(
        dots_mesh,
        color="#e74c3c",  # High-contrast crimson golf dots
        specular=0.5,
        smooth_shading=True,
        name="golf_dots"
    )

    # Camera setup (isometric 3D view)
    plotter.camera_position = [
        (0.08, 0.06, 0.09),   # Camera position
        (0.0, 0.0, 0.0),       # Focal point
        (0.0, 1.0, 0.0)        # View up (Y is vertical)
    ]

    # Telemetry HUD Text
    hud_text = plotter.add_text(
        "Initializing 6DoF Telemetry...",
        position="upper_left",
        font_size=11,
        color="white"
    )

    if not interactive:
        plotter.open_movie(output_video, framerate=25, quality=9)

    for idx in frame_indices:
        t_curr = traj["time"][idx]
        v_curr = traj["V"][idx]
        alt_curr = traj["altitude"][idx]
        om_curr = traj["omega"][idx]
        ang_curr = traj["angles"][idx]
        rpm_curr = om_curr * 60.0 / (2.0 * np.pi)

        # Update rotated dots
        R_curr = euler_to_rotation_matrix(ang_curr[0], ang_curr[1], ang_curr[2])
        new_dots = create_dots_polydata(base_dots, R_curr, dot_radius=ball_radius * 0.08)
        plotter.add_mesh(new_dots, color="#e74c3c", specular=0.5, smooth_shading=True, name="golf_dots")

        # Update HUD text
        info_str = (
            f"=== steelBallRun 3D 6DoF Telemetry ===\n"
            f"Time:       {t_curr:7.3f} s\n"
            f"Fall Speed: {v_curr:7.3f} m/s\n"
            f"Altitude:   {alt_curr:7.3f} m\n"
            f"Spin (rad): ({om_curr[0]:6.1f}, {om_curr[1]:6.1f}, {om_curr[2]:6.1f}) rad/s\n"
            f"Spin (RPM): ({rpm_curr[0]:6.0f}, {rpm_curr[1]:6.0f}, {rpm_curr[2]:6.0f}) RPM\n"
            f"Attitude:   ({ang_curr[0]:6.1f}, {ang_curr[1]:6.1f}, {ang_curr[2]:6.1f}) deg"
        )
        hud_text.SetText(0, info_str)

        plotter.render()
        if not interactive:
            plotter.write_frame()

    if not interactive:
        plotter.close()
        print(f"Animation successfully written to {output_video}")
    else:
        plotter.show()

def main():
    case_dir = Path(__file__).resolve().parent
    logfile = case_dir / "log.freeBody6DoFPimpleFoam"
    dots_file = case_dir / "golf_dots.json"

    parser = argparse.ArgumentParser(description="3D PyVista visualizer for steelBallRun.")
    parser.add_argument("--video", type=str, default=str(case_dir / "steelBallRun.mp4"), help="Output MP4 path")
    parser.add_argument("--interactive", action="store_true", help="Launch interactive 3D GUI window")
    args = parser.parse_args()

    if not dots_file.exists():
        print(f"Error: {dots_file} not found. Run meshing/generate_sphere_stls.py first.")
        sys.exit(1)

    with open(dots_file, "r") as f:
        dots_info = json.load(f)

    traj = parse_trajectory(logfile)
    if not traj:
        print(f"No trajectory found in {logfile}. Synthesizing preview trajectory from initial conditions...")
        # Synthesize demonstration frames based on initial 6DoF state
        t_arr = np.linspace(0.0, 1.0, 100)
        v_arr = 9.81 * t_arr
        alt_arr = -0.5 * 9.81 * t_arr**2
        om_init = np.array([10.0, 5.0, 15.0])
        om_arr = np.tile(om_init, (len(t_arr), 1))
        angles_arr = np.outer(t_arr, np.degrees(om_init)) % 360.0
        traj = {
            "time": t_arr,
            "V": v_arr,
            "altitude": alt_arr,
            "omega": om_arr,
            "angles": angles_arr
        }

    render_animation(traj, dots_info, args.video, interactive=args.interactive)

if __name__ == "__main__":
    main()
