#!/usr/bin/env python3
"""
setup_3d_perturbations.py

Sets up dynamic 3D-perturbation tumbling simulation cases (Roadmap Task 4).

Coordinate Frame & Physical Architecture:
- Mesh & Body Frame:
  * Body Y is the disk symmetry axis (face normal, h = 50 um), with polar inertia:
      I_yy = 0.5 * m * R^2 = 1.1278e-13 kg*m^2
  * Body X and Body Z are the orthogonal diametral axes in the disk plane, with inertia:
      I_xx = I_zz = 0.25 * m * R^2 + (1/12) * m * h^2 = 5.6436e-14 kg*m^2
- Frame Check at t = 0.30 s:
  * In the solver (`correctRotorDynamics.H`), `angularVelocity` in `0.3/uniform/` is `omegaB`,
    the angular velocity expressed in the body principal frame.
  * The baseline limit cycle is pure pitch autorotation about Body X (Omega_x ≈ -161.4 rad/s at t=0.30 s).
  * At t = 0.30 s, Euler angles are (76.26 deg, 0.17 deg, 0.02 deg), confirming that Body X
    is co-linear with World X (within 0.1 deg).
  * Consequently:
    - Omega_y is spin about the disk's own symmetry axis (coin spin). For an axisymmetric disk,
      this does not directly change the flow angle of attack and acts primarily through gyroscopic coupling.
    - Omega_z tilts the pitch axis out of plane, which tilts the disk face relative to the descent path.
      This is the primary physical perturbation that breaks planar symmetry and permits 3D tumbling/flutter.

Perturbation Cases Configured:
1. `restart_pert5pct` (Recommended Priority Run):
   * Perturbation: 5% tilt perturbation on Omega_z (pitch axis tilt).
   * Initial rate: Omega_b = (-161.3947, 0.0, +8.0697) rad/s.
   * Tilts the angular momentum vector H by beta = arctan(I_zz * Omega_z / (I_xx * |Omega_x|)) ≈ 2.86 deg.
   * Simulation window: t in [0.30, 0.58] s (duration 0.28 s >= 8.5 complete tumbling revolutions).
2. `restart_pert10pct`:
   * Perturbation: 10% tilt perturbation on Omega_z (stronger pitch axis tilt).
   * Initial rate: Omega_b = (-161.3947, 0.0, +16.1395) rad/s.
   * Tilts angular momentum vector H by beta ≈ 5.71 deg.
   * Simulation window: t in [0.30, 0.58] s (duration 0.28 s >= 8.5 complete tumbling revolutions).
3. `restart_pert5pct_y`:
   * Perturbation: 5% axial coin spin on Omega_y (spin axis).
   * Initial rate: Omega_b = (-161.3947, +8.0697, 0.0) rad/s.
   * Tilts angular momentum vector H by beta = arctan(I_yy * Omega_y / (I_xx * |Omega_x|)) = arctan(2 * 0.05) ≈ 5.71 deg via gyroscopic coupling (I_yy ≈ 2*I_xx).
   * Simulation window: t in [0.30, 0.58] s (duration 0.28 s >= 8.5 complete tumbling revolutions).

Key Observable Metrics & Analysis Protocol:
- Out-of-Plane Angle History:
  * The disk face normal in world coordinates is n_w = R(q) * (0, 1, 0)^T.
  * Out-of-plane tilt angle: theta_out(t) = arcsin(|n_w,x(t)|).
  * Out-of-plane angular velocity: Omega_out / ||Omega|| = sqrt(Omega_y^2 + Omega_z^2) / ||Omega||.
  * Tracking theta_out(t) reveals whether the 3D perturbation grows, persists, or damps
    exponentially back to the planar limit cycle.
- Trajectory Inclination:
  * Projected 2D trajectory angle: phi = arctan(v_x / |v_y|) (comparable to experimental single-plane imaging).
  * 3D trajectory angle: psi = arctan(sqrt(v_x^2 + v_z^2) / |v_y|).
- Equilibrium Settling Velocity: V_eq,est evaluated over the final >= 3 complete revolutions.
"""

import os
import shutil
import subprocess
import numpy as np

def create_case(case_name, omega_pert):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    src_case = os.path.join(base_dir, "restart_base")
    dest_case = os.path.join(base_dir, case_name)

    print(f"Setting up 3D perturbation case: {dest_case}")
    if os.path.exists(dest_case):
        shutil.rmtree(dest_case)

    os.makedirs(os.path.join(dest_case, "constant"), exist_ok=True)

    # Copy system, Allrun, Allclean
    shutil.copytree(os.path.join(src_case, "system"), os.path.join(dest_case, "system"))
    shutil.copy(os.path.join(src_case, "Allrun"), dest_case)
    shutil.copy(os.path.join(src_case, "Allclean"), dest_case)

    # Copy constant files
    for f in ["fallingFrameDict", "dynamicMeshDict", "transportProperties", "turbulenceProperties"]:
        shutil.copy(os.path.join(src_case, "constant", f), os.path.join(dest_case, "constant", f))

    # Symlink polyMesh
    os.symlink(os.path.join(src_case, "constant", "polyMesh"), os.path.join(dest_case, "constant", "polyMesh"))

    # Copy restart time directory (0.3)
    shutil.copytree(os.path.join(src_case, "0.3"), os.path.join(dest_case, "0.3"))

    # Modify 0.3/uniform/angularVelocity
    ang_vel_path = os.path.join(dest_case, "0.3", "uniform", "angularVelocity")
    with open(ang_vel_path, "r") as f:
        content = f.read()

    lines = content.splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith("value"):
            lines[i] = f"value           ({omega_pert[0]:.7f} {omega_pert[1]:.7f} {omega_pert[2]:.7f}); // 3D perturbation"
            break
    content = "\n".join(lines) + "\n"

    with open(ang_vel_path, "w") as f:
        f.write(content)

    # Set endTime to 0.58 s (>= 8.5 complete revolutions)
    control_dict_path = os.path.join(dest_case, "system", "controlDict")
    subprocess.run(["foamDictionary", "-entry", "endTime", "-set", "0.58", control_dict_path], check=True)

    print(f"  Initialized body angular velocity: Omega_b = ({omega_pert[0]:.3f}, {omega_pert[1]:.3f}, {omega_pert[2]:.3f}) rad/s")
    print(f"  Simulation window: t in [0.30, 0.58] s (>= 8.5 revs)")
    print(f"  Ready at: {dest_case}\n")


def analyze_out_of_plane(case_dir):
    """
    Parses motion.dat and logs the out-of-plane tilt angle theta_out(t)
    and angular rate ratio Omega_out / ||Omega||.
    """
    rot_file = os.path.join(case_dir, "postProcessing", "tumbleMotion", "motion.dat")
    trans_file = os.path.join(case_dir, "postProcessing", "fallingMotion", "motion.dat")
    if not (os.path.exists(rot_file) and os.path.exists(trans_file)):
        print(f"No completed telemetry found in {case_dir}")
        return None

    r = np.loadtxt(rot_file, comments="#")
    f = np.loadtxt(trans_file)
    n = min(len(r), len(f))
    r, f = r[:n], f[:n]
    t = r[:, 0]

    # Quaternions: w, x, y, z (cols 13, 14, 15, 16)
    qw, qx, qy, qz = r[:, 13], r[:, 14], r[:, 15], r[:, 16]

    # Disk face normal in body frame is n_b = (0, 1, 0)
    # Rotation matrix column 1 gives n_w = R * (0, 1, 0)^T:
    # R[0, 1] = 2*(qx*qy - qw*qz)
    # R[1, 1] = 1 - 2*(qx^2 + qz^2)
    # R[2, 1] = 2*(qy*qz + qw*qx)
    nw_x = 2.0 * (qx * qy - qw * qz)
    theta_out_deg = np.degrees(np.arcsin(np.clip(np.abs(nw_x), 0.0, 1.0)))

    # Angular rate out-of-plane:
    omega_x, omega_y, omega_z = r[:, 1], r[:, 2], r[:, 3]
    omega_mag = np.sqrt(omega_x**2 + omega_y**2 + omega_z**2)
    omega_out_ratio = np.sqrt(omega_y**2 + omega_z**2) / np.maximum(omega_mag, 1e-6)

    # 2D projected trajectory angle phi vs 3D psi:
    vx, vy, vz = f[:, 2], np.abs(f[:, 3]), f[:, 4]
    phi_deg = np.degrees(np.arctan(np.abs(vx) / vy))
    psi_deg = np.degrees(np.arctan(np.hypot(vx, vz) / vy))

    print(f"=== Out-of-Plane Kinematics Summary for {os.path.basename(case_dir)} ===")
    print(f"  Time range: t = {t[0]:.4f} to {t[-1]:.4f} s ({len(t)} steps)")
    print(f"  Initial theta_out: {theta_out_deg[0]:.2f} deg  -->  Final: {theta_out_deg[-1]:.2f} deg")
    print(f"  Peak theta_out: {np.max(theta_out_deg):.2f} deg  |  Mean: {np.mean(theta_out_deg):.2f} deg")
    print(f"  Initial Omega_out/||Omega||: {omega_out_ratio[0]*100:.2f}%  -->  Final: {omega_out_ratio[-1]*100:.2f}%")
    print(f"  Projected 2D trajectory phi: {np.mean(phi_deg):.2f} deg  |  Full 3D psi: {np.mean(psi_deg):.2f} deg")
    
    return {
        "t": t, "theta_out_deg": theta_out_deg, "omega_out_ratio": omega_out_ratio,
        "phi_deg": phi_deg, "psi_deg": psi_deg
    }


def setup_all():
    base_omega_x = -161.3947111
    
    # 1. Realization A: 5% tilt perturbation on Omega_z (Recommended priority run)
    pert_5pct_z = (base_omega_x, 0.0, abs(base_omega_x) * 0.05)
    create_case("restart_pert5pct", pert_5pct_z)

    # 2. Realization B: 10% tilt perturbation on Omega_z
    pert_10pct_z = (base_omega_x, 0.0, abs(base_omega_x) * 0.10)
    create_case("restart_pert10pct", pert_10pct_z)

    # 3. Realization C: 5% spin perturbation on Omega_y (axial coin spin)
    pert_5pct_y = (base_omega_x, abs(base_omega_x) * 0.05, 0.0)
    create_case("restart_pert5pct_y", pert_5pct_y)

    print("=" * 75)
    print("3D Perturbation Cases Successfully Configured!")
    print("Priority 1 (Recommended): cd validation/baseline_drag/restart_pert5pct && ./Allrun")
    print("Priority 2 (10% Tilt):     cd validation/baseline_drag/restart_pert10pct && ./Allrun")
    print("Priority 3 (5% Coin Spin): cd validation/baseline_drag/restart_pert5pct_y && ./Allrun")
    print("To evaluate out-of-plane history: python3 setup_3d_perturbations.py --analyze <case_dir>")
    print("=" * 75)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2 and sys.argv[1] == "--analyze":
        analyze_out_of_plane(sys.argv[2])
    else:
        setup_all()
