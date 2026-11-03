#!/usr/bin/env python3
"""
compute_steady_gci.py

Implements ASME V&V 20-2009 / Celik et al. (2008) Standard for Spatial Discretization Uncertainty:
Evaluates solutions across Coarse, Medium, Fine grids for steady broadside disk at Re = 100:
- Grid 1 (Fine)  : N1 = 368,760 cells
- Grid 2 (Medium): N2 = 212,792 cells
- Grid 3 (Coarse): N3 = 130,928 cells
"""
import os, sys
import numpy as np

MESH_DIR = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence"
D = 2.02e-3
RHO = 1.200
NU = 1.5e-5
RE = 100
V = RE * NU / D
A = np.pi * (D / 2.0)**2
Q = 0.5 * RHO * (V**2) * A

def load_case_cd(case_path):
    motion_path = os.path.join(case_path, "postProcessing", "fallingMotion", "motion.dat")
    if not os.path.exists(motion_path):
        raise FileNotFoundError(f"Missing {motion_path}")
    data = np.loadtxt(motion_path)
    t = data[:, 0]
    Fy = data[:, 27]
    # evaluate on last 30% of run
    mask = t >= t[-1] * 0.70
    Cd = Fy[mask] / Q
    Cd_mean = np.mean(Cd)
    drift = (Cd[-1] - Cd[0]) / (t[mask][-1] - t[mask][0]) if (t[mask][-1] - t[mask][0]) > 0 else 0
    return Cd_mean, drift, len(t[mask]), t[mask][0], t[mask][-1]

def run_gci():
    c3_path = os.path.join(MESH_DIR, "gci_coarse_Re100")
    c2_path = os.path.join(MESH_DIR, "gci_medium_Re100")
    c1_path = os.path.join(MESH_DIR, "gci_fine_Re100")

    cd3, drift3, n3, t3_0, t3_1 = load_case_cd(c3_path)
    cd2, drift2, n2, t2_0, t2_1 = load_case_cd(c2_path)
    cd1, drift1, n1, t1_0, t1_1 = load_case_cd(c1_path)

    N1 = 368760
    N2 = 212792
    N3 = 130928

    r21 = (N1 / N2)**(1.0 / 3.0)
    r32 = (N2 / N3)**(1.0 / 3.0)

    phi1 = cd1
    phi2 = cd2
    phi3 = cd3

    eps21 = phi2 - phi1
    eps32 = phi3 - phi2
    R = eps21 / eps32

    print("=" * 85)
    print("ASME V&V 20-2009 SPATIAL DISCRETIZATION UNCERTAINTY (GCI)")
    print("Fixed Broadside Disk at Re = 100 (Steady Axisymmetric Flow)")
    print("=" * 85)
    print(f"Grid 1 (Fine)   : N1 = {N1:,} cells | C_D1 = {phi1:.6f} | drift = {drift1:+.2e}/s (t={t1_0:.3f}..{t1_1:.3f}s)")
    print(f"Grid 2 (Medium) : N2 = {N2:,} cells | C_D2 = {phi2:.6f} | drift = {drift2:+.2e}/s (t={t2_0:.3f}..{t2_1:.3f}s)")
    print(f"Grid 3 (Coarse) : N3 = {N3:,} cells | C_D3 = {phi3:.6f} | drift = {drift3:+.2e}/s (t={t3_0:.3f}..{t3_1:.3f}s)")
    print(f"Refinement ratios: r21 = {r21:.4f}, r32 = {r32:.4f}")
    print("-" * 85)
    print(f"Differences: eps21 = {eps21:+.6e}, eps32 = {eps32:+.6e}")
    print(f"Convergence ratio R = eps21 / eps32 = {R:+.4f}")

    if 0 < R < 1:
        regime = "Monotonic Convergence"
    elif -1 < R < 0:
        regime = "Oscillatory Convergence"
    elif R > 1:
        regime = "Monotonic Divergence"
    else:
        regime = "Oscillatory Divergence"
    print(f"Convergence Regime: {regime}")

    # Solve for apparent order p
    if 0 < R < 1:
        # Fixed point iteration for transcendental equation
        p = abs(np.log(abs(eps32 / eps21)) / np.log(r21))
        for _ in range(50):
            q_p = np.log((r21**p - np.sign(eps32/eps21)) / (r32**p - np.sign(eps32/eps21)))
            p_new = (1.0 / np.log(r21)) * abs(np.log(abs(eps32 / eps21)) + q_p)
            if abs(p_new - p) < 1e-6:
                break
            p = 0.5 * (p + p_new)

        phi_ext_21 = (r21**p * phi1 - phi2) / (r21**p - 1.0)
        e_a_21 = abs(eps21 / phi1)
        e_ext_21 = abs((phi_ext_21 - phi1) / phi_ext_21)
        
        # GCI with Fs = 1.25
        GCI_fine_21 = (1.25 * e_a_21) / (r21**p - 1.0)
        GCI_med_32  = (1.25 * abs(eps32 / phi2)) / (r32**p - 1.0)
        u_num = GCI_fine_21 * phi1

        print(f"Apparent spatial order p = {p:.4f}")
        print(f"Richardson Extrapolated C_D (h -> 0) = {phi_ext_21:.6f}")
        print(f"Approximate relative error e_a,21   = {e_a_21*100:.3f}%")
        print(f"Extrapolated relative error e_ext,21 = {e_ext_21*100:.3f}%")
        print(f"GCI_fine (Grid 1)                   = {GCI_fine_21*100:.3f}%")
        print(f"ASME V&V 20 Numerical Uncertainty u_num = {u_num:.6f} ({GCI_fine_21*100:.2f}% of C_D)")
    else:
        print("Note: Grid family is not in monotonic convergence regime; computing bounds from grid span.")
        span = abs(phi1 - phi2)
        print(f"Grid span |phi1 - phi2| = {span:.6f} ({span/phi1*100:.2f}%)")

    print("=" * 85)

if __name__ == "__main__":
    run_gci()
