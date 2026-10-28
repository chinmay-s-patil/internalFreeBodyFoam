#!/usr/bin/env python3
"""
compute_gci.py

Implements ASME V&V 20-2009 / Celik et al. (2008) Standard for Spatial Discretization Uncertainty:
Evaluates solutions across Coarse, Medium, Fine grids without artificial clamping.
Properly classifies convergence into:
- Monotonic convergence (0 < R < 1)
- Monotonic divergence (R > 1)
- Oscillatory convergence (-1 < R < 0)
- Oscillatory divergence (R < -1)
"""

import os
import numpy as np

def parse_case(case_dir):
    fall_file = os.path.join(case_dir, "postProcessing", "fallingMotion", "motion.dat")
    tumble_file = os.path.join(case_dir, "postProcessing", "tumbleMotion", "motion.dat")
    
    fall_lines = []
    with open(fall_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                fall_lines.append(line.split())
                
    tumble_lines = []
    with open(tumble_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                tumble_lines.append(line.split())
                
    last_fall = [float(x) for x in fall_lines[-1]]
    last_tumble = [float(x) for x in tumble_lines[-1]]
    
    t = last_fall[0]
    Vy = abs(last_fall[3])
    Faero_y = abs(last_fall[27])
    Faero_mag = np.sqrt(last_fall[26]**2 + last_fall[27]**2 + last_fall[28]**2)
    Omega_x = last_tumble[1]
    Moment_x = last_tumble[7]
    
    return {
        'time': t,
        'Vy': Vy,
        'Faero_y': Faero_y,
        'Faero_mag': Faero_mag,
        'Omega_x': Omega_x,
        'Moment_x': Moment_x
    }

def run_asme_vv20():
    base_dir = os.path.dirname(os.path.realpath(__file__))
    
    N1 = 368760 # Fine (Grid 1)
    N2 = 212792 # Medium (Grid 2)
    N3 = 130928 # Coarse (Grid 3)
    
    r21 = (N1 / N2)**(1.0 / 3.0)
    r32 = (N2 / N3)**(1.0 / 3.0)
    r_avg = 0.5 * (r21 + r32)
    
    c1 = parse_case(os.path.join(base_dir, "fineMesh", "case"))
    c2 = parse_case(os.path.join(base_dir, "mediumMesh", "case"))
    c3 = parse_case(os.path.join(base_dir, "coarseMesh", "case"))
    
    print("=" * 85)
    print("ASME V&V 20-2009 SPATIAL DISCRETIZATION ANALYSIS (Early-Time Diagnostic at t = 0.002s)")
    print("=" * 85)
    print(f"Grid 1 (Fine)   : N1 = {N1:,} cells | rotating = 300,568 | static = 68,192")
    print(f"Grid 2 (Medium) : N2 = {N2:,} cells | rotating = 178,160 | static = 34,632 | r21 = {r21:.4f}")
    print(f"Grid 3 (Coarse) : N3 = {N3:,} cells | rotating =  96,344 | static = 34,584 | r32 = {r32:.4f}")
    print(f"Evaluation Time : t = {c1['time']:.6f} s (Startup transient: V_y ~ 0.0196 m/s)")
    print("-" * 85)
    
    qois = [
        ("Aerodynamic Drag Force Faero_y (nN)", c1['Faero_y']*1e9, c2['Faero_y']*1e9, c3['Faero_y']*1e9),
        ("Total Resultant Force Faero_mag (nN)", c1['Faero_mag']*1e9, c2['Faero_mag']*1e9, c3['Faero_mag']*1e9),
        ("Descent Velocity Vy (mm/s)", c1['Vy']*1e3, c2['Vy']*1e3, c3['Vy']*1e3),
        ("Aerodynamic Pitch Moment Mx (1e-16 N.m)", c1['Moment_x']*1e16, c2['Moment_x']*1e16, c3['Moment_x']*1e16)
    ]
    
    for name, phi1, phi2, phi3 in qois:
        eps21 = phi1 - phi2
        eps32 = phi2 - phi3
        
        # Convergence ratio R
        if abs(eps32) > 1e-15:
            R = eps21 / eps32
        else:
            R = 0.0
            
        print(f"\nTarget QOI: {name}")
        print(f"  Coarse (Grid 3)  phi3 = {phi3:.6f}")
        print(f"  Medium (Grid 2)  phi2 = {phi2:.6f}")
        print(f"  Fine   (Grid 1)  phi1 = {phi1:.6f}")
        print(f"  Differences: eps21 = {eps21:+.6e}, eps32 = {eps32:+.6e}")
        print(f"  Convergence Ratio R = eps21 / eps32 = {R:+.4f}")
        
        if 0 < R < 1:
            # Monotonic convergence
            p = np.log(1.0 / R) / np.log(r_avg)
            phi_ext21 = (r21**p * phi1 - phi2) / (r21**p - 1.0)
            ea21 = abs(eps21 / phi1)
            e_ext21 = abs((phi_ext21 - phi1) / phi_ext21)
            Fs = 1.25
            GCI21_fine = (Fs * ea21) / (r21**p - 1.0)
            GCI32_med = (Fs * abs(eps32 / phi2)) / (r32**p - 1.0)
            asymp_ratio = GCI32_med / (r21**p * GCI21_fine) if GCI21_fine > 0 else 1.0
            
            print(f"  Regime: MONOTONIC CONVERGENCE")
            print(f"  Unclamped Apparent Order p = {p:.3f}")
            print(f"  Observed Grid Sensitivity (Fine vs Medium): e_a = {ea21*100:.3f}%")
            if p <= 4.0:
                print(f"  Richardson Extrapolated Value = {phi_ext21:.6f}")
                print(f"  Fine Grid GCI21 = {GCI21_fine*100:.3f}%")
                print(f"  Asymptotic Range Index AR = {asymp_ratio:.3f}")
            else:
                print(f"  Formal GCI = Undetermined")
                print(f"  [Note] Observed order p = {p:.2f} far exceeds formal 2nd-order scheme accuracy.")
                print(f"         Solutions lie outside the asymptotic convergence range; GCI is not defensible.")
        elif -1 < R < 0:
            # Oscillatory convergence
            print(f"  Regime: OSCILLATORY CONVERGENCE (differences change sign between refinements)")
            half_range = 0.5 * abs(phi1 - phi3)
            rel_half_range = half_range / abs(phi1) if abs(phi1) > 1e-15 else 0.0
            print(f"  Observed Half-Range Indicator: Delta_half = 0.5 * |phi1 - phi3| = {half_range:.6e} ({rel_half_range*100:.3f}%)")
            print(f"  [Note] Delta_half indicates observed spread across the three grids, not a formal GCI or guaranteed error bound.")
        elif R > 1:
            print(f"  Regime: MONOTONIC DIVERGENCE (mesh refinement increases error)")
        else:
            print(f"  Regime: OSCILLATORY DIVERGENCE")
        print("-" * 85)

if __name__ == "__main__":
    run_asme_vv20()
