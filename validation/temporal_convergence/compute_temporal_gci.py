#!/usr/bin/env python3
"""
compute_temporal_gci.py

Implements ASME V&V 20-2009 Standard for Temporal Discretization Uncertainty:
Evaluates solutions across Coarse, Medium, Fine time steps (dt = 2e-5, 1e-5, 5e-6 s) with r_t = 2.0.
Verifies exact physical time matching at t = 0.001000 s via boundary interpolation.
"""

import os
import numpy as np

def parse_case_at_time(case_dir, target_time=0.001000):
    fall_file = os.path.join(case_dir, "postProcessing", "fallingMotion", "motion.dat")
    tumble_file = os.path.join(case_dir, "postProcessing", "tumbleMotion", "motion.dat")
    
    fall_arr = np.loadtxt(fall_file, comments='#')
    tumble_arr = np.loadtxt(tumble_file, comments='#')
    
    t_f = fall_arr[:, 0]
    t_t = tumble_arr[:, 0]
    
    if target_time > t_f[-1] + 1e-9 or target_time > t_t[-1] + 1e-9:
        raise ValueError(f"Target time {target_time} exceeds simulation span (fall={t_f[-1]}, tumble={t_t[-1]})")
        
    Vy = abs(np.interp(target_time, t_f, fall_arr[:, 3]))
    Faero_x = np.interp(target_time, t_f, fall_arr[:, 26])
    Faero_y = abs(np.interp(target_time, t_f, fall_arr[:, 27]))
    Faero_z = np.interp(target_time, t_f, fall_arr[:, 28])
    Faero_mag = np.sqrt(Faero_x**2 + Faero_y**2 + Faero_z**2)
    
    Omega_x = np.interp(target_time, t_t, tumble_arr[:, 1])
    Moment_x = np.interp(target_time, t_t, tumble_arr[:, 7])
    
    return {
        'time': target_time,
        'Vy': Vy,
        'Faero_y': Faero_y,
        'Faero_mag': Faero_mag,
        'Omega_x': Omega_x,
        'Moment_x': Moment_x
    }

def run_temporal_asme():
    base_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/temporal_convergence"
    target_time = 0.001000
    
    dt1 = 5.0e-6 # Quarter (Fine dt, Co = 0.25)
    dt2 = 1.0e-5 # Halved (Medium dt, Co = 0.50)
    dt3 = 2.0e-5 # Baseline (Coarse dt, Co = 1.00)
    
    r21 = dt2 / dt1 # 2.0
    r32 = dt3 / dt2 # 2.0
    r_avg = 2.0
    
    c1 = parse_case_at_time(os.path.join(base_dir, "dt_quarter"), target_time)
    c2 = parse_case_at_time(os.path.join(base_dir, "dt_halved"), target_time)
    c3 = parse_case_at_time(os.path.join(base_dir, "dt_baseline"), target_time)
    
    print("=" * 85)
    print("ASME V&V 20-2009 TEMPORAL DISCRETIZATION ANALYSIS (Early-Time Diagnostic at t = 0.001s)")
    print("=" * 85)
    print(f"Verified Evaluation Time : t = {target_time:.6f} s (Exact boundary interpolation across all cases)")
    print(f"Time Step 1 (Fine)       : dt1 = {dt1:.2e} s (Co = 0.25, N_steps = 200)")
    print(f"Time Step 2 (Medium)     : dt2 = {dt2:.2e} s (Co = 0.50, N_steps = 100) | r21 = {r21:.2f}")
    print(f"Time Step 3 (Coarse)     : dt3 = {dt3:.2e} s (Co = 1.00, N_steps =  50) | r32 = {r32:.2f}")
    print(f"Flow State               : Early descent startup (V_y ~ 0.0098 m/s, unseparated shear)")
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
        
        if abs(eps32) > 1e-15:
            R = eps21 / eps32
        else:
            R = 0.0
            
        print(f"\nTarget QOI: {name}")
        print(f"  Coarse dt (Grid 3)  phi3 = {phi3:.6f}")
        print(f"  Medium dt (Grid 2)  phi2 = {phi2:.6f}")
        print(f"  Fine dt   (Grid 1)  phi1 = {phi1:.6f}")
        print(f"  Differences: eps21 = {eps21:+.6e}, eps32 = {eps32:+.6e}")
        print(f"  Convergence Ratio R = eps21 / eps32 = {R:+.4f}")
        
        if 0 < R < 1:
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
            print(f"  Richardson Extrapolated Value = {phi_ext21:.6f}")
            print(f"  Approximate Relative Error e_a = {ea21*100:.3f}%")
            print(f"  Fine dt GCI21 = {GCI21_fine*100:.3f}%")
            print(f"  Medium dt GCI32 = {GCI32_med*100:.3f}%")
            print(f"  Asymptotic Range Index AR = {asymp_ratio:.3f} (Ideal = 1.000)")
            if p > 3.0:
                print(f"  [Note] Observed order p = {p:.2f} > formal scheme order; solutions outside asymptotic range.")
        elif -1 < R < 0:
            half_range = 0.5 * abs(phi1 - phi3)
            rel_half_range = half_range / abs(phi1) if abs(phi1) > 1e-15 else 0.0
            print(f"  Observed Half-Range Indicator: Delta_half = 0.5 * |phi1 - phi3| = {half_range:.6e} ({rel_half_range*100:.3f}%)")
            print(f"  [Note] Delta_half indicates observed time-step spread, not a formal GCI or guaranteed error bound.")
        elif R > 1:
            print(f"  Regime: MONOTONIC DIVERGENCE (time-step refinement increases error)")
        else:
            print(f"  Regime: OSCILLATORY DIVERGENCE")
        print("-" * 85)

if __name__ == "__main__":
    run_temporal_asme()
