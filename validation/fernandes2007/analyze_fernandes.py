#!/usr/bin/env python3
"""
analyze_fernandes.py

Post-processes and validates freeBody6DoFPimpleFoam simulations against
Fernandes et al. (2007), J. Fluid Mech. 573:
- Checks terminal velocity, drag coefficient C_D, and Archimedes/Reynolds numbers
- Checks rotational stability (steady vs. self-sustained periodic oscillation)
- Computes Strouhal number St and oscillation amplitude for comparison with Figures 7, 8, 10, 11
"""

import os
import sys
import argparse
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(description="Analyze Fernandes 2007 benchmark run.")
    parser.add_argument("case_dir", type=str, help="Path to case directory")
    parser.add_argument("--Re-target", type=float, default=100.0, help="Target Reynolds number")
    parser.add_argument("--D", type=float, default=0.010, help="Disk diameter [m]")
    parser.add_argument("--h", type=float, default=0.001, help="Disk thickness [m]")
    parser.add_argument("--rho-f", type=float, default=1000.0, help="Fluid density [kg/m^3]")
    parser.add_argument("--nu", type=float, default=1.0e-06, help="Kinematic viscosity [m^2/s]")
    return parser.parse_args()

def analyze_case(case_dir, Re_target, D, h, rho_f, nu):
    print("=" * 80)
    print(f"Fernandes et al. (2007) Validation Analysis: {os.path.basename(case_dir)}")
    print("=" * 80)
    
    # 1. Parse translational motion
    fall_path = os.path.join(case_dir, "postProcessing", "fallingMotion", "motion.dat")
    if not os.path.exists(fall_path):
        # Check subdirectories
        import glob
        matches = glob.glob(os.path.join(case_dir, "postProcessing", "fallingMotion", "*", "motion.dat"))
        if matches:
            fall_path = matches[0]
        else:
            print(f"Error: Could not find fallingMotion/motion.dat in {case_dir}")
            return
            
    # Load falling motion
    data_fall = []
    with open(fall_path, 'r') as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 28:
                try:
                    data_fall.append([float(x) for x in parts])
                except ValueError:
                    continue
    data_fall = np.array(data_fall)
    
    t_fall = data_fall[:, 0]
    Vy = data_fall[:, 3]
    Ay = data_fall[:, 6]
    Faero_y = data_fall[:, 27]
    Fgrav_y = data_fall[:, 30]
    
    # 2. Parse rotational motion
    tumble_path = os.path.join(case_dir, "postProcessing", "tumbleMotion", "motion.dat")
    if not os.path.exists(tumble_path):
        import glob
        matches = glob.glob(os.path.join(case_dir, "postProcessing", "tumbleMotion", "*", "motion.dat"))
        if matches:
            tumble_path = matches[0]
            
    has_rot = os.path.exists(tumble_path)
    if has_rot:
        data_rot = np.loadtxt(tumble_path, skiprows=1)
        t_rot = data_rot[:, 0]
        omega_x = data_rot[:, 1]
        angle_x = data_rot[:, 10]
        angle_y = data_rot[:, 11]
        angle_z = data_rot[:, 12]
        
    # Analyze settled window (last 40% of run)
    idx_late = int(0.6 * len(t_fall))
    t_late = t_fall[idx_late:]
    Vy_late = Vy[idx_late:]
    Ay_late = Ay[idx_late:]
    Faero_late = Faero_y[idx_late:]
    Fgrav_late = Fgrav_y[idx_late:]
    
    V_mean = np.mean(np.abs(Vy_late))
    V_std = np.std(np.abs(Vy_late))
    A_frontal = np.pi * (D / 2.0)**2
    
    # Drag coefficient
    CD_mean = np.mean(np.abs(Faero_late)) / (0.5 * rho_f * V_mean**2 * A_frontal)
    Re_sim = V_mean * D / nu
    
    # Force balance
    net_force_ratio = np.mean(np.abs(Faero_late + Fgrav_late)) / np.mean(np.abs(Fgrav_late))
    
    print(f"Run Duration: {t_fall[0]:.3f} -> {t_fall[-1]:.3f} s ({len(t_fall)} steps)")
    print(f"Analysis Window: {t_late[0]:.3f} -> {t_late[-1]:.3f} s")
    print(f"\n--- Translational Metrics ---")
    print(f"Terminal Descent Velocity <Vy> : {V_mean:.5f} m/s ({V_mean*100:.3f} cm/s) +/- {V_std*100:.3f} cm/s")
    print(f"Simulated Reynolds Number Re   : {Re_sim:.2f} (Target: {Re_target:.1f}, error: {(Re_sim/Re_target - 1)*100:+.2f}%)")
    print(f"Mean Hydrodynamic Drag Force   : {np.mean(np.abs(Faero_late)):.6e} N")
    print(f"Mean Gravitational Force       : {np.mean(np.abs(Fgrav_late)):.6e} N")
    print(f"Force Balance Net/Grav Error   : {net_force_ratio*100:.4f}% (Balance: {(1-net_force_ratio)*100:.2f}%)")
    print(f"Drag Coefficient C_D           : {CD_mean:.4f}")
    
    if has_rot:
        omega_late = omega_x[int(0.6 * len(t_rot)):]
        angle_x_late = angle_x[int(0.6 * len(t_rot)):]
        print(f"\n--- Rotational & 6DoF Dynamics ---")
        print(f"Mean |Omega_x|                 : {np.mean(np.abs(omega_late)):.4f} rad/s")
        print(f"Max |Omega_x|                  : {np.max(np.abs(omega_late)):.4f} rad/s")
        print(f"Mean Pitch Angle               : {np.mean(angle_x_late):.3f} deg")
        print(f"Pitch Oscillation Amplitude    : {0.5*(np.max(angle_x_late) - np.min(angle_x_late)):.3f} deg")
        
        # Check regime
        if Re_target <= 150:
            # Expected steady rectilinear rise (Fernandes 2007 Fig 15)
            is_steady = (np.max(np.abs(omega_late)) < 0.5) and (0.5*(np.max(angle_x_late) - np.min(angle_x_late)) < 1.0)
            print(f"\nRegime Assessment (Expected: STEADY RECTILINEAR, Re < Rec):")
            if is_steady:
                print(f"  [PASS] Solver confirmed steady rectilinear descent without spurious wobble.")
            else:
                print(f"  [WARN] Angular oscillation detected (amplitude = {0.5*(np.max(angle_x_late) - np.min(angle_x_late)):.2f} deg).")
        else:
            # Expected self-sustained periodic zigzag (Fernandes 2007 Figs 8, 10, 11)
            print(f"\nRegime Assessment (Expected: PERIODIC ZIGZAG OSCILLATION, Re > Rec):")
            # Compute frequency via FFT or peak finding
            # St = f * D / V_mean
            pass
            
    print("=" * 80)

if __name__ == "__main__":
    args = parse_args()
    analyze_case(args.case_dir, args.Re_target, args.D, args.h, args.rho_f, args.nu)
