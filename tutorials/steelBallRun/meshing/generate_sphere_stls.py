#!/usr/bin/env python3
"""
generate_sphere_stls.py

Generates high-precision conformal STLs and visual markers for steelBallRun:
1. Sphere (ball): Radius R = 25.0 mm (D = 50.0 mm)
2. Spherical AMI master interface: Radius R = 45.0 mm
3. Matching spherical cavity slave interface: Radius R = 45.0 mm
4. 32 Golf-ball visual surface marker coordinates (Fibonacci spherical lattice)
"""

import json
import os
from pathlib import Path
import numpy as np

# Physical Dimensions [m]
R_BALL = 0.025      # 25.0 mm radius (50 mm diameter)
R_AMI = 0.045       # 45.0 mm radius AMI interface
N_DOTS = 32         # 32 visual golf-ball dots for rotation tracking

def create_icosphere(radius, subdivisions=3):
    """Generate equilateral icosphere vertices and faces."""
    phi = (1.0 + np.sqrt(5.0)) / 2.0
    
    verts = [
        [-1,  phi, 0], [ 1,  phi, 0], [-1, -phi, 0], [ 1, -phi, 0],
        [ 0, -1,  phi], [ 0,  1,  phi], [ 0, -1, -phi], [ 0,  1, -phi],
        [ phi, 0, -1], [ phi, 0,  1], [-phi, 0, -1], [-phi, 0,  1]
    ]
    verts = [v / np.linalg.norm(v) for v in verts]
    
    faces = [
        [0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11],
        [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8],
        [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9],
        [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1]
    ]
    
    midpoint_cache = {}
    def get_midpoint(i1, i2):
        edge = tuple(sorted((i1, i2)))
        if edge in midpoint_cache:
            return midpoint_cache[edge]
        v1 = verts[i1]
        v2 = verts[i2]
        mid = (v1 + v2) / 2.0
        mid = mid / np.linalg.norm(mid)
        verts.append(mid)
        idx = len(verts) - 1
        midpoint_cache[edge] = idx
        return idx
    
    for _ in range(subdivisions):
        new_faces = []
        for tri in faces:
            m01 = get_midpoint(tri[0], tri[1])
            m12 = get_midpoint(tri[1], tri[2])
            m20 = get_midpoint(tri[2], tri[0])
            new_faces.append([tri[0], m01, m20])
            new_faces.append([tri[1], m12, m01])
            new_faces.append([tri[2], m20, m12])
            new_faces.append([m01, m12, m20])
        faces = new_faces
        
    verts = np.array(verts) * radius
    return verts, np.array(faces)

def write_ascii_stl(filepath, verts, faces, solid_name):
    """Write vertices and faces to an ASCII STL file."""
    with open(filepath, "w") as f:
        f.write(f"solid {solid_name}\n")
        for face in faces:
            v0 = verts[face[0]]
            v1 = verts[face[1]]
            v2 = verts[face[2]]
            
            normal = np.cross(v1 - v0, v2 - v0)
            n_norm = np.linalg.norm(normal)
            if n_norm > 1e-14:
                normal /= n_norm
            else:
                normal = np.array([0.0, 0.0, 0.0])
                
            f.write(f"  facet normal {normal[0]:.6e} {normal[1]:.6e} {normal[2]:.6e}\n")
            f.write("    outer loop\n")
            f.write(f"      vertex {v0[0]:.6e} {v0[1]:.6e} {v0[2]:.6e}\n")
            f.write(f"      vertex {v1[0]:.6e} {v1[1]:.6e} {v1[2]:.6e}\n")
            f.write(f"      vertex {v2[0]:.6e} {v2[1]:.6e} {v2[2]:.6e}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {solid_name}\n")

def generate_fibonacci_dots(n_dots, radius):
    """Generate n_dots uniformly distributed on sphere surface via Fibonacci lattice."""
    dots = []
    golden_ratio = (1.0 + np.sqrt(5.0)) / 2.0
    golden_angle = 2.0 * np.pi * (1.0 - 1.0 / golden_ratio)
    
    for i in range(n_dots):
        z = 1.0 - (2.0 * i + 1.0) / n_dots
        r = np.sqrt(max(0.0, 1.0 - z * z))
        theta = golden_angle * i
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        dots.append([float(x * radius), float(y * radius), float(z * radius)])
        
    return dots

def main():
    meshing_dir = Path(__file__).resolve().parent
    tri_dir = meshing_dir / "triSurface"
    tri_dir.mkdir(parents=True, exist_ok=True)
    
    case_dir = meshing_dir.parent
    
    print(f"Generating 3D sphere geometry (R = {R_BALL*1000:.1f} mm, D = {R_BALL*2000:.1f} mm)...")
    sphere_verts, sphere_faces = create_icosphere(R_BALL, subdivisions=3)
    sphere_stl = tri_dir / "sphere.stl"
    write_ascii_stl(sphere_stl, sphere_verts, sphere_faces, "sphere")
    print(f"  sphere.stl written: {len(sphere_verts)} vertices, {len(sphere_faces)} faces.")
    
    print(f"Generating spherical AMI surfaces (R_AMI = {R_AMI*1000:.1f} mm)...")
    ami_verts, ami_faces = create_icosphere(R_AMI, subdivisions=3)
    
    master_stl = tri_dir / "master.stl"
    write_ascii_stl(master_stl, ami_verts, ami_faces, "master")
    print(f"  master.stl written: {len(ami_verts)} vertices, {len(ami_faces)} faces.")
    
    slave_stl = tri_dir / "slave.stl"
    write_ascii_stl(slave_stl, ami_verts, ami_faces, "slave")
    print(f"  slave.stl written: {len(ami_verts)} vertices, {len(ami_faces)} faces.")
    
    # Combined rotating geometry (master sphere + ball)
    comb_rot_stl = tri_dir / "rotatingGeometry.stl"
    with open(comb_rot_stl, "w") as out_f:
        with open(master_stl, "r") as f_master:
            out_f.write(f_master.read())
        with open(sphere_stl, "r") as f_sphere:
            out_f.write(f_sphere.read())
    print("  rotatingGeometry.stl assembled.")
    
    # Visual golf-ball dots
    dots = generate_fibonacci_dots(N_DOTS, R_BALL * 1.002)  # slightly offset radially for rendering
    dots_file = case_dir / "golf_dots.json"
    with open(dots_file, "w") as f:
        json.dump({"radius": R_BALL, "n_dots": N_DOTS, "dots": dots}, f, indent=2)
    print(f"  golf_dots.json written: {N_DOTS} surface markers.")

if __name__ == "__main__":
    main()
