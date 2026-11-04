#!/usr/bin/env python3
"""
generate_benchmark_sphere_stls.py

Generates high-precision STLs for Uhlmann & Dusek (2014) benchmark:
1. Sphere: Diameter D = 1.0 m (Radius R = 0.5 m)
2. Spherical AMI Master interface: Radius R = 1.0 m (D_AMI = 2.0 m)
3. Matching spherical cavity Slave interface: Radius R = 1.0 m
"""

import os
from pathlib import Path
import numpy as np

R_SPHERE = 0.5      # 1.0 m diameter
R_AMI = 1.0         # 2.0 m diameter AMI interface

def create_icosphere(radius, subdivisions=4):
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

def main():
    out_dir = Path(__file__).resolve().parent / "triSurface"
    out_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Generating benchmark sphere geometry (D = {R_SPHERE*2:.2f} m)...")
    sphere_verts, sphere_faces = create_icosphere(R_SPHERE, subdivisions=4)
    sphere_stl = out_dir / "sphere.stl"
    write_ascii_stl(sphere_stl, sphere_verts, sphere_faces, "sphere")
    print(f"  sphere.stl written: {len(sphere_verts)} vertices, {len(sphere_faces)} faces.")
    
    print(f"Generating spherical AMI surfaces (R_AMI = {R_AMI:.2f} m)...")
    ami_verts, ami_faces = create_icosphere(R_AMI, subdivisions=4)
    
    master_stl = out_dir / "master.stl"
    write_ascii_stl(master_stl, ami_verts, ami_faces, "master")
    print(f"  master.stl written: {len(ami_verts)} vertices, {len(ami_faces)} faces.")
    
    slave_stl = out_dir / "slave.stl"
    write_ascii_stl(slave_stl, ami_verts, ami_faces, "slave")
    print(f"  slave.stl written: {len(ami_verts)} vertices, {len(ami_faces)} faces.")
    
    # Combined rotating geometry (master sphere + ball)
    comb_rot_stl = out_dir / "rotatingGeometry.stl"
    with open(comb_rot_stl, "w") as out_f:
        with open(master_stl, "r") as f_master:
            out_f.write(f_master.read())
        with open(sphere_stl, "r") as f_sphere:
            out_f.write(f_sphere.read())
    print("  rotatingGeometry.stl assembled.")

if __name__ == "__main__":
    main()
