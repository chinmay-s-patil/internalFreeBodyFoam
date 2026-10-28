#!/usr/bin/env python3
"""
generate_disk_and_spherical_stls.py

Generates high-precision conformal STLs for:
1. Thin disk geometry (D = 2.02 mm, h = 50 um)
2. Spherical AMI master interface (D = 4.0 mm)
3. Matching spherical cavity slave interface (D = 4.0 mm)
"""

import os
import numpy as np

# Dimensions in meters
D_DISK = 2.02e-3      # 2.02 mm
R_DISK = D_DISK / 2.0 # 1.01 mm
H_DISK = 50.0e-6      # 50 um = 0.05 mm
R_SPHERE = 2.0e-3     # 2.0 mm (4.0 mm diameter sphere)

N_CIRC = 120          # 120 segments along disk circumference
SUBDIV_SPHERE = 3     # 1,280 triangular faces for sphere (clean, uniform)

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

def create_disk(radius, height, n_circ=120):
    """Generate vertices and faces for a cylindrical thin disk."""
    verts = []
    faces = []
    
    y_top = height / 2.0
    y_bot = -height / 2.0
    
    # Vertex 0: Center top
    verts.append([0.0, y_top, 0.0])
    # Vertex 1: Center bottom
    verts.append([0.0, y_bot, 0.0])
    
    # Angles
    angles = np.linspace(0, 2 * np.pi, n_circ, endpoint=False)
    
    # Top rim vertices: indices 2 to 2 + n_circ - 1
    for theta in angles:
        x = radius * np.cos(theta)
        z = radius * np.sin(theta)
        verts.append([x, y_top, z])
        
    # Bottom rim vertices: indices 2 + n_circ to 2 + 2*n_circ - 1
    for theta in angles:
        x = radius * np.cos(theta)
        z = radius * np.sin(theta)
        verts.append([x, y_bot, z])
        
    # Faces for top lid (normal pointing +y)
    for i in range(n_circ):
        i_curr = 2 + i
        i_next = 2 + ((i + 1) % n_circ)
        faces.append([0, i_curr, i_next])
        
    # Faces for bottom lid (normal pointing -y)
    offset_bot = 2 + n_circ
    for i in range(n_circ):
        i_curr = offset_bot + i
        i_next = offset_bot + ((i + 1) % n_circ)
        faces.append([1, i_next, i_curr])
        
    # Faces for side rim (2 triangles per circumferential quad)
    for i in range(n_circ):
        t_curr = 2 + i
        t_next = 2 + ((i + 1) % n_circ)
        b_curr = offset_bot + i
        b_next = offset_bot + ((i + 1) % n_circ)
        
        # Quad (t_curr, b_curr, b_next, t_next)
        faces.append([t_curr, b_curr, b_next])
        faces.append([t_curr, b_next, t_next])
        
    return np.array(verts), np.array(faces)

def create_icosphere(radius, subdivisions=3):
    """Generate vertex and face arrays for an equilateral icosphere."""
    phi = (1.0 + np.sqrt(5.0)) / 2.0
    verts = [
        [-1,  phi, 0], [ 1,  phi, 0], [-1, -phi, 0], [ 1, -phi, 0],
        [0, -1,  phi], [0,  1,  phi], [0, -1, -phi], [0,  1, -phi],
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
        key = tuple(sorted([i1, i2]))
        if key in midpoint_cache:
            return midpoint_cache[key]
        mid = (np.array(verts[i1]) + np.array(verts[i2])) / 2.0
        mid = mid / np.linalg.norm(mid)
        idx = len(verts)
        verts.append(mid.tolist())
        midpoint_cache[key] = idx
        return idx

    for _ in range(subdivisions):
        new_faces = []
        for tri in faces:
            a = get_midpoint(tri[0], tri[1])
            b = get_midpoint(tri[1], tri[2])
            c = get_midpoint(tri[2], tri[0])
            new_faces.extend([
                [tri[0], a, c],
                [tri[1], b, a],
                [tri[2], c, b],
                [a, b, c]
            ])
        faces = new_faces
    
    verts = np.array(verts) * radius
    return verts, np.array(faces)

def write_ascii_stl(filepath, verts, faces, solid_name):
    """Write an ASCII STL with specified solid name."""
    with open(filepath, "w") as f:
        f.write(f"solid {solid_name}\n")
        for tri in faces:
            v0, v1, v2 = verts[tri[0]], verts[tri[1]], verts[tri[2]]
            normal = np.cross(v1 - v0, v2 - v0)
            norm = np.linalg.norm(normal)
            if norm > 0:
                normal = normal / norm
            f.write(f" facet normal {normal[0]:.16e} {normal[1]:.16e} {normal[2]:.16e}\n")
            f.write("  outer loop\n")
            f.write(f"   vertex {v0[0]:.16e} {v0[1]:.16e} {v0[2]:.16e}\n")
            f.write(f"   vertex {v1[0]:.16e} {v1[1]:.16e} {v1[2]:.16e}\n")
            f.write(f"   vertex {v2[0]:.16e} {v2[1]:.16e} {v2[2]:.16e}\n")
            f.write("  endloop\n")
            f.write(" endfacet\n")
        f.write(f"endsolid {solid_name}\n")

def main():
    surf_dir = os.path.join(OUTPUT_DIR, "triSurface")
    os.makedirs(surf_dir, exist_ok=True)
    
    print("Generating disk geometry (D = 2.02 mm, h = 50 um)...")
    disk_verts, disk_faces = create_disk(R_DISK, H_DISK, N_CIRC)
    disk_stl = os.path.join(surf_dir, "disk.stl")
    write_ascii_stl(disk_stl, disk_verts, disk_faces, "disk")
    print(f"  disk.stl written: {len(disk_verts)} vertices, {len(disk_faces)} faces.")
    
    print(f"Generating spherical AMI surfaces (Radius = {R_SPHERE*1000:.1f} mm)...")
    sphere_verts, sphere_faces = create_icosphere(R_SPHERE, SUBDIV_SPHERE)
    
    master_stl = os.path.join(surf_dir, "master.stl")
    write_ascii_stl(master_stl, sphere_verts, sphere_faces, "master")
    print(f"  master.stl written: {len(sphere_verts)} vertices, {len(sphere_faces)} faces.")
    
    slave_stl = os.path.join(surf_dir, "slave.stl")
    write_ascii_stl(slave_stl, sphere_verts, sphere_faces, "slave")
    print(f"  slave.stl written: {len(sphere_verts)} vertices, {len(sphere_faces)} faces.")
    
    # Combined rotating geometry (master sphere + disk)
    comb_rot_stl = os.path.join(surf_dir, "rotatingGeometry.stl")
    with open(comb_rot_stl, "w") as out_f:
        with open(master_stl, "r") as f_master:
            out_f.write(f_master.read())
        with open(disk_stl, "r") as f_disk:
            out_f.write(f_disk.read())
    print("  rotatingGeometry.stl assembled.")

if __name__ == "__main__":
    main()
