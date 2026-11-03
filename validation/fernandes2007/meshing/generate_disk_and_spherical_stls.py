#!/usr/bin/env python3
"""
generate_disk_and_spherical_stls.py

Generates high-precision conformal STLs for Fernandes et al. (2007) benchmark:
1. Cylindrical disk: D = 10.0 mm, h = 1.0 mm (aspect ratio chi = 10)
2. Spherical AMI master interface: R = 10.0 mm (D = 20.0 mm)
3. Matching spherical cavity slave interface: R = 10.0 mm
"""

import os
import numpy as np

# Dimensions in meters
D_DISK = 10.0e-3       # 10.0 mm
R_DISK = D_DISK / 2.0  # 5.0 mm
H_DISK = 1.0e-3        # 1.0 mm (chi = 10)
R_SPHERE = 10.0e-3     # 10.0 mm (20.0 mm diameter sphere)

N_CIRC = 120           # 120 segments along disk circumference
SUBDIV_SPHERE = 3      # 1,280 triangular faces for sphere (clean, uniform)

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
        
    # Faces for side wall (two triangles per circumferential segment)
    for i in range(n_circ):
        top_curr = 2 + i
        top_next = 2 + ((i + 1) % n_circ)
        bot_curr = offset_bot + i
        bot_next = offset_bot + ((i + 1) % n_circ)
        
        # Tri 1: top_curr -> bot_curr -> bot_next
        faces.append([top_curr, bot_curr, bot_next])
        # Tri 2: top_curr -> bot_next -> top_next
        faces.append([top_curr, bot_next, top_next])
        
    return np.array(verts), np.array(faces)


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
            
            # Normal vector via cross product
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
    surf_dir = os.path.join(OUTPUT_DIR, "triSurface")
    os.makedirs(surf_dir, exist_ok=True)
    
    print(f"Generating disk geometry (D = {D_DISK*1000:.1f} mm, h = {H_DISK*1000:.1f} mm)...")
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
