#!/usr/bin/env python3
"""
generate_spherical_stls.py

Generates conformal, high-quality icosphere STLs for the 360 mm diameter
spherical rotating domain and matching spherical cavity for propFall/propTumble.
"""

import os
import shutil
import numpy as np

RADIUS = 0.18  # 180 mm radius -> 360 mm diameter (53 mm clearance from blade tips)
SUBDIVISIONS = 4  # 5,120 faces (equilateral triangles, uniform area distribution)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROT_DIR = os.path.join(BASE_DIR, "rotatingMesh")
STAT_DIR = os.path.join(BASE_DIR, "staticMesh")
ROT_SURF = os.path.join(ROT_DIR, "constant", "triSurface")
STAT_SURF = os.path.join(STAT_DIR, "constant", "triSurface")

def create_icosphere(radius, subdivisions=4):
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
    print(f"Generating conformal spherical STLs (Radius = {RADIUS} m, Diameter = {RADIUS*2*1000:.0f} mm)...")
    verts, faces = create_icosphere(RADIUS, SUBDIVISIONS)
    print(f"  Generated icosphere: {len(verts)} vertices, {len(faces)} triangular faces.")

    # 1. Back up original cylinder STLs if not already backed up
    rot_backup = os.path.join(ROT_SURF, "orig_cylinder")
    if not os.path.exists(rot_backup):
        os.makedirs(rot_backup, exist_ok=True)
        shutil.copy2(os.path.join(ROT_SURF, "patch_1_0.stl"), rot_backup)
        shutil.copy2(os.path.join(ROT_SURF, "Part__Feature001_Geometry.stl"), rot_backup)
        print("  Backed up rotatingMesh cylinder STLs to", rot_backup)

    stat_backup = os.path.join(STAT_SURF, "orig_cylinder")
    if not os.path.exists(stat_backup):
        os.makedirs(stat_backup, exist_ok=True)
        shutil.copy2(os.path.join(STAT_SURF, "patch_3_1.stl"), stat_backup)
        shutil.copy2(os.path.join(STAT_SURF, "MeshRefinement.stl"), stat_backup)
        shutil.copy2(os.path.join(STAT_SURF, "Part__Feature_Geometry.stl"), stat_backup)
        print("  Backed up staticMesh cylinder STLs to", stat_backup)

    # 2. Write rotatingMesh spherical patch_1_0.stl
    rot_patch_1_0 = os.path.join(ROT_SURF, "patch_1_0.stl")
    write_ascii_stl(rot_patch_1_0, verts, faces, "patch_1_0")
    print("  Wrote rotatingMesh spherical patch_1_0.stl")

    # 3. Assemble rotatingMesh Part__Feature001_Geometry.stl (patch_1_0 + patch_2_1)
    rot_geom = os.path.join(ROT_SURF, "Part__Feature001_Geometry.stl")
    prop_stl = os.path.join(ROT_SURF, "patch_2_1.stl")
    with open(rot_geom, "w") as out_f:
        with open(rot_patch_1_0, "r") as f1:
            out_f.write(f1.read())
        with open(prop_stl, "r") as f2:
            out_f.write(f2.read())
    print("  Assembled rotatingMesh Part__Feature001_Geometry.stl")

    # 4. Write staticMesh spherical patch_3_1.stl & MeshRefinement.stl
    stat_patch_3_1 = os.path.join(STAT_SURF, "patch_3_1.stl")
    write_ascii_stl(stat_patch_3_1, verts, faces, "patch_3_1")
    stat_refine = os.path.join(STAT_SURF, "MeshRefinement.stl")
    write_ascii_stl(stat_refine, verts, faces, "MeshRefinement")
    print("  Wrote staticMesh spherical patch_3_1.stl and MeshRefinement.stl")

    # 5. Assemble staticMesh Part__Feature_Geometry.stl (patch_1_0 + patch_2_0 + patch_3_1 + patch_4_0)
    stat_geom = os.path.join(STAT_SURF, "Part__Feature_Geometry.stl")
    p1 = os.path.join(STAT_SURF, "patch_1_0.stl")
    p2 = os.path.join(STAT_SURF, "patch_2_0.stl")
    p4 = os.path.join(STAT_SURF, "patch_4_0.stl")
    with open(stat_geom, "w") as out_f:
        with open(p1, "r") as f:
            out_f.write(f.read())
        with open(p2, "r") as f:
            out_f.write(f.read())
        with open(stat_patch_3_1, "r") as f:
            out_f.write(f.read())
        with open(p4, "r") as f:
            out_f.write(f.read())
    print("  Assembled staticMesh Part__Feature_Geometry.stl")

    print("\nSpherical STL generation completed successfully!")

if __name__ == "__main__":
    main()
