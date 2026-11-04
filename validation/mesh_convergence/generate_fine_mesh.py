#!/usr/bin/env python3
import os
import shutil
import subprocess

target_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence/fineMesh"
src_mesh_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/meshing"

os.makedirs(target_dir, exist_ok=True)
rot_dir = os.path.join(target_dir, "rotatingMesh")
stat_dir = os.path.join(target_dir, "staticMesh")

for d, name in [(rot_dir, "rotatingMesh"), (stat_dir, "staticMesh")]:
    if os.path.exists(d):
        shutil.rmtree(d)
    shutil.copytree(os.path.join(src_mesh_dir, name), d)
    poly = os.path.join(d, "constant", "polyMesh")
    if os.path.exists(poly):
        shutil.rmtree(poly)
    tri = os.path.join(d, "constant", "triSurface")
    os.makedirs(tri, exist_ok=True)
    for stl in ["disk.stl", "master.stl", "rotatingGeometry.stl", "slave.stl"]:
        src_stl = os.path.join(src_mesh_dir, "triSurface", stl)
        if os.path.exists(src_stl):
            shutil.copy(src_stl, tri)

# Fine adjustments:
# rotatingMesh blockMesh: (46 46 46) instead of (36 36 36) -> r = 46/36 = 1.278
rot_bm = os.path.join(rot_dir, "system", "blockMeshDict")
with open(rot_bm, "r") as f:
    c = f.read()
c = c.replace("(36 36 36)", "(46 46 46)")
with open(rot_bm, "w") as f:
    f.write(c)

# staticMesh blockMesh: (30 50 30) instead of (24 40 24) -> r = 1.25
stat_bm = os.path.join(stat_dir, "system", "blockMeshDict")
with open(stat_bm, "r") as f:
    c = f.read()
c = c.replace("(24 40 24)", "(30 50 30)")
with open(stat_bm, "w") as f:
    f.write(c)

print("Fine mesh templates prepared.")
