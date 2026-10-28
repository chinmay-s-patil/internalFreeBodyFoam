#!/usr/bin/env python3
import os
import shutil
import subprocess

def run_cmd(cmd, cwd):
    print(f"[{cwd}] Running: {cmd}")
    res = subprocess.run(cmd, shell=True, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        print(f"ERROR: {res.stderr}")
        raise RuntimeError(f"Command failed: {cmd}\n{res.stderr}")
    return res.stdout

target_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence/coarseMesh"
src_mesh_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/meshing"
case_sys_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/system"

os.makedirs(target_dir, exist_ok=True)
rot_dir = os.path.join(target_dir, "rotatingMesh")
stat_dir = os.path.join(target_dir, "staticMesh")

# Copy rotatingMesh and staticMesh skeletons
for d, name in [(rot_dir, "rotatingMesh"), (stat_dir, "staticMesh")]:
    if os.path.exists(d):
        shutil.rmtree(d)
    shutil.copytree(os.path.join(src_mesh_dir, name), d)
    # clean out constant/polyMesh if present
    poly = os.path.join(d, "constant", "polyMesh")
    if os.path.exists(poly):
        shutil.rmtree(poly)
    # ensure constant/triSurface exists
    tri = os.path.join(d, "constant", "triSurface")
    os.makedirs(tri, exist_ok=True)
    for stl in ["disk.stl", "master.stl", "rotatingGeometry.stl", "slave.stl"]:
        src_stl = os.path.join(src_mesh_dir, "triSurface", stl)
        if os.path.exists(src_stl):
            shutil.copy(src_stl, tri)

# Coarse adjustments:
# rotatingMesh blockMesh: (28 28 28) instead of (36 36 36) -> r = 36/28 = 1.286
rot_bm = os.path.join(rot_dir, "system", "blockMeshDict")
with open(rot_bm, "r") as f:
    c = f.read()
c = c.replace("(36 36 36)", "(28 28 28)")
with open(rot_bm, "w") as f:
    f.write(c)

# staticMesh blockMesh: (18 30 18) instead of (24 40 24) -> r = 1.33
stat_bm = os.path.join(stat_dir, "system", "blockMeshDict")
with open(stat_bm, "r") as f:
    c = f.read()
c = c.replace("(24 40 24)", "(18 30 18)")
with open(stat_bm, "w") as f:
    f.write(c)

print("Coarse mesh templates prepared.")
