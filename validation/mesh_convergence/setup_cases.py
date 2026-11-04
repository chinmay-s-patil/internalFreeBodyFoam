#!/usr/bin/env python3
import os
import shutil

base_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence"
src_case = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case"

cases = {
    "coarse": os.path.join(base_dir, "coarseMesh", "case"),
    "medium": os.path.join(base_dir, "mediumMesh", "case"),
    "fine": os.path.join(base_dir, "fineMesh", "case")
}

# 1. Setup medium case
med_case = cases["medium"]
os.makedirs(os.path.join(med_case, "constant"), exist_ok=True)
if os.path.exists(os.path.join(med_case, "constant", "polyMesh")):
    shutil.rmtree(os.path.join(med_case, "constant", "polyMesh"))
shutil.copytree(os.path.join(src_case, "constant", "polyMesh"), os.path.join(med_case, "constant", "polyMesh"))

for c_name, c_path in cases.items():
    # Ensure system dir
    os.makedirs(os.path.join(c_path, "system"), exist_ok=True)
    for f in ["controlDict", "fvSchemes", "fvSolution"]:
        shutil.copy(os.path.join(src_case, "system", f), os.path.join(c_path, "system", f))
        
    # Ensure constant files
    for f in ["transportProperties", "turbulenceProperties", "fallingFrameDict"]:
        shutil.copy(os.path.join(src_case, "constant", f), os.path.join(c_path, "constant", f))
        
    # Ensure 0 dir
    if os.path.exists(os.path.join(c_path, "0")):
        shutil.rmtree(os.path.join(c_path, "0"))
    shutil.copytree(os.path.join(src_case, "0"), os.path.join(c_path, "0"))
    if os.path.exists(os.path.join(c_path, "0", "cellToRegion")):
        os.remove(os.path.join(c_path, "0", "cellToRegion"))
        
    # Standardize controlDict for pure spatial discretization benchmark:
    # Fixed deltaT = 2e-5 s, endTime = 0.002 s (100 steps)
    cd_file = os.path.join(c_path, "system", "controlDict")
    with open(cd_file, "r") as f:
        content = f.read()
    content = content.replace("endTime         0.5;", "endTime         0.002;")
    content = content.replace("adjustTimeStep  yes;", "adjustTimeStep  no;")
    content = content.replace("writeInterval   0.005;", "writeInterval   0.002;")
    with open(cd_file, "w") as f:
        f.write(content)
        
    print(f"Prepared standardized benchmark case for {c_name} at {c_path}")
