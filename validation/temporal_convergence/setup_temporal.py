#!/usr/bin/env python3
import os
import shutil

base_dir = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/temporal_convergence"
src_case = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence/mediumMesh/case"

dt_configs = {
    "dt_baseline": 2.0e-5, # baseline dt
    "dt_halved":   1.0e-5, # halved dt (r_t = 2.0)
    "dt_quarter":  5.0e-6  # quarter dt (r_t = 2.0)
}

for name, dt_val in dt_configs.items():
    case_path = os.path.join(base_dir, name)
    if os.path.exists(case_path):
        shutil.rmtree(case_path)
    shutil.copytree(src_case, case_path)
    
    # clean out postProcessing and old time directories
    for item in os.listdir(case_path):
        if item.replace('.', '', 1).isdigit() and item != "0":
            shutil.rmtree(os.path.join(case_path, item))
    pp_dir = os.path.join(case_path, "postProcessing")
    if os.path.exists(pp_dir):
        shutil.rmtree(pp_dir)
        
    cd_file = os.path.join(case_path, "system", "controlDict")
    with open(cd_file, "r") as f:
        c = f.read()
    c = c.replace("endTime         0.002;", "endTime         0.001;")
    c = c.replace("deltaT          2e-5;", f"deltaT          {dt_val:.1e};")
    c = c.replace("writeInterval   0.002;", "writeInterval   0.001;")
    with open(cd_file, "w") as f:
        f.write(c)
        
    print(f"Prepared temporal case {name} with dt = {dt_val:.1e} s")
