#!/usr/bin/env python3
"""
setup_gci_cases.py

Sets up the 3-mesh family for steady fixed broadside disk at Re = 100:
- Coarse (Grid 3): N3 = 130,928 cells
- Medium (Grid 2): N2 = 212,792 cells
- Fine   (Grid 1): N1 = 368,760 cells

Both static domain and near-body rotating zone are systematically refined.
"""
import os, shutil, subprocess

BASE_DIR = "/home/lavender/OpenFoamUbu/solvers/freeBodyFoam"
SRC_CASE = os.path.join(BASE_DIR, "validation/thinDisk2mm_tumbling/case")
MESH_DIR = os.path.join(BASE_DIR, "validation/mesh_convergence")

RE = 100
NU = 1.5e-5
D = 2.02e-3
RHO = 1.200
V = RE * NU / D  # 0.742574 m/s
END = 0.15       # > 2 flow-throughs in 50 mm domain

cases = {
    "gci_coarse_Re100": os.path.join(MESH_DIR, "coarseMesh/case/constant/polyMesh"),
    "gci_medium_Re100": os.path.join(MESH_DIR, "mediumMesh/case/constant/polyMesh"),
    "gci_fine_Re100":   os.path.join(MESH_DIR, "fineMesh/case/constant/polyMesh")
}

for c_name, mesh_src in cases.items():
    c_path = os.path.join(MESH_DIR, c_name)
    print(f"=== Setting up {c_name} at {c_path} ===")
    if os.path.exists(c_path):
        shutil.rmtree(c_path)
    os.makedirs(os.path.join(c_path, "constant"), exist_ok=True)
    os.makedirs(os.path.join(c_path, "system"), exist_ok=True)
    os.makedirs(os.path.join(c_path, "0"), exist_ok=True)

    # Copy mesh
    shutil.copytree(mesh_src, os.path.join(c_path, "constant/polyMesh"))

    # Copy constant files
    shutil.copy(os.path.join(SRC_CASE, "constant/dynamicMeshDict"), os.path.join(c_path, "constant/"))
    shutil.copy(os.path.join(SRC_CASE, "constant/transportProperties"), os.path.join(c_path, "constant/"))
    shutil.copy(os.path.join(SRC_CASE, "constant/turbulenceProperties"), os.path.join(c_path, "constant/"))

    # Write fallingFrameDict
    with open(os.path.join(c_path, "constant/fallingFrameDict"), "w") as f:
        f.write(f"""/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\\\    /   O peration     | Version:  2412
    \\\\  /    A nd           | Website:  www.openfoam.com
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    "constant";
    object      fallingFrameDict;
}}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

u0                  ( 0 -{V:.6f} 0 );
acceleration        ( 0 0 0 );
initialAcceleration ( 0 0 0 );

patches             ( disk );

mass                2.2113e-07;
rhoInf              {RHO};
densityModel        rhoInf;
gravityModel        constant;
groundInterrupt     off;

freedom             ( 0 0 0 );

rotor
{{
    hubOrigin         (0 0 0);
    inertiaPrincipal  (5.6436e-14 1.1278e-13 5.6436e-14);
    meshTiltAngleDeg  0.0;
    meshTiltAxis      (1 0 0);
    rotationFreedom   (0 0 0);
    initialOmega      (0 0 0);
    initialAngleDeg   0.0;
    initialAxis       (1 0 0);
    damping           (0 0 0);
    report            false;
}}

print
{{
    frameVel        on;
    frameAcc        on;
    altitude        off;
    pressureForce   off;
    viscousForce    off;
    aeroForce       on;
    gravityForce    on;
    netForce        on;
    meanFlow        off;
    relativeVel     off;
    dynamicPressure off;
}}
""")

    # Copy system files
    shutil.copy(os.path.join(SRC_CASE, "system/fvSchemes"), os.path.join(c_path, "system/"))
    shutil.copy(os.path.join(SRC_CASE, "system/fvSolution"), os.path.join(c_path, "system/"))
    shutil.copy(os.path.join(SRC_CASE, "system/decomposeParDict"), os.path.join(c_path, "system/"))

    # Write controlDict
    with open(os.path.join(c_path, "system/controlDict"), "w") as f:
        f.write(f"""/*--------------------------------*- C++ -*----------------------------------*\\
  =========                 |
  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\\\    /   O peration     | Version:  2412
    \\\\  /    A nd           | Website:  www.openfoam.com
\\*---------------------------------------------------------------------------*/
FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    "system";
    object      controlDict;
}}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

application     freeBody6DoFPimpleFoam;
startFrom       startTime;
startTime       0;
stopAt          endTime;
endTime         {END};

deltaT          2e-05;

writeControl    timeStep;
writeInterval   2500;
purgeWrite      2;

writeFormat     ascii;
writePrecision  8;
writeCompression off;
timeFormat      general;
timePrecision   6;
runTimeModifiable true;

adjustTimeStep  yes;
maxCo           1.5;
maxDeltaT       5e-05;

libs
(
    "libfreeBodyRotationalMotion.so"
    "libfallingInletVelocity.so"
);

functions
{{
    diskForces
    {{
        type            forces;
        libs            (forces);
        patches         (disk);
        rho             rhoInf;
        rhoInf          {RHO};
        CofR            (0 0 0);
        pName           p;
        UName           U;
        executeControl  timeStep;
        executeInterval 1;
        writeControl    timeStep;
        writeInterval   1;
    }}
}}
""")

    # Copy 0 files
    shutil.copy(os.path.join(SRC_CASE, "0/p_rgh"), os.path.join(c_path, "0/"))
    shutil.copy(os.path.join(SRC_CASE, "0/U"), os.path.join(c_path, "0/"))
    # Warm start
    u_path = os.path.join(c_path, "0/U")
    with open(u_path, "r") as uf:
        u_txt = uf.read()
    u_txt = u_txt.replace("internalField   uniform (0 0 0);", f"internalField   uniform (0 {V:.6f} 0);")
    with open(u_path, "w") as uf:
        uf.write(u_txt)

    # Write Allrun
    allrun_path = os.path.join(c_path, "Allrun")
    with open(allrun_path, "w") as ar:
        ar.write("""#!/bin/bash
set -euo pipefail
[ -n "${WM_PROJECT_DIR:-}" ] || . /usr/lib/openfoam/openfoam2412/etc/bashrc
rm -rf processor* log.*
decomposePar > log.decomposePar 2>&1
mpirun -np 8 freeBody6DoFPimpleFoam -parallel > log.freeBody6DoFPimpleFoam 2>&1
reconstructPar -latestTime > log.reconstructPar 2>&1 || true
""")
    os.chmod(allrun_path, 0o755)
    print(f"Prepared {c_name} successfully.")
