#!/bin/bash
set -euo pipefail
cd "${0%/*}"

echo "=== Setting up fixedDisk_Re140_domain1p5 ==="
ROOT=$(realpath "../thinDisk2mm_tumbling")
SRC="$ROOT/case"
ROT="$ROOT/meshing/rotatingMesh"
STATIC_POLY="domain1p5_static/constant/polyMesh"
CASE="fixedDisk_Re140_domain1p5"

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    . /usr/lib/openfoam/openfoam2412/etc/bashrc
fi

if [ ! -d "$STATIC_POLY" ]; then
    echo "Static mesh missing in $STATIC_POLY! Run build_domain1p5_mesh.sh first."
    exit 1
fi

rm -rf "$CASE"
mkdir -p "$CASE/constant" "$CASE/0"

echo "Copying dictionaries and fields from $SRC..."
cp -r "$SRC/system" "$CASE/"
cp "$SRC"/constant/{fallingFrameDict,dynamicMeshDict,transportProperties,turbulenceProperties} "$CASE/constant/"
cp "$SRC"/0/{U,p_rgh} "$CASE/0/"
cp "$SRC/Allrun" "$SRC/Allclean" "$CASE/"

echo "Copying 1.5x domain static polyMesh..."
cp -r "$STATIC_POLY" "$CASE/constant/"

echo "Merging meshes with rotating mesh $ROT..."
( cd "$CASE" && mergeMeshes . "$ROT" -overwrite > log.mergeMeshes )

echo "Creating cyclicAMI patches..."
( cd "$CASE" && createPatch -overwrite > log.createPatch )

echo "Running checkMesh to identify regions..."
( cd "$CASE" && checkMesh > log.checkMesh 2>&1 )

echo "Creating cell zones with topoSet..."
( cd "$CASE" && topoSet > log.topoSet )
grep -E "cells:|Mesh OK|Max non-orth" "$CASE/log.checkMesh" || true

echo "Configuring case for Re=140 fixed broadside disk..."
RE=140
NU=1.5e-5
D=2.02e-3
END=0.08
V=$(python3 -c "print(f'{$RE*$NU/$D:.6f}')") # 1.039604

FF="$CASE/constant/fallingFrameDict"
foamDictionary -entry u0                  -set "(0 -$V 0)" "$FF" >/dev/null
foamDictionary -entry acceleration        -set "(0 0 0)"   "$FF" >/dev/null
foamDictionary -entry initialAcceleration -set "(0 0 0)"   "$FF" >/dev/null
foamDictionary -entry freedom             -set "(0 0 0)"   "$FF" >/dev/null
foamDictionary -entry rotor/rotationFreedom -set "(0 0 0)" "$FF" >/dev/null
foamDictionary -entry rotor/initialOmega  -set "(0 0 0)"   "$FF" >/dev/null
foamDictionary -entry rotor/report        -set false       "$FF" >/dev/null

foamDictionary -entry internalField -set "uniform (0 $V 0)" "$CASE/0/U" >/dev/null
foamDictionary -entry endTime -set "$END" "$CASE/system/controlDict" >/dev/null
foamDictionary -entry purgeWrite -set 0 "$CASE/system/controlDict" >/dev/null

# Configure forces and AMI flux monitoring
cat > "$CASE/system/forcesFunctions" << 'EOF'
diskForces
{
    type            forces;
    libs            (forces);
    patches         (disk);
    rho             rhoInf;
    rhoInf          1.2;
    CofR            (0 0 0);
    pName           p;
    UName           U;
    executeControl  timeStep;
    executeInterval 1;
    writeControl    timeStep;
    writeInterval   1;
}
amiFluxMaster
{
    type            surfaceFieldValue;
    libs            (fieldFunctionObjects);
    regionType      patch;
    name            masterAMI;
    operation       sum;
    fields          (phi);
    writeFields     no;
    executeControl  timeStep;
    executeInterval 1;
    writeControl    timeStep;
    writeInterval   1;
}
EOF

sed -i '/^libs/i functions\n{\n    #include "forcesFunctions"\n}\n' "$CASE/system/controlDict"

# Use 14 cores for parallel run
foamDictionary -entry numberOfSubdomains -set 14 "$CASE/system/decomposeParDict" >/dev/null

echo "Case $CASE configured successfully."
echo "Cells: $(grep -oP 'cells:\s+\K[0-9]+' "$CASE/log.checkMesh" 2>/dev/null || echo 'unknown')"
echo "Domain: 1.5x scaled (45 mm x 75 mm x 45 mm, ~22.5D x 37.5D x 22.5D)"
echo "Velocity: V = $V m/s (Re = 140)"
echo "Cores: 14"
