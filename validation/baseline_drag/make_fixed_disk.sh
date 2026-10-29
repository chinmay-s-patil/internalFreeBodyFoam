#!/bin/bash
# make_fixed_disk.sh <Re> [SRC_CASE]
# Builds fixedDisk_Re<Re>/ : the 2 mm disk held fixed and broadside in a uniform stream (no gravity,
# no translation, no rotation) on the SAME mesh and numerics as the tumbling case.
# Env: END=<endTime s> (default 0.30), BASE_ONLY=1 skips nothing (reserved)
set -euo pipefail
cd "${0%/*}"
RE=${1:?usage: $0 <Re> [SRC_CASE]}
SRC=$(realpath "${2:-../thinDisk2mm_tumbling/case}")          # must hold constant/polyMesh
NU=1.5e-5; D=2.02e-3; END=${END:-0.30}
[ -n "${WM_PROJECT_DIR:-}" ] || . /usr/lib/openfoam/openfoam2412/etc/bashrc   # same env as Allrun
V=$(python3 -c "print(f'{$RE*$NU/$D:.6f}')")                    # V = Re nu / D
CASE=fixedDisk_Re${RE}

rm -rf "$CASE"; mkdir -p "$CASE/constant" "$CASE/0"
cp -r "$SRC/system" "$CASE/"                                    # identical schemes/solvers/decomposition
cp "$SRC"/constant/{fallingFrameDict,dynamicMeshDict,transportProperties,turbulenceProperties} "$CASE/constant/"
cp "$SRC"/0/{U,p_rgh} "$CASE/0/"
ln -s "$SRC/constant/polyMesh" "$CASE/constant/polyMesh"        # shared mesh, never written to
cp "$SRC/Allrun" "$SRC/Allclean" "$CASE/"

FF=$CASE/constant/fallingFrameDict
foamDictionary -entry u0                -set "(0 -$V 0)" "$FF" >/dev/null   # body moves down, relative flow +Y
foamDictionary -entry acceleration      -set "(0 0 0)"   "$FF" >/dev/null   # no gravity: no hydrostatic/buoyancy term
foamDictionary -entry initialAcceleration -set "(0 0 0)" "$FF" >/dev/null
foamDictionary -entry freedom           -set "(0 0 0)"   "$FF" >/dev/null   # frame velocity frozen
foamDictionary -entry rotor/rotationFreedom -set "(0 0 0)" "$FF" >/dev/null # disk attitude frozen (broadside)
foamDictionary -entry rotor/initialOmega    -set "(0 0 0)" "$FF" >/dev/null
foamDictionary -entry rotor/report          -set false     "$FF" >/dev/null
foamDictionary -entry internalField -set "uniform (0 $V 0)" "$CASE/0/U" >/dev/null  # warm start, no impulsive start
foamDictionary -entry endTime -set "$END" "$CASE/system/controlDict" >/dev/null

cat > "$CASE/system/forcesFunctions" <<EOT
diskForces                          // independent F and M about hub (cross-check of the hand-rolled integral)
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
amiFluxMaster                       // net flux through masterAMI, expect ~0
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
EOT
sed -i '/^libs/i functions\n{\n    #include "forcesFunctions"\n}\n' "$CASE/system/controlDict"

echo "Built $CASE  (V = $V m/s, Re_D = $RE, endTime = $END s). Run:  cd $CASE && ./Allrun"
echo "Then:  python3 analyze_fixed_disk.py $CASE --Re $RE"
