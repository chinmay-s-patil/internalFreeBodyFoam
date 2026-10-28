#!/bin/bash
. /usr/lib/openfoam/openfoam2412/etc/bashrc

BASE="/home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence/coarseMesh"

echo "=== 1. Rebuilding coarse staticMesh with smooth AMI ==="
cd "$BASE/staticMesh"
# Use (24 40 24) for smooth AMI geometry matching
sed -i 's/(18 30 18)/(24 40 24)/g' system/blockMeshDict
blockMesh
surfaceFeatureExtract
snappyHexMesh -overwrite

echo "=== 2. Re-assembling coarse case ==="
rm -rf "$BASE/case"
mkdir -p "$BASE/case/constant"
cp -r "$BASE/staticMesh/constant/polyMesh" "$BASE/case/constant/"
cp -r /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/system "$BASE/case/"
cp /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/constant/transportProperties "$BASE/case/constant/"
cp /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/constant/turbulenceProperties "$BASE/case/constant/"
cp /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/constant/fallingFrameDict "$BASE/case/constant/"
cp /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/0 "$BASE/case/" 2>/dev/null || true
cp -r /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/thinDisk2mm/case/0 "$BASE/case/"

# Set controlDict for 100 steps
sed -i 's/endTime         0.5;/endTime         0.002;/g' "$BASE/case/system/controlDict"
sed -i 's/adjustTimeStep  yes;/adjustTimeStep  no;/g' "$BASE/case/system/controlDict"
sed -i 's/writeInterval   0.005;/writeInterval   0.002;/g' "$BASE/case/system/controlDict"

# Use robust topoSetDict
cp /home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/mesh_convergence/coarseMesh/case/system/topoSetDict "$BASE/case/system/" 2>/dev/null || true

cd "$BASE/case"
mergeMeshes . ../rotatingMesh -overwrite

# Generate topoSet
cat << 'EOF' > system/topoSetDict
/*--------------------------------*- C++ -*----------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Version:  2412
    \\  /    A nd           | Website:  www.openfoam.com
     \\/     M anipulation  |
\*---------------------------------------------------------------------------*/
FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    location    "system";
    object      topoSetDict;
}
actions
(
    {
        name    rotatingCellSet;
        type    cellSet;
        action  new;
        source  sphereToCell;
        origin  (0 0 0);
        radius  0.00205;
    }
    {
        name    rotatingZone;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        set     rotatingCellSet;
    }
    {
        name    staticCellSet;
        type    cellSet;
        action  new;
        source  cellToCell;
        set     rotatingCellSet;
    }
    {
        name    staticCellSet;
        type    cellSet;
        action  invert;
    }
    {
        name    staticZone;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        set     staticCellSet;
    }
);
EOF

topoSet

# Update createPatchDict with lowWeightCorrection
cat << 'EOF' > system/createPatchDict
/*--------------------------------*- C++ -*----------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     | Version:  2412
    \\  /    A nd           | Website:  www.openfoam.com
     \\/     M anipulation  |
\*---------------------------------------------------------------------------*/
FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      createPatchDict;
}
pointSync false;
writeCyclicMatch true;
patches
(
    {
        name            masterAMI;
        patchInfo
        {
            type            cyclicAMI;
            matchTolerance  0.0001;
            transform       noOrdering;
            rotationAxis    (0 1 0);
            rotationCentre  (0 0 0);
            neighbourPatch  slaveAMI;
            method          faceAreaWeightAMI;
            lowWeightCorrection 0.1;
        }
        constructFrom patches;
        patches (master);
    }
    {
        name            slaveAMI;
        patchInfo
        {
            type            cyclicAMI;
            matchTolerance  0.0001;
            transform       noOrdering;
            rotationAxis    (0 1 0);
            rotationCentre  (0 0 0);
            neighbourPatch  masterAMI;
            method          faceAreaWeightAMI;
            lowWeightCorrection 0.1;
        }
        constructFrom patches;
        patches (slave);
    }
);
EOF

createPatch -overwrite
checkMesh > log.checkMesh
echo "Rebuilt coarse mesh successfully!"
