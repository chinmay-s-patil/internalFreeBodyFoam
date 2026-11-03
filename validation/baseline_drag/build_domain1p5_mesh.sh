#!/bin/bash
set -euo pipefail
cd "${0%/*}"

echo "=== Building 1.5x Domain Static Mesh ==="
ROOT=$(realpath "../thinDisk2mm_tumbling")
STAT="$ROOT/meshing/staticMesh"
ROT="$ROOT/meshing/rotatingMesh"
DIR="domain1p5_static"

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    . /usr/lib/openfoam/openfoam2412/etc/bashrc
fi

rm -rf "$DIR"
mkdir -p "$DIR/constant" "$DIR/system"

# Copy system and constant from STAT
cp -r "$STAT/system"/* "$DIR/system/"
cp -r "$STAT/constant"/* "$DIR/constant/"
rm -rf "$DIR/constant/polyMesh"

# Update blockMeshDict: scale domain coordinates by 1.5x
# Baseline:
#   X: -0.015 to +0.015  (30 mm)
#   Y: -0.025 to +0.025  (50 mm)
#   Z: -0.015 to +0.015  (30 mm)
# 1.5x:
#   X: -0.0225 to +0.0225 (45 mm)
#   Y: -0.0375 to +0.0375 (75 mm)
#   Z: -0.0225 to +0.0225 (45 mm)
# Cell count: (36 60 36) -> retains 1.25 mm background cell size

cat > "$DIR/system/blockMeshDict" << 'EOF'
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
    object      blockMeshDict;
}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

scale   1;

vertices
(
    (-0.0225 -0.0375 -0.0225)
    ( 0.0225 -0.0375 -0.0225)
    ( 0.0225  0.0375 -0.0225)
    (-0.0225  0.0375 -0.0225)
    (-0.0225 -0.0375  0.0225)
    ( 0.0225 -0.0375  0.0225)
    ( 0.0225  0.0375  0.0225)
    (-0.0225  0.0375  0.0225)
);

blocks
(
    hex (0 1 2 3 4 5 6 7) (36 60 36) simpleGrading (1 1 1)
);

edges
(
);

boundary
(
    inlet
    {
        type patch;
        faces
        (
            (0 1 5 4)
        );
    }
    outlet
    {
        type patch;
        faces
        (
            (3 2 6 7)
        );
    }
    atmosphere
    {
        type patch;
        faces
        (
            (0 4 7 3)
            (1 2 6 5)
            (0 3 2 1)
            (4 5 6 7)
        );
    }
);

// ************************************************************************* //
EOF

echo "Running blockMesh..."
( cd "$DIR" && blockMesh > log.blockMesh )

echo "Running snappyHexMesh..."
( cd "$DIR" && snappyHexMesh -overwrite > log.snappyHexMesh )

echo "Running checkMesh on static mesh..."
( cd "$DIR" && checkMesh > log.checkMesh )
grep -E "cells:|Mesh OK|Max non-orth" "$DIR/log.checkMesh"

echo "=== Static mesh generation complete ==="
