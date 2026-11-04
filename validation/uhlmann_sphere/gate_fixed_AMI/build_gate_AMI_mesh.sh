#!/bin/bash
set -euo pipefail
cd "${0%/*}"

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    . /usr/lib/openfoam/openfoam2412/etc/bashrc
fi

echo "=== 1. Building rotatingMesh ==="
cd meshing/rotatingMesh
mkdir -p constant/triSurface
cp ../triSurface/* constant/triSurface/
cp ../../../gate_fixed_noAMI/system/controlDict system/
blockMesh > log.blockMesh 2>&1
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1
checkMesh > log.checkMesh 2>&1
echo "rotatingMesh completed."

cd ../staticMesh
echo "=== 2. Building staticMesh ==="
mkdir -p constant/triSurface
cp ../triSurface/* constant/triSurface/
cp ../../../gate_fixed_noAMI/system/controlDict system/
blockMesh > log.blockMesh 2>&1
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1
checkMesh > log.checkMesh 2>&1
echo "staticMesh completed."

cd ../..
echo "=== 3. Merging Meshes into Case ==="
rm -rf constant/polyMesh
cp -r meshing/staticMesh/constant/polyMesh constant/

mergeMeshes . meshing/rotatingMesh -overwrite > log.mergeMeshes 2>&1
topoSet > log.topoSet 2>&1
createPatch -overwrite > log.createPatch 2>&1
checkMesh > log.checkMesh 2>&1

echo "=== Matched AMI Mesh Build Complete! ==="
tail -n 25 log.checkMesh
