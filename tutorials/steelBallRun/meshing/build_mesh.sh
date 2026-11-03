#!/bin/bash
set -euo pipefail
cd "${0%/*}"

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    . /usr/lib/openfoam/openfoam2412/etc/bashrc
fi

echo "=== 1. Generating Surface STLs ==="
python3 generate_sphere_stls.py

echo "=== 2. Building rotatingMesh ==="
cd rotatingMesh
surfaceFeatureExtract > log.surfaceFeatureExtract 2>&1
blockMesh > log.blockMesh 2>&1
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1
checkMesh > log.checkMesh 2>&1
echo "rotatingMesh completed."

cd ../staticMesh
echo "=== 3. Building staticMesh ==="
surfaceFeatureExtract > log.surfaceFeatureExtract 2>&1
blockMesh > log.blockMesh 2>&1
snappyHexMesh -overwrite > log.snappyHexMesh 2>&1
checkMesh > log.checkMesh 2>&1
echo "staticMesh completed."

cd ..
echo "=== 4. Merging Meshes into Case ==="
CASE_DIR="$(pwd)/.."
rm -rf "${CASE_DIR}/constant/polyMesh"
cp -r staticMesh/constant/polyMesh "${CASE_DIR}/constant/"

cd "${CASE_DIR}"
mergeMeshes . meshing/rotatingMesh -overwrite > log.mergeMeshes 2>&1
topoSet > log.topoSet 2>&1
createPatch -overwrite > log.createPatch 2>&1
checkMesh > log.checkMesh 2>&1

echo "=== 3D Spherical AMI Mesh Build Complete! ==="
tail -n 6 log.checkMesh
