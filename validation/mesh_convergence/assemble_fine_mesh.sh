#!/bin/bash
. /usr/lib/openfoam/openfoam2412/etc/bashrc

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "$SCRIPT_DIR/../.." && pwd)
BASE_DIR="$SCRIPT_DIR/fineMesh"

echo "=== 1. Building fine staticMesh ==="
cd "$BASE_DIR/staticMesh"
blockMesh
surfaceFeatureExtract
snappyHexMesh -overwrite

echo "=== 2. Setting up fine case ==="
mkdir -p "$BASE_DIR/case/constant"
cp -r "$BASE_DIR/staticMesh/constant/polyMesh" "$BASE_DIR/case/constant/"
cp -r "$REPO_DIR/validation/thinDisk2mm/case/system" "$BASE_DIR/case/"
cp "$REPO_DIR/validation/thinDisk2mm/case/constant/transportProperties" "$BASE_DIR/case/constant/"
cp "$REPO_DIR/validation/thinDisk2mm/case/constant/turbulenceProperties" "$BASE_DIR/case/constant/"
cp "$REPO_DIR/validation/thinDisk2mm/case/constant/fallingFrameDict" "$BASE_DIR/case/constant/"

# Copy geometric topoSetDict
cp "$SCRIPT_DIR/coarseMesh/case/system/topoSetDict" "$BASE_DIR/case/system/"

echo "=== 3. Merging meshes ==="
cd "$BASE_DIR/case"
mergeMeshes . ../rotatingMesh -overwrite
topoSet
createPatch -overwrite
checkMesh > log.checkMesh
echo "Fine mesh assembly complete!"
