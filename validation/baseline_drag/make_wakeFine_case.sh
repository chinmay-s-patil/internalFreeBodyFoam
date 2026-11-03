#!/bin/bash
# make_wakeFine_case.sh [CASE_ROOT]       (CASE_ROOT default ../thinDisk2mm_tumbling; needs built meshing/rotatingMesh)
# Builds two warm-start cases that differ ONLY in the static (outer) mesh:
#   warm_base      original static mesh
#   warm_wakeFine  static mesh refined near the AMI sphere (level 4 ~ 78 um, matches the 57 um rotating faces)
#                  and in a wake box downstream of the disk (level 2 ~ 312 um)
# Both start from the developed state (u0 = V_START, uniform relative flow, omega = OMEGA0), so a short run
# (default 0.30 s) is enough; compare them with compare_variants.py --t-from 0.15.
# The wake-box position follows the drift seen in the tumbling run (relative flow +Y and -Z).
set -euo pipefail
cd "${0%/*}"
ROOT=$(realpath "${1:-../thinDisk2mm_tumbling}")
V_START=${V_START:-1.02}; OMEGA0=${OMEGA0:--191}; END=${END:-0.30}
ROT=$ROOT/meshing/rotatingMesh; STAT=$ROOT/meshing/staticMesh
[ -n "${WM_PROJECT_DIR:-}" ] || . /usr/lib/openfoam/openfoam2412/etc/bashrc
[ -f "$ROT/constant/polyMesh/faces" ] || { echo "rotatingMesh not built: $ROT/constant/polyMesh/faces missing"; exit 1; }

warm_case()   # warm_case <name> : dicts + warm-start edits, mesh left to the caller
{
    local C=$1
    rm -rf "$C"; mkdir -p "$C/constant" "$C/0"
    cp -r "$ROOT/case/system" "$C/"
    cp "$ROOT"/case/constant/{fallingFrameDict,dynamicMeshDict,transportProperties,turbulenceProperties} "$C/constant/"
    cp "$ROOT"/case/0/{U,p_rgh} "$C/0/"
    cp "$ROOT/case/Allrun" "$ROOT/case/Allclean" "$C/"
    local FF=$C/constant/fallingFrameDict
    foamDictionary -entry u0 -set "(0 -$V_START 0)" "$FF" >/dev/null
    foamDictionary -entry initialAcceleration -set "(0 0 0)" "$FF" >/dev/null   # no spurious g at the first step
    foamDictionary -entry rotor/initialOmega -set "($OMEGA0 0 0)" "$FF" >/dev/null
    foamDictionary -entry internalField -set "uniform (0 $V_START 0)" "$C/0/U" >/dev/null
    foamDictionary -entry endTime -set "$END" "$C/system/controlDict" >/dev/null
}

# control: original mesh
warm_case warm_base
ln -s "$ROOT/case/constant/polyMesh" warm_base/constant/polyMesh

# refined static mesh
rm -rf wakeFine_static; mkdir -p wakeFine_static
cp -r "$STAT/system" "$STAT/constant" wakeFine_static/
rm -rf wakeFine_static/constant/polyMesh
python3 - wakeFine_static/system/snappyHexMeshDict <<'PY'
import re, sys
p = sys.argv[1]; s = open(p).read(); n0 = s
s = re.sub(r"(geometry\s*\{)", r"\1\n    wakeBox { type searchableBox; min (-0.004 -0.004 -0.012); max (0.004 0.024 0.006); }", s, count=1)
s = re.sub(r"(refinementRegions\s*\{)", r"\1\n        wakeBox { mode inside; levels ((1e15 2)); }", s, count=1)
s = s.replace("level (2 3);", "level (4 4);")                                           # AMI sphere surface cells ~78 um
s = s.replace("levels ((0.002 2) (0.005 1));", "levels ((0.0005 4) (0.002 3) (0.005 2) (0.012 1));")
assert s.count("wakeBox") == 2 and "(4 4)" in s and "(0.012 1)" in s, "snappyHexMeshDict edit did not apply"
open(p, "w").write(s)
PY
( cd wakeFine_static && blockMesh > log.blockMesh && snappyHexMesh -overwrite > log.snappyHexMesh && checkMesh > log.checkMesh && tail -3 log.checkMesh )

# merge with the unchanged rotating mesh, exactly as combiMesh/Allmesh does
warm_case warm_wakeFine
cp -r wakeFine_static/constant/polyMesh warm_wakeFine/constant/
( cd warm_wakeFine && mergeMeshes . "$ROT" -overwrite > log.mergeMeshes && createPatch -overwrite > log.createPatch \
  && checkMesh > log.checkMesh 2>&1 && topoSet > log.topoSet )
echo "AMI patches (slaveAMI = static side; original had 456 faces, rotating side 15528):"
grep -A3 -E "(master|slave)AMI" warm_wakeFine/constant/polyMesh/boundary | grep -E "AMI|nFaces"
grep -E "cells:|Mesh OK|Max non-orth" warm_wakeFine/log.checkMesh || true
echo "Run:  (cd warm_base && ./Allrun); (cd warm_wakeFine && ./Allrun)"
echo "Compare:  python3 compare_variants.py warm_base warm_wakeFine --t-from 0.15"
