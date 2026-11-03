#!/bin/bash
# make_restart_variants.sh <SRC_CASE> <T0> [variant ...]
# Restarts the tumbling case from reconstructed time dir T0 (e.g. 0.3) with one numerical change per variant.
# Variants: base (control, unchanged)  dtHalf (maxCo 0.75)  dtHalfOuter2 (maxCo 0.75 + nOuterCorrectors 2)
#           linUp (div(phi,U) LUST -> linearUpwind)
# Env: SPAN=<s simulated after T0, default 0.14 ~ 4 revs>  RUN=1 runs all variants sequentially
set -euo pipefail
cd "${0%/*}"
SRC=$(realpath "${1:?usage: $0 <SRC_CASE> <T0> [variant ...]}"); T0=${2:?}; shift 2
VARIANTS=("$@"); [ ${#VARIANTS[@]} -eq 0 ] && VARIANTS=(base dtHalf dtHalfOuter2 linUp)
SPAN=${SPAN:-0.14}
[ -n "${WM_PROJECT_DIR:-}" ] || . /usr/lib/openfoam/openfoam2412/etc/bashrc
END=$(python3 -c "print(f'{$T0+$SPAN:.6f}')")
[ -d "$SRC/$T0" ] || { echo "missing $SRC/$T0 (run reconstructPar -time $T0 first)"; exit 1; }
[ -d "$SRC/$T0/uniform" ] || { echo "missing $SRC/$T0/uniform (frame velocity + attitude restart state)"; exit 1; }

for v in "${VARIANTS[@]}"; do
    C=restart_$v
    rm -rf "$C"; mkdir -p "$C/constant"
    cp -r "$SRC/system" "$C/"
    cp "$SRC"/constant/{fallingFrameDict,dynamicMeshDict,transportProperties,turbulenceProperties} "$C/constant/"
    ln -s "$SRC/constant/polyMesh" "$C/constant/polyMesh"           # shared mesh, never written to
    cp -r "$SRC/$T0" "$C/"                                         # fields + rotated points + uniform/ state
    cp "$SRC/Allrun" "$SRC/Allclean" "$C/"
    sed -i "s/decomposePar -force/decomposePar -force -time $T0/" "$C/Allrun"   # only decompose the restart time

    CD=$C/system/controlDict
    foamDictionary -entry startFrom -set startTime "$CD" >/dev/null
    foamDictionary -entry startTime -set "$T0" "$CD" >/dev/null
    foamDictionary -entry endTime   -set "$END" "$CD" >/dev/null
    case $v in
        base) ;;                                                    # control: re-run with identical settings
        dtHalf)       foamDictionary -entry maxCo -set 0.75 "$CD" >/dev/null ;;
        dtHalfOuter2) foamDictionary -entry maxCo -set 0.75 "$CD" >/dev/null
                      foamDictionary -entry PIMPLE/nOuterCorrectors -set 2 "$C/system/fvSolution" >/dev/null ;;
        linUp)        sed -i -E 's/(div\(phi,U\)\s+Gauss )LUST( grad\(U\);)/\1linearUpwind\2/' "$C/system/fvSchemes"
                      grep -q linearUpwind "$C/system/fvSchemes" || { echo "linUp edit failed"; exit 1; } ;;
        *) echo "unknown variant $v"; exit 1 ;;
    esac
    echo "built $C  (start $T0 -> end $END)"
done

if [ "${RUN:-0}" = 1 ]; then
    for v in "${VARIANTS[@]}"; do (cd "restart_$v" && ./Allrun); done
else
    echo "Run each:  for v in ${VARIANTS[*]}; do (cd restart_\$v && ./Allrun); done"
fi
echo "Compare:  python3 compare_variants.py ${VARIANTS[*]/#/restart_} --t-from $(python3 -c "print($T0+0.02)")"
