#!/usr/bin/env bash
# run_pipeline.sh: Autonomous execution of the decisive CFD/6DoF validation suite
# Sequence:
#   1. Finish restart_dtHalf (currently running, ~1.5h remaining)
#   2. warm_base to 0.20 s (baseline mesh control from warm start, ~3.5h)
#   3. warm_wakeFine to 0.20 s (487k refined wake mesh from warm start, ~10h)
#   4. Extension check: evaluate limit-cycle drift (0.3% threshold); extend to 0.26 s if triggered
#   5. restart_linUp (convection scheme test: linearUpwind vs LUST, ~3h)
#   6. Post-processing, final variant comparisons, pdflatex report compilation, artifact sync
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG="$SCRIPT_DIR/pipeline.log"

log() {
    local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $1"
    echo "$msg" | tee -a "$LOG"
}

log "=========================================================="
log "Starting CFD / 6DoF Decisive Validation Pipeline"
log "=========================================================="

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    if [ -f /usr/lib/openfoam/openfoam2412/etc/bashrc ]; then
        . /usr/lib/openfoam/openfoam2412/etc/bashrc
    fi
fi

# ==========================================================
# STAGE 1: Wait for restart_dtHalf to complete
# ==========================================================
log "STAGE 1: Monitoring running restart_dtHalf..."
DTHALF_DIR="$SCRIPT_DIR/restart_dtHalf"

is_dthalf_running() {
    if kill -0 35873 2>/dev/null; then
        return 0
    fi
    for pid in $(pidof freeBody6DoFPimpleFoam reconstructPar 2>/dev/null); do
        if pwdx "$pid" 2>/dev/null | grep -q "restart_dtHalf"; then
            return 0
        fi
    done
    return 1
}

while is_dthalf_running; do
    T_CUR=$(grep -oP 'Time = \K[0-9.]+' "$DTHALF_DIR/log.freeBody6DoFPimpleFoam" 2>/dev/null | tail -1 || echo "unknown")
    log "  restart_dtHalf in progress... current Time = $T_CUR"
    sleep 90
done

# Verify restart_dtHalf completed successfully
if ! grep -q "End" "$DTHALF_DIR/log.freeBody6DoFPimpleFoam"; then
    log "ERROR: restart_dtHalf did not reach 'End'!"
    exit 1
fi
log "Stage 1 COMPLETE: restart_dtHalf finished with 'End'."

# Record metrics
log "Evaluating complete-revolution restart_dtHalf metrics vs baseline:"
python3 "$SCRIPT_DIR/compare_variants.py" \
    "$BASE_DIR/validation/thinDisk2mm_tumbling/case" \
    "$SCRIPT_DIR/restart_base" \
    "$DTHALF_DIR" \
    --t-from 0.30 2>&1 | tee -a "$LOG"

# ==========================================================
# STAGE 2: Run warm_base to 0.20 s
# ==========================================================
log "=========================================================="
log "STAGE 2: Running warm_base (baseline mesh control from warm start)..."
WARM_BASE_DIR="$SCRIPT_DIR/warm_base"
cd "$WARM_BASE_DIR" || exit 1

if [ -f "$WARM_BASE_DIR/log.freeBody6DoFPimpleFoam" ] && grep -q "End" "$WARM_BASE_DIR/log.freeBody6DoFPimpleFoam"; then
    log "  warm_base already completed with 'End'. Skipping execution."
else
    log "  Executing warm_base/Allrun..."
    ./Allrun
    STATUS=$?

    if [ $STATUS -ne 0 ] || ! grep -q "End" log.freeBody6DoFPimpleFoam; then
        log "ERROR: warm_base failed with status $STATUS or did not reach 'End'!"
        exit 1
    fi
fi
log "Stage 2 COMPLETE: warm_base reached 0.20 s with 'End'."

# ==========================================================
# STAGE 3: Run warm_wakeFine to 0.20 s
# ==========================================================
log "=========================================================="
log "STAGE 3: Running warm_wakeFine (487k refined wake mesh from identical warm start)..."
WARM_WAKEFINE_DIR="$SCRIPT_DIR/warm_wakeFine"
cd "$WARM_WAKEFINE_DIR" || exit 1

if [ -f "$WARM_WAKEFINE_DIR/log.freeBody6DoFPimpleFoam" ] && grep -q "End" "$WARM_WAKEFINE_DIR/log.freeBody6DoFPimpleFoam"; then
    log "  warm_wakeFine already completed with 'End'. Skipping execution."
else
    log "  Executing warm_wakeFine/Allrun..."
    ./Allrun
    STATUS=$?

    if [ $STATUS -ne 0 ] || ! grep -q "End" log.freeBody6DoFPimpleFoam; then
        log "ERROR: warm_wakeFine failed with status $STATUS or did not reach 'End'!"
        exit 1
    fi
fi
log "Stage 3 COMPLETE: warm_wakeFine reached 0.20 s with 'End'."

# ==========================================================
# STAGE 4: Extension Check (0.3% limit-cycle drift rule)
# ==========================================================
log "=========================================================="
log "STAGE 4: Evaluating limit-cycle convergence (drift <= 0.3%)..."
cd "$SCRIPT_DIR" || exit 1

if python3 "$SCRIPT_DIR/check_extension.py" "$WARM_BASE_DIR" "$WARM_WAKEFINE_DIR" --t-from 0.08 --thresh 0.003 2>&1 | tee -a "$LOG"; then
    log "Both warm cases converged within 0.3% limit-cycle tolerance. No extension needed."
else
    log "Extension rule triggered! Extending both cases to 0.26 s (+2 revolutions)..."
    
    # Extend warm_base
    cd "$WARM_BASE_DIR" || exit 1
    foamDictionary -entry startFrom -set "latestTime" system/controlDict >/dev/null
    foamDictionary -entry endTime -set "0.26" system/controlDict >/dev/null
    log "  Extending warm_base to 0.26 s..."
    mpirun -np 8 --bind-to core freeBody6DoFPimpleFoam -parallel >> log.freeBody6DoFPimpleFoam 2>&1
    reconstructPar > log.reconstructPar 2>&1 || true

    # Extend warm_wakeFine
    cd "$WARM_WAKEFINE_DIR" || exit 1
    foamDictionary -entry startFrom -set "latestTime" system/controlDict >/dev/null
    foamDictionary -entry endTime -set "0.26" system/controlDict >/dev/null
    log "  Extending warm_wakeFine to 0.26 s..."
    mpirun -np 8 --bind-to core freeBody6DoFPimpleFoam -parallel >> log.freeBody6DoFPimpleFoam 2>&1
    reconstructPar > log.reconstructPar 2>&1 || true

    cd "$SCRIPT_DIR" || exit 1
    python3 "$SCRIPT_DIR/check_extension.py" "$WARM_BASE_DIR" "$WARM_WAKEFINE_DIR" --t-from 0.08 --thresh 0.003 2>&1 | tee -a "$LOG" || true
fi

# Compare warm_base vs warm_wakeFine over identical window
log "Decisive Comparison: warm_base vs warm_wakeFine (developed window):"
python3 "$SCRIPT_DIR/compare_variants.py" "$WARM_BASE_DIR" "$WARM_WAKEFINE_DIR" --t-from 0.08 2>&1 | tee -a "$LOG"

# ==========================================================
# STAGE 5: Run restart_linUp
# ==========================================================
log "=========================================================="
log "STAGE 5: Running restart_linUp (convection scheme sensitivity)..."
LINUP_DIR="$SCRIPT_DIR/restart_linUp"
cd "$LINUP_DIR" || exit 1

if [ -f "$LINUP_DIR/log.freeBody6DoFPimpleFoam" ] && grep -q "End" "$LINUP_DIR/log.freeBody6DoFPimpleFoam"; then
    log "  restart_linUp already completed with 'End'. Skipping execution."
else
    log "  Executing restart_linUp/Allrun..."
    ./Allrun
    STATUS=$?

    if [ $STATUS -ne 0 ] || ! grep -q "End" log.freeBody6DoFPimpleFoam; then
        log "ERROR: restart_linUp failed with status $STATUS or did not reach 'End'!"
        exit 1
    fi
fi
log "Stage 5 COMPLETE: restart_linUp finished with 'End'."

# Compare scheme sensitivity
cd "$SCRIPT_DIR" || exit 1
log "Scheme Sensitivity Comparison: restart_base (LUST) vs restart_linUp (linearUpwind):"
python3 "$SCRIPT_DIR/compare_variants.py" "$SCRIPT_DIR/restart_base" "$LINUP_DIR" --t-from 0.30 2>&1 | tee -a "$LOG"

# ==========================================================
# STAGE 6: Compile Report and Update Artifacts
# ==========================================================
log "=========================================================="
log "STAGE 6: Recompiling validation report and updating artifacts..."
cd "$BASE_DIR/validation" || exit 1
pdflatex -interaction=nonstopmode report.tex > log.pdflatex 2>&1
pdflatex -interaction=nonstopmode report.tex > log.pdflatex 2>&1

ARTIFACT_DIR="${BRAIN_DIR:-$(find "$HOME/.gemini" -type d -name "ee694246-5ea1-4c2c-8330-99d3c3bc15c9" 2>/dev/null | head -1)}"
if [ -n "$ARTIFACT_DIR" ] && [ -d "$ARTIFACT_DIR" ]; then
    cp report.pdf "$ARTIFACT_DIR/report.pdf" 2>/dev/null || true
    cp report.pdf "$ARTIFACT_DIR/thin_disk_validation_report.pdf" 2>/dev/null || true
    cp report.md "$ARTIFACT_DIR/report.md" 2>/dev/null || true
    cp report.md "$ARTIFACT_DIR/thin_disk_validation_report.md" 2>/dev/null || true
    log "  Artifacts successfully copied to $ARTIFACT_DIR."
fi

log "=========================================================="
log "ALL 4 SIMULATIONS AND ANALYSES COMPLETED SUCCESSFULLY!"
log "=========================================================="
