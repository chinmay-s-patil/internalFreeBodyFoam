#!/bin/bash
# ==============================================================================
# run_benchmark_pipeline.sh
# Sequential execution of:
# 1. Cheap first step: fixedSphere_Re100 extension to 100 s (3 flow-throughs)
# 2. Gate Run 1: gate_fixed_noAMI (Re = 185.08, control without AMI, 50 s)
# 3. Gate Run 2: gate_fixed_AMI (Re = 185.08, matched AMI interface, 50 s)
# 4. Uhlmann Case A: caseA_free_G144 (Free 6DoF settling, 35 s)
# ==============================================================================
set -eo pipefail

BASE_DIR="/home/lavender/OpenFoamUbu/solvers/freeBodyFoam"
UHLMANN_DIR="$BASE_DIR/validation/uhlmann_sphere"
LOG_FILE="$UHLMANN_DIR/pipeline.log"

if [ -z "${WM_PROJECT_DIR:-}" ]; then
    . /usr/lib/openfoam/openfoam2412/etc/bashrc
fi

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log "================================================================================"
log "STARTING UHLMANN & DUSEK (2014) BENCHMARK & GATE VALIDATION PIPELINE"
log "================================================================================"

# ------------------------------------------------------------------------------
# STEP 1: Cheap first step (fixedSphere_Re100 extension to 100 s)
# ------------------------------------------------------------------------------
log "STEP 1/4: Running fixedSphere_Re100 extension to 100 s (~3 flow-throughs)..."
cd "$BASE_DIR/validation/baseline_drag/fixedSphere_Re100"

# Run parallel solver from latestTime to 100.0 s
mpirun -np 8 freeBody6DoFPimpleFoam -parallel >> log.freeBody6DoFPimpleFoam 2>&1 || true

log "Reconstructing latest time for fixedSphere_Re100..."
reconstructPar -latestTime >> log.reconstructPar 2>&1 || true

log "Analyzing fixedSphere_Re100 at t = 100 s..."
python3 "$BASE_DIR/validation/baseline_drag/analyze_fixed_sphere.py" . --Re 100 | tee -a "$LOG_FILE" || true

# ------------------------------------------------------------------------------
# STEP 2: Gate Run - Control (gate_fixed_noAMI, Re = 185.08, 50 s)
# ------------------------------------------------------------------------------
log "================================================================================"
log "STEP 2/4: Running gate_fixed_noAMI (Control, pure body-fitted, Re = 185.08, 50 s)..."
cd "$UHLMANN_DIR/gate_fixed_noAMI"

mpirun -np 8 freeBody6DoFPimpleFoam -parallel > log.freeBody6DoFPimpleFoam 2>&1 || true

log "Analyzing gate_fixed_noAMI..."
python3 "$UHLMANN_DIR/analyze_gate_run.py" . --label "gate_fixed_noAMI (Control, No AMI)" | tee -a "$LOG_FILE" || true

# ------------------------------------------------------------------------------
# STEP 3: Gate Run - Matched AMI (gate_fixed_AMI, Re = 185.08, 50 s)
# ------------------------------------------------------------------------------
log "================================================================================"
log "STEP 3/4: Running gate_fixed_AMI (Matched AMI, 1:1 area ratio, Re = 185.08, 50 s)..."
cd "$UHLMANN_DIR/gate_fixed_AMI"

mpirun -np 8 freeBody6DoFPimpleFoam -parallel > log.freeBody6DoFPimpleFoam 2>&1 || true

log "Analyzing gate_fixed_AMI..."
python3 "$UHLMANN_DIR/analyze_gate_run.py" . --label "gate_fixed_AMI (Matched AMI)" | tee -a "$LOG_FILE" || true

# ------------------------------------------------------------------------------
# STEP 4: Case A Free Settling (caseA_free_G144, G = 144, Free 6DoF, 35 s)
# ------------------------------------------------------------------------------
log "================================================================================"
log "STEP 4/4: Running caseA_free_G144 (Free 6DoF, G = 144, 35 s)..."
cd "$UHLMANN_DIR/caseA_free_G144"

rm -rf processor* log.*
decomposePar > log.decomposePar 2>&1 || true
mpirun -np 8 freeBody6DoFPimpleFoam -parallel > log.freeBody6DoFPimpleFoam 2>&1 || true

log "Analyzing caseA_free_G144..."
python3 "$UHLMANN_DIR/analyze_caseA_free.py" . | tee -a "$LOG_FILE" || true

log "================================================================================"
log "ALL RUNS IN PIPELINE COMPLETED AT $(date)"
log "================================================================================"
