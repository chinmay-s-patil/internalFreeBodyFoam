#!/bin/bash
# ==============================================================================
# run_overnight_validation.sh
# 
# Master sequential automation pipeline for ASME V&V 20 verification & validation:
# Runs all required simulations one after another on 8 cores.
# 
# Sequence:
#   [Priority 2: Independent Drag Checks]
#     1. fixedDisk_Re200     (broadside disk at Re=200 vs Roos & Willmarth 1971)
#     2. fixedSphere_Re100   (sphere at Re=100 vs Schiller-Naumann & Clift)
#     3. fixedSphere_Re200   (sphere at Re=200 vs Schiller-Naumann & Clift)
#   [Priority 3: Three-Mesh GCI for Steady Re=100 Fixed Disk]
#     4. gci_coarse_Re100    (N3 = 130,928 cells)
#     5. gci_medium_Re100    (N2 = 212,792 cells)
#     6. gci_fine_Re100      (N1 = 368,760 cells)
#     7. compute_steady_gci  (ASME V&V 20 spatial uncertainty u_num)
#   [Optional Diagnostic Run]
#     8. restart_weight73um  (heavier diagnostic weight step)
# ==============================================================================
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
BASE_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
LOG="$BASE_DIR/validation/overnight_pipeline.log"
exec > >(tee -a "$LOG") 2>&1

echo "================================================================================"
echo "Starting Overnight Validation Pipeline at $(date)"
echo "Host: $(hostname), Cores: 8 physical / 16 threads"
echo "================================================================================"

log_step() {
    echo ""
    echo "================================================================================"
    echo ">>> STEP: $1 at $(date)"
    echo "================================================================================"
}

# ------------------------------------------------------------------------------
# 1. Broadside Disk at Re = 200
# ------------------------------------------------------------------------------
log_step "1/7: Running fixedDisk_Re200 (Roos & Willmarth 1971 benchmark)"
cd "$BASE_DIR/validation/baseline_drag/fixedDisk_Re200"
./Allrun
echo "Finished fixedDisk_Re200. Running analysis..."
cd "$BASE_DIR"
python3 validation/baseline_drag/analyze_fixed_disk.py validation/baseline_drag/fixedDisk_Re200 --Re 200 --force-preliminary || true

# ------------------------------------------------------------------------------
# 2. Fixed Sphere at Re = 100
# ------------------------------------------------------------------------------
log_step "2/7: Running fixedSphere_Re100 (Schiller-Naumann / Clift benchmark)"
cd "$BASE_DIR/validation/baseline_drag/fixedSphere_Re100"
./Allrun
echo "Finished fixedSphere_Re100. Running analysis..."
cd "$BASE_DIR"
python3 validation/baseline_drag/analyze_fixed_sphere.py validation/baseline_drag/fixedSphere_Re100 --Re 100 || true

# ------------------------------------------------------------------------------
# 3. Fixed Sphere at Re = 200
# ------------------------------------------------------------------------------
log_step "3/7: Running fixedSphere_Re200 (Schiller-Naumann / Clift benchmark)"
cd "$BASE_DIR/validation/baseline_drag/fixedSphere_Re200"
./Allrun
echo "Finished fixedSphere_Re200. Running analysis..."
cd "$BASE_DIR"
python3 validation/baseline_drag/analyze_fixed_sphere.py validation/baseline_drag/fixedSphere_Re200 --Re 200 || true

# ------------------------------------------------------------------------------
# 4. GCI Coarse Grid (N3 = 130,928) at Re = 100
# ------------------------------------------------------------------------------
log_step "4/7: Running gci_coarse_Re100 (Grid 3)"
cd "$BASE_DIR/validation/mesh_convergence/gci_coarse_Re100"
./Allrun

# ------------------------------------------------------------------------------
# 5. GCI Medium Grid (N2 = 212,792) at Re = 100
# ------------------------------------------------------------------------------
log_step "5/7: Running gci_medium_Re100 (Grid 2)"
cd "$BASE_DIR/validation/mesh_convergence/gci_medium_Re100"
./Allrun

# ------------------------------------------------------------------------------
# 6. GCI Fine Grid (N1 = 368,760) at Re = 100
# ------------------------------------------------------------------------------
log_step "6/7: Running gci_fine_Re100 (Grid 1)"
cd "$BASE_DIR/validation/mesh_convergence/gci_fine_Re100"
./Allrun

# ------------------------------------------------------------------------------
# 7. Evaluate Formal ASME V&V 20 Spatial GCI
# ------------------------------------------------------------------------------
log_step "7/7: Computing ASME V&V 20-2009 Spatial GCI"
cd "$BASE_DIR"
python3 validation/mesh_convergence/compute_steady_gci.py || true

# ------------------------------------------------------------------------------
# Optional 8. Diagnostic 73 um weight step
# ------------------------------------------------------------------------------
if [ -d "$BASE_DIR/validation/baseline_drag/restart_weight73um" ]; then
    log_step "Optional 8: Running restart_weight73um"
    cd "$BASE_DIR/validation/baseline_drag/restart_weight73um"
    if [ -f "Allrun" ]; then
        ./Allrun || true
    fi
fi

echo ""
echo "================================================================================"
echo "Overnight Validation Pipeline Finished Successfully at $(date)"
echo "================================================================================"
