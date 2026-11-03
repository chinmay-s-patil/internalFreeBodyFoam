#!/usr/bin/env python3
"""
setup_diagnostic_weight73um.py

Sets up a diagnostic restart simulation with disk mass and inertia scaled
to an effective thickness of h = 73 um (+46% mass and inertia, ~3 sigma
outside specimen spread).

Diagnostic Purpose & Confounding Factors:
- Evaluate whether the -17% settling deficit and -16% frequency deficit
  reflect a force/mass scale offset or a fundamental flaw in the flow/wake physics.
- Confounding: Increasing mass by +46% also raises I* from ~1.40 to ~2.04, and
  Re from ~137 to ~165. Any shift in frequency f could originate from the inertial
  ratio shift, wake mode transition, or velocity scaling. It is NOT a clean mass-offset test.
- Pre-Run Scaling Expectations:
  * Pure gravitational-inertial scaling predicts:
      V_expected ~ sqrt(1.46) * 1.02 m/s ~ 1.23 m/s
      f_expected ~ 36.9 Hz (f proportional to V at constant St)
  * Dynamic similarity band: f within +/- 3% (35.8 - 38.0 Hz).
  * If f approaches ~36-37 Hz as V -> 1.23 m/s, the simulation is dynamically similar
    and points to a force/mass scale discrepancy. If f remains near 30 Hz or diverges,
    the deficit stems from flow/wake modeling errors.
- Protocol & Timescale Relaxation:
  * A +46% weight step relaxes with characteristic velocity relaxation timescale:
      tau_rel ~ V_t / (2*g) ~ 1.23 / 19.62 ~ 0.063 s.
  * A short 0.16 s window (t in [0.30, 0.46]) covers only ~2.6 tau_rel.
  * Running to endTime = 0.60 s (t in [0.30, 0.60], duration 0.30 s ~ 4.8 tau_rel)
    covers ~9 complete revolutions, allowing proper asymptotic relaxation.
  * V_eq,est is evaluated on the final >= 3 revolutions (t in [0.50, 0.60] s),
    and per-revolution drift is reported.
- Strict Rule:
  * Even a pass demonstrates only "force/mass scale consistency", never why.
  * NEVER report this diagnostic as validation agreement or parameter tuning.
"""

import os
import shutil
import subprocess

def setup_diagnostic():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    src_case = os.path.join(base_dir, "restart_base")
    diag_case = os.path.join(base_dir, "restart_weight73um")
    
    print(f"Setting up diagnostic case: {diag_case}")
    if os.path.exists(diag_case):
        shutil.rmtree(diag_case)
        
    os.makedirs(os.path.join(diag_case, "constant"), exist_ok=True)
    
    # Copy system, Allrun, Allclean
    shutil.copytree(os.path.join(src_case, "system"), os.path.join(diag_case, "system"))
    shutil.copy(os.path.join(src_case, "Allrun"), diag_case)
    shutil.copy(os.path.join(src_case, "Allclean"), diag_case)
    
    # Copy constant files
    for f in ["dynamicMeshDict", "transportProperties", "turbulenceProperties"]:
        shutil.copy(os.path.join(src_case, "constant", f), os.path.join(diag_case, "constant", f))
        
    # Symlink polyMesh
    os.symlink(os.path.join(src_case, "constant", "polyMesh"), os.path.join(diag_case, "constant", "polyMesh"))
    
    # Copy restart time directory (0.3)
    shutil.copytree(os.path.join(src_case, "0.3"), os.path.join(diag_case, "0.3"))
    
    # Modify fallingFrameDict:
    # Nominal: h = 50 um, mass = 2.2113e-07 kg, I = (5.6436e-14 1.1278e-13 5.6436e-14)
    # Ratio: 73 / 50 = 1.46
    # Diagnostic: mass = 3.2285e-07 kg, I = (8.2397e-14 1.6466e-13 8.2397e-14)
    with open(os.path.join(src_case, "constant", "fallingFrameDict"), "r") as f:
        content = f.read()
        
    content = content.replace("mass                2.2113e-07;", "mass                3.2285e-07; // Scaled for h=73um diagnostic")
    content = content.replace(
        "inertiaPrincipal  (5.6436e-14 1.1278e-13 5.6436e-14);",
        "inertiaPrincipal  (8.2397e-14 1.6466e-13 8.2397e-14); // Scaled for h=73um diagnostic"
    )
    
    with open(os.path.join(diag_case, "constant", "fallingFrameDict"), "w") as f:
        f.write(content)
        
    # Ensure endTime covers >= 9 full revolutions (~0.30 s from t=0.30 -> endTime 0.60, ~4.8 tau_rel)
    subprocess.run(["foamDictionary", "-entry", "endTime", "-set", "0.60", os.path.join(diag_case, "system", "controlDict")], check=True)
        
    print("Updated fallingFrameDict with scaled mass (3.2285e-07 kg) and inertia (8.2397e-14, 1.6466e-13).")
    print("Set endTime to 0.60 s (duration 0.30 s ~ 4.8 tau_rel, ~9 revolutions; evaluate V_eq,est on final >= 3 revs).")
    print(f"Case ready at: {diag_case}")
    print("To execute diagnostic: cd " + diag_case + " && ./Allrun")

if __name__ == "__main__":
    setup_diagnostic()
