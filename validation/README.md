# Validation & Code Verification Suite — `freeBodyFoam`

This directory contains the numerical benchmarks, code verification scripts, grid and temporal convergence studies, and reduced telemetry datasets for validating the coupled 6DoF non-inertial CFD solver `freeBody6DoFPimpleFoam`.

---

## 1. Reproducibility Policy & Architecture

### The Reproducibility Gap Resolved
Previous releases excluded `validation/code_verification/`, `validation/temporal_convergence/`, `validation/thinDisk2mm*/`, and `postProcessing/` output via `.gitignore`. Consequently, downstream users could not independently reproduce the verification numbers supporting the **"Code Verification Complete"** status:
- Explicit RK4 asymptotic order ($p = 3.9910$)
- Ballistic vacuum invariant conservation (energy drift $< 10^{-14}$)
- ASME V&V 20 Temporal Discretization Grid Convergence Index ($p = 1.566$, $\mathrm{GCI} = 0.045\%$)
- The 13-revolution tumbling limit cycle and per-revolution kinematics ($t \in [0, 0.500]\text{ s}$)

To resolve this while keeping the core repository fast and lean:
1. **Bundled in Repository (~27 MB):** All Python verification drivers, plotting scripts, case dictionaries, and full 6DoF kinematic telemetry logs (`fallingMotion/motion.dat` and `tumbleMotion/motion.dat` across all 29,715 time steps) are tracked directly in Git. All four verification metrics can be reproduced in seconds without external downloads.
2. **Archived on Zenodo (6.4 GB + 25 GB):** The raw 3D OpenFOAM volume fields ($U, p_{\mathrm{rgh}}, \phi$, and decomposed `processor0–7` subdomains) across all saved simulation checkpoints are archived as an open-access Zenodo dataset for complete volumetric reconstruction.

---

## 2. Quick-Start Reproduction Commands

All verification numbers reported in [`report.md`](./report.md), [`report.pdf`](./report.pdf), and [`docs/validation.md`](../docs/validation.md) can be executed directly:

### 2.1 RK4 Convergence Order ($p = 3.991$) & Energy Drift ($< 10^{-14}$)
Tests the mathematical implementation of Euler's rigid-body angular velocity equations across systematic time-step refinements and evaluates machine-precision conservation of invariants over 13,333 steps in vacuum:

```bash
python3 validation/code_verification/verify_6dof_integrator.py
```

**Expected Verified Output:**
```text
1A. Asymmetric Rigid-Body Precession — Systematic Time-Step Convergence:
Time Step dt (s)   Steps    L2 Error (rad/s)       Observed Order p   Theoretical Order 
-------------------------------------------------------------------------------------
2.00e-03           25       7.04047110e-03         Baseline           4.0000            
1.00e-03           50       4.50240103e-04         3.9669             4.0000            
5.00e-04           100      2.84811476e-05         3.9826             4.0000            
2.50e-04           200      1.79118283e-06         3.9910             4.0000            
-------------------------------------------------------------------------------------
Result: Mathematical formulation achieves asymptotic convergence order p = 3.9910

1B. Conservation of Physical Invariants (Thin Disk Geometry over 200 ms, 13,333 steps):
  Relative Angular Momentum Drift   : 1.90e-15 (Machine precision)
  Relative Kinetic Energy Drift     : 4.15e-15 (Machine precision)

1C. Translating Reference Frame Galilean Invariance (Zero-Drag Limit):
  Simulated Frame Acceleration a_frame : 9.81 m/s^2 (|a_frame - g| = 0.00 m/s^2)
  Relative Integration Drift           : 2.85e-13 (Double-precision roundoff)
```

---

### 2.2 13-Revolution Tumbling Run Kinematics & Energy Balance
Audits C++ rigid-body dynamic coupling, vertical frame acceleration balance, Heun velocity integration, work-energy equivalence, and cycle-by-cycle kinematic evolution over all 13 revolutions ($t = 0 \to 0.500\text{ s}$, 29,715 time steps):

```bash
python3 validation/code_verification/check_run_consistency.py \
    validation/thinDisk2mm_tumbling/case/postProcessing/tumbleMotion/motion.dat \
    validation/thinDisk2mm_tumbling/case/postProcessing/fallingMotion/motion.dat
```

**Expected Verified Output:**
```text
Rigid-body consistency of logged states
  [PASS] I*dw/dt vs Mx, rel. to max|Mx|: 2.514e-06 (tol 1e-04)
  [PASS] a_y vs g + Faero_y/m, abs [m/s2]: 7.199e-09 (tol 1e-06)
  [PASS] Vy vs trapezoid-integrated a_y, abs [m/s]: 1.222e-09 (tol 1e-06)
  [PASS] angle_x vs integral of omega_x, abs [deg]: 2.552e-01 (tol 5e-01)
  [PASS] mech. energy vs aero work, rel.: 3.194e-05 (tol 1e-03)

Per-revolution means
  rev   t_mid[ms]  f[Hz]   <Vy>[m/s]  <Fy>/W   psi[deg]
    1      15.2   32.93     0.1486   0.0040     0.05
    2      48.5   27.58     0.4720   0.0233     0.20
    3      89.0   22.38     0.8364   0.2130     0.83
    4     136.6   19.75     1.0860   0.7429     3.32
    5     182.9   23.84     1.1272   1.0088     7.32
    6     222.2   27.37     1.1111   1.0562    10.17
    7     257.3   29.56     1.0868   1.0737    12.18
    8     290.7   30.50     1.0640   1.0566    13.71
    9     323.3   30.73     1.0487   1.0368    14.69
   10     355.9   30.69     1.0387   1.0238    15.29
   11     388.5   30.55     1.0324   1.0147    15.65
   12     421.4   30.39     1.0286   1.0085    15.86
   13     454.4   30.24     1.0264   1.0046    15.97

Aitken asymptote of <Vy> (last 3 revs): 1.0236 m/s
Exp-relaxation fit: Vinf = 1.0159 m/s, tau = 97 ms, run length = 500 ms
```

---

### 2.3 Temporal Discretization GCI (ASME V&V 20)
Evaluates temporal grid convergence across three time steps ($dt = 2.0\times 10^{-5}\text{ s}, 1.0\times 10^{-5}\text{ s}, 5.0\times 10^{-6}\text{ s}$ with refinement ratio $r_t = 2.0$) at $t = 0.001000\text{ s}$:

```bash
python3 validation/temporal_convergence/compute_temporal_gci.py
```

**Expected Verified Output:**
```text
Target QOI: Aerodynamic Drag Force Faero_y (nN)
  Coarse dt (Grid 3)  phi3 = 2.611393
  Medium dt (Grid 2)  phi2 = 2.616813
  Fine dt   (Grid 1)  phi1 = 2.618644
  Differences: eps21 = +1.830659e-03, eps32 = +5.419949e-03
  Convergence Ratio R = eps21 / eps32 = +0.3378
  Regime: MONOTONIC CONVERGENCE
  Unclamped Apparent Order p = 1.566
  Richardson Extrapolated Value = 2.619578
  Approximate Relative Error e_a = 0.070%
  Fine dt GCI21 = 0.045%
  Medium dt GCI32 = 0.132%
  Asymptotic Range Index AR = 1.001 (Ideal = 1.000)
```

---

### 2.4 High-Resolution 6-Panel Validation Figure
Regenerates the publication-quality comparison against Tinklenberg et al. (2023) experimental envelopes:

```bash
python3 validation/thinDisk2mm_tumbling/plot_tumbling_detailed.py
```
Output: `validation/thinDisk2mm_tumbling/tumbling_validation_detailed.png`

---

## 3. Zenodo Archival Dataset (6.4 GB + 25 GB 3D CFD Data)

For researchers requiring raw volumetric OpenFOAM fields ($U$, $p_{\mathrm{rgh}}$, $\phi$, dynamic mesh point displacements, and decomposed `processor*` directories) to perform 3D flow visualization, vortex identification ($Q$-criterion), or interface flux re-integration, full simulation archives can be packaged and deposited on Zenodo.

- **Dataset Status:** Deposit Pending (Awaiting dataset upload to mint permanent DOI)
- **Suggested Title:** `6DoF CFD Validation Data for Falling Thin Disks in Air (freeBody6DoFPimpleFoam)`
- **DOI Placeholder:** `10.5281/zenodo.<PENDING_DEPOSIT_ID>`
- **License:** Creative Commons Attribution 4.0 International (CC-BY 4.0)

### Archive Manifest

| Archive File | Unpacked Size | Contents |
| :--- | :---: | :--- |
| `thinDisk2mm_zeroSpin_transient_6.4GB.tar.gz` | **6.4 GB** | Preserved transient drop run ($t = 0 \to 0.235\text{ s}$). Contains `polyMesh`, all time directories, and `processor0–7`. |
| `thinDisk2mm_tumbling_13rev_25GB.tar.gz` | **25.2 GB** | Continuous 13-revolution seeded tumbling run ($t = 0 \to 0.500\text{ s}$, 29,715 time steps). Contains complete checkpoint series and logs. |
| `temporal_convergence_cases.tar.gz` | **293 MB** | Complete meshes and $t = 0.001\text{ s}$ volume states for `dt_baseline`, `dt_halved`, and `dt_quarter`. |

### Packaging the Dataset for Zenodo Upload
To create the archive tarballs for uploading to Zenodo:

```bash
# 1. Package 6.4 GB transient drop dataset
tar -czf thinDisk2mm_zeroSpin_transient_6.4GB.tar.gz -C validation/thinDisk2mm case

# 2. Package 25 GB tumbling dataset
tar -czf thinDisk2mm_tumbling_13rev_25GB.tar.gz -C validation/thinDisk2mm_tumbling case

# 3. Package temporal convergence cases
tar -czf temporal_convergence_cases.tar.gz -C validation temporal_convergence/dt_baseline temporal_convergence/dt_halved temporal_convergence/dt_quarter
```

After uploading to Zenodo and reserving/publishing the record, replace `10.5281/zenodo.<PENDING_DEPOSIT_ID>` with your assigned DOI.

---

## 4. Directory Structure

```text
validation/
├── README.md                           # This reproducibility guide & Zenodo specification
├── report.md                           # Comprehensive validation report (Markdown)
├── report.pdf                           # Formal 11-page compiled PDF validation report
├── report.tex                          # LaTeX source for formal validation report
├── code_verification/
│   ├── check_run_consistency.py        # 6DoF C++ dynamics, energy balance & asymptotic extrapolation
│   └── verify_6dof_integrator.py        # RK4 order (p=3.991), vacuum invariants & C++ solver checks
├── temporal_convergence/
│   ├── compute_temporal_gci.py         # ASME V&V 20 time-step convergence script
│   ├── setup_temporal.py               # Case generation script
│   ├── dt_baseline/postProcessing/     # Coarse dt telemetry (motion.dat)
│   ├── dt_halved/postProcessing/       # Medium dt telemetry (motion.dat)
│   └── dt_quarter/postProcessing/      # Fine dt telemetry (motion.dat)
├── mesh_convergence/                   # 3-mesh spatial grid convergence study
│   ├── compute_gci.py                  # ASME V&V 20 spatial GCI script
│   └── compute_steady_gci.py           # Steady Re=100 spatial GCI script
├── thinDisk2mm/                        # Preserved transient drop benchmark
│   ├── plot_validation.py              # Telemetry parsing & validation plotting
│   ├── validation_comparison.png       # 4-panel experimental validation curves
│   └── case/postProcessing/            # 6DoF translational & rotational telemetry (motion.dat)
├── thinDisk2mm_tumbling/               # 13-revolution seeded tumbling benchmark (t = 0.500 s)
│   ├── plot_tumbling_detailed.py       # 6-panel publication comparison script
│   ├── calculate_cycles.py             # Cycle frequency and velocity calculator
│   ├── tumbling_validation_detailed.png# 6-panel validation curves (13 full turns)
│   └── case/postProcessing/            # Full 29,715-step 6DoF motion telemetry (motion.dat)
├── baseline_drag/                      # Fixed broadside disk & sphere drag benchmarks
│   ├── analyze_fixed_disk.py           # Force coefficient & stability analyzer
│   └── compare_variants.py             # Developed-state restart variant comparator
└── validation_check/                   # Flat symlink package for rapid reproduction
```
