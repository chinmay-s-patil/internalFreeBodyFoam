# Validation Check Package — `freeBody6DoFPimpleFoam`

This folder contains a consolidated collection of **symbolic links** to all reports, experimental benchmark papers, 6DoF simulation telemetry data, solver logs, and analysis scripts needed to review and reproduce the validation of `freeBody6DoFPimpleFoam`.

---

## 1. File Manifest & Symlink Directory

| Symlink Name | Target Path | Description |
| :--- | :--- | :--- |
| [`report.md`](./report.md) | `../validation/report.md` | **Primary Comprehensive Validation Report (Markdown)** detailing physics, 13-revolution data, ASME V&V 20 uncertainty, and peer-review resolutions. |
| [`report.pdf`](./report.pdf) | `../validation/report.pdf` | **Publication-Quality Formal PDF Validation Report** compiled with `pdflatex` (11 pages, full equations, figures, and tables). |
| [`report.tex`](./report.tex) | `../validation/report.tex` | **Complete LaTeX Source** for the formal validation and verification report. |
| [`tinklenberg2023_thin_disks_falling_in_air.pdf`](./tinklenberg2023_thin_disks_falling_in_air.pdf) | `../validation/papers/thin-disks-falling-in-air.pdf` | **Experimental Benchmark Paper**: Tinklenberg, Guala, Coletti (2023), *J. Fluid Mech.*, 962, A3. |
| [`varshney2012_falling_disks.pdf`](./varshney2012_falling_disks.pdf) | `../validation/papers/Varshney_2012_Nonlinearity_25_C1.pdf` | Supporting literature on 3D falling disk regimes and flutter/tumble bifurcations. |
| [`tumbling_validation_detailed.png`](./tumbling_validation_detailed.png) | `../validation/thinDisk2mm_tumbling/tumbling_validation_detailed.png` | **6-panel publication validation figure** across 13 full revolutions ($t = 0 \to 0.500\text{ s}$). |
| [`plot_tumbling_detailed.py`](./plot_tumbling_detailed.py) | `../validation/thinDisk2mm_tumbling/plot_tumbling_detailed.py` | Python script to regenerate the 6-panel tumbling validation plot. |
| [`tumbling_motion_rotation.dat`](./tumbling_motion_rotation.dat) | `../validation/thinDisk2mm_tumbling/case/postProcessing/tumbleMotion/motion.dat` | Angular velocity ($\boldsymbol{\Omega}$), Euler angles, quaternions, and moments ($29,714$ steps). |
| [`tumbling_motion_translation.dat`](./tumbling_motion_translation.dat) | `../validation/thinDisk2mm_tumbling/case/postProcessing/fallingMotion/motion.dat` | Translating frame velocities, aerodynamic forces ($\mathbf{F}_{\mathrm{aero}}$), and gravity ($29,714$ steps). |
| [`tumbling_solver.log`](./tumbling_solver.log) | `../validation/thinDisk2mm_tumbling/case/log.freeBody6DoFPimpleFoam` | Complete solver execution log with cyclicAMI weights and mass continuity checks. |
| [`tumbling_case`](./tumbling_case) | `../validation/thinDisk2mm_tumbling/case` | Case directory (0, constant, system, mesh) for the 13-revolution seeded tumbling run. |
| [`check_run_consistency.py`](./check_run_consistency.py) | `../validation/code_verification/check_run_consistency.py` | Rigorous audit script checking C++ rigid-body dynamic coupling, energy conservation ($3.2 \times 10^{-5}$), and asymptotic extrapolation. |
| [`verify_6dof_integrator.py`](./verify_6dof_integrator.py) | `../validation/code_verification/verify_6dof_integrator.py` | Independent verification of RK4 order ($p = 3.991$), non-trivial invariant conservation, and C++ diagnostics. |
| [`compute_spatial_gci.py`](./compute_spatial_gci.py) | `../validation/mesh_convergence/compute_gci.py` | ASME V&V 20 spatial grid convergence script across 3 meshes. |
| [`compute_temporal_gci.py`](./compute_temporal_gci.py) | `../validation/temporal_convergence/compute_temporal_gci.py` | ASME V&V 20 temporal convergence script across 3 time steps ($p = 1.566$). |
| [`zero_spin_validation_comparison.png`](./zero_spin_validation_comparison.png) | `../validation/thinDisk2mm/validation_comparison.png` | 4-panel validation curves from the preserved transient zero-spin drop run ($t = 0.235\text{ s}$). |
| [`plot_validation_zero_spin.py`](./plot_validation_zero_spin.py) | `../validation/thinDisk2mm/plot_validation.py` | Plotting script for the zero-spin drop run. |
| [`zero_spin_motion_rotation.dat`](./zero_spin_motion_rotation.dat) | `../validation/thinDisk2mm/case/postProcessing/tumbleMotion/motion.dat` | 6DoF rotational telemetry for zero-spin drop. |
| [`zero_spin_motion_translation.dat`](./zero_spin_motion_translation.dat) | `../validation/thinDisk2mm/case/postProcessing/fallingMotion/motion.dat` | 6DoF translational telemetry for zero-spin drop. |
| [`zero_spin_solver.log`](./zero_spin_solver.log) | `../validation/thinDisk2mm/case/log.freeBody6DoFPimpleFoam` | Complete solver execution log for zero-spin drop. |
| [`zero_spin_drift_animation.mp4`](./zero_spin_drift_animation.mp4) | `../validation/thinDisk2mm/render/disk_drift_animation.mp4` | 60 FPS 20× slow-motion 3D drift visualization video. |
| [`zero_spin_case`](./zero_spin_case) | `../validation/thinDisk2mm/case` | Case directory for the zero-spin drop run. |
| [`baseline_drag`](./baseline_drag) | `../validation/baseline_drag` | Baseline drag diagnostic cases and parametric sweep generators. |
| [`make_fixed_disk.sh`](./make_fixed_disk.sh) | `../validation/baseline_drag/make_fixed_disk.sh` | Shell script to construct fixed broadside disk benchmark cases (`fixedDisk_Re140`, `fixedDisk_Re100`). |
| [`analyze_fixed_disk.py`](./analyze_fixed_disk.py) | `../validation/baseline_drag/analyze_fixed_disk.py` | Python diagnostic for fixed-disk $C_D$, side-force envelope growth rate $\sigma$, and functionObject cross-checks. |
| [`make_restart_variants.sh`](./make_restart_variants.sh) | `../validation/baseline_drag/make_restart_variants.sh` | Shell script to generate developed-state restart variants from $t = 0.30\text{ s}$ (`base`, `dtHalf`, `dtHalfOuter2`, `linUp`). |
| [`compare_variants.py`](./compare_variants.py) | `../validation/baseline_drag/compare_variants.py` | Python tool comparing developed-state settling speed $V_{\mathrm{eq,est}}$, frequency, and peak moments across variants. |
| [`make_wakeFine_case.sh`](./make_wakeFine_case.sh) | `../validation/baseline_drag/make_wakeFine_case.sh` | Shell script constructing refined static wake mesh `warm_wakeFine` ($6,984$ static AMI faces vs. $456$ baseline). |

---

## 2. Quick-Start Reproduction Commands

All scripts can be executed directly from within this `validation_check/` directory:

```bash
# 1. Run production C++ 6DoF dynamics, energy balance, and asymptotic extrapolation audit
python3 check_run_consistency.py tumbling_motion_rotation.dat tumbling_motion_translation.dat

# 2. Run independent 6DoF RK4 integrator and production solver C++ verification
python3 verify_6dof_integrator.py

# 3. Regenerate 6-panel high-resolution tumbling validation plot
python3 plot_tumbling_detailed.py

# 4. Run spatial ASME V&V 20 Grid Convergence Index analysis
python3 compute_spatial_gci.py

# 5. Run temporal ASME V&V 20 Time-Step Convergence analysis
python3 compute_temporal_gci.py

# 6. Analyze fixed broadside disk simulations (subcritical Re = 100, baseline Re = 140, and refined wakeFine)
python3 analyze_fixed_disk.py baseline_drag/fixedDisk_Re100 --Re 100
python3 analyze_fixed_disk.py baseline_drag/fixedDisk_Re140 --Re 140
python3 analyze_fixed_disk.py baseline_drag/fixedDisk_Re140_wakeFine --Re 140

# 7. Compare developed-state restart variants from t = 0.30 s
python3 compare_variants.py baseline_drag/restart_base baseline_drag/restart_dtHalf baseline_drag/restart_dtHalfOuter2 baseline_drag/restart_linUp --t-from 0.32
```

---

## 3. Key Validation Findings At A Glance

- **Baseline Drag Deficit Source:** Both zero-spin broadside fall and tumbling autorotation settle near $1.02\text{ m/s}$ ($-17\%$ below experimental $1.23\text{ m/s}$, corresponding to $C_{D,\mathrm{paper}} \approx 1.09$ vs. $0.746$). This confirms the deficit resides in baseline aerodynamic drag under this discretization, not 6DoF dynamic coupling.
- **AMI Discontinuity & Demonstrated Drag Drop:** In the baseline mesh, the rotating AMI sphere has $15,528$ faces ($\Delta x \approx 57\ \mu\mathrm{m}$) while the static outer sphere had only $456$ faces ($\Delta x \approx 330\ \mu\mathrm{m}$) — a 34:1 mismatch. The newly constructed `warm_wakeFine` mesh increases static AMI resolution $>15\times$ to $6,984$ faces ($\approx 78\ \mu\mathrm{m}$) and embeds a $312\ \mu\mathrm{m}$ wake box, passing all OpenFOAM `checkMesh` quality criteria. Execution of `fixedDisk_Re140_wakeFine` demonstrated that broadside drag drops from $1.1609 \to 1.1225$ (a $-3.3\%$ drop) while AMI mass flux leakage drops by $5.1\times$ to $8.44 \times 10^{-8}$, confirming that interface smearing is the primary driver of baseline drag inflation.
- **Independent Force & Flux Cross-Checks:**
  - Solver hand-rolled surface traction matches OpenFOAM's independent `forces` function object to $\max |\Delta F_y| / \bar{F}_y = 1.10 \times 10^{-10}$ [PASS].
  - Net volumetric flux through `masterAMI` is $< 3.5 \times 10^{-12}\text{ m}^3\text{/s}$ ($< 4 \times 10^{-9}$ of convective throughput) [PASS].
- **C++ 6DoF Dynamic & Energy Consistency:**
  - $I_{xx} \Delta\omega_x / \Delta t$ matches $M_x$ to $2.51 \times 10^{-6}$ [PASS].
  - Integrated aerodynamic work matches mechanical energy change $\Delta E$ to $3.19 \times 10^{-5}$ over 29,715 production time steps [PASS].
- **Tumbling Frequency:** Asymptotic plateau across Revs 8–13 gives $f_t = 30.52 \pm 0.17\text{ Hz}$, comparing with the experimental distribution peak at $\sim 33\text{ Hz}$ (mean $36.4\text{ Hz}$).
- **Strouhal Number:** Simulated $St = \frac{f_t D}{\langle V_y \rangle} = 0.059$ matches the experimental Strouhal distribution peak ($St \approx 0.055 - 0.060$).
- **Zero-Spin Response:** Decays via damped rocking ($51^\circ \to 36^\circ \to 25^\circ$) toward a stable flat broadside fall without autorotation.

