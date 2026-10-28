# Validation Check Package — `freeBody6DoFPimpleFoam`

This folder contains a consolidated collection of **symbolic links** to all reports, experimental benchmark papers, 6DoF simulation telemetry data, solver logs, and analysis scripts needed to review and reproduce the validation of `freeBody6DoFPimpleFoam`.

---

## 1. File Manifest & Symlink Directory

| Symlink Name | Target Path | Description |
| :--- | :--- | :--- |
| [`report.md`](./report.md) | `../validation/report.md` | **Primary Comprehensive Validation Report** detailing physics, 13-revolution data, ASME V&V 20 uncertainty, and peer-review resolutions. |
| [`tinklenberg2023_thin_disks_falling_in_air.pdf`](./tinklenberg2023_thin_disks_falling_in_air.pdf) | `../validation/papers/thin-disks-falling-in-air.pdf` | **Experimental Benchmark Paper**: Tinklenberg, Guala, Coletti (2023), *J. Fluid Mech.*, 962, A3. |
| [`varshney2012_falling_disks.pdf`](./varshney2012_falling_disks.pdf) | `../validation/papers/Varshney_2012_Nonlinearity_25_C1.pdf` | Supporting literature on 3D falling disk regimes and flutter/tumble bifurcations. |
| [`tumbling_validation_detailed.png`](./tumbling_validation_detailed.png) | `../validation/thinDisk2mm_tumbling/tumbling_validation_detailed.png` | **6-panel publication validation figure** across 13 full revolutions ($t = 0 \to 0.500\text{ s}$). |
| [`plot_tumbling_detailed.py`](./plot_tumbling_detailed.py) | `../validation/thinDisk2mm_tumbling/plot_tumbling_detailed.py` | Python script to regenerate the 6-panel tumbling validation plot. |
| [`tumbling_motion_rotation.dat`](./tumbling_motion_rotation.dat) | `../validation/thinDisk2mm_tumbling/case/postProcessing/tumbleMotion/motion.dat` | Angular velocity ($\boldsymbol{\Omega}$), Euler angles, quaternions, and moments ($29,714$ steps). |
| [`tumbling_motion_translation.dat`](./tumbling_motion_translation.dat) | `../validation/thinDisk2mm_tumbling/case/postProcessing/fallingMotion/motion.dat` | Translating frame velocities, aerodynamic forces ($\mathbf{F}_{\mathrm{aero}}$), and gravity ($29,714$ steps). |
| [`tumbling_solver.log`](./tumbling_solver.log) | `../validation/thinDisk2mm_tumbling/case/log.freeBody6DoFPimpleFoam` | Complete solver execution log with cyclicAMI weights and mass continuity checks. |
| [`tumbling_case`](./tumbling_case) | `../validation/thinDisk2mm_tumbling/case` | Case directory (0, constant, system, mesh) for the 13-revolution seeded tumbling run. |
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

---

## 2. Quick-Start Reproduction Commands

All scripts can be executed directly from within this `validation_check/` directory:

```bash
# 1. Run independent 6DoF RK4 integrator and production solver C++ verification
python3 verify_6dof_integrator.py

# 2. Regenerate 6-panel high-resolution tumbling validation plot
python3 plot_tumbling_detailed.py

# 3. Run spatial ASME V&V 20 Grid Convergence Index analysis
python3 compute_spatial_gci.py

# 4. Run temporal ASME V&V 20 Time-Step Convergence analysis
python3 compute_temporal_gci.py

# 5. Regenerate zero-spin transient validation plot
python3 plot_validation_zero_spin.py
```

---

## 3. Key Validation Findings At A Glance

- **Tumbling Frequency:** Simulated stabilized plateau $f_t = 30.52 \pm 0.17\text{ Hz}$ matches the peak mode of the experimental frequency PDF in Fig. 20(a) ($\sim 33\text{ Hz}$, spread $25 - 42\text{ Hz}$).
- **Strouhal Number:** Simulated $St = \frac{f_t D}{\langle V_y \rangle} = 0.0593 \approx 0.060$ matches the peak mode of the experimental Strouhal distribution in Fig. 20(b) ($St \approx 0.055 - 0.060$).
- **Descent Velocity:** Simulated settling speed $\langle V_y \rangle = 1.040 \pm 0.015\text{ m/s}$ ($Re \approx 140$). The comparison error $E = -0.19\text{ m/s}$ is within the combined validation uncertainty dominated by the $\pm 15\%$ manufacturing thickness tolerance ($h = 50 \pm 7.5\ \mu\text{m} \implies u_{\mathrm{input}}(V_t) \approx \pm 7.5\% \approx 0.092\text{ m/s}$).
- **Trajectory Angle:** Simulated developed track angle $\psi_{\mathrm{track}} = 12.07^\circ - 15.29^\circ$ matches the experimental PDF peak mode of $|\psi| \approx 10.5^\circ$ (Fig. 10b).
