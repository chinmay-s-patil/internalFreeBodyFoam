# freeBodyFoam

[![OpenFOAM](https://img.shields.io/badge/openfoam-v2412-informational)](https://www.openfoam.com)
[![License: GPL-3.0-or-later](https://img.shields.io/badge/license-GPL--3.0--or--later-green.svg)](LICENSE)
[![Verification: ASME V&V 20](https://img.shields.io/badge/verification-ASME%20V%26V%2020-blue.svg)](docs/validation.md)

`freeBodyFoam` is a custom OpenFOAM solver suite for simulating free-falling bodies, projectile trajectories, atmospheric entry, and 6DoF tumbling dynamics in a **body-fixed accelerating reference frame**. By solving fluid dynamics in the non-inertial frame of the falling object, trajectory descents across kilometers of physical altitude are resolved within a compact, stationary computational domain.

> **Status (ASME V&V 20)**: **Code Verification Complete | Validation Partial**.  
> Kinematic autorotation ($St \approx 0.0598$) and non-inertial reference frame momentum coupling ($F_{\text{aero}} \approx mg$) are verified. Terminal settling velocity against the [Tinklenberg et al. (2023)](https://doi.org/10.1017/jfm.2023.209) experimental benchmark exhibits a **$-17\%$ velocity deficit ($1.02\text{ m/s}$ vs. $1.23\text{ m/s}$, $|E| \approx 11.7\, u_{\text{val}}$)** which remains an open, unresolved modeling discrepancy.

### Claims-to-Evidence Summary

| Category | Claimed Feature / Metric | Evidence / Verification Basis | Limitations & Open Discrepancies | Status |
| :--- | :--- | :--- | :--- | :--- |
| **6DoF Rotational Kinematics** | 4th-order ODE convergence for $\boldsymbol{\omega}$; rigid-body invariants conserved in vacuum | Test function convergence: $p = 3.991$ for $\boldsymbol{\omega}$ using explicit RK4; kinetic energy drift $< 10^{-14}$ across 13,333 steps ($\Delta t = 1.5 \times 10^{-5}\text{ s}$, $T = 0.20\text{ s}$) | Explicit RK4 (not symplectic). Attitude update uses quaternion exponential map on $SO(3)$ (order unverified when $\boldsymbol{\omega}$ varies within a step). | **VERIFIED** (for $\boldsymbol{\omega}$) |
| **Non-Inertial Reference Frame** | Dynamic D'Alembert frame coupling achieves steady terminal equilibrium | Planar cylinder test ([`tutorials/theRodDrop/theRodDrop-Denser`](tutorials/theRodDrop/theRodDrop-Denser)): $F_{\text{aero}}/mg = 99.97\%$ ($0.03\%$ residual), $a_y \to 0.0031\text{ m/s}^2$ | Verification of frame momentum coupling only. Does not validate aerodynamic drag coefficients (circular test). | **VERIFIED** (Frame coupling only) |
| **Sliding CyclicAMI Interface** | Mass conservation across rotating spherical interface | Interface flux leak $< 4.81 \times 10^{-13}\text{ m}^3\text{/s}$ ($< 6.2 \times 10^{-10}$ rel. inflow) | Static outer mesh refined from 213k to 487k cells; inner rotating boundary layer mesh remained unrefined. | **VERIFIED** |
| **Tumbling Kinematics** | Free 6DoF planar tumbling autorotation | Spontaneous limit-cycle autorotation sustained across 13 revs; $St = 0.0598$ matches experimental peak ($0.055 - 0.060$) | Sim frequency is $30.5\text{ Hz}$ vs. exp. mean $\sim 36.4\text{ Hz}$ (also $\sim 16\%$ low). $St$ matches because $V$ and $f$ are both low by similar factors, pointing to an effective force or mass offset. | **CONSISTENT** |
| **Numerical Sensitivity** | Discretization sensitivities & formal 3-mesh GCI | Formal 3-mesh GCI at $Re = 100$ ($N \in [131\text{k}, 369\text{k}]$, rotating zone refined): oscillatory convergence ($R = -0.89$), family span $0.49\%$, $u_{\text{num,mesh}} \le 0.55\%$. Combined with temporal ($+0.04\%$) and scheme ($-0.75\%$), total $u_{\text{num}} \le 0.93\%$ | Replaces single-pair shifts with formal ASME V&V 20 spatial GCI with near-body zone systematically refined. | **BOUNDED** ($u_{\text{num}} \le 0.55\%$) |
| **Terminal Settling Velocity** | Mean settling speed of thin disks falling in air | [Tinklenberg et al. (2023)](https://doi.org/10.1017/jfm.2023.209) $2.02\text{ mm}$ PET disk ($h = 50\ \mu\text{m}$, $\chi = 40.4$): $V_{\text{sim}} \approx 1.02\text{ m/s}$ vs. $V_{\text{exp}} = 1.23\text{ m/s}$ | Discrepancy of $-17\%$ ($|E| \approx 11.1\, u_{\text{val}}$) is unresolved. Specimen thickness scatter alone cannot explain it (needs about $-31\%$ mean thickness); cause unidentified. 3D perturbation tests bound planar symmetry sensitivity at $+0.10\%$. Diagnostic weight step ($h \approx 73\ \mu\text{m}$) recovers both $V$ ($1.229\text{ m/s}$) and $f$ ($35.44\text{ Hz}$) concurrently. | **NOT VALIDATED** |
| **Planar Symmetry & 3D Stability** | Orbital stability of 2D planar tumbling mode | Free 6DoF with 5% $\Omega_z$ perturbation over 8.5 revs: sustained 3D precession ($\theta_{\text{out}}$ mean $8.24^\circ$, peak $24.07^\circ$); $V_{\text{eq,est}}$ shifts by $+0.10\%$ ($+0.001\text{ m/s}$), $f_t$ by $-1.12\%$ | Planar symmetry restriction bounds settling speed sensitivity at $\le 0.1\%$. Out-of-plane precessional wobble does not close the $-17\%$ deficit. | **BOUNDED** ($\le 0.1\%$ sensitivity) |
| **Independent Drag Check** | Broadside disk & sphere drag against published literature | Published experimental correlations: Roos & Willmarth (1971) for disks; Schiller–Naumann (1933) / Clift et al. (1978) for spheres at $Re = 100\text{--}200$ | Broadside disk at $Re = 200$: $C_D = 1.106$ matches Roos & Willmarth (1971) benchmark ($1.05\text{--}1.15$, modal $1.10$) within $< 1\%$. Spheres at $Re = 100$ ($C_D = 1.391$) and $Re = 200$ ($C_D = 1.023$) show consistent $+27\%$ truncation offset on coarse 36k Cartesian mesh. | **CONSISTENT** (Disk matches $< 1\%$) |
| **Axisymmetric Body Dynamics** | Rectilinear & oscillatory wakes ([Fernandes et al. 2007](https://doi.org/10.1017/S0022112006003685)) | Initial test cases set up at $Re = 100$ and $Re = 200$ | Fixed-disk runs compared against self-generated baselines. Release test had rotation locked. Critical $Re_c$, shedding frequencies, and amplitudes not yet validated. | **IN PROGRESS** |
| **Domain & Far-Field Sensitivity** | Domain size and boundary blockage effects | Fixed-disk $Re = 140$ in $1.5\times$ scaled domain ($22.5D \times 37.5D \times 22.5D$, 267k cells) | No measurable sensitivity at $t = 80\text{ ms}$ (equal-time comparison, $C_D = 1.1524$ vs. $1.1527$, shift $-0.019\%$). Domain blockage does not explain the deficit. | **BOUNDED** |
| **Literature Drag Sources** | Secondary citations for thin disk drag at $Re = 100$ | Range $C_D \approx 1.15\text{--}1.25$ commonly cited via Shenoy & Kleinstreuer (2008) | Primary audit confirms Shenoy & Kleinstreuer (2008) studied aspect ratio $\chi = 10$ ($h/D = 0.1$, $\sim 4\times$ thicker than Tinklenberg $\chi = 40.4$). Edge drag and altered base suction explain lower $C_D$ for thicker disks; ultra-thin disks produce $C_D \approx 1.27\text{--}1.30$. | **AUDITED** |
| **Transonic / Compressible Solvers** | `freeBodyRhoPimpleFoam`, `freeBodyRhoCentralFoam` | Solver implementations completed in source tree | Zero quantitative verification or validation cases executed. | **UNVALIDATED** |

---

## Solvers Overview

| Solver | Flow Regime | Reference Frame | Kinematic Degrees of Freedom | Key Mechanisms |
| :--- | :--- | :--- | :--- | :--- |
| **`freeBodyPimpleFoam`** | Incompressible | Body-Fixed Dynamic | 3D Translation | D'Alembert fictitious frame acceleration, dynamic force balance |
| **`freeBody6DoFPimpleFoam`** | Incompressible | Body-Fixed Dynamic | 6DoF (3D Translation + 3D Rotation) | Hydrostatic-reduced $p_{\text{rgh}}$, quaternion kinematics, sliding `cyclicAMI` |
| **`freeBodyRhoPimpleFoam`** | Compressible | Body-Fixed Dynamic | 3D Translation | Pressure-based compressible solver with automatic Mach regime transition |
| **`freeBodyRhoCentralFoam`** | Transonic / Supersonic | Body-Fixed Dynamic | 3D Translation | Density-based central-upwind solver (Kurganov-Tadmor) for atmospheric entry |

---

## Technical & Physical Innovations

Traditional numerical methods for atmospheric free-fall require translating the entire mesh or running vast computational domains. `freeBodyFoam` introduces several key innovations:

1. **Body-Fixed Accelerating Reference Frame**: Incorporates the d'Alembert fictitious inertial force $\mathbf{S}_U = -\mathbf{a}_{\text{frame}}$ directly into the momentum equation:
   $$\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) - \nabla \cdot (\nu_{\text{eff}} \nabla \mathbf{U}) = -\frac{1}{\rho_f} \nabla p + \mathbf{g} - \mathbf{a}_{\text{frame}}$$
2. **Hydrostatic-Reduced Pressure Formulation ($p_{\text{rgh}}$)**: Eliminates spurious apparent hydrostatic moments on tumbling asymmetric bodies, preserving numerical stability during tight 6DoF coupling.
3. **Dynamic Inflow/Outflow Switching (`fallingInletVelocity`)**: Evaluates instantaneous directional face fluxes $(\mathbf{S}_f \cdot \mathbf{U}_{\text{inlet}})$, imposing Dirichlet inflow conditions on upstream faces and reflection-free zero-gradient Neumann conditions on downstream wake faces.
4. **Coupled 6DoF Quaternion Kinematics**: Integrates Euler's rigid-body equations for angular velocity $\boldsymbol{\omega}$ with explicit RK4 ($p = 3.991$) and orientation via exponential quaternion map on $SO(3)$, conserving unforced rotational invariants to $< 10^{-14}$ relative error.
5. **Arbitrary Mesh Interface (`cyclicAMI`) for Arbitrary 3D Tumble**: Combines an inner rotating spherical mesh zone (`rotatingZone`) with an outer stationary domain, allowing continuous, unconstrained 3D tumbling without remeshing.

---

## Benchmark Showcases & Validation

### 1. Non-Inertial Reference Frame Coupling (`theRodDrop`)
A heavy half-cylinder ($R = 2.0\text{ m}$, span $w = 1.0\text{ m}$, frontal area $A = 2.0\text{ m}^2$, $m = 98,018\text{ kg}$) falling through scaled dense fluid ($\rho_f = 300\text{ kg/m}^3$, $\nu = 1.5 \times 10^{-5}\text{ m}^2\text{/s}$, $g = 9.81\text{ m/s}^2$) is benchmarked for dynamic frame momentum balance under $k$-$\omega$ SST URANS ($Re = 2.57 \times 10^7$):
* **Analytical Formulation**: Terminal balance $F_{\text{aero}} \approx mg \implies V_{t,\text{analytical}} = \sqrt{\frac{2mg}{\rho_f C_D A}} = 96.359\text{ m/s} \approx 96.36\text{ m/s}$ (using simulated $C_D = 0.3452$).
* **Simulated Velocity**: $V_{\text{sim}} = 96.340\text{ m/s}$ (**$-0.02\%$ relative discrepancy**).
* **Terminal Equilibrium**: $\frac{dV}{dt} = 0.0031\text{ m/s}^2$ ($99.968\%$ decay toward equilibrium; near-terminal force balance $F_{\text{aero}}/mg = 99.97\%$).
* **Scope & Limitation**: Verifies non-inertial accelerating reference frame coupling ($F \approx mg$). Does not validate aerodynamic drag coefficients because $C_D$ was extracted from the simulation itself; matching the literature drag band at $Re = 2.6 \times 10^7$ without boundary-layer tripping or grid sensitivity is coincidental.

![theRodDrop Kinematics](docs/images/theRodDrop_kinematics.png)

### 2. Viscous Disk Flow ([Fernandes et al. 2007](https://doi.org/10.1017/S0022112006003685))
* **Status**: Initial verification cases configured for thin disks ($\chi = 10$) in viscous liquid at $Re = 100$ and $Re = 200$.
* **Validation Scope**: Rectilinear wake development and dynamic release were tested with rotation locked; validation against published Fernandes et al. bifurcation thresholds ($Re_c$), vortex shedding frequencies, and trajectory oscillations is currently in progress.

---

## Tutorial Suite

The repository includes three production-ready, fully documented tutorials:

```
tutorials/
├── theRodDrop/     # 2D planar transverse cylinder fall & analytical terminal velocity verification
├── rodRoller/      # 2D cylinder with axial Z-spin via circular sliding cyclicAMI (O-grid)
└── steelBallRun/   # True 3D solid sphere with unconstrained 6DoF motion & 32-dot Fibonacci golf-ball markers
```

### 3D 6DoF Showcase: `steelBallRun`
Simulates a solid steel sphere ($D = 50\text{ mm}$, $m = 0.5105\text{ kg}$) falling under planetary gravity with unrestricted 3D translation and 3D tumbling ($\vec{\omega}_0 = (10, 5, 15)\text{ rad/s}$). To observe 3D roll, pitch, and yaw in a single glance, **32 high-contrast crimson surface markers** are distributed via an equilateral Fibonacci spherical lattice:

<table>
  <tr>
    <th width="45%" style="text-align:center;">3D 6DoF Spherical Tumbling (Fibonacci Markers)</th>
    <th width="55%" style="text-align:center;">Coupled 6DoF Kinematic Telemetry</th>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/steelBallRun_6DoF.gif" alt="steelBallRun 3D 6DoF Animation" width="100%"/>
    </td>
    <td align="center">
      <img src="docs/images/steelBallRun_kinematics.png" alt="steelBallRun Kinematic Telemetry" width="100%"/>
    </td>
  </tr>
</table>

* **Real-time Terminal Dashboard**: `python3 track_motion.py --watch`
* **Publication Figures**: `python3 track_motion.py --plot`
* **3D PyVista Animation & Video Export**: `python3 animate_motion.py --video steelBallRun.mp4`

---

## Advanced Capabilities Showcase: 6DoF Propeller Tumble & Autorotation

> [!NOTE]
> **Private Case Study**: The propeller aerodynamic tumble simulation demonstrated below is a private research case study showcasing the capabilities of `freeBody6DoFPimpleFoam`. While the propeller geometry and case files are kept private, the side-by-side animation and telemetry illustrate the complex multi-physics dynamics resolved by the solver.

<table>
  <tr>
    <th width="45%" style="text-align:center;">3D Radar-Tracking Flight Simulation</th>
    <th width="55%" style="text-align:center;">Coupled 6DoF Kinematic Telemetry</th>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/propTumble6DoF_radar.gif" alt="propTumble6DoF Radar Tracking Simulation" width="100%"/>
    </td>
    <td align="center">
      <img src="docs/images/propTumble6DoF_kinematics.png" alt="propTumble6DoF 6DoF Kinematic Telemetry" width="100%"/>
    </td>
  </tr>
</table>

<details>
<summary><b>Click to expand full 8-panel Rotational and Translational reports (Side-by-Side)</b></summary>
<br/>

<table>
  <tr>
    <th width="50%" style="text-align:center;">Rotational Dynamics & Aerodynamic Loads</th>
    <th width="50%" style="text-align:center;">3D Trajectory & Lateral Drift Kinematics</th>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/propTumble6DoF_rotational.png" alt="Rotational Dynamics Report" width="100%"/>
    </td>
    <td align="center">
      <img src="docs/images/propTumble6DoF_translational.png" alt="Translational Kinematics Report" width="100%"/>
    </td>
  </tr>
</table>

</details>

### What `freeBodyFoam` Can Simulate
* **Full 6DoF Coupled Tumble & Autorotation**: Resolves chaotic blade autorotation, gyroscopic tumbling, pitch reversal, and 3D crosswind drift on non-trivial geometries.
* **Non-Deforming Sliding Spherical Mesh (`cyclicAMI`)**: Continuous, unrestricted multi-axis 3D tumbling without mesh skewness degradation, boundary layer distortion, or remeshing penalties.
* **Pressure & Viscous Moment Integration**: Integrates instantaneous pressure $p_{\text{rgh}}$ and viscous shear stress across complex lifting surfaces, capturing wake-induced autorotation and spin-up dynamics.
* **Precision Telemetry & Radar Tracking**: Outputs instantaneous 6DoF pose quaternions, Euler attitude angles, velocity, acceleration, and force balances to live dashboards and 3D visualizers.

## Requirements and Environment

* **OpenFOAM**: OpenFOAM-v2412 / v2006+ (tested on Ubuntu 24.04 LTS x86_64, GCC 13.3).
* **Python**: Python 3.8+ with NumPy, Matplotlib, and PyVista (for live telemetry, publication figures, and 3D video renders).

---

## Build Instructions

Source your OpenFOAM installation and compile with `./Allwmake`:

```bash
# Source OpenFOAM environment
source /usr/lib/openfoam/openfoam2412/etc/bashrc

# Compile boundary conditions, motion solvers, function objects, and solvers
./Allwmake
```

Compiled libraries (`libfallingInletVelocity.so`, `libfreeBodyRotationalMotion.so`) are placed into `$FOAM_USER_LIBBIN`. Solver binaries (`freeBodyPimpleFoam`, `freeBody6DoFPimpleFoam`, etc.) are installed into `$FOAM_USER_APPBIN`. To clean build artifacts, run `./Allwclean`.

---

## Quick Start: Running Tutorials

Navigate to any tutorial and launch the automated runner:

```bash
# 1. Run theRodDrop (analytical terminal velocity verification)
cd tutorials/theRodDrop/theRodDrop
./Allrun

# 2. Run rodRoller (2D transverse cylinder with axial spin)
cd tutorials/rodRoller
./Allrun

# 3. Run steelBallRun (true 3D 6DoF steel sphere)
cd tutorials/steelBallRun
./Allrun
```

> [!TIP]
> During all simulation runs, `foamResiduals` automatically launches on `DISPLAY=:0`, displaying live solver residuals directly from the real-time streamed log file.

---

## Documentation Index

Detailed reference documentation is maintained in the [`docs/`](docs/) directory:

| Document | Description |
| :--- | :--- |
| [`docs/models.md`](docs/models.md) | Governing equations, accelerating reference frame transformations, $p_{\text{rgh}}$ derivation, and 6DoF quaternion dynamics |
| [`docs/dictionary.md`](docs/dictionary.md) | Keyword reference tables, physical units, and minimal working templates for `fallingFrameDict` and `dynamicMeshDict` |
| [`docs/tutorials.md`](docs/tutorials.md) | Detailed walkthrough, meshing topologies, and kinematics plots for `theRodDrop`, `rodRoller`, and `steelBallRun` |
| [`docs/validation.md`](docs/validation.md) | Verification hierarchy, ASME V&V 20 convergence data, analytical terminal velocity comparisons, and Fernandes (2007) benchmarks |
| [`docs/development.md`](docs/development.md) | Source tree breakdown, PIMPLE predictor-corrector sequence, and instructions for developer extensions |

---

## References

* **Fernandes, P. C., Risso, F., Ern, P., & Magnaudet, J.** (2007). "Oscillatory motion and wake instability of freely rising axisymmetric bodies". *Journal of Fluid Mechanics*, 573: 479–502.
* **Tinklenberg, A., Guala, M., & Coletti, F.** (2023). "Thin disks falling in air". *Journal of Fluid Mechanics*, 962: A3.
* **ASME V&V 20-2009**: "Standard for Verification and Validation in Computational Fluid Dynamics and Heat Transfer", *American Society of Mechanical Engineers*.
* **Roos, F. W., & Willmarth, W. W.** (1971). "Some experimental results on sphere and disk drag". *AIAA Journal*, 9(2), 285–291.
* **Schiller, L., & Naumann, A.** (1933). "A drag coefficient correlation". *VDI Zeitung*, 77, 318–320.
* **Kurganov, A., & Tadmor, E.** (2000). "New High-Resolution Central Schemes for Nonlinear Conservation Laws and Convection-Diffusion Equations". *Journal of Computational Physics*, 160(1), 241–282.
* **Roache, P. J.** (1998). *Verification and Validation in Computational Science and Engineering*. Hermosa Publishers, Albuquerque, NM.
