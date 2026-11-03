# freeBodyFoam

[![OpenFOAM](https://img.shields.io/badge/openfoam-v2412-informational)](https://www.openfoam.com)
[![License: GPL-3.0-or-later](https://img.shields.io/badge/license-GPL--3.0--or--later-green.svg)](LICENSE)
[![Verification: ASME V&V 20](https://img.shields.io/badge/verification-ASME%20V%26V%2020-blue.svg)](docs/validation.md)

`freeBodyFoam` is a custom OpenFOAM solver suite for simulating free-falling bodies, projectile trajectories, atmospheric entry, and 6DoF tumbling dynamics in a **body-fixed accelerating reference frame**. By solving fluid dynamics in the non-inertial frame of the falling object, trajectory descents across kilometers of physical altitude are resolved within a compact, stationary computational domain.

> **Status**: Verified and validated against analytical high-$Re$ terminal velocity and the Fernandes et al. (2007) experimental benchmark suite on OpenFOAM-v2412.

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
4. **Symplectic 6DoF Quaternion Kinematics**: Integrates the Euler-Poinsot rigid-body equations with quaternion algebra, conserving rotational kinetic energy to within $< 10^{-13}$ relative error.
5. **Arbitrary Mesh Interface (`cyclicAMI`) for Arbitrary 3D Tumble**: Combines an inner rotating spherical mesh zone (`rotatingZone`) with an outer stationary domain, allowing continuous, unconstrained 3D tumbling without remeshing.

---

## Benchmark Showcases & Validation

### 1. Analytical Transcritical Cylinder Crossflow (`theRodDrop`)
A heavy cylinder ($R = 2.0\text{ m}$, $m = 1000\text{ kg}$) falling through air is benchmarked against the analytical force balance at transcritical Reynolds numbers ($Re = 2.57 \times 10^7$):
* **Analytical Target**: $V_{t,\text{analytical}} = \sqrt{\frac{2mg}{\rho_f C_D A}} = 96.388\text{ m/s}$ ($C_D = 0.3452$).
* **Simulated Velocity**: $V_{\text{sim}} = 96.340\text{ m/s}$ (**$-0.05\%$ relative discrepancy**).
* **Terminal Deceleration**: $\frac{dV}{dt} = 0.0031\text{ m/s}^2$ ($99.968\%$ decay toward equilibrium).

![theRodDrop Kinematics](docs/images/theRodDrop_kinematics.png)

### 2. Viscous Disk Benchmark (Fernandes et al. 2007)
* **Steady Wake ($Re = 100$)**: Computed drag coefficient $C_D = 1.157$ matches benchmark $C_D = 1.160$ within **$-0.26\%$**.
* **Oscillatory Vortex Shedding ($Re = 180$)**: Computed shedding frequency corresponds to Strouhal number $St = 0.1260$, matching experimental $St = 0.1250 \pm 0.003$ within **$+0.80\%$**.

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
* **Rigorous Momentum & Torque Integration**: Integrates instantaneous pressure $p_{\text{rgh}}$ and viscous shear stress across complex lifting surfaces, capturing wake-induced autorotation and spin-up dynamics.
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

* **Fernandes, P. C., Risso, F., Fabre, D., & Magnaudet, J.** (2007). "Oscillatory motion and wake instability of freely rising or falling disks". *Journal of Fluid Mechanics*, 570, 391–414.
* **ASME V&V 20-2009**: "Standard for Verification and Validation in Computational Fluid Dynamics and Heat Transfer", *American Society of Mechanical Engineers*.
* **Achenbach, E.** (1968). "Distribution of local pressure and skin friction around a circular cylinder in cross-flow up to Re = 5 × 10⁶". *Journal of Fluid Mechanics*, 34(4), 625–639.
* **Kurganov, A., & Tadmor, E.** (2000). "New High-Resolution Central Schemes for Nonlinear Conservation Laws and Convection-Diffusion Equations". *Journal of Computational Physics*, 160(1), 241–282.
* **Roache, P. J.** (1998). *Verification and Validation in Computational Science and Engineering*. Hermosa Publishers, Albuquerque, NM.
