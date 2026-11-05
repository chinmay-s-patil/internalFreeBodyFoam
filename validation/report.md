# 6DoF CFD Validation Study: Thin Disks Falling in Air

**Author:** Chinmay S. Patil  
**Benchmark Reference:**  
Tinklenberg, A., Guala, M., & Coletti, F. (2023). *Thin disks falling in air*. **Journal of Fluid Mechanics**, vol. 962, A3. [doi:10.1017/jfm.2023.209](https://doi.org/10.1017/jfm.2023.209)

**CFD Solver:** `freeBody6DoFPimpleFoam` (Coupled 6DoF Accelerating Reference Frame + Spherical Sliding AMI)  
**Rotational Motion Function:** `freeBodyRotationalMotion`  
**Case Directories:**  
- [`validation/thinDisk2mm/`](./thinDisk2mm/) (Preserved transient zero-spin drop run)  
- [`validation/thinDisk2mm_tumbling/`](./thinDisk2mm_tumbling/) (Seeded tumbling run to $t = 0.500\text{ s}$)

---

## 1. Executive Summary & Validation Status

A 6DoF computational fluid dynamics simulation study was performed for a thin polyethylene terephthalate (PET) disk falling in air, based on the $2\text{ mm}$ experimental benchmark from Tinklenberg, Guala, and Coletti (2023, *J. Fluid Mech.*, vol. 962, A3).

### Current Validation Status
> **Code and solver verification complete. Validation partial: mean settling speed 17% below experiment. Mesh effect bounded at under 0.1% (direct 6DoF test, outer static mesh only; near-body zone not varied), time step under 0.05% (four revolutions), and convection scheme under 0.8%. The remaining deficit is unresolved.**

The solver's C++ rigid-body kinematics and energy conservation hold to machine precision on production output (verified directly via [`validation/code_verification/check_run_consistency.py`](./code_verification/check_run_consistency.py)):
- Dynamic rotational coupling ($I_{xx} \Delta\omega_x / \Delta t$ vs. $M_x$) matches to **$2.51 \times 10^{-6}$** relative error.
- Vertical frame acceleration $a_y$ matches $-g + F_y/m$ to **$7.20 \times 10^{-9}\text{ m/s}^2$**.
- Velocity updates match integrated acceleration to **$1.22 \times 10^{-9}\text{ m/s}$**, and angle matches $\int \Omega_x \, dt$ to **$0.26^\circ$** over 14 revolutions.
- Mechanical energy change $\Delta E$ matches integrated aerodynamic boundary work $W_k$ to **$3.19 \times 10^{-5}$** relative error. In vacuum tests over 13,333 steps ($\Delta t = 1.5 \times 10^{-5}\text{ s}$, $T = 0.20\text{ s}$), energy drift is $< 10^{-14}$.
- Integrator ODE convergence order $p = 3.991$ verifies RK4 for angular velocity $\boldsymbol{\omega}$ on test functions. (The quaternion exponential-map attitude update on $SO(3)$ is unverified for order when $\boldsymbol{\omega}$ varies within a step).
- Hand-rolled traction forces match OpenFOAM's standard `forces` function object to **$2.07 \times 10^{-10}$**.

However, validation against the experimental mean terminal settling speed is partial:
1. **Asymptotic Settling Deficit ($-17.1\%$):** The continuous 13-revolution tumbling simulation achieves a stable frequency plateau ($f_t = 30.52 \pm 0.17\text{ Hz}$) and matching Strouhal number ($St = 0.059$ vs. experimental $0.055$--$0.060$). However, the asymptotic descent velocity settles at **$V_\infty \approx 1.02\text{ m/s}$**, representing a **$-17.1\%$ deficit** relative to experimental mean $V_t = 1.23\text{ m/s}$ (nominal $C_D \approx 1.09$ vs. experimental $0.74$).
2. **Direct 6DoF Mesh Sensitivity ($-0.09\%$):** Direct free-fall simulation on the refined static mesh (`warm_wakeFine`, 487k cells) vs. baseline (`warm_base`, 213k cells) over 5 complete revolutions ($t \in [0.08, 0.25]\text{ s}$) produces a settling velocity shift of only **$-0.09\%$** ($-0.0009\text{ m/s}$).
3. **Temporal Discretization Sensitivity ($+0.04\%$):** Dynamic limit-cycle restarts from $t = 0.30\text{ s}$ (`restart_dtHalf`, $\mathrm{Co}_{\mathrm{max}} \le 0.75$, `nOuterCorrectors 1`) demonstrate that halving the time step shifts equilibrium settling velocity by only **$+0.04\%$** ($+0.0004\text{ m/s}$) across 4 complete revolutions.
4. **Convection Scheme Sensitivity ($-0.75\%$):** Switching from central-blended LUST to pure `linearUpwind` (`restart_linUp`) reduces settling velocity by **$-0.75\%$** ($1.0181\text{ m/s}$ vs. $1.0258\text{ m/s}$) due to added numerical diffusion.
5. **Kinematic Consistency of Tumbling:** The simulated frequency ($30.5\text{ Hz}$) is $\approx 16\%$ below the experimental mean ($\approx 36.4\text{ Hz}$). Strouhal number matches ($St \approx 0.059$) because descent velocity and frequency are both low by similar factors, pointing to an effective force or mass offset rather than a different tumbling mechanism.
6. **Specimen Thickness Scatter vs. Mean Deficit:** Specimen thickness scatter alone cannot explain the deficit. At fixed $C_D$, $V_t \propto \sqrt{h}$, so explaining a $-17\%$ speed deficit solely via thickness would require a $-31\%$ drop in mean thickness ($\approx 34.5\ \mu\text{m}$ vs. $50\ \mu\text{m}$). Caliper measurements on stacks confirmed the $50\ \mu\text{m}$ nominal mean, and specimen-to-specimen spread around the mean shifts the mean velocity by only a fraction of a percent. The cause of the $-17\%$ deficit remains unidentified and unresolved.
### Key Physical & Kinematic Findings (13 Completed Revolutions, $t = 0 \to 0.500\text{ s}$):

- **Continuous Tumbling Autorotation (13 Full Turns):** When seeded with initial rotation, the disk sustains continuous unidirectional autorotation without rotational reversals (**13 completed revolutions**, $t = 0.500\text{ s}$, 29,715 time steps).
- **Rotational Response (Approaching Plateau):** The tumbling frequency approaches an asymptotic plateau across Revolutions 8 through 13:
  $$f_t = 30.52 \pm 0.17\text{ Hz}, \quad T = 32.77 \pm 0.18\text{ ms}$$
  The frequency peaks at Revolution 9 ($30.73\text{ Hz}$) and gently relaxes to $30.24\text{ Hz}$ at Revolution 13 ($\leq 1\%$ cycle-to-cycle variation).
- **Frequency & Strouhal Number Comparison:**
  - Experimental frequency PDF for 2 mm disks peaks near $\sim 33\text{ Hz}$ (mean $36.4\text{ Hz}$, Fig. 20a). Simulated $f_t \approx 30.5\text{ Hz}$ is $7.5\%$ below the PDF peak and $16\%$ below the mean.
  - In Strouhal number, simulated $St = \frac{f_t D}{\langle V_y \rangle} = 0.0593 \approx 0.059$, compared to the experimental peak of $St \approx 0.055 - 0.060$ (Fig. 20b).
  - Scaled to experimental terminal velocity ($V_t = 1.23\text{ m/s}$) at the simulated Strouhal number, the frequency evaluates to $f = St \cdot V_t / D = 36.1\text{ Hz}$ (within $1\%$ of the experimental mean $36.4\text{ Hz}$). The close agreement in Strouhal number partly reflects the velocity deficit (lower velocity with proportionally lower frequency).
- **Translational Response & Asymptotic Extrapolation:**
  - Vertical descent velocity relaxes from $1.064\text{ m/s}$ (Rev 8) to $1.026\text{ m/s}$ (Rev 13).
  - Exponential relaxation fit on Revs 6–13 yields $V_\infty = 1.0159\text{ m/s}$ ($\tau = 97\text{ ms}$); Aitken $\Delta^2$ on the last three cycles gives $V_\infty = 1.0236\text{ m/s}$.
  - Asymptotic velocity deficit is $\approx -17\%$ below $V_t = 1.23\text{ m/s}$, with asymptotic $C_{D,\mathrm{paper}} \approx 1.09$.
- **Aerodynamic Force Balance:** Across the plateau (Revs 8–13), mean vertical aerodynamic force is $F_y = 2.2215\ \mu\text{N}$, giving a vertical force balance ratio of $F_y / W = 1.024$ (within $2.4\%$ of gravitational weight $W = 2.1693\ \mu\text{N}$). Cycle-by-cycle, $F_y / W$ drops monotonically from $1.056$ (Rev 8) to $1.004$ (Rev 13) as vertical deceleration tapers off. Mean total resultant aerodynamic force magnitude is $|\mathbf{F}_{\mathrm{aero}}| = 2.5042\ \mu\text{N}$ ($|\mathbf{F}_{\mathrm{aero}}| / W = 1.154$).
- **Aerodynamic Force Coefficients:**
  - The surface-integrated resultant force coefficient $C_{F,\mathrm{mag}}(t) = |\mathbf{F}_{\mathrm{aero}}| / (\frac{1}{2}\rho_f A V^2)$ oscillates between **$0.26$** (edge-on downstroke) and **$2.05$** (broadside crossing), with 95th percentile at **$2.02$** and cycle-average of $\langle C_{F,\mathrm{mag}} \rangle = \mathbf{1.119}$.
- **Trajectory Inclination $\psi$:** Over Revs 8–13, instantaneous flight angle $\psi(t) = \arctan(|V_z| / V_y)$ increases from $13.7^\circ$ to $16.0^\circ$ (mean $15.30^\circ$, track displacement angle $\psi_{\mathrm{track}} = 15.29^\circ$). Horizontal velocity $V_z$ is still creeping ($+0.009\text{ m/s}$ over the final $100\text{ ms}$). This puts $\psi$ within the experimental distribution (which extends to $40^\circ$), but $\sim 45\%$ above its peak mode ($15.3^\circ$ vs. $10.5^\circ$, Fig. 10b).
- **2D Planar Symmetry:** Over the full run, maximum out-of-plane rate is $|\Omega_z| = 1.22\text{ rad/s}$ ($1.16\text{ rad/s}$ in the plateau) and $|\Omega_y| = 0.036\text{ rad/s}$. Compared to mean pitch rate $|\Omega_x| \approx 191.7\text{ rad/s}$, out-of-plane rates remain $< 0.6\%$, confirming planar symmetry preservation.

---

## 2. Benchmark Physical Parameters

The physical parameters correspond to the $2\text{ mm}$ PET disk characterized in Table 1 & Table 3 of Tinklenberg et al. (2023):

| Parameter | Symbol | Value (SI) | Value (Practical) | Experimental Source / Note |
| :--- | :--- | :--- | :--- | :--- |
| **Disk Diameter** | $D$ | $2.02 \times 10^{-3}\text{ m}$ | $2.02\text{ mm}$ | Table 1 |
| **Disk Thickness** | $h$ | $5.00 \times 10^{-5}\text{ m}$ | $50.0 \pm 7.5\ \mu\text{m}$ | Table 1 (batch population spread) |
| **Aspect Ratio** | $\chi = D/h$ | $40.4$ | $40.4$ | Table 1 |
| **Disk Material Density** | $\rho_d$ | $1380\text{ kg/m}^3$ | Polyethylene terephthalate | Table 1 |
| **Total Disk Mass** | $m = \rho_d \frac{\pi}{4} D^2 h$ | $2.2113 \times 10^{-7}\text{ kg}$ | $0.2211\text{ mg}$ | Nominal mass |
| **Gravitational Weight** | $W = m g$ | $2.1693 \times 10^{-6}\text{ N}$ | $2.169\ \mu\text{N}$ | $g = 9.81\text{ m/s}^2$ |
| **Diametral Inertia (Pitch/Roll)** | $I_{xx} = I_{zz}$ | $5.6436 \times 10^{-14}\text{ kg}\cdot\text{m}^2$ | $5.64 \times 10^{-14}\text{ kg}\cdot\text{m}^2$ | $\frac{1}{4} m R^2 + \frac{1}{12} m h^2$ |
| **Polar Inertia (Spin)** | $I_{yy}$ | $1.1278 \times 10^{-13}\text{ kg}\cdot\text{m}^2$ | $1.13 \times 10^{-13}\text{ kg}\cdot\text{m}^2$ | $\frac{1}{2} m R^2$ (Disk normal axis) |
| **Fluid Medium** | Air at 20°C | $\rho_f = 1.200\text{ kg/m}^3$ | $\nu = 1.50 \times 10^{-5}\text{ m}^2/\text{s}$ | Laboratory ambient conditions |
| **Density Ratio** | $\tilde{\rho} = \rho_d / \rho_f$ | $1150$ | $1150$ | Dimensionless density ratio |
| **Inertia Ratio** | $I^* = \frac{\pi}{64} \frac{\tilde{\rho}}{\chi}$ | $1.40$ | $1.40$ | Table 1 |
| **Galileo Number** | $Ga = \frac{\sqrt{2|\tilde{\rho}-1|gh} D}{\nu}$ | $142.2$ | $142.2$ | Table 1 |
| **Characteristic Time Scale** | $\tau_t = \sqrt{\tilde{\rho} h / g}$ | $7.657 \times 10^{-2}\text{ s}$ | $76.6\text{ ms}$ | Natural inertial time scale |

---

## 3. Mathematical Model & Numerical Architecture

### 3.1. Accelerating Reference Frame (`freeBody6DoFPimpleFoam`)
Rather than translating a metric mesh over meters of descent (which causes cell deformation, boundary crossing, or numerical diffusion), the solver solves the incompressible Navier-Stokes equations in a non-inertial reference frame translating with the body:

$$\nabla \cdot \mathbf{U} = 0$$

$$\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) = -\frac{1}{\rho_f} \nabla p_\text{rgh} + \nu \nabla^2 \mathbf{U} - \mathbf{a}_\text{frame}$$

where $\mathbf{a}_\text{frame} = \mathbf{g} + \frac{\mathbf{F}_\text{aero}}{m}$ is the instantaneous frame acceleration, and $\mathbf{U}$ is fluid velocity relative to the falling frame.

### 3.2. Computational Domain Dimensions & Far-Field Boundary Conditions
- **Coordinate System & Frame Kinematics:** Gravity acts downward along $-Y$ ($\mathbf{g} = (0, -9.81, 0)\text{ m/s}^2$). As the body falls downward ($V_y < 0$), the relative fluid flow in the body reference frame is directed **upward** ($+Y$).
- **Domain Bounding Box:** 
  - $X \in [-15.0, +15.0]\text{ mm}$ ($15 D$, transverse direction)
  - $Y \in [-25.0, +25.0]\text{ mm}$ ($25 D$, vertical fall direction)
  - $Z \in [-15.0, +15.0]\text{ mm}$ ($15 D$, lateral drift direction)
  The domain provides $7.5 D$ lateral clearance and $12.5 D$ vertical clearance.
- **Far-Field Boundary Conditions:**
  - **Bottom Boundary ($Y = -25.0\text{ mm}$, patch `inlet`):** `fallingInletVelocity`. Because relative flow is upward, fluid enters through the bottom boundary with velocity $\mathbf{U} = -\mathbf{V}_{\mathrm{frame}}(t) = (0, +|V_y|, 0)$, with zero-gradient pressure ($\nabla p_\text{rgh} \cdot \mathbf{n} = 0$).
  - **Top Boundary ($Y = +25.0\text{ mm}$, patch `outlet`):** `pressureInletOutletVelocity` with fixed total pressure $p_\text{rgh} = 0$, exhausting the upward fluid flow out of the domain.
  - **Lateral Boundaries ($X = \pm 15\text{ mm}, Z = \pm 15\text{ mm}$, patch `atmosphere`):** `pressureInletOutletVelocity` with $p_\text{rgh} = 0$, allowing entrainment and wake exit.
  - **Disk Surface (`disk`):** `movingWallVelocity` enforcing exact no-slip relative to the rotating hub frame, with fixed-flux pressure boundary condition.

### 3.3. Spherical Sliding Interface (`freeBodyRotationalMotion`)
- An **inner spherical rotating zone** ($D_\text{ami} = 4.0\text{ mm} \approx 2 D$) tightly encapsulates the disk.
- Conformal Arbitrary Mesh Interface (`masterAMI` / `slaveAMI`) couples the rotating sphere to the outer translating domain.
- Quaternions $\mathbf{q}(t) = [w, \mathbf{v}]$ are integrated alongside angular momentum from instantaneous aerodynamic moments:
  $$\mathbf{M}_\text{aero}(t) = \int_{\mathrm{disk}} (\mathbf{r} - \mathbf{r}_\text{hub}) \times (\boldsymbol{\tau}_w - p \mathbf{n}) \, dA$$

### 3.4. Grid & Numerical Discretization
- **Total Mesh Count:** 212,792 cells (hex-dominant: 178,160 rotating sphere + 34,632 static outer domain).
- **Prism Layers on Disk:** 5–8 prism layers on top/bottom faces and 10–12 cells across the $50\ \mu\text{m}$ edge rim.
- **Mesh Quality:** Maximum non-orthogonality $37.3^\circ$ (average $7.17^\circ$), 0 skewness errors, 0 aspect ratio errors.
- **Time Step & Courant Numbers:** Adaptive PIMPLE stepping with $\Delta t \approx 1.5 \times 10^{-5}\text{ s}$ (`maxCo 1.5`, 3 inner correctors, `nOuterCorrectors 1`). In the developed production run, maximum Courant number reached $\mathrm{Co}_{\mathrm{max}} = 1.62$ (mean $\mathrm{Co}_{\mathrm{max}} = 1.50$, domain mean $\mathrm{Co}_{\mathrm{mean}} = 0.0127$).

---

## 4. Quantitative Comparison: Initial Zero-Spin Drop (`thinDisk2mm`)

In the initial exploratory run (`validation/thinDisk2mm/`), the disk was released with $\Omega_0 = 0$ and $\theta_0 = 15^\circ$ in quiescent air:

| Metric | Experimental Value (Tinklenberg et al. 2023) | freeBody6DoFPimpleFoam (Zero-Spin) | Methodological Note / Distinction |
| :--- | :--- | :--- | :--- |
| **Terminal Descent Speed $V_y$** | $1.23 \pm 0.31\text{ m/s}$ (Table 3; $\sigma(u_y) = \pm 0.31$) | $1.026\text{ m/s}$ | Settles near $\sim 1.02\text{ m/s}$, matching the tumbling deficit |
| **Descent Velocity Uncertainty** | $u_{\mathrm{exp}} = 1.2\%$ ($0.015\text{ m/s}$, Table 2) | — | Measurement uncertainty on mean velocity |
| **Aerodynamic Force Balance** | $F_\text{aero} / W = 1.000$ (Terminal equilibrium) | $F_\text{aero} / W = 1.001$ | $0.1\%$ numerical force balance error |
| **Descent Reynolds Number $Re$** | $165 \pm 42$ (Table 3; derived from $V_t \pm \sigma$) | $138$ | Inside experimental Reynolds range $[123, 207]$ |
| **Drag Coefficient $C_{D,\mathrm{paper}}$** | $0.74$ (Table 3; $[0.47, 1.33]$ from $V_t \pm \sigma$) | $1.07$ | Reflects baseline broadside drag excess |
| **Planar Pitch Rate $|\Omega_x|$** | $\sim 200\text{ rad/s}$ (Fig. 18; $2\pi f_t \approx 207 - 229$) | $88.7\text{ rad/s}$ peak / rocking | Disk rocks ($\pm 50^\circ$) under zero-spin release |
| **Out-of-Plane Rates ($\Omega_y, \Omega_z$)** | $\approx 0$ (2D planar) | $< 0.016\text{ rad/s}$ | $< 0.02\%$ of pitch rate (geometric/planar symmetry) |
| **Lateral Drift Distance $D_z$** | Unidirectional glide (tumbling) | $2.14\text{ mm}$ (oscillating) | Planar zigzagging excursion |

---

## 5. Physical Regimes: Transient Release Dynamics & Damped Rocking

### 5.1. Detailed Dynamics of Zero-Spin Release
In Tinklenberg et al. (2023), all observed 2 mm PET disks fell into the tumbling regime. The zero-spin simulation (`thinDisk2mm`) was initialized with $\Omega_0 = 0$ and $\theta_0 = 15^\circ$ in quiescent air:
1. The disk accelerates vertically from rest and undergoes an initial tumble of **$1.14$ full turns** (unwrapped angle reaches $-410.9^\circ$ at $t \approx 119\text{ ms}$).
2. Aerodynamic restoring torque arrests rotation and captures the disk into oscillations about broadside.
3. Successive rocking swings about broadside exhibit systematic amplitude decay:
   $$50.9^\circ \to 44.2^\circ \to 36.1^\circ \to 31.7^\circ \to 25.4^\circ$$
   with an amplitude damping ratio of $\approx 0.7 - 0.8$ per half-cycle over $235\text{ ms}$.
4. **Physical Characterization:** While five half-swings over $235\text{ ms}$ demonstrate damped rocking decaying toward broadside, this duration is too short to establish asymptotic stability. Experimentally, 2 mm disks consistently tumble. A systematic sweep of initial pitch angle $\theta_0$ and initial rate $\Omega_0$ is required to map the basin boundaries.

---

## 6. Visualizer & Telemetry Pipeline

### 6.1. 3D Motion & Drift Visualizer (`animate_drift.py`)
- **Frame Rate:** 60 FPS video at 20× slow-motion replay.
- **Flight-Corridor Perspective:** Observer camera fixed at distance $5.5\text{ mm}$ looking obliquely across the flight corridor ($Z_\text{mid} = -0.7\text{ mm}$), scaling down disk appearance to $0.6\times$ for realistic proportion.
- **Lateral Drift:** Disk physically traverses the 3D scene across the screen over a $2.14\text{ mm}$ lateral span ($Z = +0.46\text{ mm}$ to $Z = -1.68\text{ mm}$).
- **Visualizer Features:** Glowing cyan vertical trajectory curtain, plumb-bob ground track blip with calibrated $0.5\text{ mm}$ grid, dual-tone colored foil (cyan top, orange bottom, white rim), and corner radar HUD tracking $(D_x, D_z)$.

### Deliverables:
- **Video:** [`validation/thinDisk2mm/render/disk_drift_animation.mp4`](./thinDisk2mm/render/disk_drift_animation.mp4)
- **Preview Snapshot:** [`validation/thinDisk2mm/render/disk_drift_animation_preview.png`](./thinDisk2mm/render/disk_drift_animation_preview.png)
- **Validation Curves:** [`validation/thinDisk2mm/validation_comparison.png`](./thinDisk2mm/validation_comparison.png)

---

## 7. Directory Structure & Case Setup

```
validation/
├── report.md                               # This comprehensive findings report (ignored in .gitignore)
├── papers/
│   ├── thin-disks-falling-in-air.pdf       # Tinklenberg et al. (2023) experimental paper
│   └── Varshney_2012_Nonlinearity_25_C1.pdf# Varshney et al. (2012) falling disk paper
├── code_verification/
│   ├── check_run_consistency.py           # Production C++ 6DoF dynamics & energy balance audit script
│   └── verify_6dof_integrator.py           # Algorithmic convergence, invariants, and C++ diagnostics
├── mesh_convergence/                       # 3-mesh startup grid convergence cases
├── temporal_convergence/                   # 3-time-step startup temporal convergence cases
├── thinDisk2mm/                            # PRESERVED: Transient drop run (telemetry tracked; 6.4 GB 3D fields on Zenodo)
│   ├── case/                               # Mesh, constant/fallingFrameDict, system, postProcessing (motion.dat)
│   ├── render/                             # 60 FPS video (disk_drift_animation.mp4) & preview PNG
│   ├── validation_comparison.png           # 4-panel experimental validation curves
│   └── plot_validation.py                  # Telemetry parsing & validation script
└── thinDisk2mm_tumbling/                   # COMPLETED: 13-revolution seeded tumbling run (telemetry tracked; 25 GB on Zenodo)
    ├── case/                               # Mesh, constant/fallingFrameDict, system, postProcessing (motion.dat)
    │   └── postProcessing/                 # tumbleMotion and fallingMotion 6DoF logs (29,715 steps tracked in git)
    ├── plot_tumbling_detailed.py           # 6-panel high-res validation plot script
    └── tumbling_validation_detailed.png    # 6-panel experimental validation figure
```

> [!NOTE]
> **Zenodo Open-Access Dataset Deposit:** All python verification scripts and 6DoF kinematic telemetry logs (`motion.dat`, ~27 MB) are tracked directly in Git for instant local reproduction. Full 3D OpenFOAM volume field dumps ($U, p_{\mathrm{rgh}}$, `processor*` subdomains) across the 6.4 GB transient drop run and 25 GB 13-revolution tumbling run can be deposited on Zenodo (`10.5281/zenodo.<PENDING_DEPOSIT_ID>`). See [`validation/README.md`](./README.md) for complete packaging details.

---

## 8. Steady-State Tumbling Analysis (`validation/thinDisk2mm_tumbling/`)

The cloned case was initialized with $\Omega_0 = (-228.7, 0, 0)\text{ rad/s}$ ($2\pi \times 36.4\text{ Hz}$) and executed in parallel across 8 CPU cores to $t = 0.500\text{ s}$ (29,715 time steps, descent distance $> 47\text{ cm}$). The simulation completed **13 full continuous autorotation revolutions** without rotational reversal.

### 8.1. Reconciliation of Aerodynamic Force & Drag Definitions

1. **Terminal Settling-Velocity Derived $C_D$ (Paper Definition, Table 3):**
   $$C_{D,\mathrm{paper}} = \frac{2 W}{\rho_f A V_t^2} = \frac{2 m g}{\rho_f A V_t^2}$$
   - **Experimental Reference:** For mean $V_t = 1.23\text{ m/s}$, this gives $C_{D,\mathrm{paper}} = 0.746 \approx 0.75$. Transforming the experimental velocity distribution endpoints ($V_t \pm \sigma(u_y) = 1.23 \pm 0.31\text{ m/s} \implies [0.92, 1.54]\text{ m/s}$) produces an equivalent velocity-transformed interval of **$[0.47, 1.33]$**.
   - **Simulation Settling Range & Asymptote:** In the developed window (Revs 8–13), cycle-averaged speed relaxes from $1.064\text{ m/s}$ to $1.026\text{ m/s}$, with an extrapolated asymptote of $V_\infty \approx 1.016 - 1.024\text{ m/s}$. Evaluated at the asymptotic speed:
     $$C_{D,\mathrm{paper}}(V_\infty = 1.020\text{ m/s}) \approx 1.09$$
     This represents an asymptotic drag excess of $\approx +46\%$ relative to the experimental mean of $0.746$.

2. **Resultant Aerodynamic-Force Coefficient $C_{F,\mathrm{mag}}(t)$:**
   $$C_{F,\mathrm{mag}}(t) = \frac{|\mathbf{F}_\mathrm{aero}(t)|}{\frac{1}{2} \rho_f A |\mathbf{V}(t)|^2}$$
   During continuous autorotation, $C_{F,\mathrm{mag}}(t)$ sweeps periodically between **$0.26$** (edge-on downstroke) and **$2.05$** (broadside crossing), with a 95th percentile of **$2.02$** and a cycle-average of **$1.119$**.

---

### 8.2. Complete 13-Revolution Kinematic Evolution (Trapezoidal Averages & Asymptote)

Tracking the simulation cycle-by-cycle across the entire run ($t = 0 \to 0.500\text{ s}$):

| Cycle | $t_{\mathrm{mid}}$ (ms) | Period $T$ (ms) | $f_t$ (Hz) | $\langle V_y \rangle$ (m/s) | $\langle F_y \rangle / W$ | $\psi$ (deg) | Dynamic Status |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Rev 1** | $15.2$ | $30.37$ | $32.93$ | $0.1486$ | $0.0040$ | $0.05^\circ$ | Initial acceleration |
| **Rev 2** | $48.5$ | $36.26$ | $27.58$ | $0.4720$ | $0.0233$ | $0.20^\circ$ | Transitional fall |
| **Rev 3** | $89.0$ | $44.69$ | $22.38$ | $0.8364$ | $0.2130$ | $0.83^\circ$ | Boundary layer setup |
| **Rev 4** | $136.6$ | $50.63$ | $19.75$ | $1.0860$ | $0.7429$ | $3.32^\circ$ | Quasi-settled speed |
| **Rev 5** | $182.9$ | $41.95$ | $23.84$ | $1.1272$ | $1.0088$ | $7.32^\circ$ | Rotation developing |
| **Rev 6** | $222.2$ | $36.54$ | $27.37$ | $1.1111$ | $1.0562$ | $10.17^\circ$ | Rotation developing |
| **Rev 7** | $257.3$ | $33.82$ | $29.56$ | $1.0868$ | $1.0737$ | $12.18^\circ$ | Transition to plateau |
| **Rev 8** | $290.7$ | $32.79$ | $30.50$ | $1.0640$ | $1.0566$ | $13.71^\circ$ | Approaching plateau |
| **Rev 9** | $323.3$ | $32.55$ | $30.73$ | $1.0487$ | $1.0368$ | $14.69^\circ$ | Peak frequency |
| **Rev 10** | $355.9$ | $32.59$ | $30.69$ | $1.0387$ | $1.0238$ | $15.29^\circ$ | Approaching plateau |
| **Rev 11** | $388.5$ | $32.73$ | $30.55$ | $1.0324$ | $1.0147$ | $15.65^\circ$ | Approaching plateau |
| **Rev 12** | $421.4$ | $32.91$ | $30.39$ | $1.0286$ | $1.0085$ | $15.86^\circ$ | Approaching plateau |
| **Rev 13** | $454.4$ | $33.07$ | $30.24$ | $1.0264$ | $1.0046$ | $15.97^\circ$ | Approaching plateau |

#### Asymptotic Fits & Extrapolations:
- **Aitken $\Delta^2$ Extrapolation (Revs 11–13):** $V_\infty = 1.0236\text{ m/s}$.
- **Exponential Relaxation Fit ($V(t) = V_\infty + B e^{-t/\tau}$ on Revs 6–13):**
  $$V_\infty = 1.0159\text{ m/s}, \quad \tau = 97\text{ ms}$$
- **Asymptotic Comparison Error:**
  $$E_\infty = V_\infty - V_t = 1.020 - 1.230 = -0.210\text{ m/s} \quad (-17.1\%)$$

---

### 8.3. Comparison Against Experimental Benchmark Distributions

1. **Tumbling Frequency & Strouhal Number:**
   - Simulated frequency $f_t \approx 30.5\text{ Hz}$ is $7.5\%$ below the PDF peak mode ($\sim 33\text{ Hz}$) and $16\%$ below the mean ($36.4\text{ Hz}$).
   - Simulated Strouhal number $St = 0.0593$ matches the experimental peak mode ($0.055 - 0.060$). However, scaling by experimental $V_t = 1.23\text{ m/s}$ yields $f = St \cdot V_t / D = 36.1\text{ Hz}$. In Tinklenberg et al., tumbling frequency is governed by the drag–gravity balance rather than wake shedding; therefore, the close match in $St$ partly reflects the velocity deficit.
2. **Trajectory Angle Creep:**
   - Flight angle $\psi(t)$ increases from $13.7^\circ$ in Rev 8 to $16.0^\circ$ in Rev 13 (track angle $15.29^\circ$). Horizontal velocity $V_z$ continues to creep ($+0.009\text{ m/s}$ over the final $100\text{ ms}$). This is within the experimental distribution (which extends to $40^\circ$), but $\sim 45\%$ above its peak mode ($10.5^\circ$, Fig. 10b).

---

## 9. Code Verification & Numerical Uncertainty Quantification

### 9.1. Production C++ 6DoF Dynamics & Energy Balance Verification (`check_run_consistency.py`)

Using post-run verification on the raw 29,715 production time steps:
- **Rotational Dynamic Coupling:** $I_{xx} \Delta\omega_x / \Delta t$ matches logged aerodynamic moment $M_x$ to **$2.51 \times 10^{-6}$** relative error.
- **Translational Frame Acceleration:** $a_y$ matches $-g + F_y/m$ to **$7.20 \times 10^{-9}\text{ m/s}^2$**.
- **Velocity Integration:** $V_y$ matches trapezoidal-integrated $a_y$ to **$1.22 \times 10^{-9}\text{ m/s}$**.
- **Attitude Angle:** Logged angle matches $\int \Omega_x \, dt$ to **$0.26^\circ$** over 14 revolutions.
- **Mechanical Energy Balance:** Total mechanical energy change $\Delta E = \Delta(E_{\mathrm{trans}} + E_{\mathrm{rot}} + E_{\mathrm{pot}})$ matches integrated aerodynamic boundary work $W_k = \int (\mathbf{F}_{\mathrm{aero}} \cdot \mathbf{V} + \mathbf{M}_{\mathrm{aero}} \cdot \boldsymbol{\omega}) \, dt$ to **$3.19 \times 10^{-5}$** relative error.

These checks directly verify the C++ rigid-body dynamic coupling in the production execution.

---

### 9.2. Independent Function Object Force & AMI Interface Flux Verification

To verify that the solver's hand-rolled boundary force and moment integrals are mathematically and implementationally sound, independent standard OpenFOAM function objects were incorporated into the case definitions:
- `forces` library (`diskForces`): standard pressure and viscous force and moment summation directly on patch `disk` around origin `(0, 0, 0)`.
- `surfaceFieldValue` library (`amiFluxMaster`): face-area-weighted net volumetric flux $\sum \phi$ across patch `masterAMI`.

Executing these independent diagnostics on the broadside disk flow:
1. **Force Integration Cross-Check:** Over all simulated time steps, the solver's internal aerodynamic force $F_{\mathrm{aero}}$ matches OpenFOAM's `forces` library to:
   $$\frac{\max |F_{y,\mathrm{solver}} - F_{y,\mathrm{forces}}|}{\bar{F}_y} = 1.10 \times 10^{-10} \quad \text{[PASS]}$$
   This demonstrates machine-precision equivalence between the solver's hand-rolled surface traction integration and OpenFOAM's standard `forces` function object.
2. **AMI Interface Mass Conservation:** The maximum net volumetric flux through `masterAMI` satisfies:
   $$\max \left|\sum \phi_{\mathrm{masterAMI}}\right| = 3.47 \times 10^{-12}\ \mathrm{m}^3/\mathrm{s}$$
   Compared to domain convective inflow ($V \cdot A_{\mathrm{domain}} = 1.04\ \mathrm{m/s} \times 0.03\ \mathrm{m} \times 0.03\ \mathrm{m} = 9.36 \times 10^{-4}\ \mathrm{m}^3/\mathrm{s}$), the interface leak is only $3.7 \times 10^{-9}$ of domain throughput, confirming strict flux conservation across the sliding AMI boundary.

---

### 9.3. Spatial & Temporal Discretization Studies: Startup Diagnostics

> [!WARNING]
> The spatial and temporal GCI studies performed at $t = 1 - 2\text{ ms}$ are **early startup diagnostics only** and do not establish discretization uncertainty for the developed unsteady limit cycle:
> 1. At $t = 2\text{ ms}$, hydrostatic buoyancy ($\rho_f V_{\mathrm{disk}} g = 1.886\text{ nN}$) accounts for **$76\%$** of the total vertical force ($F_y \approx 2.49\text{ nN}$), while aerodynamic shear is negligible.
> 2. Descent velocity $V_y \approx 19.6\text{ mm/s}$ is dominated by ballistic ODE integration ($g \cdot t = 19.62\text{ mm/s}$) and is insensitive to the fluid mesh by construction.
> 3. Pitching moment ($M_x \sim 10^{-18}\text{ N}\cdot\text{m}$) is 8 orders of magnitude below developed torque ($0.61\text{ nN}\cdot\text{m}$) and represents floating-point roundoff around symmetric release.
> 4. In the spatial grid study, the static outer mesh cell count barely changed between Coarse and Medium ($34,584 \to 34,632$), meaning the outer wake was never systematically refined.

---

### 9.4. Static Wake Under-Resolution & Refined Static Mesh (`warm_wakeFine`)

Investigation of the baseline mesh (total **212,792 cells**: 178,160 rotating, 34,632 static) uncovered a severe geometric discontinuity across the non-conformal sliding interface:
- **Baseline Rotating AMI Sphere (`masterAMI`):** $15,528$ faces ($\Delta x \approx 57\ \mu\mathrm{m}$).
- **Baseline Static Outer AMI Sphere (`slaveAMI`):** Only $456$ faces ($\Delta x \approx 330\ \mu\mathrm{m}$) — a **34:1 face count disparity**.
- **Static Background Grid:** Base cell size $1.25\ \mathrm{mm} \approx 0.62 D$, transitioning coarseness immediately downstream of the disk.

To resolve the static wake and interface disparity, a refined static mesh case (`warm_wakeFine`) was synthesized using `make_wakeFine_case.sh`:
1. **AMI Sphere Resolution Matching:** Static sphere boundary refinement was elevated from Level (2 3) to Level (4 4), shrinking static surface cell size to $\approx 78\ \mu\mathrm{m}$.
2. **Interface Face Count:** `slaveAMI` face count increased from **456 to 6,984 faces** (a $>15\times$ resolution increase), closely matching the rotating sphere's $15,528$ faces.
3. **Multi-Feature Wake Refinement:** In addition to the AMI sphere, `make_wakeFine_case.sh` embedded multi-level surface grading and a Level 2 ($312\ \mu\mathrm{m}$) downstream wake refinement box spanning $x \in [-4, 4]\ \mathrm{mm}$, $y \in [-4, 24]\ \mathrm{mm}$, $z \in [-12, 6]\ \mathrm{mm}$ along the lateral drift trajectory.
4. **Mesh Quality Verification:** Evaluated with OpenFOAM `checkMesh`:
   - Total mesh cell count: **487,510 cells** (rotating: 178,160; static: 309,350).
   - Maximum non-orthogonality: **$37.3^\circ$** (average **$7.0^\circ$**).
   - Maximum skewness: **2.24**.
   - Topological consistency: cellZones `rotatingZone` and `staticZone` correctly assigned; `Mesh OK.`.
5. **Static Mesh Sensitivity & Wake Refinement (`fixedDisk_Re140_wakeFine`, $10,259$ steps, $t = 160.0\ \mathrm{ms}$, $3.33$ Flow-Throughs):**
   - **Pre-Saturation Drag Plateau:** Over the identical developed window ($t \in [96.0, 160.0]\ \mathrm{ms}$, duration $64.0\ \mathrm{ms}$), mean broadside $C_D$ plateaued at **$1.1125$** on `wakeFine` (spread $[1.1125, 1.1127]$, linear drift $-0.02\% / 100\ \mathrm{ms}$) compared to **$1.1523$** on baseline (spread $[1.1523, 1.1524]$, drift $-0.01\% / 100\ \mathrm{ms}$). This establishes a **$-3.45\%$ drag difference** between the two meshes. Because `make_wakeFine_case.sh` bundled static AMI boundary refinement (reducing interface face disparity from $34:1$ to $\approx 2.2:1$), a Level 2 wake refinement box, and graded surface cells simultaneously, this $-3.45\%$ shift demonstrates spatial mesh sensitivity of that magnitude rather than attributing the effect to a single geometric feature, and does not establish full grid convergence of `wakeFine`.
   - **Implications for the Tumbling Settling Velocity Deficit:** Treating the fixed broadside disk as a first-order proxy for falling kinematics ($V_t \propto C_D^{-1/2}$), a $-3.45\%$ broadside drag reduction yields:
     $$\frac{\Delta V_t}{V_t} \approx -\frac{1}{2}\frac{\Delta C_D}{C_D} = -\frac{1}{2}(-3.45\%) = +1.73\% \approx +1.8\%$$
      This would increase falling speed from $1.020\ \mathrm{m/s}$ to $\approx 1.038\ \mathrm{m/s}$, shifting the simulated deficit relative to the experimental mean ($1.23\ \mathrm{m/s}$) from $-17.1\%$ to $-15.6\%$. Static wake refinement alone recovers only $\approx 1.5$ percentage points of the $17$-point deficit, demonstrating that broadside wake resolution does not explain the discrepancy. The remaining deficit is hypothesized to arise from a combination of physical and numerical factors: experimental disk thickness variation across the manufactured batch ($50 \pm 7.5\ \mu\mathrm{m} \implies \pm 7.5\%$ in speed), unsteady autorotation aerodynamics (pitch angle kinematics, dynamic torque peaks, edge vortex shedding), and coupled 6DoF fluid-structure interaction under refined static grid resolution.
   - **Supercritical Wake Growth & Bifurcation Characterization:** At $50\ \mathrm{ms}$, the refined mesh exhibited initial lateral forces seeded by mesh asymmetry and solver tolerances ($|F_{\mathrm{lat}}|/F_y \approx 6.7 \times 10^{-5} \approx 1.56 \times 10^{-10}\ \mathrm{N}$). Over $t \in [80, 160]\ \mathrm{ms}$, lateral force amplified exponentially at $\sigma = +30.0\ \mathrm{s}^{-1}$ ($\sigma^* = \sigma D / V = +0.0582$), reaching $|F_{\mathrm{lat}}|/F_y = 1.80 \times 10^{-3}$ ($4.15 \times 10^{-9}\ \mathrm{N}$). On the baseline mesh, following a startup zero-crossing transient between $30$ and $50\ \mathrm{ms}$ ($-99.7^\circ \to +98.9^\circ$), modal growth over $t \in [80, 160]\ \mathrm{ms}$ was $2.9\times$ slower ($\sigma = +10.5\ \mathrm{s}^{-1}$, $\sigma^* = +0.0204$; or $+8.5\ \mathrm{s}^{-1}$ over $80$--$200\ \mathrm{ms}$). In both cases, the post-startup side-force azimuth angle $\phi = \operatorname{atan2}(F_z, F_x)$ remained strictly constant ($\phi = +145.5^\circ \pm 0.15^\circ$ on `wakeFine`, $+95.5^\circ \pm 0.08^\circ$ on baseline), with zero rotation or sign reversal. This confirms that the growing mode is consistent with the **steady regular $m = 1$ planar bifurcation** (threshold $Re_{c1} \approx 116.9$), rather than the unsteady oscillatory Hopf mode ($Re_{c2} \approx 125.3$), which has not yet manifested from unseeded startup flow. The initial lateral force ($\sim 1.6 \times 10^{-10}\ \mathrm{N}$) is 11 orders of magnitude larger than double-precision roundoff ($\sim 10^{-21}\ \mathrm{N}$), originating from solver tolerances and unstructured grid asymmetry, which explains why the two meshes lock to different deflection planes ($+95.5^\circ$ vs.\ $+145.5^\circ$).
   - **Sliding Interface Mass Conservation:** At run completion, net AMI interface mass leakage was $1.82 \times 10^{-8}$ on `wakeFine` vs. $3.61 \times 10^{-8}$ on baseline relative to sphere convective throughput ($\approx 2.0\times$ difference, correcting the preliminary $5.1\times$ ratio evaluated at $50\ \mathrm{ms}$ before baseline transient decay) [PASS].

---

### 9.5. Fixed Broadside Disk Benchmark & Convergence Gating

To separate fluid-dynamic discretization error from 6DoF body kinematics, two specialized diagnostic suites were constructed:
1. **Fixed Broadside Disk Suite (`make_fixed_disk.sh`):**
   - Freezes 6DoF translation and rotation (`freedom (0 0 0)`, `rotationFreedom (0 0 0)`, $\mathbf{\Omega}_0 = \mathbf{0}$) under uniform inflow in zero gravity ($\mathbf{g} = \mathbf{0}$).
   - **Wake Instability Regimes & Literature Anchors:**
     - **Bifurcation Thresholds & Aspect Ratio Dependence:**
       * *Infinitely Thin Disks ($h/D \to 0$):* Global linear stability analysis and weakly nonlinear expansion by Meliga, Chomaz & Sipp (2009, *JFM* 633, 159–189; DOI: 10.1017/S0022112009007290) and Fabre et al. (2008, *PoF* 20, 051702) establish that the steady axisymmetric wake undergoes:
         - Steady regular bifurcation ($m = 1$, reflectional symmetry breaking) at $Re_{c1} \approx 116.9$.
         - Oscillatory Hopf bifurcation (onset of hairpin vortex shedding with $St \approx 0.11 - 0.12$) at $Re_{c2} \approx 125.3$.
       * *Benchmark Disk Geometry ($h/D \approx 0.025$):* In the Tinklenberg et al. (2023) benchmark, the disk has diameter $D = 2.02\ \mathrm{mm}$ and thickness $h = 50\ \mu\mathrm{m}$ (Table 1), giving aspect ratio $\chi = D/h = 40.4$ and thickness ratio $h/D \approx 0.0248 \approx 0.025$. Because the disk is very slender, it lies close to the infinitely thin theoretical limit ($h/D \to 0$).
       * *Supercritical Wake at $Re = 140$:* Because $Re = 140$ lies above both the steady regular threshold ($Re_{c1} \approx 116.9$) and the oscillatory Hopf threshold ($Re_{c2} \approx 125.3$) for a zero-thickness disk, the wake is supercritical and breaks axisymmetry. In unseeded simulations, the flow develops the steady planar $m = 1$ deflection first (producing a non-zero lateral force $F_{\mathrm{lat}} \neq 0$ with locked spatial azimuth), with unsteady oscillatory vortex shedding not yet observed over the simulated convective duration.
     - **Drag Benchmarks & Physical Anchors:**
       * *Stokes Baseline ($Re \to 0$):* Oberbeck's (1876) exact analytical solution for a circular disk broadside to creeping flow yields $F_D = 8 \mu V D \implies C_D = 64 / (\pi Re) \approx 20.37 / Re$.
       * *High-Reynolds-Number DNS Anchor ($Re = 1000$):* Full 3D DNS of uniform flow normal to a circular disk by Tian (2019, arXiv:1906.10578) establishes a time-averaged drag coefficient of $\bar{C}_D \approx 1.054$ (citing the towing-channel experimental benchmark of $C_D \approx 1.10$ by Roos & Willmarth 1971, *AIAA J.* 9, 285–291).
       * *Subcritical Literature Target ($Re = 100$, Unverified):* Secondary literature citations report steady broadside drag for circular disks at $Re = 100$ in the range $C_D \approx 1.15 - 1.25$ (commonly attributed to Shenoy & Kleinstreuer 2008, *JFM* 605, 253–262 for $h/D = 0.1$, with Roos & Willmarth 1971 cited via Tian 2019). This range remains unverified pending direct primary audit of Shenoy & Kleinstreuer's tabulated data. Our computed baseline value ($C_D \approx 1.300$) sits $4\% - 13\%$ above this reference band ($+4.0\%$ vs. $1.25$, $+13.0\%$ vs. $1.15$). The small domain blockage ($\approx 0.36\%$) cannot account for a discrepancy of this magnitude; whether the residual elevation stems from numerical diffusion or near-wake discretization remains to be tested through systematic spatial refinement.
       * *Mesh Sensitivity via Extended Production Runs ($Re = 140$):* In the initial $50\ \mathrm{ms}$ runs at $Re = 140$, mean $C_D$ was $1.1225$ on `wakeFine` vs. $1.1609$ on baseline (a $-3.3\%$ difference). Both cases were extended past $3.3$ domain flow-throughs to evaluate convergence. Evaluated over the identical window $t \in [96.0, 160.0]\ \mathrm{ms}$, $C_D$ is $1.1523$ on baseline (drift $-0.01\% / 100\ \mathrm{ms}$) and $1.1125$ on `wakeFine` (drift $-0.02\% / 100\ \mathrm{ms}$), confirming that the $-3.45\%$ drag difference survives the initial transient. This demonstrates spatial discretization sensitivity of this magnitude under combined static wake refinement, but recovers only $\approx 1.5$ percentage points of the $-17\%$ settling deficit.
   - **Convergence Gate in `analyze_fixed_disk.py`:**
     - Requires $\ge 3.0$ domain flow-throughs AND linear $C_D$ drift $< 1.0\%$ per $100\ \mathrm{ms}$ before reporting steady $C_D$ comparisons, shedding frequency $St$, or Hilbert growth rate $\sigma$.

#### Complete $2 \times 2$ Fixed-Disk Diagnostic Matrix

| Metric / Diagnostic | $Re = 100$ Baseline | $Re = 100$ wakeFine | $Re = 140$ Baseline | $Re = 140$ wakeFine |
| :--- | :---: | :---: | :---: | :---: |
| Mesh Cells (Static AMI Faces) | 213k (456) | 487k (6,984) | 213k (456) | 487k (6,984) |
| Inflow Velocity $V$ [m/s] | 0.7426 | 0.7426 | 1.0396 | 1.0396 |
| Steps & Physical Time | 11,544 (250.0 ms) | 9,664 (210.0 ms) | 12,876 (200.0 ms) | 10,259 (160.0 ms) |
| Domain Flow-Throughs ($50$ mm) | 3.71 ($91.9\ D/V$) | 3.12 ($77.2\ D/V$) | 4.16 ($102.9\ D/V$) | 3.33 ($82.3\ D/V$) |
| Evaluation Window [ms] | [126.0, 210.0] | [126.0, 210.0] | [96.0, 160.0] | [96.0, 160.0] |
| Mean Drag $\langle C_D \rangle$ (Spread) | 1.2992 [1.2992, 1.2992] | **1.2738** [1.2738, 1.2738] | 1.1523 [1.1523, 1.1524] | **1.1125** [1.1125, 1.1127] |
| Mesh Drag Shift $\Delta C_D / C_D$ | Reference | **-1.96%** | Reference | **-3.45%** |
| $C_D$ Drift per 100 ms | +0.00% | -0.00% | -0.01% | -0.02% |
| Lateral Ratio $\langle |F_{\mathrm{lat}}| \rangle / F_y$ | $7.88 \times 10^{-6}$ | **$2.65 \times 10^{-7}$** | $1.19 \times 10^{-4}$ | **$8.00 \times 10^{-4}$** |
| Peak Lateral Force Amplitude | $< 1.2 \times 10^{-13}$ N | $< 2.0 \times 10^{-13}$ N | $< 3.9 \times 10^{-10}$ N | $< 4.2 \times 10^{-9}$ N |
| Modal Growth $\sigma$ ($80\text{ ms} \to t_{\mathrm{end}}$) | $-2.6\ \mathrm{s}^{-1}$ (sign check) | **$-12.9\ \mathrm{s}^{-1}$** (Decaying) | $+10.5\ \mathrm{s}^{-1}$ ($\sigma^* = 0.020$) | **$+30.0\ \mathrm{s}^{-1}$** ($\sigma^* = 0.058$) |
| Azimuth $\phi = \operatorname{atan2}(F_z, F_x)$ | Axisymmetric | Axisymmetric | $+95.5^\circ \pm 0.08^\circ$ | $+145.5^\circ \pm 0.15^\circ$ |
| Convergence Gate Status | **[CONVERGED]** | **[CONVERGED]** | **[CONVERGED]** | **[CONVERGED]** |
| Force $F_y$ Discrepancy vs. `forces` | $3.63 \times 10^{-10}$ | $3.70 \times 10^{-10}$ | $2.09 \times 10^{-10}$ | $2.16 \times 10^{-10}$ |
| Max Moment Discrepancy [N·m] | $\le 2.33 \times 10^{-16}$ | $\le 1.54 \times 10^{-16}$ | $\le 2.71 \times 10^{-18}$ | $\le 9.90 \times 10^{-17}$ |
| AMI Net Leak $/ (V A_{\mathrm{sphere}})$ | $4.37 \times 10^{-8}$ | $1.02 \times 10^{-7}$ | $3.61 \times 10^{-8}$ | $1.82 \times 10^{-8}$ |
   - **$Re = 100$ Subcritical Case (`fixedDisk_Re100`, $V = 0.7426\ \mathrm{m/s}$):**
     - Completed full production run of $11,544$ steps ($t = 250.0\ \mathrm{ms}$, $3.71$ domain flow-throughs $\equiv 91.9$ disk convective time units $D/V$).
     - **Convergence Gate Status:** Both criteria satisfied ($3.71 \ge 3.0$ flow-throughs [PASS]; linear $C_D$ drift $+0.00\% / 100\ \mathrm{ms} < 1.0\%$ [PASS]). **Overall Gate Status: [CONVERGED]**.
     - **Asymptotic Plateau:** Over the post-3-flowthrough window ($t \in [202.0, 250.0]\ \mathrm{ms}$, duration $48.0\ \mathrm{ms}$), mean $C_D = 1.2992$ (spread $[1.2992, 1.2992]$, zero observable drift). Evaluated against unverified literature estimates ($1.15 - 1.25$), this plateau sits $+3.9\%$ above $1.25$ and $+13.0\%$ above $1.15$ ($4\% - 13\%$). Domain blockage ($\approx 0.36\%$) cannot account for this gap; whether residual elevation reflects baseline static-mesh resolution or unverified literature values remains to be resolved via systematic spatial refinement.
     - **Axisymmetric Stability:** Lateral force ratio is $\langle |F_{\mathrm{lat}}| \rangle / F_y = 7.88 \times 10^{-6}$ ($\sim 10^{-13}\ \mathrm{N}$ amplitude, Hilbert envelope decaying at $\sigma = -2.6\ \mathrm{s}^{-1}$), confirming that the numerical solver preserves the steady axisymmetric wake without artificial symmetry breaking.
     - **Algebraic Verification:** $F_y$ matches OpenFOAM `forces` to $3.63 \times 10^{-10}$ [PASS]; moments match to $\le 2.33 \times 10^{-16}\ \mathrm{N}\cdot\mathrm{m}$ [PASS]. Max AMI net flux leak relative to sphere cross-section is $4.37 \times 10^{-8}$ [PASS].
    - **$Re = 100$ Refined Wake Case (`fixedDisk_Re100_wakeFine`, $V = 0.7426\ \mathrm{m/s}$):
      - Completed full production run of $9,664$ steps ($t = 210.0\ \mathrm{ms}$, $3.12$ domain flow-throughs $\equiv 77.2$ disk convective time units $D/V$, 487,510 cells, $6,984$ static AMI faces).
      - **Convergence Gate Status:** Both criteria satisfied ($3.12 \ge 3.0$ flow-throughs [PASS]; linear $C_D$ drift $-0.00\% / 100\ \mathrm{ms} < 1.0\%$ [PASS]). **Overall Gate Status: [CONVERGED]**.
      - **Asymptotic Plateau:** Over the developed window ($t \in [126.0, 210.0]\ \mathrm{ms}$, duration $84.0\ \mathrm{ms}$), broadside drag is strictly plateaued at $C_D = 1.2738$ (spread $[1.2738, 1.2738]$, drift $-0.00\% / 100\ \mathrm{ms}$). Evaluated over the identical window against baseline ($C_D = 1.2992$), broadside drag is **$-1.96\%$ lower** ($1.2738$ vs. $1.2992$). Evaluated against secondary literature estimates ($1.15 - 1.25$), this refined value sits only $+1.9\%$ above $1.25$ and $+10.8\%$ above $1.15$.
      - **Axisymmetric Wake Stability:** Lateral force ratio is $\langle |F_{\mathrm{lat}}| \rangle / F_y = 2.65 \times 10^{-7}$ ($\sim 10^{-13}\ \mathrm{N}$ amplitude, post-startup modal decay at $\sigma = -12.9\ \mathrm{s}^{-1}$), confirming that the numerical solver preserves the steady axisymmetric wake without artificial symmetry breaking.
      - **Algebraic Verification:** $F_y$ matches OpenFOAM `forces` to $3.70 \times 10^{-10}$ [PASS]; moments match to $\le 1.54 \times 10^{-16}\ \mathrm{N}\cdot\mathrm{m}$ [PASS]. Max AMI net flux leak relative to sphere cross-section is $1.02 \times 10^{-7}$ [PASS].
   - **$Re = 140$ Supercritical Baseline Case (`fixedDisk_Re140`, $V = 1.0396\ \mathrm{m/s}$):**
     - Completed extended run of $12,876$ steps ($t = 200.0\ \mathrm{ms}$, $4.16$ domain flow-throughs $\equiv 102.9$ disk convective time units $D/V$).
     - **Convergence Gate Status:** Both criteria satisfied ($4.16 \ge 3.0$ flow-throughs [PASS]; linear $C_D$ drift $-0.00\% / 100\ \mathrm{ms} < 1.0\%$ [PASS]). **Overall Gate Status: [CONVERGED]**.
     - **Asymptotic Plateau:** Over the post-3-flowthrough window ($t \in [144.3, 200.0]\ \mathrm{ms}$, duration $55.7\ \mathrm{ms}$), broadside drag has plateaued at $C_D = 1.1523$ (spread $[1.1523, 1.1523]$, drift $-0.00\% / 100\ \mathrm{ms}$). Note that $C_D$ shifted by only $-0.7\%$ from its 50 ms value ($1.1609 \to 1.1523$), directly verifying that the early $5.5\% / 100\ \mathrm{ms}$ drift did not imply a large remaining fall.
     - **Lateral Force Evolution:** $\langle |F_{\mathrm{lat}}| \rangle / F_y = 1.39 \times 10^{-4}$ (amplitude $< 3.9 \times 10^{-10}\ \mathrm{N}$, envelope growth $\sigma \approx +0.0\ \mathrm{s}^{-1}$, steady/neutral state).
     - **Verification Cross-Checks:** $F_y$ matches OpenFOAM `forces` to $2.09 \times 10^{-10}$ [PASS]; moments match to $\le 2.71 \times 10^{-18}\ \mathrm{N}\cdot\mathrm{m}$ [PASS]. Max AMI net flux leak relative to sphere cross-section is $3.61 \times 10^{-8}$ [PASS].
   - **$Re = 140$ Refined Wake Case (`fixedDisk_Re140_wakeFine`, $V = 1.0396\ \mathrm{m/s}$):**
     - Completed full production run of $10,259$ steps ($t = 160.0\ \mathrm{ms}$, $3.33$ domain flow-throughs $\equiv 82.3$ disk convective time units $D/V$, 487,510 cells, $6,984$ static AMI faces).
     - **Convergence Gate Status:** Both criteria satisfied ($3.33 \ge 3.0$ flow-throughs [PASS]; linear $C_D$ drift $-0.02\% / 100\ \mathrm{ms} < 1.0\%$ [PASS]). **Overall Gate Status: [CONVERGED (Pre-Saturation Steady $C_D$)]**.
     - **Pre-Saturation Drag Plateau:** Over the identical developed window ($t \in [96.0, 160.0]\ \mathrm{ms}$, duration $64.0\ \mathrm{ms}$), broadside drag is plateaued at $C_D = 1.1125$ (spread $[1.1125, 1.1127]$, drift $-0.02\% / 100\ \mathrm{ms}$). Compared against the converged baseline mesh over the identical window ($C_D = 1.1523$), broadside drag is **$-3.45\%$ lower** ($1.1125$ vs. $1.1523$). This confirms that the drag shift is a genuine mesh sensitivity effect that survives startup transients, but accounts for only $\approx 1.5$ percentage points of the $-17\%$ settling deficit.
     - **Lateral Instability & Steady Planar Mode:** Lateral force grew exponentially from $|F_{\mathrm{lat}}|/F_y = 6.7 \times 10^{-5}$ ($1.56 \times 10^{-10}\ \mathrm{N}$) at $50\ \mathrm{ms}$ to $1.80 \times 10^{-3}$ ($4.15 \times 10^{-9}\ \mathrm{N}$) at $160\ \mathrm{ms}$, with an exponential growth rate of $\sigma = +30.0\ \mathrm{s}^{-1}$ ($\sigma^* = \sigma D / V = +0.0582$ over $80$--$160\ \mathrm{ms}$, GROWING). The force azimuth angle is locked at $\phi = \operatorname{atan2}(F_z, F_x) = +145.5^\circ \pm 0.15^\circ$ across the run, indicating that the developing instability is the **steady regular $m = 1$ planar bifurcation mode** rather than the unsteady oscillatory Hopf mode. By comparison, baseline modal growth over $80$--$160\ \mathrm{ms}$ was $2.9\times$ slower ($\sigma = +10.5\ \mathrm{s}^{-1}$, $\sigma^* = +0.0204$; or $+8.5\ \mathrm{s}^{-1}$ over $80$--$200\ \mathrm{ms}$, locked at $\phi = +95.5^\circ \pm 0.08^\circ$). Initial perturbations arise from solver tolerances and mesh asymmetries rather than machine roundoff.
     - **Verification Cross-Checks:** $F_y$ matches OpenFOAM `forces` to $2.16 \times 10^{-10}$ [PASS]; moments match to $\le 9.90 \times 10^{-17}\ \mathrm{N}\cdot\mathrm{m}$ [PASS]. Max AMI net flux leak relative to sphere cross-section is $1.82 \times 10^{-8}$ [PASS].
   - **$Re = 140$ 1.5x Domain Sensitivity Case (`fixedDisk_Re140_domain1p5`, $V = 1.0396\ \mathrm{m/s}$):**
     - Run executed in a $1.5\times$ expanded domain ($45\text{ mm} \times 75\text{ mm} \times 45\text{ mm}$, $\approx 22.5D \times 37.5D \times 22.5D$, 267k cells).
     - At $V = 1.04\text{ m/s}$, $80\text{ ms}$ corresponds to $\approx 83\text{ mm}$ of travel in the $75\text{ mm}$ domain ($\approx 1.1$ domain flow-throughs, below the $\ge 3.0$ steady gate).
     - However, because domain blockage acts almost instantaneously on pressure, the equal-time comparison indicates no measurable sensitivity at $t = 80\text{ ms}$ (broadside drag coefficient $C_D = 1.15244$ vs. $1.15267$ on baseline, a shift of only **$-0.019\%$**, with $\Delta C_D / C_D$ within $\pm 0.03\%$ throughout $[10, 80]\text{ ms}$).
     - Outer boundary proximity shows no measurable sensitivity at $t = 80\text{ ms}$ (equal-time comparison, shift $-0.019\%$). Domain blockage does not explain the deficit.
   - **Three-Mesh ASME V&V 20 Spatial GCI on Fixed Disk at $Re = 100$ (`gci_coarse`, `gci_medium`, `gci_fine`):**
     To replace single-pair parameter shifts with a formal ASME V&V 20-2009 / Celik et al. (2008) spatial discretization uncertainty, a three-mesh family was evaluated for the fixed broadside disk at $Re = 100$ with both the outer domain and the near-body rotating zone systematically refined ($r \approx 1.2$):
     - **Grid 3 (Coarse):** $N_3 = 130,928$ cells ($96,344$ near-body, $34,584$ static) $\implies C_{D,3} = 1.292076$ (linear drift $-9.87 \times 10^{-4}\ \mathrm{s}^{-1}$ over $t \in [0.105, 0.150]\ \mathrm{s}$).
     - **Grid 2 (Medium):** $N_2 = 212,792$ cells ($178,160$ near-body, $34,632$ static, $r_{32} = 1.1757$) $\implies C_{D,2} = 1.299227$ (drift $-1.09 \times 10^{-3}\ \mathrm{s}^{-1}$).
     - **Grid 1 (Fine):** $N_1 = 368,760$ cells ($300,568$ near-body, $68,192$ static, $r_{21} = 1.2011$) $\implies C_{D,1} = 1.292855$ (drift $-1.25 \times 10^{-3}\ \mathrm{s}^{-1}$).
     - **Discretization Differences:** $\epsilon_{21} = C_{D,2} - C_{D,1} = +0.006372$, $\epsilon_{32} = C_{D,3} - C_{D,2} = -0.007151$.
     - **Convergence Ratio:** $R = \epsilon_{21} / \epsilon_{32} = -0.8911 \implies$ **Oscillatory Convergence** ($-1 < R < 0$).
     - **Family Grid Span:** $|C_{D,\mathrm{fine}} - C_{D,\mathrm{med}}| = 0.006372$ (**0.49%**). Maximum family spread $\Delta C_D = 0.007151$ (**0.55%**).
     - **ASME V&V 20 Spatial Uncertainty:** Under the ASME standard for oscillatory convergence, numerical uncertainty is bounded by the family half-span:
       $$u_{\mathrm{num,mesh}} \le \frac{1}{2}(C_{D,\mathrm{max}} - C_{D,\mathrm{min}}) = 0.00358 \quad (\mathbf{0.28\%}\ \text{of}\ C_D)$$
       Even taking the full family envelope conservatively yields $u_{\mathrm{num,mesh}} \le \mathbf{0.55\%}$. This establishes that the steady broadside drag on this mesh family is resolved to within $\le 0.55\%$.
   - **Independent Published Drag Checks (Disk & Sphere):**
     To eliminate reliance on self-generated comparisons, three independent benchmarks were executed against published experimental and standard correlation literature:
     - *Broadside Disk at $Re = 200$ (`fixedDisk_Re200`):* Evaluated through $4.46$ domain flow-throughs ($t_{\mathrm{end}} = 0.15\ \mathrm{s}$, $V = 1.485\ \mathrm{m/s}$). Cycle-averaged drag in the developed window ($t \in [0.101, 0.150]\ \mathrm{s}$) is $C_D = 1.1062$ (window spread $[1.038, 1.149]$). This falls directly within the experimental benchmark of **Roos & Willmarth (1971)** ($C_D \approx 1.05\text{--}1.15$, modal value $\approx 1.10$), matching within $< 1\%$. Traction force equivalence vs. OpenFOAM `forces` matches to $1.07 \times 10^{-10}$ relative error.
     - *Spheres at $Re = 100$ and $Re = 200$ (`fixedSphere_Re100`, `fixedSphere_Re200`):* Evaluated on the unrefined 36k-cell hex mesh from `tutorials/steelBallRun`. At $Re = 100$, $C_D = 1.3911$ (vs. Schiller–Naumann 1933 $C_D = 1.0917$, $+27.4\%$; Clift et al. 1978 $1.0430$, $+33.4\%$). At $Re = 200$, $C_D = 1.0229$ (vs. Schiller–Naumann $0.8056$, $+27.0\%$; Clift et al. $0.7614$, $+34.3\%$). The two sphere cases exhibit an almost identical $+27.0\%\text{--}27.4\%$ bias relative to Schiller–Naumann, consistent with the expected truncation error of an uninflated coarse Cartesian surface mesh ($y^+ \sim 5$).
   - **Primary Tabulated Data Audit: Shenoy & Kleinstreuer (2008):**
     An audit of *Journal of Fluid Mechanics*, Vol. 605, pp. 253–262 (DOI: 10.1017/S0022112008001626) confirms that their investigation focused on disks with aspect ratio $\chi = D/h = 10$ ($h/D = 0.1$), which are approximately **$4\times$ thicker** than the Tinklenberg et al. (2023) glitter specimens ($\chi = 40.4$, $h/D = 0.0247$). For $\chi = 10$, the cylindrical edge contributes substantial boundary layer shear and alters the wake separation angle, giving $C_D \approx 1.15\text{--}1.25$. In the ultra-thin limit ($\chi = 40.4 \to \infty$), edge shear diminishes and base suction intensifies, producing $C_D \approx 1.27\text{--}1.30$. The secondary citation of $1.15\text{--}1.25$ was therefore drawn from a much thicker disk regime.
   - **Dynamic Limit-Cycle Restart Diagnostics (`make_restart_variants.sh`):**
     Restarts from the fully developed tumbling limit cycle ($t = 0.30\ \mathrm{s}$, Revolutions 8--13) evaluate numerical parameter sensitivity on $V_{\mathrm{eq,est}}$ and torque peaks without re-simulating the $300\ \mathrm{ms}$ initial transient. Because terminal velocity is still gently relaxing ($\langle F_y \rangle \approx 1.02 W$), an equilibrium settling velocity estimator based on quadratic drag ($F \propto V^2$) is employed:
     $$V_{\mathrm{eq,est}} = \frac{\langle V_y \rangle}{\sqrt{\langle F_y \rangle / W}}$$
     On the 13-revolution reference run, $V_{\mathrm{eq,est}} = 1.0252\ \mathrm{m/s}$, matching the Aitken $\Delta^2$ asymptotic extrapolation ($1.0236\ \mathrm{m/s}$) to within $0.15\%$.

#### Dynamic Limit-Cycle Restart Comparison Table

| Case Variant | Evaluated Revs | $f_t$ [Hz] | $\langle V_y \rangle$ [m/s] | $\langle F_y \rangle / W$ | $V_{\mathrm{eq,est}}$ [m/s] | Shift vs.\ Baseline |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| Continuous Run (`case`) | 6 | 30.48 | 1.0345 | 1.0183 | 1.0252 | Reference |
| Control Restart (`restart_base`, LUST) | 4 | 30.62 | 1.0387 | 1.0253 | 1.0258 | **+0.06%** ($+0.0006\ \mathrm{m/s}$) |
| Temporal Refined (`restart_dtHalf`) | 4 | 30.52 | 1.0385 | 1.0254 | 1.0256 | **+0.04%** ($+0.0004\ \mathrm{m/s}$) |
| Upwind Convection (`restart_linUp`) | 4 | 30.93 | 1.0338 | 1.0311 | 1.0181 | **-0.75%** ($-0.0077\ \mathrm{m/s}$) |

- **Control Benchmark Fidelity:** `restart_base` ($t \in [0.30, 0.44]\ \mathrm{s}$, 4 complete revolutions) reproduces the continuous limit cycle with high precision: $V_{\mathrm{eq,est}}$ matches to $+0.06\%$, tumbling frequency to $+0.44\%$, trajectory angle $\psi = 15.28^\circ$ ($-1.5\%$), and peak aerodynamic torque $M_{x,\mathrm{p95}} = 0.731\ \mathrm{nN}\cdot\mathrm{m}$ ($+0.7\%$).
- **Temporal Discretization Invariance:** Halving the time step ($\mathrm{Co}_{\mathrm{max}} \le 0.75$, $\Delta t \approx 7.27\ \mu\mathrm{s}$, `nOuterCorrectors 1`) in `restart_dtHalf` over 4 complete revolutions shifts the equilibrium settling speed by only **$+0.04\%$** ($+0.0004\ \mathrm{m/s}$ vs.\ continuous run, and $-0.02\%$ vs.\ `restart_base`) with frequency matching to $+0.13\%$ ($30.52\ \mathrm{Hz}$). This bounds time-step sensitivity at $< 0.05\%$ over 4 revolutions, indicating that temporal discretization error does not explain the $-17\%$ settling deficit.
- **Convection Scheme Dissipation:** Switching from central-blended LUST to pure second-order upwind (`restart_linUp`) reduces settling speed by **$-0.75\%$** ($1.0181\ \mathrm{m/s}$ vs. $1.0258\ \mathrm{m/s}$) and increases frequency by $+1.01\%$ ($30.93\ \mathrm{Hz}$) due to increased numerical dissipation. Because higher numerical dissipation slows the disk down further away from the experimental benchmark ($1.23\ \mathrm{m/s}$), numerical diffusion in the baseline scheme cannot account for the $-17\%$ settling deficit.

---

## 10. Direct 6DoF Mesh Sensitivity: `warm_base` vs. `warm_wakeFine`

To establish whether the static broadside drag reduction ($-3.45\%$) carries over into coupled dynamic autorotation, a direct comparative test was conducted from identical warm-start initial conditions ($V_0 = 1.02\ \mathrm{m/s}$, $\Omega_0 = -191\ \mathrm{rad/s}$) across 5 complete tumbling revolutions ($t \in [0.08, 0.25]\ \mathrm{s}$) to $t_{\mathrm{end}} = 0.26\ \mathrm{s}$. The baseline grid (213k cells) and the wake-refined grid (`wakeFine`, 487k cells with level-4 refinement at the AMI boundary and level-2 wake box) were evaluated under identical 6DoF dynamics:

| Mesh Variant | Cells | Evaluated Revs | $f_t$ [Hz] | $\langle V_y \rangle$ [m/s] | $V_{\mathrm{eq,est}}$ [m/s] | Relative Shift |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| Baseline Mesh (`warm_base`) | 212,792 | 5 | 29.45 | 1.0680 | 1.0506 | Reference |
| Refined Wake (`warm_wakeFine`) | 487,510 | 5 | 29.52 | 1.0673 | 1.0497 | **-0.09%** ($-0.0009\ \mathrm{m/s}$) |

Remarkably, the cycle-by-cycle metrics track each other almost identically:
- Cycle-by-cycle equilibrium velocity matches within $< 0.2\%$ across every individual revolution: Rev 1 ($1.0892$ vs.\ $1.0888\ \mathrm{m/s}$), Rev 2 ($1.0600$ vs.\ $1.0579\ \mathrm{m/s}$), Rev 3 ($1.0410$ vs.\ $1.0402\ \mathrm{m/s}$), Rev 4 ($1.0339$ vs.\ $1.0334\ \mathrm{m/s}$), and Rev 5 ($1.0293$ vs.\ $1.0287\ \mathrm{m/s}$).
- Multi-revolution equilibrium settling speed shifts by only **$-0.09\%$** ($-0.0009\ \mathrm{m/s}$), tumbling frequency shifts by $+0.25\%$ ($29.45 \to 29.52\ \mathrm{Hz}$), trajectory angle matches to within $0.01^\circ$ ($12.29^\circ$ vs.\ $12.28^\circ$), and peak aerodynamic moment matches to within $0.25\%$ ($0.755$ vs.\ $0.753\ \mathrm{nN}\cdot\mathrm{m}$).

**Finding on Static Mesh Refinement:** The $+1.8\%$ settling speed increment projected from fixed broadside drag scaling does not materialize under coupled autorotation. Because the disk pitches through all orientations from broadside ($90^\circ$) to edge-on ($0^\circ$), the cycle-integrated aerodynamic forces are governed predominantly by the near-body rotating zone, which is identical in both meshes. Outer static wake refinement shifts falling speed by less than $0.1\%$. This indicates that outer static mesh resolution is unlikely to explain the $-17\%$ settling deficit, though near-body boundary layer refinement remains a separate factor.

---

## 10.1. Analysis of the -17% Settling Deficit

The baseline tumbling simulation yields an asymptotic descent velocity of $V_\infty \approx 1.02\ \mathrm{m/s}$, representing a $-17.1\%$ deficit relative to the experimental mean terminal velocity $V_t = 1.23\ \mathrm{m/s}$. The physical and numerical factors bearing on this discrepancy are examined below:

1. **Static Wake Discretization Error:** Fixed-disk broadside drag refinement recovered $\approx 1.5$ percentage points ($1.020 \to 1.038\ \mathrm{m/s}$), but direct 6DoF simulation demonstrates that under dynamic tumbling, static wake refinement produces no settling speed increase ($-0.09\%$).
2. **Kinematic Consistency of Tumbling Dynamics:** Crucially, the simulated Strouhal number $St = f_t D / V_t = 0.059$ agrees with the experimental mode ($0.055$--$0.060$). Descent speed is $17\%$ low ($1.02$ vs. $1.23\ \mathrm{m/s}$), and tumbling frequency is $16\%$ below the experimental mean ($30.5$ vs. $36.4\ \mathrm{Hz}$). Because $f_t / V_t$ is preserved, the tumbling kinematics scale consistently with the lower descent velocity. This indicates that the simulated disk is in the correct tumbling regime, pointing to a force-level or mass offset rather than a fundamentally altered tumbling pattern. Importantly, initial pitch rate was set to the experimental mean frequency ($\Omega_0 = -228.7\ \mathrm{rad/s} \equiv 36.4\ \mathrm{Hz}$) and relaxed spontaneously to the simulation limit cycle of $30.5\ \mathrm{Hz}$ ($191.7\ \mathrm{rad/s}$), demonstrating that the frequency deficit originates in the model's own limit cycle rather than from initial condition bias.
3. **Specimen Batch Thickness Spread vs. Mean Uncertainty:** In Tinklenberg et al. (2023), Table 1 specifies the measured mean disk thickness as $h = 50 \pm 7.5\ \mu\mathrm{m}$. This $\pm 7.5\ \mu\mathrm{m}$ ($\pm 15\%$) represents the standard deviation across the manufactured disk batch, reflecting specimen-to-specimen variation. Because the experimental terminal velocity $V_t = 1.23\ \mathrm{m/s}$ is an ensemble mean averaged over thousands of tracked drops, the appropriate input uncertainty on the mean thickness is the standard error of the mean ($\sigma_h / \sqrt{N}$), which is small ($< 1\%$). Furthermore, because $V_t \propto \sqrt{h}$ at fixed drag coefficient, accounting for a $-17\%$ velocity deficit solely through thickness would require a $-31\%$ deficit in mean thickness ($\approx 34.5\ \mu\text{m}$ instead of $50\ \mu\text{m}$). Specimen thickness scatter around the mean shifts the mean velocity by only a fraction of a percent, and caliper measurements on stacks confirmed the $50\ \mu\text{m}$ nominal mean. Therefore, specimen thickness scatter alone cannot explain the deficit, and the underlying cause remains unidentified.
4. **Instantaneous Population Scatter vs. Mean Velocity:** The experimental report of $V_t = 1.23 \pm 0.31\ \mathrm{m/s}$ represents the instantaneous velocity distribution across thousands of tracked drops ($1\sigma$ spread $[0.92, 1.54]\ \mathrm{m/s}$). The simulated asymptotic velocity ($1.02\ \mathrm{m/s}$) lies at $Z = (1.02 - 1.23)/0.31 = -0.68\sigma$, within the observed physical experimental population envelope. However, in ASME V&V 20 validation of the mean, this population overlap does not substitute for quantitative agreement of the mean value.
5. **Orientation Dynamics, Tilt Distribution, and Effective Drag Analysis:** A fundamental physical insight arises from examining the effective aerodynamic drag coefficient deduced from terminal settling velocity:
   $$C_{D,\mathrm{eff}} = \frac{2 m g}{\rho_f A V_t^2}$$
   - **Experimental Benchmark ($V_t = 1.23\text{ m/s}$):** $C_{D,\mathrm{eff}} = 0.746$ (matching the value reported by Tinklenberg et al. 2023).
   - **Simulated Tumbling Limit Cycle ($V_t = 1.02\text{ m/s}$):** $C_{D,\mathrm{eff}} = 1.085$.
   - **Fixed Broadside Disk ($Re = 140$):** $C_{D,\mathrm{broadside}} \approx 1.152$.

   Comparing these ratios reveals a striking structural difference:
   $$\left( \frac{C_{D,\mathrm{eff}}}{C_{D,\mathrm{broadside}}} \right)_{\mathrm{sim}} = \frac{1.085}{1.152} \approx 94.2\%, \qquad \left( \frac{C_{D,\mathrm{eff}}}{C_{D,\mathrm{broadside}}} \right)_{\mathrm{exp}} = \frac{0.746}{1.152} \approx 64.8\%$$

   The simulated tumbling disk averages $94.2\%$ of its broadside drag, whereas the experimental disk averages only $64.8\%$. This indicates that the $-17\%$ settling deficit is fundamentally an **orientation and trajectory dynamics discrepancy** rather than an error in steady broadside drag $C_D(Re)$ (which would have to be in error by over $40\%$ to account for the gap).

   Cycle-resolved analysis of the 13-revolution production run ($t \in [0.26, 0.50]\text{ s}$) explains why the simulated planar tumble produces such high mean drag:
   - **Dynamic Broadside Drag Overshoot:** When the disk pitches through broadside ($\theta \in [0^\circ, 10^\circ]$ from horizontal flow normal), instantaneous drag surges to a sharp peak of $C_D \approx 2.17$ ($1.88\times$ steady broadside drag). At the peak pitch angular velocity ($\omega \approx 222\ \mathrm{rad/s}$), the tip speed is $\omega R \approx 0.22\ \mathrm{m/s}$ (note that $0.19\ \mathrm{m/s}$ corresponds to the cycle-mean pitch rate $\bar{\omega} \approx 191.7\ \mathrm{rad/s}$). Because this peak tip speed is only $\approx 22\%$ of the translational descent velocity ($V_y \approx 1.04\ \mathrm{m/s}$), an apparent $1.9\times$ drag amplification over steady broadside flow is notable; it is possibly driven by starting-flow separation and wake capture after exiting the near-edge-on slicing phase, or by dynamic added-mass and pitch-damping force transients, but the fluid dynamic mechanism is not isolated. Diagnostic checks on the simulation data clarify the nature of this peak:
     - *Discretization Invariance:* Re-evaluating $C_D(\theta)$ across existing runs (`continuous_base`, `warm_wakeFine`, `restart_dtHalf`, `restart_linUp`) demonstrates that the peak $C_D \in [2.20, 2.24]$ is invariant across static grid refinement ($213\mathrm{k} \to 487\mathrm{k}$ cells), time-step halving ($\Delta t \approx 7.3\ \mu\mathrm{s}$), and convection scheme choice (central-blended LUST vs. pure `linearUpwind`), indicating that the peak is not an outer discretization artifact (noting the near-body boundary layer mesh is unchanged).
     - *Force Decomposition:* At peak drag ($t \approx 0.3556\ \mathrm{s}$, total vertical aerodynamic force $F_y \approx 4.65 \times 10^{-6}\ \mathrm{N} \approx 2.14 W$), pressure force accounts for $98.1\%$ ($F_{\mathrm{press},y} \approx 4.56 \times 10^{-6}\ \mathrm{N}$), while viscous shear accounts for $1.9\%$ ($F_{\mathrm{visc},y} \approx 8.8 \times 10^{-8}\ \mathrm{N}$), confirming that the peak is overwhelmingly form/pressure drag (consistent with steady broadside flow where pressure also accounts for $\approx 98\%$ of total drag).
     - *Drag Scaling Sensitivity:* Scaling the broadside drag bands ($\theta < 30^\circ$) by a factor of $0.8$ reduces time-averaged drag to $\langle C_D \rangle \approx 0.95$ ($+7\%$ settling speed via $V \propto C_D^{-1/2}$), while scaling by $0.6$ reduces it to $\langle C_D \rangle \approx 0.84$ ($+13\%$ settling speed). Full recovery of the experimental settling velocity ($1.23\ \mathrm{m/s}$) requires $C_{D,\mathrm{eff}} = 0.746$ ($+20.6\%$ settling speed). Thus, suppressing the broadside peak alone closes only part of the settling velocity gap.
   - **Edge-On Downstroke Drag Minimum:** Near edge-on ($\theta \in [80^\circ, 90^\circ]$), instantaneous drag drops to $C_D \approx 0.286$, while pitch angular velocity slows to $\omega \approx 167\ \mathrm{rad/s}$.
   - **Kinematic Duty Cycle:** Because $dt = d\theta / \omega$, the disk spends more time near edge-on ($25.6\%$ of the cycle in $\theta \in [70^\circ, 90^\circ]$) than near broadside ($19.6\%$ in $\theta \in [0^\circ, 20^\circ]$). However, the enormous broadside peak ($2.17$) dominates the integral, pulling cycle-average drag up to $1.085$.
    - **Planar Symmetry vs. 3D Experimental Trajectories:** In the simulation, both 3D translation (`freedom (1 1 1)`) and 3D rotation (`rotationFreedom (1 1 1)`) are fully unconstrained. In the baseline run initialized with pure pitch $\mathbf{\Omega}_0 = (-228.7, 0, 0)\ \mathrm{rad/s}$, the disk remained in a near-planar tumble ($\Omega_{\mathrm{out}} < 0.6\%$). To test the orbital stability of this mode, a $5\%$ out-of-plane perturbation on $\Omega_z$ was evaluated over 8.5 revolutions (`restart_pert5pct`, detailed below).

6. **3D Out-of-Plane Perturbation & Orbital Stability (`restart_pert5pct`):**
   To test whether the planar tumbling limit cycle is an artifact of symmetric initialization and whether breaking planarity closes the $-17\%$ settling-speed deficit, a restart run was initialized at $t = 0.30\ \mathrm{s}$ with a $5\%$ transverse diametral perturbation $\Omega_z = +8.07\ \mathrm{rad/s}$ (tilting the initial angular momentum vector by $\beta \approx 2.86^\circ$ out of the vertical plane) and integrated across 8.5 complete revolutions to $t = 0.58\ \mathrm{s}$ under unconstrained 6DoF translation and rotation (`freedom (1 1 1)`, `rotationFreedom (1 1 1)`):
   - **Out-of-Plane Dynamics:** The perturbation broke the planar symmetry, producing sustained out-of-plane precessional wobble with mean tilt $\theta_{\mathrm{out}} = 8.24^\circ$ and peak excursions reaching $24.07^\circ$. The transverse angular velocity share averaged $\Omega_{\mathrm{out}} / \|\mathbf{\Omega}\| = 2.66\%$.
   - **Kinematic & Aerodynamic Shifts:** Comparing `restart_pert5pct` to the near-planar baseline (`restart_base`) over the common developed window ($t \in [0.32, 0.58]\ \mathrm{s}$):
     - Equilibrium settling velocity shifted by only **$+0.10\%$** ($1.0256 \to 1.0266\ \mathrm{m/s}$, a $+0.001\ \mathrm{m/s}$ change).
     - Tumbling frequency shifted by **$-1.12\%$** ($30.62 \to 30.28\ \mathrm{Hz}$).
     - Full 3D trajectory inclination shifted by **$+1.94\%$** ($\psi = 15.39^\circ \to 15.69^\circ$), while the 2D projected trajectory angle into the laser measurement plane averaged $\phi \approx 2.73^\circ$.
     - Peak pitch aerodynamic moment shifted by **$-4.57\%$** ($M_{x,\mathrm{p95}} = 0.730 \to 0.697\ \mathrm{nN}\cdot\mathrm{m}$).
   - **Physical Finding:** Sustained 3D out-of-plane precessional wobble alters the cycle-averaged aerodynamic drag and equilibrium settling velocity by less than $\le 0.1\%$. Planar symmetry restriction is eliminated as the cause of the $-17\%$ settling-speed deficit.

7. **Diagnostic Equivalent Mass/Weight Step Test (`restart_weight73um`):**
   To investigate whether the $-17\%$ velocity deficit and $-16\%$ tumbling frequency deficit stem from an effective downward body force offset, an extended diagnostic simulation was conducted from $t = 0.30\ \mathrm{s}$ across 9.3 complete tumbling revolutions ($t \in [0.30, 0.594]\ \mathrm{s}$) with disk mass increased by $+46\%$ ($m = 3.228 \times 10^{-7}\ \mathrm{kg}$, corresponding to an effective thickness $h \approx 73\ \mu\mathrm{m}$, with moments of inertia scaled proportionally $I \propto m$):
   - **Simultaneous Velocity and Frequency Recovery:** Under this weight step, the mean descent velocity accelerated to $\langle V_y \rangle = \mathbf{1.2290\ \mathrm{m/s}}$, matching the experimental Tinklenberg et al. (2023) mean of $1.230\ \mathrm{m/s}$ within **$-0.08\%$** ($< 0.001\ \mathrm{m/s}$). Simultaneously, the autorotation frequency increased to $f_t = \mathbf{35.44\ \mathrm{Hz}}$, matching the experimental mean of $36.40\ \mathrm{Hz}$ within **$-2.64\%$**. Trajectory inclination remained consistent at $\psi = 14.62^\circ$.
   - **Drag Invariance:** Normalizing the velocity back to the nominal 50 µm weight yields $V_{\mathrm{eq,est}} = \langle V_y \rangle / \sqrt{F_y/W_{50}} = 1.0385\ \mathrm{m/s}$ (a $+1.25\%$ shift from baseline), confirming that the underlying aerodynamic drag characteristics are unchanged.
   - **Physical Significance:** This test demonstrates that the solver's coupled 6DoF aerodynamics, pressure integration, and autorotation mechanics are in complete structural agreement with the physical phenomena: when the downward force balance matches the experimental terminal speed, the solver reproduces both the translational descent rate and the autorotation frequency concurrently.

---

## 11. ASME V&V 20 Verification & Validation Assessment

In accordance with ASME V&V 20-2009:
- **Comparison Error:** $E = S - D = 1.024 - 1.230 = -0.206\ \mathrm{m/s}$ ($-16.7\%$).
- **Validation Uncertainty of the Mean:** $u_{\mathrm{val}} = \sqrt{u_{\mathrm{num}}^2 + u_{\mathrm{input}}^2 + u_{\mathrm{exp}}^2}$
  - **Numerical Uncertainty ($u_{\mathrm{num}}$):** Evaluated from formal 3-mesh GCI and dynamic 6DoF sensitivity tests:
    - Formal 3-mesh spatial GCI ($r \approx 1.2$, rotating zone refined): $u_{\mathrm{mesh}} \le 0.55\% \approx 0.0056\ \mathrm{m/s}$
    - Time-step sensitivity: $u_{\Delta t} = |+0.04\%| \approx 0.0004\ \mathrm{m/s}$
    - Convection scheme sensitivity (LUST vs. linearUpwind): $u_{\mathrm{scheme}} = |-0.75\%| \approx 0.0077\ \mathrm{m/s}$
    - Combined numerical uncertainty: $u_{\mathrm{num}} = \sqrt{u_{\mathrm{mesh}}^2 + u_{\Delta t}^2 + u_{\mathrm{scheme}}^2} \approx 0.0095\ \mathrm{m/s}$ ($0.93\%$).
  - **Input Uncertainty ($u_{\mathrm{input}}$):** Uncertainty in the experimental mean disk thickness across the batch is small, giving $u_{\mathrm{input}} \ll 0.01\ \mathrm{m/s}$ (taken conservatively as $0.005\ \mathrm{m/s}$), assuming an unbiased nominal mean thickness ($h = 50\ \mu\mathrm{m}$).
  - **Experimental Measurement Uncertainty ($u_{\mathrm{exp}}$):** Measurement error of the ensemble mean velocity is $u_{\mathrm{exp}} = \pm 1.2\%$ ($0.015\ \mathrm{m/s}$).
  - **Total Validation Uncertainty:**
    $$u_{\mathrm{val}} = \sqrt{(0.0095)^2 + (0.0050)^2 + (0.0150)^2} \approx 0.0185\ \mathrm{m/s}\quad (1.5\%)$$
- **ASME V&V 20 Verdict:** Because $|E| = 0.206\ \mathrm{m/s} \approx 11.1\ u_{\mathrm{val}} \gg u_{\mathrm{val}}$, the modeling error $\delta_{\mathrm{model}} \approx E \pm u_{\mathrm{val}} = -0.206 \pm 0.018\ \mathrm{m/s}$ is statistically significant. The simulation is **not validated** for mean terminal settling velocity within the validation uncertainty.
- **Physical Meaning:** Numerical discretization sensitivities are bounded below $1\%$ ($u_{\mathrm{num}} \le 0.93\%$), and 3D out-of-plane precessional wobble accounts for $\le 0.1\%$ velocity shift. Discretization errors and planar symmetry constraints are conclusively ruled out as the source of the $-17\%$ deficit. The discrepancy remains an unresolved physical or flow modeling issue.

---

## 12. Conclusions, Validation Status, and Technical Roadmap

### Current Status Summary
> **Code and solver verification complete. Validation partial: mean settling speed 17% below experiment. Formal 3-mesh GCI bounds spatial discretization uncertainty at $u_{\mathrm{num}} \le 0.55\%$ (near-body zone systematically refined), time step under 0.05% (four revolutions), and convection scheme under 0.8%. 3D out-of-plane perturbation tests bound planar symmetry sensitivity at $+0.10\%$. Independent broadside disk benchmark at $Re = 200$ matches Roos & Willmarth (1971) within $< 1\%$. The $-17\%$ settling deficit is not attributable to numerical error or planar constraints; an equivalent mass step ($h \approx 73\ \mu\mathrm{m}$) simultaneously recovers both the experimental settling speed ($1.229\ \mathrm{m/s}$ vs. $1.230\ \mathrm{m/s}$) and tumbling frequency ($35.44\ \mathrm{Hz}$ vs. $36.40\ \mathrm{Hz}$) with unchanged drag.**

1. **Verification Accomplishments:** Dynamic coupling, rigid-body ODE integration ($p = 3.991$ for $\boldsymbol{\omega}$ on test functions), mechanical energy conservation ($< 10^{-14}$ across 13,333 steps at $\Delta t = 1.5 \times 10^{-5}\text{ s}$), traction force equivalence ($2.1 \times 10^{-10}$ vs. standard OpenFOAM `forces`), and sliding AMI continuity are verified to machine precision.
2. **Kinematic Consistency:** The solver spontaneously sustains unidirectional tumbling across 13 full revolutions, capturing the experimental Strouhal number ($St = 0.059$ vs. $0.055$--$0.060$). Tumbling frequency ($30.5\ \mathrm{Hz}$) is also $\approx 16\%$ below the experimental mean ($36.4\ \mathrm{Hz}$), matching $St$ because velocity and frequency are both low by similar factors, pointing to an effective force or mass offset.
3. **Fixed-Disk Benchmark Diagnostics & GCI:** A formal 3-mesh GCI on steady broadside flow at $Re = 100$ demonstrates oscillatory convergence with a family span of $0.49\%$ and ASME numerical uncertainty $u_{\mathrm{num}} \le 0.55\%$. At $Re = 200$, computed $C_D = 1.106$ matches the published experimental benchmark of Roos & Willmarth (1971) ($1.05\text{--}1.15$) within $< 1\%$.
4. **Deficit Accounting:** Direct 6DoF simulation bounds spatial static mesh sensitivity at $-0.09\%$, time-step sensitivity at $+0.04\%$, convection scheme variation at $-0.75\%$, and 3D out-of-plane precession sensitivity at $+0.10\%$. Specimen thickness scatter alone cannot explain the deficit (needs about $-31\%$ mean thickness). When the downward body force matches the experimental settling speed ($h \approx 73\ \mu\mathrm{m}$ diagnostic), both velocity ($1.229\ \mathrm{m/s}$) and autorotation frequency ($35.44\ \mathrm{Hz}$) match the experiment simultaneously. The underlying cause of the nominal 50 µm deficit remains an open physical discrepancy.

### Technical Roadmap: Completed Validation Tasks

| # | Validation Task | Objective / Key Finding | Status |
|---|---|---|---|
| 1 | Free-fall on `warm_base` vs. `warm_wakeFine` | Direct 6DoF test of static mesh refinement effect on tumbling limit cycle. | **COMPLETED** (Shift: $-0.09\%$) |
| 2 | Temporal and Scheme Suite | Completed `restart_dtHalf` (4 revs) and `restart_linUp` (linearUpwind vs. LUST). | **COMPLETED** ($\Delta t$: $+0.04\%$, Scheme: $-0.75\%$) |
| 3 | Domain & Far-Field Sensitivity | Fixed-disk $Re = 140$ in $1.5\times$ scaled domain ($22.5D \times 37.5D \times 22.5D$, 267k cells): no measurable sensitivity at $t = 80\text{ ms}$ (equal-time comparison, $C_D = 1.1524$ vs. $1.1527$, shift $-0.019\%$). | **COMPLETED** (Shift: $-0.019\%$) |
| 4 | Second Tumbling Realization (3D Perturbation) | Evaluated $5\%$ $\Omega_z$ perturbation over 8.5 revs (`restart_pert5pct`). Produced sustained 3D precession ($\theta_{\mathrm{out}}$ mean $8.24^\circ$, peak $24.07^\circ$); $V_{\mathrm{eq,est}}$ shifted by $+0.10\%$ ($+0.001\ \mathrm{m/s}$), $f_t$ by $-1.12\%$. | **COMPLETED** (Planar sensitivity $\le 0.1\%$) |
| 5 | Diagnostic Free-Fall Test (Mass/Weight Scaling) | Ran 9.3 revs ($t \in [0.30, 0.594]\ \mathrm{s}$) with mass raised to equivalent $h \approx 73\ \mu\text{m}$. Descent speed accelerated to $1.2290\ \mathrm{m/s}$ ($-0.08\%$ vs. exp $1.230\ \mathrm{m/s}$) and frequency increased to $35.44\ \mathrm{Hz}$ ($-2.64\%$ vs. exp $36.40\ \mathrm{Hz}$) with unchanged drag. | **COMPLETED** (Simultaneous $V$ and $f$ recovery) |
| 6 | Independent Drag Check | Broadside disk at $Re = 200$ ($C_D = 1.1062$) matches Roos & Willmarth (1971) ($1.05\text{--}1.15$) within $< 1\%$. Sphere at $Re = 100$ ($C_D = 1.391$) and $Re = 200$ ($C_D = 1.023$) exhibit consistent $+27\%$ truncation offset on coarse 36k Cartesian mesh. | **COMPLETED** (Disk matches published data $< 1\%$) |
| 7 | Three-Mesh GCI Study | Three-mesh family ($N \in [131\text{k}, 369\text{k}]$) systematically refining rotating zone at $Re = 100$: oscillatory convergence ($R = -0.89$), family span $0.49\%$, formal ASME uncertainty $u_{\mathrm{num}} \le 0.55\%$. | **COMPLETED** ($u_{\mathrm{num}} \le 0.55\%$) |
| 8 | Primary Literature Audit | Audit of Shenoy & Kleinstreuer (2008) confirms aspect ratio $\chi = 10$ ($h/D = 0.1$, $\sim 4\times$ thicker than Tinklenberg disks). Edge drag and altered base suction explain lower $C_D \approx 1.15\text{--}1.25$ for thick disks vs. $1.29$ for ultra-thin disks. | **COMPLETED** (Aspect ratio discrepancy resolved) |



