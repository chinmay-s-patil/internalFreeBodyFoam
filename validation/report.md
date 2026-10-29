# 6DoF CFD Validation Study: Thin Disks Falling in Air

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
**Preliminary verification and validation against a published experimental benchmark, with identified aerodynamic drag excess and wake under-resolution.** 

The solver's C++ rigid-body kinematics and energy conservation hold to high precision on production output (verified directly via [`validation/code_verification/check_run_consistency.py`](./code_verification/check_run_consistency.py)):
- $I_{xx} \Delta\omega_x / \Delta t$ matches logged aerodynamic moment $M_x$ to **$2.51 \times 10^{-6}$** relative to $\max(|M_x|)$.
- Vertical frame acceleration $a_y$ matches $-g + F_y/m$ to **$7.20 \times 10^{-9}\text{ m/s}^2$**.
- Velocity updates match integrated acceleration to **$1.22 \times 10^{-9}\text{ m/s}$**, and angle matches $\int \Omega_x \, dt$ to **$0.26^\circ$** over 14 revolutions.
- Mechanical energy change $\Delta E$ matches integrated aerodynamic boundary work $W_k$ to **$3.19 \times 10^{-5}$** relative error.

However, the physical comparisons against Tinklenberg et al. (2023) show a systematic descent velocity deficit:
1. **Asymptotic Descent Deficit ($\approx -17\%$):** While the window-averaged vertical velocity across Revolutions 8–13 is $1.040\text{ m/s}$, the descent velocity is still actively relaxing ($\tau \approx 97\text{ ms}$). An exponential relaxation fit and Aitken $\Delta^2$ extrapolation project the true asymptotic settling speed to **$V_\infty \approx 1.016 - 1.024\text{ m/s}$**. Against experimental mean terminal velocity $V_t = 1.23\text{ m/s}$, the actual asymptotic deficit is **$\approx -17\%$** (rather than $-15.4\%$), corresponding to an equivalent drag coefficient of **$C_{D,\mathrm{paper}} \approx 1.09$** (vs. experimental mean $0.746$).
2. **Deficit Sits in Baseline Aerodynamics, Not 6DoF Kinematics:** The zero-spin drop run ended near $V_y \approx 1.026\text{ m/s}$ and the tumbling run asymptotically settles near $V_y \approx 1.020\text{ m/s}$. Because tumbling and broadside orientations produce nearly identical fall speed deficits, the discrepancy is rooted in baseline disk aerodynamics and wake numerics rather than the coupled 6DoF rotational dynamics.
3. **Primary Numerical Lead — Wake Under-Resolution & Interface Smearing:**
   - The outer static mesh block has a coarse base cell of $1.25\text{ mm} \approx 0.62 D$.
   - The spherical sliding interface exhibits a severe resolution mismatch: **$15,528$ faces on the rotating side ($\approx 57\ \mu\text{m}$)** versus only **$456$ faces on the static side ($\approx 330\ \mu\text{m}$)** — a 34:1 area mismatch.
   - Refinement extends only to $0.31\text{ mm}$ out to $2\text{ mm}$ and $0.63\text{ mm}$ out to $5\text{ mm}$.
   - Coupled with the blended LUST convection scheme, backward time-stepping, and $\mathrm{Co}_{\mathrm{max}} \approx 1.5 - 1.62$ with $nOuterCorrectors = 1$, the unsteady wake ($Re_D \approx 140$) is numerically diffused immediately upon crossing the interface, suppressing natural wake shedding and creating artificial excess drag.
4. **Thickness Spread Limitation:** Disk thickness tolerance in Table 1 ($50 \pm 7.5\ \mu\text{m}$) represents the population standard deviation across the manufactured disk batch, measured with caliper stacks. The experimental mean $V_t = 1.23\text{ m/s}$ represents an ensemble mean of multiple drops across mean-thickness disks; therefore, the batch standard deviation cannot be claimed as uncertainty of the mean or used to explain half the gap.

---

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
- **Total Mesh Count:** 212,192 cells (hex-dominant).
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
├── thinDisk2mm/                            # PRESERVED: Transient drop run (6.4 GB data intact)
│   ├── case/                               # Mesh, processor0-7, postProcessing, logs
│   ├── render/                             # 60 FPS video (disk_drift_animation.mp4) & preview PNG
│   ├── validation_comparison.png           # 4-panel experimental validation curves
│   └── plot_validation.py                  # Telemetry parsing & validation script
└── thinDisk2mm_tumbling/                   # COMPLETED: 13-revolution seeded tumbling run (t = 0.500 s)
    ├── case/                               # Mesh, constant/fallingFrameDict, system, logs
    │   └── postProcessing/                 # tumbleMotion and fallingMotion 6DoF logs (29,715 steps)
    ├── plot_tumbling_detailed.py           # 6-panel high-res validation plot script
    └── tumbling_validation_detailed.png    # 6-panel experimental validation figure
```

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

### 9.2. Spatial & Temporal Discretization Studies: Startup Diagnostics

> [!WARNING]
> The spatial and temporal GCI studies performed at $t = 1 - 2\text{ ms}$ are **early startup diagnostics only** and do not establish discretization uncertainty for the developed unsteady limit cycle:
> 1. At $t = 2\text{ ms}$, hydrostatic buoyancy ($\rho_f V_{\mathrm{disk}} g = 1.886\text{ nN}$) accounts for **$76\%$** of the total vertical force ($F_y \approx 2.49\text{ nN}$), while aerodynamic shear is negligible.
> 2. Descent velocity $V_y \approx 19.6\text{ mm/s}$ is dominated by ballistic ODE integration ($g \cdot t = 19.62\text{ mm/s}$) and is insensitive to the fluid mesh by construction.
> 3. Pitching moment ($M_x \sim 10^{-18}\text{ N}\cdot\text{m}$) is 8 orders of magnitude below developed torque ($0.61\text{ nN}\cdot\text{m}$) and represents floating-point roundoff around symmetric release.
> 4. In the spatial grid study, the static outer mesh cell count barely changed between Coarse and Medium ($34,584 \to 34,632$), meaning the outer wake was never systematically refined.

---

### 9.3. Identification of Drag Excess: Wake Under-Resolution & Numerical Smearing

The primary physical lead explaining the $\approx -17\%$ settling speed deficit sits in the static wake mesh:
1. **Outer Static Block:** Base cell is $1.25\text{ mm} \approx 0.62 D$.
2. **Interface Face Area Mismatch:** Rotating AMI sphere has $15,528$ faces ($\Delta x \approx 57\ \mu\text{m}$), whereas the static side has only $456$ faces ($\Delta x \approx 330\ \mu\text{m}$) — a 34:1 face ratio.
3. **Wake Transition:** Wake refinement drops from $0.31\text{ mm}$ within $2\text{ mm}$ to $0.63\text{ mm}$ at $5\text{ mm}$, and $1.25\text{ mm}$ beyond.
4. **Numerical Diffusion:** Convection scheme LUST, backward time integration, and $\mathrm{Co}_{\mathrm{max}} \approx 1.5$ with `nOuterCorrectors 1` excessively diffuse the $Re_D \approx 140$ unsteady wake as it crosses the interface. At $Re_D \approx 116.5$, steady disk flow loses axisymmetry (Natarajan & Acrivos 1993); excessive numerical diffusion suppresses non-axisymmetric wake shedding, increases base suction, and causes artificial drag excess on both tumbling and broadside orientations.

---

## 10. Next Steps & Verification Action Plan

1. **Developed-State Restart Sweeps (from $t = 0.3\text{ s}$ over 3–4 revolutions):**
   - **Sweep (a):** Refine static mesh $2\times$ in the wake region to eliminate the AMI face mismatch.
   - **Sweep (b):** Limit time step to `maxCo 0.75` with `nOuterCorrectors 2`.
   - **Sweep (c):** Replace `LUST` with `linearUpwind` convection.
   - Evaluate whether asymptotic $V_\infty$ shifts from $1.02\text{ m/s}$ toward $1.23\text{ m/s}$.
2. **Fixed Broadside Disk Benchmark:**
   - Fix 6DoF translation and rotation (`freedom` and `rotationFreedom` to zero).
   - Simulate fixed disk at $Re = 100$ (steady axisymmetric wake) and $Re = 140$ (unsteady non-axisymmetric wake, Natarajan & Acrivos 1993) to isolate baseline drag accuracy independently of 6DoF motion.
3. **Independent Force and Flux Cross-Check:**
   Add OpenFOAM functionObjects in `controlDict` to verify independent surface force integration and interface flux conservation:
   ```cpp
   functions
   {
       diskForces
       {
           type            forces;
           libs            (forces);
           patches         (disk);
           rho             rhoInf;
           rhoInf          1.2;
           CofR            (0 0 0);
           pName           p;
           UName           U;
           writeControl    timeStep;
           writeInterval   1;
       }

       amiFluxMaster
       {
           type            surfaceFieldValue;
           libs            (fieldFunctionObjects);
           regionType      patch;
           name            masterAMI;
           operation       sum;
           fields          (phi);
           writeFields     no;
       }
   }
   ```
4. **Standard 3D Sphere Benchmarks:**
   - Sphere settling in accelerating frame against Schiller–Naumann:
     $$C_D = \frac{24}{Re} (1 + 0.15 Re^{0.687})$$
   - Sphere rotational spin-down in Stokes limit:
     $$\tau = \frac{\rho_s R^2}{15 \mu}$$
