# 6DoF CFD Validation Study: Thin Disks Falling in Air

**Benchmark Reference:**  
Tinklenberg, A., Guala, M., & Coletti, F. (2023). *Thin disks falling in air*. **Journal of Fluid Mechanics**, vol. 962, A3. [doi:10.1017/jfm.2023.209](https://doi.org/10.1017/jfm.2023.209)

**CFD Solver:** `freeBody6DoFPimpleFoam` (Coupled 6DoF Accelerating Reference Frame + Spherical Sliding AMI)  
**Rotational Motion Function:** `freeBodyRotationalMotion`  
**Case Directory:** [`validation/thinDisk2mm/`](./thinDisk2mm/) (Preserved transient run) & [`validation/thinDisk2mm_tumbling/`](./thinDisk2mm_tumbling/) (Seeded tumbling run)

---

## 1. Executive Summary & Validation Status

A 6DoF computational fluid dynamics simulation study was performed for a thin polyethylene terephthalate (PET) disk falling in air, based on the $2\text{ mm}$ experimental benchmark from Tinklenberg, Guala, and Coletti (2023, *J. Fluid Mech.*, vol. 962, A3).

### Current Validation Status
**Preliminary validation against a published experimental benchmark, with partial numerical verification.** The solver reproduces continuous autorotation for the seeded initial condition and obtains a settling-speed response within the experimental $1\sigma$ range. However, the rotational response is still evolving, and the current GCI calculations and production-code verification tests do not yet establish numerical uncertainty for the developed tumbling solution. Further systematic mesh and time-step studies, longer-cycle statistics, and direct verification of the production implementation are required before claiming comprehensive validation.

The dynamic state of the simulation is characterized as **a quasi-settled translational response with an evolving rotational response**.

#### Key Physical & Kinematic Findings:
- **Translational Response (Quasi-Settled):** The cycle-averaged vertical velocity varies between $1.086$ and $1.127\text{ m/s}$ over Revolutions 4–7, with a maximum adjacent-cycle change of approximately $3.8\%$ (between Rev 4 at $1.086\text{ m/s}$ and Rev 5 at $1.127\text{ m/s}$). The response is quasi-settled translationally, while the rotational response continues to evolve. The settling speed lies inside the experimental mean and one-standard-deviation spread of $V_t = 1.23 \pm 0.31\text{ m/s}$ ($[0.92\text{ m/s}, 1.54\text{ m/s}]$).
- **Descent Reynolds Number:** Simulated $Re = 146 - 152$, well within the experimental range of **$Re = 165 \pm 42$** ($[123, 207]$).
- **Physical Force Balance & Drag Definitions:** In the zero-spin run at its steady plateau, the aerodynamic force balances gravity within $0.1\%$ ($F_\text{aero} / W = 1.001$). In the seeded tumbling run, the terminal-velocity-equivalent drag coefficient evaluates to **$C_{D,\mathrm{paper}} = 0.89 - 0.96$**, safely inside the transformed experimental velocity uncertainty envelope of **$[0.47, 1.33]$**. This is distinct from the surface resultant force coefficient $C_{F,\mathrm{mag}}(t)$, whose cycle-average is $0.72 - 0.75$.
- **Continuous Tumbling Autorotation:** When seeded with initial rotation, the disk establishes continuous unidirectional autorotation (**$7$ full revolutions completed**, $t > 0.274\text{ s}$) with zero rotational reversals.
- **Rotational Response (Evolving Limit Cycle):** The autorotation frequency increases dynamically as fall velocity develops: rising from $19.75\text{ Hz}$ in Rev 4 ($45.7\%$ below $36.4\text{ Hz}$) to $23.84\text{ Hz}$ in Rev 5 ($34.5\%$ below), $27.37\text{ Hz}$ in Rev 6 ($24.8\%$ below), and **$29.56\text{ Hz}$ in Rev 7** (**$18.8\%$ below**). Cycle-averaged angular velocity increases from $-124.1\text{ rad/s}$ (Rev 4) to $-149.8\text{ rad/s}$ (Rev 5), $-172.0\text{ rad/s}$ (Rev 6), and $-185.8\text{ rad/s}$ (Rev 7, $18.8\%$ below the experimental trajectory-averaged mean of $228.7\text{ rad/s}$). Peak downstroke pitch rate reaches $-223.7\text{ rad/s}$ ($97.8\%$ of experimental mean).
- **Strict 2D Planar Stability:** Out-of-plane angular velocities ($\Omega_y, \Omega_z$) remain $< 0.30\text{ rad/s}$ ($< 0.2\%$ of pitch rate $\Omega_x$), preserving 2D planar symmetry without unphysical roll or yaw divergence.
- **Lateral Flight Angle:** Magnus and wake lift forces continuously deflect the disk laterally, with the instantaneous trajectory inclination angle reaching $\psi \approx 8.97^\circ$ (approaching the experimental mean of $|\psi| \approx 10.5^\circ$).

### Methodological & Numerical Verification Summary:
1. **Scope of Spatial GCI (ASME V&V 20):** Evaluated at early startup ($t = 0.002\text{ s}$, $V_y \approx 0.02\text{ m/s}$). Drag force and descent velocity exhibit oscillatory convergence ($R = -0.589$ and $R = -0.741$) with observed half-range indicators ($\Delta_\mathrm{half} = 0.110\%$ and $< 0.001\%$, reflecting observed spread across grids rather than formal error bounds). Pitching moment exhibits monotonic convergence with an apparent order $p = 13.80$, far exceeding formal scheme accuracy, leaving formal GCI undetermined. Crucially, the outer static mesh was not uniformly refined between coarse and medium grids, and these early startup tests **do not establish spatial uncertainty for the developed tumbling limit cycle**.
2. **Scope of Temporal GCI:** Evaluated at $t = 0.001000\text{ s}$ via exact boundary interpolation. Vertical aerodynamic force exhibits monotonic convergence with $p = 1.566$, $\mathrm{GCI}_t = 0.045\%$, and asymptotic ratio $A_R = 1.001$, while pitching moment exhibits early oscillatory divergence ($R = -33.2$).
3. **Production Implementation Diagnostics:** Algorithmic formulation is verified independently in Python ($p = 3.9910 \approx 4.0000$, invariants conserved to $< 10^{-15}$, frame Galilean invariance confirmed), but does not directly verify production C++ quaternion updates. Production C++ runtime diagnostics over 14,000+ time steps confirm cyclicAMI geometric interface overlap (minimum weight $0.9568 > 0$, 0 unmapped faces) and mass continuity to $\sim 1.0 \times 10^{-10}$; direct surface flux balance and coupled FSI Method of Manufactured Solutions (MMS) remain open verification objectives.
4. **Regime Identification:** Clarified that the zero-spin simulation is a *predicted fluttering solution under idealized quiescent release* rather than an experimental validation of flutter, since all 2 mm disks tumbled in the experimental drop facility.

---

## 2. Benchmark Physical Parameters

The physical parameters correspond to the $2\text{ mm}$ PET disk characterized in Table 1 & Table 3 of Tinklenberg et al. (2023):

| Parameter | Symbol | Value (SI) | Value (Practical) |
| :--- | :--- | :--- | :--- |
| **Disk Diameter** | $D$ | $2.02 \times 10^{-3}\text{ m}$ | $2.02\text{ mm}$ |
| **Disk Thickness** | $h$ | $5.00 \times 10^{-5}\text{ m}$ | $50.0\ \mu\text{m}$ |
| **Aspect Ratio** | $\chi = D/h$ | $40.4$ | $40.4$ |
| **Disk Material Density** | $\rho_d$ | $1380\text{ kg/m}^3$ | Polyethylene terephthalate (PET) |
| **Total Disk Mass** | $m = \rho_d \frac{\pi}{4} D^2 h$ | $2.2113 \times 10^{-7}\text{ kg}$ | $0.2211\text{ mg}$ |
| **Gravitational Weight** | $W = m g$ | $2.1693 \times 10^{-6}\text{ N}$ | $2.169\ \mu\text{N}$ |
| **Diametral Inertia (Pitch/Roll)** | $I_{xx} = I_{zz}$ | $5.6436 \times 10^{-14}\text{ kg}\cdot\text{m}^2$ | $5.64 \times 10^{-14}\text{ kg}\cdot\text{m}^2$ |
| **Polar Inertia (Spin)** | $I_{yy}$ | $1.1278 \times 10^{-13}\text{ kg}\cdot\text{m}^2$ | $1.13 \times 10^{-13}\text{ kg}\cdot\text{m}^2$ |
| **Fluid Medium** | Air at 20°C | $\rho_f = 1.200\text{ kg/m}^3$ | $\nu = 1.50 \times 10^{-5}\text{ m}^2/\text{s}$ |
| **Density Ratio** | $\tilde{\rho} = \rho_d / \rho_f$ | $1150$ | $1150$ |
| **Inertia Ratio** | $I^* = \frac{\pi}{64} \frac{\tilde{\rho}}{\chi}$ | $1.40$ | $1.40$ |
| **Galileo Number** | $Ga = \frac{\sqrt{2|\tilde{\rho}-1|gh} D}{\nu}$ | $142.2$ | $142.2$ |

---

## 3. Mathematical Model & Numerical Architecture

### 3.1. Accelerating Reference Frame (`freeBody6DoFPimpleFoam`)
Rather than translating a metric mesh over meters of descent (which causes mesh stretching and numerical diffusion), the solver solves the incompressible Navier-Stokes equations in a reference frame translating with the body:

$$\nabla \cdot \mathbf{U} = 0$$

$$\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) = -\frac{1}{\rho} \nabla p_\text{rgh} + \nu \nabla^2 \mathbf{U} - \mathbf{a}_\text{frame}$$

where $\mathbf{a}_\text{frame} = \mathbf{g} + \frac{\mathbf{F}_\text{aero}}{m}$ is the instantaneous frame acceleration, and the boundary condition at the outer far-field boundary dynamically imposes $\mathbf{U}_\text{inlet} = -\mathbf{V}_\text{fall}(t)$.

### 3.2. Spherical Sliding Interface (`freeBodyRotationalMotion`)
To handle unconstrained, multi-axis 3D tumbling without mesh deforming or overset interpolation errors:
- An **inner spherical rotating zone** ($D_\text{ami} = 4.0\text{ mm} \approx 2 D$) tightly encapsulates the disk.
- Conformal Arbitrary Mesh Interface (`masterAMI` / `slaveAMI`) couples the rotating sphere to the outer translating domain.
- Quaternions $\mathbf{q}(t) = [w, \mathbf{v}]$ are integrated via Runge-Kutta 4th order (RK4) from the aerodynamic moments $\mathbf{M}_\text{aero}(t) = \int (\mathbf{r} - \mathbf{r}_\text{hub}) \times (\mathbf{\tau}_w - p \mathbf{n}) \, dA$.

### 3.3. Grid & Boundary Layer Resolution
- **Total Mesh Count:** 212,192 cells (hex-dominant).
- **Prism Layers on Foil:** 5–8 layers on top/bottom faces and 10–12 cells across the $50\ \mu\text{m}$ edge rim.
- **Mesh Quality:** Maximum non-orthogonality $37.3^\circ$ (average $7.17^\circ$), 0 skewness errors, 0 aspect ratio errors.
- **Time Step:** Adaptive PIMPLE stepping with $\Delta t \approx 1.5 \times 10^{-5}\text{ s}$ ($\text{Co} < 1.0$).
- **Parallel Computing:** Decomposed across 8 CPU cores via Scotch decomposition.

---

## 4. Quantitative Comparison: Initial Zero-Spin Drop (`thinDisk2mm`)

Below is the comparison of simulated metrics from the initial transient zero-spin drop against experimental data reported in Tinklenberg et al. (2023):

| Metric | Tinklenberg et al. (2023) | freeBody6DoFPimpleFoam (Zero-Spin) | Note / Interpretation |
| :--- | :--- | :--- | :--- |
| **Terminal Descent Speed $V_y$** | $1.23 \pm 0.31\text{ m/s}$ (Table 3) | $1.03 - 1.05\text{ m/s}$ | Within experimental $1\sigma$ band ($-14.6\%$) |
| **Aerodynamic Drag Force $F_\text{aero}$** | $2.17\ \mu\text{N}$ (at terminal state) | $2.18\ \mu\text{N}$ | Exact gravitational force balance within $0.1\%$ |
| **Force Balance Ratio $F_\text{aero} / W$** | $1.000$ | $1.001$ | $0.1\%$ numerical force balance error |
| **Descent Reynolds Number $Re$** | $165 \pm 42$ (Table 3) | $138 - 141$ | Within experimental $1\sigma$ band |
| **Drag Coefficient $C_{D,\text{paper}}$** | $0.74$ (Table 3; $[0.47, 1.33]$ transformed from $V_t \pm 1\sigma$) | $1.07 - 1.13$ (Broadside flutter) | Matches broadside/flutter literature ($C_D \approx 1.10 - 1.15$) |
| **Planar Pitch Rate $|\Omega_x|$** | $228.7 \pm 45\text{ rad/s}$ (Figs. 18–20) | $88.7\text{ rad/s}$ peak / rocking | Disks rock ($\pm 50^\circ$) under zero-spin release |
| **Out-of-Plane Rates ($\Omega_y, \Omega_z$)** | $\approx 0$ (2D planar) | $< 0.016\text{ rad/s}$ | $< 0.02\%$ of pitch rate (strict planar stability) |
| **Lateral Drift Distance $D_z$** | Unidirectional glide | $2.14\text{ mm}$ (oscillating) | Planar zigzagging excursion |

---

## 5. Physical Regimes: Flutter vs. Asymptotic Tumbling

### 5.1. Predicted Flutter Under Quiescent Zero-Spin Release
In Tinklenberg et al. (2023), virtually all 2 mm PET disks fell into the tumbling regime, with no statistically significant population of non-tumbling trajectories observed. The zero-spin simulation here (`thinDisk2mm`) was initialized with $\Omega_0 = 0$ and $\theta_0 = 15^\circ$ in quiescent air. Under these idealized conditions:
1. The disk accelerates vertically from rest, pitching forward due to front-rear pressure asymmetry.
2. It reaches $\Omega_x \approx -82\text{ rad/s}$ during the initial pitch.
3. However, because $-82\text{ rad/s}$ is below the critical autorotation threshold ($\approx 200\text{ rad/s}$), when rotating through broadside, the aerodynamic restoring torque ($M_\text{aero} \approx 0.45\text{ nN}\cdot\text{m}$) halts rotation and reverses it.
4. The disk settles into a periodic **aerodynamic flutter limit cycle** (rocking between $-50^\circ$ and $+44^\circ$ at $f \approx 18.2\text{ Hz}$).

> [!NOTE]
> This flutter mode is a **predicted numerical solution for idealized quiescent release**, not an experimental validation of flutter, since the paper observed only tumbling for 2 mm disks. Demonstrating true physical bistability requires a comprehensive initial-condition sweep ($\theta_0, \Omega_0$) to map out the basin boundaries.

### 5.2. Experimental Facility vs. Simulation Release Conditions
- In the laboratory experiment, disks fell through a **3-metre vertical drop chute connected to a larger chamber**, yielding over 4 metres of total fall distance before the optical measurement zone. This long descent ensured that release transients were forgotten and disks settled into their asymptotic tumbling attractor.
- In contrast, the zero-spin CFD run reached $t \approx 0.235\text{ s}$ ($18.8\text{ cm}$ of fall). Because the release transient in quiescent air trapped the disk in the fluttering basin, verifying the experimental tumbling regime required seeding the autorotation limit cycle directly (Section 8).

---

## 6. Visualizer & Telemetry Pipeline

### 6.1. 3D Motion & Drift Visualizer (`animate_drift.py`)
A custom high-performance visualizer was built using PyVista and Pillow:
- **Frame Rate:** Fluid 60 FPS video at 20× slow-motion replay.
- **Flight-Corridor Perspective:** Observer camera fixed at distance $5.5\text{ mm}$ looking obliquely across the flight corridor ($Z_\text{mid} = -0.7\text{ mm}$), scaling down the disk appearance to $0.6\times$ for realistic proportion.
- **Prominent Lateral Drift:** The disk physically traverses the 3D scene across the screen over a $2.14\text{ mm}$ lateral span ($Z = +0.46\text{ mm}$ to $Z = -1.68\text{ mm}$).
- **Vertical Trajectory Curtain:** A translucent glowing cyan ribbon connects the 3D flight trajectory directly down to the floor ground track.
- **Plumb-Bob Ground Track & Floor Grid:** Vertical green laser line drops to a tracking blip moving over calibrated $0.5\text{ mm}$ interval distance ticks and range rings.
- **Dual-Tone Foil:** Vibrant cyan top face (`#00E5FF`), vibrant orange bottom face (`#FF6D00`), and crisp white edge rim (`#FFFFFF`).
- **Corner Radar HUD:** Real-time top-down 6DoF horizontal drift radar tracking $(D_x, D_z)$, bearing, and lateral force vectors.
- **Avionics Banner:** Digital readouts of pitch, pitch rate, descent velocity, fallen distance, drag force, and $Z$-drift.

### Deliverables:
- **Video:** [`validation/thinDisk2mm/render/disk_drift_animation.mp4`](./thinDisk2mm/render/disk_drift_animation.mp4)
- **Preview Snapshot:** [`validation/thinDisk2mm/render/disk_drift_animation_preview.png`](./thinDisk2mm/render/disk_drift_animation_preview.png)
- **Validation Curves:** [`validation/thinDisk2mm/validation_comparison.png`](./thinDisk2mm/validation_comparison.png)

---

## 7. Directory Structure & Continued Studies

Per project instructions, the original transient simulation data is preserved intact, and a cloned case is prepared for asymptotic tumbling validation:

```
validation/
├── report.md                               # This comprehensive findings report (ignored in .gitignore)
├── papers/
│   └── thin-disks-falling-in-air.pdf       # Tinklenberg et al. (2023) experimental paper
├── thinDiskValidationReport/               # Full engineering report & high-res paper figures
├── thinDisk2mm/                            # PRESERVED: Transient drop run (6.4 GB data intact)
│   ├── case/                               # Mesh, processor0-7, postProcessing, logs
│   ├── render/                             # 60 FPS video (disk_drift_animation.mp4) & preview PNG
│   ├── validation_comparison.png           # 4-panel experimental validation curves
│   ├── animate_drift.py                    # 3D visualizer script
│   └── plot_validation.py                  # Telemetry parsing & validation script
└── thinDisk2mm_tumbling/                   # NEW: Cloned case configured for steady-state tumbling
    ├── case/
    │   ├── 0/                              # Clean initial boundary conditions
    │   ├── constant/
    │   │   ├── polyMesh/                   # Ready-to-run 212k-cell spherical sliding AMI mesh
    │   │   └── fallingFrameDict            # initialOmega set to (-228.7 0 0) rad/s (Table 3 benchmark)
    │   ├── system/                         # decomposeParDict (8 cores), controlDict, fvSchemes
    │   └── Allrun                          # Parallel execution script
    ├── animate_drift.py                    # Visualizer script
    ├── plot_validation.py                  # Validation plot script
    ├── plot_tumbling_detailed.py           # 6-panel high-res validation plot script
    └── tumbling_validation_detailed.png    # 6-panel experimental validation figure
```

---

## 8. Steady-State Tumbling Analysis (`validation/thinDisk2mm_tumbling/`)

The cloned case was initialized with $\Omega_0 = (-228.7, 0, 0)\text{ rad/s}$ (the experimental mean tumbling rate from Figures 18–20) and executed in parallel across 8 CPU cores. The simulation has progressed past $13,600+$ time steps ($t > 0.274\text{ s}$, descent distance $> 24\text{ cm}$), completing **$7.0$ full continuous autorotation revolutions** without rotational reversal.

### 8.1. Reconciliation of Aerodynamic Force & Drag Definitions

A critical distinction must be made between two distinct aerodynamic coefficients:

1. **Terminal Settling-Velocity Derived $C_D$ (Paper Definition, Table 3):**
   In Tinklenberg et al. (2023), Table 3 reports a single drag coefficient $C_D = 0.74$ derived directly from the mean terminal descent velocity $V_t = 1.23\text{ m/s}$ assuming gravitational force equilibrium:
   $$C_{D,\mathrm{paper}} = \frac{2 W}{\rho_f A V_t^2} = \frac{2 m g}{\rho_f A V_t^2}$$
   Using the disk properties ($m = 2.2113 \times 10^{-7}\text{ kg}, g = 9.81\text{ m/s}^2, W = 2.1693\ \mu\text{N}, \rho_f = 1.200\text{ kg/m}^3, D = 2.02\text{ mm}, A = 3.2047 \times 10^{-6}\text{ m}^2$):
   - **Experimental Reference:** For mean $V_t = 1.23\text{ m/s}$, this gives $C_{D,\mathrm{paper}} = 0.746 \approx 0.75$. Transforming the experimental velocity endpoints ($V_t \pm 1\sigma = 1.23 \pm 0.31\text{ m/s} \implies [0.92, 1.54]\text{ m/s}$) produces an equivalent velocity-transformed interval of **$[0.47, 1.33]$**. (Note: the paper reports uncertainty on velocity, not as an independently measured $C_D$ tolerance).
   - **Simulation Settling Range:** In the simulation, across Revolutions 4–7 the cycle-averaged vertical speed settles into $V_y = 1.08 - 1.13\text{ m/s}$. Evaluated with the paper's formula:
     $$C_{D,\mathrm{paper}}(V_y = 1.086) = 0.957, \quad C_{D,\mathrm{paper}}(V_y = 1.127) = 0.888$$
   Because the late-run descent speed ($1.08 - 1.13\text{ m/s}$) is ~8–12% lower than the experimental mean ($1.23\text{ m/s}$), the equivalent terminal-force drag coefficient is **$0.89 - 0.96$**. This lies comfortably within the transformed experimental velocity envelope ($[0.47, 1.33]$), but must not be conflated with an exact match to the $0.746$ mean.

2. **Resultant Aerodynamic-Force Coefficient $C_{F,\mathrm{mag}}(t)$:**
   $$C_{F,\mathrm{mag}}(t) = \frac{|\mathbf{F}_\mathrm{aero}(t)|}{\frac{1}{2} \rho_f A |\mathbf{V}(t)|^2}$$
   Because this definition uses the total magnitude of the surface-integrated aerodynamic force ($|\mathbf{F}_\mathrm{aero}| = \sqrt{F_x^2 + F_y^2 + F_z^2}$), it includes both drag and lateral lift components. During continuous autorotation, $C_{F,\mathrm{mag}}(t)$ sweeps periodically between **$0.20$** (edge-on downstroke) and **$1.60$** (broadside crossing), yielding a time-average of **$0.72 - 0.75$**. This diagnostic reflects the periodic aerodynamic surface loading, but is conceptually distinct from the settling-based drag coefficient.

---

### 8.2. Cycle-by-Cycle Kinematic Evolution (Time-Weighted Trapezoidal Averages)

Tracking the simulation cycle-by-cycle (where each revolution represents an exact $360^\circ$ rotation) using continuous time-weighted trapezoidal integration reveals the coupled 6DoF dynamics:

| Rev | Physical Time Range | Period $T_\mathrm{rev}$ | Tumbling Freq $f_t$ | Mean $\Omega_x$ | $\Omega_x$ Range | Mean $V_y$ | $C_{D,\mathrm{paper}}$ | Status |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | $0.0 \to 30.4\text{ ms}$ | $30.37\text{ ms}$ | $32.93\text{ Hz}$ | $-206.9\text{ rad/s}$ | $[-227.9, -190.2]$ | $0.149\text{ m/s}$ | $51.09$ | Startup |
| **2** | $30.4 \to 66.6\text{ ms}$ | $36.26\text{ ms}$ | $27.58\text{ Hz}$ | $-173.3\text{ rad/s}$ | $[-190.1, -158.3]$ | $0.472\text{ m/s}$ | $5.06$ | Acceleration |
| **3** | $66.6 \to 111.3\text{ ms}$ | $44.69\text{ ms}$ | $22.38\text{ Hz}$ | $-140.6\text{ rad/s}$ | $[-158.3, -119.3]$ | $0.836\text{ m/s}$ | $1.61$ | Free fall build-up |
| **4** | $111.3 \to 161.9\text{ ms}$ | $50.63\text{ ms}$ | $19.75\text{ Hz}$ | $-124.1\text{ rad/s}$ | $[-184.8, -83.0]$ | $1.086\text{ m/s}$ | **$0.957$** | Quasi-settled speed |
| **5** | $161.9 \to 203.9\text{ ms}$ | $41.95\text{ ms}$ | $23.84\text{ Hz}$ | $-149.8\text{ rad/s}$ | $[-204.9, -100.6]$ | $1.127\text{ m/s}$ | **$0.888$** | Quasi-settled speed |
| **6** | $203.9 \to 240.4\text{ ms}$ | $36.54\text{ ms}$ | $27.37\text{ Hz}$ | $-172.0\text{ rad/s}$ | $[-216.8, -131.7]$ | $1.111\text{ m/s}$ | **$0.914$** | Quasi-settled speed |
| **7** | $240.4 \to 274.3\text{ ms}$ | $33.82\text{ ms}$ | $29.56\text{ Hz}$ | $-185.8\text{ rad/s}$ | $[-223.7, -150.0]$ | $1.087\text{ m/s}$ | **$0.955$** | Completed Rev 7 |

#### Quantitative Comparison & Dynamic State Characterization:
The simulated motion is rigorously characterized as **a quasi-settled translational response with an evolving rotational response**:
- **Translational Settling Response:** The cycle-averaged vertical velocity varies between $1.086$ and $1.127\text{ m/s}$ over Revolutions 4–7, with a maximum adjacent-cycle change of approximately $3.8\%$ (between Rev 4 at $1.086\text{ m/s}$ and Rev 5 at $1.127\text{ m/s}$). The response is quasi-settled translationally, while the rotational response continues to evolve. The settling speed lies comfortably inside the experimental mean and one-standard-deviation spread of $V_t = 1.23 \pm 0.31\text{ m/s}$ ($[0.92, 1.54]\text{ m/s}$).
- **Rotational Evolution (Like-for-Like Comparison):** 
  In Tinklenberg et al. (Figures 18–20), the experimental benchmark reports a characteristic tumbling frequency of $36.4\text{ Hz}$ and trajectory-averaged angular rates clustering at $\langle |\omega_t| \rangle = 228.7 \pm 45\text{ rad/s}$.
  Comparing cycle-averaged quantities like-for-like against these experimental benchmarks shows systematic acceleration:
  - **Revolution 4:** $f_t = 19.75\text{ Hz}$ (**$45.7\%$ below** $36.4\text{ Hz}$); Mean $\Omega_x = -124.1\text{ rad/s}$ (**$45.7\%$ below** experimental mean).
  - **Revolution 5:** $f_t = 23.84\text{ Hz}$ (**$34.5\%$ below** $36.4\text{ Hz}$); Mean $\Omega_x = -149.8\text{ rad/s}$ (**$34.5\%$ below** experimental mean).
  - **Revolution 6:** $f_t = 27.37\text{ Hz}$ (**$24.8\%$ below** $36.4\text{ Hz}$); Mean $\Omega_x = -172.0\text{ rad/s}$ (**$24.8\%$ below** experimental mean).
  - **Revolution 7:** $f_t = 29.56\text{ Hz}$ (**$18.8\%$ below** $36.4\text{ Hz}$); Mean $\Omega_x = -185.8\text{ rad/s}$ (**$18.8\%$ below** experimental mean $228.7\text{ rad/s}$).
- **Peak vs. Mean Angular Rates:** In Revolution 7, the peak downstroke pitch rate reached $-223.7\text{ rad/s}$, which is within $2.2\%$ of the experimental mean $|\omega_t| = 228.7\text{ rad/s}$. However, this peak is an instantaneous extremum occurring during the edge-on acceleration phase; the rigorous, defensible comparison is between trajectory/cycle-averaged means ($-185.8\text{ rad/s}$ in Rev 7).
- **Physical Reason for Evolving Rotation:** While the translational velocity approaches a settling balance early ($t \approx 0.15\text{ s}$), the autorotation limit cycle requires multiple cycles of continuous aerodynamic torque integration to pump energy into the rotational degree of freedom. As the disk falls through the fluid, the non-linear vortex-shedding torque continuously accelerates the spin rate toward the asymptotic limit-cycle attractor.

---

### 8.3. Trajectory Geometry: Cumulative Displacement vs. Instantaneous Velocity Angle

A clear geometric distinction must be made regarding lateral drift:
- **Cumulative Trajectory Displacement:** Starting from rest at $(0, 0, 0)$, by $t = 0.276\text{ s}$ the disk has descended $\Delta Y = 240.6\text{ mm}$ while drifting $\Delta Z = 25.59\text{ mm}$ laterally. The cumulative displacement angle is:
  $$\psi_\mathrm{disp} = \arctan\left(\frac{\Delta Z}{\Delta Y}\right) = \arctan\left(\frac{25.59}{240.6}\right) = 6.07^\circ$$
- **Instantaneous Flight Velocity Angle:** As the disk tumbles continuously in $-X$, Magnus and unsteady wake lift forces accelerate lateral velocity $V_z$ up to $0.266\text{ m/s}$ at $t = 0.276\text{ s}$. The instantaneous flight trajectory angle is:
  $$\psi(t) = \arctan\left(\frac{V_\mathrm{horiz}}{V_y}\right) = \arctan\left(\frac{0.266}{1.052}\right) = 14.19^\circ$$
  This instantaneous flight angle matches the experimental trajectory inclination distribution in Tinklenberg et al. (Figure 10, which spans $5^\circ - 20^\circ$ with mean $|\psi| \approx 10.5^\circ$). The cumulative displacement angle $\psi_\mathrm{disp}$ is smaller simply because the disk started from $V_z = 0$ and required multiple revolutions to build horizontal momentum.

---

### 8.4. Deliverables for Tumbling Case
- **6-Panel Publication Figure:** [`validation/thinDisk2mm_tumbling/tumbling_validation_detailed.png`](./thinDisk2mm_tumbling/tumbling_validation_detailed.png)
- **Preview Snapshot:** [`validation/thinDisk2mm_tumbling/render/disk_drift_animation_preview.png`](./thinDisk2mm_tumbling/render/disk_drift_animation_preview.png)
- **Validation Script:** [`validation/thinDisk2mm_tumbling/plot_tumbling_detailed.py`](./thinDisk2mm_tumbling/plot_tumbling_detailed.py)

---

## 9. Code Verification & Numerical Uncertainty Quantification (ASME V&V 20-2009 Standards)

In strict accordance with the **ASME V&V 20-2009 Standard** (*Standard for Verification and Validation in Computational Fluid Dynamics and Heat Transfer*) and **AIAA G-077-1998**, numerical errors must be rigorously segregated and quantified into:
1. **Code Verification:** Mathematical consistency and exactness of numerical algorithms independently of grid errors.
2. **Spatial Discretization Error:** Systematic grid refinement and calculation of the Grid Convergence Index ($\mathrm{GCI}_\mathrm{spatial}$).
3. **Temporal Discretization Error:** Systematic time-step refinement and calculation of the Temporal Grid Convergence Index ($\mathrm{GCI}_\mathrm{temporal}$).
4. **Limit-Cycle Stationarity:** Cycle-to-cycle asymptotic stationarity of the non-linear 6DoF attractor.

---

### 9.1. Code Verification: 6DoF RK4 Integrator, Conservation Laws, and Production C++ Diagnostics

The 6DoF dynamics and production solver implementation were verified using dedicated analytical benchmarks and production runtime diagnostics ([`validation/code_verification/verify_6dof_integrator.py`](./code_verification/verify_6dof_integrator.py)):

#### 9.1.1. Asymmetric Rigid-Body Precession & Observed Order of Convergence:
Euler's non-linear rotational equations ($\mathbf{J}_b \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{J}_b \boldsymbol{\omega}) = \mathbf{M}$) as implemented in [`src/solvers/freeBody6DoFPimpleFoam/correctRotorDynamics.H`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/src/solvers/freeBody6DoFPimpleFoam/correctRotorDynamics.H#L53-L64) were integrated across systematic time-step halving ($\Delta t = 2\text{ ms} \to 0.25\text{ ms}$) for an unforced asymmetric top ($\mathbf{I} = [1, 2, 3] \times 10^{-14}\text{ kg}\cdot\text{m}^2, \boldsymbol{\omega}_0 = [100, 150, 80]\text{ rad/s}$):

| Time Step $\Delta t$ (s) | Step Count | $L_2$ Angular Velocity Error (rad/s) | Observed Order $p$ | Theoretical Order |
| :---: | :---: | :---: | :---: | :---: |
| $2.00 \times 10^{-3}$ | 25 | $7.0405 \times 10^{-3}$ | Baseline | $4.0000$ |
| $1.00 \times 10^{-3}$ | 50 | $4.5024 \times 10^{-4}$ | **$3.9669$** | $4.0000$ |
| $5.00 \times 10^{-4}$ | 100 | $2.8481 \times 10^{-5}$ | **$3.9826$** | $4.0000$ |
| $2.50 \times 10^{-4}$ | 200 | $1.7912 \times 10^{-6}$ | **$3.9910$** | $4.0000$ |

**Finding & Qualification:** The mathematical formulation exhibits asymptotic convergence of order **$p = 3.9910 \approx 4.0000$**, verifying algorithmic 4th-order Runge-Kutta time accuracy.  
*Methodological Qualification:* This test verifies the independent algorithmic implementation of Euler's non-linear angular velocity equations; it establishes the correctness of that tested algorithm, but does not directly verify the production C++ quaternion attitude and orientation update implementation.

#### 9.1.2. Conservation of Physical Invariants (Thin Disk Geometry):
Simulating unforced 3D disk tumbling over $t = 0.20\text{ s}$ ($13,333$ steps at operational CFD $\Delta t = 1.5 \times 10^{-5}\text{ s}$):
- **Angular Momentum Conservation:** $\|\mathbf{L}_0\| = 1.29591654 \times 10^{-11}\text{ N}\cdot\text{m}\cdot\text{s} \implies \Delta \|\mathbf{L}\| / \|\mathbf{L}_0\| = \mathbf{9.97 \times 10^{-16}}$ (double-precision machine epsilon).
- **Rotational Kinetic Energy Conservation:** $E_0 = 1.48224997 \times 10^{-9}\text{ J} \implies \Delta E / E_0 = \mathbf{2.09 \times 10^{-15}}$ (machine epsilon).

#### 9.1.3. Translating Reference Frame Galilean Invariance:
In the limit of zero aerodynamic drag ($\mathbf{F}_\text{aero} = 0$), the frame acceleration equation $\mathbf{a}_\text{frame} = \mathbf{g} + \mathbf{F}_\text{aero}/m$ yields:
- Simulated frame acceleration: $|a_\text{frame} - g| = \mathbf{0.00\text{ m/s}^2}$ identically.
- Simulated frame velocity at $t = 0.20\text{ s}$: $V_\text{num} = 1.96195095\text{ m/s}$ (Exact: $1.96195095\text{ m/s}$, relative drift $2.85 \times 10^{-13}$).
- Galilean invariance confirmed: the Navier-Stokes reference frame source term $-\mathbf{a}_\text{frame}$ identically cancels gravitational acceleration $+\mathbf{g}$ in the fluid momentum equations.

#### 9.1.4. Production Solver Runtime Diagnostics (`freeBody6DoFPimpleFoam` C++ Execution):
Directly parsing execution diagnostics from the production solver run ($14,000+$ parallel time steps):
- **Spherical Sliding Interface (cyclicAMI) Geometric Coverage:**
  - Analyzed across $14,946$ dynamic interface updates.
  - Minimum AMI source weight sum: **$0.9568 > 0$** (identically **0 unmapped faces**).
  - Maximum AMI source weight sum: **$1.0226$**.
  - Global average AMI weight sum: **$0.998067$** (within $0.2\%$ of exact unity $1.000000$).
  - *Methodological Qualification:* A positive minimum source-weight sum confirms geometric interface overlap without unmapped boundary holes; however, it is not an independent measurement of integrated flux balance across the interface. Calculating and reporting the direct integrated flux mismatch across master and slave AMI patches remains an open verification item.
- **Fluid Continuity & Mass Conservation:**
  - Evaluated across $44,832$ PIMPLE continuity checks.
  - Cumulative mass continuity error: **$9.910 \times 10^{-11}$**.
  - Maximum single-step continuity error: $1.436 \times 10^{-9}$.
  - Confirms fluid mass conservation across the accelerating reference frame and rotating AMI is maintained to $\sim 10^{-10}$.
- **Open Verification Objectives:**
  - Coupled Fluid-Structure Method of Manufactured Solutions (MMS) and direct AMI interface flux balance remain open verification objectives for formal release.

---

### 9.2. Spatial Discretization Uncertainty: Grid Convergence Study Across 3 Meshes

Following ASME V&V 20-2009 guidelines, a systematic spatial grid refinement study was conducted using three conformal sliding Arbitrary Mesh Interface grids:
- **Grid 1 (Fine):** $N_1 = 368,760$ cells (rotating: 300,568, static: 68,192; max non-orthogonality $41.6^\circ$, avg $6.77^\circ$).
- **Grid 2 (Medium, Baseline Production):** $N_2 = 212,792$ cells (rotating: 178,160, static: 34,632; max non-ortho $37.3^\circ$, avg $7.17^\circ$).
- **Grid 3 (Coarse):** $N_3 = 130,928$ cells (rotating: 96,344, static: 34,584; max non-ortho $36.9^\circ$, avg $7.94^\circ$).

Refinement ratios:
$$r_{21} = \left(\frac{N_1}{N_2}\right)^{1/3} = 1.2011, \quad r_{32} = \left(\frac{N_2}{N_3}\right)^{1/3} = 1.1757$$

Simulations were evaluated at early startup $t = 0.002000\text{ s}$ ($100$ fixed time steps at $\Delta t = 2.0 \times 10^{-5}\text{ s}$, $V_y \approx 0.0196\text{ m/s}$):

| Target Quantity of Interest (QOI) | Coarse $\phi_3$ | Medium $\phi_2$ | Fine $\phi_1$ | Difference Ratio $R = \frac{\epsilon_{21}}{\epsilon_{32}}$ | Observed Regime | Apparent Order $p$ | Observed Sensitivity / Indicator | Formal GCI |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Aerodynamic Drag Force $F_{\mathrm{aero},y}$** | $3.1480\text{ nN}$ | $3.1648\text{ nN}$ | $3.1549\text{ nN}$ | $-0.5886$ | **Oscillatory Convergence** | N/A | $\Delta_\mathrm{half} = 0.00346\text{ nN}$ ($0.110\%$) | N/A (non-monotonic) |
| **Total Resultant Force $|\mathbf{F}_\mathrm{aero}|$** | $3.1480\text{ nN}$ | $3.1648\text{ nN}$ | $3.1549\text{ nN}$ | $-0.5886$ | **Oscillatory Convergence** | N/A | $\Delta_\mathrm{half} = 0.00346\text{ nN}$ ($0.110\%$) | N/A (non-monotonic) |
| **Vertical Settling Speed $V_y$** | $19.5971\text{ mm/s}$ | $19.5969\text{ mm/s}$ | $19.5970\text{ mm/s}$ | $-0.7406$ | **Oscillatory Convergence** | N/A | $\Delta_\mathrm{half} = 0.000015\text{ mm/s}$ ($< 0.001\%$) | N/A (non-monotonic) |
| **Aerodynamic Pitching Moment $M_x$** | $0.0356 \times 10^{-16}$ | $0.0582 \times 10^{-16}$ | $0.0603 \times 10^{-16}$ | $+0.0923$ | **Monotonic** | $13.80$ (unclamped) | $e_a^{21} = 3.453\%$ (grid sensitivity) | **Undetermined** (outside asymp. range) |

#### Methodological Findings & Scope Limitations:
1. **Convergence Classification Under ASME V&V 20:**
   - In accordance with standard practice, convergence is classified by the ratio of successive differences $R = \epsilon_{21}/\epsilon_{32}$.
   - For aerodynamic drag force and vertical velocity, the solution differences change sign between successive refinements ($R = -0.589$ and $R = -0.741$), identifying an **oscillatory convergence regime**. Standard Richardson extrapolation requires monotonic behavior and does not directly apply; following ASME guidance, the order $p$ is not artificially forced to 1.
   - The reported oscillation metric $\Delta_\mathrm{half} = \frac{1}{2}|\phi_1 - \phi_3|$ evaluates to **$0.110\%$** on forces and **$< 0.001\%$** on velocity. This quantity is an **observed half-range indicator** representing the observed spread across the selected grids; without additional justification, it should not be presented as a guaranteed bound on the numerical error, nor as a formal GCI.
   - For pitching moment, convergence is monotonic ($R = +0.0923$), but the unclamped apparent order is $p = 13.80$. Because this observed order exceeds the formal second-order accuracy of the underlying spatial discretization schemes, the solution is outside the asymptotic convergence range. The observed mesh sensitivity is reported as $e_a^{21} = 3.453\%$, while the formal GCI is left as **undetermined**.
2. **Early-Time Diagnostic vs. Developed Limit Cycle:**
   - This spatial study evaluated startup flow at $t = 0.002\text{ s}$, when descent velocity was only $V_y \approx 0.02\text{ m/s}$ ($< 2\%$ of terminal velocity) and boundary layers remained laminar and attached.
   - These results confirm low startup discretization sensitivity, but **they do not establish spatial discretization uncertainty for the developed tumbling limit cycle**, which involves massive flow separation, dynamic stall, and complex periodic vortex shedding.
3. **Non-Uniform Static Zone Refinement:**
   - The grids are **not geometrically similar throughout the outer domain**: the inner rotating zone was refined systematically across grids ($96\text{k} \to 178\text{k} \to 301\text{k}$), but the static zone cell count remained nearly unchanged between Coarse and Medium ($34,584$ vs. $34,632$) and was refined only on Grid 1 ($68,192$). A formal limit-cycle spatial GCI will require geometrically similar, domain-wide uniform refinement across all zones.

---

### 9.3. Temporal Discretization Uncertainty: Systematic Time-Step Refinement

Temporal discretization uncertainty was evaluated on the production mesh across three systematically halved time steps ($r_t = 2.0$):
- **Coarse Time Step:** $\Delta t_3 = 2.0 \times 10^{-5}\text{ s}$ ($\text{Co}_\mathrm{max} \approx 1.00$, $N_\mathrm{steps} = 50$)
- **Medium Time Step:** $\Delta t_2 = 1.0 \times 10^{-5}\text{ s}$ ($\text{Co}_\mathrm{max} \approx 0.50$, $N_\mathrm{steps} = 100$)
- **Fine Time Step:** $\Delta t_1 = 5.0 \times 10^{-6}\text{ s}$ ($\text{Co}_\mathrm{max} \approx 0.25$, $N_\mathrm{steps} = 200$)

Solutions were evaluated at verified identical physical time $t = 0.001000\text{ s}$ via exact boundary interpolation:

| Target Quantity of Interest (QOI) | Coarse $\Delta t_3$ | Medium $\Delta t_2$ | Fine $\Delta t_1$ | Difference Ratio $R$ | Observed Regime | Apparent Order $p$ | Temporal Index $\mathbf{GCI}_{t,\mathrm{fine}}^{21}$ | Asymptotic Ratio $A_R$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Aerodynamic Drag Force $F_{\mathrm{aero},y}$** | $2.6114\text{ nN}$ | $2.6168\text{ nN}$ | $2.6186\text{ nN}$ | $+0.3378$ | **Monotonic** | **$1.566$** | **$0.045\%$** | **$1.001$** |
| **Total Resultant Force $|\mathbf{F}_\mathrm{aero}|$** | $2.6114\text{ nN}$ | $2.6168\text{ nN}$ | $2.6186\text{ nN}$ | $+0.3378$ | **Monotonic** | **$1.566$** | **$0.045\%$** | **$1.001$** |
| **Vertical Settling Speed $V_y$** | $9.80005\text{ mm/s}$ | $9.80001\text{ mm/s}$ | $9.79999\text{ mm/s}$ | $+0.5617$ | **Monotonic** | **$0.832$** | **$0.0004\%$** | **$1.000$** |
| **Aerodynamic Pitching Moment $M_x$** | $0.0416 \times 10^{-16}\text{ N}\cdot\text{m}$ | $0.0215 \times 10^{-16}\text{ N}\cdot\text{m}$ | $0.6899 \times 10^{-16}\text{ N}\cdot\text{m}$ | $-33.23$ | **Oscillatory Divergence** | N/A | N/A | N/A |

#### Temporal Discretization Findings & Scope Limitations:
- For aerodynamic drag force, the observed temporal convergence order is **$p = 1.566$**, consistent with the blended second-order implicit backward / Crank-Nicolson PIMPLE time scheme. The fine time-step index is $\mathrm{GCI}_t = 0.045\%$ with an asymptotic ratio of $A_R = 1.001$.
- Pitching moment at this early startup instant exhibits oscillatory divergence ($R = -33.23$), reflecting transient initial shear layer development.
- As with the spatial study, these early startup tests ($t = 0.001\text{ s}$) evaluate startup time integration; they do not establish temporal discretization error for the developed unsteady tumbling limit cycle.

---

### 9.4. Limit-Cycle Dynamics: Quasi-Settled Translation vs. Evolving Rotation

Evaluating the trajectory across Revolutions 1 through 7 clarifies the state of limit-cycle convergence:

1. **Translational Response (Quasi-Settled):**
   - Revolution 4: $\langle V_y \rangle = 1.086\text{ m/s}$
   - Revolution 5: $\langle V_y \rangle = 1.127\text{ m/s}$ ($+3.8\%$)
   - Revolution 6: $\langle V_y \rangle = 1.111\text{ m/s}$ ($-1.4\%$)
   - Revolution 7: $\langle V_y \rangle = 1.087\text{ m/s}$ ($-2.2\%$)
   Mean settling speed remains within $1.086 - 1.127\text{ m/s}$ (maximum adjacent-cycle variation $\sim 3.8\%$), demonstrating a quasi-settled translational response bounded inside the experimental $1\sigma$ spread ($[0.92, 1.54]\text{ m/s}$).
2. **Rotational Response (Evolving Limit Cycle):**
   - Revolution 4: $f_t = 19.75\text{ Hz}$ ($45.7\%$ below experimental $36.4\text{ Hz}$)
   - Revolution 5: $f_t = 23.84\text{ Hz}$ ($34.5\%$ below)
   - Revolution 6: $f_t = 27.37\text{ Hz}$ ($24.8\%$ below)
   - Revolution 7: $f_t = 29.56\text{ Hz}$ ($18.8\%$ below)
   - Cycle-averaged angular rate accelerates from $-124.1\text{ rad/s}$ (Rev 4) to $-185.8\text{ rad/s}$ (Rev 7).
   - Peak downstroke rate reaches $-223.7\text{ rad/s}$ ($97.8\%$ of experimental mean).
   While the translational settling speed is quasi-settled, the rotational limit cycle continues to evolve dynamically toward the asymptotic experimental attractor.

---

### 9.5. Peer-Review Compliance & Validation Verdict

| Item | Suggested Status | Defensible Resolution in Present Work |
| :--- | :---: | :--- |
| **Citation and experimental source corrections** | **Resolved** | Corrected paper DOI ([doi:10.1017/jfm.2023.209](https://doi.org/10.1017/jfm.2023.209)), angular velocity source (Figs. 18–20), and drop facility description (> 4 m fall). |
| **Drag-coefficient definitions** | **Resolved** | Disentangled terminal settling formula $C_{D,\mathrm{paper}} = 2W / (\rho A V_y^2) \approx 0.89 - 0.96$ from surface resultant force coefficient $C_{F,\mathrm{mag}}(t) \approx 0.72 - 0.75$. |
| **Spatial GCI methodology** | **Method corrected; developed-state uncertainty open** | Removed clamping and $p=1$ forcing; classified oscillatory convergence with observed half-range indicator ($0.11\%$). Flagged out-of-range moment ($p = 13.80$) with formal GCI undetermined. Acknowledged non-uniform static mesh and noted that developed limit-cycle spatial GCI remains an open objective. |
| **Temporal sensitivity** | **Early-time study completed; developed-state sensitivity open** | Verified physical timestamp interpolation at $t = 0.001000\text{ s}$ ($p = 1.566, \mathrm{GCI}_t = 0.045\%$). Noted that early-time forces do not represent developed 6DoF limit-cycle uncertainty. |
| **Code verification** | **Partial; production quaternion and interface-flux verification open** | Verified algorithmic formulation ($p = 3.9910$, invariants conserved to $< 10^{-15}$, frame Galilean invariance confirmed). Production C++ log verifies AMI geometric coverage and mass continuity, while direct quaternion and surface flux balance remain open verification tasks. |
| **Limit-cycle characterisation** | **Improved; rotational stationarity not yet established** | Formally characterized as a quasi-settled translational response ($V_y \approx 1.086 - 1.127\text{ m/s}$, max adjacent cycle variation $\sim 3.8\%$) with an evolving rotational response ($19.75 \to 29.56\text{ Hz}$ across Rev 4–7). |

### Final Validation Verdict:
> **Preliminary validation against a published experimental benchmark, with partial numerical verification.** The solver reproduces continuous autorotation for the seeded initial condition and obtains a settling-speed response within the experimental $1\sigma$ range. However, the rotational response is still evolving, and the current GCI calculations and production-code verification tests do not yet establish numerical uncertainty for the developed tumbling solution. Further systematic mesh and time-step studies, longer-cycle statistics, and direct verification of the production implementation are required before claiming comprehensive validation.
