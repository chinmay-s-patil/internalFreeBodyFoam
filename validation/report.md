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
**Preliminary validation against a published experimental benchmark, with identified numerical, input, and physical uncertainties.** The solver reproduces continuous autorotation for the seeded initial condition, completing 13 full revolutions ($t = 0.500\text{ s}$, 29,715 steps) and approaching a rotational frequency plateau of $f_t = 30.52 \pm 0.17\text{ Hz}$ across Revolutions 8–13 ($\leq 1\%$ cycle-to-cycle variation). 

The simulated vertical settling velocity ($V_y \approx 1.03 - 1.05\text{ m/s}$, mean $1.040\text{ m/s}$) is $\sim 15.4\%$ below the experimental mean terminal velocity ($V_t = 1.23\text{ m/s}$). Under the ASME V&V 20 framework:
- The comparison error is $E = S - D = 1.040 - 1.230 = -0.190\text{ m/s}$ ($-15.4\%$).
- Experimental measurement uncertainty on the mean terminal velocity is $u_{\mathrm{exp}} = 1.2\% = 0.015\text{ m/s}$ (Table 2).
- The manufacturing thickness tolerance of $h = 50 \pm 7.5\ \mu\text{m}$ ($\pm 15\%$, Table 1) introduces an input parameter uncertainty in terminal velocity of $u_{\mathrm{input}}(V_t) \approx \pm 7.5\%$ ($\pm 0.092\text{ m/s}$).
- With combined validation uncertainty $u_{\mathrm{val}} = \sqrt{u_{\mathrm{exp}}^2 + u_{\mathrm{input}}^2} \approx 7.6\%$ (before adding numerical spatial/temporal discretization uncertainty), the comparison error $|E| \approx 2 u_{\mathrm{val}}$. Thus, the thickness tolerance accounts for roughly **half** the observed velocity gap. Matching $1.23\text{ m/s}$ purely through thickness variation would require $h \approx 70\ \mu\text{m}$ (about $+2.6\sigma$).

The startup-based GCI studies ($t = 1 - 2\text{ ms}$) evaluate early-time sensitivity ($V_y \approx g \cdot t$, $M_x \sim 10^{-18}\text{ N}\cdot\text{m}$ roundoff noise) and do not establish spatial/temporal discretization error for the developed unsteady limit cycle. Direct verification of the production C++ quaternion algorithm, formal domain-wide limit-cycle GCI, and bracket runs ($h = 42.5$ and $57.5\ \mu\text{m}$) remain open objectives.

---

### Key Physical & Kinematic Findings (13 Completed Revolutions, $t = 0 \to 0.500\text{ s}$):

- **Continuous Tumbling Autorotation (13 Full Turns):** When seeded with initial rotation, the disk sustains continuous unidirectional autorotation without rotational reversals (**13 completed revolutions**, $t = 0.500\text{ s}$, 29,715 time steps).
- **Rotational Response (Approaching a Plateau):** Rather than reaching a completely stationary limit cycle, the tumbling frequency approaches an asymptotic plateau across Revolutions 8 through 13:
  $$f_t = 30.52 \pm 0.17\text{ Hz}, \quad T = 32.77 \pm 0.18\text{ ms}$$
  The frequency peaks at Revolution 9 ($30.73\text{ Hz}$) and gently drifts to $30.24\text{ Hz}$ at Revolution 13, remaining within $\leq 1\%$ cycle-to-cycle variation.
- **Frequency & Strouhal Number Comparison:**
  - Experimental frequency PDF for 2 mm disks peaks near $\sim 33\text{ Hz}$ (with distribution spanning $25 - 42\text{ Hz}$ and mean $36.4\text{ Hz}$, Fig. 20a). Simulated $f_t \approx 30.5\text{ Hz}$ is $7.5\%$ below the PDF peak and $16\%$ below the mean.
  - In Strouhal number, simulated $St = \frac{f_t D}{\langle V_y \rangle} = 0.0593 \pm 0.0006 \approx 0.059$, compared to the experimental peak of $St \approx 0.055 - 0.060$ (Fig. 20b).
  - Scaled to experimental terminal velocity ($V_t = 1.23\text{ m/s}$) at the simulated Strouhal number, the frequency evaluates to $f = St \cdot V_t / D = 36.1\text{ Hz}$ (within $1\%$ of the experimental mean $36.4\text{ Hz}$). The close agreement in Strouhal number partly reflects the velocity deficit. In nondimensional time units ($\tau_t = \sqrt{\tilde{\rho} h / g} = 76.6\text{ ms}$), the nondimensional frequency is $f_t \tau_t = 2.34$.
- **Translational Response & Settling Speed:** Vertical descent velocity settles from $1.064\text{ m/s}$ (Rev 8) to $1.026\text{ m/s}$ (Rev 13), with cycle-to-cycle change dropping to $-0.16\%$. The late-run mean is $\langle V_y \rangle = 1.040\text{ m/s}$ ($Re \approx 140$).
- **Aerodynamic Force Balance:** Across the plateau (Revs 8–13), mean vertical aerodynamic force is $F_y = 2.2215\ \mu\text{N}$, giving a vertical force balance ratio of $F_y / W = 1.024$ (within $2.4\%$ of gravitational weight $W = 2.1693\ \mu\text{N}$). Cycle-by-cycle, $F_y / W$ drops monotonically from $1.056$ (Rev 8) to $1.004$ (Rev 13) as vertical deceleration tapers off. Mean total resultant aerodynamic force magnitude is $|\mathbf{F}_{\mathrm{aero}}| = 2.5042\ \mu\text{N}$ ($|\mathbf{F}_{\mathrm{aero}}| / W = 1.154$).
- **Aerodynamic Force Coefficients:**
  - The settling-equivalent drag coefficient evaluated at simulated speed is $C_{D,\mathrm{paper}} = 2W / (\rho_f A \langle V_y \rangle^2) = 1.042 \pm 0.03$. This falls inside the transformed velocity envelope of $[0.47, 1.33]$ derived from the experimental velocity distribution spread $V_t \pm \sigma(u_y) = 1.23 \pm 0.31\text{ m/s}$.
  - The surface-integrated resultant force coefficient $C_{F,\mathrm{mag}}(t) = |\mathbf{F}_{\mathrm{aero}}| / (\frac{1}{2}\rho_f A V^2)$ oscillates between **$0.26$** (edge-on downstroke) and **$2.05$** (broadside crossing), with 95th percentile at **$2.02$** and cycle-average of $\langle C_{F,\mathrm{mag}} \rangle = \mathbf{1.119}$. *(Note: Earlier drafts reported $0.72 - 0.75$; that number was an error from an earlier integration window and is corrected here).*
- **Instantaneous Drag Coefficient PDF ($C_{D,\mathrm{inst}}$, Figure 13):**
  - Using projected area $A_{\mathrm{inst}} = A |p_y| = A |\cos\theta|$, the simulated $C_{D,\mathrm{inst}}$ distribution mode is at $\approx 1.05 - 1.2$.
  - This compares with the experimental mode of $\approx 0.75$ in Tinklenberg et al. (Fig. 13). Scaling the experimental mode by the velocity-squared ratio:
    $$0.75 \times \left(\frac{V_{\mathrm{exp}}}{V_{\mathrm{sim}}}\right)^2 = 0.75 \times \left(\frac{1.23}{1.04}\right)^2 = 0.75 \times 1.399 \approx 1.05$$
    The orientation–drag behavior matches the experimental distribution shape, confirming that the discrepancy is governed by the mean settling speed deficit.
- **Trajectory Inclination $\psi$:** Over Revs 8–13, instantaneous flight angle $\psi(t) = \arctan(|V_z| / V_y)$ oscillates between $12.46^\circ$ and $17.19^\circ$ (mean $15.30^\circ$, track displacement angle $\psi_{\mathrm{track}} = 15.29^\circ$). Horizontal velocity $V_z$ continues to creep slightly ($+0.009\text{ m/s}$ over the final $100\text{ ms}$). This puts $\psi$ within the experimental distribution (which extends to $40^\circ$), but $\sim 45\%$ above its peak mode ($15.3^\circ$ vs. $10.5^\circ$, Fig. 10b).
- **2D Planar Symmetry:** Over the full run, maximum out-of-plane rate is $|\Omega_z| = 1.22\text{ rad/s}$ ($1.16\text{ rad/s}$ in the plateau) and $|\Omega_y| = 0.036\text{ rad/s}$. Compared to mean pitch rate $|\Omega_x| \approx 191.7\text{ rad/s}$, out-of-plane rates remain $< 0.6\%$, reflecting planar symmetry preservation.

---

## 2. Benchmark Physical Parameters

The physical parameters correspond to the $2\text{ mm}$ PET disk characterized in Table 1 & Table 3 of Tinklenberg et al. (2023):

| Parameter | Symbol | Value (SI) | Value (Practical) | Experimental Source / Note |
| :--- | :--- | :--- | :--- | :--- |
| **Disk Diameter** | $D$ | $2.02 \times 10^{-3}\text{ m}$ | $2.02\text{ mm}$ | Table 1 |
| **Disk Thickness** | $h$ | $5.00 \times 10^{-5}\text{ m}$ | $50.0 \pm 7.5\ \mu\text{m}$ | Table 1 ($\pm 15\%$ manufacturing tolerance) |
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
- **Time Step & Courant Numbers:** Adaptive PIMPLE stepping with $\Delta t \approx 1.5 \times 10^{-5}\text{ s}$ (3 PIMPLE correctors). In the developed production run, maximum Courant number reached $\mathrm{Co}_{\mathrm{max}} = 1.62$ (mean $\mathrm{Co}_{\mathrm{max}} = 1.50$, domain mean $\mathrm{Co}_{\mathrm{mean}} = 0.0127$), remaining numerically stable throughout.

---

## 4. Quantitative Comparison: Initial Zero-Spin Drop (`thinDisk2mm`)

In the initial exploratory run (`validation/thinDisk2mm/`), the disk was released with $\Omega_0 = 0$ and $\theta_0 = 15^\circ$ in quiescent air:

| Metric | Experimental Value (Tinklenberg et al. 2023) | freeBody6DoFPimpleFoam (Zero-Spin) | Methodological Note / Distinction |
| :--- | :--- | :--- | :--- |
| **Terminal Descent Speed $V_y$** | $1.23 \pm 0.31\text{ m/s}$ (Table 3; $\sigma(u_y) = \pm 0.31$) | $1.03 - 1.05\text{ m/s}$ | Falls inside natural velocity distribution; $-15.4\%$ from mean |
| **Descent Velocity Uncertainty** | $u_{\mathrm{exp}} = 1.2\%$ ($0.015\text{ m/s}$, Table 2) | — | Measurement uncertainty on mean velocity |
| **Aerodynamic Force Balance** | $F_\text{aero} / W = 1.000$ (Terminal equilibrium) | $F_\text{aero} / W = 1.001$ | $0.1\%$ numerical force balance error |
| **Descent Reynolds Number $Re$** | $165 \pm 42$ (Table 3; derived from $V_t \pm \sigma$) | $138 - 141$ | Inside experimental Reynolds range $[123, 207]$ |
| **Drag Coefficient $C_{D,\mathrm{paper}}$** | $0.74$ (Table 3; $[0.47, 1.33]$ from $V_t \pm \sigma$) | $1.07 - 1.13$ (Broadside flutter) | Matches broadside flutter literature ($C_D \approx 1.10 - 1.15$) |
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
   with an amplitude damping ratio of $\approx 0.7 - 0.8$ per half-cycle.
4. **Physical Characterization:** This response is a **damped rocking motion decaying toward a flat, stable broadside descent**, rather than a self-sustained flutter limit cycle.

> [!NOTE]
> In Tinklenberg et al. (Fig. 1 and Table 3), a stable broadside fall regime exists in the parameter space for 1 mm and 3 mm disks, while 2 mm disks tumbled. In experimental drop facilities (Field et al. 1997; Tinklenberg et al. 2023), ambient disturbances and boundary-layer destabilization over $> 4\text{ m}$ fall distances kick disks into the tumbling attractor. A 2D initial-condition sweep $(\theta_0, \Omega_0)$ is needed to map the basin of attraction between stable fall and tumbling.

### 5.2. Experimental Facility vs. Simulation Fall Distance
- In the laboratory experiment, disks fell through a **3-metre vertical drop chute connected to a larger chamber**, yielding over 4 metres of fall distance before reaching the optical measurement zone. This ensured release transients were forgotten.
- In contrast, the zero-spin CFD run reached $t \approx 0.235\text{ s}$ ($18.8\text{ cm}$ of fall). Because the release transient in quiescent air trapped the disk in a decaying broadside mode, verifying the tumbling regime required seeding rotation directly (Section 8).

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
   In Tinklenberg et al. (2023), Table 3 reports a single drag coefficient $C_D = 0.74$ derived directly from the mean terminal descent velocity $V_t = 1.23\text{ m/s}$ assuming gravitational force equilibrium:
   $$C_{D,\mathrm{paper}} = \frac{2 W}{\rho_f A V_t^2} = \frac{2 m g}{\rho_f A V_t^2}$$
   Using the disk properties ($m = 2.2113 \times 10^{-7}\text{ kg}, g = 9.81\text{ m/s}^2, W = 2.1693\ \mu\text{N}, \rho_f = 1.200\text{ kg/m}^3, D = 2.02\text{ mm}, A = 3.2047 \times 10^{-6}\text{ m}^2$):
   - **Experimental Reference:** For mean $V_t = 1.23\text{ m/s}$, this gives $C_{D,\mathrm{paper}} = 0.746 \approx 0.75$. Transforming the experimental velocity distribution endpoints ($V_t \pm \sigma(u_y) = 1.23 \pm 0.31\text{ m/s} \implies [0.92, 1.54]\text{ m/s}$) produces an equivalent velocity-transformed interval of **$[0.47, 1.33]$**. (The paper reports $\pm 0.31\text{ m/s}$ as the velocity distribution spread $\sigma(u_y)$, not as an independently measured $C_D$ uncertainty).
   - **Simulation Settling Range:** In the developed window (Revolutions 8–13), the cycle-averaged vertical speed settles into $V_y = 1.026 - 1.064\text{ m/s}$ (mean $1.040\text{ m/s}$). Evaluated with the paper's formula:
     $$C_{D,\mathrm{paper}}(\langle V_y \rangle = 1.040\text{ m/s}) = 1.042 \pm 0.03$$
     This falls inside the transformed experimental velocity envelope ($[0.47, 1.33]$), but is $\sim 40\%$ higher than the $0.746$ mean value because simulated fall speed is $\sim 15.4\%$ lower than $1.23\text{ m/s}$.

2. **Resultant Aerodynamic-Force Coefficient $C_{F,\mathrm{mag}}(t)$:**
   $$C_{F,\mathrm{mag}}(t) = \frac{|\mathbf{F}_\mathrm{aero}(t)|}{\frac{1}{2} \rho_f A |\mathbf{V}(t)|^2}$$
   During continuous autorotation, $C_{F,\mathrm{mag}}(t)$ sweeps periodically between **$0.26$** (edge-on downstroke) and **$2.05$** (broadside crossing), with a 95th percentile of **$2.02$** and a cycle-average of **$1.119$**. *(Earlier drafts reported $0.72 - 0.75$; that number was an error from an earlier integration window and is corrected here).*

---

### 8.2. Complete 13-Revolution Kinematic Evolution (Time-Weighted Trapezoidal Averages)

Tracking the simulation cycle-by-cycle (where each revolution represents an exact $360^\circ$ rotation) using continuous time-weighted trapezoidal integration across the entire run ($t = 0 \to 0.500\text{ s}$):

| Cycle | Time Interval (ms) | Period $T$ (ms) | $f_t$ (Hz) | $St = \frac{f_t D}{\langle V_y \rangle}$ | $f_t \tau_t$ | Mean $V_y$ (m/s) | $\Delta V_y$ | Mean $C_{F,\mathrm{mag}}$ | $C_{D,\mathrm{paper}}$ | Dynamic Status |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **Rev 1** | $0.0 \to 30.4$ | $30.37$ | $32.93$ | $0.4531$ | $2.521$ | $0.1468$ | — | $0.265$ | $52.35$ | Initial acceleration |
| **Rev 2** | $30.4 \to 66.6$ | $36.26$ | $27.58$ | $0.1181$ | $2.112$ | $0.4716$ | $+221.2\%$ | $0.123$ | $5.07$ | Transitional fall |
| **Rev 3** | $66.6 \to 111.3$ | $44.69$ | $22.38$ | $0.0541$ | $1.713$ | $0.8353$ | $+77.1\%$ | $0.344$ | $1.62$ | Boundary layer setup |
| **Rev 4** | $111.3 \to 161.9$ | $50.63$ | $19.75$ | $0.0367$ | $1.512$ | $1.0858$ | $+30.0\%$ | $0.798$ | $0.957$ | Quasi-settled speed |
| **Rev 5** | $161.9 \to 203.9$ | $41.95$ | $23.84$ | $0.0427$ | $1.825$ | $1.1268$ | $+3.77\%$ | $1.002$ | $0.889$ | Rotation developing |
| **Rev 6** | $203.9 \to 240.4$ | $36.54$ | $27.37$ | $0.0498$ | $2.095$ | $1.1106$ | $-1.44\%$ | $1.058$ | $0.915$ | Rotation developing |
| **Rev 7** | $240.4 \to 274.3$ | $33.82$ | $29.56$ | $0.0550$ | $2.263$ | $1.0863$ | $-2.19\%$ | $1.108$ | $0.956$ | Transition to plateau |
| **Rev 8** | $274.3 \to 307.0$ | $32.79$ | $30.50$ | $0.0579$ | $2.335$ | $1.0637$ | $-2.08\%$ | $1.120$ | $0.997$ | Approaching plateau |
| **Rev 9** | $307.0 \to 339.6$ | $32.55$ | $30.73$ | $0.0592$ | $2.352$ | $1.0482$ | $-1.45\%$ | $1.119$ | $1.027$ | Approaching plateau |
| **Rev 10** | $339.6 \to 372.2$ | $32.59$ | $30.69$ | $0.0597$ | $2.349$ | $1.0385$ | $-0.93\%$ | $1.119$ | $1.046$ | Approaching plateau |
| **Rev 11** | $372.2 \to 404.9$ | $32.73$ | $30.55$ | $0.0598$ | $2.339$ | $1.0318$ | $-0.64\%$ | $1.118$ | $1.060$ | Approaching plateau |
| **Rev 12** | $404.9 \to 437.8$ | $32.91$ | $30.39$ | $0.0597$ | $2.327$ | $1.0280$ | $-0.38\%$ | $1.117$ | $1.068$ | Approaching plateau |
| **Rev 13** | $437.8 \to 470.9$ | $33.07$ | $30.24$ | $0.0595$ | $2.315$ | $1.0263$ | $-0.16\%$ | $1.117$ | $1.071$ | Approaching plateau |

#### Plateau Statistics (Revolutions 8–13, $t = 274.3\text{ ms} \to 470.9\text{ ms}$):
- **Tumbling Frequency:** $f_t = 30.52 \pm 0.17\text{ Hz}$ (variation $< 0.6\%$; range $30.24 - 30.73\text{ Hz}$).
- **Cycle Period:** $T = 32.77 \pm 0.18\text{ ms}$ (range $32.55 - 33.07\text{ ms}$).
- **Strouhal Number:** $St = \frac{f_t D}{\langle V_y \rangle} = 0.0593 \pm 0.0006 \approx 0.059$.
- **Nondimensional Frequency:** $f_t \tau_t = 2.336 \pm 0.013$.
- **Vertical Settling Speed:** $\langle V_y \rangle = 1.040 \pm 0.015\text{ m/s}$ (settling with adjacent cycle variation down to $-0.16\%$).
- **Vertical Force Balance:** Mean $F_y = 2.2215\ \mu\text{N}$, $F_y / W = 1.0241$ (dropping $1.056 \to 1.004$ across Revs 8–13). Mean $|\mathbf{F}_{\mathrm{aero}}| = 2.5042\ \mu\text{N}$ ($|\mathbf{F}_{\mathrm{aero}}| / W = 1.1544$).
- **Equivalent Drag Coefficient:** $C_{D,\mathrm{paper}} = 1.042 \pm 0.03$.

---

### 8.3. Comparison Against Experimental Benchmark Distributions

1. **Tumbling Frequency & Strouhal Number:**
   - In Tinklenberg et al., Figure 20(a) shows the measured PDF of tumbling frequency for 2 mm disks. The PDF exhibits a peak mode around $\sim 33\text{ Hz}$ and an overall distribution spanning $25 - 42\text{ Hz}$ (mean $36.4\text{ Hz}$). The simulation's frequency of $30.52\text{ Hz}$ is $7.5\%$ below the PDF peak and $16\%$ below the mean.
   - In Figure 20(b), the experimental Strouhal number distribution peaks at $St \approx 0.055 - 0.060$. The simulated value $St = 0.0593$ falls within the experimental peak mode. Scaled to $V_t = 1.23\text{ m/s}$, the frequency becomes $f = St \cdot V_t / D = 36.1\text{ Hz}$ (matching the $36.4\text{ Hz}$ mean within $1\%$), confirming that the close Strouhal match partly reflects the velocity deficit.

2. **Terminal Descent Velocity & Validation Uncertainty (ASME V&V 20):**
   - Experimental mean terminal velocity is $V_t = 1.23\text{ m/s}$, with measurement uncertainty $u_{\mathrm{exp}} = 1.2\%$ ($0.015\text{ m/s}$, Table 2).
   - Simulated settling speed is $V_y = 1.040\text{ m/s} \implies E = S - D = -0.190\text{ m/s}$ ($-15.4\%$).
   - **Input Parameter Uncertainty ($u_{\mathrm{input}}$):** From Table 1 of the paper, the PET disk thickness is specified as $h = 50 \pm 7.5\ \mu\text{m}$ ($\pm 15\%$ tolerance). Since terminal velocity scales approximately as $V_t \propto \sqrt{m} \propto \sqrt{h}$, this $15\%$ thickness variation produces an input uncertainty of $u_{\mathrm{input}}(V_t) \approx 7.5\% \approx 0.092\text{ m/s}$.
   - Combined uncertainty before numerical discretization error is $u_{\mathrm{val}} \approx \sqrt{7.5^2 + 1.2^2}\% \approx 7.6\%$. The comparison error $|E| = 15.4\%$ is about $2 u_{\mathrm{val}}$. Therefore, the thickness tolerance accounts for roughly **half** the observed velocity gap.
   - Matching $1.23\text{ m/s}$ purely via thickness scaling would require $h \approx 70\ \mu\text{m}$ (about $+2.6\sigma$). The direct test to isolate $u_{\mathrm{input}}$ is two extra runs at $h = 42.5$ and $57.5\ \mu\text{m}$.

3. **Instantaneous Drag Coefficient PDF ($C_{D,\mathrm{inst}}$, Figure 13):**
   - Evaluating the instantaneous drag coefficient PDF using projected area $A_{\mathrm{inst}} = A |p_y| = A |\cos\theta|$ yields a simulated mode at $\sim 1.05 - 1.2$.
   - In Tinklenberg et al. (Fig. 13), the measured $C_{D,\mathrm{inst}}$ PDF mode for 2 mm disks is at $\approx 0.75$.
   - The ratio $(V_{\mathrm{exp}} / V_{\mathrm{sim}})^2 = (1.23 / 1.04)^2 \approx 1.40$. Scaling the experimental mode by $1.40$ gives $0.75 \times 1.40 = 1.05$, exactly matching the simulated distribution mode. This confirms that the orientation-dependent drag behavior is consistent with the experiment, and the difference is governed by the mean settling speed deficit.

4. **Trajectory Geometry & Drift Angle Reconciliation:**
   - Over Revs 8–13, instantaneous flight angle $\psi(t) = \arctan(|V_z| / V_y)$ oscillates between $12.46^\circ$ and $17.19^\circ$ (mean $15.30^\circ$, track displacement angle $\psi_{\mathrm{track}} = 15.29^\circ$). Horizontal velocity $V_z$ continues to creep slightly ($+0.009\text{ m/s}$ over the final $100\text{ ms}$).
   - In Tinklenberg et al., Figure 10(b) shows the probability distribution of trajectory inclination $|\psi|$ from linear track fits, with a peak mode at $\approx 10.5^\circ$ and a long tail extending to $40^\circ$. The simulated trajectory angle is **within the experimental distribution, but above its peak mode** ($15.3^\circ$ vs. $10.5^\circ$).

---

### 8.4. Deliverables for Tumbling Case
- **6-Panel Publication Figure:** [`validation/thinDisk2mm_tumbling/tumbling_validation_detailed.png`](./thinDisk2mm_tumbling/tumbling_validation_detailed.png)
- **Validation Script:** [`validation/thinDisk2mm_tumbling/plot_tumbling_detailed.py`](./thinDisk2mm_tumbling/plot_tumbling_detailed.py)

---

## 9. Code Verification & Numerical Uncertainty Quantification (ASME V&V 20 Standards)

### 9.1. Code Verification: 6DoF Integrator, Conservation Laws, and Production C++ Diagnostics

#### 9.1.1. Asymmetric Rigid-Body Precession & Observed Order of Convergence:
Euler's non-linear rotational equations ($\mathbf{J}_b \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{J}_b \boldsymbol{\omega}) = \mathbf{M}$) were integrated across systematic time-step halving ($\Delta t = 2\text{ ms} \to 0.25\text{ ms}$) for an unforced asymmetric body ($\mathbf{I} = [1, 2, 3] \times 10^{-14}\text{ kg}\cdot\text{m}^2, \boldsymbol{\omega}_0 = [100, 150, 80]\text{ rad/s}$):

| Time Step $\Delta t$ (s) | Step Count | $L_2$ Angular Velocity Error (rad/s) | Observed Order $p$ | Theoretical Order |
| :---: | :---: | :---: | :---: | :---: |
| $2.00 \times 10^{-3}$ | 25 | $7.0405 \times 10^{-3}$ | Baseline | $4.0000$ |
| $1.00 \times 10^{-3}$ | 50 | $4.5024 \times 10^{-4}$ | **$3.9669$** | $4.0000$ |
| $5.00 \times 10^{-4}$ | 100 | $2.8481 \times 10^{-5}$ | **$3.9826$** | $4.0000$ |
| $2.50 \times 10^{-4}$ | 200 | $1.7912 \times 10^{-6}$ | **$3.9910$** | $4.0000$ |

**Finding & Methodological Qualification:** The mathematical formulation exhibits asymptotic convergence of order **$p = 3.9910 \approx 4.0000$**, verifying algorithmic 4th-order Runge-Kutta time accuracy.  
*Qualification:* This benchmark verifies the mathematical formulation of Euler's equations as implemented in Python; it does not directly test the production C++ quaternion attitude updates in `freeBodyRotationalMotion`.

#### 9.1.2. Conservation of Physical Invariants (Non-Trivial Precession of Symmetric Top):
To test genuine non-linear gyroscopic coupling, initial angular velocity was specified with non-zero transverse and polar components: $\boldsymbol{\omega}_0 = (200.0, 50.0, 30.0)\text{ rad/s}$. Because $I_{yy} = 2 I_{xx} \ne I_{xx}$, the gyroscopic cross-product $\boldsymbol{\omega} \times (\mathbf{J} \boldsymbol{\omega})$ produces active precession ($\dot{\omega}_x \ne 0, \dot{\omega}_z \ne 0$):
- Simulating unforced 3D precession over $t = 0.20\text{ s}$ ($13,333$ steps at operational CFD $\Delta t = 1.5 \times 10^{-5}\text{ s}$):
  - **Angular Momentum Conservation:** $\|\mathbf{L}_0\| = 1.27305037 \times 10^{-11}\text{ N}\cdot\text{m}\cdot\text{s} \implies \Delta \|\mathbf{L}\| / \|\mathbf{L}_0\| = \mathbf{1.90 \times 10^{-15}}$ (machine epsilon).
  - **Rotational Kinetic Energy Conservation:** $E_0 = 1.29509120 \times 10^{-9}\text{ J} \implies \Delta E / E_0 = \mathbf{4.15 \times 10^{-15}}$ (machine epsilon).

#### 9.1.3. Reference Frame Galilean Invariance:
In the limit of zero aerodynamic drag ($\mathbf{F}_\text{aero} = 0$), the frame acceleration equation $\mathbf{a}_\text{frame} = \mathbf{g} + \mathbf{F}_\text{aero}/m$ yields $|a_\text{frame} - g| = 0.00\text{ m/s}^2$ identically. The simulated frame velocity at $t = 0.20\text{ s}$ is $V_\text{num} = 1.96195095\text{ m/s}$ (analytical: $1.96195095\text{ m/s}$, relative drift $2.85 \times 10^{-13}$). This check verifies algebraic cancellation of the fictitious frame acceleration $-\mathbf{a}_\text{frame}$ and gravity $+\mathbf{g}$ in the translating Navier-Stokes source terms.

#### 9.1.4. Production Solver Diagnostics (`freeBody6DoFPimpleFoam` C++ Execution):
Directly parsing execution diagnostics from the completed production solver run (29,715 time steps):
- **Spherical Sliding Interface (cyclicAMI) Geometric Coverage:**
  - Evaluated across 29,715 dynamic interface rotations.
  - Minimum AMI source weight sum: **$0.9565 > 0$** (identically **0 unmapped faces**).
  - Maximum AMI source weight sum: **$1.0231$**.
  - Global average AMI weight sum: **$0.998068$** (within $0.2\%$ of exact unity $1.000000$).
  - *Methodological Qualification:* Weight variations between $0.9565$ and $1.0231$ ($-4.3\%$ to $+2.3\%$) reflect geometric interpolation variation across arbitrary polyhedral face intersections. A positive minimum weight confirms face coverage without geometric holes, but does not independently establish surface flux conservation. Integrated surface flux mismatch calculation across master/slave patches remains an open verification item.
- **Fluid Continuity & Mass Conservation:**
  - Evaluated across $89,142$ PIMPLE continuity checks.
  - Final cumulative continuity error: **$-1.381 \times 10^{-9}$**.
  - Maximum single-step continuity error: $1.436 \times 10^{-9}$.
  - Confirms fluid mass conservation across the accelerating reference frame and rotating AMI is maintained to $\sim 10^{-9}$.

---

### 9.2. Spatial Discretization Study & Early-Time Limitations

A spatial grid study was evaluated across three conformal sliding Arbitrary Mesh Interface grids:
- **Grid 1 (Fine):** $N_1 = 368,760$ cells ($N_{\mathrm{rotating}} = 300,568, N_{\mathrm{static}} = 68,192$)
- **Grid 2 (Medium, Baseline):** $N_2 = 212,792$ cells ($N_{\mathrm{rotating}} = 178,160, N_{\mathrm{static}} = 34,632$)
- **Grid 3 (Coarse):** $N_3 = 130,928$ cells ($N_{\mathrm{rotating}} = 96,344, N_{\mathrm{static}} = 34,584$)

Refinement ratios:
$$r_{21} = \left(\frac{N_1}{N_2}\right)^{1/3} = 1.2011, \quad r_{32} = \left(\frac{N_2}{N_3}\right)^{1/3} = 1.1757$$

> [!WARNING]
> Both refinement ratios ($r \approx 1.18 - 1.20$) fall below the standard recommended threshold of $r \geq 1.30$ (ASME V&V 20-2009; Roache, 1998; Celik et al., 2008).

Simulations were evaluated at early startup $t = 0.002000\text{ s}$ ($100$ fixed time steps at $\Delta t = 2.0 \times 10^{-5}\text{ s}$, $V_y \approx 0.0196\text{ m/s}$):

| Target Quantity of Interest (QoI) | Coarse $\phi_3$ | Medium $\phi_2$ | Fine $\phi_1$ | Ratio $R = \frac{\epsilon_{21}}{\epsilon_{32}}$ | Observed Regime | Apparent Order $p$ | Observed Sensitivity Indicator | Formal GCI |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Aerodynamic Force $F_{\mathrm{aero},y}$** | $3.1480\text{ nN}$ | $3.1648\text{ nN}$ | $3.1549\text{ nN}$ | $-0.5886$ | **Oscillatory** | N/A | $\Delta_\mathrm{half} = 0.00346\text{ nN}$ ($0.110\%$) | N/A |
| **Descent Velocity $V_y$** | $19.5971\text{ mm/s}$ | $19.5969\text{ mm/s}$ | $19.5970\text{ mm/s}$ | $-0.7406$ | **Oscillatory** | N/A | $\Delta_\mathrm{half} = 0.000015\text{ mm/s}$ ($< 0.001\%$) | N/A |
| **Pitching Moment $M_x$** | $3.56 \times 10^{-18}\text{ N}\cdot\text{m}$ | $5.82 \times 10^{-18}\text{ N}\cdot\text{m}$ | $6.03 \times 10^{-18}\text{ N}\cdot\text{m}$ | $+0.0923$ | **Monotonic** | $13.80$ (unclamped) | $e_a^{21} = 3.453\%$ (grid sensitivity) | **Undetermined** |

#### Critical Methodological Critique of Early-Time QoIs:
1. **Pitching Moment Roundoff Noise ($M_x \sim 10^{-18}\text{ N}\cdot\text{m}$):**
   - The pitching moment values evaluated at $t = 0.002\text{ s}$ are on the order of $10^{-18}\text{ N}\cdot\text{m}$. By contrast, the characteristic aerodynamic moment during developed tumbling reaches $M_x \approx 6.1 \times 10^{-10}\text{ N}\cdot\text{m}$ ($0.61\text{ nN}\cdot\text{m}$), and the flutter restoring moment is $\approx 0.45\text{ nN}\cdot\text{m}$.
   - The startup values are roughly **8 orders of magnitude smaller** than the physically developed moments. Differences at $10^{-18}\text{ N}\cdot\text{m}$ represent floating-point roundoff rather than resolved physical aerodynamics, explaining the unphysical apparent order $p = 13.80$.
2. **Descent Velocity Insensitivity ($V_y \approx g \cdot t$):**
   - At $t = 2\text{ ms}$, the simulated descent velocity is $V_y = 19.597\text{ mm/s}$ (purely ballistic velocity $g \cdot t = 19.620\text{ mm/s}$). Drag is less than $0.15\%$ of gravity. Differences at the $10^{-5}$ level are insensitive to fluid mesh resolution by construction; reporting "$< 0.001\%$" reflects the triviality of drag at $2\text{ ms}$, not fluid mesh convergence.
3. **Non-Uniform Static Mesh Refinement:**
   - The static outer mesh cell count was kept nearly identical between Coarse ($34,584$) and Medium ($34,632$) and refined only on Grid 1 ($68,192$), violating strict geometric similarity across all three grids.
4. **Requirements for a Valid Limit-Cycle Spatial GCI:**
   - Requires geometrically similar domain-wide grid refinement with $r \geq 1.30$, evaluated on developed limit-cycle metrics (cycle-averaged settling speed $\langle V_y \rangle$, tumbling frequency $f_t$, and peak moment $M_x$).

---

### 9.3. Temporal Discretization Study & Early-Time Limitations

Temporal discretization was evaluated on the production mesh across three systematically halved time steps ($r_t = 2.0$):
- **Coarse Time Step:** $\Delta t_3 = 2.0 \times 10^{-5}\text{ s}$ ($\text{Co}_\mathrm{max} \approx 1.00$ at startup, $N_\mathrm{steps} = 50$)
- **Medium Time Step:** $\Delta t_2 = 1.0 \times 10^{-5}\text{ s}$ ($\text{Co}_\mathrm{max} \approx 0.50$ at startup, $N_\mathrm{steps} = 100$)
- **Fine Time Step:** $\Delta t_1 = 5.0 \times 10^{-6}\text{ s}$ ($\text{Co}_\mathrm{max} \approx 0.25$ at startup, $N_\mathrm{steps} = 200$)

Evaluated at physical time $t = 0.001000\text{ s}$ via exact boundary interpolation:

| Target Quantity of Interest (QoI) | Coarse $\Delta t_3$ | Medium $\Delta t_2$ | Fine $\Delta t_1$ | Ratio $R$ | Observed Regime | Apparent Order $p$ | Temporal Index $\mathbf{GCI}_{t,\mathrm{fine}}^{21}$ | Asymptotic Ratio $A_R$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Aerodynamic Force $F_{\mathrm{aero},y}$** | $2.6114\text{ nN}$ | $2.6168\text{ nN}$ | $2.6186\text{ nN}$ | $+0.3378$ | **Monotonic** | **$1.566$** | **$0.045\%$** | **$1.001$** |
| **Descent Velocity $V_y$** | $9.80005\text{ mm/s}$ | $9.80001\text{ mm/s}$ | $9.79999\text{ mm/s}$ | $+0.5617$ | **Monotonic** | **$0.832$** | **$0.0004\%$** | **$1.000$** |
| **Pitching Moment $M_x$** | $4.16 \times 10^{-18}\text{ N}\cdot\text{m}$ | $2.15 \times 10^{-18}\text{ N}\cdot\text{m}$ | $6.90 \times 10^{-17}\text{ N}\cdot\text{m}$ | $-33.23$ | **Oscillatory Divergence** | N/A | N/A | N/A |

#### Temporal Discretization Findings:
- For aerodynamic force, the observed order is $p = 1.566$, consistent with the blended second-order implicit backward / Crank-Nicolson PIMPLE scheme, yielding $\mathrm{GCI}_t = 0.045\%$ and $A_R = 1.001$.
- Pitching moment at this instant ($10^{-17} - 10^{-18}\text{ N}\cdot\text{m}$) is machine-precision noise around zero, producing oscillatory divergence ($R = -33.23$).
- Note that the labels $\mathrm{Co} \approx 1.0, 0.5, 0.25$ strictly held at early startup; in the developed production run, Courant numbers reached $\mathrm{Co}_{\mathrm{max}} \approx 1.62$, exceeding the startup "coarse" level.

---

### 9.4. Peer-Review Compliance & Technical Assessment

| Validation Criterion / Topic | Evaluation Status | Findings & Methodological Assessment |
| :--- | :---: | :--- |
| **Citation and Experimental Source Attribution** | **Resolved** | Corrected paper DOI to [doi:10.1017/jfm.2023.209](https://doi.org/10.1017/jfm.2023.209); cited angular velocity from Figs. 18–20; corrected drop facility description (> 4 m fall distance). |
| **Experimental Velocity Spread vs. Measurement Uncertainty** | **Resolved** | Disentangled instantaneous velocity fluctuation spread ($\sigma(u_y) = \pm 0.31\text{ m/s}$, Fig. 12) from mean measurement uncertainty ($u_{\mathrm{exp}} = 1.2\% = 0.015\text{ m/s}$, Table 2). Evaluated validation error within ASME V&V 20: manufacturing thickness tolerance ($50 \pm 7.5\ \mu\text{m}$) yields $u_{\mathrm{input}}(V_t) \approx \pm 7.5\% \approx 0.092\text{ m/s}$, explaining roughly half the $15.4\%$ gap ($|E| \approx 2 u_{\mathrm{val}}$). |
| **Aerodynamic Drag Coefficient Definitions** | **Resolved** | Separated terminal settling formula $C_{D,\mathrm{paper}} = 2W / (\rho A V_y^2) = 1.042 \pm 0.03$ from surface resultant force coefficient $C_{F,\mathrm{mag}}(t)$ (range $0.26 - 2.05$, mean $1.119$). Noted that earlier reported $0.72 - 0.75$ was an error and is corrected. |
| **Instantaneous Drag Coefficient PDF ($C_{D,\mathrm{inst}}$)** | **Resolved** | Simulated mode using $A_{\mathrm{inst}} = A |p_y|$ is $\sim 1.05 - 1.2$. Matches experimental mode of $\sim 0.75$ scaled by $(V_{\mathrm{exp}}/V_{\mathrm{sim}})^2 = (1.23/1.04)^2 \approx 1.40$, confirming orientation–drag consistency. |
| **Experimental Angular Velocity Representation** | **Resolved** | Removed fictitious "$\pm 45\text{ rad/s}$" attribution. Referenced observed tumbling rate "around 200 rad/s" (Fig. 18) and frequency distribution peak at $\sim 33\text{ Hz}$ ($2\pi f \approx 207\text{ rad/s}$, Fig. 20a). |
| **Spatial & Temporal GCI QoI Validity** | **Critique Documented; Developed GCI Open** | Explicitly documented that startup $t = 1 - 2\text{ ms}$ QoIs are insensitive ($V_y \approx g\cdot t$) or dominated by roundoff noise ($M_x \sim 10^{-18}\text{ N}\cdot\text{m}$). Noted refinement ratio limitation ($r \approx 1.18 - 1.20 < 1.30$) and non-uniform static mesh. Defined requirements for formal limit-cycle GCI. |
| **Code Verification Scope** | **Partially Verified; Production MMS Open** | Verified algorithmic formulation ($p = 3.9910$, non-trivial precession invariants conserved to $< 10^{-14}$, frame Galilean cancellation confirmed). Production C++ log verifies AMI geometric coverage (weights $0.9565 - 1.0231$) and cumulative continuity to $-1.381 \times 10^{-9}$, while direct quaternion tests and MMS remain open. |
| **Limit-Cycle Characterisation (13 Revolutions)** | **Demonstrated (Approaching Plateau)** | Extended simulation to $t = 0.500\text{ s}$ (13 completed revolutions). Characterized as approaching a plateau across Revs 8–13: $f_t = 30.52 \pm 0.17\text{ Hz}$, $T = 32.77 \pm 0.18\text{ ms}$, $\langle V_y \rangle = 1.040 \pm 0.015\text{ m/s}$ (adjacent cycle variation down to $-0.16\%$). |
| **Strouhal Number & Trajectory Angle Matching** | **Resolved** | Simulated Strouhal number $St = 0.0593 \approx 0.059$ matches Fig. 20(b) peak ($St \approx 0.055 - 0.060$). Noted that close $St$ match partly reflects velocity deficit. Developed track angle $\psi_{\mathrm{track}} = 15.29^\circ$ (instantaneous $12.5^\circ - 17.2^\circ$, mean $15.3^\circ$) is within experimental distribution, but above peak mode ($15.3^\circ$ vs. $10.5^\circ$). |
| **Zero-Spin Response Characterization** | **Resolved** | Characterized as $1.14$ turns followed by damped rocking ($51^\circ \to 36^\circ \to 25^\circ$) decaying toward a flat, stable fall, rather than a self-sustained limit cycle. |

---

## 10. Future Validation Targets & Defensible Next Steps

To transition `freeBody6DoFPimpleFoam` from preliminary validation to comprehensive production-ready validation, the following concrete validation targets and numerical studies are defined:

1. **Thickness Tolerance Bracket Simulations:**
   - Execute two bracket simulations at the experimental manufacturing thickness tolerance limits ($h = 42.5\ \mu\text{m}$ and $h = 57.5\ \mu\text{m}$, $\pm 15\%$) to directly quantify input parameter uncertainty $u_{\mathrm{input}}$ on settling velocity, Strouhal number, and autorotation frequency.
2. **Comparison with High-Order Experimental Statistical Targets:**
   - **Velocity Fluctuation PDF $u_y / V_t$ (Figure 12):** Evaluate the probability distribution of normalized instantaneous vertical velocity against the experimental distribution, which shows standard deviation $\sigma(u_y)/V_t \approx 0.25$.
   - **Orientation Autocorrelation Function $p(t)$ (Figure 19b):** Compute the orientation autocorrelation of the disk normal vector:
     $$\rho_p(\tau) = \frac{\langle \mathbf{p}(t) \cdot \mathbf{p}(t + \tau) \rangle}{\langle |\mathbf{p}|^2 \rangle}$$
     In Tinklenberg et al. (Fig. 19b), $\rho_p(\tau)$ exhibits a pronounced damped oscillation with an initial zero-crossing at $\tau \approx 13.5\text{ ms}$ and an oscillation period of $\approx 27\text{ ms}$, reflecting coherent autorotation.
3. **Initial Condition Parameter Sweep $(\Omega_0, \theta_0)$:**
   - Conduct a systematic 2D parameter sweep over release pitch angle $\theta_0 \in [0^\circ, 90^\circ]$ and initial rotation $\Omega_0 \in [0, 300]\text{ rad/s}$ to formally map the basin of attraction between stable broadside fall and tumbling.
4. **Limit-Cycle Spatial & Temporal GCI Study:**
   - Construct three domain-wide, geometrically similar meshes with uniform refinement ratio $r \geq 1.30$ across both inner rotating and outer static domains.
   - Run all three grids across developed revolutions (e.g., through 10+ cycles) and compute ASME V&V 20 GCI directly on limit-cycle QoIs: cycle-averaged settling speed $\langle V_y \rangle$, tumbling frequency $f_t$, and peak pitching moment $M_x$.
5. **Direct Production Code Verification:**
   - Implement an automated unit test suite inside OpenFOAM verifying the production C++ quaternion integration in `freeBodyRotationalMotion`.
   - Implement a post-processing utility to compute integrated face-by-face mass and momentum flux mismatches across the `masterAMI` and `slaveAMI` sliding patches during production tumbling.
