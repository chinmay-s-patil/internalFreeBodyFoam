# Benchmark Verification & Physical Validation

Quantitative verification hierarchy, non-inertial reference frame coupling checks, and physical benchmarks for `freeBodyFoam`.

---

## 1. Validation Status & ASME V&V 20 Overview

> **Status (ASME V&V 20)**: **Code Verification Complete | Validation Partial**.  
> The 6DoF ODE integrator convergence, vacuum invariant conservation, non-inertial reference frame momentum coupling, and sliding cyclicAMI continuity are verified. Tumbling autorotation kinematics are dynamically consistent ($St \approx 0.0598$). However, terminal settling velocity against the experimental benchmark of [Tinklenberg et al. (2023)](https://doi.org/10.1017/jfm.2023.209) exhibits an unresolved **$-17\%$ deficit** ($1.02\text{ m/s}$ vs. $1.23\text{ m/s}$, $|E| \approx 11.7\, u_{\text{val}}$) and is **not validated**.

### Claims-to-Evidence Summary

| Category | Claimed Feature / Metric | Evidence / Verification Basis | Limitations & Open Discrepancies | Status |
| :--- | :--- | :--- | :--- | :--- |
| **6DoF Rotational Kinematics** | 4th-order ODE convergence for $\boldsymbol{\omega}$; rigid-body invariants conserved in vacuum | Test function convergence: $p = 3.991$ for $\boldsymbol{\omega}$ using explicit RK4; kinetic energy drift $< 10^{-14}$ across 13,333 steps ($\Delta t = 1.5 \times 10^{-5}\text{ s}$, $T = 0.20\text{ s}$) | Explicit RK4 (not symplectic). Attitude update uses quaternion exponential map on $SO(3)$ (order unverified when $\boldsymbol{\omega}$ varies within a step). | **VERIFIED** (for $\boldsymbol{\omega}$) |
| **Non-Inertial Reference Frame** | Dynamic D'Alembert frame coupling achieves steady terminal equilibrium | Planar cylinder test ([`tutorials/theRodDrop/theRodDrop-Denser`](../tutorials/theRodDrop/theRodDrop-Denser)): $F_{\text{aero}}/mg = 99.97\%$ ($0.03\%$ residual), $a_y \to 0.0031\text{ m/s}^2$ | Verification of frame momentum coupling only. Does not validate aerodynamic drag coefficients (circular test). | **VERIFIED** (Frame coupling only) |
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

## 2. Code Verification (Rigid-Body Kinematics & sliding AMI)

### 2.1 6DoF Rotational ODE Integration
The 6DoF rotational dynamics in [`freeBody6DoFPimpleFoam`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C) are integrated sequentially in [`correctRotorDynamics.H`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/src/solvers/freeBody6DoFPimpleFoam/correctRotorDynamics.H):
1. **Angular Velocity ($\boldsymbol{\omega}$)**: Solved using standard explicit 4th-order Runge-Kutta (RK4) on Euler's equations:
   $$\mathbf{J}_b \frac{d\boldsymbol{\omega}_b}{dt} + \boldsymbol{\omega}_b \times (\mathbf{J}_b \boldsymbol{\omega}_b) = \mathbf{M}_b$$
   Algorithmic convergence testing on asymmetric precession test functions demonstrates asymptotic convergence order $p = 3.9910$ (matching theoretical RK4).
2. **Attitude Orientation ($\mathbf{q}$)**: Updated using the exponential map on $SO(3)$ (Rodrigues rotation vector formula) via OpenFOAM quaternion algebra:
   $$\mathbf{q}^{n+1} = \mathbf{q}^n \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_b^{n+1} \Delta t)$$
   *Note: Because $\boldsymbol{\omega}$ varies continuously across a time step, the sequential exponential map update is generally lower order than the 4th-order ODE step for $\boldsymbol{\omega}$.*
3. **Conservation of Invariants in Vacuum**:
   In zero-torque ballistic motion over $13,333$ steps ($\Delta t = 1.5 \times 10^{-5}\text{ s}$, $T = 0.20\text{ s}$):
   - Relative angular momentum drift: $|\Delta \|\mathbf{L}\| / \|\mathbf{L}_0\|| < 1.0 \times 10^{-14}$ (double-precision machine precision)
   - Relative rotational kinetic energy drift: $|\Delta E / E_0| < 1.0 \times 10^{-14}$
   - Quaternion norm invariance: $|\|\mathbf{q}\| - 1.0| < 5.0 \times 10^{-15}$
   *Methodological note: RK4 is an explicit, non-symplectic integrator; strict invariant conservation holds to machine precision in the unforced limit, but does not guarantee symplectic Hamiltonian preservation under general external forcing.*

---

## 3. Frame Coupling Verification: Planar Cylinder (`theRodDrop`)

The case [`tutorials/theRodDrop/theRodDrop-Denser`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/tutorials/theRodDrop/theRodDrop-Denser) verifies the non-inertial accelerating reference frame implementation under high-momentum hydrodynamic drag.

### 3.1 Case Parameters & Numerical Setup
* **Geometry**: 2D planar half-cylinder of radius $R = 2.0\text{ m}$ (diameter $D = 4.0\text{ m}$), unit span $w = 1.0\text{ m}$, frontal area $A = R \cdot w = 2.0\text{ m}^2$ (with symmetry plane at $X = 0$).
* **Body Mass**: $m = 98,018\text{ kg}$ ($\rho_s = 7800\text{ kg/m}^3$, steel cylinder volume $V = \pi R^2 w \approx 12.566\text{ m}^3$).
* **Scaled Fluid Parameters**: $\rho_f = 300\text{ kg/m}^3$, kinematic viscosity $\nu = 1.5 \times 10^{-5}\text{ m}^2\text{/s}$, planetary gravity $g = 9.81\text{ m/s}^2$ ($F_{\text{grav}} = 961,556.58\text{ N}$).
* **Turbulence Model**: Reynolds-Averaged Navier-Stokes (RAS) with $k$-$\omega$ SST (`simulationType RAS; RASModel kOmegaSST;`).
* **Reynolds Number**: $Re = \frac{V D}{\nu} = \frac{96.34 \times 4.0}{1.5 \times 10^{-5}} = 2.57 \times 10^7$.

### 3.2 Dynamic Force Balance & Terminal Approach
The simulation was integrated to $t = 47.48\text{ s}$:
* Aerodynamic drag: $F_{\text{aero}} = 961,253.14\text{ N}$
* Planetary weight: $F_{\text{grav}} = 961,556.58\text{ N}$
* Force balance ratio: $F_{\text{aero}} / F_{\text{grav}} = 99.968\%$ ($0.032\%$ residual imbalance)
* Frame acceleration: $a_y(t=0) = 9.81\text{ m/s}^2 \to a_y(t=47.48\text{ s}) = 0.003096\text{ m/s}^2$ ($99.968\%$ decay toward equilibrium)

### 3.3 Scope & Limitations of this Check
Integrating surface traction on the cylinder yields $C_D = \frac{F_{\text{aero}}}{\frac{1}{2}\rho_f V^2 A} = 0.3452$. Substituting $C_D = 0.3452$ into the terminal velocity formula yields:
$$V_{t,\text{analytical}} = \sqrt{\frac{2 m g}{\rho_f C_D A}} = \sqrt{\frac{2 \times 961556.58}{300 \times 0.3452 \times 2.0}} = \sqrt{9285.02} = 96.359\text{ m/s} \approx 96.36\text{ m/s}$$
Compared to simulated velocity $V_{\text{sim}} = 96.340\text{ m/s}$, the relative discrepancy is:
$$\epsilon_V = \frac{96.340 - 96.359}{96.359} = \mathbf{-0.02\%}$$

> [!WARNING]
> **This is a frame coupling verification, not an aerodynamic validation.**  
> Because $C_D = 0.3452$ is calculated from the simulation's own output, recovering $V_{\text{sim}} \approx V_t$ to within $0.02\%$ is a mathematical consequence of the force balance $F_{\text{aero}} \approx mg$ ($99.97\%$). It verifies that the accelerating reference frame d'Alembert formulation reaches dynamic equilibrium as $a_{\text{frame}} \to 0$. It does **not** validate the physical drag of a cylinder at $Re = 2.57 \times 10^7$; matching the transcritical cylinder drag band ($0.30 - 0.35$) with 2D $k$-$\omega$ SST URANS without boundary-layer trip or grid refinement is coincidental.

---

## 4. Physical Benchmark: Thin Disks Falling in Air ([Tinklenberg et al. 2023](https://doi.org/10.1017/jfm.2023.209))

The primary experimental benchmark models a thin circular polyester disk ($D = 2.02\text{ mm}$, nominal $h = 50\ \mu\text{m}$, $\chi = 40.4$, $\rho_d = 1380\text{ kg/m}^3$) falling freely in air.

### 4.1 Kinematic Consistency
* **Autorotation Stability**: Spontaneous unidirectional autorotation sustained across 13 completed revolutions ($29,715$ time steps) with planar out-of-plane rate $|\Omega_z| / |\Omega_x| < 0.6\%$.
* **Tumbling Frequency**: Simulated frequency settles at $f_t = 30.52 \pm 0.17\text{ Hz}$, which is $\approx 16\%$ below the experimental mean ($\approx 36.4\text{ Hz}$).
* **Strouhal Number**: Simulated Strouhal number $St = \frac{f_t D}{V_t} = 0.0598$ matches the experimental peak mode ($0.055 - 0.060$).  
* **Dynamic Implication**: Strouhal number matches because both fall velocity ($1.02\text{ m/s}$) and tumbling frequency ($30.5\text{ Hz}$) are low by approximately $16-17\%$. This indicates dynamic similarity (the kinematics scale consistently with speed), pointing to an effective force or mass offset rather than a different tumbling mode.

### 4.2 Terminal Settling Velocity Deficit & ASME V&V 20 Assessment
* **Comparison Error**: $E = V_{\text{sim}} - V_{\text{exp}} = 1.024\text{ m/s} - 1.230\text{ m/s} = -0.206\text{ m/s}$ ($-16.7\%$).
* **Validation Uncertainty**:
  - Direct 6DoF static mesh sensitivity: $u_{\text{mesh}} = |-0.09\%| \approx 0.0009\text{ m/s}$
  - Temporal sensitivity (time-step halving): $u_{\Delta t} = |+0.04\%| \approx 0.0004\text{ m/s}$
  - Convection scheme sensitivity (LUST vs. linearUpwind): $u_{\text{scheme}} = |-0.75\%| \approx 0.0077\text{ m/s}$
  - Numerical uncertainty: $u_{\text{num}} \approx 0.0078\text{ m/s}$ ($0.76\%$)
  - Input parameter uncertainty (mean thickness): $u_{\text{input}} \approx 0.0050\text{ m/s}$
  - Experimental measurement uncertainty: $u_{\text{exp}} = \pm 1.2\%$ ($0.0150\text{ m/s}$)
  - Combined validation uncertainty: $u_{\text{val}} = \sqrt{u_{\text{num}}^2 + u_{\text{input}}^2 + u_{\text{exp}}^2} \approx 0.0176\text{ m/s}$ ($1.4\%$)
* **Verdict**: $|E| = 0.206\text{ m/s} \approx 11.7\, u_{\text{val}} \gg u_{\text{val}}$. The simulation is **not validated** for mean terminal settling velocity.

### 4.3 Why Specimen Thickness Scatter Does Not Explain the Deficit
Tinklenberg et al. (2023, Table 1) report $h = 50 \pm 7.5\ \mu\text{m}$ as the specimen-to-specimen standard deviation across the manufactured glitter batch.
1. **Ensemble Averaging**: Because experimental $V_t = 1.23\text{ m/s}$ is an ensemble mean of thousands of drops, the uncertainty on the mean thickness is the standard error of the mean ($\sigma / \sqrt{N}$), which is small ($< 1\%$). Specimen scatter around the mean shifts the mean velocity by only a fraction of a percent.
2. **Velocity Scaling**: Since $V_t \propto \sqrt{h}$ at fixed drag coefficient, explaining a $-17\%$ speed deficit solely via thickness would require a $-31\%$ drop in mean thickness ($\approx 34.5\ \mu\text{m}$ instead of $50\ \mu\text{m}$). The paper explicitly verified the $50\ \mu\text{m}$ mean using caliper measurements on stacks.
3. **Hypothetical Imperfections**: Any hypothetical edge imperfections or non-planar curvature would add drag and reduce physical descent speed, widening the gap (since the simulation is already slower than experiment); this cannot explain why physical disks fall faster.

---

## 5. Technical Roadmap & Action Plan

| # | Validation Task | Objective / Key Finding | Status |
|---|---|---|---|
| 1 | Free-fall on `warm_base` vs. `warm_wakeFine` | Direct 6DoF test of static mesh refinement on tumbling limit cycle. | **COMPLETED** (Shift: $-0.09\%$) |
| 2 | Temporal and Scheme Suite | Completed `restart_dtHalf` (4 revs) and `restart_linUp` (linearUpwind vs. LUST). | **COMPLETED** ($\Delta t$: $+0.04\%$, Scheme: $-0.75\%$) |
| 3 | Domain & Far-Field Sensitivity | Fixed-disk $Re = 140$ in $1.5\times$ scaled domain ($22.5D \times 37.5D \times 22.5D$, 267k cells): no measurable sensitivity at $t = 80\text{ ms}$ (equal-time comparison, $C_D = 1.1524$ vs. $1.1527$, shift $-0.019\%$). | **COMPLETED** (Shift: $-0.019\%$) |
| 4 | Second Tumbling Realization (3D Perturbation) | Run with non-zero out-of-plane perturbation ($\Omega_y, \Omega_z \neq 0$) to test whether 3D wobbling/gliding breaks strict planar symmetry, reduces peak broadside drag ($C_D \approx 2.17$), and relaxes trajectory angle ($\psi \approx 15.3^\circ \to 10.5^\circ$ mode). | Pending ($\approx 12\ \mathrm{h}$) |
| 5 | Diagnostic Free-Fall Test (Mass/Weight Scaling) | Run extended test ($t \in [0.30, 0.60]\text{ s}$, duration $0.30\text{ s} \approx 4.8\, \tau_{\mathrm{rel}}$ with $\tau_{\mathrm{rel}} \approx V_t/(2g) \approx 0.06\text{ s}$) with weight raised to match effective $h \approx 73\ \mu\text{m}$. Evaluate $V_{\mathrm{eq,est}}$ across final $\ge 3$ revs and track per-revolution drift. | Pending ($\approx 8\ \mathrm{h}$) |
| 6 | Independent Drag Check | Benchmark a sphere or fixed disk across $Re = 100\text{--}200$ against published $C_D(Re)$ literature data (e.g. Roos & Willmarth 1971 for disks; Schiller–Naumann 1933 or Clift et al. 1978 for spheres), not self-generated runs. | Pending ($\approx 8\ \mathrm{h}$) |
| 7 | Three-Mesh GCI Study | Conduct formal grid convergence study on steady $Re = 100$ refining near-body and outer zones. | Pending ($\approx 10\ \mathrm{h}$) |
| 8 | Primary Literature Audit | Audit Shenoy & Kleinstreuer (2008) primary tabulated data to replace unverified $C_D \approx 1.15\text{--}1.25$ secondary citation at $Re = 100$. | Pending (Literature review) |

---

## 6. Formal Verification Documents

The complete 17-page ASME V&V 20 report is maintained in the `validation/` directory:
* PDF Document: [`validation/report.pdf`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/report.pdf)
* LaTeX Source: [`validation/report.tex`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/report.tex)
* Markdown Source: [`validation/report.md`](file:///home/lavender/OpenFoamUbu/solvers/freeBodyFoam/validation/report.md)
