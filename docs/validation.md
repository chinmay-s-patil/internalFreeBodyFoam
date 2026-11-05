# Benchmark Verification & Physical Validation

Quantitative verification hierarchy, non-inertial reference frame coupling checks, and physical benchmarks for `freeBodyFoam`.

---

## 1. Validation Status & ASME V&V 20 Overview

> **Status (ASME V&V 20)**: **Code Verification Complete (Incompressible 6DoF) | Validation Partial**.  
> The incompressible 6DoF ODE integrator convergence, unforced invariant drift, non-inertial reference frame momentum coupling, and sliding cyclicAMI continuity are verified. Tumbling autorotation kinematics are dynamically consistent ($St \approx 0.0598$). However, terminal settling velocity against the experimental benchmark of [Tinklenberg et al. (2023)](https://doi.org/10.1017/jfm.2023.209) exhibits an unresolved **$-17\%$ deficit** ($1.02\text{ m/s}$ vs. $1.23\text{ m/s}$, $|E| \approx 11.2\, u_{\text{val}}$) and is **not validated**. Compressible solver implementations are experimental with zero verification cases executed.

### Claims-to-Evidence Summary

| Area | Headline Result | Status |
| :--- | :--- | :--- |
| 6DoF rotational integrator | RK4 order p = 3.991; invariant drift < 1e-14 in vacuum | Verified (ω only) |
| Non-inertial frame coupling | F_aero/mg = 99.97 % at terminal state | Verified (frame only, not drag) |
| Sliding cyclicAMI | Interface flux leak < 4.8e-13 m³/s | Verified |
| Numerical uncertainty | 3-mesh GCI; u_num ≤ 0.93 % (Re = 100) | Bounded |
| Tumbling kinematics | St = 0.0598 (exp. 0.055–0.060) | Consistent |
| Terminal settling velocity | −17 % vs. Tinklenberg et al. (2023) | Not validated |
| Independent drag checks | Broadside disk matches < 1 % (Roos & Willmarth 1971); sphere evaluated | Consistent (Disk) / Offset bounded (Sphere) |
| Spectral sphere benchmark | Case A (G = 144, Re = 185) vs. Uhlmann & Dušek (2014) | Completed (V within 1.0 % of spectral) |
| Axisymmetric wakes | Test cases at Re = 100, 200 vs. Fernandes et al. (2007) | Partial (rectilinear regime only; Re_c and oscillations not tested) |
| Domain sensitivity | Shift ΔC_D = −0.019 % in 1.5× domain (267k cells) | Bounded |
| Literature drag audit | Shenoy & Kleinstreuer (2008) audited (χ = 10) | Audited |
| Compressible solvers | freeBodyRho{Pimple,Central}Foam in source tree | Experimental (unvalidated) |

---

## 2. Code Verification (Rigid-Body Kinematics & Sliding AMI)

### 2.1 6DoF Rotational ODE Integration
The 6DoF rotational dynamics in [`freeBody6DoFPimpleFoam`](../src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C) are integrated sequentially in [`correctRotorDynamics.H`](../src/solvers/freeBody6DoFPimpleFoam/correctRotorDynamics.H):
1. **Angular Velocity ($\boldsymbol{\omega}$)**: Solved using standard explicit 4th-order Runge-Kutta (RK4) on Euler's equations:
   $$\mathbf{J}_b \frac{d\boldsymbol{\omega}_b}{dt} + \boldsymbol{\omega}_b \times (\mathbf{J}_b \boldsymbol{\omega}_b) = \mathbf{M}_b$$
   Algorithmic convergence testing on asymmetric precession test functions demonstrates asymptotic convergence order $p = 3.9910$ (matching theoretical RK4).
2. **Attitude Orientation ($\mathbf{q}$)**: Updated using the exponential map on $SO(3)$ (Rodrigues rotation vector formula) via OpenFOAM quaternion algebra:
   $$\mathbf{q}^{n+1} = \mathbf{q}^n \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_b^{n+1} \Delta t)$$
   *Note: Because $\boldsymbol{\omega}$ varies continuously across a time step, the sequential exponential map update is lower order than the 4th-order ODE step for $\boldsymbol{\omega}$; its formal integration order when $\boldsymbol{\omega}$ varies within a step is unverified.*
3. **Conservation of Invariants in Vacuum**:
   In unforced zero-torque ballistic motion over $13,333$ steps ($\Delta t = 1.5 \times 10^{-5}\text{ s}$, $T = 0.20\text{ s}$):
   - Relative angular momentum drift: $|\Delta \|\mathbf{L}\| / \|\mathbf{L}_0\|| < 1.0 \times 10^{-14}$
   - Relative rotational kinetic energy drift: $|\Delta E / E_0| < 1.0 \times 10^{-14}$
   - Quaternion norm invariance: $|\|\mathbf{q}\| - 1.0| < 5.0 \times 10^{-15}$

   *Methodological note: RK4 is an explicit, non-symplectic integrator and does not conserve energy or angular momentum exactly. The drift below $10^{-14}$ at $\Delta t = 1.5 \times 10^{-5}\text{ s}$ is a truncation error that scales as $\mathcal{O}(\Delta t^4)$ and happens to be below machine epsilon at this tiny step size. RK4 does not guarantee Hamiltonian preservation over long integration horizons under external forcing.*

---

## 3. Frame Coupling Verification: Planar Cylinder (`theRodDrop`)

The case [`tutorials/theRodDrop/theRodDrop-Denser`](../tutorials/theRodDrop/theRodDrop-Denser) verifies the non-inertial accelerating reference frame implementation under high-momentum hydrodynamic drag.

### 3.1 Case Parameters & Numerical Setup
* **Geometry**: 2D planar half-cylinder of radius $R = 2.0\text{ m}$ (diameter $D = 4.0\text{ m}$), unit span $w = 1.0\text{ m}$, frontal half-area $A = R \cdot w = 2.0\text{ m}^2$ (with symmetry plane at $X = 0$).
* **Body Mass**: $m = 98,018\text{ kg}$ (based on full cylinder volume $\pi R^2 w \approx 12.566\text{ m}^3$ at steel density $\rho_s = 7800\text{ kg/m}^3$, simulated in half-symmetry domain).
* **Scaled Fluid Parameters**: $\rho_f = 300\text{ kg/m}^3$, kinematic viscosity $\nu = 1.5 \times 10^{-5}\text{ m}^2\text{/s}$, planetary gravity $g = 9.81\text{ m/s}^2$ ($F_{\text{grav}} = 961,556.58\text{ N}$).
* **Turbulence Model**: Reynolds-Averaged Navier-Stokes (RAS) with $k\text{--}\omega$ SST (`simulationType RAS; RASModel kOmegaSST;`).
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
> **Scope & Limitations (Code Verification Only):**
> 1. **Circularity**: Because $C_D = 0.3452$ is extracted from the simulation's own output, recovering $V_{\text{sim}} \approx V_t$ to within $0.02\%$ is a mathematical verification of the accelerating reference frame momentum coupling ($a_{\text{frame}} \to 0$ as $F_{\text{aero}} \to mg$). It does **not** validate aerodynamic drag coefficients; matching the transcritical cylinder drag band ($0.30 - 0.35$) without boundary-layer tripping or grid sensitivity is coincidental.
> 2. **Buoyancy**: Because $p_{\text{rgh}}$ is integrated for $F_{\text{aero}}$, hydrostatic buoyancy is omitted. In this dense fluid ($\rho_f = 300\text{ kg/m}^3$), buoyancy would physically represent $\rho_f / \rho_s \approx 3.85\%$ of body weight. The setup is self-consistent for non-buoyant frame coupling verification, but does not model buoyant hydrodynamic bodies.

---

## 4. Physical Benchmark: Thin Disks Falling in Air ([Tinklenberg et al. 2023](https://doi.org/10.1017/jfm.2023.209))

The primary experimental benchmark models a thin circular polyester disk ($D = 2.02\text{ mm}$, nominal $h = 50\ \mu\text{m}$, $\chi = 40.4$, $\rho_d = 1380\text{ kg/m}^3$) falling freely in air ($\rho_{\infty} = 1.204\text{ kg/m}^3$).

### 4.1 Kinematic Consistency
* **Autorotation Stability**: Spontaneous unidirectional autorotation sustained across 13 completed revolutions ($29,715$ time steps) with planar out-of-plane rate $|\Omega_z| / |\Omega_x| < 0.6\%$.
* **Tumbling Frequency**: Simulated frequency settles at $f_t = 30.52 \pm 0.17\text{ Hz}$, which is $\approx 16\%$ below the experimental mean ($\approx 36.4\text{ Hz}$).
* **Strouhal Number**: Evaluated with the mean settling speed during the 13-revolution autorotation limit cycle ($V_{\text{tumble}} \approx 1.031\text{ m/s}$):
  $$St = \frac{f_t D}{V_{\text{tumble}}} = \frac{30.52 \times 0.00202}{1.031} \approx 0.0598$$
  (Evaluating with the nominal vertical settling speed $V_{\text{sim}} = 1.024\text{ m/s}$ yields $St \approx 0.0602$. Both match the experimental peak mode $0.055\text{--}0.060$.)
* **Dynamic Implication**: Strouhal number matches because both fall velocity ($1.02\text{--}1.03\text{ m/s}$) and tumbling frequency ($30.5\text{ Hz}$) are low by approximately $16-17\%$. This indicates dynamic kinematic similarity (frequency scales proportionally with velocity), pointing to an effective force or mass offset rather than a different tumbling mode.

### 4.2 Terminal Settling Velocity Deficit & ASME V&V 20 Assessment
* **Comparison Error**: $E = V_{\text{sim}} - V_{\text{exp}} = 1.024\text{ m/s} - 1.230\text{ m/s} = -0.206\text{ m/s}$ ($-16.7\%$).
* **Validation Uncertainty**:
  - Spatial discretization (3-mesh GCI at steady $Re = 100$, near-body zone refined): $u_{\text{mesh}} = 0.55\% \approx 0.0056\text{ m/s}$ (evaluated at $Re = 100$ and applied to free-fall at $Re \approx 137$)
  - Temporal discretization (time-step halving $\Delta t = 15\ \mu\text{s} \to 7.5\ \mu\text{s}$): $u_{\Delta t} = 0.04\% \approx 0.0004\text{ m/s}$
  - Convection scheme sensitivity (LUST vs. linearUpwind): $u_{\text{scheme}} = 0.75\% \approx 0.0077\text{ m/s}$
  - Combined numerical uncertainty: $u_{\text{num}} = \sqrt{0.55^2 + 0.04^2 + 0.75^2}\% = 0.93\% \approx 0.0095\text{ m/s}$
  - Input parameter uncertainty (mean disk thickness standard error): $u_{\text{input}} \approx 0.0050\text{ m/s}$ ($0.41\%$)
  - Experimental measurement uncertainty: $u_{\text{exp}} = \pm 1.2\%$ ($0.0150\text{ m/s}$)
  - Combined validation uncertainty:
    $$u_{\text{val}} = \sqrt{u_{\text{num}}^2 + u_{\text{input}}^2 + u_{\text{exp}}^2} = \sqrt{0.0095^2 + 0.0050^2 + 0.0150^2} \approx 0.0185\text{ m/s}\ (1.50\%)$$
* **Verdict**:
  $$\frac{|E|}{u_{\text{val}}} = \frac{0.206\text{ m/s}}{0.01845\text{ m/s}} \approx 11.2\quad (\approx 11.1\text{ with unrounded components}) \gg 1$$
  The simulation is **not validated** for mean terminal settling velocity.

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
| 3 | Domain & Far-Field Sensitivity | Fixed-disk $Re = 140$ in $1.5\times$ scaled domain ($22.5D \times 37.5D$, 267k cells): shift $-0.019\%$. | **COMPLETED** (Blockage does not explain deficit) |
| 4 | Second Tumbling Realization (3D Perturbation) | 5% $\Omega_z$ perturbation over 8.5 revs: sustained 3D precession ($\bar{\theta}_{\text{out}} = 8.24^\circ$); settling speed shifts $+0.10\%$, $f_t$ by $-1.12\%$. | **COMPLETED** (Planar symmetry bounds sensitivity $\le 0.1\%$) |
| 5 | Diagnostic Free-Fall Test (Mass/Weight Scaling) | Weight raised to effective $h \approx 73\ \mu\text{m}$: concurrently recovers $V = 1.229\text{ m/s}$ and $f = 35.44\text{ Hz}$. | **COMPLETED** (Confirms force/mass calibration sensitivity) |
| 6 | Independent Drag Check | Broadside disk at $Re = 200$: $C_D = 1.106$ matches Roos & Willmarth (1971) within $< 1\%$. Sphere checks at $Re = 100, 200$ show $+27\%$ Cartesian offset. | **COMPLETED** (Disk: $< 1\%$ match; Sphere: $+27\%$ Cartesian offset identified) |
| 7 | Three-Mesh GCI Study | Formal ASME V&V 20 spatial GCI at $Re = 100$: oscillatory convergence ($R = -0.89$), $u_{\text{mesh}} \le 0.55\%$, total $u_{\text{num}} \le 0.93\%$. | **COMPLETED** |
| 8 | Primary Literature Audit | Shenoy & Kleinstreuer (2008) audit: studied aspect ratio $\chi = 10$ ($4\times$ thicker than Tinklenberg $\chi = 40.4$); thicker disks have lower $C_D \approx 1.15\text{--}1.25$. | **COMPLETED** |
| 9 | Spectral Sphere Benchmark (Uhlmann & Dušek 2014) | Quantitative verification against spectral reference code (Jenny et al. 2004, Uhlmann & Dušek 2014) for heavy rigid sphere ($G = 144$). | **COMPLETED** (Settling velocity matches spectral reference within $+1.0\%$, $V = -0.900\text{ m/s}$ vs $-0.909\text{ m/s}$) |
| 10 | Axisymmetric Body Dynamics (Fernandes et al. 2007) | Rectilinear wake, bifurcation threshold ($Re_c$), and trajectory oscillations for thin disks at $Re = 100, 200$. | **PARTIAL** (Rectilinear regime only; $Re_c$ and oscillations not tested) |

---

## 6. Formal Verification Documents & Data Availability

The complete 17-page ASME V&V 20 report is maintained in the `validation/` directory:
* Reproducibility & Data Manifest: [`validation/README.md`](../validation/README.md)
* PDF Document: [`validation/report.pdf`](../validation/report.pdf)
* LaTeX Source: [`validation/report.tex`](../validation/report.tex)
* Markdown Source: [`validation/report.md`](../validation/report.md)
* Archival CFD Datasets (6.4 GB local verification datasets + 25 GB 3D CFD field data): Open-access repository deposit in progress; self-contained packaging scripts provided in [`validation/README.md`](../validation/README.md).

---

## 7. References

* **Clift, R., Grace, J. R., & Weber, M. E.** (1978). *Bubbles, Drops, and Particles*. Academic Press, New York.
* **Fernandes, P. C., Risso, F., Ern, P., & Magnaudet, J.** (2007). "Oscillatory motion and wake instability of freely rising axisymmetric bodies". *Journal of Fluid Mechanics*, 573: 479–502.
* **Jenny, M., Dušek, J., & Bouchet, G.** (2004). "Instabilities and transition of a sphere freely falling or ascending in a fluid". *Journal of Fluid Mechanics*, 508: 201–239.
* **Roos, F. W., & Willmarth, W. W.** (1971). "Some experimental results on sphere and disk drag". *AIAA Journal*, 9(2): 285–291.
* **Schiller, L., & Naumann, A.** (1933). "A drag coefficient correlation". *VDI Zeitung*, 77: 318–320.
* **Shenoy, A. V., & Kleinstreuer, C.** (2008). "Flow over a thin circular disk at low-to-moderate Reynolds numbers". *Journal of Fluid Mechanics*, 605: 253–262.
* **Tinklenberg, A., Guala, M., & Coletti, F.** (2023). "Thin disks falling in air". *Journal of Fluid Mechanics*, 962: A3.
* **Uhlmann, M., & Dušek, J.** (2014). "The motion of a single heavy sphere in ambient fluid: A benchmark for interface-resolved simulation". *International Journal of Multiphase Flow*, 59: 221–243.
* **ASME V&V 20-2009**: *Standard for Verification and Validation in Computational Fluid Dynamics and Heat Transfer*. American Society of Mechanical Engineers, New York.

