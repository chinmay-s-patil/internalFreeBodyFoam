# Tutorials & Demonstration Cases

Comprehensive guide and execution walkthrough for all official `freeBodyFoam` tutorials.

---

## Tutorial Suite Summary

| Tutorial | Regime | Dimensions | Translation | Rotation | Dynamic Mesh | Key Verification Metric |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| [`theRodDrop`](#1-theroddrop) | Transcritical URANS | 2D Planar | Vertical $Y$ | None | `staticFvMesh` | Frame coupling balance ($-0.02\%$ from simulated $C_D$) |
| [`rodRoller`](#2-rodroller) | Laminar / Transient | 2D Planar | Vertical $Y$ | Axial $Z$ | `cyclicAMI` (O-Grid) | Constrained 6DoF torque & spin coupling |
| [`steelBallRun`](#3-steelballrun) | Subsonic 6DoF | True 3D | Full 3D | Full 3D | Spherical `cyclicAMI` | 32-dot Fibonacci golf-ball 3D tumble tracking |

---

## 1. `theRodDrop`

* **Case Path**: [`tutorials/theRodDrop/theRodDrop`](../tutorials/theRodDrop/theRodDrop) & [`tutorials/theRodDrop/theRodDrop-Denser`](../tutorials/theRodDrop/theRodDrop-Denser)
* **Solver**: [`freeBodyPimpleFoam`](../src/solvers/freeBodyPimpleFoam/freeBodyPimpleFoam.C)
* **Geometry**: 2D transverse half-cylinder with symmetry plane at $X = 0$ ($R = 2.0\text{ m}$, unit span $L = 1.0\text{ m}$, frontal half-area $A = R \cdot L = 2.0\text{ m}^2$)
* **Medium**: Scaled dense fluid ($\rho_f = 300.0\text{ kg/m}^3$, $\nu = 1.5 \times 10^{-5}\text{ m}^2/\text{s}$, $g = 9.81\text{ m/s}^2$)
* **Mass**: $m = 98,018.0\text{ kg}$ (based on full cylinder volume $\pi R^2 L = 12.566\text{ m}^3$ at steel density $\rho_s = 7800\text{ kg/m}^3$, simulated in half-symmetry domain)

### Physics & Analytical Benchmark
At terminal velocity, the gravitational force balances the aerodynamic drag:
$$m g = \frac{1}{2} \rho_f V_t^2 C_D A$$
$$V_{t,\text{analytical}} = \sqrt{\frac{2 m g}{\rho_f C_D A}} = \sqrt{\frac{2 \times 98018 \times 9.81}{300 \times 0.3452 \times 2.0}} = 96.359\text{ m/s} \approx 96.36\text{ m/s}$$
where simulated $C_D = 0.3452$ under $k\text{--}\omega$ SST URANS at transcritical Reynolds number $Re = 2.57 \times 10^7$.

### Results
* **Simulated Terminal Velocity**: $V_{\text{sim}} = 96.340\text{ m/s}$ (Relative discrepancy: **$-0.02\%$**).
* **Force Equilibrium**: Aerodynamic drag balances weight to within $99.968\%$ ($F_{\text{aero}}/mg = 99.97\%$, residual decay $\frac{dV}{dt} = 0.0031\text{ m/s}^2$).

> [!WARNING]
> **Scope & Limitations (Code Verification Only):**
> 1. **Circularity**: Because $C_D = 0.3452$ is extracted from the simulation's own output, recovering $V_{\text{sim}} \approx V_t$ to within $0.02\%$ is a mathematical verification of the accelerating reference frame momentum coupling ($a_{\text{frame}} \to 0$ as $F_{\text{aero}} \to mg$). It does **not** validate aerodynamic drag prediction.
> 2. **Buoyancy**: Because $p_{\text{rgh}}$ is integrated for $F_{\text{aero}}$, hydrostatic buoyancy is omitted. In this dense fluid ($\rho_f = 300\text{ kg/m}^3$), buoyancy would physically represent $\rho_f / \rho_s \approx 3.85\%$ of body weight. The setup is self-consistent for non-buoyant frame coupling verification, but does not model buoyant hydrodynamic bodies.

<p align="center">
  <img src="images/theRodDrop_kinematics.png" alt="theRodDrop Kinematics" width="70%"/>
</p>

### Execution
```bash
cd tutorials/theRodDrop/theRodDrop
./Allclean
./Allrun
python3 plot_motion.py
```

---

## 2. `rodRoller`

* **Case Path**: [`tutorials/rodRoller`](../tutorials/rodRoller)
* **Solver**: [`freeBody6DoFPimpleFoam`](../src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C)
* **Geometry**: Solid steel transverse rod ($D = 0.04\text{ m}$, $L = 0.05\text{ m}$, mass $m = 0.4932\text{ kg}$)
* **Kinematics**:
  - Vertical translation along $Y$ (`freedom (0 1 0);`).
  - Pure axial rotation about $Z$ (`rotationFreedom (0 0 1);`).
  - Initial spin: $\omega_{z,0} = 10\text{ rad/s}$ ($95.5\text{ RPM}$).
  - Axial moment of inertia: $I_{zz} = \frac{1}{2} m R^2 = 9.864 \times 10^{-5}\text{ kg}\cdot\text{m}^2$.

### Mesh Topology
A 2D concentric cylindrical O-grid generated via [`system/generate_mesh.py`](../tutorials/rodRoller/system/generate_mesh.py) features a circular sliding interface (`cyclicAMI`) separating the inner rotating zone (`rotatingZone`, 1,920 cells) from the outer rectangular domain (3,200 cells). Block vertices follow radial-first right-handed indexing to guarantee positive Jacobian determinants.

![rodRoller Kinematics](images/rodRoller_kinematics.png)

### Execution
```bash
cd tutorials/rodRoller
./Allclean
./Allrun
python3 plot_motion.py
```

---

## 3. `steelBallRun`

* **Case Path**: [`tutorials/steelBallRun`](../tutorials/steelBallRun)
* **Solver**: [`freeBody6DoFPimpleFoam`](../src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C)
* **Geometry**: True solid steel sphere ($R = 25\text{ mm}$, $D = 50\text{ mm}$, $\rho_s = 7800\text{ kg/m}^3$)
* **Kinematics**:
  - Full 3D translation (`freedom (1 1 1);`).
  - Full 3D rotation / tumbling (`rotationFreedom (1 1 1);`).
  - Initial spin: $\vec{\omega}_0 = (10, 5, 15)\text{ rad/s}$ ($95, 48, 143\text{ RPM}$).
  - Solid sphere isotropic inertia: $I = \frac{2}{5} m R^2 = 1.27627 \times 10^{-4}\text{ kg}\cdot\text{m}^2$.

### Mesh & Golf-Ball Marker System
- **Mesh**: 36,280 cells comprising an inner rotating spherical mesh enclosed by a spherical `cyclicAMI` interface ($R_{\text{AMI}} = 45\text{ mm}$) and an external computational wind tunnel.
- **Visual Markers**: 32 high-contrast crimson surface dots distributed via Fibonacci spherical spiral ([`golf_dots.json`](../tutorials/steelBallRun/golf_dots.json)). Rotating with the body's instantaneous quaternion $\mathbf{R}(t)$, they make 3D roll, pitch, and yaw crystal clear in every frame.

![steelBallRun Kinematics](images/steelBallRun_kinematics.png)

### Tools & Visualizers
* **Live Dashboard**: `python3 track_motion.py --watch` (CLI real-time telemetry).
* **Publication Plots**: `python3 track_motion.py --plot` (generates 4-panel trajectory figure).
* **3D PyVista Animation**: `python3 animate_motion.py --video steelBallRun.mp4` (generates high-definition MP4 video with telemetry HUD).

### Execution
```bash
cd tutorials/steelBallRun
./Allclean
./Allrun
```
