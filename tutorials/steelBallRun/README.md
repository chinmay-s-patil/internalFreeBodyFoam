# steelBallRun Tutorial (True 3D 6DoF Solid Sphere)

## 1. Overview
The **`steelBallRun`** tutorial implements a true 3D solid steel sphere (ball) falling through air with **unconstrained 6DoF kinematics** (full 3D translation + full 3D rotation/tumbling).

To make 3D rotation, roll, pitch, and yaw crystal clear in a single glance, the sphere features **32 high-contrast visual surface markers ("golf ball dots")** distributed via an equilateral Fibonacci spherical lattice. In the 3D PyVista animations and tracking dashboard, users can immediately observe the sphere spin up, tumble, precess, and drift in real time.

---

## 2. Physical & Dynamic Parameters
- **Geometry**: True solid sphere
- **Radius**: $R = 25\text{ mm}$ ($D = 50\text{ mm}$)
- **Material**: Solid structural steel ($\rho_s = 7800\text{ kg/m}^3$)
- **Mass**: $m = \frac{4}{3} \pi R^3 \rho_s \approx 0.5105\text{ kg}$
- **Moments of Inertia**:
  $$I_{xx} = I_{yy} = I_{zz} = \frac{2}{5} m R^2 = 1.27627 \times 10^{-4}\text{ kg}\cdot\text{m}^2$$
- **Translational Freedom**: Full 3D (`freedom (1 1 1);`)
- **Rotational Freedom**: Full 3D (`rotationFreedom (1 1 1);`)
- **Initial Spin**: $\vec{\omega}_0 = (10, 5, 15)\text{ rad/s}$ ($\approx 95, 48, 143\text{ RPM}$)
- **Working Fluid**: Incompressible air ($\rho_f = 1.204\text{ kg/m}^3$, $\nu = 1.5 \times 10^{-5}\text{ m}^2/\text{s}$)

---

## 3. Mesh & Solver Architecture
- **Inner Rotating Zone (`rotatingZone`)**: Dynamic spherical mesh enclosing the steel ball ($R = 25\text{ mm}$) out to a spherical sliding interface (`masterAMI`, $R_{\mathrm{AMI}} = 45\text{ mm}$).
- **Outer Domain**: Rectangular computational wind tunnel $[-0.25, 0.25] \times [-0.5, 0.5] \times [-0.25, 0.25]\text{ m}$ with spherical cavity (`slaveAMI`, $R_{\mathrm{AMI}} = 45\text{ mm}$).
- **Interface**: Non-conformal sliding `cyclicAMI` coupling.
- **Solver**: `freeBody6DoFPimpleFoam` with `libfreeBodyRotationalMotion.so`.

---

## 4. Running the Simulation
```bash
./Allclean
./Allrun
```

---

## 5. Live Tracking & 3D Visualizer

### Live Terminal Dashboard
```bash
python3 track_motion.py --watch
```
Displays live telemetry: fall speed $V_y(t)$, 3D acceleration $\vec{a}$, 3D spin rate $\vec{\omega}$ [rad/s & RPM], Euler attitude angles $(\theta_x, \theta_y, \theta_z)$, drag vs gravity balance, and aerodynamic reaction torque $\vec{M}$.

### Publication Plots
```bash
python3 track_motion.py --plot
```
Generates `steelBallRun_kinematics.png` (4 publication-grade panels).

### 3D PyVista Animation & Video Export
```bash
python3 animate_motion.py --video steelBallRun.mp4
```
Renders a high-resolution 3D animation showing the metallic steel ball with its 32 red golf-ball dots rotating through 3D space with live HUD telemetry.
