# rodRoller Tutorial

## 1. Overview
The **`rodRoller`** tutorial demonstrates constrained 6DoF simulation of a transverse circular rod (cylinder) falling under gravity in air while being free to spin exclusively about its longitudinal axis ($Z$-axis).

### Kinematic Constraints
- **Translational Freedom**: Locked in $X$ and $Z$, free in vertical $Y$ (`freedom (0 1 0);`).
- **Rotational Freedom**: Locked in pitch ($X$) and yaw ($Y$), free in axial spin ($Z$) (`rotationFreedom (0 0 1);`).
- **Domain Topology**: 2D planar hex grid with a dynamic concentric sliding cylindrical interface (`cyclicAMI`) separating the inner `rotatingZone` from the outer stationary box domain.

## 2. Physical & Dynamic Parameters
- **Rod Diameter**: $D = 0.04\text{ m}$ ($R = 0.02\text{ m}$)
- **Rod Span**: $L = 0.05\text{ m}$
- **Material Density**: Solid Steel ($\rho_s = 7850\text{ kg/m}^3$)
- **Rod Mass**: $m = \rho_s \cdot (\pi R^2 L) \approx 0.4932\text{ kg}$
- **Axial Moment of Inertia**: $I_{zz} = \frac{1}{2} m R^2 = 9.864 \times 10^{-5}\text{ kg}\cdot\text{m}^2$
- **Transverse Inertia**: $I_{xx} = I_{yy} = \frac{1}{12} m (3R^2 + L^2) = 1.52 \times 10^{-4}\text{ kg}\cdot\text{m}^2$
- **Initial Spin**: $\omega_{z,0} = 10\text{ rad/s}$ ($\approx 95.5\text{ RPM}$)
- **Working Fluid**: Incompressible air ($\rho_f = 1.204\text{ kg/m}^3$, $\nu = 1.5 \times 10^{-5}\text{ m}^2/\text{s}$)

## 3. Running the Simulation
```bash
./Allclean
./Allrun
```

To plot the kinematics trajectory:
```bash
python3 plot_motion.py
```
This generates `rodRoller_kinematics.png` showing vertical speed $V_y(t)$, axial spin rate $\omega_z(t)$, aerodynamic drag vs weight, and hydrodynamic reaction torque $M_z(t)$.
