# Physical Formulations & Governing Equations

Mathematical models, reference frame transformations, and 6DoF coupled dynamical formulations implemented in `freeBodyFoam`.

---

## 1. Non-Inertial Reference Frame Formulation

Traditional numerical modeling of atmospheric free-fall and projectile trajectories across kilometers of descent requires either dynamic mesh motion (moving mesh domain over vast distances) or adaptive grid translation. `freeBodyFoam` adopts a **body-fixed accelerating reference frame** where the body remains centered in the computational mesh while fluid accelerations and body-force terms model the trajectory.

### 1.1 Coordinate Frame Definitions

* **Inertial World Frame** ($\mathcal{F}_I$):  
  Stationary coordinate system with planetary gravity $\mathbf{g} = (0, -g, 0)$:

  $\mathcal{F}_I = \{\mathbf{O}_I; \hat{\mathbf{e}}_x, \hat{\mathbf{e}}_y, \hat{\mathbf{e}}_z\}$

* **Body-Fixed Reference Frame** ($\mathcal{F}_B$):  
  Translating frame with velocity $\mathbf{V}_{\mathrm{frame}}(t)$ and acceleration $\mathbf{a}_{\mathrm{frame}}(t) = d\mathbf{V}_{\mathrm{frame}}/dt$:

  $\mathcal{F}_B = \{\mathbf{O}_B; \hat{\mathbf{b}}_1, \hat{\mathbf{b}}_2, \hat{\mathbf{b}}_3\}$

### 1.2 Velocity and Acceleration Transformations

For a fluid particle at position $\mathbf{r}$ relative to the body origin $\mathbf{O}_B$:

$$
\mathbf{U}_I = \mathbf{V}_{\mathrm{frame}} + \mathbf{U}
$$

where $\mathbf{U}$ is the fluid velocity relative to the accelerating frame. Taking the material derivative:

$$
\left(\frac{D\mathbf{U}_I}{Dt}\right)_I = \mathbf{a}_{\mathrm{frame}} + \left(\frac{D\mathbf{U}}{Dt}\right)_B + 2\mathbf{\Omega}_{\mathrm{frame}} \times \mathbf{U} + \mathbf{\Omega}_{\mathrm{frame}} \times (\mathbf{\Omega}_{\mathrm{frame}} \times \mathbf{r}) + \frac{d\mathbf{\Omega}_{\mathrm{frame}}}{dt} \times \mathbf{r}
$$

In `freeBodyFoam`, the primary reference frame translates without domain rotation ($\mathbf{\Omega}_{\mathrm{frame}} = \mathbf{0}$), while body tumbling and spin are handled via dynamic mesh sliding interfaces (`cyclicAMI` with `rotatingZone`) or solid-body rotation functions. Consequently, the apparent acceleration simplifies to:

$$
\left(\frac{D\mathbf{U}_I}{Dt}\right)_I = \left(\frac{D\mathbf{U}}{Dt}\right)_B + \mathbf{a}_{\mathrm{frame}}
$$

---

## 2. Fluid Governing Equations

### 2.1 Incompressible Momentum Transport (`freeBodyPimpleFoam`, `freeBody6DoFPimpleFoam`)

Substituting into the Navier-Stokes equations introduces the d'Alembert fictitious body force $\mathbf{S}_U = -\mathbf{a}_{\mathrm{frame}}$:

$$
\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) - \nabla \cdot (\nu_{\mathrm{eff}} \nabla \mathbf{U}) = -\frac{1}{\rho_f} \nabla p + \mathbf{g} - \mathbf{a}_{\mathrm{frame}}
$$

In free-fall where planetary gravity drives the descent, the effective net body acceleration is $\mathbf{g} - \mathbf{a}_{\mathrm{frame}}$.

### 2.2 Hydrostatic-Reduced Pressure Formulation ($p_{\mathrm{rgh}}$) & Buoyancy

When integrating surface forces on an asymmetric or rotating body, the uniform net frame acceleration field $\mathbf{f}_{\mathrm{frame}} = \mathbf{g} - \mathbf{a}_{\mathrm{frame}}$ induces an apparent hydrostatic gradient $\nabla p_{\mathrm{hydro}} = \rho_f \mathbf{f}_{\mathrm{frame}}$. 

`freeBody6DoFPimpleFoam` decomposes total kinematic pressure into hydrostatic-reduced kinematic pressure $p_{\mathrm{rgh}}$:

$$
p_{\mathrm{rgh}} = p - \rho_f (\mathbf{f}_{\mathrm{frame}} \cdot \mathbf{r})
$$

$$
\nabla p = \nabla p_{\mathrm{rgh}} + \rho_f \mathbf{f}_{\mathrm{frame}}
$$

Substituting into the momentum equation yields:

$$
\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) - \nabla \cdot (\nu_{\mathrm{eff}} \nabla \mathbf{U}) = -\frac{1}{\rho_f} \nabla p_{\mathrm{rgh}}
$$

Aerodynamic force $\mathbf{F}_{\mathrm{aero}}$ and torque $\mathbf{M}_{\mathrm{aero}}$ are evaluated by directly integrating $p_{\mathrm{rgh}}$ and viscous shear stress $\boldsymbol{\tau}$ over specified body patches.

> [!NOTE]
> **Physical Implications of $p_{\mathrm{rgh}}$ Force and Moment Integration:**
> 1. **Buoyancy Omission**: By integrating $p_{\mathrm{rgh}}$ instead of total pressure $p$, the Archimedean hydrostatic buoyancy force $\rho_f V_{\mathrm{body}} \mathbf{g}$ and the acceleration-dependent apparent buoyancy $\rho_f V_{\mathrm{body}} \mathbf{a}_{\mathrm{frame}}$ are omitted from $\mathbf{F}_{\mathrm{aero}}$. In atmospheric descent where $\rho_f / \rho_{\mathrm{body}} \ll 1$ (e.g. air at $1.2\text{ kg/m}^3$ vs. polyester at $1380\text{ kg/m}^3$, ratio $\approx 0.09\%$), this buoyancy neglect is physically negligible. In dense liquid simulations (such as `theRodDrop-Denser` with $\rho_f = 300\text{ kg/m}^3$), buoyancy physically accounts for $\approx 3.85\%$ of body weight; the solver remains self-consistent for validating reference frame momentum coupling ($a_{\mathrm{frame}} \to 0$ as $F_{\mathrm{aero}} \to mg$), but this assumes a non-buoyant body.
> 2. **Hydrostatic Moments**: The hydrostatic torque about the rotation center (hub) is $\mathbf{M}_{\mathrm{hydro}} = (\mathbf{r}_{\mathrm{CB}} - \mathbf{r}_{\mathrm{hub}}) \times \rho_f V_{\mathrm{body}} \mathbf{f}_{\mathrm{frame}}$. Eliminating this term via $p_{\mathrm{rgh}}$ is strictly physical when the hub coincides with the body's center of buoyancy ($\mathbf{r}_{\mathrm{hub}} = \mathbf{r}_{\mathrm{CB}}$) or for homogeneous bodies where center of mass equals center of buoyancy. If the hub is offset from the center of buoyancy in a dense fluid, a physical hydrostatic righting/capsizing moment exists and would need explicit tracking.

### 2.3 Compressible Formulations (`freeBodyRhoPimpleFoam`, `freeBodyRhoCentralFoam`)

For transonic and supersonic descent, density varies with pressure and temperature. The continuity, momentum, and total energy equations become:

$$
\frac{\partial \rho}{\partial t} + \nabla \cdot (\rho \mathbf{U}) = 0
$$

$$
\frac{\partial (\rho \mathbf{U})}{\partial t} + \nabla \cdot (\rho \mathbf{U} \mathbf{U}) + \nabla p - \nabla \cdot \boldsymbol{\tau} = \rho (\mathbf{g} - \mathbf{a}_{\mathrm{frame}})
$$

$$
\frac{\partial (\rho E)}{\partial t} + \nabla \cdot (\mathbf{U} (\rho E + p)) - \nabla \cdot (\boldsymbol{\tau} \cdot \mathbf{U}) + \nabla \cdot \mathbf{q} = \rho (\mathbf{g} - \mathbf{a}_{\mathrm{frame}}) \cdot \mathbf{U}
$$

where $E = e + \frac{1}{2}|\mathbf{U}|^2$.

---

## 3. Boundary Conditions: `fallingInletVelocity`

At the domain outer boundaries, fluid enters with velocity equal to the negative of the instantaneous frame velocity $\mathbf{V}_{\mathrm{frame}}(t)$. The custom boundary condition [`fallingInletVelocity`](../src/BCs/fallingInletVelocity/fallingInletVelocityFvPatchVectorField.H) automatically couples with `fallingFrameDict`:

$$
\mathbf{U}_{\mathrm{inlet}}(t) = - \mathbf{V}_{\mathrm{frame}}(t)
$$

### Directional Inflow/Outflow Switching

To prevent artificial reflections of turbulent wake structures at domain boundaries, each boundary face evaluates the directional flux test:

$$
\phi_{\mathrm{test}} = \mathbf{S}_f \cdot \mathbf{U}_{\mathrm{inlet}}
$$

* **Inflow Faces** ($\mathbf{S}_f \cdot \mathbf{U}_{\mathrm{inlet}} < 0$): Dirichlet condition $\mathbf{U} = \mathbf{U}_{\mathrm{inlet}}(t)$.
* **Outflow Faces** ($\mathbf{S}_f \cdot \mathbf{U}_{\mathrm{inlet}} \ge 0$): Zero-gradient Neumann condition $\nabla_{\mathbf{n}} \mathbf{U} = \mathbf{0}$ or pressure-inlet-outlet.

---

## 4. 6DoF Kinematics & Rotor Dynamics

In [`freeBody6DoFPimpleFoam`](../src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C), the body kinematics comprise 3 translational degrees of freedom and 3 rotational degrees of freedom.

### 4.1 Translational Dynamics

The frame acceleration is governed by Newton's second law:

$$
m_{\mathrm{body}} \mathbf{a}_{\mathrm{frame}} = m_{\mathrm{body}} \mathbf{g} + \mathbf{F}_{\mathrm{aero}} + \mathbf{F}_{\mathrm{ext}}
$$

where the aerodynamic force is evaluated over all body boundary patches $\mathcal{S}_{\mathrm{body}}$:

$$
\mathbf{F}_{\mathrm{aero}} = \sum_{\mathrm{patches}} \int_{\mathcal{S}} \left( -p_{\mathrm{rgh}} \mathbf{n} + \boldsymbol{\tau} \cdot \mathbf{n} \right) dS
$$

A directional freedom mask $\mathbf{M}_{\mathrm{trans}} \in \{0, 1\}^3$ constrains allowable motions:

$$
\mathbf{a}_{\mathrm{frame}} = \mathbf{M}_{\mathrm{trans}} \odot \left( \mathbf{g} + \frac{\mathbf{F}_{\mathrm{aero}} + \mathbf{F}_{\mathrm{ext}}}{m_{\mathrm{body}}} \right)
$$

For example, $\mathbf{M}_{\mathrm{trans}} = (0, 1, 0)$ models pure vertical fall, while $\mathbf{M}_{\mathrm{trans}} = (1, 1, 1)$ permits 3D trajectory drift.

### 4.2 Rotational Dynamics (Euler-Poinsot Equations)

In the body-fixed principal axis frame, rotational motion is governed by Euler's equations:

$$
\mathbf{J}_b \frac{d\boldsymbol{\omega}_b}{dt} + \boldsymbol{\omega}_b \times (\mathbf{J}_b \boldsymbol{\omega}_b) = \mathbf{M}_b
$$

where:
* $\mathbf{J}_b = \mathrm{diag}(I_{xx}, I_{yy}, I_{zz})$ is the principal diagonal moment of inertia tensor.
* $\boldsymbol{\omega}_b = (\omega_1, \omega_2, \omega_3)^T$ is the angular velocity in the body frame.
* $\mathbf{M}_b = \mathbf{R}^T \mathbf{M}_{\mathrm{aero}}$ is the total moment projected onto the body principal axes:

$$
\mathbf{M}_{\mathrm{aero}} = \sum_{\mathrm{patches}} \int_{\mathcal{S}} (\mathbf{r} - \mathbf{r}_{\mathrm{hub}}) \times \left( -p_{\mathrm{rgh}} \mathbf{n} + \boldsymbol{\tau} \cdot \mathbf{n} \right) dS
$$

### 4.3 Quaternion Attitude Kinematics

Body attitude is tracked using unit quaternions $\mathbf{q} = (w, x, y, z)$ to avoid gimbal lock:

$$
\frac{d\mathbf{q}}{dt} = \frac{1}{2} \mathbf{q} \otimes \begin{pmatrix} 0 \\ \boldsymbol{\omega}_b \end{pmatrix}
$$

At each time step $\Delta t$, the orientation quaternion is updated using the exponential Rodrigues map:

$$
\Delta \mathbf{q} = \left( \cos\left(\frac{|\boldsymbol{\omega}_b|\Delta t}{2}\right), \; \frac{\boldsymbol{\omega}_b}{|\boldsymbol{\omega}_b|} \sin\left(\frac{|\boldsymbol{\omega}_b|\Delta t}{2}\right) \right)
$$

$$
\mathbf{q}^{n+1} = \frac{\mathbf{q}^n \otimes \Delta \mathbf{q}}{\|\mathbf{q}^n \otimes \Delta \mathbf{q}\|}
$$

The rotation matrix $\mathbf{R}(\mathbf{q}) \in SO(3)$ transforms vectors between body and world frames:

$$
\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2)
\end{bmatrix}
$$

---

## 5. Sliding Mesh Coupling (`cyclicAMI` & `rotatingZone`)

For rotating bodies (`rodRoller`, `steelBallRun`), `freeBodyFoam` utilizes Arbitrary Mesh Interface (`cyclicAMI`) coupling:
1. **Dynamic Mesh Zone (`rotatingZone`)**: Surrounds the body and undergoes rigid-body rotation driven by [`libfreeBodyRotationalMotion.so`](../src/motion/freeBodyRotationalMotion).
2. **Stationary Far-Field Domain**: Discretizes the external computational domain.
3. **AMI Interface**: Interpolates fluxes conservatively across non-conformal faces:

$$
\phi_{\mathrm{target}} = \sum_j w_{ij} \phi_{\mathrm{source}, j}, \quad \sum_j w_{ij} = 1
$$
