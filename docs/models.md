# Physical Formulations & Governing Equations

Mathematical models, reference frame transformations, and 6DoF coupled dynamical formulations implemented in `freeBodyFoam`.

---

## 1. Non-Inertial Reference Frame Formulation

Traditional numerical modeling of atmospheric free-fall and projectile trajectories across kilometers of descent requires either dynamic mesh motion (moving mesh domain over vast distances) or adaptive grid translation. `freeBodyFoam` adopts a **body-fixed accelerating reference frame** where the body remains centered in the computational mesh while fluid accelerations and body-force terms model the trajectory.

### 1.1 Coordinate Frame Definitions
Let:
* $\mathcal{F}_I = \{\mathbf{O}_I; \hat{\mathbf{e}}_x, \hat{\mathbf{e}}_y, \hat{\mathbf{e}}_z\}$ denote the stationary, inertial world frame (with planetary gravity $\mathbf{g} = (0, -g, 0)$).
* $\mathcal{F}_B = \{\mathbf{O}_B; \hat{\mathbf{b}}_1, \hat{\mathbf{b}}_2, \hat{\mathbf{b}}_3\}$ denote the body-fixed reference frame translating with instantaneous frame velocity $\mathbf{V}_{\text{frame}}(t)$ and frame acceleration $\mathbf{a}_{\text{frame}}(t) = \frac{d\mathbf{V}_{\text{frame}}}{dt}$.

### 1.2 Velocity and Acceleration Transformations
For a fluid particle at position $\mathbf{r}$ relative to the body origin $\mathbf{O}_B$:
$$\mathbf{U}_I = \mathbf{V}_{\text{frame}} + \mathbf{U}$$

where $\mathbf{U}$ is the fluid velocity relative to the accelerating frame. Taking the material derivative:
$$\left(\frac{D\mathbf{U}_I}{Dt}\right)_I = \mathbf{a}_{\text{frame}} + \left(\frac{D\mathbf{U}}{Dt}\right)_B + 2\mathbf{\Omega}_{\text{frame}} \times \mathbf{U} + \mathbf{\Omega}_{\text{frame}} \times (\mathbf{\Omega}_{\text{frame}} \times \mathbf{r}) + \frac{d\mathbf{\Omega}_{\text{frame}}}{dt} \times \mathbf{r}$$

In `freeBodyFoam`, the primary reference frame translates without domain rotation ($\mathbf{\Omega}_{\text{frame}} = \mathbf{0}$), while body tumbling and spin are handled via dynamic mesh sliding interfaces (`cyclicAMI` with `rotatingZone`) or solid-body rotation functions. Consequently, the apparent acceleration simplifies to:
$$\left(\frac{D\mathbf{U}_I}{Dt}\right)_I = \left(\frac{D\mathbf{U}}{Dt}\right)_B + \mathbf{a}_{\text{frame}}$$

---

## 2. Fluid Governing Equations

### 2.1 Incompressible Momentum Transport (`freeBodyPimpleFoam`, `freeBody6DoFPimpleFoam`)
Substituting into the Navier-Stokes equations introduces the d'Alembert fictitious body force $\mathbf{S}_U = -\mathbf{a}_{\text{frame}}$:

$$\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) - \nabla \cdot (\nu_{\text{eff}} \nabla \mathbf{U}) = -\frac{1}{\rho_f} \nabla p + \mathbf{g} - \mathbf{a}_{\text{frame}}$$

In free-fall where planetary gravity drives the descent, the effective net body acceleration is $\mathbf{g} - \mathbf{a}_{\text{frame}}$.

### 2.2 Hydrostatic-Reduced Pressure Formulation ($p_{\text{rgh}}$)
When integrating surface forces on an asymmetric or rotating body, the uniform body acceleration field $\mathbf{f}_{\text{frame}} = \mathbf{g} - \mathbf{a}_{\text{frame}}$ induces an apparent hydrostatic gradient $\nabla p_{\text{hydro}} = \rho_f \mathbf{f}_{\text{frame}}$. 

Integrating the total pressure $p$ around an asymmetric body introduces spurious non-zero moments that cause numerical instability in 6DoF coupling. `freeBody6DoFPimpleFoam` overcomes this by decomposing total kinematic pressure into hydrostatic-reduced kinematic pressure $p_{\text{rgh}}$:

$$p_{\text{rgh}} = p - \rho_f (\mathbf{f}_{\text{frame}} \cdot \mathbf{r})$$

$$\nabla p = \nabla p_{\text{rgh}} + \rho_f \mathbf{f}_{\text{frame}}$$

Substituting into the momentum equation yields:
$$\frac{\partial \mathbf{U}}{\partial t} + \nabla \cdot (\mathbf{U} \mathbf{U}) - \nabla \cdot (\nu_{\text{eff}} \nabla \mathbf{U}) = -\frac{1}{\rho_f} \nabla p_{\text{rgh}}$$

Aerodynamic force and torque evaluation directly integrates $p_{\text{rgh}}$ and viscous shear stress $\boldsymbol{\tau}$, eliminating fictitious hydrostatic contributions.

### 2.3 Compressible Formulations (`freeBodyRhoPimpleFoam`, `freeBodyRhoCentralFoam`)
For transonic and supersonic descent, density varies with pressure and temperature. The continuity, momentum, and total energy equations become:

$$\frac{\partial \rho}{\partial t} + \nabla \cdot (\rho \mathbf{U}) = 0$$

$$\frac{\partial (\rho \mathbf{U})}{\partial t} + \nabla \cdot (\rho \mathbf{U} \mathbf{U}) + \nabla p - \nabla \cdot \boldsymbol{\tau} = \rho (\mathbf{g} - \mathbf{a}_{\text{frame}})$$

$$\frac{\partial (\rho E)}{\partial t} + \nabla \cdot (\mathbf{U} (\rho E + p)) - \nabla \cdot (\boldsymbol{\tau} \cdot \mathbf{U}) + \nabla \cdot \mathbf{q} = \rho (\mathbf{g} - \mathbf{a}_{\text{frame}}) \cdot \mathbf{U}$$

where $E = e + \frac{1}{2}|\mathbf{U}|^2$.

---

## 3. Boundary Conditions: `fallingInletVelocity`

At the domain outer boundaries, fluid enters with velocity equal to the negative of the instantaneous frame velocity $\mathbf{V}_{\text{frame}}(t)$. The custom boundary condition [`fallingInletVelocity`](../src/BCs/fallingInletVelocity/fallingInletVelocityFvPatchVectorField.H) automatically couples with `fallingFrameDict`:

$$\mathbf{U}_{\text{inlet}}(t) = - \mathbf{V}_{\text{frame}}(t)$$

### Directional Inflow/Outflow Switching
To prevent artificial reflections of turbulent wake structures at domain boundaries, each boundary face evaluates the directional flux test:
$$\phi_{\text{test}} = \mathbf{S}_f \cdot \mathbf{U}_{\text{inlet}}$$
* **Inflow Faces** ($\mathbf{S}_f \cdot \mathbf{U}_{\text{inlet}} < 0$): Dirichlet condition $\mathbf{U} = \mathbf{U}_{\text{inlet}}(t)$.
* **Outflow Faces** ($\mathbf{S}_f \cdot \mathbf{U}_{\text{inlet}} \ge 0$): Zero-gradient Neumann condition $\nabla_{\mathbf{n}} \mathbf{U} = \mathbf{0}$ or pressure-inlet-outlet.

---

## 4. 6DoF Kinematics & Rotor Dynamics

In [`freeBody6DoFPimpleFoam`](../src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C), the body kinematics comprise 3 translational degrees of freedom and 3 rotational degrees of freedom.

### 4.1 Translational Dynamics
The frame acceleration is governed by Newton's second law:
$$m_{\text{body}} \mathbf{a}_{\text{frame}} = m_{\text{body}} \mathbf{g} + \mathbf{F}_{\text{aero}} + \mathbf{F}_{\text{ext}}$$

where the aerodynamic force is evaluated over all body boundary patches $\mathcal{S}_{\text{body}}$:
$$\mathbf{F}_{\text{aero}} = \sum_{\text{patches}} \int_{\mathcal{S}} \left( -p_{\text{rgh}} \mathbf{n} + \boldsymbol{\tau} \cdot \mathbf{n} \right) dS$$

A directional freedom mask $\mathbf{M}_{\text{trans}} \in \{0, 1\}^3$ constrains allowable motions:
$$\mathbf{a}_{\text{frame}} = \mathbf{M}_{\text{trans}} \odot \left( \mathbf{g} + \frac{\mathbf{F}_{\text{aero}} + \mathbf{F}_{\text{ext}}}{m_{\text{body}}} \right)$$

For example, $\mathbf{M}_{\text{trans}} = (0, 1, 0)$ models pure vertical fall, while $\mathbf{M}_{\text{trans}} = (1, 1, 1)$ permits 3D trajectory drift.

### 4.2 Rotational Dynamics (Euler-Poinsot Equations)
In the body-fixed principal axis frame, rotational motion is governed by Euler's equations:
$$\mathbf{J}_b \frac{d\mathbf{\omega}_b}{dt} + \mathbf{\omega}_b \times (\mathbf{J}_b \mathbf{\omega}_b) = \mathbf{M}_b$$

where:
* $\mathbf{J}_b = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the principal diagonal moment of inertia tensor.
* $\mathbf{\omega}_b = (\omega_1, \omega_2, \omega_3)^T$ is the angular velocity in the body frame.
* $\mathbf{M}_b = \mathbf{R}^T \mathbf{M}_{\text{aero}}$ is the total moment projected onto the body principal axes:
  $$\mathbf{M}_{\text{aero}} = \sum_{\text{patches}} \int_{\mathcal{S}} (\mathbf{r} - \mathbf{r}_{\text{hub}}) \times \left( -p_{\text{rgh}} \mathbf{n} + \boldsymbol{\tau} \cdot \mathbf{n} \right) dS$$

### 4.3 Quaternion Attitude Kinematics
Body attitude is tracked using unit quaternions $\mathbf{q} = (w, x, y, z)$ to avoid gimbal lock:
$$\frac{d\mathbf{q}}{dt} = \frac{1}{2} \mathbf{q} \otimes \begin{pmatrix} 0 \\ \mathbf{\omega}_b \end{pmatrix}$$

At each time step $\Delta t$, the orientation quaternion is updated using the exponential Rodrigues map:
$$\Delta \mathbf{q} = \left( \cos\left(\frac{|\mathbf{\omega}_b|\Delta t}{2}\right), \; \frac{\mathbf{\omega}_b}{|\mathbf{\omega}_b|} \sin\left(\frac{|\mathbf{\omega}_b|\Delta t}{2}\right) \right)$$
$$\mathbf{q}^{n+1} = \frac{\mathbf{q}^n \otimes \Delta \mathbf{q}}{\|\mathbf{q}^n \otimes \Delta \mathbf{q}\|}$$

The rotation matrix $\mathbf{R}(\mathbf{q}) \in SO(3)$ transforms vectors between body and world frames:
$$\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
1 - 2(y^2 + z^2) & 2(xy - wz) & 2(xz + wy) \\
2(xy + wz) & 1 - 2(x^2 + z^2) & 2(yz - wx) \\
2(xz - wy) & 2(yz + wx) & 1 - 2(x^2 + y^2)
\end{bmatrix}$$

---

## 5. Sliding Mesh Coupling (`cyclicAMI` & `rotatingZone`)

For rotating bodies (`rodRoller`, `steelBallRun`), `freeBodyFoam` utilizes Arbitrary Mesh Interface (`cyclicAMI`) coupling:
1. **Dynamic Mesh Zone (`rotatingZone`)**: Surrounds the body and undergoes rigid-body rotation driven by [`libfreeBodyRotationalMotion.so`](../src/motion/freeBodyRotationalMotion).
2. **Stationary Far-Field Domain**: Discretizes the external wind-tunnel domain.
3. **AMI Interface**: Interpolates fluxes conservatively across non-conformal faces:
   $$\phi_{\text{target}} = \sum_j w_{ij} \phi_{\text{source}, j}, \quad \sum_j w_{ij} = 1$$
