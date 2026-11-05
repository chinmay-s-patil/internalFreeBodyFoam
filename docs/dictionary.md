# Dictionary Reference: `fallingFrameDict` & `dynamicMeshDict`

Configuration keywords, subdictionaries, physical units, and setup templates for `freeBodyFoam`.

---

## 1. File Placement

In a `freeBodyFoam` case, the kinematics and reference frame properties reside in the `constant/` directory:

```
case/
├── 0/
│   ├── U                   # Flow velocity with fallingInletVelocity
│   └── p_rgh               # Hydrostatic-reduced pressure
├── constant/
│   ├── dynamicMeshDict     # Mesh motion solver & sliding cyclicAMI controls
│   ├── fallingFrameDict    # Frame acceleration, 6DoF rotor mass, inertia, freedoms
│   ├── transportProperties # Fluid viscosity nu
│   └── turbulenceProperties# Simulation type (laminar / RAS / LES)
└── system/
    ├── controlDict         # Solver application & functionObjects
    ├── fvSchemes           # Discretization schemes
    └── fvSolution          # Linear solvers & PIMPLE controls
```

---

## 2. Keyword Reference: `fallingFrameDict`

Located at `constant/fallingFrameDict`.

### 2.1 Translational Reference Frame Keywords

| Keyword | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `u0` | `vector` | `(0 0 0)` | Initial velocity vector of falling body in world frame [m/s]. |
| `acceleration` | `vector` | `(0 -9.81 0)` | Planetary gravity vector $\mathbf{g}$ [m/s$^2$]. |
| `patches` | `wordList` | *(required)* | Boundary patch names of the falling body over which forces/moments are integrated. |
| `mass` | `scalar` | *(required)* | Total mass of the falling body $m_{\text{body}}$ [kg]. |
| `freedom` | `vector` | `(0 1 0)` | Directional translational freedom mask $(M_x, M_y, M_z) \in \{0, 1\}^3$. Set `(1 1 1)` for unrestricted 3D drift. |
| `rhoInf` | `scalar` | `1.204` | Fluid reference density $\rho_f$ for aerodynamic force normalization [kg/m$^3$]. |
| `densityModel` | `word` | `rhoInf` | Density evaluation model (`rhoInf` or `thermo`). |
| `gravityModel` | `word` | `constant` | Gravity model (`constant` or `tabulated`). |
| `groundInterrupt` | `Switch` | `off` | Terminates simulation if body altitude reaches ground level ($y \le y_{\text{ground}}$). |

### 2.2 Rotational Subdictionary: `rotor`

The optional `rotor` subdictionary enables 6DoF rotational kinetics in [`freeBody6DoFPimpleFoam`](../src/solvers/freeBody6DoFPimpleFoam/freeBody6DoFPimpleFoam.C):

| Keyword | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `hubOrigin` | `point` | `(0 0 0)` | Center of rotation / center of mass $\mathbf{r}_{\text{hub}}$ in body coordinates [m]. |
| `inertiaPrincipal` | `vector` | *(required)* | Principal moments of inertia $(I_{xx}, I_{yy}, I_{zz})$ [kg$\cdot$m$^2$]. |
| `rotationFreedom` | `vector` | `(1 1 1)` | 3D rotational freedom mask $(R_{\text{roll}}, R_{\text{pitch}}, R_{\text{yaw}}) \in \{0, 1\}^3$. |
| `initialOmega` | `vector` | `(0 0 0)` | Initial angular velocity vector $\mathbf{\omega}_0$ in body principal frame [rad/s]. |
| `initialAxis` | `vector` | `(0 0 1)` | Initial rotation axis for attitude angle specification. |
| `initialAngleDeg` | `scalar` | `0.0` | Initial attitude angle about `initialAxis` [degrees]. |
| `damping` | `vector` | `(0 0 0)` | Linear rotational aerodynamic damping coefficients $[\text{N}\cdot\text{m}\cdot\text{s/rad}]$. |
| `report` | `Switch` | `true` | Outputs angular velocity (rad/s & RPM), Euler angles, and moments to console at each time step. |

### 2.3 Diagnostic Logging Subdictionary: `print`

Controls console diagnostics written at each time step:

```foam
print
{
    frameVel        on;     // Output instantaneous frame velocity V_frame
    frameAcc        on;     // Output instantaneous frame acceleration a_frame
    altitude        on;     // Output integrated trajectory altitude
    aeroForce       on;     // Output integrated aerodynamic drag/lift force
    gravityForce    on;     // Output planetary gravitational weight (m*g)
    netForce        on;     // Output net dynamic vector force
}
```

---

## 3. Keyword Reference: `dynamicMeshDict`

Located at `constant/dynamicMeshDict`. Controls dynamic mesh motion and rotating zones.

| Keyword | Type | Example | Description |
| :--- | :--- | :--- | :--- |
| `dynamicFvMesh` | `word` | `dynamicMotionSolverFvMesh` | Dynamic mesh class (`staticFvMesh` for non-rotating bodies; `dynamicMotionSolverFvMesh` for rotating zones). |
| `motionSolver` | `word` | `solidBody` | Mesh motion solver algorithm. |
| `cellZone` | `word` | `rotatingZone` | Target `cellZone` undergoing rigid-body rotation. |
| `solidBodyMotionFunction` | `word` | `freeBodyRotationalMotion` | Dynamic motion provider coupled with the solver 6DoF quaternion state. |
| `origin` | `point` | `(0 0 0)` | Pivot center of rotation [m]. |

---

## 4. Minimal Working Templates

### 4.1 Template 1: Static Mesh Vertical Fall (`theRodDrop`)

For symmetric or non-rotating bodies falling along a single axis without dynamic mesh deformation:

```foam
// constant/fallingFrameDict
u0                  ( 0 0 0 );
acceleration        ( 0 -9.81 0 );
patches             ( cylinder );
mass                1000.0;
freedom             ( 0 1 0 );
rhoInf              1.204;
densityModel        rhoInf;
gravityModel        constant;
groundInterrupt     off;

// constant/dynamicMeshDict
dynamicFvMesh       staticFvMesh;
```

### 4.2 Template 2: 2D Transverse Cylinder with Axial Spin (`rodRoller`)

Constrained 6DoF with vertical $Y$-fall and axial $Z$-rotation using `cyclicAMI`:

```foam
// constant/fallingFrameDict
u0                  ( 0 0 0 );
acceleration        ( 0 -9.81 0 );
patches             ( rod );
mass                0.4932;
freedom             ( 0 1 0 );      // Fall constrained to Y
rhoInf              1.204;
densityModel        rhoInf;

rotor
{
    hubOrigin         (0 0 0);
    inertiaPrincipal  (1.52e-4 1.52e-4 9.864e-5);
    rotationFreedom   (0 0 1);      // Rotation constrained to Z
    initialOmega      (0 0 10.0);   // Initial spin 10 rad/s
    report            true;
}

// constant/dynamicMeshDict
libs                    ("libfreeBodyRotationalMotion.so");
dynamicFvMesh           dynamicMotionSolverFvMesh;
motionSolver            solidBody;
cellZone                rotatingZone;
solidBodyMotionFunction freeBodyRotationalMotion;
origin                  (0 0 0);
```

### 4.3 Template 3: True 3D Unrestricted 6DoF Sphere (`steelBallRun`)

Full 3D translation + full 3D tumble/rotation with spherical `cyclicAMI`:

```foam
// constant/fallingFrameDict
u0                  ( 0 0 0 );
acceleration        ( 0 -9.81 0 );
patches             ( sphere );
mass                0.510509;
freedom             ( 1 1 1 );      // Full 3D translation

rhoInf              1.204;
densityModel        rhoInf;

rotor
{
    hubOrigin         (0 0 0);
    inertiaPrincipal  (1.27627e-4 1.27627e-4 1.27627e-4);
    rotationFreedom   (1 1 1);      // Full 3D rotation
    initialOmega      (10.0 5.0 15.0); // 3-axis spin
    report            true;
}

// constant/dynamicMeshDict
libs                    ("libfreeBodyRotationalMotion.so");
dynamicFvMesh           dynamicMotionSolverFvMesh;
motionSolver            solidBody;
cellZone                rotatingZone;
solidBodyMotionFunction freeBodyRotationalMotion;
origin                  (0 0 0);
```

---

## 5. Boundary Condition Reference: `0/U` & `0/p_rgh`

```foam
// 0/U
boundaryField
{
    bodyWall
    {
        type            movingWallVelocity;  // For rotating zones
        value           uniform (0 0 0);
    }
    inlet
    {
        type            fallingInletVelocity;// Couples with V_frame
        value           uniform (0 0 0);
    }
    outlet
    {
        type            pressureInletOutletVelocity;
        value           $internalField;
    }
    masterAMI { type cyclicAMI; value $internalField; }
    slaveAMI  { type cyclicAMI; value $internalField; }
    farField  { type slip; }
}

// 0/p_rgh
boundaryField
{
    bodyWall  { type fixedFluxPressure; value uniform 0; }
    inlet     { type fixedFluxPressure; value uniform 0; }
    outlet    { type fixedValue; value uniform 0; }
    masterAMI { type cyclicAMI; value $internalField; }
    slaveAMI  { type cyclicAMI; value $internalField; }
    farField  { type zeroGradient; }
}
```
