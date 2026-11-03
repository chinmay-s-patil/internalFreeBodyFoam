# Developer & Solver Architecture Guide

Architecture, execution sequence, numerical coupling, and source tree organization for `freeBodyFoam`.

---

## 1. Source Tree Structure

```
src/
├── BCs/                                # Boundary condition libraries
│   └── fallingInletVelocity/           # Dynamic accelerating frame inlet BC (libfallingInletVelocity.so)
│       ├── fallingInletVelocityFvPatchVectorField.C
│       └── fallingInletVelocityFvPatchVectorField.H
├── motion/                             # Dynamic mesh & sliding interface motion functions
│   └── freeBodyRotationalMotion/       # 6DoF coupled solid-body motion solver (libfreeBodyRotationalMotion.so)
│       ├── freeBodyRotationalMotion.C
│       └── freeBodyRotationalMotion.H
├── models/                             # Shared atmospheric and wind models (libfreeFallingModels.so)
│   └── meanFlowModel/                  # Inflow profiles (constant, powerLaw, tabulated)
├── functionObjects/                    # Diagnostic and logging function objects
│   └── fallingMotion/                  # Runtime trajectory and force monitor (libfreeFallingFunctionObjects.so)
├── solvers/                            # Free-falling dynamic solvers
│   ├── shared/                         # Shared predictor-corrector kinematics & logging headers
│   │   ├── createFallingFrameFields.H  # Reads fallingFrameDict and instantiates kinematics
│   │   ├── predictFallingFrameMotion.H # Step predictor for frame kinematics
│   │   └── correctFallingFrameMotion.H # Corrects frame acceleration & updates altitude
│   ├── freeBodyPimpleFoam/             # Incompressible transient solver (pure translation)
│   │   ├── freeBodyPimpleFoam.C
│   │   ├── UEqn.H                      # Momentum equation with -a_frame body force
│   │   └── pEqn.H                      # PIMPLE pressure equation
│   ├── freeBody6DoFPimpleFoam/         # 6DoF coupled PIMPLE solver (translation + rotation)
│   │   ├── freeBody6DoFPimpleFoam.C
│   │   ├── createRotorState.H          # Quaternion & principal inertia initialization
│   │   ├── predictRotorDynamics.H      # Predictor step for rotor attitude qBody
│   │   ├── correctRotorDynamics.H      # Evaluates aerodynamic moments & integrates Euler equations
│   │   ├── UEqn.H
│   │   └── pEqn.H                      # Hydrostatic-reduced p_rgh formulation
│   ├── freeBodyRhoPimpleFoam/          # Pressure-based compressible solver (subsonic/transonic)
│   └── freeBodyRhoCentralFoam/         # Density-based central-upwind solver (supersonic re-entry)
└── utilities/
    └── writeFFDict/                    # Automatic dictionary generation utility
```

---

## 2. Solver Execution Sequence

### 2.1 PIMPLE Time-Step Lifecycle (`freeBody6DoFPimpleFoam.C`)

In each time step $\Delta t$, the solver couples non-inertial fluid dynamics with 6DoF rigid-body motion through a predictor-corrector sequence:

```
[ Start of Time Step t^(n+1) ]
               │
               ▼
1. predictRotorDynamics.H
   Advance attitude quaternion qBody by dt using omegaB^(n)
   Update registered uniform fields attitudeW and attitudeVec
               │
               ▼
2. mesh.controlledUpdate()
   libfreeBodyRotationalMotion reads attitude fields
   Inner rotatingZone mesh rotates; cyclicAMI reconnects face weights
               │
               ▼
3. PIMPLE Outer Corrector Loop
   ├── UEqn.H: Momentum predictor with -a_frame body force
   └── pEqn.H: Pressure-continuity loop solving for p_rgh
               │
               ▼
4. correctFallingFrameMotion.H
   Integrate surface pressure and shear stress -> F_aero
   Update frame acceleration: a_frame = M_trans * (g + F_aero / m)
   Advance velocity: V_frame += a_frame * dt
   Advance altitude: y_altitude += V_frame * dt
               │
               ▼
5. correctRotorDynamics.H
   Integrate surface moments about hubOrigin -> M_aero
   Transform to body principal axes: M_b = R^T * M_aero
   Apply rotation freedom mask: M_b = cmptMultiply(M_b, rotFreedom)
   Solve Euler-Poinsot equations: J_b * d(omega_b)/dt + omega_b x (J_b * omega_b) = M_b
   Correct angular velocity omega_b^(n+1) and attitude quaternion qBody^(n+1)
               │
               ▼
6. FunctionObjects & Console Logging
   Output V_frame, a_frame, forces, Omega [rad/s & RPM], Euler angles [deg]
```

---

## 3. Master Compilation & Cleaning

### 3.1 Compilation
Source your OpenFOAM v2412 environment and build all libraries and solvers:

```bash
# Source OpenFOAM environment
source /usr/lib/openfoam/openfoam2412/etc/bashrc

# Master build script
./Allwmake
```

Compiled shared libraries are installed into `$FOAM_USER_LIBBIN`:
* `libfallingInletVelocity.so`
* `libfreeBodyRotationalMotion.so`
* `libfreeFallingModels.so`
* `libfreeFallingFunctionObjects.so`

Compiled application binaries are installed into `$FOAM_USER_APPBIN`:
* `freeBodyPimpleFoam`
* `freeBody6DoFPimpleFoam`
* `freeBodyRhoPimpleFoam`
* `freeBodyRhoCentralFoam`
* `writeFFDict`

### 3.2 Cleaning
To remove all intermediate compilation object files (`Make/linux*`, `lnInclude/`):

```bash
./Allwclean
```
