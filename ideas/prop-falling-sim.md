
# Falling prop / drone on tFFPimpleFoam — concepts, code review, build plan

Scope: `src/solvers/tFFPimpleFoam` (body-force, force-coupled free fall). Legacy solvers/tutorials ignored.
Code below is a **skeleton, not compiled**. API names follow OpenFOAM-v2412 (ESI) as I remember them; check against your tree.

---

## 0. TL;DR

1. `tFFPimpleFoam` is already a dynamic-mesh solver in structure (`createDynamicFvMesh`, `controlledUpdate`, `Uf`, `correctPhi`, `gh`/`p_rgh` fix-up after mesh motion). You do **not** need a separate "pimpleDyMFoam version". You need to add: moments, a rotational state, and a motion function that reads that state.
2. Do **not** use `sixDoFRigidBodyMotion` or overset. Your solver already owns the dynamics (loads, frame acceleration). Integrate rotation yourself and let a tiny custom `solidBodyMotionFunction` rotate the mesh from your state. AMI is only between rotating and static regions, and a sphere maps onto itself under any rotation.
3. Frame stays **translating but non-rotating**. Rotation is mesh motion, so no Coriolis/centrifugal terms; the existing uniform pseudo-force `-a_frame` stays exactly as is.
4. Cases 1 and 2 are the **same solver**. Only a body-frame rotation mask differs.
5. Case 3 puts the frame on the **hub** (not the COG), with `v_hub`, `a_hub` from rigid-body kinematics of the drone. That is the one real structural change.
6. Incompressible `tFFPimpleFoam` is enough. Free-fall speeds and tip speeds are far below Ma 0.3, so skip rhoPimple/rhoCentral and the Mach switch.

---

## 1. Review of `tFFPimpleFoam` (what exists, what is missing)

### Already there and reusable as is

- Registered uniform state fields (`frameVelocity`, `frameAcceleration`, `relativeVelocity`, forces, …). The BC and any new motion function read these via the registry. Same pattern works for attitude.
- Force integration over `patches` from `p_rgh*rho*Sf` and `devReff & Sf*rho` (sign convention is correct: `Sf` points into the body, `devReff` already carries the minus sign).
- `p_rgh` formulation, so the net force excludes the buoyancy-like term (`rho*V*a`). Negligible for a prop.
- After `mesh.changing()`: `gh`, `ghf` recomputed and `p_rgh += gh_old - gh`. Needed once the mesh rotates. Already in.
- `meanFlow` (constant/powerLaw/tabulated) feeds `relativeVelocity = Umean - frameVelocity`, so wind already enters through the inlet BC.

### Gaps

| #  | Item                                                                                                        | Consequence                                                                                                                                                 |
| -- | ----------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| G1 | No moments, no rotational state                                                                             | Nothing can tumble or spin.                                                                                                                                 |
| G2 | No mesh motion driven by solver state (`dynamicMeshDict` is `staticFvMesh` in the tutorial)             | Need a motion function reading the registry.                                                                                                                |
| G3 | `freedom` default `(0 1 0)`                                                                             | Wind drift needs`(1 1 1)` (or at least x/z free).                                                                                                         |
| G4 | Loads from step n fields → explicit Euler for n+1, once per time step                                      | Loosely coupled. Fine for the heavy bodies you ran so far;**not guaranteed for a light prop** (added mass, see §1.1).                                |
| G5 | `updateFallingFrameMotion.H` mixes physics, wind model, diagnostics, file I/O, and uses `static` locals | If you move the force part into the PIMPLE outer loop (G4 fix), the statics (history list, file ptr) would be appended per outer iteration. Split it first. |
| G6 | Terminal-velocity detector uses `                                                                           | a_frame                                                                                                                                                     |

### 1.1 Added-mass check for a light prop (estimate, verify with your numbers)

Thin disk moving broadside: `m_a ≈ (8/3) ρ a³`. For a = 0.1 m, ρ = 1.2 kg/m³ → ~3 g. A 8" prop is ~10–15 g, so `m_a/m ≈ 0.2–0.3`. Probably stable with the explicit coupling, but close enough that you want the option of outer-corrector sub-iteration with under-relaxation (see §3.5). Rotational added inertia is of similar relative size. Heavy drone body (case 3) is a non-issue.

---

## 2. Architecture

```
tFF6DoFPimpleFoam  (copy of tFFPimpleFoam)
 ├─ updateAeroLoads.H        force + MOMENT about hub from CFD patches
 ├─ updateTranslation.H      existing frame accel/vel integration (cases 1,2) or hub kinematics (case 3)
 ├─ updateAttitude.H         Euler eqs + quaternion update (cases 1,2) / drone state (case 3)
 └─ registers: attitudeW, attitudeVec, spinAngle (uniform fields, same pattern as frameVelocity)

libtFFMotion.so
 └─ registryNested : solidBodyMotionFunction
        reads attitudeW/attitudeVec (+ spinAngle) from mesh registry → septernion
```

Per-step order (keep it; it matches the existing flow):
`loads(n)` → integrate state to n+1 → write state to registry → `mesh.controlledUpdate()` reads it → AMI updates → PIMPLE.

---

## 3. Building blocks

### 3.1 Moment about the hub (extend the existing force loop)

Hub = mesh rotation centre = frame origin (so `Cf` is already relative to the right point if the mesh origin is at the hub; otherwise subtract `hubOrigin`).

```cpp
vector mAero = vector::zero;
forAll(patchNames, i)
{
    const label patchi = mesh.boundaryMesh().findPatchID(patchNames[i]);
    if (patchi == -1) continue;

    const fvPatchScalarField&     p_p   = p_rgh.boundaryField()[patchi];
    const fvPatchSymmTensorField& tau_p = devTau.boundaryField()[patchi];
    const vectorField&            Sf    = mesh.boundary()[patchi].Sf();
    const vectorField&            Cf    = mesh.boundary()[patchi].Cf();   // current (moved) centres

    forAll(p_p, facei)
    {
        const vector dFp = p_p[facei]*Sf[facei]*rhoAtmosphere.value();
        const vector dFv = (tau_p[facei] & Sf[facei])*rhoAtmosphere.value();
        fPress += dFp;
        fVisc  += dFv;
        mAero  += (Cf[facei] - hubOrigin) ^ (dFp + dFv);
    }
}
reduce(mAero, sumOp<vector>());
aeroMoment.value() = mAero;      // register it like aeroForce
```

### 3.2 Rotational state and dict additions

`constant/fallingFrameDict`:

```
rotor
{
    hubOrigin         (0 0 0);     // mesh coords, = rotation centre
    inertiaPrincipal  (Ixx Iyy Izz);   // kg m^2, principal axes = body axes
    rotationFreedom   (0 1 0);     // body-frame mask: case 1 = spin axis only, case 2 = (1 1 1)
    initialOmega      (0 0 0);     // body frame, rad/s
    initialAxis       (1 0 0);     // small seed tilt, see note
    initialAngleDeg   2;
}
freedom (1 1 1);                   // translation mask, let the wind push it
```

Note: a perfectly axisymmetric prop dropped perfectly aligned will not wobble in an exact simulation. Seed a small tilt and/or ω, otherwise you wait for numerical noise.

`createRotorState.H`:

```cpp
quaternion qBody(quaternion::I);                 // body -> world (apply initial tilt here)
vector     omegaB(rotor.get<vector>("initialOmega"));
const vector Ib(rotor.get<vector>("inertiaPrincipal"));
const vector rotFreedom(rotor.get<vector>("rotationFreedom"));
const vector hubOrigin(rotor.get<vector>("hubOrigin"));

uniformDimensionedScalarField attitudeW  (IOobject("attitudeW",  runTime.timeName(), "uniform", mesh,
    IOobject::READ_IF_PRESENT, IOobject::AUTO_WRITE), dimensionedScalar("attitudeW", dimless, qBody.w()));
uniformDimensionedVectorField attitudeVec(IOobject("attitudeVec", runTime.timeName(), "uniform", mesh,
    IOobject::READ_IF_PRESENT, IOobject::AUTO_WRITE), dimensionedVector("attitudeVec", dimless, qBody.v()));
```

(Restart: if `READ_IF_PRESENT` finds the files, rebuild `qBody` from them instead of identity. Same for `omegaB`.)

### 3.3 Attitude update (cases 1, 2)

Euler's equations in the body frame, free of any sixDoF dependency. Gyroscopic coupling comes from the `ω × Iω` term, so wobble/precession/gyro stabilisation appear without extra code. **Izz must be realistic** or this case tells you nothing.

```cpp
// moment: world -> body
const tensor R = qBody.R();                       // convention check: see §6 test T2
vector Mb = R.T() & aeroMoment.value();
Mb = cmptMultiply(Mb, rotFreedom);

auto domega = [&](const vector& w) -> vector
{
    return cmptDivide(Mb - (w ^ cmptMultiply(Ib, w)), Ib);
};

const scalar dt = runTime.deltaTValue();
const vector k1 = domega(omegaB);
const vector k2 = domega(omegaB + 0.5*dt*k1);
const vector k3 = domega(omegaB + 0.5*dt*k2);
const vector k4 = domega(omegaB + dt*k3);
omegaB += (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
omegaB  = cmptMultiply(omegaB, rotFreedom);

const scalar wMag = mag(omegaB);
if (wMag > SMALL)
{
    qBody = qBody*quaternion(omegaB/wMag, wMag*dt);   // exact for constant omegaB over the step
    qBody.normalise();
}
attitudeW.value()   = qBody.w();
attitudeVec.value() = qBody.v();
```

RK4 only fixes the gyroscopic term's energy drift. The CFD loads stay first-order lagged.

### 3.4 Motion function (copy `rotatingMotion` as the template)

Template: `src/dynamicMesh/motionSolvers/displacement/solidBody/solidBodyMotionFunctions/rotatingMotion/`.

```cpp
Foam::septernion Foam::solidBodyMotionFunctions::registryNested::transformation() const
{
    const polyMesh& mesh = time_.lookupObject<polyMesh>(polyMesh::defaultRegion);

    // may be called before the solver has registered its fields
    if (!mesh.foundObject<uniformDimensionedScalarField>("attitudeW"))
    {
        return septernion::I;
    }

    const quaternion qAtt
    (
        mesh.lookupObject<uniformDimensionedScalarField>("attitudeW").value(),
        mesh.lookupObject<uniformDimensionedVectorField>("attitudeVec").value()
    );

    quaternion q = qAtt;
    if (withSpin_)    // case 3 IP zone only
    {
        const scalar th =
            mesh.lookupObject<uniformDimensionedScalarField>("spinAngle").value();
        q = qAtt*quaternion(spinAxisLocal_, th);   // spin in body frame first, attitude on top
    }

    return septernion(-origin_)*q*septernion(origin_);
}
```

Reads `origin`, `withSpin`, `spinAxis` from the coeffs dict.

### 3.5 `dynamicMeshDict`

Case 1/2 (single inner zone):

```
libs            ("libtFFMotion.so");
dynamicFvMesh   dynamicMotionSolverFvMesh;
motionSolver    solidBody;
cellZone        inner;                       // sphere minus prop
solidBodyMotionFunction registryNested;
origin          (0 0 0);
withSpin        false;
```

Case 3 (two zones, IP inherits IAS rotation inside its own transform):

```
motionSolver    multiSolidBody;
IAS { solidBodyMotionFunction registryNested; origin (0 0 0); withSpin false; }
IP  { solidBodyMotionFunction registryNested; origin (0 0 0); withSpin true; spinAxis (0 1 0); }
```

(Layout of `multiSolidBody` coeffs differs between versions; zone names are the keys. AMI patches are separate faces, so IAS/IP points are not shared across the interface.)

Optional sub-iteration (G4): set `nOuterCorrectors 3; moveMeshOuterCorrectors yes;` and move **only** the load + state update into the outer loop, restoring `state_n` at the start of each outer iteration and relaxing the loads (`F = F_old + ω(F_new − F_old)`, ω ≈ 0.5). Keep logging/diagnostics outside the loop (G5).

---

## 4. Case 1 — falling prop, spin only (+ drift from wind)

Physics: translation from `m a = m g + F_aero` (existing), spin about its own axis from `Iz Ω̇ = Q_aero` (+ small bearing friction if you want it).

Implementation: `rotationFreedom (0 1 0)` (spin axis in body frame), `freedom (1 1 1)`, `meanFlow` set to the wind you want.

Mesh choices:

- **Disk IP + static**: AMI between a disk and a disk-shaped hole only matches for rotation about the disk axis. Fine here (pure spin), fewer cells.
- **Sphere − prop**: same mesh as case 2, so you build/debug the AMI sphere once. My recommendation: use the sphere from the start; the disk mesh is throwaway.

What this case validates: torque sign, spin ODE, `solidBody` motion function reading the registry, AMI on the rotating interface, `p_rgh` fix-up after motion, and that the loads don't pick up spurious contributions from the rotating interface.

---

## 5. Case 2 — free prop, tumble + spin + gyroscopic effects

Same solver, `rotationFreedom (1 1 1)`, seed tilt/ω (§3.2).

- One rigid body, mesh = sphere − prop, AMI to the static outer cylinder. Spin and tumble are both just body rotation; **no nesting, no spin sub-DOF**.
- Gyroscopic stabilisation emerges from `ω × Iω` with correct `Izz`, `Ixx`. Axisymmetric prop: `Ixx = Iyy`.
- Torque-free precession gives you a clean check of the integrator (§6, T1).
- Sphere AMI: keep interface cell sizes on both sides within ~1:1, uniform interface mesh, avoid inflation layers crossing the interface, check AMI weights/coverage in the first steps.
- Time step: limited by Courant at the interface (tip speed) rather than by the fall speed.

---

## 6. Case 3 — drone coupling (nested IAS/IP, hub-fixed frame)

### 6.1 Why nested here and not before

The prop now has a hinge DOF relative to the body: attitude comes from the drone, spin is relative to it. IAS (sphere) follows attitude. IP (disk) additionally spins about the local axis. Tilting IP without IAS would break its disk interface. IP's transform is the composite `q_att ⊗ q_spin(axis_local, θ)`, which the motion function already does (§3.4).

### 6.2 Frame on the hub, not the COG

The IAS sphere is centred on the hub. If the frame sat on the COG, the sphere would translate inside the AMI hole whenever the drone pitches (`v_hub = v_cog + ω × r`), and the AMI stops matching. So:

```
r_w   = R r_h                                  // hub offset, world
v_hub = v_cog + ω_w × r_w                      // -> frameVelocity   (feeds relativeVelocity / inlet BC)
a_hub = a_cog + α_w × r_w + ω_w × (ω_w × r_w)  // -> frameAcceleration (pseudo-force -a_hub, unchanged form)
```

`ω_w = R ω_b`, `α_w = R ω̇_b`. Take `v_hub` from kinematics rather than integrating `a_hub`, and keep both consistent. Mesh then only **rotates** about the hub: exactly what the AMI sphere handles.

### 6.3 Drone dynamics (body frame, N props, rotors as rigid mass in `I_tot`)

```
m a_cog = m g + R F_tot_b                      (+ fuselage drag model if wanted)
I_tot ω̇ + ω × (I_tot ω + Σ h_i) + Σ I_p Ω̇_i a_i = M_tot_b
h_i = I_p Ω_i a_i                              // relative spin angular momentum, a_i = signed spin axis
I_p (Ω̇_i + a_i·ω̇) = Q_aero,i − τ_motor,i     // τ_motor = 0 for freewheeling props
```

Motor torque is internal to the total system and cancels in the body equation. `M_tot_b` contains `r_i × F_i + Q_aero,i a_i`, so the prop's aero torque reaches the body through the aerodynamic moment, not through the motor.

Loads: CFD gives `F_1, M_1` for the one resolved prop about the hub (world) → body frame via `R^T`. Others:

```
F_i_b = map_i(F_1_b),   M_i_b = map_i(M_1_b)
F_tot_b = Σ F_i_b,      M_tot_b = Σ ( r_i × F_i_b + M_i_b )
```

`map_i` = rotation about the drone's vertical axis by the arm azimuth; counter-rotating props are the **mirror** image, not just a rotation, so spin-axis torque flips sign and in-plane components reflect.

### 6.4 Assumptions you are signing up for (be explicit in the thesis/report)

- **Symmetric inflow**: replicating prop 1's loads is only valid for near-axial relative flow. Under tilt/crosswind each prop sees a different inflow and the sum is wrong.
- **Isolated prop**: no wake interaction with the fuselage, arms or neighbouring props. For a descending drone (vortex-ring-like regimes) that matters.
- Fuselage aerodynamics not in CFD; add a lumped drag/moment model or accept the omission.

### 6.5 What is custom vs reused

Reused: all of tFF's dyM/force/BC infrastructure and `registryNested`. New: `updateAttitude.H` replaced by the §6.3 drone integrator, `spinAngle`/`Ω_i` ODE, hub kinematics feeding `frameVelocity`/`frameAcceleration`. No `rigidBodyDynamics`, no `sixDoFRigidBodyMotion`.

---

## 7. Verification checklist (in this order)

| ID | Test                                                                    | Pass criterion                                                                                 |
| -- | ----------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| T0 | Case 1 mesh/AMI with ρ-scaled loads switched off, prescribed`omegaB` | Flow identical (<1%) to a static-mesh run in the same relative flow; no force noise at the AMI |
| T1 | Torque-free symmetric top, no fluid, seeded tilt + spin                 | Precession rate`≈ (Izz − Ixx)/Ixx · ω_z`;                                                |
| T2 | Rotate a known vector with`qBody.R()` vs the mesh rotation            | Same direction as mesh motion (fixes`R` vs `R.T()` convention)                             |
| T3 | Torque sign: pitched prop in axial flow                                 | Spin accelerates in the physically expected direction                                          |
| T4 | Case 2 sensitivity: seed tilt, dt, interface cell ratio                 | Wobble amplitude/frequency insensitive to these                                                |
| T5 | Case 3 bookkeeping with**stubbed** loads (no CFD)                 | Hover, constant-torque, and gyroscopic cases match hand/ODE solutions                          |
| T6 | Case 3 with CFD, symmetric axial descent                                | Resolved prop loads steady; frame kinematics consistent (`d v_hub/dt = a_hub`)               |

---

## 8. Build order

1. Copy `tFFPimpleFoam` → `tFF6DoFPimpleFoam`; split `updateFallingFrameMotion.H` (G5), add moment (§3.1). Run the existing sphere tutorial: results must not change.
2. Add rotor state + `registryNested`, run T1/T2 without CFD (ρ ≈ 0).
3. Case 1 (sphere mesh, spin-only mask), T0/T3.
4. Case 2 (full mask, seed tilt), T4. Add outer-corrector sub-iteration only if you see coupling oscillations.
5. Case 3: stub loads first (T5), then hub-frame kinematics, then CFD (T6).
