# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-11-05

### Added
- **Solvers Suite:**
  - `freeBodyPimpleFoam`: Incompressible solver with 3D translational dynamic accelerating reference frame.
  - `freeBody6DoFPimpleFoam`: Incompressible solver with full 6DoF coupling (3D translating frame + spherical sliding `cyclicAMI` for 3D rotational dynamics).
  - `freeBodyRhoPimpleFoam`: Pressure-based compressible solver with dynamic non-inertial accelerating frame.
  - `freeBodyRhoCentralFoam`: Density-based central-upwind (Kurganov-Tadmor) transonic/supersonic solver for atmospheric entry.
- **Boundary Conditions & Motion Classes:**
  - `fallingInletVelocity`: Dynamic boundary condition providing counter-inflow compensating for instantaneous accelerating frame velocity $\mathbf{V}_{\mathrm{frame}}(t)$.
  - `freeBodyRotationalMotion`: Dynamic mesh class for spherical sliding mesh interface (`cyclicAMI`) driven by explicit 4th-order Runge-Kutta rigid-body Euler equations and quaternion exponential map attitude integration.
- **Verification & Validation Suite:**
  - Independent 4th-order RK4 angular velocity integrator verification demonstrating asymptotic convergence order $p = 3.9910$.
  - Machine-precision conservation of physical invariants (kinetic energy drift $4.15 \times 10^{-15}$, angular momentum drift $1.90 \times 10^{-15}$) across 13,333 steps in vacuum.
  - ASME V&V 20 temporal discretization grid convergence analysis ($p = 1.566$, fine $\mathrm{GCI} = 0.045\%$).
  - Full 13-revolution continuous autorotation benchmark of falling thin disks in air against Tinklenberg et al. (2023, *J. Fluid Mech.*, 962, A3), with verified dynamic coupling ($2.51 \times 10^{-6}$), vertical acceleration balance ($7.20 \times 10^{-9}\text{ m/s}^2$), and work-energy equivalence ($3.19 \times 10^{-5}$).
  - Complete 6DoF kinematic telemetry datasets (`motion.dat`, ~27 MB) tracked in repository for immediate, local verification reproduction.
- **Tutorials & Documentation:**
  - Comprehensive documentation suite under `docs/` covering mathematical formulation, dictionary syntax, physical models, and ASME V&V 20 validation.
  - Tutorial test cases: `theRodDrop` (high-momentum cylinder descent), `rodRoller` (2D planar rotating cylinder with cyclicAMI), and `steelBallRun` (3D heavy sphere 6DoF descent with visual surface tracking).
  - Formal publication-quality 11-page LaTeX validation report with compiled PDF under `validation/`.
- **Project Metadata:**
  - Added `CITATION.cff`, `.zenodo.json`, and `CHANGELOG.md`.
