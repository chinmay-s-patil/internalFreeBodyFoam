#!/usr/bin/env python3
"""
check_run_consistency.py

Post-run consistency checks for freeBody6DoFPimpleFoam motion logs (tumbleMotion + fallingMotion).
Checks that the logged states obey the rigid-body equations the solver claims to integrate,
and extrapolates cycle-averaged quantities to their asymptote (so a still-relaxing run is not
reported as a plateau).

Usage:
    python3 check_run_consistency.py <tumbleMotion/motion.dat> <fallingMotion/motion.dat> \
        [--mass 2.2113e-7] [--Ixx 5.6436e-14] [--g 9.81]
"""

import argparse
import numpy as np


def cycle_mean(a, t, t0, t1):
    """Time-weighted trapezoidal mean of a(t) over [t0, t1]."""
    m = (t > t0) & (t < t1)
    tt = np.concatenate(([t0], t[m], [t1]))
    aa = np.concatenate(([np.interp(t0, t, a)], a[m], [np.interp(t1, t, a)]))
    trapz = np.trapezoid if hasattr(np, "trapezoid") else np.trapz
    return trapz(aa, tt) / (t1 - t0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rot", nargs="?", default="validation/thinDisk2mm_tumbling/case/postProcessing/tumbleMotion/motion.dat")
    ap.add_argument("trans", nargs="?", default="validation/thinDisk2mm_tumbling/case/postProcessing/fallingMotion/motion.dat")
    ap.add_argument("--mass", type=float, default=2.2113e-7)  # kg
    ap.add_argument("--Ixx", type=float, default=5.6436e-14)  # kg m^2, pitch axis
    ap.add_argument("--g", type=float, default=9.81)          # m/s^2
    a = ap.parse_args()

    r = np.loadtxt(a.rot, comments="#")
    f = np.loadtxt(a.trans)
    assert np.allclose(r[:, 0], f[:, 0], rtol=1e-5), "time grids differ"
    t, dt = r[:, 0], np.diff(r[:, 0])
    m, g, W = a.mass, a.g, a.mass * a.g
    wx, Mx = r[:, 1], r[:, 7]
    Vx, Vy, Vz = f[:, 2], f[:, 3], f[:, 4]
    Ay, Fx, Fy, Fz = f[:, 6], f[:, 26], f[:, 27], f[:, 28]

    def report(name, val, tol):
        print(f"  [{'PASS' if val < tol else 'FAIL'}] {name}: {val:.3e} (tol {tol:.0e})")

    print("Rigid-body consistency of logged states")
    dw = np.diff(wx) / dt  # (w_{n+1} - w_n)/dt, should equal Mx_{n+1}/Ixx for planar motion
    report("I*dw/dt vs Mx, rel. to max|Mx|", np.abs(dw - Mx[1:] / a.Ixx).max() / np.abs(Mx / a.Ixx).max(), 1e-4)
    report("a_y vs g + Faero_y/m, abs [m/s2]", np.abs(Ay - (-g + Fy / m)).max(), 1e-6)
    Vtr = Vy[0] + np.cumsum(0.5 * (Ay[1:] + Ay[:-1]) * dt)  # trapezoidal (Heun) velocity update
    report("Vy vs trapezoid-integrated a_y, abs [m/s]", np.abs(Vtr - Vy[1:]).max(), 1e-6)
    ang = np.unwrap(np.radians(r[:, 10]))
    ang_i = ang[0] + np.concatenate(([0], np.cumsum(0.5 * (wx[1:] + wx[:-1]) * dt)))
    report("angle_x vs integral of omega_x, abs [deg]", np.degrees(np.abs(ang - ang_i)).max(), 0.5)
    y = np.concatenate(([0], np.cumsum(0.5 * (Vy[1:] + Vy[:-1]) * dt)))
    E = 0.5 * m * (Vx**2 + Vy**2 + Vz**2) + 0.5 * a.Ixx * wx**2 + m * g * y
    P = Fx * Vx + Fy * Vy + Fz * Vz + Mx * wx  # aero power on the body
    Wk = np.concatenate(([0], np.cumsum(0.5 * (P[1:] + P[:-1]) * dt)))
    report("mech. energy vs aero work, rel.", np.abs((E - E[0]) - Wk).max() / np.abs(Wk).max(), 1e-3)

    # per-revolution table and asymptote
    deg = np.degrees(ang)
    sgn = -1.0 if deg[-1] < 0 else 1.0
    revs, k, t0 = [], 1, 0.0
    for i in range(1, len(deg)):
        tgt = sgn * 360.0 * k
        if (deg[i] - tgt) * (deg[i - 1] - tgt) < 0:  # crossed k full turns
            fr = (tgt - deg[i - 1]) / (deg[i] - deg[i - 1])
            tc = t[i - 1] + fr * (t[i] - t[i - 1])
            revs.append((t0, tc))
            t0, k = tc, k + 1
    vy, vh = np.abs(Vy), np.hypot(Vx, Vz)
    print("\nPer-revolution means")
    print("  rev   t_mid[ms]  f[Hz]   <Vy>[m/s]  <Fy>/W   psi[deg]")
    Vm, Tm = [], []
    for i, (t0, t1) in enumerate(revs, 1):
        Vm.append(cycle_mean(vy, t, t0, t1))
        Tm.append(0.5 * (t0 + t1))
        psi = np.degrees(np.arctan(cycle_mean(vh, t, t0, t1) / Vm[-1]))
        print(f"  {i:3d}  {1e3*Tm[-1]:8.1f}  {1/(t1-t0):6.2f}  {Vm[-1]:9.4f}  {cycle_mean(np.abs(Fy), t, t0, t1)/W:7.4f}  {psi:7.2f}")

    Vm, Tm = np.array(Vm), np.array(Tm)
    d1, d2 = Vm[-2] - Vm[-3], Vm[-1] - Vm[-2]
    print(f"\nAitken asymptote of <Vy> (last 3 revs): {Vm[-1] - d2**2 / (d2 - d1):.4f} m/s")
    best = None
    for tau in np.linspace(0.02, 0.5, 481):  # Vy(t) = Vinf + B exp(-t/tau) on revs >= 6
        X = np.vstack([np.ones(len(Tm[5:])), np.exp(-Tm[5:] / tau)]).T
        c = np.linalg.lstsq(X, Vm[5:], rcond=None)[0]
        e = ((X @ c - Vm[5:])**2).sum()
        if best is None or e < best[0]:
            best = (e, tau, c)
    print(f"Exp-relaxation fit: Vinf = {best[2][0]:.4f} m/s, tau = {1e3*best[1]:.0f} ms, run length = {1e3*t[-1]:.0f} ms")


if __name__ == "__main__":
    main()
