#!/usr/bin/env python3
"""
analyze_fixed_disk.py <case> --Re <Re>

Baseline-drag diagnostics for the fixed broadside disk (make_fixed_disk.sh):
  C_D  = F_y / (0.5 rho V^2 A)                 mean and drift over the analysis window
  F_lat/F_y                                    side force; ~0 if the wake stays axisymmetric
  sigma                                        growth rate of the side-force envelope (>0 grows, <0 decays)
  St   = f D / V                               dominant frequency of the side force
  cross-check of the solver's own F_aero against OpenFOAM's `forces` function object (if present)
Physics to compare against (Natarajan & Acrivos 1993): the wake of a thin disk loses axisymmetry at Re_D ~ 116.5,
so Re = 100 should give sigma < 0 and F_lat -> 0, while Re = 140 should give sigma > 0 and a finite side force.
A mesh/scheme that is too dissipative shows sigma < 0 at Re = 140.
"""
import argparse, glob, os, re
import numpy as np

RHO, D = 1.2, 2.02e-3
A = np.pi * D**2 / 4


def load_forces_dat(path):
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        v = [float(x) for x in re.sub(r"[()]", " ", line).split()]
        rows.append(v[:10])  # t, total(3), pressure(3), viscous(3)
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("case")
    ap.add_argument("--Re", type=float, required=True)
    ap.add_argument("--frac", type=float, default=0.4)  # last fraction of the run used as analysis window
    a = ap.parse_args()

    f = np.loadtxt(os.path.join(a.case, "postProcessing/fallingMotion/motion.dat"))
    t, Vy = f[:, 0], np.abs(f[:, 3])
    Fx, Fy, Fz = f[:, 26], f[:, 27], f[:, 28]
    V = Vy.mean()
    print(f"run: {len(t)} steps, t_end = {1e3*t[-1]:.1f} ms, V = {V:.4f} m/s "
          f"(Re_D = {V*D/1.5e-5:.1f}, spread {np.ptp(Vy)/V:.1e}), flow-throughs of 50 mm = {t[-1]*V/0.05:.1f}")

    q = 0.5 * RHO * V**2 * A
    Cd = Fy / q
    Flat = np.hypot(Fx, Fz)
    w = t >= t[0] + (1 - a.frac) * (t[-1] - t[0])
    slope = np.polyfit(t[w], Cd[w], 1)[0] * 0.1  # drift per 100 ms
    print(f"\nC_D (window {1e3*t[w][0]:.0f}-{1e3*t[-1]:.0f} ms): mean {Cd[w].mean():.4f}, "
          f"min {Cd[w].min():.4f}, max {Cd[w].max():.4f}, drift {slope:+.4f} per 100 ms")
    print(f"F_lat/F_y: mean {np.mean(Flat[w]/Fy[w]):.4f}, rms {np.sqrt(np.mean((Flat[w]/Fy[w])**2)):.4f}")

    # envelope growth rate of the side force: max |F_lat| in 2 ms bins, log-linear fit
    nb = max(int((t[w][-1] - t[w][0]) / 2e-3), 3)
    edges = np.linspace(t[w][0], t[w][-1], nb + 1)
    tb, eb = [], []
    for i in range(nb):
        m = (t >= edges[i]) & (t < edges[i + 1])
        if m.any() and Flat[m].max() > 0:
            tb.append(0.5 * (edges[i] + edges[i + 1])); eb.append(Flat[m].max())
    sig = np.polyfit(tb, np.log(eb), 1)[0]
    print(f"side-force envelope growth rate sigma = {sig:+.1f} 1/s  (sigma D/V = {sig*D/V:+.4f}); "
          f"{'GROWING' if sig > 0 else 'DECAYING'}")

    # dominant frequency of side force (resampled to a uniform grid)
    tu = np.arange(t[w][0], t[-1], 2e-5)
    for name, F in (("F_x", Fx), ("F_z", Fz)):
        s = np.interp(tu, t, F); s -= s.mean()
        nfft = 8 * len(s)  # zero-padded for finer frequency reading
        S = np.abs(np.fft.rfft(s * np.hanning(len(s)), nfft)); fr = np.fft.rfftfreq(nfft, 2e-5)
        k = np.argmax(S[1:]) + 1
        print(f"{name}: peak {fr[k]:.1f} Hz -> St = {fr[k]*D/V:.3f}, amplitude {2*S[k]/np.hanning(len(s)).sum()/Fy[w].mean():.4f} of mean F_y")

    for p in glob.glob(os.path.join(a.case, "postProcessing/diskForces/*/force*.dat")):
        g = load_forces_dat(p)
        Fy2 = np.interp(t, g[:, 0], g[:, 2])
        d = np.abs(Fy2[w] - Fy[w]).max() / np.abs(Fy[w]).mean()
        print(f"\ncross-check vs `forces` ({os.path.basename(os.path.dirname(p))}): max |dF_y|/mean F_y = {d:.2e} "
              f"[{'PASS' if d < 1e-2 else 'FAIL'}]")
        break
    for p in glob.glob(os.path.join(a.case, "postProcessing/amiFluxMaster/*/surfaceFieldValue.dat")):
        g = np.loadtxt(p, comments="#")
        print(f"AMI net flux through masterAMI: max |sum phi| = {np.abs(g[:,1]).max():.2e} m3/s "
              f"(inflow V*A_domain = {V*0.03*0.03:.2e} m3/s)")
        break


if __name__ == "__main__":
    main()
