#!/usr/bin/env python3
"""
compare_variants.py <case> [<case> ...] [--t-from T] [--mass m] [--Ixx I]

Compares developed-state metrics between cases over complete revolutions inside the window t >= T.
The first case is the reference. V_eq,est = <Vy> / sqrt(<Fy>/W) estimates the settling speed at which <Fy> = W
(quasi-steady F ~ V^2), so short windows that are still relaxing remain comparable (on the 13-rev run it gives
1.024 m/s against an Aitken asymptote of 1.0236 m/s).
"""
import argparse, os
import numpy as np


def rev_edges(t, ang_deg, t_from):
    i0 = np.searchsorted(t, t_from)
    s = np.abs(ang_deg - ang_deg[i0])  # tumbling is unidirectional, s grows monotonically
    edges, k = [t[i0]], 1
    while True:
        j = np.searchsorted(s[i0:], 360.0 * k) + i0
        if j >= len(t):
            break
        fr = (360.0 * k - s[j - 1]) / (s[j] - s[j - 1])
        edges.append(t[j - 1] + fr * (t[j] - t[j - 1])); k += 1
    return edges


def cmean(a, t, t0, t1):
    m = (t > t0) & (t < t1)
    tt = np.concatenate(([t0], t[m], [t1]))
    aa = np.concatenate(([np.interp(t0, t, a)], a[m], [np.interp(t1, t, a)]))
    trapz_fn = getattr(np, "trapezoid", getattr(np, "trapz", None))
    return trapz_fn(aa, tt) / (t1 - t0)


def metrics(case, t_from, W):
    r = np.loadtxt(os.path.join(case, "postProcessing/tumbleMotion/motion.dat"), comments="#")
    f = np.loadtxt(os.path.join(case, "postProcessing/fallingMotion/motion.dat"))
    t = r[:, 0]
    ang = np.degrees(np.unwrap(np.radians(r[:, 10])))
    e = rev_edges(t, ang, t_from)
    if len(e) < 2:
        return None
    vy, fy, vh = np.abs(f[:, 3]), np.abs(f[:, 27]), np.hypot(f[:, 2], f[:, 4])
    Vm = np.array([cmean(vy, t, a, b) for a, b in zip(e[:-1], e[1:])])
    Fm = np.array([cmean(fy, t, a, b) for a, b in zip(e[:-1], e[1:])])
    Hm = np.array([cmean(vh, t, a, b) for a, b in zip(e[:-1], e[1:])])
    m = (t >= e[0]) & (t <= e[-1])
    return dict(n=len(Vm), ft=1.0 / np.mean(np.diff(e)), V=Vm.mean(), FW=Fm.mean() / W,
                Veq=Vm.mean() / np.sqrt(Fm.mean() / W), psi=np.degrees(np.arctan(Hm.mean() / Vm.mean())),
                Mpk=np.percentile(np.abs(r[m, 7]), 95) * 1e9, tend=t[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--t-from", type=float, default=0.0)
    ap.add_argument("--mass", type=float, default=2.2113e-7)
    a = ap.parse_args()
    W = a.mass * 9.81
    res = [(c, metrics(c, a.t_from, W)) for c in a.cases]
    ref = next((m for _, m in res if m), None)
    print(f"{'case':<22}{'revs':>5}{'f_t[Hz]':>9}{'<Vy>':>8}{'<Fy>/W':>8}{'V_eq,est':>10}{'psi[deg]':>10}{'Mx_p95[nNm]':>13}")
    for c, m in res:
        if m is None:
            print(f"{c:<22}  no complete revolution after t = {a.t_from}"); continue
        print(f"{os.path.basename(c.rstrip('/')):<22}{m['n']:>5}{m['ft']:>9.2f}{m['V']:>8.4f}{m['FW']:>8.4f}{m['Veq']:>10.4f}{m['psi']:>10.2f}{m['Mpk']:>13.3f}")
    if ref:
        print("\nrelative to first case [%]:  f_t   V_eq,est   psi   Mx_p95")
        for c, m in res[1:]:
            if m:
                d = lambda k: 100 * (m[k] / ref[k] - 1)
                print(f"  {os.path.basename(c.rstrip('/')):<20}{d('ft'):+8.2f}{d('Veq'):+10.2f}{d('psi'):+8.2f}{d('Mpk'):+9.2f}")
        print("\nRead-out: V_eq,est shifts of more than ~1-2 % between variants mean the baseline is not converged in that parameter;"
              "\nthe 17 % deficit to the experiment (1.23 m/s) is only explained if refinement moves V_eq,est by an amount of that size.")


if __name__ == "__main__":
    main()
