#!/usr/bin/env python3
"""
check_extension.py <case> [<case> ...] [--t-from T] [--thresh 0.003] [--mass m]

Checks the drift in V_eq,est between the last two complete revolutions inside t >= T.
Returns exit code 0 if all cases have drift <= thresh (developed limit cycle converged).
Returns exit code 1 if any case has drift > thresh or has fewer than 2 complete revolutions (extension needed).
"""
import argparse, os, sys
import numpy as np

# Import rev_edges and cmean from compare_variants in same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare_variants import rev_edges, cmean


def check_case(case, t_from, W, thresh):
    tumble_path = os.path.join(case, "postProcessing/tumbleMotion/motion.dat")
    fall_path = os.path.join(case, "postProcessing/fallingMotion/motion.dat")
    if not (os.path.exists(tumble_path) and os.path.exists(fall_path)):
        print(f"[{case}] Missing motion.dat files!")
        return False, None, None

    r = np.loadtxt(tumble_path, comments="#")
    f = np.loadtxt(fall_path)
    n = min(len(r), len(f))
    r, f = r[:n], f[:n]
    t = r[:, 0]
    _, u_idx = np.unique(t, return_index=True)
    r, f, t = r[u_idx], f[u_idx], t[u_idx]
    ang = np.degrees(np.unwrap(np.radians(r[:, 10])))
    e = rev_edges(t, ang, t_from)
    if len(e) < 3:
        print(f"[{case}] Only {len(e)-1} complete revolution(s) after t = {t_from:.3f} s (need at least 2 for drift check)")
        return False, len(e) - 1, None

    vy, fy = np.abs(f[:, 3]), np.abs(f[:, 27])
    Vm = [cmean(vy, t, a, b) for a, b in zip(e[:-1], e[1:])]
    Fm = [cmean(fy, t, a, b) for a, b in zip(e[:-1], e[1:])]
    Veq = [v / np.sqrt(F / W) for v, F in zip(Vm, Fm)]

    n_revs = len(Veq)
    v_last = Veq[-1]
    v_prev = Veq[-2]
    drift = abs(v_last - v_prev) / v_prev

    print(f"[{case}] {n_revs} complete revolutions after t = {t_from:.3f} s:")
    for i, (a, b, v) in enumerate(zip(e[:-1], e[1:], Veq)):
        diff_str = f"{(v / Veq[i-1] - 1) * 100:+.3f}%" if i > 0 else "ref"
        print(f"  Rev {i+1} [t={a:.4f}..{b:.4f} s]: V_eq = {v:.4f} m/s ({diff_str})")

    print(f"  Drift between last two revs: {drift * 100:.3f}% (threshold: {thresh * 100:.2f}%)")
    converged = drift <= thresh
    status = "CONVERGED (no extension needed)" if converged else "DRIFTING (extension recommended)"
    print(f"  Status: {status}\n")
    return converged, n_revs, drift


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cases", nargs="+")
    ap.add_argument("--t-from", type=float, default=0.08)
    ap.add_argument("--thresh", type=float, default=0.003)
    ap.add_argument("--mass", type=float, default=2.2113e-7)
    a = ap.parse_args()
    W = a.mass * 9.81

    all_converged = True
    for c in a.cases:
        conv, n_revs, drift = check_case(c, a.t_from, W, a.thresh)
        if not conv:
            all_converged = False

    if all_converged:
        print("All cases converged within limit-cycle tolerance.")
        sys.exit(0)
    else:
        print("One or more cases require extension.")
        sys.exit(1)


if __name__ == "__main__":
    main()
