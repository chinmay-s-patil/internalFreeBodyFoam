#!/usr/bin/env python3
"""
analyze_fixed_disk.py <case> --Re <Re> [--t-start T] [--frac FRAC] [--force-preliminary]

Baseline-drag diagnostics for the fixed broadside disk (make_fixed_disk.sh):
  C_D = F_y / (0.5 rho V^2 A)                mean, min/max, and linear drift over window
  F_lat/F_y                                  side-force ratio (expected ~0 if subcritical, finite if supercritical)
  sigma                                      growth rate of side-force envelope using Hilbert transform
  St = f D / V                               dominant frequency of lateral shedding with explicit
                                             Rayleigh resolution check (Delta f = 1/T_window)
  Full 3D force & moment cross-check against OpenFOAM's `forces` function object (F_x, F_y, F_z, M_x, M_y, M_z)
  AMI interface flux leak normalized against domain inflow and rotating sphere cross-section

Physics reference:
  - Meliga, Chomaz & Sipp (2009, J. Fluid Mech. 633, 159-189; DOI: 10.1017/S0022112009007290):
    * Steady regular bifurcation at Re_c1 ~ 116.9 (wake loses axisymmetry, planar-symmetric steady state)
    * Unsteady oscillatory Hopf bifurcation at Re_c2 ~ 125.3 (unsteady hairpin vortex shedding with St ~ 0.11 - 0.12)
  - Stokes creeping flow baseline: C_D = 64 / (pi * Re) ≈ 20.37 / Re (Oberbeck 1876).
  - High-Re DNS anchor: C_D ≈ 1.054 at Re = 1000 (Tian 2019, arXiv:1906.10578; citing Roos & Willmarth 1971, C_D ~ 1.10).
  - At Re = 100: subcritical wake, must be steady and axisymmetric (sigma < 0, F_lat -> 0).
    Literature broadside drag estimates span C_D ~ 1.15 - 1.25 (unverified secondary estimate cited for h/D = 0.1;
    Shenoy & Kleinstreuer 2008 has not been directly audited).
    (Note: Chrust et al. 2013 PoF 25, 044102 is a free-fall 6-DoF trajectory study; the fixed-body transition study
    is Chrust et al. 2010 JFM 665, 199-208).
  - At Re = 140: supercritical wake above regular bifurcation (Re_c1 ~ 116.9) and Hopf threshold (Re_c2 ~ 125.3).
    For our slender disk (h = 50 um, D = 2.02 mm, h/D ≈ 0.025, chi = 40.4), which lies close to the zero-thickness limit,
    the wake breaks axisymmetry. When starting from unperturbed flow, linear modal amplification of the steady planar mode
    from numerical seeds (~1e-10 N, originating in mesh asymmetry and solver tolerances) proceeds exponentially before
    non-linear saturation.

Convergence Gate:
  Requires >= 3.0 domain flow-throughs AND C_D drift < 1.0% per 100 ms to evaluate steady C_D,
  shedding Strouhal number, and envelope growth rate.
"""
import argparse, glob, os, re, sys
import numpy as np
from scipy.signal import hilbert

RHO, D = 1.2, 2.02e-3
A = np.pi * D**2 / 4
R_SPHERE = 2.02e-3  # 4.04 mm diameter rotating sphere (radius = 2.02 mm)
A_SPHERE = np.pi * R_SPHERE**2


def load_forces_dat(path):
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        v = [float(x) for x in re.sub(r"[()]", " ", line).split()]
        rows.append(v[:10])  # t, total(3), pressure(3), viscous(3)
    return np.array(rows)


def load_moments_dat(path):
    rows = []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        v = [float(x) for x in re.sub(r"[()]", " ", line).split()]
        rows.append(v[:10])
    return np.array(rows)


def compute_hilbert_growth(t_eval, signal):
    """Computes growth rate sigma from the log-linear fit of the Hilbert envelope."""
    s = signal - np.mean(signal)
    if np.all(np.abs(s) < 1e-18):
        return 0.0, np.zeros_like(t_eval)
    env = np.abs(hilbert(s))
    env_safe = np.maximum(env, 1e-18)
    n = len(t_eval)
    i0, i1 = int(0.1 * n), int(0.9 * n)
    if i1 - i0 > 5:
        p = np.polyfit(t_eval[i0:i1], np.log(env_safe[i0:i1]), 1)
        sigma = p[0]
    else:
        p = np.polyfit(t_eval, np.log(env_safe), 1)
        sigma = p[0]
    return sigma, env


def main():
    ap = argparse.ArgumentParser(description="Baseline drag and wake stability diagnostics for fixed disk.")
    ap.add_argument("case", help="Path to OpenFOAM case directory")
    ap.add_argument("--Re", type=float, required=True, help="Nominal Reynolds number")
    ap.add_argument("--t-start", type=float, default=None, help="Start time of analysis window (s)")
    ap.add_argument("--frac", type=float, default=0.4, help="Fraction of run if --t-start is omitted (default: 0.4)")
    ap.add_argument("--force-preliminary", action="store_true",
                    help="Force spectral and envelope analysis even if convergence gate is not met")
    a = ap.parse_args()

    motion_path = os.path.join(a.case, "postProcessing/fallingMotion/motion.dat")
    if not os.path.isfile(motion_path):
        print(f"Error: {motion_path} not found.")
        sys.exit(1)

    f = np.loadtxt(motion_path)
    t, Vy = f[:, 0], np.abs(f[:, 3])
    Fx, Fy, Fz = f[:, 26], f[:, 27], f[:, 28]
    V = Vy.mean()
    t_tot = t[-1] - t[0]
    n_flowthroughs = t[-1] * V / 0.05

    print(f"================================================================================")
    print(f"Case: {a.case}")
    print(f"Run duration: {len(t)} steps, t_end = {1e3*t[-1]:.2f} ms, V = {V:.4f} m/s "
          f"(Re_D = {V*D/1.5e-5:.1f}, velocity spread {np.ptp(Vy)/V:.1e})")
    print(f"Total Domain Flow-Throughs (L = 50 mm): {n_flowthroughs:.2f}")

    # Determine analysis window
    t_3ft = 3.0 * 0.05 / V  # Time for 3 domain flow-throughs
    if a.t_start is not None:
        t_start = a.t_start
    else:
        if t[-1] >= t_3ft * 1.2:
            t_start = t_3ft
            print(f"Window selection: using post-3-flowthrough data (t_start = {1e3*t_start:.1f} ms)")
        else:
            t_start = t[0] + (1.0 - a.frac) * t_tot
            print(f"Window selection: total run < 3 flow-throughs; using final {a.frac*100:.0f}% of run (t_start = {1e3*t_start:.1f} ms)")

    w = t >= t_start
    if np.sum(w) < 10:
        print("Error: analysis window contains fewer than 10 points.")
        sys.exit(1)

    tw = t[w]
    T_win = tw[-1] - tw[0]
    df_rayleigh = 1.0 / T_win if T_win > 0 else 0.0

    q = 0.5 * RHO * V**2 * A
    Cd = Fy / q
    Flat = np.hypot(Fx, Fz)
    drift_100ms = np.polyfit(tw, Cd[w], 1)[0] * 0.1
    drift_pct = (drift_100ms / Cd[w].mean()) * 100.0

    # Evaluate Convergence Gate:
    # Requires >= 3.0 domain flow-throughs AND drift below 1.0% per 100 ms
    gate_ft_pass = (n_flowthroughs >= 3.0)
    gate_drift_pass = (abs(drift_pct) < 1.0)
    conv_gate_passed = gate_ft_pass and gate_drift_pass

    print(f"\n--- Convergence Gate Assessment ---")
    print(f"  Criterion 1: Flow-throughs >= 3.0       : {n_flowthroughs:.2f} -> [{'PASS' if gate_ft_pass else 'FAIL (mid-transient)'}]")
    print(f"  Criterion 2: C_D drift < 1.0% / 100 ms   : {drift_pct:+.2f}% / 100 ms -> [{'PASS' if gate_drift_pass else 'FAIL (actively drifting)'}]")
    print(f"  Overall Gate Status                      : [{'CONVERGED' if conv_gate_passed else 'UNCONVERGED / MID-TRANSIENT'}]")

    print(f"\n--- Aerodynamic Force Coefficients (Window: {1e3*tw[0]:.1f} - {1e3*tw[-1]:.1f} ms, duration {1e3*T_win:.1f} ms) ---")
    print(f"C_D: mean = {Cd[w].mean():.4f}, window spread = [{Cd[w].min():.4f}, {Cd[w].max():.4f}]")
    print(f"C_D drift: {drift_100ms:+.4f} per 100 ms ({drift_pct:+.2f}% / 100 ms) "
          f"[{'PLATEAUED' if gate_drift_pass else 'DRIFTING (mid-transient)'}]")
    print(f"Lateral force ratio |F_lat|/F_y: mean = {np.mean(Flat[w]/Fy[w]):.4e}, max = {np.max(Flat[w]/Fy[w]):.4e}")

    if not conv_gate_passed and not a.force_preliminary:
        print(f"\n[CONVERGENCE GATE NOT MET]")
        print(f"  The simulation has not reached developed asymptotic state ({n_flowthroughs:.2f} flow-throughs < 3.0; "
              f"drift {drift_pct:+.2f}%/100 ms >= 1.0%).")
        print(f"  Shedding Strouhal number (St), envelope growth rate (sigma), and steady literature C_D comparisons")
        print(f"  are withheld to avoid treating transient startup artifacts as asymptotic wake physics.")
        print(f"  (Re-run with `--force-preliminary` to inspect raw intermediate signals during diagnostic testing).")
    else:
        prefix = "[PRELIMINARY] " if not conv_gate_passed else ""
        # Resample to uniform grid for FFT and Hilbert transform
        dt_resample = min(2e-5, (tw[-1] - tw[0]) / max(len(tw), 500))
        tu = np.arange(tw[0], tw[-1], dt_resample)

        # Frequency analysis
        print(f"\n--- {prefix}Spectral & Shedding Frequency Analysis (Rayleigh resolution Delta f = {df_rayleigh:.1f} Hz) ---")
        f_peak_found = None
        for name, F in (("F_x", Fx), ("F_z", Fz)):
            s = np.interp(tu, t, F)
            s -= s.mean()
            if np.ptp(s) < 1e-15:
                print(f"{name}: signal amplitude negligible (< 1e-15 N); axisymmetric flow.")
                continue
            nfft = 8 * len(s)
            S = np.abs(np.fft.rfft(s * np.hanning(len(s)), nfft))
            fr = np.fft.rfftfreq(nfft, dt_resample)
            k = np.argmax(S[1:]) + 1
            f_dom = fr[k]
            St_dom = f_dom * D / V
            amp = 2.0 * S[k] / np.hanning(len(s)).sum()
            amp_rel = amp / Fy[w].mean()
            n_cycles = T_win * f_dom
            f_peak_found = f_dom

            status_flags = []
            if f_dom <= 1.2 * df_rayleigh:
                status_flags.append(f"UNRESOLVED (f_peak <= Delta f = {df_rayleigh:.1f} Hz)")
            if n_cycles < 3.0:
                status_flags.append(f"SHORT ({n_cycles:.1f} cycles in window; need >= 8-10)")

            status_str = f" [{', '.join(status_flags)}]" if status_flags else " [RESOLVED]"
            print(f"{name}: peak f = {f_dom:.1f} Hz -> St = {St_dom:.4f}, amp = {amp:.2e} N ({amp_rel*100:.3f}% of F_y), "
                  f"cycles = {n_cycles:.1f}{status_str}")

        # Side-force azimuth angle analysis (distinguishes steady planar m=1 mode from oscillatory shedding)
        phi_rad = np.arctan2(Fz[w], Fx[w])
        phi_deg = np.degrees(phi_rad)
        mean_phi = np.degrees(np.arctan2(np.mean(np.sin(phi_rad)), np.mean(np.cos(phi_rad))))
        std_phi = np.std(phi_deg)
        print(f"\n--- Side-Force Azimuth Orientation (Plane of Deflection) ---")
        if np.max(Flat[w]) < 1e-12:
            print(f"Azimuth: signal amplitude negligible (< 1e-12 N); axisymmetric flow.")
        else:
            mode_type = "STEADY PLANAR DEFLECTION (m=1 regular mode)" if std_phi < 5.0 else "ROTATING / OSCILLATING (shedding mode)"
            print(f"Azimuth phi = atan2(F_z, F_x): mean = {mean_phi:+6.1f} deg, variation = +/-{std_phi:.2f} deg [{mode_type}]")

        # Growth rate of lateral perturbations using Hilbert envelope and post-startup log-linear fit
        print(f"\n--- {prefix}Side-Force Envelope Growth Rate ---")
        # Post-startup exponential growth (t >= 80 ms to exclude zero-crossing transient)
        m_dev80 = (t >= 0.080) & (t <= tw[-1])
        if np.sum(m_dev80) > 10 and np.max(Flat[m_dev80]) > 1e-13:
            p_dev80 = np.polyfit(t[m_dev80], np.log(np.maximum(Flat[m_dev80], 1e-18)), 1)
            sig_dev80 = p_dev80[0]
            sig_dev_star80 = sig_dev80 * D / V
            dev_regime80 = "GROWING" if sig_dev80 > 1.0 else ("DECAYING" if sig_dev80 < -1.0 else "STEADY / NEUTRAL")
            print(f"Post-startup modal growth (80 - {1e3*tw[-1]:.0f} ms): sigma = {sig_dev80:+6.1f} 1/s "
                  f"(sigma* = sigma D/V = {sig_dev_star80:+7.4f}); {dev_regime80}")

        for name, F in (("F_x", Fx), ("F_z", Fz), ("|F_lat|", Flat)):
            s = np.interp(tu, t, F)
            sig, env = compute_hilbert_growth(tu, s)
            sig_ndim = sig * D / V
            regime = "GROWING" if sig > 1.0 else ("DECAYING" if sig < -1.0 else "STEADY / NEUTRAL")
            qualifier = " (PRELIMINARY: short window / startup phase)" if not conv_gate_passed else ""
            print(f"Intra-window Hilbert {name:<8}: sigma = {sig:+8.1f} 1/s  (sigma* = sigma D/V = {sig_ndim:+7.4f}); {regime}{qualifier}")

    # Comprehensive multi-axis cross-check against OpenFOAM forces (always executed)
    print(f"\n--- Verification Cross-Check vs OpenFOAM `forces` Library ---")
    forces_found = False
    for p in sorted(glob.glob(os.path.join(a.case, "postProcessing/diskForces/*/force*.dat"))):
        g = load_forces_dat(p)
        gt = g[:, 0]
        gw = (gt >= tw[0]) & (gt <= tw[-1])
        if np.sum(gw) > 0:
            dfy = np.abs(np.interp(gt[gw], t, Fy) - g[gw, 2])
            dfx = np.abs(np.interp(gt[gw], t, Fx) - g[gw, 1])
            dfz = np.abs(np.interp(gt[gw], t, Fz) - g[gw, 3])
            rel_err_y = dfy.max() / max(np.abs(Fy[w]).mean(), 1e-12)
            print(f"Force F_y relative error: {rel_err_y:.2e} [{'PASS' if rel_err_y < 1e-4 else 'FAIL'}]")
            print(f"Lateral forces max |dF_x| = {dfx.max():.2e} N, max |dF_z| = {dfz.max():.2e} N")
            forces_found = True
            break
    if not forces_found:
        print("Note: no `force*.dat` found in postProcessing/diskForces.")

    # Cross-check moments against OpenFOAM diskForces/moment.dat
    tumble_path = os.path.join(a.case, "postProcessing/tumbleMotion/motion.dat")
    if os.path.isfile(tumble_path):
        for p in sorted(glob.glob(os.path.join(a.case, "postProcessing/diskForces/*/moment*.dat"))):
            gm = load_moments_dat(p)
            tm = np.loadtxt(tumble_path, comments="#")
            tt, mx, my, mz = tm[:, 0], tm[:, 7], tm[:, 8], tm[:, 9]
            gw = (gm[:, 0] >= tw[0]) & (gm[:, 0] <= tw[-1])
            if np.sum(gw) > 0:
                dmx = np.abs(np.interp(gm[gw, 0], tt, mx) - gm[gw, 1]).max()
                dmy = np.abs(np.interp(gm[gw, 0], tt, my) - gm[gw, 2]).max()
                dmz = np.abs(np.interp(gm[gw, 0], tt, mz) - gm[gw, 3]).max()
                print(f"Moments max difference: |dMx| = {dmx:.2e} N.m, |dMy| = {dmy:.2e} N.m, |dMz| = {dmz:.2e} N.m [{'PASS' if max(dmx, dmy, dmz) < 1e-8 else 'FAIL'}]")
                break

    # AMI Mass Flux Conservation
    for p in sorted(glob.glob(os.path.join(a.case, "postProcessing/amiFluxMaster/*/surfaceFieldValue.dat"))):
        g = np.loadtxt(p, comments="#")
        max_flux = np.abs(g[:, 1]).max()
        q_domain = V * 0.03 * 0.03
        q_sphere = V * A_SPHERE
        print(f"\n--- Sliding CyclicAMI Interface Mass Conservation ---")
        print(f"Max |sum(masterAMI) phi|: {max_flux:.2e} m3/s")
        print(f"Relative to domain inflow (V * A_domain = {q_domain:.2e} m3/s): {max_flux/q_domain:.2e}")
        print(f"Relative to sphere cross-section (V * A_sphere = {q_sphere:.2e} m3/s): {max_flux/q_sphere:.2e} "
              f"[{'PASS (tolerance check)' if max_flux/q_sphere < 1e-4 else 'FAIL'}]")
        break
    print(f"================================================================================\n")


if __name__ == "__main__":
    main()
