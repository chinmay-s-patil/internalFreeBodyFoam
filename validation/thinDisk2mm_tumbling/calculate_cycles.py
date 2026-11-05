#!/usr/bin/env python3
import numpy as np

# Load tumbleMotion/motion.dat
# Cols: 0: Time, 1: Omega_x, 2: Omega_y, 3: Omega_z, 4: RPM_x, ..., 10: Angle_x(deg)
tumble_file = "validation/thinDisk2mm_tumbling/case/postProcessing/tumbleMotion/motion.dat"
fall_file = "validation/thinDisk2mm_tumbling/case/postProcessing/fallingMotion/motion.dat"

# Load data, skipping comments
tumble_data = []
with open(tumble_file, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = [float(x) for x in line.split()]
        tumble_data.append(parts)

tumble_arr = np.array(tumble_data)
time_t = tumble_arr[:, 0]
omega_x = tumble_arr[:, 1]
angle_x = np.degrees(np.unwrap(np.radians(tumble_arr[:, 10]))) # unwrapped continuous degrees

# Load falling data
fall_data = []
with open(fall_file, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = [float(x) for x in line.split()]
        fall_data.append(parts)

fall_arr = np.array(fall_data)
time_f = fall_arr[:, 0]
vy = np.abs(fall_arr[:, 3]) # vertical fall speed

# Physical constants
m = 2.2113e-7
g = 9.81
W = m * g
rho = 1.200
D = 2.02e-3
A = np.pi * 0.25 * D**2

def time_average(t_pts, y_pts, t0, t1):
    """Compute true continuous time-weighted integral average with boundary interpolation."""
    if t1 <= t0:
        return 0.0
    y0 = np.interp(t0, t_pts, y_pts)
    y1 = np.interp(t1, t_pts, y_pts)
    mask = (t_pts > t0) & (t_pts < t1)
    sub_t = np.concatenate(([t0], t_pts[mask], [t1]))
    sub_y = np.concatenate(([y0], y_pts[mask], [y1]))
    trapz_func = np.trapezoid if hasattr(np, 'trapezoid') else np.trapz
    integral = trapz_func(sub_y, sub_t)
    return integral / (t1 - t0)

print(f"Total time points: tumble={len(time_t)}, fall={len(time_f)}")
print(f"Time range: {time_t[0]:.6f} to {time_t[-1]:.6f} s")
print(f"Total rotation: {angle_x[-1]:.2f} deg ({abs(angle_x[-1])/360.0:.2f} revolutions)")

# Detect cycle crossings (every -360 deg)
# Note angle_x starts at 0 and decreases (negative rotation)
# Rev k ends when angle_x crosses -360 * k
revolutions = []
target_rev = 1
start_idx = 0

for i in range(1, len(angle_x)):
    target_deg = -360.0 * target_rev
    if angle_x[i] <= target_deg and angle_x[i-1] > target_deg:
        # Interpolate exact crossing time
        frac = (target_deg - angle_x[i-1]) / (angle_x[i] - angle_x[i-1])
        t_cross = time_t[i-1] + frac * (time_t[i] - time_t[i-1])
        revolutions.append({
            'rev': target_rev,
            'start_idx': start_idx,
            'end_idx': i,
            't_end': t_cross
        })
        start_idx = i
        target_rev += 1

print("\n--- Completed Revolutions (Time-Weighted Trapezoidal Averages) ---")
t_start = 0.0
for r in revolutions:
    k = r['rev']
    t_end = r['t_end']
    T_rev = t_end - t_start
    f_t = 1.0 / T_rev
    
    # Time-weighted averages
    om_mean = time_average(time_t, omega_x, t_start, t_end)
    vy_mean = time_average(time_f, vy, t_start, t_end)
    
    # Min/max within cycle
    mask_t = (time_t >= t_start) & (time_t <= t_end)
    om_min = np.min(omega_x[mask_t]) # most negative
    om_max = np.max(omega_x[mask_t]) # least negative
    
    # CD paper from time-averaged settling velocity
    cd_paper = (2.0 * W) / (rho * A * vy_mean**2)
    
    print(f"Rev {k}: t={t_start*1000:.1f} -> {t_end*1000:.1f} ms (T={T_rev*1000:.2f} ms, f={f_t:.2f} Hz) | Mean Om_x={om_mean:.1f} rad/s [{om_min:.1f}, {om_max:.1f}] | Mean Vy={vy_mean:.4f} m/s | CD_paper={cd_paper:.3f}")
    t_start = t_end

# If there is an active incomplete revolution
if t_start < time_t[-1]:
    k = target_rev
    t_end = time_t[-1]
    T_partial = t_end - t_start
    deg_partial = abs(angle_x[-1] - (-360.0 * (k - 1)))
    f_inst = (deg_partial / 360.0) / T_partial if T_partial > 0 else 0
    om_mean = time_average(time_t, omega_x, t_start, t_end)
    vy_mean = time_average(time_f, vy, t_start, t_end)
    mask_t = (time_t >= t_start)
    om_min = np.min(omega_x[mask_t])
    om_max = np.max(omega_x[mask_t])
    cd_paper = (2.0 * W) / (rho * A * vy_mean**2)
    print(f"Rev {k} (Incomplete): t={t_start*1000:.1f} -> {t_end*1000:.1f} ms ({deg_partial:.1f} deg completed) | Mean Om_x={om_mean:.1f} rad/s [{om_min:.1f}, {om_max:.1f}] | Mean Vy={vy_mean:.4f} m/s | CD_paper={cd_paper:.3f}")
