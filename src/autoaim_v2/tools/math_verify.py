#!/usr/bin/env python3
"""
Numeric verification study for the RoboTO 1v1 auto-aim redesign.
Run inside the ROS container (numpy available).

Sections:
  1. Latency -> miss distance at the plate (why every ms matters)
  2. Ballistics: quadratic-drag RK4, Newton pitch solve, vs no-drag closed form
  3. Measurement noise anatomy: bearing vs PnP depth (spherical EKF justification)
  4. Spin-fire: spray duty cycle vs phase-timed shots hit probability
  5. EKF omega convergence sim (spherical obs + yaw-optimized armor angle)
  6. Two-plate visibility geometry
"""
import numpy as np

G = 9.81

# ---------------------------------------------------------------- section 1
print("=" * 70)
print("1. LATENCY -> MISS AT PLATE")
print("=" * 70)
plate_half_w = 0.135 / 2          # small armor width 135 mm
r = 0.20                          # armor ring radius
for rpm in (100, 200, 300, 400):
    w = rpm * 2 * np.pi / 60
    for dt_ms in (1, 5, 10, 20):
        miss = w * dt_ms * 1e-3 * r   # tangential displacement of plate center
        print(f"  rpm={rpm:3d} (w={w:5.1f} rad/s)  dt={dt_ms:2d} ms -> plate moves "
              f"{miss*1000:6.1f} mm ({100*miss/plate_half_w:5.1f}% of half-width)")

# translation case
for v in (1.0, 2.0, 3.0):
    for dt_ms in (5, 10, 20):
        miss = v * dt_ms * 1e-3
        print(f"  translating v={v:.0f} m/s dt={dt_ms:2d} ms -> {miss*1000:5.1f} mm")

# ---------------------------------------------------------------- section 2
print()
print("=" * 70)
print("2. BALLISTICS: quadratic drag RK4 + Newton pitch solve")
print("=" * 70)
# 17 mm TPU pellet
m = 3.2e-3
d_pellet = 16.8e-3
A = np.pi * d_pellet ** 2 / 4
Cd = 0.47
rho = 1.204
k_over_m = 0.5 * rho * Cd * A / m   # [1/m]
print(f"  k/m = {k_over_m:.5f} 1/m")

def fly(v0, pitch, x_target, dt=1e-3):
    """Integrate planar trajectory with quadratic drag until x >= x_target.
    Returns (t, y) at x_target (linear interp on last step)."""
    vx = v0 * np.cos(pitch)
    vy = v0 * np.sin(pitch)
    x = y = t = 0.0
    while x < x_target and t < 2.0:
        v = np.hypot(vx, vy)
        ax = -k_over_m * v * vx
        ay = -G - k_over_m * v * vy
        # RK2 midpoint is plenty at dt=1ms; use it
        vxm = vx + 0.5 * dt * ax
        vym = vy + 0.5 * dt * ay
        vm = np.hypot(vxm, vym)
        axm = -k_over_m * vm * vxm
        aym = -G - k_over_m * vm * vym
        x_new = x + vxm * dt
        y_new = y + vym * dt
        vx += axm * dt
        vy += aym * dt
        if x_new >= x_target and x_new > x:
            f = (x_target - x) / (x_new - x)
            return t + f * dt, y + f * (y_new - y)
        x, y, t = x_new, y_new, t + dt
    return t, y

def solve_pitch(v0, gd, dz, iters=6):
    """Newton iteration on pitch so that y(gd) == dz."""
    pitch = np.arctan2(dz, gd)
    for _ in range(iters):
        t, y = fly(v0, pitch, gd)
        # derivative by finite difference
        eps = 1e-4
        _, y2 = fly(v0, pitch + eps, gd)
        dydp = (y2 - y) / eps
        if abs(dydp) < 1e-9:
            break
        pitch -= (y - dz) / dydp
    t, y = fly(v0, pitch, gd)
    return pitch, t, y

v0 = 25.0
print(f"  v0 = {v0} m/s")
print("   gd    dz   | pitch(deg) t_fly(ms) resid(mm) | nodrag t(ms) drop_diff(mm)")
for gd in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
    for dz in (-0.3, 0.0, 0.3):
        pitch, t, y = solve_pitch(v0, gd, dz)
        resid = (y - dz) * 1000
        # no-drag closed form for comparison
        t_nd = gd / (v0 * np.cos(pitch))
        # drop difference if we had aimed with no-drag model at same pitch
        y_nd = gd * np.tan(pitch) - 0.5 * G * t_nd ** 2
        print(f"  {gd:4.1f} {dz:5.2f} | {np.degrees(pitch):8.3f} {t*1000:8.1f} "
              f"{resid:8.3f} | {t_nd*1000:8.1f} {abs(y - y_nd)*1000:8.1f}")

# ---------------------------------------------------------------- section 3
print()
print("=" * 70)
print("3. MEASUREMENT NOISE ANATOMY (960x600, fx~730)")
print("=" * 70)
fx = 730.0
W_plate = 0.135
sig_px = 0.7   # keypoint localization sigma
for dist in (1.5, 3.0, 4.5, 6.0):
    w_px = W_plate * fx / dist
    sig_bearing = sig_px / fx                      # rad
    sig_lateral = sig_bearing * dist
    sig_depth = dist / w_px * sig_px * np.sqrt(2)  # both edges move
    print(f"  d={dist:3.1f} m: plate {w_px:5.1f} px | bearing sigma {sig_bearing*1e3:5.2f} mrad "
          f"({sig_lateral*1000:5.1f} mm) | PnP depth sigma ~{sig_depth*1000:6.1f} mm "
          f"| ratio {sig_depth/sig_lateral:5.1f}x")

# ---------------------------------------------------------------- section 4
print()
print("=" * 70)
print("4. SPIN FIRE: spray duty vs timed shots")
print("=" * 70)
duty = 4 * plate_half_w * 2 / (2 * np.pi * r)   # fraction of time a plate covers center ray
print(f"  center-hold spray duty (r={r}, plate {2*plate_half_w*1000:.0f} mm): {duty*100:.1f}%")

def timed_hit_prob(rpm, sigma_arrival_ms):
    w = rpm * 2 * np.pi / 60
    half_window = (plate_half_w / r) / w   # seconds: |phase| < wh/r
    # P(|err| < half_window), err ~ N(0, sigma)
    from math import erf, sqrt
    return erf(half_window / (sigma_arrival_ms * 1e-3) / sqrt(2))

# arrival-time error budget
tx_quant = 1.5        # event-driven serial quantization ms (rms)
feeder_sig = 6.0      # feeder single-shot delay dispersion ms (MEASURE!)
phase_sig_ms = 3.0    # EKF phase error expressed in ms at 200 rpm (see sec 5)
for v_sig_pct in (1.5, 3.0):
    for dist in (2.0, 4.0):
        t_f, _ = fly(25.0, 0.0, dist)
        v_sig_ms = t_f * v_sig_pct / 100 * 1000
        sigma = np.sqrt(tx_quant**2 + feeder_sig**2 + phase_sig_ms**2 + v_sig_ms**2)
        print(f"  sigma_arrival (v_sig={v_sig_pct}%, d={dist} m, t_f={t_f*1000:.0f} ms) "
              f"= {sigma:.1f} ms")
        for rpm in (100, 200, 300, 400):
            p = timed_hit_prob(rpm, sigma)
            print(f"    rpm={rpm:3d}: timed p_hit={p*100:5.1f}%  (spray {duty*100:.0f}%)")

# ---------------------------------------------------------------- section 5
print()
print("=" * 70)
print("5. EKF OMEGA CONVERGENCE (simplified sim, 120 Hz, yaw-opt armor angle)")
print("=" * 70)
rng = np.random.default_rng(7)
def sim_omega_convergence(rpm, n_frames=60, sig_armor_yaw_deg=2.0):
    w_true = rpm * 2 * np.pi / 60
    dt = 1 / 120
    # states: [theta, w]; obs: armor angle of visible face (mod pi/2 jumps folded)
    x = np.array([0.0, 0.0])
    P = np.diag([0.2 ** 2, 20.0 ** 2])
    q_w = 200.0  # angular accel noise
    R = np.radians(sig_armor_yaw_deg) ** 2
    errs = []
    theta_true = 0.0
    for i in range(n_frames):
        theta_true += w_true * dt
        F = np.array([[1, dt], [0, 1]])
        Q = q_w * np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
        x = F @ x
        P = F @ P @ F.T + Q
        z = theta_true + rng.normal(0, np.sqrt(R))
        # fold measurement to nearest face vs prediction (like association)
        k = np.round((z - x[0]) / (np.pi / 2))
        z_eff = z - k * np.pi / 2
        H = np.array([[1.0, 0.0]])
        S = H @ P @ H.T + R
        K = P @ H.T / S
        x = x + (K * (z_eff - x[0])).ravel()
        P = (np.eye(2) - K @ H) @ P
        errs.append(abs(x[1] - w_true))
    return errs

for rpm in (120, 240, 360):
    errs = sim_omega_convergence(rpm)
    w_true = rpm * 2 * np.pi / 60
    # phase error over a 150 ms prediction horizon caused by omega error
    for n in (12, 24, 48):
        e = errs[n - 1]
        phase_mm = e * 0.150 * r * 1000
        print(f"  rpm={rpm:3d}: after {n:2d} frames ({n/120*1000:3.0f} ms) "
              f"|w_err|={e:5.2f} rad/s -> {phase_mm:5.1f} mm over 150 ms horizon")

# ---------------------------------------------------------------- section 6
print()
print("=" * 70)
print("6. TWO-PLATE VISIBILITY")
print("=" * 70)
# plates at theta and theta+90; both 'visible' if obliquity < max_oblique (65 deg)
max_obl = np.radians(65)
# face i visible when |angle(face_normal, -LOS)| < max_obl
# faces at phi and phi+90 -> both visible when phi in (90-max_obl, max_obl)
lo = np.pi / 2 - max_obl
hi = max_obl
frac = max(0.0, (hi - lo)) / (np.pi / 2)
print(f"  with max_oblique=65deg: both plates visible {frac*100:.0f}% of spin phase")
print(f"  -> two-plate EKF update available ~{frac*100:.0f}% of frames on a spinner")
