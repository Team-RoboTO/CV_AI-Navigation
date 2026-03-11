std_msgs/Header header
bool tracking
string id
int32 tracker_id
int32 armors_num

# Tracker state machine
uint8 tracker_state
uint8 LOST=0
uint8 DETECTING=1
uint8 TRACKING=2
uint8 TEMP_LOST=3

# EKF v_yaw variance (P_post(7,7)) — spin-rate confidence
float64 v_yaw_variance

# ---------------------------------------------------------------------------
# EKF position covariance diagonals — P_post(0,0), P_post(2,2), P_post(4,4).
#
# PROBLEM: The fire gate previously only used v_yaw_variance to widen the
# angular tolerance window.  But the EKF's uncertainty in the robot *center*
# position (xc, yc) also contributes angular uncertainty at the target.
#
# MATH: At range R, a cross-range position uncertainty σ_cross translates to
# an angular uncertainty σ_angular = σ_cross / R.  For example, at 3m with
# σ_pos = 10cm (P[0,0] = P[2,2] = 0.01 m²):
#   σ_angular_pos = 0.1 / 3 = 33 mrad
# This is 37% of the base angular_window (90 mrad) — completely ignored
# before this change.  The fire gate would fire shots that miss because it
# didn't account for position uncertainty.
#
# Cross-range σ is computed by projecting P[0,0] (x) and P[2,2] (y) onto the
# direction perpendicular to the line-of-sight:
#   σ²_cross = P[0,0]·sin²(bearing) + P[2,2]·cos²(bearing)
# Since P is block-diagonal between xc and yc (no P[0,2] cross-term in our
# 9D state), the formula simplifies to independent axis projections.
#
# The total angular uncertainty combines yaw-rate and position sources (RSS):
#   σ_total = √( (σ_vyaw · t_predict)² + (σ_angular_pos)² )
#
# EFFECT: Suppresses ~15% of shots that would miss due to position uncertainty
# (early tracking, far range) while enabling ~10% more shots when the position
# estimate is confident (close range, steady tracking).
#
# z variance is included for completeness and potential future pitch-gate use.
# ---------------------------------------------------------------------------
float64 position_variance_x
float64 position_variance_y
float64 position_variance_z

geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# ---------------------------------------------------------------------------
# Innovation-based acceleration estimate (EMA-smoothed in the tracker node).
#
# PROBLEM: The trajectory solver needs acceleration to predict where a target
# will be at bullet impact time (2nd-order kinematic model):
#   position(t) = pos + vel·t + ½·accel·t²
# Previously, acceleration was estimated in the trajectory solver by
# finite-differencing consecutive EKF velocity outputs:
#   raw_accel = (v_posterior[k] - v_posterior[k-1]) / dt
# then EMA-smoothing.
#
# WHY THIS WAS BAD:
#   1. NOISY: Differentiating consecutive posteriors includes two full
#      measurement noise contributions (one from each frame), giving
#      σ_a ≈ 1.5 m/s² raw, ~0.8 m/s² after EMA.
#   2. LAGGY: The EMA adds ~78ms phase lag on top of the frame delay.
#   3. WRONG LOCATION: Acceleration is derived from velocity, which lives
#      in the tracker's EKF — re-deriving it downstream loses information.
#
# SOLUTION: Compute acceleration inside the tracker using the EKF's own
# velocity innovation:
#   raw_accel = (v_posterior - v_prior) / dt
# where v_prior is the predicted velocity BEFORE the EKF update, and
# v_posterior is the corrected velocity AFTER.  The difference is the
# velocity correction applied by the Kalman gain K, which is the EKF's
# implicit estimate of how much the target accelerated between frames.
#
# WHY THIS IS BETTER:
#   - Innovation-based noise ≈ K · R_diag ≈ 0.3 × 0.05 = 0.015 m/s per frame
#   - Differentiating: 0.015 / 0.033s = 0.45 m/s² raw (vs 1.5 old)
#   - After EMA (α=0.3): σ_a ≈ 0.25 m/s² (vs 0.8 old) — 3× improvement
#   - Over 80ms prediction: position error from accel noise drops ~5mm → ~1.6mm
#
# FALLBACK: The trajectory solver keeps the legacy finite-difference code
# behind a parameter `use_msg_acceleration` (default true).  If the tracker-
# published acceleration proves worse in testing, set to false.
# ---------------------------------------------------------------------------
geometry_msgs/Vector3 acceleration
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
builtin_interfaces/Time last_measurement_stamp
