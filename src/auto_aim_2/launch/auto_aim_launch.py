from launch import LaunchDescription
from launch_ros.actions import Node

# =============================================================================
# PARAMETER TUNING GUIDE — read this before touching anything on the field
#
# QUICK FIELD CHECKLIST:
#   1.  barrel_offset_z       Measure with ruler: camera centre → barrel exit, negative = barrel below
#   2.  bullet_speed          Measure with chronograph; wrong value = systematic height miss
#   3.  time_bias             System latency (detection + serial + gimbal). Start 0.025, add 0.010/step
#   4.  fire_lock_yaw/pitch   How tightly the gimbal must be on target before firing.
#                             Too tight → never fires.  Too loose → misses edge of plate.
#   5.  gimbal.yaw_sign /     If gimbal moves in wrong direction flip to -1.0
#       gimbal.pitch_sign
#   6.  chassis_heading_index Once micro firmware sends chassis heading in RX packet,
#                             set to the correct index (e.g. 4). Until then leave -1.
#   7.  ego_velocity_available Set to False until micro sends validated vx/vy.
#                              Only flip to True after verifying with ros2 topic echo /micro_status
# =============================================================================

def generate_launch_description():

    # ── AUTO-AIM NODE ─────────────────────────────────────────────────────────
    auto_aim_params = [

        # ── DETECTOR INPUT ──────────────────────────────────────────────────
        # target_classes:
        #   '0' = blue armor,  '3' = red armor.  Change for your alliance.
        {'target_classes': ['0']},
        {'use_keypoints': True},
        {'keypoint_topic': '/detector/armors_keypoints'},
        # min_keypoint_score: raise to 0.3–0.5 if you get false detections on walls/floor.
        {'min_keypoint_score': 0.0},
        # max_reproj_error: max reprojection error [px] before PnP result is rejected.
        # Average pixel distance between original keypoints and the keypoints obtained
        # by re-projecting the 3D PnP solution. A high value means the geometry didn't
        # fit well — usually a sign of bad keypoint detection or wrong armor size model.
        # 25 px is permissive enough to accept noisy keypoints from a tired model
        # while still rejecting obvious geometric nonsense. Reduce to 10–15 if you
        # see accurate keypoints generating jittery distance/PnP. Raise to 40+ if
        # too many valid detections are being rejected (look for REJECTED logs).
        {'max_reproj_error': 25.0},

        # ── DETECTION FILTERS ───────────────────────────────────────────────
        {'light_ratio': 0.85},
        # max_armor_distance: hard range filter [m] — reject if armor XY > this.
        {'max_armor_distance': 6.0},
        # max_armor_z: reject armors above this height [m] (removes ceiling reflections).
        {'max_armor_z': 4.0},

        # ── TRACKER STATE MACHINE ────────────────────────────────────────────
        # confirm_frames: consecutive detections before DETECTING → TRACKING.
        # At 70 Hz: 3 frames = 43 ms.  Reduce to 2 for faster initial lock.
        {'confirm_frames': 3},
        # lost_timeout [s]: coast time before TEMP_LOST → LOST.
        # At 70 Hz this is 0.30 / (1/70) ≈ 21 frames of coasting.
        {'lost_timeout': 0.30},

        # ── EKF PROCESS NOISE ────────────────────────────────────────────────
        # q_pos: how much the target XY position can accelerate [m/s²]².
        # Higher → tracker reacts faster to sudden direction changes but noisier.
        {'q_pos': 5.0},
        # q_yaw: how much the spin rate can change per step.
        # Higher → tracker reacts faster to RPM changes (spin-up/spin-down).
        # For spinning-top robots that change speed: raise to 15–20.
        {'q_yaw': 10.0},
        {'q_r': 1e-6},

        # ── EKF MEASUREMENT NOISE ────────────────────────────────────────────
        # r_pos_base + r_pos_slope*dist = position noise [m].
        # Higher → trust detection less, smoother but slower response.
        {'r_pos_base': 0.05},
        {'r_pos_slope': 0.04},
        {'r_yaw_base': 0.05},
        {'r_yaw_slope': 0.005},
        # max_oblique_deg: beyond this angle the armor yaw measurement is ignored.
        {'max_oblique_deg': 65.0},

        # ── VELOCITY DAMPING ─────────────────────────────────────────────────
        # alpha_pos: per-frame damping on XY velocity at ref_freq.
        # 0.99 ≈ very light damping — target velocity is preserved across frames.
        # Reduce toward 0.90 if you want faster velocity decay when target stops.
        {'alpha_pos': 0.98},
        # alpha_yaw: per-frame damping on spin rate (vyaw).
        # 1.00 = NO damping — the EKF freely tracks any spin rate the measurements
        # support (200 RPM, 300 RPM, spin-up, spin-down).
        # DO NOT reduce below 0.97 for spinning-top opponents.
        {'alpha_yaw': 1.00},
        # alpha_coast: damping while TEMP_LOST (no measurement).
        # 0.95 decays velocity to ~13% in 0.4 s — keeps prediction alive briefly.
        {'alpha_coast': 0.95},

        # ── ARMOR GEOMETRY ───────────────────────────────────────────────────
        # initial_radius: first guess for robot center-to-armor distance [m].
        # Standard RoboMaster: small robot ≈ 0.24 m, hero ≈ 0.28 m.
        {'initial_radius': 0.24},
        # radius_ema_alpha: how fast radius adapts from measurements.
        {'radius_ema_alpha': 0.05},

        # ── BALLISTICS ───────────────────────────────────────────────────────
        # bullet_speed [m/s]: MEASURE WITH CHRONOGRAPH.
        # A 1 m/s error causes ~1–2 cm height miss at 3 m (≈ 15% of plate height).
        {'bullet_speed': 25.0},
        {'gravity': 9.8},
        # gimbal_height [m]: height of camera/gimbal pivot above ground.
        {'gimbal_height': 0.420},

        # ── BARREL OFFSET ────────────────────────────────────────────────────
        # Offset from camera optical centre to barrel exit, in gimbal body frame [m].
        #   x: forward  (positive = barrel further forward than camera)
        #   y: left      (positive = barrel to the left of camera)
        #   z: up        (NEGATIVE if barrel is below camera — typical RoboMaster setup)
        #
        # HOW TO MEASURE: use a ruler from the centre of the camera lens to the
        # barrel centre at the muzzle.
        # Start with z = −(measured mm)/1000.  Tune by firing at a static target
        # at 2 m: if shots land below aim point, make z less negative (closer to 0).
        {'barrel_offset_x': 0.0},
        {'barrel_offset_y': -0.03},
        {'barrel_offset_z': -0.0},    # ← TUNE: measure on your robot

        # ── FIRE GATE ────────────────────────────────────────────────────────
        # angular_window [rad]: maximum angle between the face normal and the
        # barrel bearing for the face to be considered "hittable".
        # Too wide → fires at oblique faces (ball skims edge or misses).
        # Too narrow → never fires.
        # At 3 m, armor half-width subtends ≈ 1.3°.  A 7° window gives ~5× margin.
        # Tune: if you get hits on the side frame, reduce; if it never fires, raise.
        {'angular_window': 3.14},       # ← TUNE: 0.10–0.18 rad (6°–10°)
        {'window_ref_dist': 1.0},
        {'min_fire_dist': 0.2},
        {'max_fire_dist': 6.0},

        # ── TIMING / PREDICTION ──────────────────────────────────────────────
        # time_bias [s]: TOTAL system latency added to bullet flight time.
        # = detection pipeline latency + serial round-trip + gimbal response time.
        # Typical breakdown:
        #   detection @ 70 Hz → frame latency ≈ 14 ms
        #   serial TX/RX round-trip ≈ 10–15 ms
        #   gimbal mechanical response ≈ 10–20 ms
        #   → start at 0.025 s and increase in 0.010 s steps.
        # Symptom of too large: aim lands behind the plate (over-predicts rotation).
        # Symptom of too small: aim lands in front of the plate.
        {'time_bias': 0.025},           # ← TUNE: 0.020–0.060 s
        # ref_freq: SET THIS TO YOUR ACTUAL DETECTION RATE.
        # Check with: ros2 topic hz /detector/armors_keypoints
        # Used in EKF damping formula: alpha^(dt * ref_freq).
        # Wrong value → over/under-damping regardless of alpha settings.
        {'ref_freq': 70.0},             # ← SET: measure with ros2 topic hz

        # ── MATCH GATES ──────────────────────────────────────────────────────
        # max_match_dist [m]: max distance between predicted and measured armor.
        {'max_match_dist': 0.5},
        # maha_threshold: Mahalanobis gate for data association.
        # χ²(4) at 95% = 9.49 (strict), at 99% = 13.3 (permissive).
        # 13.3 is more forgiving for noisy keypoint PnP — detections that are slightly
        # off-predicted still associate instead of being rejected. With strict 9.49
        # you may see frequent re-init via the divergence path in handleArmorJump.
        # If you observe false associations (tracker briefly snaps onto a wrong robot
        # passing by), lower to 9.49. If you observe the tracker frequently losing
        # the target and re-initializing, raise toward 16.3 (99.5%) or 18.5 (99.9%).
        {'maha_threshold': 13.3},

        # ── TARGET SWITCHING ────────────────────────────────────────────────
        {'switch_range_ratio': 0.85},
        {'switch_cooldown': 10},
        # same_target_identity_dist [m]: how close two detections must be to
        # count as the same physical robot. class_id from YOLO is COLOR not
        # identity; with multiple enemies of the same color we need spatial
        # proximity to decide same-vs-different. If a new detection is within
        # this distance of the current target's predicted face, treat as the
        # same robot (no switch). Otherwise treat as different.
        # Tune up if robots can be back-to-back. Tune down if two passing
        # enemies get merged into one track.
        {'same_target_identity_dist': 1.0},

        # ── COMMAND SMOOTHING / LIMITS ───────────────────────────────────────
        # cmd_smooth_alpha: EMA on the yaw/pitch destination sent to the gimbal.
        # 0.85 means 85% of the new target is applied each frame → ~1 frame lag.
        # Reduce if yaw still feels sluggish; 1.0 = no smoothing.
        {'cmd_smooth_alpha': 1.00},
        # cmd_deadband_yaw/pitch [rad]: below this error the axis is held still.
        # Must be LESS than fire_lock_yaw/pitch.
        {'cmd_deadband_yaw': 0.005},
        {'cmd_deadband_pitch': 0.005},
        # cmd_rate_limit_yaw/pitch [rad/s]: 0 = disabled.
        {'cmd_rate_limit_yaw': 0.0},
        {'cmd_rate_limit_pitch': 0.0},
        # fire_lock_yaw/pitch [rad]: gimbal must be within this error before firing.
        # At 3 m, 0.035 rad ≈ 10.5 cm — roughly the armor half-width.
        # Tighter = fewer misses but harder to satisfy. Start at 0.05 and reduce.
        {'fire_lock_yaw': 0.025},        # ← TUNE: 0.025–0.060 rad
        {'fire_lock_pitch': 0.12},      # ← TUNE: 0.020–0.050 rad
        # micro_pitch_feedback_opposite_sign: True if the micro reports pitch with
        # opposite sign from the command (e.g. command −0.10 → feedback +0.10).
        # Verify with: ros2 topic echo /micro_status  while pointing up/down.
        {'micro_pitch_feedback_opposite_sign': True},

        # ── COMMAND SAFETY ───────────────────────────────────────────────────
        {'cmd_hold_time': 0.25},
        # cmd_max_delta_yaw [rad]: max one-frame jump in yaw command.
        # Raised to 0.80 to allow fast correction after a face-jump (armor switch).
        {'cmd_max_delta_yaw': 0.80},
        {'cmd_max_delta_pitch': 0.80},
        {'require_aim_inside_frame': False},

        # ── EGO-MOTION COMPENSATION ──────────────────────────────────────────
        # Compensates for our robot's own movement so the EKF does not mistake
        # our translation for target velocity.
        #
        # ENABLE ONLY AFTER: (a) basic fire gate is working without it, and
        # (b) you have verified that /micro_status[2] and [3] are valid vx/vy.
        {'use_ego_motion_compensation': True},
        #
        # ego_velocity_available: THE SAFETY SWITCH.
        #   False = micro is NOT sending valid vx/vy yet (or quality is unknown).
        #           Ego-motion integration is completely disabled. robot_x_/robot_y_
        #           stay at zero. Safe for all stationary tests and early field tests.
        #   True  = micro IS sending validated vx/vy in /micro_status[2] and [3].
        #           Only flip this AFTER verifying data quality with:
        #             ros2 topic echo /micro_status
        #           and confirming vx/vy look reasonable while driving.
        {'ego_velocity_available': False},  # ← SET True once micro sends good vx/vy
        #
        # ego_velocity_body_frame: True = micro sends velocity in chassis body frame.
        # False = micro sends velocity already in world frame (preferred).
        # If True, the chassis heading from chassis_heading_index_ is used to
        # rotate to world frame. Without a valid chassis heading the rotation
        # falls back to the gimbal yaw (WRONG during chassis spin).
        {'ego_velocity_body_frame': True},
        {'ego_velocity_scale_x': 1.0},
        {'ego_velocity_scale_y': 1.0},
        {'ego_velocity_max': 3.0},
        # ego_position_max_drift [m]: dead-reckoning reset threshold.
        # When integrated robot position drifts beyond this, BOTH robot_x_/y_ AND
        # the tracker's EKF state are shifted by the same delta so the target
        # doesn't appear to jump in the tracker's frame.
        # DEFAULT 0.0 = disabled. Set to ~half the arena diagonal (e.g. 4.0)
        # ONLY after you've validated that velocity quality is good and the
        # frame-shift behaves correctly across the reset boundary.
        {'ego_position_max_drift': 0.0},
        # chassis_heading_index: index in /micro_status for chassis absolute yaw [rad].
        # -1 = not available (falls back to gimbal yaw with a warning).
        # Once the micro firmware adds chassis heading to the RX packet, set this
        # to the correct index (e.g. 4 if it comes after vx/vy at [2] and [3]).
        {'chassis_heading_index': -1},  # ← SET: once micro firmware is updated

        # ── GIMBAL SIGNS ─────────────────────────────────────────────────────
        # Flip to -1.0 if the gimbal moves in the wrong direction on that axis.
        # Do NOT use this to fix a visualisation offset — it changes the command.
        {'gimbal.yaw_sign': 1.0},
        {'gimbal.pitch_sign': -1.0},
    ]

    # ── SERIAL BRIDGE NODE ────────────────────────────────────────────────────
    serial_params = [
        # Check port with:  ls /dev/tty{ACM,USB}*
        # For a stable name add a udev rule (see serial node comments).
        {'serial_port': '/dev/ttyACM0'},
        {'serial_baudrate': 500000},
        # serial_tx_hz: rate at which TX packets are sent to the micro.
        # Must be <= micro RX handler rate. 100 Hz matches a 1 kHz micro loop.
        {'serial_tx_hz': 100.0},
        # serial_reconnect_interval [s]: how often to retry if port is down.
        {'serial_reconnect_interval': 2.0},
        # serial_rx_timeout [s]: if no valid RX for this long, reconnect.
        # Covers micro reflash, cable unplug, firmware hang.
        {'serial_rx_timeout': 3.0},
    ]

    # ── VIEWER NODE ───────────────────────────────────────────────────────────
    viewer_params = [
        {'micro_pitch_feedback_opposite_sign': False},
    ]

    return LaunchDescription([
        Node(
            package='auto_aim_2',
            executable='serial_new_communication_USB_C_micro_imu_v9.py',
            name='micro_communications_node',
            parameters=serial_params,
            output='screen',
        ),
        Node(
            package='auto_aim_2',
            executable='auto_aim_2_node',
            name='auto_aim_2',
            parameters=auto_aim_params,
            output='screen',
        ),
        Node(
            package='auto_aim_2',
            executable='viewer_node.py',
            name='auto_aim_viewer',
            parameters=viewer_params,
            output='screen',
        ),
    ])
