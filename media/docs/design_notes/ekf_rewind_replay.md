# Design Note — EKF Rewind / Replay

Status: **design only**. Not implemented. The current code keeps the EKF
synchronous-to-latest-detection and uses a measured-latency prediction
lead instead. This note captures what a future rewind/replay change
would have to handle, so the work can be scoped without re-deriving it
in a hurry.

## Motivation

Today the prediction lead is

```
pred_t_total = (publish_time - image_stamp) + gimbal_response + ema_delay + flight_time
```

The first term is large (≈30..80 ms) and dominates the prediction error
budget. A rewind/replay scheme would (a) anchor the EKF state at the
image stamp of each measurement and (b) roll the state forward to *now*
only at command time. The lead then collapses to

```
pred_t_total = gimbal_response + ema_delay + flight_time
```

which is much smaller and much less affected by jitter in pipeline
timing.

## Required state history

Buffer the last *N* predictions, sized to cover the worst-case
detection latency at the lowest expected frame rate:

```
N = ceil(max_detection_latency_s * ref_freq) + safety_margin
  ≈ ceil(0.20 * 30) + 4 = 10
```

Each entry stores:

* timestamp (image stamp)
* `x_` (9-vector)
* `P_` (9x9, symmetric)
* `radius_`, `other_radius_`, `dz_`, `dz_initialized_`
* `last_yaw_` (for unwrap continuity)
* `state_`, `detect_count_`, `lost_count_`, `switch_cooldown_counter_`,
  `match_count_`, `miss_count_`, `last_match_time_s_`, `last_meas_quality_`

That is essentially the full Tracker state. Keeping it in a contiguous
ring instead of `std::deque` keeps the per-predict cost amortised.

## Replay loop

When a new measurement arrives at image stamp `t_meas`:

1. Find the buffer entry with the largest `t <= t_meas`. If none exists
   (measurement older than the buffer), discard the measurement and
   log a `STALE_MEASUREMENT` reject.
2. Restore Tracker state from that entry.
3. `ekfPredict(t_meas - t_anchor)` to roll forward to the measurement's
   own time.
4. Run the normal association + update path. Record the new state into
   the buffer, **invalidating** all entries with `t > t_meas`.
5. After the update, run a rolling forward predict to `now`, **without
   writing it back to the buffer** — the forward predict is for command
   generation only.

`computeAim` then uses the now-anchored state with the much smaller
`pred_t_total`.

## Risks

* **Inverse predict is not free**: the current `ekfPredict` uses
  damping factors `b = alpha^(dt * ref_freq)` which are not symmetric.
  Replay must use the actual `dt` between buffered samples; we cannot
  invert a single multi-step predict.
* **Radius EMA**: today `radius_` updates after each accepted
  measurement. Replay must reset `radius_` from the buffered anchor
  before re-running, otherwise an out-of-order measurement that
  bypassed the update path leaves a stale radius in the rolled-forward
  state.
* **Target switching**: `shouldSwitch()` reads the *current* target
  range. During replay it must read the buffered range at `t_anchor`,
  not the latest. Same for `switch_cooldown_counter_`.
* **Yaw unwrap**: `unwrapYaw` is path-dependent. Need to restore
  `last_yaw_` from the anchor, otherwise the wrap can flip between
  replay and the original run.
* **Mahalanobis association**: the gate uses the latest `P_`. Replay
  must use the anchor's `P_`, not the post-replay one.
* **Determinism**: the state after replay must match the state we
  would have computed had every measurement arrived in order. A unit
  test that runs detections in random order and compares to the
  in-order result is mandatory before merging.

## Tests

* `replay_round_trip`: feed N detections in-order, snapshot final
  state. Re-feed a permutation of those detections, compare.
* `replay_outlier`: feed an outlier, then a real detection at an
  earlier image stamp. Verify the outlier is overwritten cleanly.
* `replay_stale_drop`: feed a detection older than the buffer. Verify
  it is rejected without corrupting state.
* `replay_target_switch`: trigger a switch in the middle of a replay.
  Verify the cooldown counter is sourced from the anchor, not the
  current state.
* `replay_radius_ema`: verify `radius_` is bit-for-bit identical
  between replay and in-order runs at the boundary frame.

## Out of scope for this design

* Multi-target replay (i.e. two simultaneous tracks): keep the rewind
  single-target for now. Multi-target manager is a separate work item.
* IMU rewind: `/micro_imu` would also need a buffered history if the
  rotation at `t_anchor` is needed. For now, the camera-to-odom
  rotation is treated as latest-only.
