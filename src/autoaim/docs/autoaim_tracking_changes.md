# Autoaim Tracking Changes

> **SUPERSEDED IN PART (spinning-target revision).** A later pass that targeted
> spinning-enemy aim removed or changed several items below. Specifically:
> the **adaptive `q_pos`** controller was REMOVED (`q_pos`/`q_yaw` are static now);
> the **four-face fire window** is capped so it no longer expands at close range;
> `vyaw_fire_threshold` was deleted (dead); `LEGACY_time_bias` is the new name of
> the fallback horizon; and a vyaw-from-timing **anti-spurious-jump gate** plus
> **dual-solution PnP yaw** were added. See `INSTRUCTIONS.md` →
> "Spinning-Target Tuning" for the current behavior. The sections below are kept
> as historical record of the earlier tuning pass.

This document records the tracking and fire-gating changes made during the
RoboMaster auto-aim tuning pass. It is meant to explain what changed, why it
changed, and what should be watched during live testing.

## Goals

- Reduce fire dropouts from single missed detection frames.
- Make fire-lock tolerances scale with target range instead of using one fixed
  angular threshold everywhere.
- Use four-face prediction only when the spin estimate is trusted.
- Adapt horizontal process noise online so stationary targets stay smooth while
  maneuvering targets remain responsive.
- Keep pitch-sign handling consistent with the observed micro firmware behavior.

## Pitch Sign Handling

The standard, hero, and sentry launch files use:

- `micro_pitch_feedback_opposite_sign: True`
- `micro_pitch_lock_opposite_sign: False`
- `gimbal.pitch_sign: -1.0`

These two pitch flags do different jobs:

- `micro_pitch_feedback_opposite_sign` corrects raw micro feedback before it is
  used for geometry, camera-to-odom projection, and PnP.
- `micro_pitch_lock_opposite_sign` converts raw pitch feedback into the same
  convention as the command echo for fire-lock comparison.

The observed logs showed raw pitch feedback and command echo had the same sign,
so lock-space inversion must be false. Changing feedback inversion to false
caused the geometry pitch to oscillate, so feedback inversion remains true.

## Tracking Grace And TEMP_LOST Fire

`track_grace_misses` allows the tracker to stay in `TRACKING` through a few
missed frames. This prevents one blurred or edge-on frame from instantly
stopping fire.

`fire_in_temp_lost` exists but is disabled in launch by default. If enabled,
TEMP_LOST firing is bounded by:

- `temp_lost_fire_max`
- `P(7,7) <= vyaw_conf_p_max`

The TEMP_LOST miss counter is restarted when entering TEMP_LOST so the
coast-fire window means frames in TEMP_LOST, not total missed frames including
the TRACKING grace period.

## Range-Scaled Fire Lock

The fire-lock tolerance is now:

```text
tolerance = armor_half_extent / range
```

Then it is clamped to:

```text
fire_lock_min <= tolerance <= fire_lock_max_*
```

The parameters are:

- `fire_lock_k_yaw`: armor half-width used for yaw tolerance.
- `fire_lock_k_pitch`: armor half-height used for pitch tolerance.
- `fire_lock_min`: long-range lower bound.
- `fire_lock_max_yaw`, `fire_lock_max_pitch`: close-range caps.

`fire_lock_yaw` and `fire_lock_pitch` remain legacy fixed thresholds and are
used as default values for the new max parameters when a launch file does not
override them.

Reasoning: a fixed threshold is too strict very close and arbitrary far away.
The armor plate has an angular size, so the lock window should follow that
geometry while still respecting explicit safety caps.

## Four-Face Prediction

Four-face prediction is now gated by spin confidence:

```text
P(7,7) <= vyaw_conf_p_max
```

The old gate used only `abs(vyaw) >= vyaw_fire_threshold`. That could allow
four-face prediction when spin was large but not yet reliable, and could reject
four-face prediction when spin was slow but well-estimated.

The launch files were also tightened to avoid making side faces too easy to
accept:

- `angular_window: 0.40`
- `window_ref_dist: 3.0`

## Adaptive q_pos

The tracker now adapts horizontal process noise for `xc` and `yc` only.
`za`, `yaw`, and `vyaw` remain fixed-noise models.

The controller computes 2D position NIS:

```text
nis = y_xy^T S_xy^-1 y_xy
```

For 2 degrees of freedom, the target expected value is 2. The controller keeps
an EMA of the NIS and scales `q_pos` multiplicatively:

```text
q_pos_scale = clamp(q_pos_scale * (nis_ema / 2)^beta,
                    q_adapt_min,
                    q_adapt_max)
```

The effective horizontal process noise is:

```text
q_pos_effective = q_pos * q_pos_scale
```

The launch files now use `q_pos: 5.0` as the stationary/glide base value. The
adaptive scale can raise it during maneuvers. `q_yaw` remains static because
spin is handled separately through face-jump timing and covariance confidence.

The `q_adapt_*` parameters are intentionally not sanitized in code. Keep:

- `q_adapt_lambda` in `[0, 1)`
- `q_adapt_beta > 0`
- `q_adapt_min > 0`
- `q_adapt_max >= q_adapt_min`

## Debug State And Viewer

`/debug_state` now includes:

- effective `q_pos`
- static `q_yaw`
- base `q_pos`
- `q_pos_scale`
- `nis_pos_ema`

The viewer displays them as:

```text
Q pos/yaw: effective/static (base B xS, NIS N)
```

Expected behavior:

- `nis_pos_ema` should sit near 2 when Q/R are consistent.
- `q_pos_scale` should rise during target maneuvers.
- `q_pos_scale` should fall toward its lower range when the target is still.

## Launch Tuning Summary

`standard.launch.py`, `hero.launch.py`, and `sentry.launch.py` now share the
same adaptive horizontal process-noise structure:

- `q_pos: 5.0`
- `q_adapt_enable: True`
- `q_adapt_lambda: 0.95`
- `q_adapt_beta: 0.30`
- `q_adapt_min: 0.5`
- `q_adapt_max: 8.0`

`q_yaw` is left robot-specific:

- standard: `10.0`
- hero: `20.0`
- sentry: `10.0`

Fire-lock range scaling is explicit in each launch with:

- `fire_lock_k_yaw: 0.0675`
- `fire_lock_k_pitch: 0.0625`
- `fire_lock_min: 0.015`
- `fire_lock_max_yaw: 0.12`
- `fire_lock_max_pitch: 0.10`

## Validation Checklist

After rebuilding and sourcing the workspace:

1. Confirm pitch is stable and does not oscillate vertically.
2. Watch `/debug_state.pitch_lock` and `/debug_state.pitch_cmd_echo`.
3. Confirm single-frame missed detections do not immediately drop fire.
4. Watch `Q pos/yaw` in the viewer while moving a target.
5. Verify detector FPS separately from viewer/debug FPS.
6. If side-face selection looks too eager, lower `angular_window` or
   `vyaw_conf_p_max`.
7. If stationary aim jitters, lower `q_adapt_min`, lower base `q_pos`, or reduce
   `q_adapt_beta`.
8. If moving targets lag, raise `q_adapt_max`, base `q_pos`, or `q_adapt_beta`
   carefully.
