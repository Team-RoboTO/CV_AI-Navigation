# WORKLOG — RoboMaster / ARC Auto-Aim

> Companion to `INSTRUCTIONS.md`. `INSTRUCTIONS.md` = *how to build/run/tune*.
> `WORKLOG.md` = *what we changed, why, what is broken now, and how to resume
> the work from any device or any fresh chat session.*
>
> **Language:** English, to match the codebase and `INSTRUCTIONS.md`, so any
> teammate or AI session can pick it up.
> **Convention:** newest entries on top of each section. Every entry records
> *the reasoning*, not just the change. Convert relative dates to absolute.

---

## 0. How to resume this work (read this first)

If you are a new session / new device, do this in order:

1. Read `INSTRUCTIONS.md` (build, launch, parameter reference).
2. Read **§4 Active investigation** below — that is the live problem.
3. Pull the live signals while a robot is running:
   ```bash
   ros2 topic echo /debug_state           # full tracker/fire telemetry
   ros2 topic hz   /detector/armors_keypoints   # real detector rate (sets ref_freq)
   ros2 run rqt_image_view rqt_image_view /tracker/debug_image   # viewer HUD
   ```
4. The single most useful artifact is `/debug_state`. The field glossary is in
   **§6**. Captured samples live in `log1.txt`/`log2.txt`/`log3.txt` (root of
   repo) and are the basis of the §4/§4b/§4c diagnoses. `ricerca1.md` is the
   Chinese-pipeline survey + retrofit roadmap that drives §5; `ricerca2.md`
   adds later notes used in the §4c rework decision.
5. Key source files and what lives in them:

   | File | Role |
   |---|---|
   | `src/autoaim/src/tracker.cpp` | EKF, multi-face association, ballistics, **fire gate** (`computeAim`) |
   | `src/autoaim/include/autoaim/tracker.hpp` | `TrackerConfig` (all tunables + docs), state structs |
   | `src/autoaim/src/autoaim_node.cpp` | ROS glue, PnP→odom yaw, **command lock gate**, `/debug_state` publisher |
   | `src/autoaim/src/pnp_solver.cpp` | IPPE planar PnP, returns both yaw solutions |
   | `src/autoaim/launch/standard.launch.py` | **live parameter values** for the standard robot |
   | `src/autoaim/viewer_node.py` | HUD overlay (`/tracker/debug_image`) |
   | `src/autoaim/msg/DebugState.msg` | `/debug_state` schema |

6. State vector (9D EKF), referenced everywhere:
   ```
   x = [ xc, vxc, yc, vyc, za, vza, yaw, vyaw, r ]
        0   1    2   3    4   5    6    7     8
   ```
   `xc,yc,za` = robot CENTER in odom; `yaw,vyaw` = spin angle/rate of the robot;
   `r` = radius center→armor. A visible armor *face* is reconstructed from the
   center + a 90°·k offset by `predictFace()`. The target is **one physical
   robot with 4 faces**, never 4 targets.

---

## 1. System architecture (one paragraph)

YOLO keypoint detector → planar **PnP** (IPPE, both solutions) → TF to odom →
**9D spinning-top EKF** with **multi-face association** → 4-face prediction at
bullet-impact time → iterative **ballistic** solver (gravity) → gimbal angle
command (`/cmd_vel_AI`) → **two-stage fire gate**. The fire gate is the heart of
the current problem, so it is spelled out fully in §4.

Fire is permitted only if **both** stages pass:
- **Stage A — facing/aim-plan (tracker, `computeAim`)**: `aim.fire = tracking &&
  range∈[min,max] && margin ≥ 0`, where `margin = fire_window − |phase_error|`.
- **Stage B — gimbal lock (node)**: `command_locked = yaw_locked && pitch_locked`,
  using **range-scaled** tolerances (`fire_lock_k_*/range`, clamped).

`fire_decision = aim.fire && command_valid && command_locked && !holding`.

---

## 2. Change log — the big tracker refactor (DONE, was `prompt1.md`)

> This section absorbs the now-deleted `prompt1.md`. It was the spec for a
> structural refactor that has since been **implemented** (commit `8982bc9`,
> 2026-06-21, "new stuff"). Recorded here as *done*, with the reasoning, so the
> intent survives even though the prompt file is gone.

**Why the refactor happened.** The old tracker checked 4 faces in `computeAim()`
but the EKF *association/update* was still single-face (`armorFromState(x_)` +
`ekfMahalanobis(z)` with no face index). A clean detection of an *incoming* face
got compared against the *wrong* predicted face → high Mahalanobis → `matched=
false` → `TEMP_LOST` → stale yaw/vyaw → bad face scheduling → random high/low
aim and ~45° side shots. Adding face lookahead on top of a single-face EKF made
it worse, not better.

**Goal.** Turn an "armor-current-face tracker" into a **single physical-robot
tracker with multi-face measurement association** — *without* a full MHT/IMM/PLL.

**Implemented phases** (see `tracker.cpp`/`tracker.hpp`):

1. **Face-indexed model.** `predictFace(x, face_idx, t)` is the *single* source of
   face geometry (position + yaw + radius + z-offset). Replaces duplicated face
   math in association, switching, update, aim. `armorFromState()` is now just
   `predictFace(x,0,0).position`.
   ```
   offset_i = i·π/2 ;  θ_i = yaw + vyaw·t + offset_i
   r_i = (i odd) ? other_radius : r ;  z_i = (i odd) ? dz : 0
   face = (xc+vxc·t − r_i·cosθ_i,  yc+vyc·t − r_i·sinθ_i,  za+vza·t + z_i)
   ```
2. **Multi-face association.** Every detection is gated against **all 4 faces ×
   yaw-hypotheses** (`associateDetections`); the lowest-Mahalanobis face wins.
   No heap in the hot loop (`std::array<…,4>`).
3. **Face-indexed EKF update** (`ekfUpdateFace`, `ekfMahalanobisFace`) with a
   per-face Jacobian and **angular-wrapped** yaw innovation
   (`angles::shortest_angular_distance`). If `yaw_replaced_by_bearing`, yaw is a
   *position-only* update (`R(3,3)=1e6`) — a faked bearing-yaw never moves spin.
4. **`handleArmorJump()` removed.** A face change is now a *normal* association to
   a different `face_idx`. Spin is *observed* from face-index transitions
   (`observeFaceAssociation`) but the observer **never owns/resets the EKF**.
5. **Radius/dz policy.** Even faces use `radius_`, odd faces `other_radius_`;
   each updated only when its parity is confidently associated. `dz` frozen
   unless `adapt_dz_enable` (default **false**) — the old code learned a wrong dz
   sign from one bad jump → random high/low misses.
6. **Target switching by nearest predicted face** (`shouldSwitch`): a different
   face of the same robot is the same target (spatial identity, not class_id).
7. **Face lookahead = aim planning, not fire permission.** Lookahead only points
   the gimbal at an incoming face early; firing still needs `margin ≥ 0`.
8. **Honest phase confidence** (`phaseConfident()`): true if not spinning OR if
   recent accepted face-transition timing confirms the spin — **not** merely
   because `P(7,7)` is numerically small.
9–10. Kept Chinese-pipeline-inspired robot-centric multi-face tracking; launch
   defaults set for the standard robot.

**Acceptance intent (still the contract):** clean incoming-face PnP → `matched=
true`; `matched_face_idx` changes with the visible face; no `TEMP_LOST` just
because the face changed; `yaw_replaced_by_bearing` never updates yaw; `dz`
fixed unless enabled; **fire still requires `margin ≥ 0`**; no heap in the per-
face loop.

---

## 3. Change log — spinning-target (小陀螺) revision (DONE)

Captured in `INSTRUCTIONS.md → Spinning-Target Tuning`. Summary of the five
changes and the reasoning:

1. **Adaptive `q_pos` removed** — it adapted horizontal-center noise, but a
   spinner's center is ~still, so it reacted to yaw/PnP model error and
   overshot the lead. `q_pos`/`q_yaw` now static.
2. **Anti-spurious-jump gate on the spin estimator** — a PnP flip looks like a
   90° face jump; the old code locked a WRONG vyaw (80–100% blend + `P(7,7)→1.0`
   after ONE jump) → ~45° side shots. Now a jump drives vyaw only if PnP is
   clean (`vyaw_timing_max_reproj`), yaw is real, and **two consecutive
   same-direction estimates agree** (`vyaw_timing_consistency`).
3. **Dual-solution PnP yaw** — planar IPPE returns two yaw solutions; the
   detector passes both, the tracker keeps whichever is closer to predicted yaw.
   Kills frame-to-frame yaw flicker that polluted vyaw. *(NOTE: this same
   mechanism is implicated in the §4 bug — see "cause 3".)*
4. **Fire window no longer expands up close** — `win = angular_window ·
   min(window_ref_dist/range, 1.0)`. The old cap of 2.0 DOUBLED the facing
   tolerance up close → fired on ~46° off-facing plates ("hits the wheels").
   Capped at 1.0 it only shrinks far away.
5. **`alpha_yaw = 1.0`** (no spin damping) — a top spins at constant rate;
   damping vyaw only lags the phase prediction. *(NOTE: combined with `q_yaw=20`
   this is also implicated in §4 — see "cause 2".)*

---

## 4. ACTIVE INVESTIGATION — static fire indecision (2026-06-22)

### 4.1 Symptom (reported)
Both robots stationary, one directly in front of the other. Our robot **aims
correctly but flickers between FIRE and HOLD**; the viewer shows the block reason
is **`fire_window_margin`**. Historically (before the refactor) it also shot
crooked / low / 45° into the wheels and had long no-fire stretches.

### 4.2 The fire gate, exactly (so the math is unambiguous)
From `tracker.cpp::computeAim` (Stage A) and `autoaim_node.cpp` (Stage B):

```
aim.fire   = tracking && range∈[0.2, 6.0] && (margin ≥ 0)
margin     = win − |phase_error|
win        = angular_window · min(window_ref_dist / range, 1)
           = 0.35 · min(3.0/0.93, 1) = 0.35 rad   (≈ 20°)
phase_error= wrap( θ_impact − bearing_impact )
θ_impact   = yaw + vyaw · T            // plate normal at impact, T = pred_t ≈ 0.10 s
bearing    = atan2(face_y − barrel_y, face_x − barrel_x)   // line of sight to plate

command_locked = |yaw_err| ≤ tol_yaw  &&  |pitch_err| ≤ tol_pitch   // range-scaled
fire_decision  = aim.fire && command_valid && command_locked && !holding
```

`phase_error` is a **facing angle**: 0 when the plate normal points straight back
at the barrel; grows as the plate turns edge-on. The window only ever *shrinks*
with distance now.

### 4.3 Evidence from `log1.txt` (5 consecutive frames, static target)

| frame | det_yaw | state_yaw | vyaw [rad/s] | vyaw_rpm | phase | margin | yaw_lock | aim_fire | yaw_hyp |
|------:|--------:|----------:|-------------:|---------:|------:|-------:|:--------:|:--------:|:-------:|
| 1 | 0.116 | 0.186 | +0.046 | +0.44 | −0.274 | **+0.076** | ✅ | ✅ | 0 |
| 2 | 0.118 | 0.163 | −0.037 | −0.35 | −0.298 | **+0.052** | ✅ | ✅ | 0 |
| 3 | 0.101 | 0.148 | −0.124 | −1.19 | −0.323 | **+0.027** | ✅ | ✅ | 0 |
| 4 | 0.118 | 0.132 | −0.148 | −1.42 | −0.331 | **+0.019** | ✅ | ✅ | 0 |
| 5 | 0.023 | 0.126 | −0.294 | −2.81 | −0.372 | **−0.022** | ✅ | ❌ | **1** |

Invariant across all 5 frames: `yaw_locked = pitch_locked = true`,
`yaw_err ≈ 0.011`, `pitch_err ≈ −0.0076` — **the gimbal is dead-on the plate the
entire time**. The only thing toggling fire is Stage A's `margin ≥ 0`.

### 4.4 Root cause — three independent problems STACK

**Decompose:** `phase_error ≈ (θ_now − bearing) + vyaw·T`.

**Cause 1 — large STATIC facing error eats ~85% of the window.**
`θ_now − bearing ≈ 0.11 − 0.42 = −0.31 rad (≈18°)`. This is in the **raw PnP
measurement** (`det_yaw` vs the geometric bearing), *not* an EKF artifact: the
detector reports the plate normal at 0.11 while the line of sight is at ~0.42.
So even with `vyaw = 0`, `margin ≈ 0.35 − 0.31 = 0.04` — already on the edge.
*Either the enemy is genuinely ~18° off-square (normal, and exactly what the
facing window is meant to tolerate), or PnP yaw has a systematic bias (see §4.6
bench check).* Either way the design conclusion is the same (Fix 1).

**Cause 2 — phantom spin: `vyaw` wanders on a STATIC target.**
`q_yaw = 20` (launch) + `alpha_yaw = 1.0` (no damping) let vyaw absorb PnP-yaw
noise as fake rotation: it drifts +0.44 → −2.81 rpm in 84 ms. A near-facing
plate has the *least* observable yaw (small rotations barely change the image),
yet the measurement-noise model **trusts** near-facing yaw (`yaw_f = 1/|cosθ|⁴`
≈ 1 when facing). So noise flows straight into vyaw. The `vyaw·T` term then adds
≈ −0.03 rad — the final push over the edge.

**Cause 3 — IPPE dual-solution flip injects a yaw jump (the frame-5 trigger).**
Frames 1–4 use yaw hypothesis 0 (`det_yaw ≈ 0.11`); frame 5 flips to hypothesis
1 (`det_yaw = 0.023`). The tracker picks "whichever IPPE solution is closer to
predicted yaw", but the prediction has been drifting down (cause 2), so the
*alternate* solution eventually wins → 0.087 rad yaw step → vyaw spikes to
−2.81 rpm → `phase = −0.372` → `margin < 0` → **fire blocked**. Next frame it
recovers → flicker. This is a feedback loop: noise → vyaw drift → prediction
drift → solution flip → bigger vyaw spike.

**Net:** a perfectly-aimed, locked shot at a static plate is refused because the
plate sits ~18° off-square against a 20° window, and noise (vyaw drift + IPPE
flip) repeatedly tips the last 2° over the edge.

### 4.5 Fixes — **APPLIED 2026-06-22** (user chose "Fix 1+2+3 completo"; build OK)

Applied values & locations:
- **Fix 1** `tracker.cpp::computeAim` — facing gate now `spinning ? margin≥0 :
  |phase_error|≤static_facing_max`. New param `static_facing_max = 0.6` rad.
  `autoaim_node.cpp::fireBlockReason` reworked so `fire_window_margin` is reported
  only when the facing gate is the real blocker.
- **Fix 2** `tracker.cpp::measurementNoise` — U-shaped yaw trust
  `yaw_f /= max(sin²(a), yaw_facing_obs_floor)`, new param
  `yaw_facing_obs_floor = 0.05`. **`q_yaw 20 → 7`** in `standard.launch.py`.
- **Fix 3** `tracker.cpp::associateDetections` — IPPE alternate solution must beat
  the primary by `ippe_alt_penalty` Mahalanobis to be chosen; new param
  `ippe_alt_penalty = 1.0`. Stored `best.mahalanobis` stays raw (gate unaffected).

All three params declared in `autoaim_node.cpp` and set in `standard.launch.py`.
`hero.launch.py` / `sentry.launch.py` inherit the safe declared defaults; mirror
the launch entries there once validated on the standard robot.

**On-robot validation still required** (see §7). Original proposal text kept below.

---

#### Original proposal (reasoning preserved)

**FIX 1 (root, highest value, bullet-safe) — decouple aim-lock from the facing
window for non-spinning targets.**
The facing window exists to *time shots on a spinning plate*. When the target is
effectively static (`|vyaw| < face_lookahead_min_vyaw`, 0.35 rad/s ≈ 3.3 rpm),
the plate is at a fixed angle and the gimbal is already locked on its center →
fire on lock. Keep a loose sanity cap so a truly edge-on/garbage plate is still
rejected. Sketch (`computeAim`, replacing the final `margin ≥ 0` in `aim.fire`):
```cpp
const bool spinning = std::abs(x_(7)) >= cfg_.face_lookahead_min_vyaw;
const bool facing_ok = spinning
    ? (best.margin >= 0.0)                                   // strict for spinners
    : (std::abs(best.phase_error) <= cfg_.static_facing_max); // ~0.6 rad ≈ 34°
aim.fire = best.valid && fireStatePermits() && range_ok && facing_ok;
```
*Why bullet-safe:* fire still requires `command_locked` + range + a static-facing
sanity cap. You do not spray; you stop **refusing** shots you would land. This is
what strong teams do — gate static fire on hit geometry, not a tight angle.

**FIX 2 (reduce phantom spin) — cool `q_yaw` and/or damp vyaw, and fix the yaw
noise direction.**
- Lower `q_yaw` 20 → ~6–8, OR set `alpha_yaw ≈ 0.9` so vyaw decays toward 0 with
  no real spin trend. (Re-tune the spinner case afterward; §3 wanted 1.0 for
  spinners, so prefer lowering `q_yaw` and keep `alpha_yaw=1.0`.)
- Better: make `r_yaw` reflect real observability — inflate yaw noise **near
  face-on** too (currently it only inflates when oblique). A U-shaped yaw noise
  vs obliquity stops vyaw running away on a static-facing plate.

**FIX 3 (kill the flip trigger) — hysteresis on the IPPE yaw-hypothesis choice.**
Require the alternate solution to beat the primary by a margin before switching,
or freeze hypothesis switching when `|vyaw|` is small and the plate is near-
facing (where yaw is barely observable and barely matters). Removes the frame-5
step injection.

**Priority:** Fix 1 alone removes the reported indecision. Fixes 2 & 3 remove the
underlying noise so the system is robust (and helps spinning accuracy too).

### 4.6 Bench verification checklist (do before/with the fix)
1. **Is the 0.31 rad static facing error real cant or a PnP-yaw bias?**
   Place both robots *deliberately square*, read `/debug_state`: compare `det_yaw`
   to the bearing (≈ `cmd_yaw`). If `phase → ~0` → real cant (Fix 1 still right).
   If `phase ≈ 0.3` with squared robots → **systematic PnP-yaw bias**.
2. If biased, suspect the yaw extraction: `autoaim_node.cpp` takes yaw via
   `tf2::Matrix3x3(q_odom).getRPY(r,p,y)` on the *full* armor rotation. With the
   camera pitched −0.087 rad, RPY can couple pitch into yaw. Extracting the plate
   **normal vector** (`R.col(0)` → odom → `atan2(ny, nx)`) is more robust. A real
   yaw bias would also explain the historical "shoots crooked" symptom — worth
   10 minutes.
3. Confirm `ref_freq` matches the real detector rate (`ros2 topic hz
   /detector/armors_keypoints`); the launch sets 60.0.

### 4.7 Guardrails (do NOT regress these while fixing)
- Do **not** widen `angular_window` to paper over Cause 1 — that re-opens the
  ~45° side-shot on spinners (the §3.4 regression). Fix the *static* path
  instead (Fix 1).
- Do **not** raise `maha_threshold` to hide association errors.
- Do **not** re-enable adaptive `q_pos`.
- Keep `dz` frozen (`adapt_dz_enable=false`) until multi-face association is
  proven on a spinner.
- Fire must always still require `command_locked` and a finite, in-range command.

---

## 4b. ACTIVE INVESTIGATION — moving-target follow (overshoot/drift) (2026-06-22)

### 4b.1 Symptom (reported)
Following an enemy that **translates** into position: the aim **overshoots / drifts**
even though fire is (correctly) held by the window. Our robot was stationary, so
**ego-motion is disabled** (`robot_x/y = 0`, `robot_vx/vy = 0`) — the lead comes
purely from the target's EKF velocity. Source: `log2.txt`.

### 4b.2 Evidence from `log2.txt` (6 frames, ~60 Hz, enemy translating)

| field | trend | reading |
|---|---|---|
| `distance` | 2.50 → 2.55 m | receding ≈ 0.7 m/s |
| `cmd_yaw` | 0.346 → 0.323 | bearing slewing ≈ −0.35 rad/s (lateral motion) |
| `micro_yaw` vs `cmd_yaw` | `yaw_error ≈ 0` | gimbal **tracks the command tightly** (no servo lag) |
| `det_yaw` | 0.168,0.214,0.248,0.340,0.418 | **±14° frame-to-frame** despite `reproj 0.01–0.13 px` |
| `phase_error` | ≈ −0.08 | plate only ~5° off facing → **near-facing** |
| `vyaw` | ≈ −0.10 (−1.0 rpm) | **phantom spin** — the enemy is translating, not spinning |
| `pd` (pos innov) | 0.06–0.11 m | "accepted" only because R models ~0.15 m noise |
| `state_yaw` | ~0.27, smooth | EKF smooths the noisy `det_yaw` well (U-shape working) |

So: the **gimbal servo is not the problem** (it follows `cmd_yaw` to ~0 error).
The problem is that **`cmd_yaw` itself is a laggy/over-led estimate**.

### 4b.3 Root causes

**Cause A (dominant) — isotropic over-smoothing → lag + stop-overshoot.**
`measurementNoise` modelled position noise **isotropically**: `R = (r_pos_base +
r_pos_slope·range)²` on x,y,z = `(0.05+0.04·2.5)² = 0.15² m²` at 2.5 m. But a
monocular PnP fix is **precise in bearing** (pixel-accurate, ~1–2 cm laterally at
2.5 m) and **poor only in depth** (range). Forcing the precise lateral channel to
carry 15 cm of noise makes the EKF over-smooth → the velocity estimate **lags
during motion** and, crucially, **persists when the target stops** (the EKF takes
many frames to accept v→0 when it distrusts position) → the aim keeps leading a
stopped enemy = **overshoot**. `pd = 6–11 cm` was being swallowed as "within
noise" instead of correcting the track.

**Cause B — phantom `vyaw` from the EKF position↔yaw coupling.**
The measurement Jacobian couples armor position to spin: `H(0,6)=r·sinθ`,
`H(1,6)=−r·cosθ`. A translating target's position-lead error is partly
mis-attributed to **yaw**, driving a phantom `vyaw ≈ −0.10` with no real
rotation. `computeAim` then applied a **rotational lead** (`θ_impact = yaw +
vyaw·T`) to the aim → a small consistent lateral **drift** (~4 mm here; scales
with speed/closeness — a fast close pass could reach cm-level, and a large
phantom vyaw would otherwise be led on the held face even when 4-face prediction
is correctly suppressed).

**Cause C (already mitigated) — near-facing yaw is unobservable.**
`det_yaw` swings ±14° with sub-pixel reprojection — the classic planar
flip/face-on regime. The U-shape from §4 Fix 2 is doing its job (`state_yaw`
stays smooth), so this is *confirmation*, not a new bug. It is also why Cause B
matters: near face-on the yaw carries no real information, so any vyaw is suspect.

*Not changed this round (documented tuning levers):* `alpha_pos = 0.995` biases
the steady-state lead slightly low; `q_pos = 10` is responsive. Revisit only if
follow is still soft after the R split.

### 4b.4 Fixes — **APPLIED 2026-06-22** (build OK)

- **Anisotropic position noise** (`tracker.cpp::measurementNoise`): split into
  **radial** (along the line of sight, large, range-growing — keeps `r_pos_*`)
  and **tangential** (perpendicular, small — new `r_pos_tang_base=0.025`,
  `r_pos_tang_slope=0.010`; ~0.05 m at 2.5 m vs 0.15 m before). The 2×2 odom xy
  block is `diag(radial,tangential)` rotated by the bearing (eigenvalues stay
  positive → PD). Obliquity `xyz_f` still inflates both; vertical uses the
  tangential scale. Result: the EKF **follows the lateral motion ~3× tighter**
  (better tracking, faster stop-detection → less overshoot) while range stays
  stable. Revert by setting `r_pos_tang_* == r_pos_*`.
- **Rotational-lead gating** (`tracker.cpp::computeAim`): the `vyaw` lead is now
  applied to the aim only when the spin is **timing-confirmed**
  (`phase_timing_confident_ && |vyaw| ≥ face_lookahead_min_vyaw`); otherwise the
  aim uses a copy of the state with `vyaw = 0`. **Translation lead always
  applies.** Kills the phantom rotational drift on straight-moving enemies.

### 4b.5 Validation & tuning (on robot)
1. Strafing enemy: `cmd_yaw` should lead smoothly and **settle without
   overshoot when the enemy stops**. Watch `pd` — it should drop (tighter fit).
2. If lateral aim looks **jittery**, raise `r_pos_tang_base` toward 0.04–0.05.
   If it still **lags/over-smooths**, lower `r_pos_tang_slope` toward 0.005.
3. Confirm `vyaw` no longer produces visible lateral drift on a non-spinning
   mover; on a real spinner, `apply_spin_lead` must re-engage (needs
   `phase_timing_confident_`).
4. If steady-motion lead trails, try `alpha_pos = 1.0` (pure CV) next.

---

## 4c. ACTIVE INVESTIGATION — spinning enemy: vyaw never locks (2026-06-22)

### 4c.1 Symptom (reported)
Enemy **rotating**, our robot stationary: `vyaw` is completely wrong; the aim
sits beside the robot like a **ghost** mimicking its motion and shooting air;
never moves to the incoming armor / never fires (always `fire_window_margin`);
**huge jump** when aiming at a raised incoming plate; **very slow** to recover
when the spin stops; off-screen exit makes the aim drift. Source: `log3.txt`.

### 4c.2 Root cause — spin-bootstrap catch-22 (smoking gun)
`log3.txt`: `det_yaw≈0.7` but `state_yaw≈−1.28` (EKF reads the plate as **face 1**,
constant innovation `yd≈0.44` barely corrected); `vyaw` **ramps** 0.4→3.5 rpm (a
real 小陀螺 is 200–300 rpm) and never locks; `matched_face_idx` stuck at 1,
`jump_detected=false` every frame.

The refactor (prompt1.md Phase 4) **removed raw-yaw-jump detection** and made spin
timing depend on **`matched_face_idx` transitions**. But the face index only
changes if the EKF predicts a ~90° rotation → needs `vyaw≠0` → needs a detected
jump → needs a face transition. **Circular.** From spin-start (`vyaw≈0`) the first
handoff is never detected, so `vyaw` never bootstraps. Reconstructing
`armor = center − r·[cosθ,sinθ]` from the wrong yaw = the **ghost**. Wrong phase
from the wrong state = the **never-fires**. `alpha_yaw=1.0` (no decay) = **slow
spin-down**. `alpha_coast=0.98` (barely damped) = **off-screen drift**.

**It is the STATE that is fundamentally wrong, not the prediction** (lead at 1 m is
~0.1 s; aiming at "now" still misses). Also: the §4 static fix (`q_yaw↓` + U-shape)
**conflicts** with spinning — the two regimes need opposite yaw-trust.

### 4c.3 User prompts and review context (2026-06-23)

The 2026-06-23 session started as a review request, not an implementation request:
the user asked to inspect the changed code, logs, `WORKLOG.md`, and the intended
implementation before judging the previous agent's work. The critical prompt was
the `log3.txt` issue report, summarized in the user's own terms:

- there is still a lot of overshoot;
- when the enemy rotates and our robot is stationary, `vyaw` is completely wrong;
- armor following is bad: it does not move to the first/next armor, does not follow
  low-rate rotations well, and jumps hard when aiming at a raised incoming armor;
- it overestimates target motion in forward/back/side movement and visually aims
  too far;
- it keeps reporting no-fire due to window margin;
- when the robot exits the screen, our aim keeps drifting slightly in that exit
  direction;
- sometimes the aim sits far beside the enemy as if tracking a ghost robot, then
  takes too long to recalibrate after the spin stops.

Follow-up prompts asked whether the proposed `IMPLEMENTATION_PLAN.md` rework was
better than a quick fix / safety patch, then asked to also inspect `ricerca1.md`
and `ricerca2.md` for ideas. Final instruction: **apply the rework judged best at
the root of the problem.**

### 4c.4 Decision: apply a targeted root rework, not only a safety patch

The safety-only patch would have prevented some unsafe fire decisions, but it
would not have fixed the state corruption behind the ghost aim. The full plan had
larger long-term items (trajectory fire gate, latency ledger, gimbal delay model),
but those are not needed to break the immediate `log3` failure loop.

Chosen implementation slice:

1. **Separate regimes explicitly.** `phaseConfident()` is now the union of
   `confirmedSpin()` and `staticConfirmed()`. A target with unknown phase is no
   longer treated as "static enough" merely because `|vyaw|` is currently small.
2. **Bootstrap spin from raw visible-plate yaw handoff.** Reintroduce a raw-yaw
   handoff observer, but as a bounded timing input rather than the old EKF owner.
   This detects the first ~90° visible-plate jump before `matched_face_idx`
   transitions are reliable, breaking the catch-22.
3. **Keep face-index timing as a second source.** The old `observeFaceAssociation`
   timing path still feeds the same estimator once face indices really start
   changing, so the rework does not depend on raw yaw forever.
4. **Stabilize association before spin confirmation.** Add
   `face_index_switch_penalty`: before spin timing is confirmed, a different face
   must beat the held face by more than a tiny Mahalanobis fluctuation. This
   targets the wrong-face lock that produced the ghost robot.
5. **Aim at the observed armor while phase is unknown.** When spin is not
   confirmed, `computeAim()` no longer reconstructs all four faces from a yaw
   state that may be off by one face. It uses the last directly observed armor
   position/yaw plus translational lead only. Full four-face scheduling returns
   only after `confirmedSpin()`.
6. **Freeze geometry learning while phase is unknown.** Radius/dz updates from
   uncertain associations are blocked until either static or spin is confirmed.
   This prevents a raised/entering armor or wrong face label from poisoning the
   robot geometry.
7. **Make unknown phase conservative for firing.** The relaxed static-facing gate
   is allowed only under `staticConfirmed()`. Unknown phase and confirmed spin both
   require strict `margin >= 0`.
8. **Improve debug, not just behavior.** `/debug_state` now exposes
   `static_confirmed` and `confirmed_spin`, the viewer shows the active regime, and
   fire-block reasons distinguish `phase_unknown_window_margin`,
   `spin_window_margin`, and `static_facing_margin`.

Why this exact slice:

- It addresses the state failure first. In `log3`, prediction was not merely a bit
  too aggressive; the EKF was aiming from a wrong face/yaw state. More tuning on
  lead would not fix that.
- It keeps the old bullet-safety principle: no firing on unknown spinner phase.
- It avoids a full MHT/IMM/PLL rewrite. `ricerca1.md`/`ricerca2.md` both point
  toward disciplined robot-centric tracking, latency accounting, and trajectory
  fire decisions, but not toward adding heavy probabilistic machinery before the
  basic spin handoff is reliable.
- It preserves the 2026-06-22 static/moving fixes but removes their dangerous
  side effect: "small `vyaw`" no longer means "safe static target" until the target
  has actually been stable for multiple observations.

### 4c.5 Fixes — **APPLIED 2026-06-23** (build OK)

Files changed:

- `src/autoaim/include/autoaim/tracker.hpp`
  - Added `TrackerDebugInfo::{static_confirmed, confirmed_spin}`.
  - Added `TrackerConfig::face_index_switch_penalty` (default 4.0).
  - Added helpers/state for raw-yaw handoff bootstrap, last observed armor, regime
    checks, and shared spin-timing reset/feed functions.
- `src/autoaim/src/tracker.cpp`
  - Added `confirmedSpin()` / `staticConfirmed()` and changed `phaseConfident()` to
    require one of those real regimes.
  - Factored spin timing into `feedSpinTimingJump()`.
  - Added `observeRawYawHandoff()`: unwraps raw relative yaw (`yaw_meas - bearing`)
    and accepts only bounded ~90° jumps (`yaw_jump_thresh` to 2.2 rad) as visible
    armor handoffs.
  - Kept `observeFaceAssociation()` as a face-index timing source, now feeding the
    same jump estimator.
  - Added association hysteresis through `face_index_switch_penalty` before spin is
    confirmed.
  - Stored the last accepted observed armor and used it for aim while spin is not
    confirmed.
  - Blocked radius/dz learning while phase is unknown.
  - Changed facing/fire gate: confirmed spin => strict margin; static confirmed =>
    relaxed static cap; unknown phase => strict margin.
  - Reset spin timing/observed armor state on target loss.
- `src/autoaim/src/autoaim_node.cpp`
  - Declared `face_index_switch_penalty`.
  - Published `static_confirmed` / `confirmed_spin`.
  - Updated fire-block reason semantics for the three regimes.
- `src/autoaim/msg/DebugState.msg`
  - Added `static_confirmed` and `confirmed_spin`.
- `src/autoaim/viewer_node.py`
  - Added HUD regime line: spin/static/phase.
- `src/autoaim/launch/standard.launch.py`
  - Set `face_index_switch_penalty = 4.0` for the standard robot.

Verification:

```bash
git diff --check
colcon build --packages-select autoaim --symlink-install --allow-overriding autoaim
```

Both passed on 2026-06-23 after the rework.

### 4c.6 Known remaining risks / likely next changes

- **`vyawConfident()` name is now misleading.** It aliases `phaseConfident()`, which
  can be true for `staticConfirmed()`. This is acceptable for current code paths
  but should be renamed/deprecated or changed to `confirmedSpin()` in the debug
  surface to avoid future confusion.
- **Raw yaw handoff sign must be validated on robot.** The convention currently
  uses `yaw_dir = -sign(raw_yaw_jump)`, matching the four-face model convention.
  If `log4` shows accepted jumps with inverted `vyaw`, flip this sign.
- **Debug should expose jump source next.** Useful fields for `log4`: `jump_source`
  (`raw_yaw` vs `face_idx`), `raw_rel_yaw`, `raw_yaw_jump`,
  `raw_handoff_accepted`. This will make spin-bootstrap failures obvious.
- **Unknown-phase TEMP_LOST/coast should become more conservative.** If the target
  exits the screen and aim still drifts, damp or freeze translational velocity much
  faster while phase is unknown and detections are stale.
- **Longer-term clean design:** split the tracker into two modes:
  directly-observed armor tracker before phase confirmation, robot-center/four-face
  tracker after spin confirmation. The current rework already applies that idea to
  aim planning, but the EKF update still uses the center+face measurement model.

---

## 4d. ACTIVE INVESTIGATION — spin bootstrap dead + fire black-hole (2026-06-23, REWORK APPLIED)

### 4d.1 Symptoms (reported, this round)
Validating the §4c rework on the standard robot, three issues with the new build:
1. **Static**: `vyaw` looks off (a few −0.x rpm) on a robot that is dead still.
2. **Slightly moving enemy**: a shot that would clearly hit is never fired —
   always blocked by the window/phase margin. "Doesn't make sense."
3. **Hard spinner** (captured in `bagsa/debug_stateY20260623_172232`): never fires,
   always `phase_unknown_window_margin`. **The grave one.**

Artifacts: `log4_still.txt` (the STATIC case — it now fires fine) and the rosbag
`bagsa/…` (374 `/debug_state` msgs, the SPINNER case).

### 4d.2 Evidence — the bag is a real ~100 rpm spinner the tracker can't see
Deserialised the 374 msgs (`/tmp/analyze_bag.py`, `/tmp/spin_analysis.py`):

| Metric | Value | Reading |
|---|---|---|
| rate | **43 Hz**, dt 10–80 ms, 9 dropouts | launch `ref_freq=60` is wrong; dt irregular |
| handoffs (det_yaw steps >0.6) | 56, every **~147 ms**, 42 +dir / 14 −dir | **real spin ≈ 10.7 rad/s ≈ 102 rpm**, one way, IPPE-polluted |
| `confirmed_spin` | **0/374** | the §4c bootstrap NEVER fires |
| `jump_detected` / `vyaw_timing_accepted` | **0/374** | `observeRawYawHandoff` never calls the estimator |
| `matched_face_idx` | **0 for all 374** | face never advances (pinned) |
| `vyaw_rpm` | noise ±3 rpm, 101 sign flips, mean 0.10 | should be ~+100 rpm one-way |
| `fire_block_reason` | `phase_unknown_window_margin` **240/374**, `firing` 38/374 | matches symptom #3 |
| `aim_fire` / `cmd_fire` | 35% / **10%** | basically never shoots |
| counterfactual | `|phase|≤0.6`: **70%** fireable; `≤0.96` (55°): **86%** | tracking is fine, the GATE throws it away |

### 4d.3 Root cause — TWO defects that compound
**R1 — the spin bootstrap is UNREACHABLE.** `observeRawYawHandoff` needs a per-frame
`rel_yaw` step in `[yaw_jump_thresh=1.20, 2.2]` AND `one_face_jump` AND two
consecutive **same-direction** estimates consistent within ±35% with dt∈[25,700] ms.
On a real spinner the handoffs (~147 ms) are polluted by an IPPE flip almost every
other frame (assoc hypothesis 241 primary / 133 alternate), so "two consecutive
same-direction consistent" essentially never holds. On top of that the face index
is **pinned to 0** by `face_index_switch_penalty=4.0` (added pre-`confirmedSpin`), so
the second timing source (`observeFaceAssociation`) is dead too, and `q_yaw=7` +
the U-shaped yaw distrust (§4 Fix 2, tuned for STATIC) make the EKF refuse to track
the rotating yaw. Three independent locks all close the path to `confirmed_spin`.

**R2 — "unknown phase" is a fire black-hole.** The §4c gate was three regimes:
`confirmed_spin → margin≥0` / `static_confirmed → |phase|≤0.6` / **else → margin≥0
(strict)**. Anything that is neither spin-confirmed (R1 ⇒ never) nor a perfectly
stable static plate lands in the strict "unknown" bucket and is refused. That single
mechanism explains all three symptoms: #3 (spinner stuck unknown), #2 (a slightly
moving plate falls out of `static_confirmed` on vyaw noise but never reaches
`confirmed_spin` → unknown → strict), #1 (residual phantom vyaw from the same yaw
distrust). It is a **regime-classification** failure, not a tuning problem.

**Why the §4c raw-yaw bootstrap was the wrong primitive:** yaw is the IPPE-ambiguous
channel. Using a discrete yaw "handoff" to bootstrap spin, then hard-gating fire on
that confirmation, makes the whole system hostage to the noisiest signal.

### 4d.4 The rework (APPLIED 2026-06-23, build OK) — three pieces

**(A) Position-based spin observer (the real bootstrap).** `Tracker::
updateSpinObserver()` estimates spin from the ARMOR ORBIT, immune to the IPPE yaw
flip. Physics: the spin center sits behind the visible (front-facing) plate at one
radius ALONG THE LINE OF SIGHT, so the per-frame center estimate is `armor +
r·[cos(bearing), sin(bearing)]` (bearing = pixel-accurate; **not** the flipping PnP
yaw). The windowed mean of those is the center; the **median** armor angular
velocity about it is ω, with the ~90° handoff frames rejected as `|Δangle|>1.0`
outliers. Confirmed only with enough in-face samples, low robust spread (MAD), and
a sign majority. Offline validation (`/tmp/test_observer3.py`): ω=10.7 → est 10.0,
valid 100%; direction correct; **static and pure translation correctly NOT
confirmed**. The earlier raw-position centroid badly underestimated ω (the front-arc
bias) — the line-of-sight center estimate is what fixed it.
- On confirm it blends ω into `x_(7)` (like the timing path) and sets
  `phase_timing_confident_`. On a full window with no spin it bleeds `vyaw` fast
  (×0.5/frame) and releases confidence → fast spin-down recovery (§3.2).
- **Ghost fix:** on the RISING EDGE of confirmed fast spin (`|ω|≥reseed_min_omega`)
  it re-seeds the EKF center + yaw phase ONCE from the observer geometry (centroid +
  observed-armor angle), then inflates P so the EKF re-converges. Before this the
  EKF center orbits with the armor (the §4c "ghost") and the four-face model aims at
  a phantom robot.

**(B) Regime-adaptive yaw trust.** While `spinning_regime_` (observer verdict of the
previous frame): `q_yaw → q_yaw_spin=80` in `ekfPredict`, and the U-shape floor →
`yaw_facing_obs_floor_spin=1.0` in `measurementNoise` (trust the rotating frontal
yaw). Static keeps the old anti-phantom values. The face index also de-pins
automatically once `confirmedSpin()` (the existing penalty is gated on
`!confirmedSpin()`).

**(C) Asymmetric geometric fire gate (replaces the three regimes).** In
`computeAim`, one physical rule: the selected plate must be within an angular window
of facing-you, **generous while approaching / static** (`fire_window_approach=0.60`),
**strict while leaving** (`fire_window_leave=0.25`), distance-scaled (only shrinks
far away). This is the COD-2026 55°/20° idea (`ricerca2.md`) and roadmap #1 of
`ricerca1.md`. It works uniformly for static / translating / spinning, killing the
"unknown phase" black-hole. Bullet economy stays safe: fire still needs
`command_locked` (Stage B) + range + finite command. Set `asymmetric_fire_enable=
false` to restore the three-regime logic. `aim.facing_ok`/`aim.approaching` now carry
the decision; the node mirrors them (removed the drifted duplicate logic) and reports
`facing_window_approach`/`facing_window_leave`.

### 4d.5 Files changed
- `tracker.hpp`: `SpinSample`/observer state, `was_confirmed_spin_`,
  `spinning_regime_`; config (`spin_observer_*`, `spin_obs_reseed_*`,
  `q_yaw_spin`, `yaw_facing_obs_floor_spin`, `asymmetric_fire_enable`,
  `fire_window_approach/leave`); `AimResult::{facing_ok, approaching}`;
  `updateSpinObserver` decl.
- `tracker.cpp`: `updateSpinObserver()`; regime-adaptive `ekfPredict`/
  `measurementNoise`; observer blend + spin-down + rising-edge re-seed in
  `update()`; asymmetric gate in `computeAim`; observer reset in `resetSpinTiming`.
- `autoaim_node.cpp`: declare new params; `fireBlockReason` asymmetric reasons;
  debug `facing_ok` mirrors `aim.facing_ok`.
- `standard.launch.py`: all §4d params.
- `viewer_node.py`: regime HUD shows observer ω (rpm) + sample count.

### 4d.6 On-robot validation protocol (REQUIRED — not yet run on hardware)
1. **Spinner (the bag case):** `confirmed_spin` should latch true within ~0.3–0.5 s;
   viewer `obsω` ≈ true rpm with correct sign; `matched_face_idx` cycles 0→1→2→3;
   `cmd_fire` rises sharply from 10%. If `obsω` sign is inverted vs reality, the
   observer math is sign-correct by construction — suspect the bearing/ego frame.
2. **Ghost check:** while confirming, the aim must snap onto the robot (re-seed), not
   sit beside it. If it still ghosts at the true spin rate, raise
   `spin_obs_reseed_min_omega` is wrong direction — lower it / check centroid.
3. **Static (log4 regression):** must still fire, `confirmed_spin=false`, `vyaw≈0`,
   no flicker (asymmetric approach window = old static cap).
4. **Slightly-moving:** should now fire when geometrically hitting (was the #2 bug).
5. **Spin-down:** stop the spinner → `confirmed_spin` drops, `vyaw→0` within a few
   frames; aim settles.
6. **Bullet economy:** watch for over-firing on plates that are leaving — if so,
   lower `fire_window_leave` toward 0.18.
7. **Medium spin (2–5 rad/s):** the observer is marginal there (offline valid_frac
   ~0.5); it falls back to the asymmetric gate on the observed armor. If medium
   spinners under-fire, relax `spin_obs_consistency` toward 0.8 (watch translation
   false-positives) or lower `spin_obs_reseed_min_omega`.
8. Fix `ref_freq` to the real detector rate (was 60, bag shows 43).

### 4d.7 Known risks / next
- The re-seed snaps the EKF state — kept behind `spin_obs_reseed_enable` and gated on
  fast spin (centroid only ≈ center when the window spans a good arc). Validate it
  does not introduce a transient jump on borderline spins.
- Medium-spin confirmation is the weak spot (see step 7). A cleaner long-term fix is
  a proper circle/center estimate (or the robot-centric reprojection tracker, roadmap
  #7) so spin is observed continuously instead of via a windowed median.
- `observeRawYawHandoff` is now redundant as the primary bootstrap; left in as a
  secondary timing source. Consider removing once the observer is proven on hardware.

### 4d.8 Round 2 — second bag (`bagsa/debug_state…182411`, REWORK round 1 ON robot)
The §4d round-1 build was run on the robot. **It broke the catch-22**: `confirmed_spin`
363/486 (75%), `matched_face_idx` cycles 0→1→2→3, `faces_checked=4`, `aim_fire` up to
52%, vyaw tracks ~60–80 rpm. But user reports persist: (a) slight enemy translation
still doesn't fire; (b) on rotation the aim "exits the robot" and the rpm oscillates
0↔70↔0. Bag confirms: `cmd_fire` still **9%**; `yaw_locked` only 35%; block reasons
split between `facing_window_*` (203) and `yaw/pitch_unlocked` (211).

**Two distinct round-2 bugs found from the time series:**

1. **Selection ↔ gate DISCONNECT (the dominant fire-blocker).** Face *selection*
   still used the old strict `fire_valid = margin>=0` (0.35 window) while the *fire
   gate* used the new asymmetric 0.60 window. So a currently-facing plate (|phase|
   ~0.4–0.5) was NOT "fire-valid", lookahead switched the aim to the **incoming
   face ~90° away** (`face_switch_reason=incoming_lookahead`, phase ≈ −1.3), the
   gimbal pointed at the side of the robot, never locked, never fired — exactly
   "l'aim esce dal robot". **Fix:** `c.fire_valid` now uses the same per-face
   asymmetric window as the gate; `facing_ok = best.fire_valid` (one source of
   truth). The aim stays on the plate the gate would actually fire at; lookahead
   only engages when no face is within the window.

2. **Spin estimate flicker (the 0↔70 rpm).** Three causes: (i) the re-seed fired on
   EVERY `confirmed_spin` rising edge, and confirmed_spin toggled frame-to-frame, so
   the center was snapped repeatedly → `pd` spikes (up to 1.0 m) and a wandering
   aim; (ii) a single observer-invalid frame (detector miss / handoff-heavy window)
   halved vyaw and dropped confidence; (iii) bad-center frames spiked ω up to
   179 rpm and the 0.5 blend injected that into vyaw. **Fixes:** re-seed **once per
   spin episode** (`spin_reseed_done_`); **hold** confirmed spin through
   `spin_lost_grace=6` invalid frames before releasing; **reject** observer ω above
   `spin_obs_max_omega=40 rad/s` or differing from current vyaw by >`spin_obs_
   outlier_ratio=2.5`×; lower blend 0.5→**0.35** (smoother rpm).

**Round-2 build OK. On-robot re-validation pending** (same §4d.6 protocol; expect
`yaw_locked` and `cmd_fire` to rise sharply, rpm to be steady, aim to stay on the
plate). If translation-following is still soft AFTER this, it is the §4b lead tuning
(`q_pos`/`alpha_pos`), a separate axis.

**Round-2 additions (also applied, build OK):**
- **`DebugState` now carries the raw inputs + state** for OFFLINE replay (so the
  next bag can be re-run through the spin observer / EKF without the robot):
  `det_x_raw/det_y_raw/det_z_raw/det_yaw_raw` (selected detection, odom, pre-EKF),
  `ego_x/ego_y`, `state_xc/vxc/yc/vyc/vza`, `spin_centroid_x/y`, `spin_omega`,
  `spin_invalid_streak`. (`state_yaw`, `vyaw`, `radius` were already published.)
- **Translation follow:** `alpha_pos 0.995 → 1.0` (pure CV). 0.995 at ref_freq 60
  bled ~26%/s of the velocity estimate — a constant drag that made the EKF
  under-lead / lag a slightly-moving enemy ("non segue bene"). The anisotropic R
  still collapses velocity to ~0 on stop, so this removes the lag without re-adding
  stop-overshoot. If a static target shows phantom lead, lower toward 0.99.

### 4d.9 Round 3 — center-aim for spinners + translation overshoot (bag `…190343`)
Third bag (mostly a ~60 rpm spinner, 926 msgs, NOW with the raw-detection fields).
User: translation now **overshoots** left-right on slow targets; spin aim still
"random", no improvement. Bag analysis (raw fields let us reconstruct geometry):
- **Spin is tracked, center is NOT ghosting:** `|center − raw detection|` median
  **0.234 m ≈ radius 0.232** (the earlier "1.16 m" was an artifact of det=(0,0)
  no-detection frames). `confirmed_spin` 93%, faces cycle.
- **The spin failure is the AIM, not the state:** `incoming_lookahead` is the face
  reason **59%** of frames; the aim chases each plate then JUMPS to the next
  incoming one ~90° away → `cmd_yaw` frame-to-frame jump mean **58 mrad, p90 167,
  max 395**, 30% of frames >50 mrad. At 60 rpm a handoff is every ~0.25 s, so
  chasing plates means slewing the gimbal 90° every 0.25 s — it can never settle →
  `yaw_locked` 37%, `cmd_fire` 11%. This is the "aim va a caso".
- **Translation overshoot:** `alpha_pos=1.0` (round-2) over-led; also the orbiting
  armor leaks into the CENTER velocity (`state |v|` mean 0.21, max 0.93 m/s phantom)
  via `q_pos=10`, inflating the lead.

**Round-3 fixes (applied, build OK):**
1. **Center-aim for a confirmed spinner** (`spin_aim_center_enable=true`,
   `computeAim`). When `confirmed_spin`, the aim azimuth/pitch point at the spin
   center's **facing point** (center pushed one radius toward the barrel = where a
   facing plate sits), NOT the chased rotating face. The center bearing is
   near-stationary, so the gimbal can lock; firing still requires a plate within the
   facing window at impact (`best.fire_valid`). **Offline-validated on the bag:**
   frame-to-frame aim jump drops **58 → 11.6 mrad mean** (p90 167 → 23, >50 mrad
   30% → 1%). This is the key spin fix — a steady aim that can actually lock and
   fire when a plate sweeps through.
2. **`q_pos 10 → 5`**: calms the phantom center velocity (less center-aim wobble,
   less translation over-lead).
3. **`alpha_pos 1.0 → 0.99`**: a touch of damping curbs the round-2 translation
   overshoot; the round-2 gate fix handles "doesn't fire" separately now.

On-robot re-validation pending. Expect: steady aim on a spinner that locks and
fires when a plate faces; no left-right overshoot on slow translation.

> The raw-detection fields added in round 2 are what made round 3 **data-driven**:
> the center-aim benefit (5× steadier aim) was measured from the recorded bag
> BEFORE touching the robot. Keep recording with these fields.

---

## 4e. FULL FIRE-CONTROL REWORK — shoots-wheels + overshoot (2026-06-23, APPLIED, build OK)

### 4e.1 Symptoms (reported, this round) + the bag
Validating round 3 on the standard robot (bag `bagsa/debug_stateY20260623_193335`,
1541 msg / 35 s: spinner ~55–65 rpm 0–20 s, then translation 20–30 s, then stop):
1. **Slight left-right translation** → aim **overshoots**, goes further L/R than it
   should, sometimes **jumps back/forward**, doesn't follow accurately (we are still).
2. **Hard spinner** → aim lands at near-**random** values around the robot, never
   pitch/yaw inside the plate; **fire given when the aim is on the wheels** at close
   range, NOT given when the aim is over the plate; `vyaw` "oscillates 80–110 rpm"
   on a robot that wasn't rotating that hard.

### 4e.2 Evidence — round 3 fixed the STATE, the FIRE-CONTROL is what's broken
Round 3 (center-aim) **worked for what it does**: `confirmed_spin` 81%, faces cycle
0→1→2→3, center NOT ghosting (`|center−det|` median 0.219 ≈ radius 0.218), and when
`confirmed_spin` holds two frames the aim is steady (cmd_yaw jump 12 mrad). But:

| Metric | Value | Reading |
|---|---|---|
| `cmd_fire` off-plate | **49%** of fires (`pixels_inside` 51% when firing) | shoots wheels |
| `\|phase_error\|` at fire | mean 0.217, **max 0.597** | fires up to 34° off-facing |
| phantom center `\|v\|` (spin) | **0.295 m/s mean, 1.23 max** (robot still) | overshoot source |
| aim lead from phantom v | 32 mrad mean, **241 max** | the L/R overshoot + jumps |
| `vyaw_rpm` (confirmed_spin) | 55 ± **22.8** (f2f 3.2) | noisy spin |
| `yaw_locked` | **59%**; 238 frames `aim_fire`&!fire = `yaw_unlocked` | lock fails |
| cmd_yaw big jumps (>80 mrad) | 61, **62% coincide with `confirmed_spin` toggle** | aim flips mode |

### 4e.3 Root cause — ONE root, four faces (the FIRE-CONTROL/output layer)
Rounds 1–3 fixed STATE ESTIMATION. What was left broken is the layer above:
**we aim at one point and validate fire at a different point on a rotating
reference, while the orbit contaminates the center velocity.**

1. **Fire gate DECOUPLED from the aim point (the wheel-shot root).** Center-aim
   points the gimbal at the spin-center facing point `P_c`, but fire was permitted
   when the *rotating selected face* was within `fire_window_approach=0.60` (34°).
   Those are different points: at r=0.22 a 34° plate is `r·sin34°≈0.12 m` (a full
   plate width) from `P_c` → the bullet flies to `P_c` where no plate is. COD's
   `should_fire_spin_target` (ricerca2.md:290) uses the window only as `phase_ok`
   (selection) PLUS `aim_ok` (predicted aim error tiny) — **we implemented only the
   window half.** Also: `pixels_inside` only checks "inside the camera frame", never
   "inside the plate"; no impact-in-plate test existed.
2. **Phantom center velocity → overshoot.** A single front-plate can't separate
   "center translating" from "armor orbiting"; with the normal `q_pos` the EKF
   center chases the orbiting armor → phantom velocity (and the center POSITION
   wobbles too: even the windowed slope of `state_xc` gives 0.8 m/s at a dead stop —
   so neither the observer centroid nor a slope is usable as a clean velocity).
3. **`confirmed_spin` toggle → aim flips center-aim↔face-chase → no lock.** The
   redundant raw-yaw/timing path (`jump_detected` 796, `vyaw_timing_accepted` 726)
   was still resetting `phase_timing_confident_=false` on IPPE-noisy yaw reversals,
   toggling the regime; each toggle = a >80 mrad gimbal jump = lost lock.
4. **Noisy `vyaw`.** `q_yaw_spin=80` fully trusted the IPPE-flipping frontal yaw and
   the timing path double-wrote `vyaw`, jittering it ±23 rpm. (The position observer
   ω is actually *noisier* — std 30 — so vyaw must stay the smoothed EKF value, just
   with fewer noise injections.)

### 4e.4 The rework (APPLIED 2026-06-23, build OK) — four pieces
**(1) IMPACT-INSIDE-PLATE fire gate** (`tracker.cpp::computeAim`, new config
`fire_require_impact_inside`/`plate_half_width=0.065`/`plate_half_height=0.050`).
The physical trigger: the predicted bullet impact (= the commanded aim
`abs_yaw/abs_pitch`) must land inside the selected plate's board at impact time:
`lateral = best.range·sin(aim.abs_yaw − best.bearing)`, `vertical = best.range·
sin(aim.abs_pitch − best.abs_pitch)`, require `|lateral|≤half_w && |vertical|≤
half_h`. For NON-center aim the command IS the plate center (lateral≈0 ⇒ inside), so
the asymmetric window stays the only active gate there (obliquity guard). For the
center-aim spinner it fires ONLY when a plate sweeps through `P_c` — the wheel-shot
fix. Folds into `aim.fire`; new fields `AimResult::{impact_inside,impact_lateral,
impact_vertical}` + `/debug_state` mirrors + `fire_block_reason="impact_outside_plate"`.
This is roadmap #1 / COD's two-stage gate. **Offline indication on the bag:** the
gate keeps ~73% of previously fire-eligible spinner frames (|phase|≲asin(0.065/r)≈
0.30) and drops the 0.30–0.60 tail that produced the off-plate shots.

**(2) Regime-adaptive CENTER stiffness** (`ekfPredict`, new `q_pos_spin=0.8`,
`alpha_pos_spin=0.90`). While `spinning_regime_`, the horizontal center uses a small
q + strong damping so it stops chasing the orbit → phantom velocity bleeds to ~0 and
the center converges to the orbit mean. Mirrors the existing `q_yaw_spin` pattern;
vertical keeps normal `q_pos`. A real translating spinner still follows (slowly) —
and under-leading costs far fewer balls than the overshoot did.

**(3) Subordinate the raw-yaw timing path to the position observer**
(`feedSpinTimingJump`). While `spinning_regime_` the timing path may RAISE confidence
but must NOT (a) reset `phase_timing_confident_=false` or (b) write `vyaw` — the
observer's `spin_lost_grace` is the single owner of spin release. Kills the regime
toggle (and one vyaw-noise source).

**(4) Calmer spin yaw** (`q_yaw_spin 80→40`). The observer owns the rate; the EKF
only needs enough q to keep the phase aligned. Less IPPE-yaw noise into vyaw.

Also: `ref_freq 60→45` (real detector rate from the bags is ~44 Hz; 60 over-scaled
the per-frame damping normalization). Re-confirm with `ros2 topic hz`.

### 4e.5 Files changed
- `tracker.hpp`: config `fire_require_impact_inside`/`plate_half_width`/
  `plate_half_height`, `q_pos_spin`/`alpha_pos_spin`, `q_yaw_spin` default 80→40;
  `AimResult::{impact_inside,impact_lateral,impact_vertical}`.
- `tracker.cpp`: regime-adaptive `q_pos`/`alpha_pos` in `ekfPredict`; impact gate in
  `computeAim` folded into `aim.fire`; raw-yaw path subordinated in `feedSpinTimingJump`.
- `autoaim_node.cpp`: declare new params; publish impact fields; `fireBlockReason`
  `impact_outside_plate`.
- `DebugState.msg`: `impact_inside`/`impact_lateral`/`impact_vertical`.
- `standard.launch.py`: all §4e params + `q_yaw_spin 40` + `ref_freq 45`.
- `viewer_node.py`: HUD impact line (inside + lat/vert mm).

### 4e.6 On-robot validation protocol (REQUIRED — not yet run on hardware)
1. **Spinner (wheel-shot fix):** `cmd_fire` should now fire ONLY when the HUD
   `Impact:Y` (lat/vert within plate); watch that fires no longer land on the wheels.
   `fire_block_reason` should show `impact_outside_plate` while waiting for a plate
   to reach the facing point (this is correct, not a bug).
2. **Slow translation (overshoot fix):** aim should follow without the L/R overshoot
   / back-forward jumps; `state_vxc/vyc` (phantom) should be much smaller during a
   pure spin. If a real translating enemy is UNDER-led (lags), raise `q_pos_spin`
   toward 2.0 or `alpha_pos_spin` toward 0.95.
3. **Regime stability:** `confirmed_spin` should stop toggling; `yaw_locked` and
   `cmd_fire` should rise; cmd_yaw jumps should drop.
4. **vyaw:** steadier; if still noisy, lower `spin_obs_vyaw_blend` toward 0.2.
5. **Static regression:** must still fire (impact gate trivially satisfied; aim = plate
   center).
6. **Bullet economy:** if it under-fires on a fast spinner (plate sweeps too fast to
   catch inside the plate window), raise `plate_half_width` toward 0.08, or widen the
   impact horizon by aiming slightly ahead of `P_c` (future work). If it over-fires
   on leaving plates, that's the asymmetric `fire_window_leave` — lower toward 0.18.

### 4e.7 Known risks / next (toward "team forte")
- The impact gate is evaluated at a SINGLE impact time; a proper **trajectory fire
  score** over a 50–150 ms horizon (roadmap #1 full) + the **latency ledger**
  (roadmap #2) is the next step — it would also let us fire slightly BEFORE the plate
  reaches `P_c` to account for the sweep speed.
- The center/vyaw separation is still a 9-D EKF fighting an observability limit; the
  clean long-term design is the two-mode split (directly-observed-armor tracker before
  spin, robot-center/4-face after) or the robot-centric reprojection tracker
  (roadmap #7).
- **PnP yaw via normal vector** (§4.6.2) is still queued — the weak input feeding vyaw.

---

## 4f. THE NYQUIST CEILING + observability-gated rework (2026-06-24, APPLIED, build OK)

### 4f.1 Symptoms (reported after §4e on robot)
Very slow to follow a translating enemy (missed ~10 balls shooting air at the
start), then overshoot L/R; misses almost everything on a slow L/R mover; on a
spinning chassis misses ~80–90% (hits 10–20%, "many by luck"): pitch not keeping
up on alternating low/high armors, shoots L/R of the plates (wheels), fires at
plates already leaving, wrong lead timing ("thinks a wide turn" / "already did a
full turn before the bullet arrives"). User asked: refactor, wrong params, or what?

### 4f.2 The decisive finding — it's the DETECTOR RATE vs the spin (physics)
Bag `debug_state_20260624_193721` (2923 msg). §4e WORKED for the wheel-shot: fires
off-plate **174→5**, phase at fire **0.245→0.121**, `pixels_inside` 99%. But:
- **Lock is now the bottleneck**: 597 frames `aim_fire`&!`cmd_fire`, all yaw/pitch
  unlocked. NOT-locked frames have `cmd_yaw` f2f move **34 mrad** vs the servo's
  **7 mrad/frame** — we command jumps the gimbal physically can't follow.
- **Raw detection is EXCELLENT** when sampled: still enemy → **4 mm** f2f jitter,
  0.4 px reproj. Spinning enemy → **119 mm** f2f (p90 319, max 539) but STILL
  0.6 px reproj, `yaw_replaced_by_bearing` 0%. So the 119 mm jumps are REAL motion.
- Geometry: `Δθ = 0.119/0.21 = 0.57 rad = 32°/frame`, p90 87°/frame ⇒ true spin
  **≈235 rpm**. At 44 Hz a 4-faced armor passes a face every ~2.8 frames →
  **at/over Nyquist → aliased**. Three independent ω estimates disagreed wildly
  (handoff cadence 400 / orbit-net 2–10 / EKF 50 rpm) — the signature of aliasing.
- **Proof a tracker refactor can't fix it:** I implemented the chosen circle-fit
  observer OFFLINE (`/tmp/circlefit.py`) — it ALSO failed (ω std ±108 rpm, fit
  resid 35–43 mm, center vel 0.46 m/s even when still), because the armor doesn't
  trace a clean circle when sampled at 44 Hz with 30–90°/frame steps.

**Verdict: the dominant limiter is the 44 Hz detection rate vs a fast top — Nyquist,
not code.** No EKF/observer/circle-fit beats it. Strong teams run 100–300 fps
(ricerca2: ShanghaiTech 300fps/12ms) precisely for this. The detector here is a
clean CUDA-preprocess + TensorRT pipeline but runs a **fully serial blocking loop**
(grab → `stream.synchronize` infer → CPU postprocess+3 publishes incl. a per-frame
JSON → no GPU/CPU overlap), so it caps at ~44 Hz. The user confirmed ~40–50 Hz.

### 4f.3 The recalibrated plan (user: "do what's best to win") — A + B + C + D
The full circle-fit EKF refactor was DROPPED (proven not to help at 44 Hz). Instead:
- **A — raise the detector rate (THE lever, framerate-dependent).** Added a per-stage
  **latency ledger** to `zed_detector.py::loop` (`time.perf_counter` around grab /
  infer / postprocess / publish, EMA, logged every ~90 frames as `LATENCY[ms]
  grab=.. infer=.. post=.. pub=.. | loop=.. (..Hz)`). This tells the user WHICH
  stage eats the ~23 ms. Likely suspects: the serial blocking loop (no overlap) +
  the per-frame JSON publish. Pipelining (double-buffer GPU infer ∥ CPU post) +
  dropping/throttling the JSON are the next steps. *User to run + report.*
- **B — make the command lockable (framerate-independent).** Center-aim
  STICKINESS (`center_aim_hold_frames=8`): keep center-aim through brief
  `confirmed_spin` dropouts so the azimuth doesn't flip center-aim↔face-chase (a
  ~90° jump; 62% of >80 mrad jumps were at a regime toggle). Did NOT add a tight
  command rate-limit on purpose — limiting to the servo rate would make
  cmd≈servo and report "locked" while OFF-target (fire off-target). Fixing the
  jump SOURCE is correct.
- **C — kill the translation overshoot cleanly.** Reverted the erratic
  regime-adaptive q_pos band-aid effect (q_pos_spin 0.8→**2.0**, the sticky regime
  removes the 0.8↔10 flicker) and added `spin_trans_lead_cap=0.30 m/s`: while
  `confirmed_spin`, cap the center speed led into the aim (in `computeAim`, applied
  to `x_aim(1/3)` AND the center-aim point) so the phantom 0.3–1.2 m/s can't
  overshoot, while a real slow translation still follows.
- **D — don't lead on a garbage omega.** Rotational lead now gated on
  OBSERVABILITY: `omega_reliable = confirmed_spin && |vyaw| ≤ omega_reliable_max
  (15 rad/s ≈143 rpm)`. Above it (the aliased fast top) the rotational lead is
  ZEROED → pure center-aim + opportunistic impact-gate fire (the physical best at
  44 Hz). Below it (slow spin, well-sampled) the phase lead still applies.

### 4f.4 What this does and does NOT fix (honest)
- FIXES (framerate-independent): the lock-breaking command flips (B), the L/R
  translation overshoot (C), the wrong-phase lead on a fast top (D, now we don't
  pretend to predict an aliased spin).
- Does NOT fix: actually HITTING a ~235 rpm top at 44 Hz — that needs **A** (higher
  rate). At 44 Hz the honest best is center-aim + fire when a plate is at the facing
  point; hit rate on a fast top is physically capped until the rate goes up.
- `dz` (alternating high/low armor): NOT this bag's problem — measured Δz between
  even/odd faces = **1 cm**, pitch_error not parity-dependent. The "aims low on the
  high armor" was the pitch not keeping up during fast motion (the lock/Radice-1
  issue), not a dz-model gap. Left dz disabled.

### 4f.5 Files changed
- `tracker.hpp`: config `omega_reliable_max`, `spin_trans_lead_cap`,
  `center_aim_hold_frames`; `q_pos_spin` 0.8→2.0, `alpha_pos_spin` 0.90→0.93;
  mutable `center_aim_hold_`.
- `tracker.cpp` `computeAim`: observability-gated rotational lead; translational
  lead cap; center-aim stickiness + capped center velocity in the center-aim point;
  `resetSpinTiming` clears `center_aim_hold_`.
- `autoaim_node.cpp`: declare the new params (+ updated q_pos_spin/alpha_pos_spin).
- `standard.launch.py`: §4f params.
- `zed_detector.py`: per-stage latency ledger in `loop()`.

### 4f.6 On-robot validation (REQUIRED)
1. **Run the detector, read `LATENCY[ms] ... (..Hz)`** — report the breakdown; that
   sets the framerate plan (A). This is the single most important number now.
2. Translation: overshoot should be gone; if it now UNDER-leads a real mover, raise
   `spin_trans_lead_cap` toward 0.6 or `q_pos_spin` toward 5.
3. Spin: aim should stay steady (lock should hold far more); `cmd_fire` up. Fast top
   hit rate is capped until A.
4. If a SLOW spinner under-fires (we dropped its lead), raise `omega_reliable_max`.

### 4f.7 Next (toward winning)
- **A is the headline.** Pipeline the detector loop (double-buffer), drop/throttle
  the per-frame JSON, confirm FP16 engine. Target ≥90 Hz → then a real phase tracker
  (even the circle-fit) becomes viable and the fast-top hit rate jumps.
- Then: latency ledger end-to-end (node side too) + trajectory fire over the horizon.

### 4f.8 Detector profiling — RealSense is the ACTIVE detector (2026-06-24)
**Critical:** the STANDARD robot (the 44 Hz one) runs **`realsense_detector.cpp`**
(C++, package `autoaim_realsense`, `DEFAULT_CAMERA="realsense"`), NOT `zed_detector.py`.
The SENTRY uses the ZED (`zed_detector.py`). Both run the SAME engine and have the
same serial-loop + per-frame-JSON issues. `trtexec` on the shared engine:
- **input 1×3×960×960** (NOT 640), **GPU compute 15.1 ms** (66 fps ceiling for
  inference alone). Serial loop adds ~8 ms CPU → ~44 Hz. H2D 0.88 ms, D2H 0.009 ms.
- GPU **clock 0.918 GHz** (Orin maxes ~1.3) → likely NOT in MAXN/`jetson_clocks`.

**The two biggest levers are camera-independent and code-risk-free (USER actions):**
1. **Re-export the model at 640×640** (0.44× pixels → infer ~7 ms → ~110 fps ceiling).
2. **`sudo nvpmodel -m 0 && sudo jetson_clocks`** (~−25 % infer).

**Applied (stage 1 + ledger), build OK:**
- `realsense_detector.cpp` (ACTIVE): `publish_keypoint_json` param (default **false** —
  the JSON topic has no subscriber, pure CPU waste) + per-stage **latency ledger**
  (`LATENCY[ms] gpu/decode/pub | loop Hz`, every 90 frames). `realsense.yaml` params.
- `zed_detector.py` (SENTRY): same JSON param + ledger + **software pipeline**
  (`pipeline_inference`, default true) — process frame N-1 on CPU while GPU computes
  N; +1 frame latency, compensated by the latency-aware prediction.

**Stage 3 (pipeline) on the C++ RealSense detector: DEFERRED until the ledger is read.**
Rationale: after removing the JSON, if `decode`+`pub` is small the pipeline buys little,
and an untested CUDA double-buffer refactor on the ACTIVE competition detector is high
risk. Measure first (`LATENCY[ms]`), then decide. (Stage 2 / MEM.GPU was rejected:
saves ~0.9 ms H2D vs a 15 ms infer, and risks a cross-CUDA-context crash.)

### 4f.9 Pitch-slow + face-priority — SAME root, not dz (2026-06-24)
User (bag 193721): gimbal too slow up/down (aims low, the high armor already passed);
doesn't prioritise incoming armors / doesn't switch to a shootable face. Diagnosis:
- **Pitch is the same command-jumpiness as yaw:** `cmd_pitch` f2f **17.7 mrad** (p90 45)
  vs servo **11 mrad/frame** → `pitch_locked` only 66 %. The gimbal can't EXECUTE the
  jumpy command, it's not a slow servo per se.
- **Selection logic is fine:** `selected_face == best_face` **100 %**; `incoming_lookahead`
  IS used (30–50 %). So it's NOT "ignores incoming"; the command it produces just can't
  be locked, and at 44 Hz the incoming-face TIMING is aliased anyway.
- **NOT a dz problem:** even/odd face Δz = **1 cm** in this bag; pitch_error not
  parity-dependent. The "aims low on the high armor" is the pitch command lagging during
  fast motion, i.e. the lock/Radice-1 issue, not a height-model gap.
⇒ Addressed by §4f B/C/D (sticky center-aim → steady pitch too; untested on robot) +
the framerate lift. No separate fix needed yet; re-evaluate with a fresh bag after
those land. If face PRIORITY is still wrong then, revisit `computeAim` candidate
ordering (FireValid vs Incoming) directly.

---

## 4g. WALKED-SPINNER FOLLOW LAG + bag …201048 problem audit (2026-06-25, APPLIED, build OK)

### 4g.1 Request
"Follow the enemy MUCH more closely (not slowly) when it moves (we are still, the
enemy moves) but do NOT overshoot." Plus: list the problems in the last bag
(`bagsa/debug_state_20260624_201048`, 15045 msgs / 336 s) that need addressing.

### 4g.2 What the bag actually is (offline replay of the raw fields)
This is **almost entirely a fast SPINNING TOP**, sometimes walked across and brought
from 2.9 m → 0.34 m, with a static tail. It is NOT a clean pure-translation test.
Per 10 s window the armor PATH is 50–90 m while net center displacement is ~0.1–1 m
(`detnet/detpath ≈ 0.01`) and the plate normal rotates 100–180 rad — a spinner.
Frame classification by local plate-rotation is clean: rot<1 rad/0.6 s (static/
translation) → `confirmed_spin` 5 %; rot>3 → 77 %. So the regime observer is NOT
broadly false-triggering; the §4d/§4e/§4f machinery is doing its job.

### 4g.3 The "segue lentamente" — measured: a POSITION lag, not a lead deficit
Compared the EKF center (`state_xc/yc`) to a heavily-smoothed line-of-sight center
(`det + r·[cosθ,sinθ]_bearing`, the orbit-averaged truth) while the spinner is
WALKED (100–140 s):

| segment | EKF center lag vs true center | pos err |
|---|---|---|
| 100–140 s walked spinner | **3 frames ≈ 66 ms** | **0.16 m** |
| 140–160 s approach (close) | 1 frame ≈ 22 ms | 0.08 m |
| 220–260 s fast spinner, still | (phase-only, no real motion) | 0.06 m |

So when the spinner translates, the aim trails the true center by ~66 ms / 16 cm.
Crucially this is a **position-tracking lag**, not a lead problem:
- The cleanest velocity estimate available is the EKF `vxc/vyc` itself: on a NON-
  translating spinner it reads **0.04 m/s** (the q_pos_spin stiffening works). The
  observer **centroid** velocity is garbage (0.12 m/s still / 1.45 m/s walked) — the
  §4e note "neither the centroid nor a windowed slope is a clean velocity" is
  CONFIRMED, so a centroid-velocity lead is off the table.
- The spin lead is already bled ~97 %/s (`alpha_pos_spin=0.93` at ref_freq 48) and
  hard-capped at `spin_trans_lead_cap=0.30` (binds 32 % of moving-spin frames, mostly
  clipping the orbit PHANTOM, not real translation). So overshoot is already guarded.

⇒ The lag is set by **`q_pos_spin`** (center stiffness). It was lowered to reject the
orbit "ghost"; the cost is trailing real translation.

### 4g.4 The fix — APPLIED 2026-06-25 (build OK), single knob
`q_pos_spin` **2.0 → 4.0** (`tracker.hpp` default + `standard.launch.py`). Doubles the
center responsiveness so the walked-spinner lag drops toward ~9 cm, while:
- it stays ≪ the non-spin `q_pos=10`, so the center still rejects the orbit (it does
  not start chasing the orbiting armor → no ghost);
- overshoot is unchanged: the velocity led into the aim is still bled ~97 %/s and
  capped at 0.30 m/s;
- lock is unaffected: the bag's center-aim command is already steady (cmd_yaw f2f
  jitter 5–6 mrad med, p90 17–19); a still-spinner center wobble of ~0.07→0.10 m/s is
  ~2 mrad ≪ the lock tolerance (27 mrad @2.5 m, ~108 mrad @0.6 m).
Tunable: push toward 6.0 if a walked spinner still lags AND the gimbal keeps lock;
drop back to 2.0 to revert. Left `alpha_pos_spin`/`spin_trans_lead_cap` untouched so
the on-robot validation isolates one variable.
NOTE — regimes: this knob only affects a CONFIRMED spinner. A NON-spinning enemy that
merely translates uses `q_pos=10` / `alpha_pos=0.99` (already the user-validated
no-overshoot setting; `alpha_pos=1.0` was reverted in round 3 for overshoot — do NOT
re-raise it to chase follow speed). If a pure-translation test still feels slow, the
right lever is a regime-split tighter tangential R or an asymmetric (responsive-up /
damp-on-stop) velocity damper — deliberately NOT done here to avoid re-adding the
round-3 overshoot blind.

### 4g.5 Problems found in bag …201048 (audit — report, mostly NOT code bugs)
1. **The last ~60 s (280–340 s, 2483 frames = 18 % of the bag) is a PARKED GIMBAL,
   not autoaim.** Autoaim commanded correctly (`cmd_yaw` 0.571 ≈ target bearing 0.505)
   but `micro_yaw` was frozen at 0.255 (std 0.018) — the physical gimbal sat ~18° off
   the command the whole time → `yaw_locked` 0 %, `cmd_fire` 0 %, block reason
   `yaw_pitch_unlocked`. The lock gate correctly refused to fire. **Action: exclude
   this tail from any tuning stats; confirm the gimbal was actually driven
   (shooting/enable) during the run — if it genuinely could not reach the command,
   that is a hardware/gimbal-control issue, separate from autoaim.** This single
   segment drags the whole-bag `yaw_locked` from ~70 % down to 60 % and `cmd_fire` down.
2. **LOCK is the fire bottleneck (framerate-bound).** `aim_fire` 46 % but `cmd_fire`
   only 17 %; both-locked 43 %. At 2.5 m `tol_yaw = 0.0675/2.5 ≈ 27 mrad` but the
   residual command/track jitter on the aliased fast top is ~30 mrad → lock fails
   ~half the time. This is the §4f Nyquist wall: at 44 Hz the command cannot be made
   steady enough for the tight FAR-range lock window. Lever = **detector rate**
   (§4f.A — read the `LATENCY[ms]` ledger); secondary = re-balance `fire_lock_k_yaw`
   vs the real far-range jitter.
3. **The top is ALIASED at 44 Hz.** Raw plate normal moves ~14 rad/s; `vyaw_rpm` sits
   26–55 (smoothed/capped) while raw handoffs imply a much faster top. The rotational
   lead is (correctly) zeroed above `omega_reliable_max`, so on the fast top the
   system is center-aim + opportunistic impact-gate fire only — hit rate is physically
   capped until the framerate rises. Not a regression; the §4f conclusion stands.
4. **Center-follow lag on a WALKED spinner ~66 ms / 16 cm** (§4g.3) — the "segue
   lentamente"; addressed by the q_pos_spin change above.
5. **Fire-gate blocks (spin, non-firing frames):** `facing_window_approach` 4556,
   `yaw_pitch_unlocked` 3041, `impact_outside_plate` 1819, `facing_window_leave` 664.
   The facing/impact blocks are mostly CORRECT (no wheel-shots — the §4e gate works);
   combined with the lock failures they keep `cmd_fire` ~17 %. "Honest but quiet" on a
   fast top → again a framerate story, not a gate bug.
6. **(Minor) detector hiccups:** ~289 frames dt>50 ms (one 339 ms gap), 9 dropouts.
   Grace/coast absorbs them (`TEMP_LOST` only 5 %), but watch the detector loop
   stability via the §4f.A ledger.

### 4g.6 On-robot validation (REQUIRED — not yet run on hardware)
1. **Walked spinner (the target case):** aim should trail the moving center LESS
   (was ~66 ms / 16 cm). Watch `state_xc/yc` vs the raw detection; the gimbal should
   sit on the center more tightly while it is walked. If still laggy AND lock holds,
   raise `q_pos_spin` toward 6.0.
2. **Overshoot check:** a walked spinner that STOPS must settle without L/R overshoot
   (the cap + alpha_pos_spin bleed are unchanged, so this should hold). If it now
   overshoots, lower `spin_trans_lead_cap` toward 0.25.
3. **Lock regression:** `yaw_locked`/`cmd_fire` must not DROP vs the previous build
   (the center-aim wobble should stay ~2 mrad). If lock drops on a still spinner,
   `q_pos_spin` is too high — back off toward 3.0.
4. **Static regression:** still robot must still fire (this knob is spin-only).

### 4g.7 ADAPTIVE FOLLOW — fast on sudden changes + less overshoot (APPLIED 2026-06-25, build OK)
**User clarification (round 2):** "Don't touch the fire margin — improve the AIM: it
must follow SUDDEN changes much faster AND overshoot LESS, and make it switchable off."
(Also confirmed §4g.5#1: the 280–340 s tail was the gimbal deliberately OFF — not a
bug.) "Switchable" → a new `*_enable` flag; the conservative FIRE gating ("honest but
quiet") is separately already switchable via `fire_require_impact_inside=false` /
`asymmetric_fire_enable=false`.

**Why a new mechanism (not a knob):** "follow sudden changes faster" wants light
velocity damping (hold the lead); "overshoot less" wants heavy damping (kill the lead
on a stop). No single `alpha_pos` does both — that is the whole §4b↔§4d.9 oscillation
(1.0 over-led/overshot, 0.99 lags). The fix makes the horizontal-center velocity
damping **react in one frame to whether the latest measurement still confirms the
lead**, using the POSITION INNOVATION, not a lagged velocity EMA.

**The signal** (`tracker.cpp::update`, consumed next frame in `ekfPredict`):
`follow_signal = 1 + (innov · v) / (|v|^2 · dt)` where `innov` is the armor-position
innovation vs the post-predict prediction. ≈1 while the target advances as the velocity
predicts (moving), →0 the instant it falls behind the lead (stop), <0 on a reversal.
`w = clamp(follow_signal, 0, 1)`; `alpha_xy = alpha_stop + w·(alpha_move − alpha_stop)`
damps only the horizontal center (xc,yc); vertical/yaw unchanged.

**Why the innovation, not the realized-velocity EMA:** I first implemented the EMA of
the per-frame center displacement — the offline CV-Kalman sim showed it is slightly
WORSE (the EMA detects the stop ~2 frames late, after the overshoot peak has formed).
The innovation reacts immediately. Sim (move→stop, faithful 1-D CV-Kalman, q_pos=10,
40–60 seeds): static `alpha_pos=0.99` → follow lag **54 mm**, peak overshoot **75 mm**;
innovation-adaptive (`alpha_move=1.0`, `alpha_stop=0.70`) → lag **43 mm**, overshoot
**60 mm** — BOTH better. Reversal: lag 52→42 mm, overshoot no worse.

**Gated to the NON-SPIN regime** (`!spinning_regime_`): on a 小陀螺 the orbit leak
co-drives the innovation and the velocity, so the signal would read "moving" on a still
top — a confirmed spinner keeps the proven `q_pos_spin`/`alpha_pos_spin`/lead-cap path.
This targets exactly the "we are still, the enemy translates" case the user described.

**Switchable:** `adaptive_follow_enable=false` restores the static `alpha_pos`. Two new
`/debug_state` fields for offline tuning: `adaptive_follow_w` (1 light … 0 hard) and
`realized_center_speed`.

**Files:** `tracker.hpp` (config `adaptive_follow_enable`/`alpha_move=1.0`/`alpha_stop
=0.70`/`realized_ema`; `TrackerDebugInfo`+state `follow_signal_`); `tracker.cpp`
(`ekfPredict` adaptive `b_xy`; `update()` computes `follow_signal_` + realized-speed
telemetry; `initFromDetection` seeds them); `autoaim_node.cpp` (declare params + publish
2 debug fields); `DebugState.msg` (+2 fields); `standard.launch.py` (params). `colcon
build` OK.

**On-robot validation (REQUIRED):**
1. **Sudden translation (the case):** a NON-spinning enemy that darts L/R or starts/stops
   — aim should snap onto it faster and STOP without sailing past. Watch `adaptive_follow_w`
   (→0 at each stop/reversal) and `state_vxc/vyc` collapsing right after a stop.
2. **Overshoot:** if it still sails past on a stop, lower `alpha_stop` toward 0.55. If it
   feels twitchy/under-damped mid-move, raise `alpha_stop` toward 0.85.
3. **A/B:** flip `adaptive_follow_enable` to compare against the old static behaviour.
4. **Spinner unaffected:** this is non-spin only; a confirmed spinner must behave exactly
   as the §4g.4 build (the q_pos_spin change owns that case).

## 5. Backlog — accuracy & fire-rate upgrades (bullet-economy aware)

Context: **ARC has a ~750-ball cap.** Optimize *hit probability per ball*, not
raw rate. **Primary source: `ricerca1.md`** (repo root) — a survey of strong
Chinese RM pipelines 2024–2026 (Tongji SuperPower, WUST/awakening, Z_LION,
CSU FYT, …) with a prioritized retrofit roadmap. Key finding: the edge is
**engineering/integration, not exotic probabilistic tracking** — don't chase
explicit ω̇ or full MHT. Roadmap, ordered by impact/effort (their estimates):

| # | Upgrade | Why first | Effort | Impact |
|--:|---|---|---|---|
| 1 | **Trajectory-based fire decision** (replace the fire window) | Convert correct tracking into real DPS; score the shot over a 50–150 ms future horizon using predicted error + gimbal dynamic saturation + dwell time + fire delay, not an instantaneous angle | 2–3 wk | **Very high** |
| 2 | **Per-component latency ledger** | Sum measured tx + infer + serial + gimbal + fire per frame; the killer is latency *variance*, not mean | 1–2 wk | **Very high** |
| 3 | **Gimbal delay model / LUT** | Settle time/overshoot vs Δyaw/Δpitch — makes the trajectory gate physical | 1–2 wk | High |
| 4 | **State-guided geometric ROI** + camera-shape-aware detector input (no letterbox) | Recall at distance + less wasted compute; keep current YOLO | 1–2 wk | Med–high |
| 5 | **Sticky target scoring** (dwell + distance + visibility + lock bonus) | High value in 3v3, cheap | 3–5 d | Med–high |
| 6 | **Ego-motion compensation** (odom+IMU) | Decisive for shoot-on-move first-burst accuracy | 2–4 wk | High |
| 7 | **Robot-centric tracker** with image-space (reprojection) update | Track continuity, face-switch stability; run in shadow first | 4–6 wk | High |
| 8 | **Common world frame** + turret-aware navigation | Mostly 3v3 | 3–5 wk | Med–high |
| 9 | Explicit ω̇ / full MHT | Not the public differentiator | 3–6 wk | Low–med (defer) |

**Connection to the §4 fix:** Fix 1 already takes the first step toward roadmap
#1 — it stops treating a tight instantaneous facing angle as fire permission for
non-spinning targets. The natural continuation is the **hit-probability gate**:
predict the bullet impact point at flight time and fire iff it lands inside the
plate's physical rectangle (project half-width/half-height through obliquity),
for spinners and static alike. That subsumes both `margin` and `static_facing_max`
with one physical, bullet-economical test. Pair it with roadmap #2/#3 (the
latency ledger already partly exists via `use_measured_latency`) so the future
horizon it scores over is trustworthy.

Also queued (smaller, from the §4 work): **PnP yaw via normal vector** instead of
`getRPY` (§4.6.2) — the weak input feeding vyaw and the facing gate.

---

## 6. `/debug_state` field glossary (the diagnostic surface)

| Field | Meaning |
|---|---|
| `state` / `state_id` | LOST / DETECTING / TRACKING / TEMP_LOST |
| `aim_fire` | Stage A result: tracker target solution + range + regime-specific facing gate |
| `cmd_fire` / `fire_decision` | Final fire (Stage A **and** Stage B lock) |
| `yaw_locked`/`pitch_locked` | Stage B: gimbal within range-scaled lock tol |
| `yaw_error`/`pitch_error` | command vs micro feedback [rad] |
| `margin` / `fire_margin` | `fire_window − |phase_error|`; required `>=0` for confirmed spin and unknown phase |
| `phase` / `phase_error` | facing angle: plate normal vs line of sight [rad] |
| `fire_window` | `angular_window · min(window_ref_dist/range,1)` |
| `vyaw` / `vyaw_rpm` | estimated spin rate (state `x(7)`) |
| `p_vyaw` | `P(7,7)`, vyaw covariance |
| `det_yaw` | measured plate-normal yaw fed to the EKF this frame |
| `state_yaw` | EKF yaw `x(6)` **before** the update |
| `matched` / `mahalanobis` | association accepted? and its distance |
| `matched_face_idx` / `last_matched_face_idx` | which of the 4 faces matched |
| `association_yaw_hypothesis` | 0=primary IPPE, 1=alternate IPPE, −1=position-only |
| `face_switch_reason` | why this aim face was chosen (`fire_valid`, `incoming_lookahead`, `fallback_*`, `hysteresis_hold_*`) |
| `phase_confident` | true when either `confirmed_spin` or `static_confirmed` is true |
| `confirmed_spin` | spin timing has accepted consistent same-direction ~90° handoffs and `|vyaw|` is above the spin threshold |
| `static_confirmed` | multiple stable same-face observations with low yaw error and low `|vyaw|`; only then static relaxed facing gate is allowed |
| `prediction_time` / `flight_time` / `prediction_bias` | horizon breakdown [s] |
| `radius` / `dz` | EKF radius `x(8)` / armor-pair height step |
| `fire_block_reason` | first failing gate (e.g. `phase_unknown_window_margin`, `spin_window_margin`, `static_facing_margin`, `yaw_unlocked`) |

**Offline-replay fields (WORKLOG §4d):** `det_x_raw/det_y_raw/det_z_raw/det_yaw_raw`
(selected detection in odom, pre-EKF), `ego_x/ego_y`, `state_xc/state_vxc/state_yc/
state_vyc/state_vza` (EKF center pos+vel), `spin_centroid_x/y`, `spin_omega`,
`spin_invalid_streak`.

**Expanded debug surface (WORKLOG §4g — added 2026-06-25 for follow/overshoot/aim work):**

| Field | Meaning |
|---|---|
| `adaptive_follow_w` | adaptive-follow weight: 1 = light damping (moving, lead held) … 0 = hard (stop/reversal); 1.0 when off / spinning |
| `follow_signal_raw` | the un-clamped follow signal: **<0 ⇒ reversal**, ≈0 stop, ≈1 moving, >1 accelerating |
| `realized_center_speed` | \|EMA of per-frame EKF-center velocity\| [m/s] (telemetry) |
| `spinning_regime` | regime flag the EKF used this frame (q_yaw/q_pos/yaw-trust; gates adaptive-follow OFF). Distinct from `confirmed_spin` (which gates the AIM) |
| `aim_target_x/y/z` | odom point the gimbal is actually aiming at (post center-aim/ballistics) |
| `applied_lead_vx` / `applied_lead_vy` | translational lead **actually folded into the aim** [m/s], AFTER `spin_trans_lead_cap` — compare to `state_vxc/vyc` to see how hard the cap bit |
| `applied_lead_vyaw` | rotational lead used [rad/s]; **0 when omega-gated** — compare to `vyaw` |
| `omega_reliable` | was the rotational lead applied (false ⇒ aliased/unconfirmed spin → center-aim only) |
| `center_aimed` | center-aim engaged (gimbal at the spin-centre facing point, not chasing a face) |
| `frame_dt` | detection interval [s]; **1/`frame_dt` = live detector Hz** (rate/aliasing diagnostic) |
| `pipeline_latency` | measured capture→aim latency [s]; horizon = this + `actuation_latency` |
| `num_target_detections` | target-class plates this frame (**0 ⇒ detector miss**, vs association failure) |
| `target_generation` | bumps on every track (re)acquire/switch — **spots track resets** that explain discontinuities |

---

## 7. Next action — ON-ROBOT VALIDATION (2026-06-23 rework applied, build OK)

Code is in and compiles. Validate on the standard robot:

1. Relaunch (`standard.launch.py` is symlink-installed; C++ was rebuilt).
2. Capture a fresh `log4.txt` with `/debug_state` while reproducing the old
   `log3` spinner case. First fields to inspect:
   - `confirmed_spin` should become true after consistent handoffs;
   - `jump_detected`, `one_face_jump`, `dt_jump`, `vyaw_est_from_timing`,
     `vyaw_timing_accepted`, `consecutive_same_dir_jumps`;
   - `matched_face_idx` should no longer stay wrong forever while the visible
     armor changes;
   - before `confirmed_spin`, `faces_checked` should be 1 and
     `face_lookahead_active` false;
   - after `confirmed_spin`, `faces_checked` should become 4 and lookahead can
     schedule incoming faces.
3. **Spin sign check:** if accepted jumps make `vyaw_rpm` plausible magnitude but
   opposite direction, flip the sign in `observeRawYawHandoff()`.
4. **Ghost aim check:** while `confirmed_spin=false`, the aim should stay on/near
   the directly observed armor, not a reconstructed armor beside the robot.
5. **Static face-off regression:** both robots still, facing. Expect
   `static_confirmed=true` after a few stable frames; `cmd_fire` should stay
   stable while `command_locked` holds.
6. **Moving-target regression:** strafing/receding target should still show reduced
   overshoot from §4b. If aim drifts after detections disappear, apply the next
   unknown-phase coast damping from §4c.6.
7. **Bench yaw-bias check** (§4.6): squared robots → is `phase ≈ 0` (real cant,
   done) or `≈ 0.3` (PnP-yaw bias → switch yaw extraction to the normal vector)?

If validated, mirror `face_index_switch_penalty` plus the earlier §4 parameters
into `hero`/`sentry` launches and commit.

### Reasoning ledger (most recent first)
- **2026-06-25 (applied, §4g debug-surface expansion)** — User asked for more
  `/debug_state` variables to debug the follow/overshoot/aim work data-driven. Added 14
  fields (msg now 127), documented in §6: the LEAD actually applied
  (`applied_lead_vx/vy/vyaw` — post-cap/post-omega-gate, vs `state_vxc/vyc`/`vyaw` to
  see how hard the cap/gate bit), the aim point (`aim_target_x/y/z`), the aim-mode
  decisions (`omega_reliable`, `center_aimed`, `spinning_regime`), the adaptive-follow
  internals (`follow_signal_raw` <0=reversal), and timing/health (`frame_dt` ⇒ live Hz,
  `pipeline_latency`, `num_target_detections`, `target_generation` ⇒ track resets).
  Plumbed via `AimResult` (lead/mode), `TrackerDebugInfo` (regime/signal), and node
  members (dt/det-count). `colcon build` OK; all 14 verified present in the regenerated
  msg. Keep recording — these make the next bag analysis self-contained.
- **2026-06-25 (applied, §4g.7 ADAPTIVE FOLLOW)** — User clarified: don't touch fire,
  improve the AIM — follow SUDDEN changes much faster AND overshoot LESS, switchable.
  No single `alpha_pos` does both (1.0 overshoots, 0.99 lags — the §4b↔§4d.9 loop), so
  added an innovation-gated horizontal-center velocity damping: `follow_signal = 1 +
  (innov·v)/(|v|²·dt)` (≈1 moving → light `alpha_move=1.0`; →0 stop / <0 reverse → hard
  `alpha_stop=0.70`), reacting in ONE frame. First tried a realized-velocity EMA — the
  offline CV-Kalman sim showed it detects the stop too LATE (slightly worse); the
  innovation reacts immediately: sim move→stop lag **54→43 mm** AND overshoot **75→60
  mm**. Gated to NON-spin (a spinner's orbit confounds the signal; it keeps
  `q_pos_spin`). Switch: `adaptive_follow_enable`. New debug fields `adaptive_follow_w`
  /`realized_center_speed`. The "honest but quiet" FIRE gating is separately switchable
  via existing `fire_require_impact_inside`/`asymmetric_fire_enable`. `colcon build` OK.
  **On-robot validation pending** (§4g.7). Confirmed the 280–340 s bag tail was the
  gimbal deliberately off (not a bug).
- **2026-06-25 (applied, §4g walked-spinner follow lag)** — Replayed the last bag
  (`debug_state_20260624_201048`, 15045 msgs / 336 s). It is a fast SPINNING TOP
  (armor path 50–90 m/10 s, net center ~0) walked across + brought close, NOT a
  translation test. Measured the "segue lentamente": while WALKED, the EKF center
  lags the true (orbit-averaged) center by **~66 ms / 16 cm** — a POSITION lag set by
  `q_pos_spin=2.0` (kept stiff to reject the orbit ghost), NOT a lead deficit (the
  spin velocity is already bled ~97 %/s + capped 0.30; the observer centroid velocity
  is confirmed garbage). Fix: **`q_pos_spin` 2.0 → 4.0** — doubles center
  responsiveness (lag → ~9 cm) while staying ≪ non-spin q_pos=10 (still rejects the
  orbit) and adding NO overshoot (velocity bleed + cap unchanged) and NO lock risk
  (center wobble ~2 mrad ≪ tol). `colcon build` OK. **On-robot validation pending**
  (§4g.6). Bag problems reported (§4g.5): the last 60 s is a PARKED GIMBAL (correct
  command, frozen `micro_yaw` 18° off → not a code bug, but pollutes the stats); LOCK
  is the fire bottleneck (`aim_fire` 46 % vs `cmd_fire` 17 %, tight 27 mrad far-range
  lock vs ~30 mrad aliased-spin jitter) — framerate-bound (§4f.A); the top is aliased
  at 44 Hz so the rotational lead is correctly off; impact/facing gates correctly
  refuse off-plate shots. Net: the deeper fire-rate lever remains detector FPS, not
  tracker tuning.
- **2026-06-24 (applied, §4f NYQUIST CEILING + observability rework)** — §4e on robot
  fixed the wheel-shot (off-plate 174→5) but exposed the real wall. Bag `…193721`:
  raw detection is excellent (4 mm still / 0.6 px reproj) but a SPINNING enemy moves
  the armor **119 mm = 32°/frame (p90 87°)** → true spin **≈235 rpm**, and at 44 Hz
  that is **ALIASED (Nyquist)**. PROVED the user-chosen circle-fit refactor ALSO
  fails offline (ω std ±108). So the limiter is the **detector rate, not the
  tracker**. Recalibrated to A+B+C+D: **A** latency-ledger instrumentation in the
  detector loop (find why 44 not ~120 Hz — serial blocking loop + per-frame JSON);
  **B** sticky center-aim (no regime-flip 90° jumps → gimbal locks); **C** revert
  erratic q_pos_spin (0.8→2.0) + `spin_trans_lead_cap=0.30` (kill phantom-velocity
  overshoot); **D** observability-gated rotational lead (`omega_reliable_max=15`
  rad/s — above it, no lead on the aliased garbage omega, pure center-aim +
  opportunistic fire). Honest: B/C/D fix lock+overshoot+timing; actually HITTING a
  235 rpm top needs A (higher fps). `colcon build` OK. **On-robot validation pending**
  — first deliverable: report `LATENCY[ms]` from the detector.
- **2026-06-23 (applied, §4e FULL FIRE-CONTROL REWORK)** — Bag `…193335` (spinner +
  translation): round 3 fixed STATE (confirmed_spin 81%, center not ghosting) but the
  FIRE-CONTROL layer was broken — 49% of fires off-plate (shoots wheels), phantom
  center velocity 0.3–1.2 m/s (translation overshoot), confirmed_spin toggles → aim
  flips → yaw_locked 59%. Root: aim point (center facing point) DECOUPLED from the
  fire gate (rotating-face 0.60 window). Applied 4 fixes: (1) **impact-inside-plate
  gate** — fire only if the predicted bullet impact lands on the selected plate's
  board (COD two-stage gate; the wheel-shot fix); (2) **regime-adaptive center
  stiffness** `q_pos_spin=0.8`/`alpha_pos_spin=0.90` — kills phantom velocity at the
  source (mirrors q_yaw_spin); (3) **subordinate the raw-yaw timing path to the
  position observer** — no more regime toggle / vyaw double-write; (4) `q_yaw_spin
  80→40`. Both (2) and (3) are AIM fixes (overshoot + regime-flip jumps), not just
  fire. Corrected the ref_freq misconception (it's a damping time-constant base, NOT
  a rate to match — real dt is already used; reverted 45→60). `colcon build` OK.
  **On-robot validation pending** (§4e.6). Offline: impact gate keeps ~73% of
  fire-eligible spinner frames, drops the 0.30–0.60 phase tail (the off-plate shots).
- **2026-06-23 (applied, §4d round 2)** — Ran §4d round-1 on robot (bag `…182411`):
  catch-22 broken (confirmed_spin 75%, faces cycle), but `cmd_fire` still 9%, aim
  "exits the robot", rpm oscillates 0↔70. Root: (1) face SELECTION still used the
  strict `margin>=0` while the GATE used the asymmetric window → lookahead aimed at
  the incoming face 90° away → no lock → no fire; (2) re-seed fired on every
  confirmed_spin toggle + single-frame invalid dropped confidence + bad-center ω
  spikes. Fixes: `fire_valid` = asymmetric gate (selection↔gate unified); re-seed
  once per episode; spin-hold grace (6 frames); ω outlier/max rejection; blend
  0.5→0.35. Build OK. **On-robot re-validation pending.**
- **2026-06-23 (applied, §4d)** — Validated the §4c rework with a STATIC log
  (`log4_still.txt`, now fires) and a real ~100 rpm SPINNER rosbag (`bagsa/`).
  Bag analysis: `confirmed_spin`/`jump_detected` 0/374, `matched_face_idx` 0/374,
  `cmd_fire` 10% — the §4c raw-yaw bootstrap **never triggered** (IPPE flip + strict
  gates + face pinning + static-tuned yaw distrust all block it), and the
  three-regime fire gate left every real frame in a strict "unknown phase"
  black-hole. Applied a root rework (user chose "rework + geometric fire gate"):
  **(A)** position/orbit-based spin observer (line-of-sight center estimate, median
  ω, IPPE-immune; offline-validated ω=10.7→10.0, static/translation rejected) with
  vyaw blend + fast spin-down + one-time EKF re-seed (ghost fix); **(B)**
  regime-adaptive yaw trust (`q_yaw_spin`, floor→1.0 while spinning, face de-pin on
  confirm); **(C)** asymmetric geometric fire gate (approach 0.60 / leave 0.25, COD
  55/20 idea) replacing the three regimes. `colcon build` OK. **On-robot validation
  pending** (protocol in §4d.6).
- **2026-06-23 (applied)** — User asked to apply the best root rework after
  reviewing `WORKLOG.md`, `IMPLEMENTATION_PLAN.md`, `log3.txt`,
  `ricerca1.md`, and `ricerca2.md`. Implemented the §4c focused rework:
  explicit static/spin regimes, raw-yaw handoff bootstrap, shared jump timing,
  pre-confirmation face-index penalty, observed-armor aim path while phase is
  unknown, geometry-learning freeze before phase confirmation, conservative
  unknown-phase fire gate, and new debug flags/reasons. `git diff --check` OK;
  `colcon build --packages-select autoaim --symlink-install --allow-overriding
  autoaim` OK.
- **2026-06-22 (diagnosis, deferred)** — Diagnosed spinning-enemy failure from
  `log3.txt` (§4c): **spin-bootstrap catch-22** — refactor replaced raw-yaw-jump
  detection with face-index transitions, which can't start from `vyaw≈0`, so
  `vyaw` never locks and `state_yaw` corrupts (ghost aim, no fire, slow recovery).
  Also flagged: the §4 static yaw-trust changes conflict with spinning. No code
  changed — wrote **`IMPLEMENTATION_PLAN.md`** (rework + quick wins + context) to
  execute later (user's choice).
- **2026-06-22 (applied, follow)** — Diagnosed moving-target overshoot/drift from
  `log2.txt` (§4b). Servo tracks `cmd_yaw` fine; the estimate itself lagged due to
  **isotropic over-smoothing** (15 cm modelled noise on the pixel-precise lateral
  channel) + **phantom vyaw** from EKF position↔yaw coupling. Applied **anisotropic
  position noise** (radial/tangential split) and **rotational-lead gating** (vyaw
  lead only for timing-confirmed spin). `colcon build` OK. Pending on-robot test.
- **2026-06-22 (applied)** — Implemented Fix 1+2+3 (user choice). `tracker.hpp`
  +3 config fields; `tracker.cpp` static facing gate / U-shaped yaw noise / IPPE
  hysteresis; `autoaim_node.cpp` param decls + `fireBlockReason` reorder;
  `standard.launch.py` `q_yaw 20→7` + 3 new params. `colcon build` OK (1m18s).
  Pending on-robot validation.
- **2026-06-22 (diagnosis)** — Diagnosed static fire indecision from `log1.txt`.
  Gimbal locked the whole time; only Stage A `margin≥0` toggles fire. Root cause =
  static facing error (~18°) eating the 0.35 window + vyaw phantom-spin drift
  (`q_yaw=20`/`alpha_yaw=1.0`) + IPPE yaw-hypothesis flip. Created this WORKLOG;
  absorbed and deleted `prompt1.md`.
- **2026-06-21** (`8982bc9`) — Implemented the multi-face refactor (§2) and the
  spinning revision (§3). DebugState expanded; viewer HUD updated.
