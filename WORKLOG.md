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
