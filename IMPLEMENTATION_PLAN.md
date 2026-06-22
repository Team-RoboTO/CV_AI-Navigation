# IMPLEMENTATION PLAN — Tracker Spin/Follow Rework (to do later)

> **Status:** PLANNING — nothing in this file is implemented yet. It is the
> agreed to-do list (the spin-bootstrap **rework** + the optional **quick wins**)
> plus the full context of the chats that produced it, so any device / any fresh
> session can pick it up.
>
> **Companions:** `INSTRUCTIONS.md` (build/run/params), `WORKLOG.md` (history +
> live diagnoses; see §4, §4b, §4c), `ricerca1.md` (Chinese-pipeline roadmap).
> **Branch:** `autoaim_improvement`. **Build:** `colcon build --packages-select
> autoaim --symlink-install --allow-overriding autoaim`.
> **Language:** English to match the code; user observations preserved faithfully.

---

## 0. TL;DR — what to implement, in order

1. **Quick wins (optional, low-risk, ~1h):** stop the worst visible misbehaviour
   now — off-screen drift, slow spin-down recovery, translation overshoot. §3.
2. **Spin-bootstrap rework (the real fix):** make `vyaw` actually lock on a
   spinning enemy by restoring **gated raw-yaw-jump detection** (independent of
   the EKF-predicted face), + **regime-adaptive yaw trust**, + **face-association
   stability**, + **dz/raised-armor** handling. §4.
3. **Validate** on a real spinner with a long log. §6.

The quick wins treat symptoms; the rework removes the root cause. Both are safe
to do in either order, but the rework is what fixes spinning targets for real.

---

## 1. Context — the conversation so far

Three `/debug_state` captures drove this work (all with **our robot stationary,
ego-motion disabled**, geometric/barrel offsets measured-correct per the user):

| Log | Scenario | Finding | Status |
|---|---|---|---|
| `log1.txt` | Both still, facing | Fire **flickered** on a perfectly-aimed static plate (facing window too strict + phantom vyaw + IPPE flip) | **Fixed** — WORKLOG §4 (Fix 1+2+3) |
| `log2.txt` | Enemy translating | **Overshoot/drift**; gimbal servo fine, the *estimate* lagged (isotropic over-smoothing + phantom vyaw coupling) | **Partly fixed** — WORKLOG §4b (anisotropic R + vyaw-lead gating) |
| `log3.txt` | Enemy **rotating** | **vyaw completely wrong**, ghost aim, never fires, slow recovery → spin estimation never bootstraps | **THIS PLAN** — WORKLOG §4c |

### 1.1 Already implemented (do not redo)
- **Fix 1** static facing gate: `spinning ? margin≥0 : |phase_error|≤static_facing_max(0.6)`.
- **Fix 2** U-shaped yaw trust (`yaw_facing_obs_floor=0.05`) + **`q_yaw 20→7`**.
- **Fix 3** IPPE alternate-solution hysteresis (`ippe_alt_penalty=1.0`).
- **Anisotropic position noise** (`r_pos_tang_base=0.025`, `r_pos_tang_slope=0.010`).
- **Rotational-lead gating** in `computeAim` (vyaw lead only when timing-confirmed).

> ⚠️ **Important caveat (drives §4.2):** Fix 2 (`q_yaw↓` + U-shape) was tuned for
> the STATIC case and **conflicts with the spinning case** — it makes the EKF
> slower and more reluctant to track a rotating yaw. The two regimes need
> opposite yaw-trust. The rework must make this **regime-adaptive**.

### 1.2 User-reported symptoms on the SPINNING/MOVING case (log3 + verbal)
Captured verbatim-faithful so nothing is lost:
1. "Still a lot of overshoot."
2. "When the enemy rotates (we were always stationary) the **vyaw is completely
   wrong**."
3. "It **never moves to the incoming armor first**; even at low rotation it
   doesn't follow the armors rotating out, doesn't shoot them; when it aims at
   the incoming armor (which may be **raised**) it makes a **huge jump** that
   throws the parameters way off."
4. "Still overshoot, and it **doesn't follow any enemy movement** (forward/back or
   lateral), **overestimating** and moving the aim too much, never firing (fire is
   correctly false — I mean what I see on the viewer)."
5. Question: "Is this because it **predicts where it will be**, or is it
   **fundamentally wrong**? Because it always says it won't fire for window margin."
6. "When the robot **exits the screen**, ours keeps moving slightly in the exit
   direction — is it our error?"
7. "Sometimes the aim is **completely off** the robot (like aiming at a **ghost
   robot** beside the enemy, mimicking its movements but shooting air), and it
   takes a while to recalibrate."
8. "That recalibration is **very slow even when the robot abruptly stops
   spinning** — not immediate/natural."

### 1.3 Hard constraints / facts the user gave
- **ARC ~750-ball cap** → optimize hit-probability-per-ball, never spray.
- All tests with **our robot stationary**, ego-motion disabled.
- Barrel/geometric offsets **measured correct**; do not chase those.
- Detector/PnP are clean (reproj ~0.01–0.5 px) — the weakness is downstream
  (state estimation), not detection.

---

## 2. Root cause (the spin-bootstrap catch-22)

From `log3.txt` (enemy rotating), 5 frames:

| | f1 | f2 | f3 | f4 | f5 |
|---|---|---|---|---|---|
| `det_yaw` (measured) | 0.66 | 0.73 | 0.79 | 0.69 | 0.62 |
| `state_yaw` (EKF) | −1.30 | −1.28 | −1.25 | −1.21 | −1.14 |
| `vyaw_rpm` | 0.41 | 1.42 | 2.16 | 2.89 | 3.55 |
| `matched_face_idx` | 1 | 1 | 1 | 1 | 1 |
| `jump_detected` | false×5 | | | | |

- **State yaw is wrong by ~90°+.** EKF thinks face-0 normal is −1.28 and reads the
  plate as **face 1** (predicted normal `−1.28+π/2 = 0.29`) vs measured 0.66–0.79
  → constant innovation `yd≈0.44` it barely corrects. Reconstructing
  `armor = center − r·[cosθ,sinθ]` from this wrong yaw places the aim off the
  real plate → **the "ghost robot."**
- **vyaw never locks:** it ramps 0.4→3.5 rpm (a real 小陀螺 is 200–300 rpm =
  20–30 rad/s). The fast timing estimator never fires.
- **The catch-22:** the refactor (prompt1.md Phase 4) **removed raw-yaw-jump
  detection** and replaced it with **`matched_face_idx` transitions**. But the
  face index only changes if the EKF predicts a ~90° rotation, which needs
  `vyaw≠0`, which needs a detected jump, which needs a face transition… So from
  spin-start (`vyaw≈0`) the EKF predicts no handoff → never detects the first
  handoff → `vyaw` never bootstraps. **Spin never starts being tracked.**

**Answer to user Q5:** it is **fundamentally the STATE** that is wrong, not the
prediction. At 1 m the lead is short (T≈0.1 s); even aiming at "now" misses
because the reconstructed plate is misplaced. Prediction only *amplifies* it. The
"won't fire for window margin" is a **symptom** of the wrong state (phase_error
computed from a wrong yaw), not a fire-tuning problem.

**Symptom → cause map:** #2 (vyaw) & #7 (ghost) = wrong yaw/no bootstrap; #3 & #4
(no incoming face / no follow / no fire) = `face_lookahead_active` never turns on
+ wrong phase; #6 (off-screen drift) = TEMP_LOST coast barely damped; #8 (slow
spin-down) = `alpha_yaw=1.0` never decays vyaw; #1 & #4 (translation overshoot) =
wrong-yaw-corrupted center inflates velocity + `q_pos=10`; #3 (raised-armor jump)
= `dz=0` no height model.

---

## 3. Quick wins (optional, low-risk) — implement first to stop the worst

### 3.1 Off-screen drift (symptom #6)
- **Cause:** in `TEMP_LOST`, `ekfPredict` coasts with the last velocity, damped by
  `alpha_coast`. Launch sets `alpha_coast=0.98` → at 60 Hz only ~62 % decays over
  the 0.4 s `lost_timeout`, so the aim keeps drifting in the exit direction.
- **Change:** `standard.launch.py` `alpha_coast 0.98 → 0.88` (≈`0.88^24≈0.05` over
  the timeout → velocity gone fast). Optionally also **freeze the command** (stop
  emitting new yaw/pitch targets, hold last) after `K=3` `TEMP_LOST` frames.
- **Risk:** low. A briefly-occluded spinner re-acquires on the next detection.
- **Validate:** target exits frame → aim settles within ~0.1–0.2 s, no drift.

### 3.2 Spin-down recovery (symptom #8)
- **Cause:** `alpha_yaw=1.0` (vyaw never decays) + `q_yaw=7` (slow EKF
  correction); after the spinner stops, vyaw stays high and the aim keeps rotating.
- **Change (preferred, targeted):** add a **spin-down detector** in `update()` —
  if measured yaw is stable (`|yd| < ε` for `N≈5` frames) AND no face transition,
  force `x_(7) → 0` (or decay hard) and `phase_timing_confident_ = false`.
- **Change (safety net):** `alpha_yaw 1.0 → 0.97` so an unconfirmed vyaw bleeds to
  ~0 over ~1 s (a real spinner is re-confirmed every handoff, so it stays).
- **Risk:** medium — gate so a genuine slow spin is not killed. Tie the decay to
  "no recent confirmed handoff."
- **Validate:** spinner stops abruptly → vyaw→0 within ~0.2–0.3 s.

### 3.3 Translation overshoot (symptoms #1, #4)
- **Cause:** `q_pos=10` (responsive velocity → overshoot) + wrong-yaw-corrupted
  center inflating velocity + `alpha_pos=0.995`.
- **Change:** `q_pos 10 → 5`. If still soft afterwards, try `alpha_pos → 1.0`
  (remove steady-state lead bias) — one at a time.
- **Note:** a large part of this overshoot is **downstream of the wrong yaw**
  (§2). Expect the §4 rework to remove more of it than `q_pos` alone.
- **Risk:** low–medium (lower q_pos → less responsive to real maneuvers; tune).
- **Validate:** strafing enemy → aim settles without overshoot on stop; `pd` drops.

---

## 4. The spin-bootstrap REWORK (the real fix)

Goal: `vyaw` locks to the true spin within ~2–3 handoffs and stays steady;
`state_yaw` stays consistent with the visible face; the aim tracks the rotating
plates and pre-aims the incoming one; spin-down is fast. Keep the EKF
architecture; restore robust bootstrap + make yaw-trust regime-aware.

### 4.1 Restore gated raw-yaw-jump spin detection  ← the linchpin
Detect the ~90° plate handoff from the **raw measurement sequence**, independent
of the EKF-predicted face (this is what breaks the catch-22).

- Track the **unwrapped measured visible-plate yaw** frame-to-frame (the node
  already inward-corrects `det.yaw`; keep an unwrapped running value).
- A **handoff candidate** = a step of ≈ ±π/2 in that unwrapped yaw between
  consecutive detections (the visible plate is replaced by its neighbour). Use
  the **bearing-relative** yaw `wrap(det_yaw − bearing)` so translation doesn't
  masquerade as a handoff.
- **Reuse the existing anti-spurious gates** (`vyaw_timing_max_reproj`,
  `vyaw_timing_consistency`, `vyaw_timing_min/max_dt`) so a PnP flip is not
  mistaken for a handoff.
- On a credible handoff: seed `x_(7) = (π/2)/dt_jump · dir` and set
  `phase_timing_confident_ = true`. **Do NOT reset the EKF** (stay faithful to the
  refactor — feed only the timing estimator).
- Once seeded, the EKF starts predicting handoffs and the existing
  `matched_face_idx`-transition path takes over for steady state.
- This is essentially a **better-gated `handleArmorJump`** (which the refactor
  removed) that drives only `vyaw`, not the whole state.
- **Acceptance:** from a standing start, a spinning enemy makes `jump_detected`
  fire on handoffs and `vyaw_rpm` reach the true rate within ~2–3 jumps.

### 4.2 Regime-adaptive yaw trust  ← resolves the static↔spin conflict
The static fix (U-shape + low `q_yaw`) must NOT apply when the target is spinning.

- Define a `spinning` flag = `phase_timing_confident_ && |vyaw| ≥ min`.
- **When spinning:** relax/disable `yaw_facing_obs_floor` (trust yaw evolution) and
  use a higher effective `q_yaw` so the EKF tracks the rotating yaw.
- **When static/translating:** keep the U-shape + low `q_yaw` (anti-phantom-spin).
- Implement as a scale factor on `yaw_facing_obs_floor` / `q_yaw` keyed off the
  flag (a couple of new params, defaulting to current static values).
- **Acceptance:** static face-off keeps the §4 static fix (no flicker, vyaw≈0);
  spinner tracks the yaw without lag.

### 4.3 Face-association stability (anti wrong-face lock-in)
Stop the spurious face switch that rotated `state_yaw` to −1.28.

- Only let `matched_face_idx` change when supported by the unwrapped yaw having
  rotated past the handoff boundary (**sequential** transitions; no 0→2 jumps),
  OR require the new face to beat the current by a Mahalanobis margin (a
  **face-index hysteresis**, analogous to `ippe_alt_penalty`).
- Do **not** let the EKF rotate the center yaw by ~π/2 to fit a face switch unless
  the spin is confirmed (§4.1).
- Consider locking the face index from the bootstrap until the next confirmed
  handoff.
- **Acceptance:** `matched_face_idx` cycles 0→1→2→3 in step with the real spin;
  `yd` stays small (<0.1); no −1.28-style center corruption.

### 4.4 dz / raised-armor handling (symptom #3, the "huge jump")
- **Cause:** `dz=0` (adapt disabled) models all 4 faces at one height; switching to
  a physically-raised armor causes a pitch jump the EKF then fights.
- **Change:** measure the real armor-pair height step for the standard robot and
  set `initial_dz` to it with verified sign, keeping `adapt_dz_enable=false`. Only
  enable `adapt_dz` after §4.1–4.3 make association stable.
- **Acceptance:** switching to the raised pair gives a smooth pitch change, no jump.

### 4.5 Risks & guardrails for the rework
- Touches the core estimator → **capture a long spin log first** (§6) and validate
  incrementally; keep every change behind a param so it reverts cleanly.
- Keep the §4 static guarantees (no flicker; vyaw≈0 when still).
- Do **not** widen `angular_window`, do **not** raise `maha_threshold`, keep the
  strict fire gate. Bullet economy (750 cap) stays paramount.

---

## 5. Open questions to resolve while implementing
- True spin rate of the test enemy? (Needed to validate the lock.) Measure from a
  long log: handoff period `dt` → `ω = (π/2)/dt`.
- Is the ~18° static facing error (WORKLOG §4) real cant or a PnP-yaw bias? The
  bench check (squared robots → is `phase≈0`?) is still pending and would also
  inform whether to switch yaw extraction to the **normal vector** (`R.col(0)`)
  instead of `getRPY` — relevant to §4.1/§4.3 because cleaner yaw = easier
  handoff detection.
- Should the aim fall back to the **directly observed armor** (not the
  reconstructed face) whenever spin is unconfirmed, to kill the ghost even before
  §4.1 lands? (Cheap robustness option.)

## 6. Validation protocol (run for each step)
1. `ros2 topic hz /detector/armors_keypoints` — confirm `ref_freq` (launch=60).
2. **Static face-off:** must keep WORKLOG §4 behaviour (no flicker, `vyaw≈0`).
3. **Slow rotation:** `matched_face_idx` cycles; aim follows the rotating plate;
   pre-aims the incoming one; fires when one is facing (`margin≥0`).
4. **Fast 小陀螺:** `vyaw_rpm` locks within ~2–3 jumps and is steady (not ramping);
   no ghost; no wheel/45° shots.
5. **Spin-stop:** `vyaw→0` within ~0.2–0.3 s.
6. **Off-screen exit:** aim stops within ~0.1–0.2 s (no drift).
7. **Translation:** follows fwd/back/lateral without overshoot; `pd` small.

## 7. Param reference (current live values, `standard.launch.py`)
`q_pos=10`, `q_yaw=7`, `q_r=1e-6`, `alpha_pos=0.995`, `alpha_yaw=1.0`,
`alpha_coast=0.98`, `r_pos_base=0.05`, `r_pos_slope=0.04`, `r_pos_tang_base=0.025`,
`r_pos_tang_slope=0.010`, `r_yaw_base=0.05`, `r_yaw_slope=0.005`,
`yaw_facing_obs_floor=0.05`, `max_oblique_deg=75`, `angular_window=0.35`,
`window_ref_dist=3.0`, `static_facing_max=0.6`, `ippe_alt_penalty=1.0`,
`face_lookahead_min_vyaw=0.35`, `face_lookahead_horizon=0.35`,
`vyaw_timing_min_dt=0.025`, `vyaw_timing_max_dt=0.700`,
`vyaw_timing_max_reproj=8.0`, `vyaw_timing_consistency=0.35`, `maha_threshold=25`,
`max_match_dist=1.1`, `initial_dz=0.0`, `adapt_dz_enable=False`, `bullet_speed=25`,
`gimbal_height=0.420`, `barrel_offset_x/y/z=0.0/0.02/−0.05`.
</content>
