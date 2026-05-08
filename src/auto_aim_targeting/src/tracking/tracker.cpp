#include "auto_aim_targeting/tracking/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// ============================================================================
// tracker.cpp — Single-robot EKF tracker ("spinning top" model).
//
// One Tracker instance per tracked robot.  TrackerManager owns a map
// of these and calls them in pipeline order each frame.
//
// ENTRYPOINTS (called by Trackers, in pipeline order):
//
//   constructor            Set state to LOST, zero vectors, store config.
//   init / initFromArmor   Bootstrap from first detection (LOST → DETECTING).
//   computeAdaptiveDamping Set per-tracker velocity decay before predict.
//   predictStep            EKF predict only (before global data association).
//   computeAssignmentCost  Mahalanobis distance for detection→tracker matching.
//   update                 Main per-frame loop: associate → classify → EKF update.
//   updateSmoothedAcceleration  EMA accel from innovation-based velocity delta.
//   snapshot               Extract immutable state snapshot for planning.
//   predictCenter/Face     Forward-predict for trajectory planning.
//
// PRIVATE HELPERS (stepdown — appear after the entrypoints that call them):
//
//   initEKF                 Seed 9D state from one PnP pose.
//   updateArmorsNum         Infer robot type (always NORMAL_4 for now).
//   handleArmorJump         Face-switch: yaw snap, radius swap, dz update.
//   orientationToYaw        Quaternion → continuous yaw (unwrapped).
//   getArmorPositionFromState  Center state → armor plate position.
//   updateFaceContext       Resolve which face index was just matched.
//   updateStationaryState   Detect stationary targets.
//   resetStationaryState    Clear stationary detection.
//   buildFaceBindingFromState  Build FaceBinding for a given face index.
// ============================================================================

namespace rm_auto_aim
{
Tracker::Tracker(const TrackerConfig & config)
: tracker_state(LOST),
  tracked_id(std::string("")),
  position_error_norm(0.0),
  yaw_error_abs(0.0),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  config_(config),
  // WHY π/3 (60°): 4-armor robots have faces 90° apart.  The yaw gate must be
  // wide enough to accept noisy PnP measurements (±15° at 3m) but narrow enough
  // to detect a 90° jump (face switch).  60° is the midpoint: a measurement
  // within ±60° of predicted yaw = same face; beyond 60° = different face.
  max_match_yaw_diff_(M_PI / 3.0)
{
}

// ---------------------------------------------------------------------------
// init — pick a target and bootstrap the EKF from scratch.
//
// Called when the tracker is in LOST state (no active target).  We need to
// pick ONE armor to start tracking.  Why closest-to-image-center?  Because
// edge-of-frame detections have worse PnP accuracy (more lens distortion,
// less resolution), and center detections are more likely to be the robot
// the operator is pointing at.
// ---------------------------------------------------------------------------
void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  double min_distance = DBL_MAX;
  this->tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      this->tracked_armor = armor;
    }
  }

  this->initEKF(this->tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  this->tracked_id = this->tracked_armor.number;
  this->tracker_state = DETECTING;

  this->updateArmorsNum(this->tracked_armor);
  this->setLastMeasurementTime(armors_msg->header.stamp);
  this->updateFaceContext(this->tracked_armor, this->target_state);
  this->resetStationaryState();
  this->last_measurement_point_ = this->tracked_armor.pose.position;
  this->has_last_measurement_point_ = true;
}

void Tracker::initFromArmor(const Armor & armor)
{
  this->tracked_armor = armor;
  this->initEKF(armor);
  this->tracked_id = armor.number;
  this->tracker_state = DETECTING;
  this->updateArmorsNum(armor);
  this->updateFaceContext(armor, this->target_state);
  this->resetStationaryState();
  this->last_measurement_point_ = armor.pose.position;
  this->has_last_measurement_point_ = true;
}

// ---------------------------------------------------------------------------
// update — the core per-frame tracking loop.
//
// This is the most complex function in the tracker.  Here's the big picture:
//
// PROBLEM: The camera sees one or more armor detections each frame.  We need
// to figure out which detection (if any) corresponds to the robot we're
// already tracking, then update our belief about where that robot is.
//
// APPROACH (in order):
//   1. PREDICT  — Advance the EKF forward in time using the motion model.
//                 This gives us a "best guess" of where the armor should be
//                 NOW, before looking at any new detections.
//
//   2. ASSOCIATE — Among all detected armors, find the one that best matches
//                  our prediction.  We only consider armors with the same
//                  YOLO class ID ("same_id_armors") because color = team.
//                  If multiple same-ID armors exist (robot showing 2 faces),
//                  we pick the one closest to our prediction in the statistical
//                  sense (Mahalanobis distance, not raw Euclidean).
//
//   3. CLASSIFY — Does the match look like:
//      MATCH: position close + yaw close     → normal EKF update
//      JUMP:  position close + yaw far off   → robot rotated, new face visible
//      MISS:  position too far               → probably noise, skip update
//
//   4. UPDATE   — Feed the matched measurement into the EKF to correct our
//                 state estimate.  Also update the radius scalar KFs.
//
//   5. STATE MACHINE — Transition between DETECTING/TRACKING/TEMP_LOST/LOST
//                      based on whether we found a match this frame.
// ---------------------------------------------------------------------------
void Tracker::update(const Armors::SharedPtr & armors_msg)
{
  // === STEP 1: PREDICT ===
  // If predictStep() was already called (for global data association), skip
  // the predict and reuse its result.  Otherwise, do it here (backward compat).
  Eigen::VectorXd ekf_prediction;
  if (!this->predicted_) {
    ekf_prediction = this->ekf.predict();
    this->r_active_kf_.predict();
    this->r_other_kf_.predict();
    // Write scalar KF radius back into EKF state vector.
    // WHY DECOUPLE: radius is managed by its own 1D Kalman filter (more stable
    // for a quasi-static parameter than letting the 9D EKF handle it).  But the
    // EKF still has radius in its state vector (for the observation model h(x)).
    // decoupleState zeroes the cross-covariance P(8,j) and P(j,8) for j≠8, and
    // sets P(8,8) = r_kf.P.  This prevents the radius from influencing other
    // states' Kalman gains (and vice versa), effectively making it read-only
    // from the EKF's perspective.
    ekf_prediction(8) = this->r_active_kf_.x;
    this->ekf.setState(ekf_prediction);
    this->ekf.decoupleState(8, this->r_active_kf_.P);
    // Store prior velocities for innovation-based acceleration estimation.
    // These are the EKF's predicted velocities BEFORE seeing the new detection.
    // After ekf.update(), the posterior velocities will differ by the Kalman
    // correction.  The difference (v_post - v_prior) / dt is the EKF's implicit
    // acceleration estimate — used by the tracker node to populate the Target
    // message's acceleration field (see fillTargetMsg in tracker_node.cpp).
    //
    // WHY HERE AND NOT AFTER ekf.update():  We need the velocity state at the
    // prior (predicted) stage, which is overwritten by ekf.update().  Capturing
    // it here ensures we have the correct pre-update values.
    this->prior_vx = ekf_prediction(1);  // v_xc predicted
    this->prior_vy = ekf_prediction(3);  // v_yc predicted
    this->prior_vz = ekf_prediction(5);  // v_za predicted
  } else {
    // predictStep() already ran — target_state holds the prediction.
    // prior_vx/vy/vz were already set in predictStep().
    ekf_prediction = this->target_state;
  }
  this->predicted_ = false;
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

  bool matched = false;
  this->measurement_valid = false;
  // Default: if we don't find a match, the EKF just coasts on its prediction.
  this->target_state = ekf_prediction;

  // When two faces of the same robot are visible simultaneously, we can measure
  // the OTHER pair's radius/height without waiting for a jump.  We defer this
  // measurement until after the EKF update so we use the corrected (posterior)
  // center position, which is more accurate than the raw prediction.
  bool has_other_pair_armor = false;
  double other_pair_x = 0.0, other_pair_y = 0.0, other_pair_z = 0.0;
  double other_pair_yaw = 0.0;

  if (!armors_msg->armors.empty()) {
    // === STEP 2: ASSOCIATE ===
    // Only consider detections with the same YOLO class ID as our target.
    // Class ID encodes color (red/blue), so this filters out teammates.
    std::vector<Armor> same_id_armors;
    auto predicted_position = this->getArmorPositionFromState(ekf_prediction);

    for (const auto & armor : armors_msg->armors) {
      if (armor.number == this->tracked_id) {
        same_id_armors.push_back(armor);
      }
    }

    if (!same_id_armors.empty()) {
      Armor selected_armor;

      if (same_id_armors.size() > 1) {
        // Multiple same-ID armors visible (robot showing 2+ faces).
        // Use Mahalanobis distance (not Euclidean) to pick the best match.
        // WHY: Mahalanobis accounts for the EKF's uncertainty shape — if the
        // filter is very confident in X but uncertain in Y, a candidate that
        // is close in X but far in Y is penalised correctly.  Euclidean
        // distance treats all axes equally and can pick the wrong face.
        double min_maha = DBL_MAX;
        double predicted_yaw = ekf_prediction(6);
        int selected_idx = 0;
        for (size_t i = 0; i < same_id_armors.size(); i++) {
          auto p = same_id_armors[i].pose.position;
          // Extract raw yaw without updating last_yaw_ (non-destructive)
          tf2::Quaternion tf_q;
          tf2::fromMsg(same_id_armors[i].pose.orientation, tf_q);
          double roll, pitch, raw_yaw;
          tf2::Matrix3x3(tf_q).getRPY(roll, pitch, raw_yaw);
          // Unwrap relative to EKF prediction so the innovation is correct
          double unwrapped_yaw = predicted_yaw +
            angles::shortest_angular_distance(predicted_yaw, raw_yaw);

          Eigen::Vector4d z(p.x, p.y, p.z, unwrapped_yaw);
          double maha = this->ekf.mahalanobis(z);
          if (maha < min_maha) {
            min_maha = maha;
            selected_armor = same_id_armors[i];
            selected_idx = static_cast<int>(i);
          }
        }

        // If we see TWO faces of the same robot simultaneously, we can measure
        // the OTHER pair's geometry (r2, dz) right now instead of waiting for
        // a jump event.  This is more accurate because both measurements come
        // from the same frame (no temporal drift between them).
        //
        // WHY DEFERRED: We store the data now but process it AFTER the EKF
        // update (below), so we use the corrected center position.  Processing
        // it here with the raw prediction would introduce velocity-dependent
        // bias into the radius measurement.
        //
        // WHY RAW RPY: We extract yaw via getRPY() instead of orientationToYaw()
        // because the latter updates last_yaw_ (the unwrapping state), and we
        // haven't decided which armor is the "primary" one yet.  Calling
        // orientationToYaw on the wrong one would corrupt future yaw unwrapping.
        if (this->tracked_armors_num == ArmorsNum::NORMAL_4) {
          tf2::Quaternion sel_q;
          tf2::fromMsg(selected_armor.pose.orientation, sel_q);
          double sel_roll, sel_pitch, sel_yaw;
          tf2::Matrix3x3(sel_q).getRPY(sel_roll, sel_pitch, sel_yaw);

          for (size_t i = 0; i < same_id_armors.size(); i++) {
            if (static_cast<int>(i) == selected_idx) continue;
            // Reject duplicates: if within 10cm, it's the same physical plate.
            // WHY 10cm: two detections of the same plate (from overlapping YOLO boxes or
            // minor bbox noise) can be up to ~5-8cm apart due to PnP error.  10cm rejects
            // these while being small enough to never confuse two physically separate faces
            // (minimum separation at r=0.22m is ~31cm at 90° apart).
            double ddx = same_id_armors[i].pose.position.x - selected_armor.pose.position.x;
            double ddy = same_id_armors[i].pose.position.y - selected_armor.pose.position.y;
            double ddz = same_id_armors[i].pose.position.z - selected_armor.pose.position.z;
            if (std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz) < 0.10) continue;

            tf2::Quaternion other_q;
            tf2::fromMsg(same_id_armors[i].pose.orientation, other_q);
            double other_roll, other_pitch, other_yaw;
            tf2::Matrix3x3(other_q).getRPY(other_roll, other_pitch, other_yaw);
            double yaw_sep = std::abs(angles::shortest_angular_distance(sel_yaw, other_yaw));
            // ~90° separation means different pair
            if (yaw_sep > M_PI / 4.0 && yaw_sep < 3.0 * M_PI / 4.0) {
              has_other_pair_armor = true;
              other_pair_x = same_id_armors[i].pose.position.x;
              other_pair_y = same_id_armors[i].pose.position.y;
              other_pair_z = same_id_armors[i].pose.position.z;
              other_pair_yaw = other_yaw;  // raw RPY yaw for radial projection
              break;  // only one other-pair armor expected
            }
          }
        }
      } else {
        selected_armor = same_id_armors[0];
      }

      // Compute position and yaw differences for the selected armor
      auto p = selected_armor.pose.position;
      Eigen::Vector3d position_vec(p.x, p.y, p.z);
      double position_diff = (predicted_position - position_vec).norm();
      double measured_yaw = this->orientationToYaw(selected_armor.pose.orientation);
      double yaw_innov_signed =
        angles::shortest_angular_distance(ekf_prediction(6), measured_yaw);
      double yaw_diff = std::abs(yaw_innov_signed);

      // === OBLIQUENESS CHECK ===
      // WHY THIS EXISTS: When a robot is turning, one face might be nearly
      // edge-on to the camera (>65°).  At such extreme angles, PnP cannot
      // resolve the yaw reliably — the armor plate is just a thin line of
      // pixels, so the orientation becomes essentially random noise.
      //
      // Without this check, the large yaw error would look like an "armor jump"
      // and trigger handleArmorJump(), which snaps yaw and swaps radii —
      // completely wrong when the robot hasn't actually switched faces.
      //
      // Solution: compute how oblique the view is.  If it's beyond the
      // threshold, we IGNORE the yaw gate entirely (treat as "match" regardless
      // of yaw_diff).  The R matrix already inflates yaw noise at high obliquity,
      // so the Kalman gain for yaw → 0 and the bad yaw measurement is harmlessly
      // ignored.  The XYZ position is still fused normally.
      // Compute face obliqueness: the angle between the armor's outward
      // normal (measured_yaw) and the direction from armor to camera.
      //
      //   bearing_opp = atan2(-py, -px) = direction FROM armor TOWARD camera
      //   face_angle  = |measured_yaw − bearing_opp|  (mod 2π, in [0, π])
      //
      //   face_angle = 0°  → armor is perfectly face-on to camera (ideal)
      //   face_angle = 90° → armor is edge-on (only a thin sliver visible)
      //
      // At high obliquity, PnP cannot resolve yaw reliably, so we flag it
      // to bypass the yaw gate (prevents false armor-jump triggers).
      double face_angle = 0.0;
      if (p.x * p.x + p.y * p.y > 0.01) {  // guard: target not at camera origin
        double bearing_opp = std::atan2(-p.y, -p.x);
        face_angle = std::abs(std::remainder(measured_yaw - bearing_opp, 2.0 * M_PI));
      }
      const double max_yaw_oblique_rad = this->config_.max_yaw_oblique_deg * M_PI / 180.0;
      bool yaw_oblique = face_angle > max_yaw_oblique_rad;

      this->position_error_norm = position_diff;
      this->yaw_error_abs = yaw_diff;
      this->yaw_innovation = yaw_innov_signed;
      this->position_innovation = Eigen::Vector3d(p.x, p.y, p.z) - predicted_position;
      this->face_obliquity_angle = face_angle;
      this->tracked_armor = selected_armor;

      // === STEP 3: CLASSIFY — is this a normal match, an armor jump, or noise? ===
      //
      // MATCH (position close, yaw close or oblique):
      //   The most common case.  The detected armor is the same face we've been
      //   tracking.  Feed it into the EKF to correct our state estimate.
      //
      // JUMP (position close, yaw far off, NOT oblique):
      //   The robot rotated far enough (~90° or ~180°) that a DIFFERENT face is
      //   now the one closest to us.  This is normal for spinning robots — every
      //   fraction of a second, a new face comes into view.  We need special
      //   handling: swap radii, snap yaw, recalculate center position.
      //
      // MISS (position too far):
      //   The detection is far from where we expect.  This usually means either
      //   (a) a false positive from YOLO, or (b) the EKF has diverged.  We
      //   ignore it to prevent corrupting the filter with bad data.
      //
      // WHY A YAW GATE AT ALL?  Without it, a slowly rotating robot would always
      // look like a "match" (position is close), and we'd feed a yaw measurement
      // that's 90° off into the EKF.  The yaw gate catches the moment the visible
      // face switches, so we can handle it correctly (handleArmorJump).
      //
      // POSITION PRE-GATE: kept as a coarse outlier reject only.  PnP yaw noise
      // on near-planar targets (e.g. visible-light test on a phone screen) makes
      // the EKF-derived face position wobble well past tens of cm, while the
      // measurement itself is still statistically consistent.  The Mahalanobis
      // gate at maha < mahalanobis_match_gate (≈ χ²₄ 99% quantile) inside the
      // MATCH branch is the real statistical rejection — the position gate just
      // keeps wildly far detections from reaching Mahalanobis at all.
      if (position_diff < this->config_.max_match_distance &&
          (yaw_diff < this->max_match_yaw_diff_ || yaw_oblique)) {
        // Matched armor found — verify with Mahalanobis before fusing.
        // WHY A SECOND GATE (Mahalanobis) AFTER POSITION+YAW?
        // The position/yaw gates are simple axis-aligned checks.  Mahalanobis
        // is a *statistical* gate that accounts for the EKF's covariance shape.
        // A detection might pass the loose position gate but be statistically
        // improbable given the filter's current uncertainty — e.g. if P is tiny
        // in one axis, even a small offset is multiple-sigma unlikely.
        // The Mahalanobis gate catches these cases and prevents filter corruption.
        if (yaw_oblique && yaw_diff >= this->max_match_yaw_diff_) {
          RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"),
            "Oblique view (%.0f°) — bypassing yaw gate (yaw_diff=%.1f°), R_yaw inflated",
            face_angle * 180.0 / M_PI, yaw_diff * 180.0 / M_PI);
        }
        this->measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
        double maha = this->ekf.mahalanobis(this->measurement);
        // WHY 13.3: Mahalanobis² follows a chi-squared distribution with k DOF
        // (k=4 for our [x,y,z,yaw] measurement).  The 99% quantile of χ²(4) is
        // ~13.3.  This means: if the detection is truly from our tracked target,
        // there's only a 1% chance of exceeding 13.3.  Values above this almost
        // certainly indicate a wrong detection or severe filter divergence.
        if (maha < this->config_.mahalanobis_match_gate) {
          matched = true;
          this->measurement_valid = true;
          this->target_state = this->ekf.update(this->measurement);

          // === RADIUS ADAPTATION ===
          //
          // GOAL: Estimate the orbit radius r (distance from robot center to
          // armor plate) using the corrected EKF state (posterior, not prior).
          //
          // METHOD: Project the (center − armor) vector onto the yaw direction.
          //   r_measured = (xc − xa)·cos(yaw) + (yc − ya)·sin(yaw)
          //
          // WHY PROJECTION? The center-to-armor vector is [dx, dy], but the
          // armor sits along the yaw direction.  The dot product with
          // [cos(yaw), sin(yaw)] extracts the radial component, rejecting
          // any tangential error from imperfect center estimation.
          //
          // GUARDS:
          //   - Skip when oblique (face_angle > threshold): at edge-on views,
          //     the yaw estimate is unreliable → projection degenerates.
          //   - Skip at long range (> radius_adaptation_max_range): PnP noise grows
          //     and the radius measurement becomes too noisy to be useful.
          //   - Clamp r_measured to [0.10, 0.45]: physically plausible bounds.
          //   - Inflate R of the scalar KF by yaw uncertainty: if the EKF
          //     isn't sure about yaw, the radius measurement is less reliable.
          {
            double yaw = this->target_state(6);
            // Check obliquity using posterior yaw (more accurate than prior)
            double bearing_opp_r = std::atan2(-this->measurement(1), -this->measurement(0));
            double face_angle_r = std::abs(std::remainder(yaw - bearing_opp_r, 2.0 * M_PI));
            if (face_angle_r < max_yaw_oblique_rad) {
              // Project center-to-armor vector onto yaw direction → radial distance
              double dx = this->target_state(0) - this->measurement(0);  // xc - xa
              double dy = this->target_state(2) - this->measurement(1);  // yc - ya
              double r_measured = dx * std::cos(yaw) + dy * std::sin(yaw);
              double range = std::sqrt(
                this->target_state(0) * this->target_state(0) +
                this->target_state(2) * this->target_state(2) +
                this->target_state(4) * this->target_state(4));
              if (range < this->config_.radius_adaptation_max_range &&
                  r_measured > 0.10 && r_measured < 0.45) {
                // Inflate the scalar KF's R by yaw uncertainty: uncertain yaw
                // means the projection direction is wrong, so trust r_measured less
                double yaw_var = this->ekf.getVariance(6);
                double yaw_factor = 1.0 + this->config_.radius_yaw_uncertainty_scale * yaw_var;
                double saved_R = this->r_active_kf_.R;
                this->r_active_kf_.R = saved_R * yaw_factor;  // temporarily inflate R
                this->r_active_kf_.update(r_measured);
                this->r_active_kf_.R = saved_R;                // restore original R
              }
            }
            // Write back the KF's current radius estimate into the EKF state
            this->target_state(8) = this->r_active_kf_.x;
          }

          // Other pair: deferred multi-armor measurement using posterior center.
          // Skip when other-pair face is edge-on (oblique > threshold).
          if (has_other_pair_armor) {
            double bearing_opp_o = std::atan2(-other_pair_y, -other_pair_x);
            double face_angle_o = std::abs(std::remainder(other_pair_yaw - bearing_opp_o, 2.0 * M_PI));
            if (face_angle_o < max_yaw_oblique_rad) {
              double odx = this->target_state(0) - other_pair_x;
              double ody = this->target_state(2) - other_pair_y;
              double r_other_measured =
                odx * std::cos(other_pair_yaw) + ody * std::sin(other_pair_yaw);
              if (r_other_measured > 0.10 && r_other_measured < 0.45) {
                double yaw_var = this->ekf.getVariance(6);
                double yaw_factor = 1.0 + this->config_.radius_yaw_uncertainty_scale * yaw_var;
                double saved_R = this->r_other_kf_.R;
                this->r_other_kf_.R = saved_R * yaw_factor;
                this->r_other_kf_.update(r_other_measured);
                this->r_other_kf_.R = saved_R;
              }
            }
            this->another_r = this->r_other_kf_.x;

            // dz from same-frame observation (more accurate than jump-based:
            // both measurements are simultaneous, no temporal drift).
            double dz_measured = other_pair_z - this->target_state(4);
            dz_measured = std::max(-0.25, std::min(dz_measured, 0.25));
            if (!this->dz_initialized_) {
              this->dz = dz_measured;
              this->dz_initialized_ = true;
            } else {
              this->dz = this->config_.height_offset_smoothing_factor * dz_measured + (1.0 - this->config_.height_offset_smoothing_factor) * this->dz;
            }
          }

          // -----------------------------------------------------------------
          // SECONDARY FACE FUSION
          //
          // PROBLEM: The primary EKF update only fuses ONE armor face.
          //   When the robot shows two faces simultaneously (visible at the
          //   corner), we have a second independent measurement that contains
          //   information about the robot center from a ~90° different angle.
          //   Ignoring it wastes valuable geometric constraints.
          //
          // WHY IT HELPS (geometrically):
          //   The primary face constrains the center position along ONE
          //   direction (the face normal, i.e. yaw direction).  The
          //   secondary face constrains the center from ~90° away.
          //   Together, they constrain the center in 2D — reducing the
          //   position uncertainty from an elongated ellipse to a tight
          //   circle.  Quantitatively: σ_cross_range drops ~30-50% when
          //   both faces contribute.
          //
          // HOW IT WORKS:
          //   1. Compute the yaw offset from primary to secondary face
          //      (should be ~±π/2 for adjacent faces on a 4-armor robot).
          //      Round to nearest π/2 to reject PnP noise in the offset.
          //   2. Build h_2(x): the observation model for the secondary face:
          //        xa₂ = xc − r₂·cos(yaw + offset)
          //        ya₂ = yc − r₂·sin(yaw + offset)
          //        za₂ = za + dz
          //        yaw₂ = yaw + offset
          //   3. Build H_2 (Jacobian of h_2) and R_2 (noise, with
          //      secondary_r_inflation multiplier for safety — the secondary
          //      face may be more oblique or at worse PnP quality).
          //   4. Mahalanobis gate to reject outliers before fusing.
          //   5. Call ekf.updateWithModel() — a sequential update that
          //      treats the primary update's posterior as its prior.
          //
          // WHY DEFERRED (not done inline with the primary update):
          //   The primary EKF update corrects the center position first.
          //   Using the corrected center (posterior) as the starting point
          //   for the secondary update gives a better linearisation point
          //   (smaller Taylor approximation error in H_2).
          // -----------------------------------------------------------------
          if (has_other_pair_armor && this->config_.use_secondary_face_fusion) {
            double sel_yaw_val = this->measurement(3);
            double yaw_offset = angles::shortest_angular_distance(sel_yaw_val, other_pair_yaw);
            // Round to nearest ±π/2 (guard against PnP noise in yaw)
            yaw_offset = std::round(yaw_offset / (M_PI / 2)) * (M_PI / 2);

            // Current EKF yaw (from posterior after primary update)
            double current_yaw = this->target_state(6);
            double unwrapped_other_yaw = current_yaw + yaw_offset;
            double r_other = this->r_other_kf_.x;

            // Secondary measurement vector
            Eigen::Vector4d z_2(other_pair_x, other_pair_y, other_pair_z, unwrapped_other_yaw);

            // Predicted measurement from current state: h_2(x)
            Eigen::Vector4d z_pred_2;
            z_pred_2(0) = this->target_state(0) - r_other * std::cos(current_yaw + yaw_offset);
            z_pred_2(1) = this->target_state(2) - r_other * std::sin(current_yaw + yaw_offset);
            z_pred_2(2) = this->target_state(4) + this->dz;
            z_pred_2(3) = current_yaw + yaw_offset;

            // Jacobian H_2 (4×9)
            Eigen::MatrixXd H_2 = Eigen::MatrixXd::Zero(4, 9);
            H_2(0, 0) = 1.0;
            H_2(0, 6) = r_other * std::sin(current_yaw + yaw_offset);
            H_2(1, 2) = 1.0;
            H_2(1, 6) = -r_other * std::cos(current_yaw + yaw_offset);
            H_2(2, 4) = 1.0;
            H_2(3, 6) = 1.0;

            // R_2: replicate the u_r formula for the secondary face
            double dist_2 = std::sqrt(
              other_pair_x * other_pair_x +
              other_pair_y * other_pair_y +
              other_pair_z * other_pair_z);
            double ps_2 = this->config_.position_noise_base + this->config_.position_noise_slope * dist_2;
            double ys_2 = this->config_.yaw_noise_base + this->config_.yaw_noise_slope * dist_2;
            double xyz_af_2 = 1.0, yaw_af_2 = 1.0;
            if (other_pair_x * other_pair_x + other_pair_y * other_pair_y > 1e-12) {
              double bearing_opp_2 = std::atan2(-other_pair_y, -other_pair_x);
              double face_angle_2 = std::abs(
                std::remainder(other_pair_yaw - bearing_opp_2, 2.0 * M_PI));
              double cos_fa_2 = std::cos(face_angle_2);
              xyz_af_2 = 1.0 / std::max(cos_fa_2 * cos_fa_2, 0.04);
              double cos_pow_2 = std::pow(std::abs(cos_fa_2), this->config_.yaw_obliquity_exponent);
              yaw_af_2 = 1.0 / std::max(cos_pow_2, 1e-4);
              double max_oblique_rad = this->config_.max_yaw_oblique_deg * M_PI / 180.0;
              if (face_angle_2 > max_oblique_rad) {
                yaw_af_2 = 1e6;
              }
            }
            Eigen::Matrix4d R_2 = Eigen::Matrix4d::Zero();
            R_2(0, 0) = ps_2 * ps_2 * xyz_af_2 * this->config_.secondary_noise_multiplier;
            R_2(1, 1) = ps_2 * ps_2 * xyz_af_2 * this->config_.secondary_noise_multiplier;
            R_2(2, 2) = ps_2 * ps_2 * xyz_af_2 * this->config_.secondary_noise_multiplier;
            R_2(3, 3) = ys_2 * ys_2 * yaw_af_2 * this->config_.secondary_noise_multiplier;

            // Mahalanobis gate on the secondary update
            Eigen::Vector4d innov_2 = z_2 - z_pred_2;
            Eigen::MatrixXd S_2 = H_2 * this->ekf.P_post_view() * H_2.transpose() + R_2;
            Eigen::LDLT<Eigen::MatrixXd> S_2_ldlt(S_2);
            double maha_2 = 1e9;
            if (S_2_ldlt.info() == Eigen::Success && S_2_ldlt.isPositive()) {
              maha_2 = innov_2.transpose() * S_2_ldlt.solve(innov_2);
            }

            if (maha_2 < this->config_.secondary_mahalanobis_gate) {
              // Write current target_state into EKF before the secondary update
              this->ekf.setState(this->target_state);
              this->ekf.decoupleState(8, this->r_active_kf_.P);
              this->target_state = this->ekf.updateWithModel(z_2, z_pred_2, H_2, R_2);
              RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"),
                "Secondary face fusion (maha=%.1f)", maha_2);
            }
          }

          this->ekf.setState(this->target_state);
          // Sync EKF covariance for radius with the scalar KF: zero cross-terms
          // so the externally-managed r doesn't pollute Kalman gains or Mahalanobis.
          this->ekf.decoupleState(8, this->r_active_kf_.P);
          this->setLastMeasurementTime(armors_msg->header.stamp);
          this->updateFaceContext(selected_armor, this->target_state);
          this->updateStationaryState(selected_armor);
          RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update (maha=%.1f)", maha);
        } else {
          if (maha >= 1e8) {
            RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
              "EKF: singular innovation covariance (maha=%.0f) — possible divergence", maha);
          } else {
            RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
              "Measurement rejected: Mahalanobis=%.1f > %.1f", maha, this->config_.mahalanobis_match_gate);
          }
        }
      } else if (yaw_diff > this->max_match_yaw_diff_ && !yaw_oblique) {
        matched = true;
        this->measurement_valid = true;
        this->handleArmorJump(selected_armor);
        this->setLastMeasurementTime(armors_msg->header.stamp);
        this->updateFaceContext(selected_armor, this->target_state);
        this->updateStationaryState(selected_armor);
      } else {
        // No matched armor found
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
      }
    }
  }

  // Safety-net clamp on radius [0.12, 0.40] m.
  // WHY THESE BOUNDS: RoboMaster armor plates are physically 12-40cm from the
  // robot center.  Values outside this range are always wrong (EKF diverged,
  // scalar KF corrupted, or bad PnP measurement).  The clamp prevents a bad
  // radius from cascading into wrong center-to-armor offsets, which would
  // corrupt all subsequent h(x) observations and compound the divergence.
  if (this->target_state(8) < 0.12 || this->target_state(8) > 0.4) {
    this->target_state(8) = std::max(0.12, std::min(this->target_state(8), 0.4));
    this->r_active_kf_.x = this->target_state(8);
    this->r_active_kf_.P = this->config_.radius_filter_initial_covariance;  // reset KF uncertainty after clamp
    this->ekf.setState(this->target_state);
    this->ekf.decoupleState(8, this->r_active_kf_.P);
  }

  // Clamp v_yaw to physically plausible range.
  // RoboMaster robots cannot spin faster than ~config_.max_yaw_rate rad/s;
  // values beyond this indicate EKF divergence from a bad detection.
  if (std::abs(this->target_state(7)) > this->config_.max_yaw_rate) {
    this->target_state(7) = std::copysign(this->config_.max_yaw_rate, this->target_state(7));
    this->ekf.setState(this->target_state);
    this->ekf.resetCovariance();
    // resetCovariance restores P0 which has cross-terms = 0 for r,
    // but re-decouple to ensure P(8,8) matches the scalar KF.
    this->ekf.decoupleState(8, this->r_active_kf_.P);
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "v_yaw clamped — covariance reset");
  }

  // Tracking state machine:
  //
  // WHY A STATE MACHINE (not just "tracked / not tracked")?
  //   False positives from YOLO are frequent (reflections, overlapping robots,
  //   glare).  Without a confirmation phase (DETECTING), a single false-positive
  //   would immediately trigger gimbal movement and potentially fire.
  //   TEMP_LOST provides graceful degradation during brief occlusions
  //   (YOLO drops 1-3 frames when a face is partially occluded) without
  //   resetting the EKF, which is expensive to re-converge.
  //
  //   LOST ──► DETECTING ──(tracking_thres consecutive matches)──► TRACKING
  //                │                                                  │
  //            any miss                                            any miss
  //                ▼                                                  ▼
  //              LOST                                            TEMP_LOST
  //                                                           │          │
  //                                              match again  │          │ lost_thres misses
  //                                                           ▼          ▼
  //                                                        TRACKING     LOST
  if (this->tracker_state == DETECTING) {
    if (matched) {
      this->detect_count_++;
      this->lost_count_ = 0;
      if (this->detect_count_ >= this->tracking_thres) {
        this->detect_count_ = 0;
        this->tracker_state = TRACKING;
      }
    } else {
      // Allow several grace misses before dropping. PnP yaw noise on rotating
      // targets routinely causes 2-3 consecutive position/yaw gate failures
      // before the EKF center estimate converges. Killing too early forces a
      // respawn loop where v_yaw never gets estimated (each new tracker
      // initialises with v_yaw=0), which in turn keeps the EKF center wobbling.
      this->lost_count_++;
      if (this->lost_count_ > 3) {
        this->detect_count_ = 0;
        this->lost_count_ = 0;
        this->tracker_state = LOST;
        this->matched_face_ = FaceBinding{};
        this->resetStationaryState();
      }
    }
  } else if (this->tracker_state == TRACKING) {
    if (!matched) {
      // First missed frame: don't drop immediately, give the EKF a chance to
      // coast through a brief occlusion or detection dropout.
      this->tracker_state = TEMP_LOST;
      this->lost_count_++;
      if (this->matched_face_.valid) {
        this->matched_face_.fresh = false;
      }
    }
    // If matched: stay in TRACKING, nothing to do.
  } else if (this->tracker_state == TEMP_LOST) {
    if (!matched) {
      // Still no detection — increment miss counter and wait up to lost_thres
      // frames before giving up and resetting to LOST.
      this->lost_count_++;
      if (this->lost_count_ > this->lost_thres) {
        this->lost_count_ = 0;
        this->tracker_state = LOST;
        this->matched_face_ = FaceBinding{};
        this->resetStationaryState();
      }
    } else {
      // Detection recovered — snap back to TRACKING immediately without
      // requiring re-confirmation (detect_count_ threshold).
      this->tracker_state = TRACKING;
      this->lost_count_ = 0;
    }
  }

  // Drop target if it is beyond max tracking range
  if (this->tracker_state != LOST) {
    double xc = this->target_state(0), yc = this->target_state(2), za = this->target_state(4);
    double range = std::sqrt(xc * xc + yc * yc + za * za);
    if (range > this->config_.max_track_range) {
      this->tracker_state = LOST;
      this->matched_face_ = FaceBinding{};
      this->resetStationaryState();
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
        "Target beyond max range (%.1fm > %.1fm) — LOST", range, this->config_.max_track_range);
    }
  }

  // -----------------------------------------------------------------------
  // POST-UPDATE INVARIANT: Normalize yaw to [-π, π].
  //
  // WHY: orientationToYaw() unwraps yaw continuously (e.g. 3.1 → 3.2 → 3.3
  // instead of wrapping to -2.9).  This is necessary for v_yaw estimation —
  // without it, a wrap from +π to −π looks like a −2π jump and corrupts the
  // velocity estimate.  But over time, yaw can accumulate to −37+ rad.
  //
  // PROBLEM: Large absolute yaw values don't affect cos/sin (periodic), but:
  //   1. Float precision degrades at large values (cos(37.0) is less precise
  //      than cos(0.8) due to fewer significant digits in the fractional part).
  //   2. The trajectory solver computes face_yaw = yaw + i × face_spacing,
  //      and accumulated error compounds.
  //
  // SOLUTION: Normalize yaw back to [-π, π] at the end of every frame.
  // v_yaw is unaffected because it depends on yaw CHANGES, not absolute value.
  // We also sync last_yaw_ so the next frame's orientationToYaw() unwraps
  // relative to the normalized value.
  // -----------------------------------------------------------------------
  double wrapped_yaw = angles::normalize_angle(this->target_state(6));
  if (wrapped_yaw != this->target_state(6)) {
    this->target_state(6) = wrapped_yaw;
    this->last_yaw_ = wrapped_yaw;  // keep orientationToYaw unwrapping in sync
    this->ekf.setState(this->target_state);
  }
}

void Tracker::computeAdaptiveDamping(double dt)
{
  (void)dt;  // reserved for future time-dependent damping
  const double moving_position_decay =
    1.0 - 0.25 * (1.0 - this->config_.position_decay_baseline);
  const double moving_yaw_decay =
    1.0 - 0.25 * (1.0 - this->config_.yaw_decay_baseline);

  if (this->tracker_state == TRACKING) {
    Eigen::Vector3d vel(
      this->target_state(1), this->target_state(3), this->target_state(5));
    double speed = vel.norm();

    double dot = this->position_innovation.dot(vel);
    bool position_overshooting = (dot < 0.0) && (speed > 0.3);

    if (position_overshooting) {
      double scale = std::min(
        this->position_error_norm / this->config_.position_overshoot_threshold, 1.0);
      this->position_velocity_decay =
        1.0 - scale * (1.0 - this->config_.position_decay_baseline);
    } else if (speed > 0.3) {
      this->position_velocity_decay = moving_position_decay;
    } else {
      this->position_velocity_decay = this->config_.position_decay_baseline;
    }

    double v_yaw = this->target_state(7);
    bool yaw_overshooting =
      (this->yaw_innovation * v_yaw < 0) && (std::abs(v_yaw) > 0.5);

    if (yaw_overshooting) {
      double yaw_scale = std::min(
        this->yaw_error_abs / this->config_.yaw_overshoot_threshold, 1.0);
      double min_decay =
        this->config_.yaw_decay_baseline * this->config_.coast_decay_multiplier;
      this->yaw_velocity_decay = 1.0 - yaw_scale * (1.0 - min_decay);
    } else if (std::abs(v_yaw) > 0.5) {
      this->yaw_velocity_decay = moving_yaw_decay;
    } else {
      this->yaw_velocity_decay = this->config_.yaw_decay_baseline;
    }
  } else if (this->tracker_state == TEMP_LOST) {
    this->position_velocity_decay =
      this->config_.position_decay_baseline * this->config_.coast_decay_multiplier;
    this->yaw_velocity_decay =
      this->config_.yaw_decay_baseline * this->config_.coast_decay_multiplier;
  } else {
    this->position_velocity_decay = this->config_.position_decay_baseline;
    this->yaw_velocity_decay = this->config_.yaw_decay_baseline;
  }

  if (this->stationary_frame_count_ >= this->config_.stationary_required_frames) {
    this->position_velocity_decay =
      std::min(this->position_velocity_decay, this->config_.position_decay_baseline);
    this->yaw_velocity_decay =
      std::min(this->yaw_velocity_decay, this->config_.yaw_decay_baseline);

    for (int idx : {1, 3, 5, 7}) {
      this->target_state(idx) *= this->config_.stationary_velocity_collapse_factor;
      if (std::abs(this->target_state(idx)) < this->config_.stationary_zero_velocity_threshold) {
        this->target_state(idx) = 0.0;
      }
    }

    this->ekf.setState(this->target_state);
    this->ekf.decoupleState(8, this->r_active_kf_.P);
  }
}

// ---------------------------------------------------------------------------
// predictStep — EKF predict only (no measurement update).
//
// Called by the multi-tracker data association loop BEFORE update().
// After this, computeAssignmentCost() can be called to compute Mahalanobis
// distances for global detection-to-tracker assignment.
//
// Sets the predicted_ flag so update() skips its internal predict step.
// ---------------------------------------------------------------------------
Eigen::VectorXd Tracker::predictStep()
{
  Eigen::VectorXd ekf_prediction = this->ekf.predict();
  this->r_active_kf_.predict();
  this->r_other_kf_.predict();
  ekf_prediction(8) = this->r_active_kf_.x;
  this->ekf.setState(ekf_prediction);
  this->ekf.decoupleState(8, this->r_active_kf_.P);
  // Store prior velocities for innovation-based acceleration estimation.
  // Same rationale as in update()'s internal predict path above: capture the
  // EKF's velocity prediction BEFORE the measurement update, so the tracker
  // node can compute (v_posterior - v_prior) / dt as the acceleration signal.
  // predictStep() is the primary path (called by the multi-tracker data
  // association loop), so this is where prior_v* is set most of the time.
  this->prior_vx = ekf_prediction(1);  // v_xc predicted
  this->prior_vy = ekf_prediction(3);  // v_yc predicted
  this->prior_vz = ekf_prediction(5);  // v_za predicted
  this->target_state = ekf_prediction;
  this->predicted_ = true;
  return ekf_prediction;
}

// ---------------------------------------------------------------------------
// computeAssignmentCost — Mahalanobis distance from prediction to a detection.
//
// Used by the tracker node's global data association to decide which tracker
// each detection belongs to.  Does NOT modify last_yaw_ (the yaw unwrapping
// state), so it's safe to call for cross-tracker comparison without corrupting
// the tracker's internal state.
//
// Must be called after predictStep().
// ---------------------------------------------------------------------------
double Tracker::computeAssignmentCost(const Armor & armor)
{
  auto p = armor.pose.position;
  // Extract raw yaw without updating last_yaw_ (non-destructive)
  tf2::Quaternion tf_q;
  tf2::fromMsg(armor.pose.orientation, tf_q);
  double roll, pitch, raw_yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, raw_yaw);
  // Unwrap relative to EKF prediction so the innovation is correct
  double predicted_yaw = this->target_state(6);
  double unwrapped_yaw = predicted_yaw +
    angles::shortest_angular_distance(predicted_yaw, raw_yaw);
  Eigen::Vector4d z(p.x, p.y, p.z, unwrapped_yaw);
  return this->ekf.mahalanobis(z);
}

// ---------------------------------------------------------------------------
// initEKF — bootstrap the EKF state from a single armor detection.
//
// Given the armor plate position (xa, ya, za) and its yaw orientation,
// we back-calculate the robot center position:
//
//   center = armor + r · [cos(yaw), sin(yaw)]
//
// This is the inverse of getArmorPositionFromState().  All velocities are
// initialised to zero (we have no motion information from a single frame).
//
// Both scalar radius KFs are reset with initial guesses (r1, r2) and high
// covariance so they converge quickly once measurements start flowing.
// ---------------------------------------------------------------------------
void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  // Reset yaw unwrapping history so orientationToYaw starts fresh
  this->last_yaw_ = 0;
  double yaw = this->orientationToYaw(a.pose.orientation);

  // Back-calculate robot center from armor position:
  //   armor = center − r·[cos,sin]  ⟹  center = armor + r·[cos,sin]
  this->target_state = Eigen::VectorXd::Zero(9);
  double r = this->config_.initial_close_pair_orbit_radius;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);

  // Reset other-pair geometry estimates
  this->dz = 0;
  this->another_r = this->config_.initial_far_pair_orbit_radius;
  this->dz_initialized_ = false;

  // Reset scalar KFs with initial radii and configured noise parameters.
  // High P_init (σ ≈ 8cm) ensures the gain is large for the first few frames,
  // allowing the radius to converge quickly from the initial guess.
  this->r_active_kf_.reset(this->config_.initial_close_pair_orbit_radius, this->config_.radius_filter_initial_covariance);
  this->r_active_kf_.Q = this->config_.radius_filter_process_noise;
  this->r_active_kf_.R = this->config_.radius_filter_measurement_noise;
  this->r_other_kf_.reset(this->config_.initial_far_pair_orbit_radius, this->config_.radius_filter_initial_covariance);
  this->r_other_kf_.Q = this->config_.radius_filter_process_noise;
  this->r_other_kf_.R = this->config_.radius_filter_measurement_noise;

  //                xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
  this->target_state << xc, 0,    yc, 0,    za, 0,    yaw, 0,     r;

  this->ekf.setState(this->target_state);
}

// Color-based YOLO class IDs (0=blue,1=grey,2=purple,3=red) cannot distinguish
// robot types (hero, sentry, balance, outpost). Always assume NORMAL_4 so
// yaw-jump thresholds and trajectory face-spacing are correct for standard robots.
// BALANCE_2 / OUTPOST_3 enums are kept for future use when robot-type detection
// is available (e.g. via a separate classifier or communication from the lower computer).
void Tracker::updateArmorsNum(const Armor & armor)
{
  (void)armor;  // unused until robot-type detection is available
  this->tracked_armors_num = ArmorsNum::NORMAL_4;
  this->max_match_yaw_diff_ = M_PI / 3.0;  // ~60°
}

// ---------------------------------------------------------------------------
// handleArmorJump — process the event when a different face becomes visible.
//
// WHY THIS HAPPENS:
//   RoboMaster robots spin continuously.  With 4 armor plates spaced 90° apart,
//   every ~0.1-0.5s (depending on spin speed), the face pointing at us changes.
//   From the EKF's perspective, the measured yaw suddenly jumps by ~90° or ~180°.
//
// WHAT THIS FUNCTION DOES:
//   1. Detects spin reversals (v_yaw sign disagrees with jump direction)
//   2. Snaps the EKF's yaw to the new measurement
//   3. For 90° jumps (pair switch): swaps the radius KFs and updates dz
//      because even/odd faces have different radii and heights
//   4. Checks if the EKF has diverged (position too far off) and hard-resets if so
//   5. Inflates covariance to reflect the uncertainty introduced by snapping
//   6. Feeds the measurement through the EKF update step so center position
//      (xc, yc) gets properly corrected — without this, only yaw/radius/za
//      would be updated and the center would drift
//
// WHY NOT JUST LET THE EKF HANDLE IT?
//   A 90° yaw jump is far beyond the EKF's linear approximation range.  The
//   Jacobian H is linearized around the current yaw, and at 90° off, the
//   linearization is completely wrong — the EKF would either reject the
//   measurement (Mahalanobis too high) or corrupt the state.  Snapping
//   yaw first and then running the update gives the EKF a good linearization
//   point for correcting the position.
// ---------------------------------------------------------------------------
void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = this->orientationToYaw(current_armor.pose.orientation);
  // Is the robot spinning the same direction as before, or did it reverse?
  // jump_direction > 0 means the new face is CCW from the old face.
  // If that disagrees with v_yaw's sign, the robot changed spin direction.
  double jump_direction = angles::shortest_angular_distance(this->target_state(6), yaw);
  double jump_angle = std::abs(jump_direction);
  // Revert back to < 0 to correctly detect spin reversal when signs disagree
  if (jump_direction * this->target_state(7) < 0) {
    this->target_state(7) = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Spin reversal — v_yaw zeroed");
  }
  this->target_state(6) = yaw;
  this->updateArmorsNum(current_armor);
  // Only 4-armor robots have alternating radii and heights.
  // A 4-armor robot has two pairs of faces:
  //   Pair A (faces 0, 2): radius = r1, height = za
  //   Pair B (faces 1, 3): radius = r2, height = za + dz
  //
  // When the robot spins, the visible face switches:
  //   ~90° jump  → pair switch (A↔B): different radius and height
  //   ~180° jump → same pair (0↔2 or 1↔3): same radius and height
  if (this->tracked_armors_num == ArmorsNum::NORMAL_4) {
    // Classify jump angle: [45°, 135°] = pair switch, otherwise same pair
    bool is_pair_switch = (jump_angle > M_PI / 4.0) && (jump_angle < 3.0 * M_PI / 4.0);
    if (is_pair_switch) {
      // CRITICAL: Negate dz because the reference frame changed.
      // dz is defined as (za_other - za_current).  After a pair switch, the roles
      // of "current" and "other" swap, so the stored dz must be negated to stay
      // consistent.  Without this, the EMA receives measurements with alternating
      // sign on every pair switch, causing dz to converge toward zero on spinning
      // robots — effectively losing the height-difference estimate.
      if (this->dz_initialized_) {
        this->dz = -this->dz;
      }

      // Measure height difference between the old face (za in state) and new face
      // (current_armor.z).  EMA-smooth to reject single-frame PnP noise.
      double new_dz = this->target_state(4) - current_armor.pose.position.z;
      new_dz = std::max(-0.25, std::min(new_dz, 0.25));  // clamp to physical bounds
      if (!this->dz_initialized_) {
        this->dz = new_dz;              // First measurement: accept directly
        this->dz_initialized_ = true;
      } else {
        this->dz = this->config_.height_offset_smoothing_factor * new_dz + (1.0 - this->config_.height_offset_smoothing_factor) * this->dz;  // EMA
      }
      // Snap za to the new face's measured height
      this->target_state(4) = current_armor.pose.position.z;
      this->target_state(5) = 0.0;  // v_za corrupted by the za snap — zero it

      // Swap the radius scalar KFs: the KF that was tracking pair A now
      // tracks pair B, and vice versa.  Each KF retains its own covariance,
      // so the swapped-in KF (which hasn't been updated while unobserved)
      // has grown P → higher gain → fast re-convergence to the new pair's
      // actual radius.
      std::swap(this->r_active_kf_, this->r_other_kf_);
      this->target_state(8) = this->r_active_kf_.x;
      this->r_other_kf_.x = std::max(0.12, std::min(this->r_other_kf_.x, 0.4));
      this->another_r = this->r_other_kf_.x;
    }
  }
  this->yaw_innovation = 0.0;
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // ---------------------------------------------------------------------------
  // DIVERGENCE DETECTION & HARD RESET
  //
  // PROBLEM: After snapping yaw (and optionally za, r), the EKF's inferred
  // armor position (from the new state) may be far from where the actual
  // detection was.  This means the center position (xc, yc) is wrong —
  // the EKF has diverged, and no amount of Kalman updating can fix it
  // because the state is in the wrong basin.
  //
  // DETECTION: Compare the inferred armor position (from the post-snap
  // state) with the actual detection.  If the error > config_.max_match_distance
  // (~15cm), the center must be wrong.
  //
  // WHY THIS CAN HAPPEN:
  //   1. The EKF was tracking at a wrong center for several frames (e.g.
  //      velocity overshoot moved xc too far), and the jump exposed it.
  //   2. A jump was misclassified (90° instead of 180° or vice versa),
  //      so the wrong radius was applied to back-calculate center.
  //   3. Two nearby robots confused the data association, and the EKF
  //      latched onto the wrong one.
  //
  // SOLUTION: Hard reset everything — back-calculate center from the
  // current detection using the current radius estimate, zero all
  // velocities, restore P0, and reset both radius KFs and dz.
  // This is expensive (loses all velocity/acceleration history) but is
  // the only way to recover from a completely wrong center position.
  //
  // WHY NOT JUST INFLATE COVARIANCE?
  //   When xc is off by >15cm and P says σ=3cm, inflating P by 4× gives
  //   σ=6cm — still nowhere near the true error.  The Kalman gain won't
  //   be large enough to pull the state back.  A hard reset (P→P0 with
  //   σ=32cm for position, σ=1m/s for velocity) gives the filter room
  //   to re-converge from the new position within 3-5 frames.
  // ---------------------------------------------------------------------------
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = this->getArmorPositionFromState(this->target_state);
  if ((current_p - infer_p).norm() > this->config_.max_match_distance) {
    double r = this->target_state(8);
    this->target_state(0) = p.x + r * cos(yaw);  // xc (back-calculated from armor + radius)
    this->target_state(1) = 0;                   // vxc → 0 (no velocity info)
    this->target_state(2) = p.y + r * sin(yaw);  // yc
    this->target_state(3) = 0;                   // vyc → 0
    this->target_state(4) = p.z;                 // za (from detection)
    this->target_state(5) = 0;                   // vza → 0
    this->target_state(7) = 0;                   // v_yaw → 0 (spin direction unknown)
    this->ekf.resetCovariance();                 // P → P0 (high uncertainty for fast re-convergence)
    this->dz = 0.0;                              // reset geometry estimates
    this->another_r = this->config_.initial_far_pair_orbit_radius;
    this->dz_initialized_ = false;
    this->r_active_kf_.reset(this->config_.initial_close_pair_orbit_radius, this->config_.radius_filter_initial_covariance);
    this->r_active_kf_.Q = this->config_.radius_filter_process_noise;
    this->r_active_kf_.R = this->config_.radius_filter_measurement_noise;
    this->r_other_kf_.reset(this->config_.initial_far_pair_orbit_radius, this->config_.radius_filter_initial_covariance);
    this->r_other_kf_.Q = this->config_.radius_filter_process_noise;
    this->r_other_kf_.R = this->config_.radius_filter_measurement_noise;
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  this->ekf.setState(this->target_state);

  // ---------------------------------------------------------------------------
  // POST-JUMP COVARIANCE INFLATION
  //
  // WHY: The yaw (and optionally za, v_yaw) were written directly into the
  // state vector, bypassing the EKF's principled update mechanism.  The EKF
  // still thinks P(6,6) reflects the gradual accumulation of process noise —
  // it has no idea we just jumped yaw by 90°.  If we don't inflate, the
  // filter will be overconfident about the new yaw value.
  //
  // HOW MUCH: Factor of 4 on each snapped state → variance ×4 → σ ×2.
  //   If P(6,6) was 0.01 rad² (σ≈6°), inflation gives 0.04 rad² (σ≈12°).
  //   This is reasonable: the snapped yaw comes from a single PnP measurement
  //   (typical σ≈5-10° at 3m), so doubling the uncertainty reflects that we
  //   lost the multi-frame averaging benefit.
  //
  // WHY NOT BALANCE_2: A 180° jump on a 2-armor robot always triggers the
  // divergence reset above (the position error is always large because r
  // doesn't change but the face is on the opposite side).  resetCovariance()
  // already restores P0, which is even more conservative than 4× inflation.
  //
  // WHY ALSO za AND v_za FOR NORMAL_4: On a 90° pair switch, za was snapped
  // to the new face's height and v_za was zeroed.  Both are now uncertain.
  // ---------------------------------------------------------------------------
  if (this->tracked_armors_num != ArmorsNum::BALANCE_2) {
    this->ekf.inflateCovariance(6, 4.0);  // yaw:   variance ×4 (~2× std-dev)
    this->ekf.inflateCovariance(7, 4.0);  // v_yaw: variance ×4
  }
  if (this->tracked_armors_num == ArmorsNum::NORMAL_4) {
    this->ekf.inflateCovariance(4, 4.0);  // za:   variance ×4 (snapped to new face)
    this->ekf.inflateCovariance(5, 4.0);  // v_za: variance ×4 (zeroed, uncertain)
  }

  // ---------------------------------------------------------------------------
  // Sync prior → posterior so the subsequent ekf.update() uses the post-jump
  // state and inflated covariance as its linearisation point.  Without this,
  // update() would read the OLD x_pri/P_pri from predict() — computing
  // innovation and Kalman gain against a stale pre-jump state, which would
  // produce completely wrong corrections (the yaw innovation would be ~90°
  // instead of ~0°, overwhelming the linear approximation).
  // ---------------------------------------------------------------------------
  this->ekf.syncPrior();

  // Decouple radius before the EKF update so the Kalman gain for r ≈ 0
  // and the update doesn't introduce spurious cross-covariance.
  this->ekf.decoupleState(8, this->r_active_kf_.P);

  // ---------------------------------------------------------------------------
  // EKF UPDATE AFTER JUMP
  //
  // WHY NEEDED: Everything above (yaw snap, za snap, r swap, covariance
  // inflation) modified the state but didn't fuse the actual measurement.
  // The center position (xc, yc) is still at the raw prediction value from
  // predict() — the armor observation hasn't been used to correct it yet.
  //
  // By calling ekf.update() with the measurement, the Kalman gain pulls
  // xc and yc towards the observed armor position, using the post-jump
  // yaw and radius as the linearisation point.  This is critical: without
  // it, xc/yc would drift by the prediction error each jump, compounding
  // over multiple jumps.
  // ---------------------------------------------------------------------------
  this->measurement = Eigen::Vector4d(p.x, p.y, p.z, yaw);
  this->target_state = this->ekf.update(this->measurement);

  // Yaw-projected radius measurement for the newly visible pair.
  // Skip when face is nearly edge-on (oblique > threshold).
  {
    double yaw_post = this->target_state(6);
    double bearing_opp_j = std::atan2(-p.y, -p.x);
    double face_angle_j = std::abs(std::remainder(yaw_post - bearing_opp_j, 2.0 * M_PI));
    const double max_yaw_oblique_rad_j = this->config_.max_yaw_oblique_deg * M_PI / 180.0;
    if (face_angle_j < max_yaw_oblique_rad_j) {
      double dx = this->target_state(0) - p.x;  // xc - xa
      double dy = this->target_state(2) - p.y;  // yc - ya
      double r_measured = dx * std::cos(yaw_post) + dy * std::sin(yaw_post);
      if (r_measured > 0.10 && r_measured < 0.45) {
        double yaw_var = this->ekf.getVariance(6);
        double yaw_factor = 1.0 + this->config_.radius_yaw_uncertainty_scale * yaw_var;
        double saved_R = this->r_active_kf_.R;
        this->r_active_kf_.R = saved_R * yaw_factor;
        this->r_active_kf_.update(r_measured);
        this->r_active_kf_.R = saved_R;
      }
    }
    this->target_state(8) = this->r_active_kf_.x;
    this->ekf.setState(this->target_state);
    this->ekf.decoupleState(8, this->r_active_kf_.P);
  }
}

// ---------------------------------------------------------------------------
// orientationToYaw — convert quaternion to a *continuous* yaw angle.
//
// PnP returns yaw in [-π, π], which wraps discontinuously (e.g. π → -π).
// The EKF's v_yaw estimation relies on smooth yaw differences between frames.
// If yaw jumps from +3.1 to -3.1, the filter computes Δyaw ≈ -6.2 instead
// of the true +0.08 — corrupting v_yaw for many frames.
//
// Solution: unwrap each new yaw measurement relative to the previous one.
//   raw_yaw ∈ [-π, π]       (from RPY decomposition)
//   delta = shortest_angular_distance(last_yaw_, raw_yaw)   ∈ [-π, π]
//   unwrapped = last_yaw_ + delta    → continuous, can grow beyond ±π
//
// The absolute value of unwrapped yaw can drift (e.g. -37 rad after minutes
// of spinning).  This is normalised back to [-π, π] periodically in update()
// to avoid floating-point precision issues.
// ---------------------------------------------------------------------------
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Unwrap: add the shortest angular delta to last_yaw_ for continuity
  yaw = this->last_yaw_ + angles::shortest_angular_distance(this->last_yaw_, yaw);
  this->last_yaw_ = yaw;
  return yaw;
}

// ---------------------------------------------------------------------------
// getArmorPositionFromState — convert center-based EKF state to armor position.
//
// The EKF tracks the robot *center*, but measurements come from the *armor plate*.
// The plate is offset from center by radius r in the direction of yaw:
//
//   center = (xc, yc)
//   armor  = center − r · [cos(yaw), sin(yaw)]
//
// The minus sign is a convention: yaw points along the face outward normal,
// so the armor is behind the center when viewed from the front.
//
//     Camera ─────────→  [Armor plate]  ←─r─→  (Robot center)
//                         xa, ya                 xc, yc
//                         ← face normal (yaw) ───┘
// ---------------------------------------------------------------------------
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);  // Armor X = center X − r·cos(yaw)
  double ya = yc - r * sin(yaw);  // Armor Y = center Y − r·sin(yaw)
  return Eigen::Vector3d(xa, ya, za);
}

void Tracker::updateSmoothedAcceleration(double dt)
{
  if (dt < 0.005 || dt > 0.2) return;

  double raw_accel_x = (this->target_state(1) - this->prior_vx) / dt;
  double raw_accel_y = (this->target_state(3) - this->prior_vy) / dt;
  double raw_accel_z = (this->target_state(5) - this->prior_vz) / dt;

  double xyz_decay = std::max(
    std::pow(this->position_velocity_decay, dt * this->config_.refresh_frequency),
    1e-6);
  double damping_correction = (1.0 / xyz_decay - 1.0) / dt;
  raw_accel_x -= this->prior_vx * damping_correction;
  raw_accel_y -= this->prior_vy * damping_correction;
  raw_accel_z -= this->prior_vz * damping_correction;

  double time_adjusted_alpha = 1.0 - std::pow(
    1.0 - this->config_.acceleration_smoothing_factor,
    dt * this->config_.refresh_frequency);

  constexpr double kMaxPlausibleAccel = 12.0;
  auto clamp = [](double value, double limit) {
    return std::max(-limit, std::min(value, limit));
  };

  this->smoothed_acceleration_(0) =
    time_adjusted_alpha * clamp(raw_accel_x, kMaxPlausibleAccel) +
    (1.0 - time_adjusted_alpha) * this->smoothed_acceleration_(0);
  this->smoothed_acceleration_(1) =
    time_adjusted_alpha * clamp(raw_accel_y, kMaxPlausibleAccel) +
    (1.0 - time_adjusted_alpha) * this->smoothed_acceleration_(1);
  this->smoothed_acceleration_(2) =
    time_adjusted_alpha * clamp(raw_accel_z, kMaxPlausibleAccel) +
    (1.0 - time_adjusted_alpha) * this->smoothed_acceleration_(2);
}

TrackSnapshot Tracker::snapshot() const
{
  TrackSnapshot snapshot;
  snapshot.tracker_id = this->tracker_id;
  snapshot.tracker_state = this->tracker_state;
  snapshot.tracking =
    this->tracker_state == TRACKING || this->tracker_state == TEMP_LOST;
  snapshot.temp_lost = this->tracker_state == TEMP_LOST;
  snapshot.id = this->tracked_id;
  snapshot.armors_num = static_cast<int>(this->tracked_armors_num);
  snapshot.position.x = this->target_state(0);
  snapshot.position.y = this->target_state(2);
  snapshot.position.z = this->target_state(4);
  snapshot.velocity.x = this->target_state(1);
  snapshot.velocity.y = this->target_state(3);
  snapshot.velocity.z = this->target_state(5);
  const auto accel = this->smoothedAcceleration();
  snapshot.acceleration.x = accel(0);
  snapshot.acceleration.y = accel(1);
  snapshot.acceleration.z = accel(2);
  snapshot.yaw = this->target_state(6);
  snapshot.v_yaw = this->target_state(7);
  snapshot.radius_1 = this->target_state(8);
  snapshot.radius_2 = this->another_r;
  snapshot.dz = this->dz;
  snapshot.yaw_variance = this->ekf.getVariance(6);
  snapshot.v_yaw_variance = this->ekf.getVariance(7);
  snapshot.position_variance_x = this->ekf.getVariance(0);
  snapshot.position_variance_y = this->ekf.getVariance(2);
  snapshot.position_variance_z = this->ekf.getVariance(4);
  snapshot.last_measurement_stamp = this->last_measurement_time_;
  snapshot.measurement_fresh = this->matched_face_.fresh;
  snapshot.matched_face = this->matched_face_;
  snapshot.stationary_confidence = this->stationary_confidence;
  return snapshot;
}

Eigen::Vector3d Tracker::predictCenter(double dt) const
{
  Eigen::Vector3d center(this->target_state(0), this->target_state(2), this->target_state(4));
  Eigen::Vector3d velocity(this->target_state(1), this->target_state(3), this->target_state(5));
  return center + velocity * dt + 0.5 * this->smoothedAcceleration() * dt * dt;
}

FaceBinding Tracker::predictFace(int face_index, double dt) const
{
  FaceBinding face = this->buildFaceBindingFromState(this->target_state, face_index, false);
  const auto center = this->predictCenter(dt);
  const int faces = std::max<int>(static_cast<int>(this->tracked_armors_num), 1);
  const double face_spacing = 2.0 * M_PI / faces;
  const double future_yaw = this->target_state(6) + this->target_state(7) * dt + face_index * face_spacing;
  face.position.x = center.x() - face.radius * std::cos(future_yaw);
  face.position.y = center.y() - face.radius * std::sin(future_yaw);
  face.position.z = center.z() + face.dz_offset;
  face.yaw = future_yaw;

  const double face_to_camera = std::atan2(-face.position.y, -face.position.x);
  const double face_normal = future_yaw + M_PI;
  face.obliquity =
    std::abs(angles::shortest_angular_distance(face_normal, face_to_camera));
  face.visibility = std::clamp(std::cos(face.obliquity), 0.0, 1.0);
  face.observable =
    face.obliquity <= this->config_.visible_direct_obliquity_threshold_deg * M_PI / 180.0;
  face.fresh = this->matched_face_.fresh;
  return face;
}

FaceBinding Tracker::predictMatchedFace(double dt) const
{
  if (!this->matched_face_.valid) {
    return FaceBinding{};
  }
  FaceBinding face = this->matched_face_;
  const auto center = this->predictCenter(dt);
  face.position.x = center.x() - face.radius * std::cos(face.yaw + this->target_state(7) * dt);
  face.position.y = center.y() - face.radius * std::sin(face.yaw + this->target_state(7) * dt);
  face.position.z = center.z() + face.dz_offset;
  face.yaw = face.yaw + this->target_state(7) * dt;
  const double face_to_camera = std::atan2(-face.position.y, -face.position.x);
  const double face_normal = face.yaw + M_PI;
  face.obliquity =
    std::abs(angles::shortest_angular_distance(face_normal, face_to_camera));
  face.visibility = std::clamp(std::cos(face.obliquity), 0.0, 1.0);
  face.observable =
    face.obliquity <= this->config_.visible_direct_obliquity_threshold_deg * M_PI / 180.0;
  return face;
}

FaceBinding Tracker::buildFaceBindingFromState(
  const Eigen::VectorXd & x,
  int face_index,
  bool fresh) const
{
  FaceBinding face;
  const int faces = std::max<int>(static_cast<int>(this->tracked_armors_num), 1);
  const double face_spacing = 2.0 * M_PI / faces;
  const bool alternate_pair = (faces == 4) && (std::abs(face_index) % 2 == 1);
  const double radius = alternate_pair ? this->another_r : x(8);
  const double dz_offset = alternate_pair ? this->dz : 0.0;
  const double face_yaw = x(6) + face_index * face_spacing;

  face.valid = true;
  face.face_index = face_index;
  face.alternate_pair = alternate_pair;
  face.radius = radius;
  face.dz_offset = dz_offset;
  face.position.x = x(0) - radius * std::cos(face_yaw);
  face.position.y = x(2) - radius * std::sin(face_yaw);
  face.position.z = x(4) + dz_offset;
  face.yaw = face_yaw;

  const double face_to_camera = std::atan2(-face.position.y, -face.position.x);
  const double face_normal = face_yaw + M_PI;
  face.obliquity =
    std::abs(angles::shortest_angular_distance(face_normal, face_to_camera));
  face.visibility = std::clamp(std::cos(face.obliquity), 0.0, 1.0);
  face.observable =
    face.obliquity <= this->config_.visible_direct_obliquity_threshold_deg * M_PI / 180.0;
  face.fresh = fresh;
  return face;
}

void Tracker::updateFaceContext(const Armor & selected_armor, const Eigen::VectorXd & state)
{
  this->matched_face_ = resolveMatchedFace(
    state,
    this->tracked_armors_num,
    this->another_r,
    this->dz,
    selected_armor,
    this->config_.visible_direct_obliquity_threshold_deg);
  this->matched_face_.fresh = true;
}

void Tracker::updateStationaryState(const Armor & selected_armor)
{
  const auto & p = selected_armor.pose.position;
  const double speed = std::sqrt(
    this->target_state(1) * this->target_state(1) +
    this->target_state(3) * this->target_state(3) +
    this->target_state(5) * this->target_state(5));
  const double yaw_rate = std::abs(this->target_state(7));
  const double innovation = this->position_innovation.norm();

  if (this->has_last_measurement_point_) {
    const double dx = p.x - this->last_measurement_point_.x;
    const double dy = p.y - this->last_measurement_point_.y;
    const double dz = p.z - this->last_measurement_point_.z;
    const double measurement_delta = std::sqrt(dx * dx + dy * dy + dz * dz);

    const bool stationary_now =
      measurement_delta <= this->config_.stationary_measurement_threshold &&
      innovation <= this->config_.stationary_innovation_threshold &&
      speed <= this->config_.stationary_speed_threshold &&
      yaw_rate <= this->config_.stationary_yaw_rate_threshold;
    this->stationary_frame_count_ = stationary_now ? this->stationary_frame_count_ + 1 : 0;
  } else {
    this->stationary_frame_count_ = 0;
    this->has_last_measurement_point_ = true;
  }

  this->last_measurement_point_ = p;
  this->has_last_measurement_point_ = true;
  this->stationary_confidence = std::min(
    1.0,
    static_cast<double>(this->stationary_frame_count_) /
      std::max(this->config_.stationary_required_frames, 1));
}

void Tracker::resetStationaryState()
{
  this->stationary_frame_count_ = 0;
  this->stationary_confidence = 0.0;
  this->has_last_measurement_point_ = false;
}

FaceBinding resolveMatchedFace(
  const Eigen::VectorXd & state,
  ArmorsNum armors_num,
  double other_radius,
  double dz,
  const auto_aim_interfaces::msg::Armor & armor,
  double visible_direct_obliquity_threshold_deg)
{
  FaceBinding best;
  const int faces = std::max<int>(static_cast<int>(armors_num), 1);
  const double spacing = 2.0 * M_PI / faces;

  tf2::Quaternion tf_q;
  tf2::fromMsg(armor.pose.orientation, tf_q);
  double roll, pitch, raw_yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, raw_yaw);
  const double measured_yaw = state(6) + angles::shortest_angular_distance(state(6), raw_yaw);

  double best_score = std::numeric_limits<double>::infinity();

  for (int i = 0; i < faces; ++i) {
    FaceBinding candidate;
    const bool alternate_pair = (faces == 4) && (i % 2 == 1);
    const double radius = alternate_pair ? other_radius : state(8);
    const double dz_offset = alternate_pair ? dz : 0.0;
    const double face_yaw = state(6) + i * spacing;

    candidate.valid = true;
    candidate.face_index = i;
    candidate.alternate_pair = alternate_pair;
    candidate.radius = radius;
    candidate.dz_offset = dz_offset;
    candidate.position.x = state(0) - radius * std::cos(face_yaw);
    candidate.position.y = state(2) - radius * std::sin(face_yaw);
    candidate.position.z = state(4) + dz_offset;
    candidate.yaw = face_yaw;

    const double face_to_camera = std::atan2(-candidate.position.y, -candidate.position.x);
    const double face_normal = face_yaw + M_PI;
    candidate.obliquity =
      std::abs(angles::shortest_angular_distance(face_normal, face_to_camera));
    candidate.visibility = std::clamp(std::cos(candidate.obliquity), 0.0, 1.0);
    candidate.observable =
      candidate.obliquity <= visible_direct_obliquity_threshold_deg * M_PI / 180.0;
    candidate.fresh = true;

    const double dx = armor.pose.position.x - candidate.position.x;
    const double dy = armor.pose.position.y - candidate.position.y;
    const double dz_err = armor.pose.position.z - candidate.position.z;
    const double pos_err = std::sqrt(dx * dx + dy * dy + dz_err * dz_err);
    const double yaw_err =
      std::abs(angles::shortest_angular_distance(candidate.yaw, measured_yaw));
    const double score = pos_err + 0.10 * yaw_err;

    if (score < best_score) {
      best_score = score;
      best = candidate;
    }
  }

  best.position = armor.pose.position;
  best.yaw = measured_yaw;
  return best;
}

}  // namespace rm_auto_aim
