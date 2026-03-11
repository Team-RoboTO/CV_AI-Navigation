#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>
#include <vector>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance, double max_track_range)
: tracker_state(LOST),
  tracked_id(std::string("")),
  info_position_diff(0.0),
  info_yaw_diff(0.0),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  // WHY π/3 (60°): 4-armor robots have faces 90° apart.  The yaw gate must be
  // wide enough to accept noisy PnP measurements (±15° at 3m) but narrow enough
  // to detect a 90° jump (face switch).  60° is the midpoint: a measurement
  // within ±60° of predicted yaw = same face; beyond 60° = different face.
  max_match_yaw_diff_(M_PI / 3.0),
  max_track_range_(max_track_range),
  // WHY 65°: beyond ~65° obliquity, the armor plate is < 2 light-bar widths of
  // pixels in the image, and PnP yaw degenerates to noise.  Empirically tuned.
  max_yaw_oblique_angle_(65.0 * M_PI / 180.0)
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
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  updateArmorsNum(tracked_armor);
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
  if (!predicted_) {
    ekf_prediction = ekf.predict();
    r_active_kf_.predict();
    r_other_kf_.predict();
    // Write scalar KF radius back into EKF state vector.
    // WHY DECOUPLE: radius is managed by its own 1D Kalman filter (more stable
    // for a quasi-static parameter than letting the 9D EKF handle it).  But the
    // EKF still has radius in its state vector (for the observation model h(x)).
    // decoupleState zeroes the cross-covariance P(8,j) and P(j,8) for j≠8, and
    // sets P(8,8) = r_kf.P.  This prevents the radius from influencing other
    // states' Kalman gains (and vice versa), effectively making it read-only
    // from the EKF's perspective.
    ekf_prediction(8) = r_active_kf_.x;
    ekf.setState(ekf_prediction);
    ekf.decoupleState(8, r_active_kf_.P);
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
    prior_vx = ekf_prediction(1);  // v_xc predicted
    prior_vy = ekf_prediction(3);  // v_yc predicted
    prior_vz = ekf_prediction(5);  // v_za predicted
  } else {
    // predictStep() already ran — target_state holds the prediction.
    // prior_vx/vy/vz were already set in predictStep().
    ekf_prediction = target_state;
  }
  predicted_ = false;
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

  bool matched = false;
  measurement_valid = false;
  // Default: if we don't find a match, the EKF just coasts on its prediction.
  target_state = ekf_prediction;

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
    auto predicted_position = getArmorPositionFromState(ekf_prediction);

    for (const auto & armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
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
          double maha = ekf.mahalanobis(z);
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
        if (tracked_armors_num == ArmorsNum::NORMAL_4) {
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
      double measured_yaw = orientationToYaw(selected_armor.pose.orientation);
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
      bool yaw_oblique = face_angle > max_yaw_oblique_angle_;

      info_position_diff = position_diff;
      info_yaw_diff = yaw_diff;
      info_yaw_innov_signed = yaw_innov_signed;
      info_position_innov = Eigen::Vector3d(p.x, p.y, p.z) - predicted_position;
      info_face_angle = face_angle;
      tracked_armor = selected_armor;

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
      if (position_diff < max_match_distance_ &&
          (yaw_diff < max_match_yaw_diff_ || yaw_oblique)) {
        // Matched armor found — verify with Mahalanobis before fusing.
        // WHY A SECOND GATE (Mahalanobis) AFTER POSITION+YAW?
        // The position/yaw gates are simple axis-aligned checks.  Mahalanobis
        // is a *statistical* gate that accounts for the EKF's covariance shape.
        // A detection might pass the loose position gate but be statistically
        // improbable given the filter's current uncertainty — e.g. if P is tiny
        // in one axis, even a small offset is multiple-sigma unlikely.
        // The Mahalanobis gate catches these cases and prevents filter corruption.
        if (yaw_oblique && yaw_diff >= max_match_yaw_diff_) {
          RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"),
            "Oblique view (%.0f°) — bypassing yaw gate (yaw_diff=%.1f°), R_yaw inflated",
            face_angle * 180.0 / M_PI, yaw_diff * 180.0 / M_PI);
        }
        measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
        double maha = ekf.mahalanobis(measurement);
        // WHY 13.3: Mahalanobis² follows a chi-squared distribution with k DOF
        // (k=4 for our [x,y,z,yaw] measurement).  The 99% quantile of χ²(4) is
        // ~13.3.  This means: if the detection is truly from our tracked target,
        // there's only a 1% chance of exceeding 13.3.  Values above this almost
        // certainly indicate a wrong detection or severe filter divergence.
        if (maha < maha_match_threshold_) {
          matched = true;
          measurement_valid = true;
          target_state = ekf.update(measurement);

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
          //   - Skip at long range (> r_adapt_max_dist_): PnP noise grows
          //     and the radius measurement becomes too noisy to be useful.
          //   - Clamp r_measured to [0.10, 0.45]: physically plausible bounds.
          //   - Inflate R of the scalar KF by yaw uncertainty: if the EKF
          //     isn't sure about yaw, the radius measurement is less reliable.
          {
            double yaw = target_state(6);
            // Check obliquity using posterior yaw (more accurate than prior)
            double bearing_opp_r = std::atan2(-measurement(1), -measurement(0));
            double face_angle_r = std::abs(std::remainder(yaw - bearing_opp_r, 2.0 * M_PI));
            if (face_angle_r < max_yaw_oblique_angle_) {
              // Project center-to-armor vector onto yaw direction → radial distance
              double dx = target_state(0) - measurement(0);  // xc - xa
              double dy = target_state(2) - measurement(1);  // yc - ya
              double r_measured = dx * std::cos(yaw) + dy * std::sin(yaw);
              double range = std::sqrt(
                target_state(0) * target_state(0) +
                target_state(2) * target_state(2) +
                target_state(4) * target_state(4));
              if (range < r_adapt_max_dist_ &&
                  r_measured > 0.10 && r_measured < 0.45) {
                // Inflate the scalar KF's R by yaw uncertainty: uncertain yaw
                // means the projection direction is wrong, so trust r_measured less
                double yaw_var = ekf.getVariance(6);
                double yaw_factor = 1.0 + r_yaw_uncertainty_scale_ * yaw_var;
                double saved_R = r_active_kf_.R;
                r_active_kf_.R = saved_R * yaw_factor;  // temporarily inflate R
                r_active_kf_.update(r_measured);
                r_active_kf_.R = saved_R;                // restore original R
              }
            }
            // Write back the KF's current radius estimate into the EKF state
            target_state(8) = r_active_kf_.x;
          }

          // Other pair: deferred multi-armor measurement using posterior center.
          // Skip when other-pair face is edge-on (oblique > threshold).
          if (has_other_pair_armor) {
            double bearing_opp_o = std::atan2(-other_pair_y, -other_pair_x);
            double face_angle_o = std::abs(std::remainder(other_pair_yaw - bearing_opp_o, 2.0 * M_PI));
            if (face_angle_o < max_yaw_oblique_angle_) {
              double odx = target_state(0) - other_pair_x;
              double ody = target_state(2) - other_pair_y;
              double r_other_measured =
                odx * std::cos(other_pair_yaw) + ody * std::sin(other_pair_yaw);
              if (r_other_measured > 0.10 && r_other_measured < 0.45) {
                double yaw_var = ekf.getVariance(6);
                double yaw_factor = 1.0 + r_yaw_uncertainty_scale_ * yaw_var;
                double saved_R = r_other_kf_.R;
                r_other_kf_.R = saved_R * yaw_factor;
                r_other_kf_.update(r_other_measured);
                r_other_kf_.R = saved_R;
              }
            }
            another_r = r_other_kf_.x;

            // dz from same-frame observation (more accurate than jump-based:
            // both measurements are simultaneous, no temporal drift).
            double dz_measured = other_pair_z - target_state(4);
            dz_measured = std::max(-0.25, std::min(dz_measured, 0.25));
            if (!dz_initialized_) {
              dz = dz_measured;
              dz_initialized_ = true;
            } else {
              dz = dz_adapt_alpha_ * dz_measured + (1.0 - dz_adapt_alpha_) * dz;
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
          if (has_other_pair_armor && use_secondary_face_fusion) {
            double sel_yaw_val = measurement(3);
            double yaw_offset = angles::shortest_angular_distance(sel_yaw_val, other_pair_yaw);
            // Round to nearest ±π/2 (guard against PnP noise in yaw)
            yaw_offset = std::round(yaw_offset / (M_PI / 2)) * (M_PI / 2);

            // Current EKF yaw (from posterior after primary update)
            double current_yaw = target_state(6);
            double unwrapped_other_yaw = current_yaw + yaw_offset;
            double r_other = r_other_kf_.x;

            // Secondary measurement vector
            Eigen::Vector4d z_2(other_pair_x, other_pair_y, other_pair_z, unwrapped_other_yaw);

            // Predicted measurement from current state: h_2(x)
            Eigen::Vector4d z_pred_2;
            z_pred_2(0) = target_state(0) - r_other * std::cos(current_yaw + yaw_offset);
            z_pred_2(1) = target_state(2) - r_other * std::sin(current_yaw + yaw_offset);
            z_pred_2(2) = target_state(4) + dz;
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
            double ps_2 = r_xyz_base_ + r_xyz_slope_ * dist_2;
            double ys_2 = r_yaw_base_ + r_yaw_slope_ * dist_2;
            double xyz_af_2 = 1.0, yaw_af_2 = 1.0;
            if (other_pair_x * other_pair_x + other_pair_y * other_pair_y > 1e-12) {
              double bearing_opp_2 = std::atan2(-other_pair_y, -other_pair_x);
              double face_angle_2 = std::abs(
                std::remainder(other_pair_yaw - bearing_opp_2, 2.0 * M_PI));
              double cos_fa_2 = std::cos(face_angle_2);
              xyz_af_2 = 1.0 / std::max(cos_fa_2 * cos_fa_2, 0.04);
              double cos_pow_2 = std::pow(std::abs(cos_fa_2), r_yaw_angle_power_);
              yaw_af_2 = 1.0 / std::max(cos_pow_2, 1e-4);
              double max_oblique_rad = max_yaw_oblique_deg_ * M_PI / 180.0;
              if (face_angle_2 > max_oblique_rad) {
                yaw_af_2 = 1e6;
              }
            }
            Eigen::Matrix4d R_2 = Eigen::Matrix4d::Zero();
            R_2(0, 0) = ps_2 * ps_2 * xyz_af_2 * secondary_r_inflation;
            R_2(1, 1) = ps_2 * ps_2 * xyz_af_2 * secondary_r_inflation;
            R_2(2, 2) = ps_2 * ps_2 * xyz_af_2 * secondary_r_inflation;
            R_2(3, 3) = ys_2 * ys_2 * yaw_af_2 * secondary_r_inflation;

            // Mahalanobis gate on the secondary update
            Eigen::Vector4d innov_2 = z_2 - z_pred_2;
            Eigen::MatrixXd S_2 = H_2 * ekf.P_post_view() * H_2.transpose() + R_2;
            Eigen::LDLT<Eigen::MatrixXd> S_2_ldlt(S_2);
            double maha_2 = 1e9;
            if (S_2_ldlt.info() == Eigen::Success && S_2_ldlt.isPositive()) {
              maha_2 = innov_2.transpose() * S_2_ldlt.solve(innov_2);
            }

            if (maha_2 < secondary_maha_threshold) {
              // Write current target_state into EKF before the secondary update
              ekf.setState(target_state);
              ekf.decoupleState(8, r_active_kf_.P);
              target_state = ekf.updateWithModel(z_2, z_pred_2, H_2, R_2);
              RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"),
                "Secondary face fusion (maha=%.1f)", maha_2);
            }
          }

          ekf.setState(target_state);
          // Sync EKF covariance for radius with the scalar KF: zero cross-terms
          // so the externally-managed r doesn't pollute Kalman gains or Mahalanobis.
          ekf.decoupleState(8, r_active_kf_.P);
          RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update (maha=%.1f)", maha);
        } else {
          if (maha >= 1e8) {
            RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
              "EKF: singular innovation covariance (maha=%.0f) — possible divergence", maha);
          } else {
            RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
              "Measurement rejected: Mahalanobis=%.1f > %.1f", maha, maha_match_threshold_);
          }
        }
      } else if (yaw_diff > max_match_yaw_diff_ && !yaw_oblique) {
        // Yaw jumped (and not oblique) — gate with relaxed Mahalanobis before accepting
        measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
        double maha = ekf.mahalanobis(measurement);
        if (maha < maha_jump_threshold_) {  // relaxed gate for jumps
          handleArmorJump(selected_armor);
          matched = true;
          measurement_valid = true;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
            "Armor jump rejected: Mahalanobis=%.1f > %.1f", maha, maha_jump_threshold_);
        }
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
  if (target_state(8) < 0.12 || target_state(8) > 0.4) {
    target_state(8) = std::max(0.12, std::min(target_state(8), 0.4));
    r_active_kf_.x = target_state(8);
    r_active_kf_.P = r_kf_P_init_;  // reset KF uncertainty after clamp
    ekf.setState(target_state);
    ekf.decoupleState(8, r_active_kf_.P);
  }

  // Clamp v_yaw to physically plausible range.
  // RoboMaster robots cannot spin faster than ~v_yaw_max_ rad/s;
  // values beyond this indicate EKF divergence from a bad detection.
  if (std::abs(target_state(7)) > v_yaw_max_) {
    target_state(7) = std::copysign(v_yaw_max_, target_state(7));
    ekf.setState(target_state);
    ekf.resetCovariance();
    // resetCovariance restores P0 which has cross-terms = 0 for r,
    // but re-decouple to ensure P(8,8) matches the scalar KF.
    ekf.decoupleState(8, r_active_kf_.P);
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
  if (tracker_state == DETECTING) {
    if (matched) {
      // Accumulate consecutive matches; promote only after tracking_thres frames
      // to avoid locking onto a single false-positive detection.
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      // Any miss in DETECTING resets the counter — the candidate was unreliable.
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      // First missed frame: don't drop immediately, give the EKF a chance to
      // coast through a brief occlusion or detection dropout.
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
    // If matched: stay in TRACKING, nothing to do.
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      // Still no detection — increment miss counter and wait up to lost_thres
      // frames before giving up and resetting to LOST.
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      // Detection recovered — snap back to TRACKING immediately without
      // requiring re-confirmation (detect_count_ threshold).
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }

  // Drop target if it is beyond max tracking range
  if (tracker_state != LOST) {
    double xc = target_state(0), yc = target_state(2), za = target_state(4);
    double range = std::sqrt(xc * xc + yc * yc + za * za);
    if (range > max_track_range_) {
      tracker_state = LOST;
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
        "Target beyond max range (%.1fm > %.1fm) — LOST", range, max_track_range_);
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
  double wrapped_yaw = angles::normalize_angle(target_state(6));
  if (wrapped_yaw != target_state(6)) {
    target_state(6) = wrapped_yaw;
    last_yaw_ = wrapped_yaw;  // keep orientationToYaw unwrapping in sync
    ekf.setState(target_state);
  }
}

void Tracker::initFromArmor(const Armor & armor)
{
  initEKF(armor);
  tracked_id = armor.number;
  tracker_state = DETECTING;
  updateArmorsNum(armor);
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
  Eigen::VectorXd ekf_prediction = ekf.predict();
  r_active_kf_.predict();
  r_other_kf_.predict();
  ekf_prediction(8) = r_active_kf_.x;
  ekf.setState(ekf_prediction);
  ekf.decoupleState(8, r_active_kf_.P);
  // Store prior velocities for innovation-based acceleration estimation.
  // Same rationale as in update()'s internal predict path above: capture the
  // EKF's velocity prediction BEFORE the measurement update, so the tracker
  // node can compute (v_posterior - v_prior) / dt as the acceleration signal.
  // predictStep() is the primary path (called by the multi-tracker data
  // association loop), so this is where prior_v* is set most of the time.
  prior_vx = ekf_prediction(1);  // v_xc predicted
  prior_vy = ekf_prediction(3);  // v_yc predicted
  prior_vz = ekf_prediction(5);  // v_za predicted
  target_state = ekf_prediction;
  predicted_ = true;
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
  double predicted_yaw = target_state(6);
  double unwrapped_yaw = predicted_yaw +
    angles::shortest_angular_distance(predicted_yaw, raw_yaw);
  Eigen::Vector4d z(p.x, p.y, p.z, unwrapped_yaw);
  return ekf.mahalanobis(z);
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
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Back-calculate robot center from armor position:
  //   armor = center − r·[cos,sin]  ⟹  center = armor + r·[cos,sin]
  target_state = Eigen::VectorXd::Zero(9);
  double r = initial_r1_;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);

  // Reset other-pair geometry estimates
  dz = 0;
  another_r = initial_r2_;
  dz_initialized_ = false;

  // Reset scalar KFs with initial radii and configured noise parameters.
  // High P_init (σ ≈ 8cm) ensures the gain is large for the first few frames,
  // allowing the radius to converge quickly from the initial guess.
  r_active_kf_.reset(initial_r1_, r_kf_P_init_);
  r_active_kf_.Q = r_kf_Q_;
  r_active_kf_.R = r_kf_R_;
  r_other_kf_.reset(initial_r2_, r_kf_P_init_);
  r_other_kf_.Q = r_kf_Q_;
  r_other_kf_.R = r_kf_R_;

  //                xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
  target_state << xc, 0,    yc, 0,    za, 0,    yaw, 0,     r;

  ekf.setState(target_state);
}

// Color-based YOLO class IDs (0=blue,1=grey,2=purple,3=red) cannot distinguish
// robot types (hero, sentry, balance, outpost). Always assume NORMAL_4 so
// yaw-jump thresholds and trajectory face-spacing are correct for standard robots.
// BALANCE_2 / OUTPOST_3 enums are kept for future use when robot-type detection
// is available (e.g. via a separate classifier or communication from the lower computer).
void Tracker::updateArmorsNum(const Armor & armor)
{
  (void)armor;  // unused until robot-type detection is available
  tracked_armors_num = ArmorsNum::NORMAL_4;
  max_match_yaw_diff_ = M_PI / 3.0;  // ~60°
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
  double yaw = orientationToYaw(current_armor.pose.orientation);
  // Is the robot spinning the same direction as before, or did it reverse?
  // jump_direction > 0 means the new face is CCW from the old face.
  // If that disagrees with v_yaw's sign, the robot changed spin direction.
  double jump_direction = angles::shortest_angular_distance(target_state(6), yaw);
  double jump_angle = std::abs(jump_direction);
  if (jump_direction * target_state(7) < 0) {
    // Spin reversal detected — zero v_yaw to prevent EKF from extrapolating in wrong direction
    target_state(7) = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Spin reversal — v_yaw zeroed");
  }
  target_state(6) = yaw;
  updateArmorsNum(current_armor);
  // Only 4-armor robots have alternating radii and heights.
  // A 4-armor robot has two pairs of faces:
  //   Pair A (faces 0, 2): radius = r1, height = za
  //   Pair B (faces 1, 3): radius = r2, height = za + dz
  //
  // When the robot spins, the visible face switches:
  //   ~90° jump  → pair switch (A↔B): different radius and height
  //   ~180° jump → same pair (0↔2 or 1↔3): same radius and height
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    // Classify jump angle: [45°, 135°] = pair switch, otherwise same pair
    bool is_pair_switch = (jump_angle > M_PI / 4.0) && (jump_angle < 3.0 * M_PI / 4.0);
    if (is_pair_switch) {
      // Measure height difference between the old face (za in state) and new face
      // (current_armor.z).  EMA-smooth to reject single-frame PnP noise.
      double new_dz = target_state(4) - current_armor.pose.position.z;
      new_dz = std::max(-0.25, std::min(new_dz, 0.25));  // clamp to physical bounds
      if (!dz_initialized_) {
        dz = new_dz;              // First measurement: accept directly
        dz_initialized_ = true;
      } else {
        dz = dz_adapt_alpha_ * new_dz + (1.0 - dz_adapt_alpha_) * dz;  // EMA
      }
      // Snap za to the new face's measured height
      target_state(4) = current_armor.pose.position.z;
      target_state(5) = 0.0;  // v_za corrupted by the za snap — zero it

      // Swap the radius scalar KFs: the KF that was tracking pair A now
      // tracks pair B, and vice versa.  Each KF retains its own covariance,
      // so the swapped-in KF (which hasn't been updated while unobserved)
      // has grown P → higher gain → fast re-convergence to the new pair's
      // actual radius.
      std::swap(r_active_kf_, r_other_kf_);
      target_state(8) = r_active_kf_.x;
      another_r = r_other_kf_.x;
      another_r = std::max(0.12, std::min(another_r, 0.4));  // physical clamp
    }
  }
  info_yaw_innov_signed = 0.0;
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
  // state) with the actual detection.  If the error > max_match_distance_
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
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc (back-calculated from armor + radius)
    target_state(1) = 0;                   // vxc → 0 (no velocity info)
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc → 0
    target_state(4) = p.z;                 // za (from detection)
    target_state(5) = 0;                   // vza → 0
    target_state(7) = 0;                   // v_yaw → 0 (spin direction unknown)
    ekf.resetCovariance();                 // P → P0 (high uncertainty for fast re-convergence)
    dz = 0.0;                              // reset geometry estimates
    another_r = initial_r2_;
    dz_initialized_ = false;
    r_active_kf_.reset(initial_r1_, r_kf_P_init_);
    r_active_kf_.Q = r_kf_Q_;
    r_active_kf_.R = r_kf_R_;
    r_other_kf_.reset(initial_r2_, r_kf_P_init_);
    r_other_kf_.Q = r_kf_Q_;
    r_other_kf_.R = r_kf_R_;
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  ekf.setState(target_state);

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
  if (tracked_armors_num != ArmorsNum::BALANCE_2) {
    ekf.inflateCovariance(6, 4.0);  // yaw:   variance ×4 (~2× std-dev)
    ekf.inflateCovariance(7, 4.0);  // v_yaw: variance ×4
  }
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    ekf.inflateCovariance(4, 4.0);  // za:   variance ×4 (snapped to new face)
    ekf.inflateCovariance(5, 4.0);  // v_za: variance ×4 (zeroed, uncertain)
  }

  // ---------------------------------------------------------------------------
  // Sync prior → posterior so the subsequent ekf.update() uses the post-jump
  // state and inflated covariance as its linearisation point.  Without this,
  // update() would read the OLD x_pri/P_pri from predict() — computing
  // innovation and Kalman gain against a stale pre-jump state, which would
  // produce completely wrong corrections (the yaw innovation would be ~90°
  // instead of ~0°, overwhelming the linear approximation).
  // ---------------------------------------------------------------------------
  ekf.syncPrior();

  // Decouple radius before the EKF update so the Kalman gain for r ≈ 0
  // and the update doesn't introduce spurious cross-covariance.
  ekf.decoupleState(8, r_active_kf_.P);

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
  measurement = Eigen::Vector4d(p.x, p.y, p.z, yaw);
  target_state = ekf.update(measurement);
  // Yaw-projected radius measurement for the newly visible pair.
  // Skip when face is nearly edge-on (oblique > threshold).
  {
    double yaw_post = target_state(6);
    double bearing_opp_j = std::atan2(-p.y, -p.x);
    double face_angle_j = std::abs(std::remainder(yaw_post - bearing_opp_j, 2.0 * M_PI));
    if (face_angle_j < max_yaw_oblique_angle_) {
      double dx = target_state(0) - p.x;  // xc - xa
      double dy = target_state(2) - p.y;  // yc - ya
      double r_measured = dx * std::cos(yaw_post) + dy * std::sin(yaw_post);
      if (r_measured > 0.10 && r_measured < 0.45) {
        double yaw_var = ekf.getVariance(6);
        double yaw_factor = 1.0 + r_yaw_uncertainty_scale_ * yaw_var;
        double saved_R = r_active_kf_.R;
        r_active_kf_.R = saved_R * yaw_factor;
        r_active_kf_.update(r_measured);
        r_active_kf_.R = saved_R;
      }
    }
    target_state(8) = r_active_kf_.x;
    ekf.setState(target_state);
    ekf.decoupleState(8, r_active_kf_.P);
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
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
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

}  // namespace rm_auto_aim
