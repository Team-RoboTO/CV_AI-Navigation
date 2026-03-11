#ifndef ARMOR_TRACKER__ARMOR_HPP_
#define ARMOR_TRACKER__ARMOR_HPP_

#include <opencv2/core.hpp>

#include <string>

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// ArmorType — the two physical armor plate sizes used in RoboMaster.
//
// SMALL (AM02): 140 mm wide × 125 mm tall — mounted on standard robots.
// LARGE (AM12): 235 mm wide × 127 mm tall — mounted on hero robots.
//
// The PnP solver uses different 3D point models for each type (see pnp_solver.cpp),
// so the correct type must be identified before solvePnP() is called.
// Type is inferred from the YOLO bounding-box aspect ratio:
//   width/height > 1.5  →  LARGE   (aspect ≈ 1.85)
//   width/height ≤ 1.5  →  SMALL   (aspect ≈ 1.12)
// ---------------------------------------------------------------------------
enum class ArmorType { SMALL = 0, LARGE = 1 };

const std::string ARMOR_TYPE_STR[] = {"small", "large"};

// ---------------------------------------------------------------------------
// Light — one vertical light bar of an armor plate.
//
// Each armor plate has two light bars (left and right) that glow under the
// LED illumination.  The PnP solver matches the four corners of these two
// bars (top and bottom of each bar) against the known 3D plate geometry to
// recover the 3D pose.
//
// All coordinates are in the image (pixel) frame:
//   x → right,  y → down  (OpenCV convention)
// ---------------------------------------------------------------------------
struct Light
{
  cv::Point2f top;     // Top endpoint of the light bar (pixel coords)
  cv::Point2f bottom;  // Bottom endpoint of the light bar (pixel coords)
  cv::Point2f center;  // Midpoint of the light bar (for visualisation only)
};

// ---------------------------------------------------------------------------
// Armor — a single detected armor plate, populated before PnP solving.
//
// Flow:
//   YOLO bounding box
//     → inward-scaled rect (light_ratio parameter)
//     → Armor.left_light / right_light corners set from rect edges
//     → PnP solver → rvec + tvec → 3D pose in camera frame
//
// The 'number' field carries the YOLO class ID string (e.g. "3" = red).
// The class ID encodes the team color (0=blue, 2=purple, 3=red) but NOT
// the robot type (hero, standard, etc.), which is not available from YOLO.
// ---------------------------------------------------------------------------
struct Armor
{
  Light left_light;              // Left light-bar corners (pixel frame)
  Light right_light;             // Right light-bar corners (pixel frame)
  ArmorType type;                // SMALL or LARGE (from bbox aspect ratio)
  cv::Point2f center;            // Bbox center (pixel frame, for distance-to-center)
  std::string number;            // YOLO class ID string (team color code)
  std::string classfication_result;  // Same as number; kept for debug display
  float confidence;              // YOLO detection confidence score [0, 1]
};

}  // namespace rm_auto_aim

// Also expose in the armor_detector namespace so that pnp_solver.hpp
// (which includes "armor_detector/armor.hpp") resolves the same types.
namespace armor_detector
{
using rm_auto_aim::ArmorType;
using rm_auto_aim::ARMOR_TYPE_STR;
using rm_auto_aim::Light;
using rm_auto_aim::Armor;
}  // namespace armor_detector

#endif  // ARMOR_TRACKER__ARMOR_HPP_
