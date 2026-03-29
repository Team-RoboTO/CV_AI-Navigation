#ifndef ARMOR_TRACKER__ARMOR_HPP_
#define ARMOR_TRACKER__ARMOR_HPP_

#include <opencv2/core.hpp>

#include <string>

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// ArmorType — the two physical armor plate sizes used in RoboMaster.
//
// SMALL (AM02): 140 mm wide × 125 mm tall — commonly used on standard robots.
// LARGE (AM12): 235 mm wide × 127 mm tall — commonly used on hero robots.
//
// IMPORTANT DESIGN NOTE:
//   We no longer trust the raw YOLO bounding-box aspect ratio alone to decide
//   the armor type.  An axis-aligned bounding box gets narrower when the plate
//   is viewed obliquely, so a real LARGE armor can easily look "small" in 2D.
//
//   The current pipeline therefore does this instead:
//     1. Build the same 2D corner approximation for the detection.
//     2. Solve PnP with the SMALL 3D model.
//     3. Solve PnP again with the LARGE 3D model.
//     4. Reproject both solutions back into the image.
//     5. Keep the model with the lower reprojection error.
//
//   Reprojection error means:
//     "If this 3D hypothesis were correct, how close would its projected image
//      points land to the image points we actually observed?"
//
//   Lower reprojection error = better geometric agreement.
// ---------------------------------------------------------------------------
enum class ArmorType { SMALL = 0, LARGE = 1 };

const std::string ARMOR_TYPE_STR[] = {"small", "large"};

// ---------------------------------------------------------------------------
// Light — one vertical light bar of an armor plate.
//
// Each armor plate has two light bars (left and right).  The PnP solver uses
// the top and bottom endpoints of both bars as its four 2D image points.
//
// Image coordinate convention (OpenCV):
//   x → right
//   y → down
// ---------------------------------------------------------------------------
struct Light
{
  cv::Point2f top;     // Top endpoint of the light bar in pixels
  cv::Point2f bottom;  // Bottom endpoint of the light bar in pixels
  cv::Point2f center;  // Midpoint of the light bar (debug / visualization)
};

// ---------------------------------------------------------------------------
// Armor — one detected armor plate before or after PnP.
//
// Upstream detector:
//   A YOLO detector gives an axis-aligned bounding box.
//
// Tracker-side geometric approximation:
//   We shrink that bounding box inward with light_ratio_ to approximate where
//   the light bars most likely are.
//
// PnP stage:
//   The four approximated bar endpoints are matched against the known 3D armor
//   geometry to recover the 3D pose.
//
// The 'number' field currently carries the YOLO class string used for target
// color / class filtering in the tracker.
// ---------------------------------------------------------------------------
struct Armor
{
  Light left_light;                  // Left light-bar endpoints in image space
  Light right_light;                 // Right light-bar endpoints in image space
  ArmorType type = ArmorType::SMALL; // Physical type chosen by the PnP stage
  cv::Point2f center;                // Detection center in image space
  std::string number;                // YOLO class string (used for filtering)
  std::string classfication_result;  // Legacy debug field kept for compatibility
  float confidence = 0.0F;           // Detector confidence in [0, 1]
};

}  // namespace rm_auto_aim

// Also expose in the armor_detector namespace so that any legacy include path
// expecting armor_detector::Armor resolves to the same concrete type.
namespace armor_detector
{
using rm_auto_aim::ArmorType;
using rm_auto_aim::ARMOR_TYPE_STR;
using rm_auto_aim::Light;
using rm_auto_aim::Armor;
}  // namespace armor_detector

#endif  // ARMOR_TRACKER__ARMOR_HPP_
