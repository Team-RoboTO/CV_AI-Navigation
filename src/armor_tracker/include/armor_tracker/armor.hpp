#ifndef ARMOR_TRACKER__ARMOR_HPP_
#define ARMOR_TRACKER__ARMOR_HPP_

#include <opencv2/core.hpp>

#include <string>

namespace rm_auto_aim
{

enum class ArmorType { SMALL = 0, LARGE = 1 };

const std::string ARMOR_TYPE_STR[] = {"small", "large"};

struct Light
{
  cv::Point2f top;
  cv::Point2f bottom;
  cv::Point2f center;
};

struct Armor
{
  Light left_light;
  Light right_light;
  ArmorType type;
  cv::Point2f center;
  std::string number;
  std::string classfication_result;
  float confidence;
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
