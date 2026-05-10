#ifndef AUTO_AIM__DETECTION_ADAPTER_HPP_
#define AUTO_AIM__DETECTION_ADAPTER_HPP_

#include <set>
#include <string>
#include <vector>

#include <vision_msgs/msg/detection2_d_array.hpp>

namespace auto_aim
{

// One YOLO bbox after the class filter and the trivial-size rejection.
// The coordinates are still in the image plane; PnP is the next stage.
struct DetectionCandidate
{
  double cx = 0;        // bbox center x [px]
  double cy = 0;        // bbox center y [px]
  double w  = 0;        // bbox width    [px]
  double h  = 0;        // bbox height   [px]
  std::string class_id;
  double confidence = 0;
};

// Stateless converter from vision_msgs/Detection2DArray to a vector of
// validated candidates. Encapsulates the class filter and the minimum-size
// gate that used to live inline in the detection callback.
struct DetectionAdapter
{
  // Convert one frame.
  //   target_classes - set of YOLO class IDs to keep (others ignored).
  //   min_pixel_size - reject any bbox with width or height below this.
  static std::vector<DetectionCandidate> convert(
    const vision_msgs::msg::Detection2DArray & msg,
    const std::set<std::string> & target_classes,
    double min_pixel_size = 1.0);
};

}  // namespace auto_aim
#endif
