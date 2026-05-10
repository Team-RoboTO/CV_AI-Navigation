#include "auto_aim/detection_adapter.hpp"

namespace auto_aim
{

std::vector<DetectionCandidate> DetectionAdapter::convert(
  const vision_msgs::msg::Detection2DArray & msg,
  const std::set<std::string> & target_classes,
  double min_pixel_size)
{
  std::vector<DetectionCandidate> out;
  out.reserve(msg.detections.size());

  for (const auto & det : msg.detections) {
    if (det.results.empty()) continue;
    const auto & cid = det.results[0].hypothesis.class_id;
    if (target_classes.find(cid) == target_classes.end()) continue;

    DetectionCandidate c;
    c.cx = det.bbox.center.position.x;
    c.cy = det.bbox.center.position.y;
    c.w  = det.bbox.size_x;
    c.h  = det.bbox.size_y;
    if (c.w < min_pixel_size || c.h < min_pixel_size) continue;

    c.class_id   = cid;
    c.confidence = det.results[0].hypothesis.score;
    out.push_back(std::move(c));
  }

  return out;
}

}  // namespace auto_aim
