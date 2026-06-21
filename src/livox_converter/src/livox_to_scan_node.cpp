#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

class LivoxToScanNode : public rclcpp::Node
{
public:
  LivoxToScanNode() : Node("livox_to_scan")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_scan");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    publish_every_n_ = std::max<int>(1, static_cast<int>(declare_parameter<int>("publish_every_n", 1)));
    keep_every_n_ = std::max<int>(1, static_cast<int>(declare_parameter<int>("keep_every_n", 4)));

    angle_min_ = declare_parameter<double>("angle_min", -3.141592653589793);
    angle_max_ = declare_parameter<double>("angle_max", 3.141592653589793);
    angle_increment_ = declare_parameter<double>("angle_increment", 0.020);  // ~1.15 deg, lighter than 0.015
    scan_time_ = declare_parameter<double>("scan_time", 0.20);
    time_increment_ = declare_parameter<double>("time_increment", 0.0);

    range_min_ = declare_parameter<double>("range_min", 0.20);
    range_max_ = declare_parameter<double>("range_max", 4.0);
    z_min_ = declare_parameter<double>("z_min", -0.55);
    z_max_ = declare_parameter<double>("z_max", -0.15);
    drop_zero_points_ = declare_parameter<bool>("drop_zero_points", true);
    valid_tag_ = declare_parameter<int>("valid_tag", -1);  // -1 disabled
    reflectivity_min_ = declare_parameter<double>("reflectivity_min", 0.0);

    max_raw_points_ = declare_parameter<int>("max_raw_points", 0);        // 0 = no hard cap
    max_used_points_ = declare_parameter<int>("max_used_points", 0);      // 0 = no hard cap
    log_stats_period_sec_ = declare_parameter<double>("log_stats_period_sec", 5.0);

    const int qos_depth = std::max<int>(1, static_cast<int>(declare_parameter<int>("qos_depth", 1)));
    const std::string input_reliability = declare_parameter<std::string>("input_reliability", "best_effort");
    const std::string output_reliability = declare_parameter<std::string>("output_reliability", "reliable");

    if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
      throw std::runtime_error("Invalid scan angles");
    }

    bin_count_ = static_cast<int>(std::ceil((angle_max_ - angle_min_) / angle_increment_));
    if (bin_count_ <= 0 || bin_count_ > 4000) {
      throw std::runtime_error("Invalid bin_count; check angle_increment");
    }

    ranges_.resize(static_cast<size_t>(bin_count_));
    intensities_.resize(static_cast<size_t>(bin_count_));

    rclcpp::QoS input_qos{rclcpp::KeepLast(qos_depth)};
    input_qos.durability_volatile();
    if (input_reliability == "reliable") {
      input_qos.reliable();
    } else {
      input_qos.best_effort();
    }

    rclcpp::QoS output_qos{rclcpp::KeepLast(qos_depth)};
    output_qos.durability_volatile();
    if (output_reliability == "best_effort") {
      output_qos.best_effort();
    } else {
      output_qos.reliable();
    }

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, output_qos);
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, input_qos,
      std::bind(&LivoxToScanNode::onMsg, this, std::placeholders::_1));

    last_stats_wall_ = now();

    RCLCPP_INFO(
      get_logger(),
      "C++ Livox CustomMsg -> LaserScan ready: %s -> %s; rate=%.2fHz, keep_every_n=%d, bins=%d, range=%.2f..%.2fm, z=%.2f..%.2fm, output_qos=%s",
      input_topic_.c_str(), output_topic_.c_str(), publish_rate_hz_, keep_every_n_, bin_count_,
      range_min_, range_max_, z_min_, z_max_, output_reliability.c_str());
  }

private:
  void onMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (publisher_->get_subscription_count() == 0) {
      return;
    }

    ++cb_count_;
    if (publish_every_n_ > 1 && (cb_count_ % static_cast<uint64_t>(publish_every_n_)) != 0) {
      ++dropped_msgs_;
      return;
    }

    const auto now_wall = std::chrono::steady_clock::now();
    if (publish_rate_hz_ > 0.0) {
      if (now_wall < next_publish_wall_) {
        ++dropped_msgs_;
        return;
      }
      next_publish_wall_ = now_wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    }

    const float inf = std::numeric_limits<float>::infinity();
    std::fill(ranges_.begin(), ranges_.end(), inf);
    std::fill(intensities_.begin(), intensities_.end(), 0.0f);

    const double rmin2 = range_min_ * range_min_;
    const double rmax2 = range_max_ * range_max_;
    int used = 0;
    int raw_seen = 0;

    const auto & pts = msg->points;
    in_points_ += static_cast<uint64_t>(pts.size());

    for (size_t i = 0; i < pts.size(); i += static_cast<size_t>(keep_every_n_)) {
      if (max_raw_points_ > 0 && raw_seen >= max_raw_points_) {
        break;
      }
      ++raw_seen;

      const auto & p = pts[i];
      const double x = static_cast<double>(p.x);
      const double y = static_cast<double>(p.y);
      const double z = static_cast<double>(p.z);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (drop_zero_points_ && std::abs(x) < 1e-6 && std::abs(y) < 1e-6 && std::abs(z) < 1e-6) {
        continue;
      }
      if (z < z_min_ || z > z_max_) {
        continue;
      }
      if (valid_tag_ >= 0 && static_cast<int>(p.tag) != valid_tag_) {
        continue;
      }
      const double refl = static_cast<double>(p.reflectivity);
      if (refl < reflectivity_min_) {
        continue;
      }

      const double r2 = x * x + y * y;
      if (r2 < rmin2 || r2 > rmax2) {
        continue;
      }

      const double angle = std::atan2(y, x);
      if (angle < angle_min_ || angle >= angle_max_) {
        continue;
      }
      const int index = static_cast<int>((angle - angle_min_) / angle_increment_);
      if (index < 0 || index >= bin_count_) {
        continue;
      }

      const float r = static_cast<float>(std::sqrt(r2));
      if (r < ranges_[static_cast<size_t>(index)]) {
        ranges_[static_cast<size_t>(index)] = r;
        intensities_[static_cast<size_t>(index)] = static_cast<float>(refl);
      }

      ++used;
      if (max_used_points_ > 0 && used >= max_used_points_) {
        break;
      }
    }

    if (used == 0) {
      ++empty_scans_;
      maybeLogStats();
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    if (!frame_id_.empty()) {
      scan.header.frame_id = frame_id_;
    }
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_min_ + bin_count_ * angle_increment_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = static_cast<float>(time_increment_);
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = ranges_;
    scan.intensities = intensities_;

    publisher_->publish(scan);
    used_points_ += static_cast<uint64_t>(used);
    ++published_;
    maybeLogStats();
  }

  void maybeLogStats()
  {
    if (log_stats_period_sec_ <= 0.0) {
      return;
    }
    const auto t = now();
    if ((t - last_stats_wall_).seconds() < log_stats_period_sec_) {
      return;
    }
    last_stats_wall_ = t;
    const double avg_used = published_ > 0 ? static_cast<double>(used_points_) / static_cast<double>(published_) : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "stats: published=%lu dropped_msgs=%lu empty=%lu in_points=%lu used_points=%lu avg_used=%.0f",
      published_, dropped_msgs_, empty_scans_, in_points_, used_points_, avg_used);
    published_ = 0;
    dropped_msgs_ = 0;
    empty_scans_ = 0;
    in_points_ = 0;
    used_points_ = 0;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  double publish_rate_hz_;
  int publish_every_n_;
  int keep_every_n_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double time_increment_;
  double range_min_;
  double range_max_;
  double z_min_;
  double z_max_;
  bool drop_zero_points_;
  int valid_tag_;
  double reflectivity_min_;
  int max_raw_points_;
  int max_used_points_;
  double log_stats_period_sec_;
  int bin_count_;

  std::vector<float> ranges_;
  std::vector<float> intensities_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;

  uint64_t cb_count_{0};
  uint64_t published_{0};
  uint64_t dropped_msgs_{0};
  uint64_t empty_scans_{0};
  uint64_t in_points_{0};
  uint64_t used_points_{0};
  rclcpp::Time last_stats_wall_;
  std::chrono::steady_clock::time_point next_publish_wall_{std::chrono::steady_clock::time_point::min()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxToScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
