#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/msg/segment_angle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Diagnostic only: subscribes to feedback; never publishes actuator commands.
class PipelineRateProbe : public rclcpp::Node
{
public:
  PipelineRateProbe() : Node("pipeline_rate_probe")
  {
    const auto interval = declare_parameter<double>("report_interval_sec", 5.0);
    if (interval <= 0.0) {throw std::invalid_argument("report_interval_sec must be positive");}
    const bool visualization = declare_parameter<bool>("visualization", true);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    const std::vector<std::pair<std::string, std::string>> image_topics = {
      {"color", "/camera/camera/color/image_rect_raw"},
      {"aligned_depth", "/camera/camera/aligned_depth_to_color/image_raw"},
      {"crop", "/estimated_segment_crop_image"},
      {"skeleton", "/estimated_segment_skeleton_image"},
    };
    for (const auto & entry : image_topics) {
      if (!visualization && (entry.first == "crop" || entry.first == "skeleton")) {continue;}
      stats_[entry.first];
      subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
        entry.second, qos,
        [this, name = entry.first](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          record(name, msg->header.stamp);
        }));
    }
    stats_["angles"];
    subscriptions_.push_back(create_subscription<custom_interfaces::msg::SegmentAngle>(
      "/estimated_segment_angle", qos,
      [this](custom_interfaces::msg::SegmentAngle::ConstSharedPtr msg) {
        record("angles", msg->header.stamp);
      }));
    if (visualization) {
      stats_["markers"];
      subscriptions_.push_back(create_subscription<visualization_msgs::msg::MarkerArray>(
        "/estimated_segment_reconstruction_markers", qos,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          if (!msg->markers.empty()) {record("markers", msg->markers.front().header.stamp);}
        }));
    }
    window_start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::duration<double>(interval), [this]() {report();});
    RCLCPP_INFO(get_logger(),
      "Rates count received messages over wall time; age is source timestamp to arrival. "
      "Use on the camera host with a common ROS clock. visualization=%s",
      visualization ? "true (subscribes to crop/skeleton/arrows)" : "false");
  }

private:
  struct Stats
  {
    size_t count{0}, repeats{0}, backwards{0}, missing{0};
    int64_t previous_stamp{0};
    std::chrono::steady_clock::time_point previous_arrival{};
    double max_gap_ms{0.0};
    std::vector<double> ages;
  };

  void record(const std::string & name, const builtin_interfaces::msg::Time & stamp)
  {
    auto & stats = stats_.at(name);
    const int64_t ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    const auto arrival = std::chrono::steady_clock::now();
    if (stats.previous_stamp != 0) {
      const auto delta = ns - stats.previous_stamp;
      stats.repeats += delta == 0;
      stats.backwards += delta < 0;
      if (delta > 0) {
        const auto periods = std::llround(static_cast<double>(delta) * 30e-9);
        if (periods > 1) {stats.missing += static_cast<size_t>(periods - 1);}
      }
      stats.max_gap_ms = std::max(stats.max_gap_ms,
        std::chrono::duration<double, std::milli>(arrival - stats.previous_arrival).count());
    }
    stats.previous_stamp = ns;
    stats.previous_arrival = arrival;
    ++stats.count;
    stats.ages.push_back(static_cast<double>(get_clock()->now().nanoseconds() - ns) * 1e-6);
  }

  void report()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - window_start_).count();
    for (auto & entry : stats_) {
      auto & stats = entry.second;
      if (stats.count == 0) {
        RCLCPP_WARN(get_logger(), "%s: 0 Hz (no new messages)", entry.first.c_str());
        continue;
      }
      std::sort(stats.ages.begin(), stats.ages.end());
      const double p50 = stats.ages[stats.ages.size() / 2];
      const double p95 = stats.ages[static_cast<size_t>(0.95 * (stats.ages.size() - 1))];
      RCLCPP_INFO(get_logger(),
        "%s: %.2f Hz | age p50/p95=%.1f/%.1f ms | max_gap=%.1f ms | "
        "repeat=%zu backwards=%zu missing_30hz=%zu",
        entry.first.c_str(), stats.count / elapsed, p50, p95, stats.max_gap_ms,
        stats.repeats, stats.backwards, stats.missing);
      stats.count = stats.repeats = stats.backwards = stats.missing = 0;
      stats.max_gap_ms = 0.0;
      stats.ages.clear();
    }
    window_start_ = now;
  }

  std::map<std::string, Stats> stats_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point window_start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PipelineRateProbe>());
  rclcpp::shutdown();
}
