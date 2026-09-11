#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "realsense2_camera_msgs/msg/metadata.hpp"

class MetadataRateProbe : public rclcpp::Node
{
public:
  MetadataRateProbe()
  : Node("metadata_rate_probe")
  {
    topic_ = declare_parameter<std::string>(
      "topic", "/camera/camera/depth/metadata");
    expected_fps_ = declare_parameter<double>("expected_fps", 30.0);
    report_interval_sec_ = declare_parameter<double>("report_interval_sec", 5.0);

    if (expected_fps_ <= 0.0 || report_interval_sec_ <= 0.0) {
      throw std::invalid_argument(
              "expected_fps and report_interval_sec must be greater than zero");
    }

    subscription_ = create_subscription<realsense2_camera_msgs::msg::Metadata>(
      topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&MetadataRateProbe::callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(report_interval_sec_),
      std::bind(&MetadataRateProbe::report, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening to %s with BEST_EFFORT/KEEP_LAST(1); expected %.1f Hz",
      topic_.c_str(), expected_fps_);
  }

private:
  using Metadata = realsense2_camera_msgs::msg::Metadata;
  using SteadyClock = std::chrono::steady_clock;

  static int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
           static_cast<int64_t>(stamp.nanosec);
  }

  void callback(const Metadata::ConstSharedPtr msg)
  {
    const auto now = SteadyClock::now();
    const int64_t header_ns = stamp_nanoseconds(msg->header.stamp);

    if (!initialized_) {
      count_ = 1;
      first_arrival_ = now;
      last_arrival_ = now;
      previous_arrival_ = now;
      first_header_ns_ = header_ns;
      last_header_ns_ = header_ns;
      previous_header_ns_ = header_ns;
      min_arrival_dt_sec_ = std::numeric_limits<double>::infinity();
      max_arrival_dt_sec_ = 0.0;
      min_header_dt_sec_ = std::numeric_limits<double>::infinity();
      max_header_dt_sec_ = 0.0;
      estimated_missing_frames_ = 0;
      initialized_ = true;
      return;
    }

    const double arrival_dt = std::chrono::duration<double>(
      now - previous_arrival_).count();
    min_arrival_dt_sec_ = std::min(min_arrival_dt_sec_, arrival_dt);
    max_arrival_dt_sec_ = std::max(max_arrival_dt_sec_, arrival_dt);

    const int64_t header_dt_ns = header_ns - previous_header_ns_;
    if (header_dt_ns > 0) {
      const double header_dt = static_cast<double>(header_dt_ns) * 1e-9;
      min_header_dt_sec_ = std::min(min_header_dt_sec_, header_dt);
      max_header_dt_sec_ = std::max(max_header_dt_sec_, header_dt);
      const auto expected_periods = static_cast<int64_t>(
        std::llround(header_dt * expected_fps_));
      if (expected_periods > 1) {
        estimated_missing_frames_ += expected_periods - 1;
      }
    }

    ++count_;
    last_arrival_ = now;
    previous_arrival_ = now;
    last_header_ns_ = header_ns;
    previous_header_ns_ = header_ns;
  }

  void report()
  {
    if (!initialized_ || count_ < 2) {
      RCLCPP_WARN(get_logger(), "No metadata received yet on %s", topic_.c_str());
      initialized_ = false;
      return;
    }

    const double arrival_span = std::chrono::duration<double>(
      last_arrival_ - first_arrival_).count();
    const double header_span = static_cast<double>(
      last_header_ns_ - first_header_ns_) * 1e-9;
    const double arrival_hz = arrival_span > 0.0 ?
      static_cast<double>(count_ - 1) / arrival_span : 0.0;
    const double header_hz = header_span > 0.0 ?
      static_cast<double>(count_ - 1) / header_span : 0.0;

    RCLCPP_INFO(
      get_logger(),
      "topic=%s | received=%zu | arrival=%.2f Hz | header=%.2f Hz | "
      "arrival_dt=%.2f..%.2f ms | header_dt=%.2f..%.2f ms | "
      "estimated_missing=%ld",
      topic_.c_str(), count_, arrival_hz, header_hz,
      min_arrival_dt_sec_ * 1e3, max_arrival_dt_sec_ * 1e3,
      min_header_dt_sec_ * 1e3, max_header_dt_sec_ * 1e3,
      static_cast<long>(estimated_missing_frames_));

    initialized_ = false;
  }

  std::string topic_;
  double expected_fps_{30.0};
  double report_interval_sec_{5.0};
  rclcpp::Subscription<Metadata>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool initialized_{false};
  size_t count_{0};
  int64_t first_header_ns_{0};
  int64_t last_header_ns_{0};
  int64_t previous_header_ns_{0};
  int64_t estimated_missing_frames_{0};
  SteadyClock::time_point first_arrival_;
  SteadyClock::time_point last_arrival_;
  SteadyClock::time_point previous_arrival_;
  double min_arrival_dt_sec_{std::numeric_limits<double>::infinity()};
  double max_arrival_dt_sec_{0.0};
  double min_header_dt_sec_{std::numeric_limits<double>::infinity()};
  double max_header_dt_sec_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MetadataRateProbe>());
  rclcpp::shutdown();
  return 0;
}
