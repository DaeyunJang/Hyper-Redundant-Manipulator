#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageRateProbe : public rclcpp::Node
{
public:
  ImageRateProbe()
  : Node("image_rate_probe")
  {
    topic_ = declare_parameter<std::string>(
      "topic", "/camera/camera/aligned_depth_to_color/image_raw");
    expected_fps_ = declare_parameter<double>("expected_fps", 30.0);
    report_interval_sec_ = declare_parameter<double>("report_interval_sec", 5.0);
    qos_depth_ = declare_parameter<int64_t>("qos_depth", 1);

    if (expected_fps_ <= 0.0) {
      throw std::invalid_argument("expected_fps must be greater than zero");
    }
    if (report_interval_sec_ <= 0.0) {
      throw std::invalid_argument("report_interval_sec must be greater than zero");
    }
    if (qos_depth_ <= 0) {
      throw std::invalid_argument("qos_depth must be greater than zero");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(
      static_cast<size_t>(qos_depth_));
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      topic_, qos,
      std::bind(&ImageRateProbe::image_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(report_interval_sec_),
      std::bind(&ImageRateProbe::report, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening to %s with BEST_EFFORT/KEEP_LAST(%ld); expected %.1f Hz",
      topic_.c_str(), static_cast<long>(qos_depth_), expected_fps_);
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  static double seconds_between(
    const SteadyClock::time_point & newer,
    const SteadyClock::time_point & older)
  {
    return std::chrono::duration<double>(newer - older).count();
  }

  static int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL +
           static_cast<int64_t>(stamp.nanosec);
  }

  void reset_window(const SteadyClock::time_point & now, int64_t header_ns)
  {
    window_count_ = 1;
    window_bytes_ = last_payload_bytes_;
    window_first_arrival_ = now;
    window_last_arrival_ = now;
    window_first_header_ns_ = header_ns;
    window_last_header_ns_ = header_ns;
    previous_arrival_ = now;
    previous_header_ns_ = header_ns;
    min_arrival_dt_sec_ = std::numeric_limits<double>::infinity();
    max_arrival_dt_sec_ = 0.0;
    min_header_dt_sec_ = std::numeric_limits<double>::infinity();
    max_header_dt_sec_ = 0.0;
    estimated_missing_frames_ = 0;
    non_monotonic_header_count_ = 0;
    window_initialized_ = true;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    const auto now = SteadyClock::now();
    const int64_t header_ns = stamp_nanoseconds(msg->header.stamp);
    last_payload_bytes_ = msg->data.size();

    if (!received_first_message_) {
      RCLCPP_INFO(
        get_logger(), "First image: %ux%u, encoding=%s, payload=%.3f MiB, frame_id=%s",
        msg->width, msg->height, msg->encoding.c_str(),
        static_cast<double>(last_payload_bytes_) / (1024.0 * 1024.0),
        msg->header.frame_id.c_str());
      received_first_message_ = true;
    }

    if (!window_initialized_) {
      reset_window(now, header_ns);
      return;
    }

    const double arrival_dt = seconds_between(now, previous_arrival_);
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
    } else {
      ++non_monotonic_header_count_;
    }

    ++window_count_;
    window_bytes_ += last_payload_bytes_;
    window_last_arrival_ = now;
    window_last_header_ns_ = header_ns;
    previous_arrival_ = now;
    previous_header_ns_ = header_ns;
  }

  void report()
  {
    if (!window_initialized_ || window_count_ < 2) {
      RCLCPP_WARN(get_logger(), "No image stream received yet on %s", topic_.c_str());
      window_initialized_ = false;
      return;
    }

    const double arrival_span = seconds_between(
      window_last_arrival_, window_first_arrival_);
    const double header_span = static_cast<double>(
      window_last_header_ns_ - window_first_header_ns_) * 1e-9;
    const double arrival_hz = arrival_span > 0.0 ?
      static_cast<double>(window_count_ - 1) / arrival_span : 0.0;
    const double header_hz = header_span > 0.0 ?
      static_cast<double>(window_count_ - 1) / header_span : 0.0;
    const double throughput_mib_s = arrival_span > 0.0 ?
      static_cast<double>(window_bytes_) / (1024.0 * 1024.0) / arrival_span : 0.0;

    RCLCPP_INFO(
      get_logger(),
      "topic=%s | received=%zu | arrival=%.2f Hz | header=%.2f Hz | "
      "arrival_dt=%.2f..%.2f ms | header_dt=%.2f..%.2f ms | "
      "estimated_missing=%ld | non_monotonic=%zu | throughput=%.1f MiB/s",
      topic_.c_str(), window_count_, arrival_hz, header_hz,
      min_arrival_dt_sec_ * 1e3, max_arrival_dt_sec_ * 1e3,
      min_header_dt_sec_ * 1e3, max_header_dt_sec_ * 1e3,
      static_cast<long>(estimated_missing_frames_),
      non_monotonic_header_count_, throughput_mib_s);

    window_initialized_ = false;
  }

  std::string topic_;
  double expected_fps_{30.0};
  double report_interval_sec_{5.0};
  int64_t qos_depth_{1};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool received_first_message_{false};
  bool window_initialized_{false};
  size_t window_count_{0};
  size_t non_monotonic_header_count_{0};
  size_t last_payload_bytes_{0};
  uint64_t window_bytes_{0};
  int64_t window_first_header_ns_{0};
  int64_t window_last_header_ns_{0};
  int64_t previous_header_ns_{0};
  int64_t estimated_missing_frames_{0};
  SteadyClock::time_point window_first_arrival_;
  SteadyClock::time_point window_last_arrival_;
  SteadyClock::time_point previous_arrival_;
  double min_arrival_dt_sec_{std::numeric_limits<double>::infinity()};
  double max_arrival_dt_sec_{0.0};
  double min_header_dt_sec_{std::numeric_limits<double>::infinity()};
  double max_header_dt_sec_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageRateProbe>());
  rclcpp::shutdown();
  return 0;
}
