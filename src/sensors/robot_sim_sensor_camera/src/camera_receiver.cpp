#include <chrono>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace
{

std::string number_text(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

std::string stamp_text(const builtin_interfaces::msg::Time & stamp)
{
  std::ostringstream stream;
  stream << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec;
  return stream.str();
}

struct TopicStats
{
  std::string topic;
  std::size_t count{0};
  std::chrono::steady_clock::time_point first_time;
  std::chrono::steady_clock::time_point last_time;
  builtin_interfaces::msg::Time last_stamp;
  std::string frame_id;

  double hz() const
  {
    if (count < 2) {
      return 0.0;
    }
    const auto elapsed =
      std::chrono::duration<double>(last_time - first_time).count();
    if (elapsed <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(count - 1) / elapsed;
  }
};

class CameraReceiver : public rclcpp::Node
{
public:
  CameraReceiver()
  : rclcpp::Node("camera_receiver")
  {
    const auto image_topic =
      declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    expected_min_hz_ = declare_parameter<double>("expected_min_hz", 1.0);
    if (expected_min_hz_ < 0.0) {
      expected_min_hz_ = 0.0;
    }
    auto log_period_sec = declare_parameter<double>("log_period_sec", 5.0);
    if (log_period_sec <= 0.0) {
      log_period_sec = 5.0;
    }
    receiver_type_ = declare_parameter<std::string>("receiver_type", "camera");
    sensor_name_ = declare_parameter<std::string>("sensor_name", "camera");

    add_subscription<sensor_msgs::msg::Image>("image", image_topic);
    add_subscription<sensor_msgs::msg::CameraInfo>("camera_info", camera_info_topic);

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(log_period_sec)),
      std::bind(&CameraReceiver::publish_health, this));
  }

private:
  template<typename MsgT>
  void add_subscription(const std::string & label, const std::string & topic)
  {
    if (topic.empty()) {
      throw std::runtime_error(label + " topic parameter is empty");
    }
    stats_[label].topic = topic;
    subscriptions_.push_back(create_subscription<MsgT>(
      topic,
      rclcpp::SensorDataQoS(),
      [this, label](typename MsgT::SharedPtr msg) {
        update_stats(label, msg->header.stamp, msg->header.frame_id);
      }));
    RCLCPP_INFO(get_logger(), "subscribing %s: %s", label.c_str(), topic.c_str());
  }

  void update_stats(
    const std::string & label,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id)
  {
    auto & stat = stats_[label];
    const auto now = std::chrono::steady_clock::now();
    if (stat.count == 0) {
      stat.first_time = now;
    }
    stat.last_time = now;
    stat.last_stamp = stamp;
    stat.frame_id = frame_id;
    ++stat.count;
  }

  void publish_health()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    std::ostringstream log;
    log << "sensor=" << sensor_name_ << " type=" << receiver_type_;

    for (const auto & item : stats_) {
      const auto & label = item.first;
      const auto & stat = item.second;
      const double rate = stat.hz();

      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = std::string(get_fully_qualified_name()) + "/" + label;
      status.hardware_id = "robot_sim_sensor_camera";
      if (stat.count == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "waiting for messages";
      } else if (expected_min_hz_ > 0.0 && rate < expected_min_hz_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "below expected_min_hz";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "receiving";
      }

      status.values.push_back(key_value("topic", stat.topic));
      status.values.push_back(key_value("count", std::to_string(stat.count)));
      status.values.push_back(key_value("hz", number_text(rate)));
      status.values.push_back(key_value("expected_min_hz", number_text(expected_min_hz_)));
      status.values.push_back(key_value("last_stamp", stamp_text(stat.last_stamp)));
      status.values.push_back(key_value("frame_id", stat.frame_id));
      array.status.push_back(status);

      log << " " << label << "[count=" << stat.count << ",hz=" << number_text(rate)
          << ",frame=" << stat.frame_id << "]";
    }

    diagnostics_pub_->publish(array);
    RCLCPP_INFO(get_logger(), "%s", log.str().c_str());
  }

  diagnostic_msgs::msg::KeyValue key_value(
    const std::string & key,
    const std::string & value) const
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  std::map<std::string, TopicStats> stats_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double expected_min_hz_{1.0};
  std::string receiver_type_;
  std::string sensor_name_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraReceiver>());
  rclcpp::shutdown();
  return 0;
}
