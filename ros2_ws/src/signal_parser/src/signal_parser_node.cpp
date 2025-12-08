#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cstdint>

namespace
{
inline uint16_t be_u16(const uint8_t* d) {
  return static_cast<uint16_t>((d[0] << 8) | d[1]);
}

inline int16_t be_i16(const uint8_t* d) {
  return static_cast<int16_t>((d[0] << 8) | d[1]);
}

inline bool has_bytes(const can_msgs::msg::Frame& f, size_t n) {
  return f.dlc >= n;
}
} // namespace

class SignalParser : public rclcpp::Node {
public:
  SignalParser()
  : Node("signal_parser")
  {
    speed_id_   = declare_parameter<int>("speed_id",   0x100);
    steer_id_   = declare_parameter<int>("steer_id",   0x101);
    battery_id_ = declare_parameter<int>("battery_id", 0x102);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();

    sub_can_ = create_subscription<can_msgs::msg::Frame>(
      "can_rx", qos,
      std::bind(&SignalParser::on_can_frame, this, std::placeholders::_1));

    pub_odom_  = create_publisher<nav_msgs::msg::Odometry>("/vehicle/odometry", qos);
    pub_batt_  = create_publisher<std_msgs::msg::Float32>("/vehicle/status", qos);
    pub_steer_ = create_publisher<std_msgs::msg::Float32>("/vehicle/steering", qos);

    RCLCPP_INFO(get_logger(),
                "signal_parser ready (IDs: speed=0x%X steer=0x%X batt=0x%X)",
                speed_id_, steer_id_, battery_id_);
  }

private:
  void on_can_frame(const can_msgs::msg::Frame& f)
  {
    const auto id = static_cast<int>(f.id);

    if (id == speed_id_) {
      if (!has_bytes(f, 2)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Speed frame dlc<2 (%u)", f.dlc);
        return;
      }
      const uint16_t raw = be_u16(f.data.data());
      const float speed_mps = static_cast<float>(raw) * 0.01f;
      publish_odom(speed_mps);
      return;
    }

    if (id == steer_id_) {
      if (!has_bytes(f, 2)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Steer frame dlc<2 (%u)", f.dlc);
        return;
      }
      const int16_t raw = be_i16(f.data.data());
      const float steering_deg = static_cast<float>(raw) * 0.1f;
      std_msgs::msg::Float32 m;
      m.data = steering_deg;
      pub_steer_->publish(m);
      return;
    }

    if (id == battery_id_) {
      if (!has_bytes(f, 1)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Battery frame dlc<1 (%u)", f.dlc);
        return;
      }
      const float pct = static_cast<float>(f.data[0]);
      std_msgs::msg::Float32 m;
      m.data = pct;
      pub_batt_->publish(m);
      return;
    }
  }

  void publish_odom(float speed_mps)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = speed_mps;
    pub_odom_->publish(odom);
  }

  int speed_id_{0x100};
  int steer_id_{0x101};
  int battery_id_{0x102};

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_batt_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignalParser>());
  rclcpp::shutdown();
  return 0;
}
