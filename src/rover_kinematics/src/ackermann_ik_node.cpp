// ackermann_ik_node: geometry_msgs/Twist -> 4 steer positions + 4 wheel velocities.
// Thin ROS layer over the pure rover_kinematics library (ADR-0002).
// Safety: watchdog zeroes drive if no /cmd_vel within cmd_timeout (steer held).
// Unachievable wz is clamped to the min-turning-radius envelope; point turn -> zero drive.
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rover_kinematics/double_ackermann.hpp"

using namespace std::chrono_literals;

class AckermannIkNode : public rclcpp::Node {
 public:
  AckermannIkNode() : Node("ackermann_ik_node") {
    cfg_.wheelbase    = declare_parameter("wheelbase", 0.882);
    cfg_.track        = declare_parameter("track", 0.834);
    cfg_.wheel_radius = declare_parameter("wheel_radius", 0.1465);
    cfg_.max_steer    = declare_parameter("max_steer", 1.5708);
    max_wheel_speed_  = declare_parameter("max_wheel_speed", 28.0);   // rad/s, AK10-9 KV100
    drive_axis_sign_  = declare_parameter("drive_axis_sign", -1.0);   // wheel joint axis is -X in model
    steer_axis_sign_  = declare_parameter("steer_axis_sign", 1.0);
    cmd_timeout_      = declare_parameter("cmd_timeout", 0.5);        // s

    kin_ = std::make_unique<rover_kinematics::DoubleAckermann>(cfg_);
    RCLCPP_INFO(get_logger(), "double-Ackermann IK up: L=%.3f W=%.3f r=%.4f min_R=%.3f m",
                cfg_.wheelbase, cfg_.track, cfg_.wheel_radius, kin_->min_turning_radius());

    const char* topics[8] = {
      "/karasimsek/cmd/fl_steer", "/karasimsek/cmd/fr_steer",
      "/karasimsek/cmd/rl_steer", "/karasimsek/cmd/rr_steer",
      "/karasimsek/cmd/fl_wheel", "/karasimsek/cmd/fr_wheel",
      "/karasimsek/cmd/rl_wheel", "/karasimsek/cmd/rr_wheel"};
    for (int i = 0; i < 8; ++i)
      pubs_[i] = create_publisher<std_msgs::msg::Float64>(topics[i], 10);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          last_cmd_time_ = now();
          compute(*msg);
        });

    timer_ = create_wall_timer(20ms, [this]() { publish(); });  // 50 Hz
    last_cmd_time_ = now();
  }

 private:
  void compute(const geometry_msgs::msg::Twist& t) {
    rover_kinematics::BodyTwist bt{t.linear.x, t.angular.z};

    // clamp wz into the achievable envelope instead of rejecting (teleop-friendly)
    const double rmin = kin_->min_turning_radius();
    if (std::abs(bt.vx) > 1e-6 && std::abs(bt.wz) > 1e-9) {
      const double wz_max = std::abs(bt.vx) / rmin;
      if (std::abs(bt.wz) > wz_max) bt.wz = (bt.wz > 0 ? wz_max : -wz_max);
    }

    rover_kinematics::WheelCommands w;
    if (!kin_->ik(bt, w)) {           // point turn or other unachievable: stop drive, hold steer
      for (int i = 0; i < 4; ++i) cmd_[4 + i] = 0.0;
      return;
    }
    for (int i = 0; i < 4; ++i) {
      cmd_[i] = steer_axis_sign_ * w.steer_rad[i];
      double omega = drive_axis_sign_ * w.omega_rads[i];
      omega = std::clamp(omega, -max_wheel_speed_, max_wheel_speed_);
      cmd_[4 + i] = omega;
    }
  }

  void publish() {
    // watchdog: no fresh cmd_vel -> zero drive (keep last steer, avoids scrub-in-place)
    if ((now() - last_cmd_time_).seconds() > cmd_timeout_)
      for (int i = 0; i < 4; ++i) cmd_[4 + i] = 0.0;
    std_msgs::msg::Float64 m;
    for (int i = 0; i < 8; ++i) { m.data = cmd_[i]; pubs_[i]->publish(m); }
  }

  rover_kinematics::Config cfg_{};
  std::unique_ptr<rover_kinematics::DoubleAckermann> kin_;
  double max_wheel_speed_{28.0}, drive_axis_sign_{-1.0}, steer_axis_sign_{1.0}, cmd_timeout_{0.5};
  std::array<double, 8> cmd_{};  // 0-3 steer, 4-7 wheel
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 8> pubs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannIkNode>());
  rclcpp::shutdown();
  return 0;
}
