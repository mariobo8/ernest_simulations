#ifndef ernest_CONTROLLER__ernest_CONTROLLER_HPP_
#define ernest_CONTROLLER__ernest_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ernest_controller/odometry.hpp"
#include "ernest_controller/speed_limiter.hpp"
#include "ernest_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace ernest_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ErnestController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  ernest_CONTROLLER_PUBLIC
  ErnestController();

  ernest_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  ernest_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ernest_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ernest_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  ernest_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ernest_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ernest_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ernest_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ernest_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ernest_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct WheelHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
  };

  CallbackReturn configure_side(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);

  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;

  struct WheelParams
  {
    size_t wheels_per_side = 0;
    double separation = 0.0;  // w.r.t. the midpoint of the wheel width
    double radius = 0.0;      // Assumed to be the same for both wheels
    double separation_multiplier = 1.0;
    double left_radius_multiplier = 1.0;
    double right_radius_multiplier = 1.0;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time previous_publish_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace ernest_controller
#endif  // ernest_CONTROLLER__ernest_CONTROLLER_HPP_
