#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MyControllerNode : public rclcpp::Node {
public:
    MyControllerNode() : Node("my_controller_node") {
        // Initialize publisher for twist commands
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize subscriber for odometry
        odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MyControllerNode::odometryCallback, this, std::placeholders::_1));

        // Set your proportional gain here
        proportional_gain_ = 0.5;  // Adjust as needed
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {

        // For example, let's say you want to control linear velocity based on position error
        double desired_position = 5.0;  // Set your desired position
        double current_position = odometry_msg->pose.pose.position.x;
        double error = desired_position - current_position;

        // Proportional control law: cmd = Kp * error
        double cmd_linear_velocity = proportional_gain_ * error;

        // Create and publish the twist message
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = cmd_linear_velocity;

        twist_publisher_->publish(std::move(twist_msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    double proportional_gain_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyControllerNode>());
    rclcpp::shutdown();
    return 0;
}
