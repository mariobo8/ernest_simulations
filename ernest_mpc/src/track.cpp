#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>
#include <vector>


class SquareFollower : public rclcpp::Node {
public:
    SquareFollower() : Node("square_follower") {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive/cmd_vel_unstamped", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr odom) {
                odomCallback(odom);
            });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            publishTwist();
        });

        // Initial values
        linear_speed_ = 0.1;   // Adjust the linear speed as needed
        angular_speed_ = 0.2;  // Adjust the angular speed as needed
        side_length_ = 1.0;    // Adjust the side length of the square as needed
        current_leg_ = 0;
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        // You can use odometry data if needed
        robot_pose_ = odom->pose.pose;
    }

    void publishTwist() {
        geometry_msgs::msg::Twist twist;

        // Move forward along the current leg of the square
        twist.linear.x = linear_speed_;

        // Rotate when reaching the end of a leg
        if (current_leg_ == 0 && robot_pose_.position.x >= side_length_) {
            current_leg_ = 1;
            twist.linear.x = 0.0;  // Stop linear motion
            twist.angular.z = angular_speed_;  // Start angular motion
        } else if (current_leg_ == 1 && robot_pose_.position.y >= side_length_) {
            current_leg_ = 2;
            twist.linear.x = 0.0;  // Stop linear motion
            twist.angular.z = angular_speed_;  // Start angular motion
        } else if (current_leg_ == 2 && robot_pose_.position.x <= 0.0) {
            current_leg_ = 3;
            twist.linear.x = 0.0;  // Stop linear motion
            twist.angular.z = angular_speed_;  // Start angular motion
        } else if (current_leg_ == 3 && robot_pose_.position.y <= 0.0) {
            current_leg_ = 0;
            twist.linear.x = 0.0;  // Stop linear motion
            twist.angular.z = 0.0;  // Stop angular motion
        }

        cmd_vel_publisher_->publish(twist);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose robot_pose_;
    double linear_speed_;
    double angular_speed_;
    double side_length_;
    int current_leg_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SquareFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
