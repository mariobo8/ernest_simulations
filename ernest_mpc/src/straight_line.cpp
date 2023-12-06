#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>
#include <vector>


class StraightPathFollower : public rclcpp::Node
{
public:
    StraightPathFollower()
        : Node("straight_path_follower"), distance_to_travel_(10.0), linear_velocity_(1)
    {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&StraightPathFollower::odom_callback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive/cmd_vel_unstamped", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&StraightPathFollower::timer_callback, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_distance_traveled_ = msg->pose.pose.position.x;
    }

    void timer_callback()
    {
        if (current_distance_traveled_ < distance_to_travel_)
        {
            auto cmd_vel_msg = geometry_msgs::msg::Twist();
            cmd_vel_msg.linear.x = linear_velocity_;

            RCLCPP_INFO(this->get_logger(), "Publishing on cmd_vel: linear.x = %f", cmd_vel_msg.linear.x);
            RCLCPP_INFO(this->get_logger(), "the distance traveled = %f", current_distance_traveled_);
            cmd_vel_publisher_->publish(cmd_vel_msg);
        }
        else
        {
            // Stop the robot when the desired distance is reached
            auto cmd_vel_msg = geometry_msgs::msg::Twist();
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_publisher_->publish(cmd_vel_msg);

            RCLCPP_INFO(this->get_logger(), "Reached the desired distance. Stopping the robot.");
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double distance_to_travel_;
    double current_distance_traveled_;
    double linear_velocity_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto straight_path_follower = std::make_shared<StraightPathFollower>();

    rclcpp::spin(straight_path_follower);

    rclcpp::shutdown();

    return 0;
}
