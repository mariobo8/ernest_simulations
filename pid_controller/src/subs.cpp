#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdometrySubscriber : public rclcpp::Node {
public:
    OdometrySubscriber() : Node("odometry_subscriber") {
        // Create an odometry subscriber
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/p3d/odom", 100, std::bind(&OdometrySubscriber::odometry_callback, this, std::placeholders::_1));
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
        // Handle odometry data here
        RCLCPP_INFO(this->get_logger(), "Received odometry data: x=%f, y=%f",
                    odometry_msg->pose.pose.position.x,
                    odometry_msg->pose.pose.position.y);

        tf2::Quaternion q(
          odometry_msg->pose.pose.orientation.x,
          odometry_msg->pose.pose.orientation.y,
          odometry_msg->pose.pose.orientation.z,
          odometry_msg->pose.pose.orientation.w);

        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "yaw=%f",
                    yaw);
        
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    float x, y, z, w;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create an instance of the OdometrySubscriber class
    auto node = std::make_shared<OdometrySubscriber>();

    // Spin the node
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
