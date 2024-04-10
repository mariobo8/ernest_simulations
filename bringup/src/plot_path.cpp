#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>


class PathNode : public rclcpp::Node
{
public:
    PathNode() : Node("path_node")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&PathNode::odom_cb, this, std::placeholders::_1));
    }

private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {   
        RCLCPP_INFO(this->get_logger(), "cb");
        path_.header = msg->header;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp.sec = msg->header.stamp.sec;
        pose.header.stamp.nanosec = msg->header.stamp.nanosec;
        pose.header.frame_id = "pivot";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pose.pose = msg->pose.pose;
        path_.poses.push_back(pose);
        path_pub_->publish(path_);
        RCLCPP_INFO(this->get_logger(), "pub");
        
    }

    nav_msgs::msg::Path path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathNode>());
    rclcpp::shutdown();
    return 0;
}
