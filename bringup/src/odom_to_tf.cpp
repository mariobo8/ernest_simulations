#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OdomToTF : public rclcpp::Node {
    public:
        OdomToTF() : Node("odom_to_tf") {
            std::string odom_topic;
            frame_id = this->declare_parameter("frame_id", std::string("odom"));
            child_frame_id = this->declare_parameter("child_frame_id", std::string("base_link"));
            RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic.c_str());
            if (frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
            }               
            if (child_frame_id != "") {
                RCLCPP_INFO(this->get_logger(), "child_frame_id set to %s", child_frame_id.c_str());
            }
            sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(), std::bind(&OdomToTF::odomCallback, this, _1));
            tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
    private:
        std::string frame_id, child_frame_id;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const {

            geometry_msgs::msg::TransformStamped tfs_;
            tfs_.header = msg->header;
            tfs_.header.stamp = this->now();
            tfs_.header.frame_id = frame_id != "" ? frame_id : tfs_.header.frame_id;
            tfs_.child_frame_id = child_frame_id != "" ? child_frame_id : msg->child_frame_id;
            tfs_.transform.translation.x = msg->pose.pose.position.x;
            tfs_.transform.translation.y = msg->pose.pose.position.y;
            tfs_.transform.translation.z = msg->pose.pose.position.z;

            tfs_.transform.rotation = msg->pose.pose.orientation;

            tfb_->sendTransform(tfs_);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTF>());
    rclcpp::shutdown();
    return 0;
}