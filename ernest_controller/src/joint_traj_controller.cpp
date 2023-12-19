#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class JointTrajectoryPublisher : public rclcpp::Node {
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher") {
        // Create joint trajectory publisher
        joint_trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_controller/joint_trajectory", 10);

        // Set up a timer to publish joint angles periodically
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1),  // adjust the frequency as needed
            [this]() {
                auto joint_trajectory_msg = buildJointTrajectoryMessage();
                joint_trajectory_publisher_->publish(joint_trajectory_msg);
            });
    }

private:
    trajectory_msgs::msg::JointTrajectory buildJointTrajectoryMessage() {
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.header.stamp = now();

        // Fill in joint names
        joint_trajectory_msg.joint_names = {"front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint", "rear_right_steer_joint", "pivot_joint"};

        // Fill in joint positions (angles)
        joint_trajectory_msg.points.resize(1);
        joint_trajectory_msg.points[0].positions = {1.0, 1.0, 0.0, 0.0, 0.0};  // adjust joint angles

        return joint_trajectory_msg;
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
