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
                RCLCPP_INFO(
                        get_logger(),
                        "Trajectory message:\n"
                        "Header stamp: %ld.%09d\n"
                        "Joint names: %s\n"
                        "Positions: [%f, %f, %f, %f, %f]\n"
                        "Time from start: %ld.%09d",
                        joint_trajectory_msg.header.stamp.sec,
                        joint_trajectory_msg.header.stamp.nanosec,
                        joint_trajectory_msg.joint_names[0].c_str(),
                        joint_trajectory_msg.points[0].positions[0],
                        joint_trajectory_msg.points[0].positions[1],
                        joint_trajectory_msg.points[0].positions[2],
                        joint_trajectory_msg.points[0].positions[3],
                        joint_trajectory_msg.points[0].positions[4],
                        joint_trajectory_msg.points[0].time_from_start.sec,
                        joint_trajectory_msg.points[0].time_from_start.nanosec
                        );
                joint_trajectory_publisher_->publish(joint_trajectory_msg);
            });
        
    }


    trajectory_msgs::msg::JointTrajectory buildJointTrajectoryMessage() {
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        joint_trajectory_msg.header.stamp = rclcpp::Time(0, 0);

        // Fill in joint names
        joint_trajectory_msg.joint_names = {"front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint", "rear_right_steer_joint", "pivot_joint"};

        // Fill in joint positions (angles)
        joint_trajectory_msg.points.resize(1);
        joint_trajectory_msg.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0};  // adjust joint angle
    
        joint_trajectory_msg.points[0].time_from_start.sec = 0;
        joint_trajectory_msg.points[0].time_from_start.nanosec = 0;
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
