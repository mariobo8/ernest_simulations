#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node {
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher"),
          publisher_(create_publisher<trajectory_msgs::msg::JointTrajectory>(
              "/joint_controller/joint_trajectory", 10)),
          timer_(create_wall_timer(
              1000ms, std::bind(&JointTrajectoryPublisher::timer_callback, this))) {

            // Load joint positions from file
            // Initialize the joint trajectory message
            joint_trajectory_msg_.joint_names = {"front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint", "rear_right_steer_joint", "pivot_joint"};
            joint_trajectory_msg_.points.resize(joint_positions_.size());
            loadJointPositions();
            storePositions();

          }

    void timer_callback() {
        // Update the timestamp before publishing
        joint_trajectory_msg_.header.stamp = now();
        RCLCPP_INFO(this->get_logger(),"saverio %f", joint_trajectory_msg_.points[0].positions);
        publisher_->publish(joint_trajectory_msg_);
    }

    // Open txt file
    void loadJointPositions() {
        std::ifstream inputFile("/home/mariobo/ernest_ws/src/ernest_simulations/ernest_controller/matlab/steering_angles.txt");

        if (!inputFile.is_open()) {
            std::cerr << "Error opening file." << std::endl;
            return;
        }

        double value;
        while (inputFile >> value) {
            joint_positions_.push_back(value * 10);
        }

        inputFile.close();
    }

    // Save trajectories in a matrix
    trajectory_msgs::msg::JointTrajectory storePositions() {
        for (size_t i = 0; i < joint_positions_.size(); i++) {
            joint_trajectory_msg_.points[i].positions = {
                joint_positions_[i],
                joint_positions_[i],
                0.0,
                0.0,
                0.0
            };
        }
        return joint_trajectory_msg_;
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> joint_positions_;
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}