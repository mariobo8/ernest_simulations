#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <iostream>
#include <vector>
#include <fstream>

using namespace std::chrono_literals;

class JointTrajectoryPublisher : public rclcpp::Node {
public:
    JointTrajectoryPublisher()
        : Node("joint_trajectory_publisher") {
        // Create joint trajectory publisher
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_controller/joint_trajectory", 10);

        // Set up a timer to publish joint angles periodically
        timer_= this->create_wall_timer(
              1000ms, std::bind(&JointTrajectoryPublisher::timer_callback, this));

                // Load joint positions from file
                // Initialize the joint trajectory message
        joint_trajectory_msg.joint_names = {"front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint", "rear_right_steer_joint", "pivot_joint"};
        
        loadJointPositions();
        buildJointTrajectoryMessage();
        
        }

    

    void timer_callback() {
        
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
                joint_trajectory_msg.points[0].time_from_start.nanosec);

       // for (size_t i = 0; i < joint_positions_.size(); ++i) {
       //     RCLCPP_INFO(this->get_logger(), "Element %zu: %f", i, joint_positions_[i]);
       // }
        joint_trajectory_publisher_->publish(joint_trajectory_msg);
        this->timer_->cancel();
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
            joint_positions_.push_back(value);
        }

        inputFile.close();
    }

    trajectory_msgs::msg::JointTrajectory buildJointTrajectoryMessage() {
        joint_trajectory_msg.header.stamp = now();

        // Fill in joint names
        joint_trajectory_msg.joint_names = {"front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint", "rear_right_steer_joint", "pivot_joint"};

        // Fill in joint positions (angles)
        joint_trajectory_msg.points.resize(joint_positions_.size());

        for (size_t i = 0; i < joint_positions_.size(); i++) {
            joint_trajectory_msg.points[i].positions = {joint_positions_[i],
                                                        joint_positions_[i],
                                                        0.0,
                                                        i*0.1,
                                                        0.0};  // adjust joint angle
            if (joint_trajectory_msg.points[i-1].time_from_start.nanosec == 90000000){
                count_++;
                joint_trajectory_msg.points[i].time_from_start.sec = count_;
                joint_trajectory_msg.points[i].time_from_start.nanosec = 0;
                j=0;
            } else {
                j++;
                joint_trajectory_msg.points[i].time_from_start.sec = count_;
                joint_trajectory_msg.points[i].time_from_start.nanosec = j*10000000;
            }



            RCLCPP_INFO(this->get_logger(), "sec %f", joint_trajectory_msg.points[i].time_from_start.sec);
            RCLCPP_INFO(this->get_logger(), "nanosec %f", joint_trajectory_msg.points[i].time_from_start.nanosec);
        }

        return joint_trajectory_msg;
    }
private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    std::vector<double> joint_positions_;
    int count_ = 0;
    size_t j = 0;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointTrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
