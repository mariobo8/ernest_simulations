#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("velocity_publisher"), current_index_(0) {
        // Create velocity publisher
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);
        steering_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10);
        // Set up a timer to publish velocity commands periodically
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TrajectoryPublisher::timer_callback, this));

    loadJointPositions();
    loadJointVel();
    }

    void timer_callback() {
        if (current_index_ < velocities_.size()) {
            // Create a Float64MultiArray message
            std_msgs::msg::Float64MultiArray velocity_msg;
            std_msgs::msg::Float64MultiArray steering_msg;
            // Populate the data field with velocities
            velocity_msg.data = {velocities_[current_index_], velocities_[current_index_]};
            steering_msg.data = {st_angles_[current_index_], st_angles_[current_index_], 0.0, 0.0, 0.0};
            // Publish the Float64MultiArray message
            RCLCPP_INFO(this->get_logger(),"curent_index: %zu", current_index_);
            velocity_publisher_->publish(velocity_msg);
            steering_publisher_->publish(steering_msg);

            // Move to the next set of velocities
            current_index_++;
            
        } else {
            // Stop the timer when all velocities have been published
            timer_->cancel();
            rclcpp::shutdown();
        }
    }

    void loadJointPositions() {
        // Construct the relative path
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("controller");
        std::string full_path = package_share_directory + "/matlab/steering_angles.txt";
        RCLCPP_INFO(this->get_logger(),"ss %s", full_path.c_str());
        std::ifstream inputFile(full_path);

        if (!inputFile.is_open()) {
            std::cerr << "Error opening file." << std::endl;
            return;
        }

        double value;
        while (inputFile >> value) {
            st_angles_.push_back(value);
        }

        inputFile.close();
    }

    void loadJointVel() {
        // Construct the relative path
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("controller");
        std::string full_path = package_share_directory + "/matlab/speed.txt";
        RCLCPP_INFO(this->get_logger(),"ss %s", full_path.c_str());
        std::ifstream inputFile(full_path);

        if (!inputFile.is_open()) {
            std::cerr << "Error opening file." << std::endl;
            return;
        }

        double value;
        while (inputFile >> value) {
            velocities_.push_back(value / wheel_radius);
        }

        inputFile.close();
    }


private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> velocities_;
    std::vector<double> st_angles_;
    size_t current_index_;
    std::string package_share_directory;
    float wheel_radius = 0.2;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
