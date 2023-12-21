#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node {
public:
    VelocityPublisher() : Node("velocity_publisher") {
        // Create velocity publisher
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        // Set up a timer to publish velocity commands periodically
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&VelocityPublisher::timer_callback, this));

        // Initialize the velocities
        velocities_ = {1.0, 1.0, 1.0,1.0, 1.0, 1.0,1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // Adjust velocities for each joint
        current_index_ = 0;
    }

    void timer_callback() {
        if (current_index_ < velocities_.size()) {
            // Create a Float64MultiArray message
            std_msgs::msg::Float64MultiArray velocity_msg;

            // Populate the data field with velocities
            velocity_msg.data = {velocities_[current_index_], velocities_[current_index_]};

            // Publish the Float64MultiArray message
            velocity_publisher_->publish(velocity_msg);

            // Move to the next set of velocities
            current_index_++;
        } else {
            // Stop the timer when all velocities have been published
            timer_->cancel();
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> velocities_;
    size_t current_index_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
