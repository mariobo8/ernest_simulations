#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

float desired_pos = 2.0;
float p = 10;

class MotorPositionController : public rclcpp::Node
{
  public:
    MotorPositionController()
    : Node("effort_test_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&MotorPositionController::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 1);      
    }
    float curr_pos;
    float err;
    float drive;

  private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
      std_msgs::msg::Float64MultiArray commands;    
      RCLCPP_INFO(this->get_logger(), "%s", msg->name[0].c_str());
      RCLCPP_INFO(this->get_logger(), "%f", msg->position[0]);      
      RCLCPP_INFO(this->get_logger(), "MSG Received");

      curr_pos = msg->position[0];
      err = desired_pos - curr_pos;
      drive = p * err;
      RCLCPP_INFO(this->get_logger(), "%f", drive);
      commands.data.push_back(drive);
      publisher_->publish(commands);
      
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorPositionController>());
  rclcpp::shutdown();
  return 0;
}