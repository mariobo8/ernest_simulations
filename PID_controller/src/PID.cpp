#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {
public:
   TrajectoryPublisher() : Node("velocity_publisher")  {
      // Create velocity publisher
      velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
         "/velocity_controller/commands", 10);
      steering_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
         "/position_controller/commands", 10);
      // Create odometry subscriber
      odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
         "/p3d/odom", 10, std::bind(&TrajectoryPublisher::odometry_callback, this, std::placeholders::_1));

      // Set up a timer to publish velocity commands periodically
      timer_ = this->create_wall_timer(
         100ms, std::bind(&TrajectoryPublisher::timer_callback, this));

   }

   void timer_callback() {
      if (abs(dist - dist_r) > 0.2 ) {
         // Create a Float64MultiArray message
         //std_msgs::msg::Float64MultiArray velocity_msg;
         //std_msgs::msg::Float64MultiArray steering_msg;
         // Populate the data field with velocities
         velocity_msg.data = {cmd_v, cmd_v};
         steering_msg.data = {cmd_y, cmd_y, -cmd_y, -cmd_y, cmd_y};

         velocity_publisher_->publish(velocity_msg);
         steering_publisher_->publish(steering_msg);
      } else {
         // Stop the timer when all velocities have been published
         stop_vel.data = {0, 0};
         stop_steer.data = {0, 0, 0.0, 0.0, 0.0};
         velocity_publisher_->publish(velocity_msg);
         steering_publisher_->publish(steering_msg);
         saveToCSV("odometry_data.csv");
         timer_->cancel();
         rclcpp::shutdown();
      }
   }

   void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
   // Handle odometry data here
      
      x = odometry_msg->pose.pose.position.x;
      y = odometry_msg->pose.pose.position.y;

      RCLCPP_INFO(this->get_logger(), "distance: dist=%f, ang_dist=%f",
                  e_dist,
                  e_yaw);

      tf2::Quaternion q(
         odometry_msg->pose.pose.orientation.x,
         odometry_msg->pose.pose.orientation.y,
         odometry_msg->pose.pose.orientation.z,
         odometry_msg->pose.pose.orientation.w);

      
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
   
      // compute the yaw ref
      yaw_r = atan2(y_r, x_r);   

      // compute the error on the distanodometry_msg->ce
      dist_r = sqrt(y_r*y_r + x_r*x_r);
      dist = sqrt(y*y+ x*x);
      e_dist = dist_r - dist;
      cmd_v = e_dist * kpv;

      //compute error on yaw
      e_yaw = yaw_r - yaw;
      cmd_y = - e_yaw * kpy;

      //save sata
      position_history.emplace_back(x, y);


        
   }

   void saveToCSV(const std::string& filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
        for (const auto& position : position_history) {
            file << position.first << "," << position.second << "\n";
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Data saved to %s", filename.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", filename.c_str());
    }
   }


private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;  // Added odometry subscriber
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> velocities_;
    std::vector<double> st_angles_;
    size_t current_index_;
    std::string package_share_directory;
    float wheel_radius = 0.2;

    std_msgs::msg::Float64MultiArray velocity_msg, stop_vel;
    std_msgs::msg::Float64MultiArray steering_msg, stop_steer;

    std::vector<std::pair<double, double>> position_history;

    //waypoint
    float x_r = 5;
    float y_r = 5;
    float yaw_r, dist_r = 1, dist, x, y, e_dist, cmd_v, e_yaw, cmd_y;
    float kpv = 0.7;
    float kpy = 0.8;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
