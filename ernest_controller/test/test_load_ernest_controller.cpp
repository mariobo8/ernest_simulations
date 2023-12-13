#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "test_common.hpp"

TEST(TestLoadErnestController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(
      ernest_controller_testing::diffbot_urdf),
    executor, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller("test_ernest_controller", "ernest_controller/ErnestController"));

  rclcpp::shutdown();
}
