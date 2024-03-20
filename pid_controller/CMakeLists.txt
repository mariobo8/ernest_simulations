cmake_minimum_required(VERSION 3.5)
project(pid_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
#find_package(tf2_geometry_msgs REQUIRED)

# Add the ament_index_cpp include directory
include_directories(${ament_index_cpp_INCLUDE_DIRS})

add_executable(trajectory_controller src/trajectory_controller.cpp)
ament_target_dependencies(trajectory_controller rclcpp trajectory_msgs nav_msgs ament_index_cpp)

add_executable(position_controller src/position_controller.cpp)
ament_target_dependencies(position_controller rclcpp trajectory_msgs nav_msgs sensor_msgs 
ament_index_cpp)

add_executable(odom_subs src/subs.cpp)
ament_target_dependencies(odom_subs rclcpp trajectory_msgs nav_msgs ament_index_cpp)

add_executable(pid_controller src/PID.cpp)
ament_target_dependencies(pid_controller rclcpp trajectory_msgs nav_msgs ament_index_cpp)

target_link_libraries (odom_subs Eigen3::Eigen)
install(TARGETS

  pid_controller
  odom_subs
  trajectory_controller
  position_controller

  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY   matlab
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
