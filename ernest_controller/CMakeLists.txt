cmake_minimum_required(VERSION 3.5)
project(ernest_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(ernest_controller SHARED
  src/ernest_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_include_directories(ernest_controller PRIVATE include)
ament_target_dependencies(ernest_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ernest_controller PRIVATE "ERNEST_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(ernest_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface ernest_plugin.xml)

add_executable(joint_controller src/joint_traj_controller.cpp)
ament_target_dependencies(joint_controller rclcpp trajectory_msgs)

install(TARGETS
  joint_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY  launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ernest_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)

  ament_add_gmock(test_ernest_controller
    test/test_ernest_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_ernest_controller.yaml)
  target_include_directories(test_ernest_controller PRIVATE include)
  target_link_libraries(test_ernest_controller
    ernest_controller
  )

  ament_target_dependencies(test_ernest_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(
    test_load_ernest_controller
    test/test_load_ernest_controller.cpp
  )
  target_include_directories(test_load_ernest_controller PRIVATE include)
  ament_target_dependencies(test_load_ernest_controller
    controller_manager
  )
  target_compile_definitions(test_load_ernest_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  ament_add_gmock(
    test_accumulator
    test/test_accumulator.cpp
  )

  target_include_directories(test_accumulator PRIVATE include)
  ament_target_dependencies(test_accumulator)

endif()

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ernest_controller
)
ament_package()
