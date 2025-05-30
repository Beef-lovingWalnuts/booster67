cmake_minimum_required(VERSION 3.8)
project(booster_nav2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 添加SDK头文件路径
include_directories(
  include
  /home/booster/Workspace/boosterxjw/booster_robotics_sdk_release/include
  # 替换为实际SDK头文件路径
)

# 添加可执行文件
add_executable(booster_nav2_example
  src/booster_nav2_example.cpp
)

# 链接库
target_link_libraries(booster_nav2_example
  ${rclcpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  # booster_robot_sdk::b1_loco_client
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
