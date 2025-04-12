cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED PATHS /home/pich/uav_ws/install/px4_msgs)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(/usr/include/eigen3)


add_executable(ekf_filter src/ekf_filter.cpp)
add_executable(Joy_stick_control src/Joy_stick_control.cpp)
add_executable(D_star src/D_star.cpp)
add_executable(Trajectory_controll src/Trajectory_controll.cpp)
ament_target_dependencies(ekf_filter rclcpp sensor_msgs px4_msgs geometry_msgs)
ament_target_dependencies(Joy_stick_control rclcpp sensor_msgs px4_msgs geometry_msgs)
ament_target_dependencies(D_star rclcpp px4_msgs geometry_msgs visualization_msgs nav_msgs std_srvs)
ament_target_dependencies(Trajectory_controll rclcpp px4_msgs geometry_msgs visualization_msgs nav_msgs std_srvs)


install(TARGETS ekf_filter Joy_stick_control D_star Trajectory_controll
          DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME})
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
