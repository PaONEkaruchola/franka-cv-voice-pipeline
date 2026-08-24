cmake_minimum_required(VERSION 3.8)
project(my_panda_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(panda_kinematics src/panda_kinematics.cpp)
ament_target_dependencies(panda_kinematics rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS panda_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(grasping_logic src/grasping_logic.cpp)
target_include_directories(grasping_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(grasping_logic PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  grasping_logic
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "geometry_msgs"
  "sensor_msgs"
)

install(DIRECTORY
  launch
  urdf
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS grasping_logic
  DESTINATION lib/${PROJECT_NAME})

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
