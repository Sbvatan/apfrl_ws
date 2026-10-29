cmake_minimum_required(VERSION 3.10)
project(apf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
include_directories(
  include  
  ${ament_INCLUDE_DIRS}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(apfrl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apfrl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Compares the two gradPhi formulations through the real calculateVel path,
# over the obstacle counts and apf_k range this robot actually sees. Not a node:
# run it directly to see where the analytic form stops being usable.
find_package(Eigen3 REQUIRED)
add_executable(k_needed test/k_needed.cpp src/apf_ext.cpp)
target_include_directories(k_needed PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(k_needed rclcpp)
install(TARGETS k_needed DESTINATION lib/${PROJECT_NAME})

add_executable(k_sweep test/k_sweep.cpp src/apf_ext.cpp)
target_include_directories(k_sweep PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(k_sweep rclcpp)
install(TARGETS k_sweep DESTINATION lib/${PROJECT_NAME})

add_executable(grad_equiv test/grad_equiv.cpp src/apf_ext.cpp)
target_include_directories(grad_equiv PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(grad_equiv rclcpp)
install(TARGETS grad_equiv DESTINATION lib/${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/main.cpp src/apf_ext.cpp src/ros_comm.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  rclcpp_action
  std_msgs
  apfrl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  apfrl_interfaces
  nav_msgs
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

install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
 
ament_package()
