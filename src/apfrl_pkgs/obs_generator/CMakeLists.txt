cmake_minimum_required(VERSION 3.5)
project(obs_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apfrl_interfaces REQUIRED)

# ── Single library: algorithm + RosComm wrapper. Used by both the
# composable component and the standalone executable. ──────────────────
add_library(${PROJECT_NAME}_component SHARED
  src/ros_comm.cpp
  src/obs_generator.cpp
  src/convexhull.cpp
  src/ellipse_fit.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp rclcpp_components
  std_msgs geometry_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs
  apfrl_interfaces)

# Composable component registration. rclcpp_components_register_node also
# generates a thin executable named `<pkg>_laser_node` for backwards-compat
# launches that expect that name; the standalone exe below is the canonical
# entry point.
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN     "obs_generator::RosComm"
  EXECUTABLE ${PROJECT_NAME}_laser_node)

# Standalone executable (canonical) — same RosComm, plain rclcpp::spin.
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

# ── Install ─────────────────────────────────────────────────────────────
install(TARGETS
    ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_component)
ament_export_dependencies(
  rclcpp rclcpp_components
  std_msgs geometry_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs
  apfrl_interfaces)

ament_package()
