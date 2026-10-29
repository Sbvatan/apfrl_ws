cmake_minimum_required(VERSION 3.8)
project(apfrl)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 11)
endif()

set(CMAKE_CXX_FLAGS "-D_GLIBCXX_USE_CXX11_ABI=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(CMAKE_PREFIX_PATH $ENV{TORCH_DIR})
set(CUDA_INCLUDE_DIRS "${CUDA_HOME}/include")
set(Torch_PATH  "$ENV{TORCH_DIR}"/share/cmake/Torch)
message($ENV{TORCH_DIR})
message(${Torch_PATH})
# find dependencies
if ($ENV{USER} STREQUAL "xavier")
  add_definitions(-DXAVIER=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CUDA REQUIRED)
find_package(Torch REQUIRED PATHS ${Torch_PATH})
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(apfrl_interfaces REQUIRED)
find_package(zed_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)



add_executable(${PROJECT_NAME}_node
src/apfrl/main.cpp 
src/apfrl/apfrl.cpp 
src/apfrl/ros_comm.cpp 
src/metrics_calculator.cpp
)
add_executable(social_${PROJECT_NAME}_node src/social_apfrl/main.cpp src/social_apfrl/social_apfrl.cpp src/social_apfrl/ros_comm.cpp src/metrics_calculator.cpp)
add_executable(human_goal_pub_node src/human_goal_pub.cpp)



ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  tf2
  Torch
  tf2_ros
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  apfrl_interfaces
  zed_msgs
)

ament_target_dependencies(social_${PROJECT_NAME}_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  Torch
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  apfrl_interfaces
  zed_msgs
)

ament_target_dependencies(human_goal_pub_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  apfrl_interfaces
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
  zed_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  "${TORCH_LIBRARIES}"
)

target_link_libraries(social_${PROJECT_NAME}_node
  "${TORCH_LIBRARIES}"
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${TORCH_INCLUDE_DIRS}
)

target_include_directories(social_${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${TORCH_INCLUDE_DIRS}
)

target_include_directories(human_goal_pub_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# target_include_directories(${PROJECT_NAME}_node PUBLIC
#   ${HOME}/Libraries/libtorch/lib/
# )

# target_include_directories(social_${PROJECT_NAME}_node PUBLIC
#   ${HOME}/Libraries/libtorch/lib/
# )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(TARGETS ${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME})


install(TARGETS social_${PROJECT_NAME}_node
DESTINATION lib/${PROJECT_NAME})

install(TARGETS human_goal_pub_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
launch
models
rviz
DESTINATION share/${PROJECT_NAME}/)

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
