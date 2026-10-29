cmake_minimum_required(VERSION 3.8)
project(apfrl_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # APF / RL planning
  "msg/Obstacle.msg"
  "msg/Obstacles.msg"

  # Sim actor follow goals (consumed by apfrl planner / training)
  "msg/FollowGoal.msg"
  "msg/FollowGoals.msg"

  # Navigation command bus (absorbed from bagrob_interfaces)
  "msg/NavigationStatus.msg"
  "srv/SendCommand.srv"
  "action/NavigateToPose.action"

  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install shared command/parameter headers used across navigation nodes
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
