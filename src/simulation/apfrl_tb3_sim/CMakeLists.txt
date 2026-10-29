cmake_minimum_required(VERSION 3.8)
project(apfrl_tb3_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/gt_odom_gz.py scripts/rviz_goal_bridge.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
