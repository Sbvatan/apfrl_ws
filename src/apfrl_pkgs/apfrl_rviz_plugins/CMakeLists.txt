cmake_minimum_required(VERSION 3.8)
project(apfrl_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Qt MOC for the Q_OBJECT/Q_SLOTS in ObstaclesDisplay
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(apfrl_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/obstacles_display.hpp
  include/${PROJECT_NAME}/obstacles_visual.hpp
)

set(${PROJECT_NAME}_SOURCES
  src/obstacles_display.cpp
  src/obstacles_visual.cpp
)

# Plugin shared library — name MUST match the <library path="..."> attribute
# in rviz_plugin_description.xml so pluginlib can find it.
add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_HEADERS}
  ${${PROJECT_NAME}_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt5::Widgets
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  apfrl_interfaces
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  tf2
  tf2_ros
)

# Export plugin XML so RViz can discover this library at runtime.
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES rviz_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  apfrl_interfaces
  pluginlib
  rviz_common
  rviz_default_plugins
  rviz_rendering
)

ament_package()
