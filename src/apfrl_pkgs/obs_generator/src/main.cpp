// main.cpp — standalone executable for obs_generator.
//
// The same RosComm class is registered as a composable component in
// ros_comm.cpp (RCLCPP_COMPONENTS_REGISTER_NODE), so the only thing this
// executable adds is the rclcpp lifecycle. Everything (timer, tf wait,
// algorithm tick) lives inside the Node.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "obs_generator/ros_comm.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<obs_generator::RosComm>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
