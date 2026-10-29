#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "data_structures.h"
#include "social_apfrl/ros_comm.h"
#include "social_apfrl/social_apfrl.h"
#include <string>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ros_comm = std::make_shared<RosComm>();
  SocialApfRl apf_rl(ros_comm->model_path_);
  // Inference loop runs at the rate parameter exposed by ros_comm.
  // Default 3 Hz if unset, matching the legacy training-time cadence.
  rclcpp::Rate loop_rate(ros_comm->rate > 0.0 ? ros_comm->rate : 3.0);
  while (rclcpp::ok()) {
    SocialInput input{};
    SocialOutput output{};
    if (ros_comm->GetData(input)) {
      apf_rl.Step(input, output);
      ros_comm->Publish(output);
    }
    rclcpp::spin_some(ros_comm);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
