#ifndef APF_ROS_COMM_H
#define APF_ROS_COMM_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>
#include <mutex>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "apfrl_interfaces/msg/obstacles.hpp"
#include "apfrl_interfaces/srv/send_command.hpp"
#include "apfrl_interfaces/msg/navigation_status.hpp"
#include "apfrl_interfaces/action/navigate_to_pose.hpp"
#include "apf/apf_ext.hpp"
#include "data_structures.h"
#include "apfrl_interfaces/commands.h"

using namespace std::chrono_literals;

class RosComm : public rclcpp::Node
{
public:
  explicit RosComm(double rate);
  void PublishVelocity(const Velocity &velocity);
  void PublishNavigationStatus(int status, double distance_to_target, const std::string &message = "");
  bool GetData(Input &input);
  // void getMinBetaCurrentFromAPF(double minBetaCurrent);
  // void PublishMinBetaCurrent();
  double xScale;

private:
  void ObsCallback(const apfrl_interfaces::msg::Obstacles::SharedPtr obs_msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void TargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg);
  void ApfKCallback(const std_msgs::msg::Float32::SharedPtr k_msg);
  void ControlTimerCallback();
  // void ApfBetaKCallback(const std_msgs::Float32ConstPtr &betak_msg);
  void CommandCallback(
      const std::shared_ptr<apfrl_interfaces::srv::SendCommand::Request> request,
      std::shared_ptr<apfrl_interfaces::srv::SendCommand::Response> response);
  void executeNavigateToPose(std::shared_ptr<rclcpp_action::ServerGoalHandle<apfrl_interfaces::action::NavigateToPose>> goal_handle);

  void TransformPose(Pose &pose, std::string target_frame);
  // Async TF wait: a timer polls for base_link<->odom and sets transform_ready_
  // when found, then cancels itself. Replaces the old blocking wait_transform()
  // which deadlocked because the executor was not yet spinning during ctor.
  void checkTransform();

  rclcpp::Subscription<apfrl_interfaces::msg::Obstacles>::SharedPtr obs_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr apfk_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr transform_check_timer_;
  std::atomic<bool> transform_ready_{false};
  // ros::Subscriber apf_betak_sub_;
  // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr minBetaCurrent_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<apfrl_interfaces::msg::NavigationStatus>::SharedPtr nav_status_pub_;

  rclcpp::Service<apfrl_interfaces::srv::SendCommand>::SharedPtr apf_service_;
  rclcpp_action::Server<apfrl_interfaces::action::NavigateToPose>::SharedPtr navigate_to_pose_server_;
  rclcpp::Client<apfrl_interfaces::srv::SendCommand>::SharedPtr navigatior_client_;

  Input apf_data_;
  Input apf_prev_data_;

  rclcpp::Rate rate_;
  bool is_new_target_;
  bool is_new_obs_;
  bool is_got_target_;
  // Opt-in gate on /apf/enable, default true so nothing changes for anyone
  // who never publishes it. The trainer holds it false across reset: a
  // teleport does not zero wheel velocity, and this node would otherwise
  // keep steering towards the finished episode's goal while the env tries
  // to brake, so an episode began with up to 0.79 m/s of inherited motion
  // and 0.25 m off its spawn cell.
  std::atomic<bool> enabled_{true};
  // Set while the latest /goal is apfrl's stop, a waypoint at the robot's own
  // origin in base_link: the controller then commands zero until the next
  // waypoint instead of steering back to where that stop was issued.
  std::atomic<bool> holding_{false};
  bool is_new_odom_;
  std::atomic<bool> stop_requested_{false};
  std::mutex execution_mutex_;

  // Shared target for runtime updates (from navigator SET_TARGET command)
  Pose shared_map_target_;
  std::atomic<bool> target_updated_{false};
  std::mutex target_mutex_;

  double apf_only_k_coef_;
  // Bounds applied to apf_k whatever its source. The upper one used to be a
  // hardcoded 5.0, which capped both the count-based heuristic and the policy.
  double apf_k_min_{1.0};
  double apf_k_max_{20.0};
  // double minBetaCurrent_;
  double minGamma_;
  double goal_reach_threshold_;
  double control_period_;
  bool verbose_;
  std::string model_path;
  geometry_msgs::msg::TransformStamped odom_transform_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  APF_EXT apf_;
};

#endif // APF_ROS_COMM_H
