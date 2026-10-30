#ifndef APFRL_ROS_COMM_H
#define APFRL_ROS_COMM_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <apfrl_interfaces/msg/obstacles.hpp>
#include "data_structures.h"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "apfrl_interfaces/srv/send_command.hpp"
#include "apfrl_interfaces/msg/navigation_status.hpp"
#include "apfrl_interfaces/action/navigate_to_pose.hpp"
#include "apfrl_interfaces/commands.h"
#include <mutex>
#include <atomic>
// No APF here by design. This node runs the learned policy and publishes its
// waypoint on /goal and its gain on /apf_k; apf_node (package `apf`) owns the
// potential field and the velocity. That is the topology every policy in this
// workspace was trained under, and the one social_apfrl already used. A
// duplicate APF_EXT used to live in this package (added by 9b73dc07 during the
// ROS 2 learning merge, identical math to apf/'s copy but with setParams
// stripped, so its envelope was compile-time and drifted from training).
#include "apfrl/apfrl.h"

#if XAVIER
#include <zed_interfaces/msg/objects_stamped.hpp>
#endif

using namespace std::chrono_literals;

class RosComm : public rclcpp::Node 
{
public:
  explicit RosComm();
  void Publish(const Output &output);
  void PublishNavigationStatus(int status, double distance_to_target, const std::string& message = "");
  bool GetData(SocialInput &input);

  std::string model_path_;
  double rate;
  // Distance to the action goal at which the navigation action succeeds and the
  // robot is told to hold. Keep it >= ApfRl::kReachDistance (0.2 m): inside that
  // radius ApfRl::Step already outputs the stop waypoint, so the robot parks
  // there and a smaller threshold is never reached -- the goal never finishes.
  double goal_reach_threshold_;

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr apfk_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wp_pub_;
  rclcpp::Publisher<apfrl_interfaces::msg::NavigationStatus>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr current_apf_goal_pub_;
  rclcpp::Subscription<apfrl_interfaces::msg::Obstacles>::SharedPtr humans_sub_;

  rclcpp::Client<apfrl_interfaces::srv::SendCommand>::SharedPtr navigatior_client_;
  rclcpp::Service<apfrl_interfaces::srv::SendCommand>::SharedPtr apfrl_service_;

  rclcpp_action::Server<apfrl_interfaces::action::NavigateToPose>::SharedPtr navigate_to_pose_server_;

  std::shared_ptr<ApfRl> apfrl_;

  SocialInput apfrl_input_;
  SocialInput prev_apfrl_input_;

  std::unique_ptr<rclcpp::Rate> rate_;
  std::mutex data_mutex_;  // protects apfrl_input_, prev_*, is_new_* flags
  bool is_new_target_;
  bool is_new_k_;
  bool is_new_apf_target_;
  bool is_new_scan_;
  bool is_new_odom_;
  bool is_got_target_;
  bool is_apf_got_target_;
  bool is_new_humans_;
  bool is_new_obs_;

  // Hands the policy's waypoint + gain to apf_node. Called at policy rate.
  void PublishWaypoint(double wp_x, double wp_y, double apf_k);

  // Shared target for runtime updates (from navigator SET_TARGET command)
  Pose shared_map_target_;
  std::atomic<bool> target_updated_{false};
  std::mutex target_mutex_;
  std::atomic<bool> stop_requested_{false};
  std::mutex execution_mutex_;

  geometry_msgs::msg::TransformStamped odom_transform_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr transform_check_timer_;
  std::atomic<bool> transform_ready_{false};

  // Matches APFRL_env_Ata.py laser_max. NOT the lidar's <max>: the sensor reads
  // to 150 m, but the policy only ever saw returns from inside a 21 m training
  // room, so anything beyond one room diagonal must be clipped to stay in the
  // distribution it was fitted on.
  // Runtime, not compile-time: the clamp belongs to the policy's observation
  // contract, and the two profiles disagree (28 m for BARN, 150 m for the
  // pre-BARN weights). Set once from the obs_profile parameter via
  // ApfRl::MaxRangeFor, so the clamp and the normaliser can never drift apart.
  double max_range_;
  static constexpr double kMaxHumanRange = 7.0;

  void TargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg);
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  #if XAVIER
  void HumansCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr humans_msg);
  #else
  void HumansCallback(const apfrl_interfaces::msg::Obstacles::SharedPtr humans_msg);
  #endif
  void TransformPose(Pose& pose, std::string target_frame);
  // Async TF wait — same rationale as apf::checkTransform.
  void checkTransform();
  void CommandCallback(
    const std::shared_ptr<apfrl_interfaces::srv::SendCommand::Request> request,
    std::shared_ptr<apfrl_interfaces::srv::SendCommand::Response> response);
  void executeNavigateToPose(std::shared_ptr<rclcpp_action::ServerGoalHandle<apfrl_interfaces::action::NavigateToPose>> goal_handle);

};


#endif // APFRL_ROS_COMM_H
