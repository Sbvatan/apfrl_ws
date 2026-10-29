#ifndef SOCIAL_APFRL_ROS_COMM_H
#define SOCIAL_APFRL_ROS_COMM_H

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <apfrl_interfaces/msg/obstacles.hpp>
#include "data_structures.h"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#if XAVIER
#include <zed_interfaces/msg/objects_stamped.hpp>
#endif

using namespace std::placeholders;

class RosComm : public rclcpp::Node 
{
public:
  explicit RosComm();
  void Publish(const SocialOutput &output);
  bool GetData(SocialInput &input);
  std::string model_path_;
  double rate;
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr apfk_pub_;
  // 4th action — drives obstacle-enlargement-factor on the planner side via
  // obs_generator's human_coef sub. See SocialOutput::mu in data_structures.h.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mu_pub_;
  // Deprecated 4th action — kept publishing zero so subscribers still listening
  // on the old topic don't see a missing publisher. To be reused later.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr apf_betak_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr wp_pub_;
  rclcpp::Subscription<apfrl_interfaces::msg::Obstacles>::SharedPtr humans_sub_;
  SocialInput data_;
  SocialInput prev_data_;
  rclcpp::Rate *rate_;
  bool is_new_target_;
  bool is_new_scan_;
  bool is_new_odom_;
  bool is_got_target_;
  bool is_new_humans_;
  geometry_msgs::msg::TransformStamped odom_transform_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr transform_check_timer_;
  std::atomic<bool> transform_ready_{false};

  // Matches SocialAPFRL_env_Ata.py laser_max, and the lidar's <max>.
  static constexpr double kMaxRange = 150.0;
  static constexpr double kMaxHumanRange = 7.0;

  void TargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg);
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  #if XAVIER
  void HumansCallback(const zed_interfaces::ObjectsStampedConstPtr &humans_msg);
  #else
  void HumansCallback(const apfrl_interfaces::msg::Obstacles::SharedPtr humans_msg);
  #endif
  void TransformPose(Pose& pose, std::string target_frame);
  // Async TF wait — same rationale as apf::checkTransform.
  void checkTransform();
};

#endif // SOCIAL_APFRL_ROS_COMM_H
