#include <algorithm>
#include <cmath>

#include "social_apfrl/ros_comm.h"
#include <chrono>

using namespace std::chrono_literals;

RosComm::RosComm() : Node("social_apfrl"), data_(), is_new_target_(false), is_new_scan_(false), is_new_odom_(false), is_got_target_(false)
{
  this->declare_parameter("model_path","");
  this->declare_parameter("rate",0.0);
  this->get_parameter("model_path",model_path_); 
  this->get_parameter("rate",rate);

  rate_ = new rclcpp::Rate(rate); 
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1U, std::bind(&RosComm::TargetCallback, this,_1));
  // SensorDataQoS, not the default. /scan is published BEST_EFFORT, as sensor
    // streams are; a RELIABLE subscriber is incompatible and receives nothing.
    // rclcpp logs "offering incompatible QoS, no messages will be sent to it"
    // once and carries on, so the planner just sits there with an empty scan.
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&RosComm::ScanCallback, this,_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1U, std::bind(&RosComm::OdomCallback, this,_1));
#if XAVIER
  humans_sub_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>("/humans", 1U, std::bind(&RosComm::HumansCallback, this,_1));
#else
  humans_sub_ =this->create_subscription<apfrl_interfaces::msg::Obstacles>("/humans", 1U, std::bind(&RosComm::HumansCallback, this,_1));
#endif
  wp_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 1U);
  apfk_pub_ = this->create_publisher<std_msgs::msg::Float32>("/apf_k", 1U);
  // 4th action — current convention. Drives the obstacle-enlargement-factor
  // function on the planner side.
  mu_pub_ = this->create_publisher<std_msgs::msg::Float32>("/apfrl_mu", 1U);
  // Deprecated 4th action — kept publishing zero so subscribers that still
  // listen on the old topic don't see a missing publisher. Reusable later.
  apf_betak_pub_ =this->create_publisher<std_msgs::msg::Float32>("/apf_beta_k", 1U);
  prev_data_ = {};
  // Async TF wait — see apf::checkTransform for rationale.
  transform_check_timer_ = this->create_wall_timer(
      300ms, std::bind(&RosComm::checkTransform, this));
}

#if XAVIER
void RosComm::HumansCallback(const zed_interfaces::ObjectsStampedConstPtr &humans_msg)
{
  data_.humans = {};
  prev_data_.humans = {};
  auto frame = humans_msg->header.frame_id;
  for (auto human : humans_msg->objects)
  {
    Pose temp_human{};
    temp_human.x = human.position[0];
    temp_human.y = human.position[1];
    temp_human.frame = frame;
    prev_data_.humans.push_back(temp_human);
    TransformPose(temp_human, "base_link");

    data_.humans.push_back(temp_human);
  }
  is_new_humans_ = true;
}
#else
void RosComm::HumansCallback(const apfrl_interfaces::msg::Obstacles::SharedPtr humans_msg)
{
  data_.humans = {};
  prev_data_.humans = {};
  auto frame = humans_msg->header.frame_id;
  for (auto human : humans_msg->obs)
  {
    Pose temp_human{};
    temp_human.x = human.pose.x;
    temp_human.y = human.pose.y;

    temp_human.frame = frame;
    prev_data_.humans.push_back(temp_human);
    TransformPose(temp_human, "base_link");
    auto dist = sqrt(temp_human.x * temp_human.x + temp_human.y * temp_human.y);
    if (dist > kMaxHumanRange)
    {
      continue;
    }
    data_.humans.push_back(temp_human);
  }
  is_new_humans_ = true;
}
#endif

bool RosComm::GetData(SocialInput &input)
{
  // Refuse to deliver data until TF is ready — GetData calls TransformPose
  // (target & humans), which would silently no-op on TF failure and produce
  // wrong control commands.
  if (!transform_ready_.load()) {
    return false;
  }
  input = {};
  if (is_new_scan_)
  {
    prev_data_.scan = data_.scan;
    input.scan = data_.scan;
    is_new_scan_ = false;
  }
  else
  {
    input.scan = prev_data_.scan;
  }

  if (is_new_odom_)
  {
    input.robot_pose = data_.robot_pose;
    prev_data_.robot_pose = data_.robot_pose;
    is_new_odom_ = false;
  }
  else
  {
    input.robot_pose = prev_data_.robot_pose;
  }
  if (is_new_target_)
  {
    // To keep it in odom frame, if given in base_link frame
    TransformPose(data_.target, "odom");
    prev_data_.target = data_.target;
    TransformPose(data_.target, "base_link");
    input.target = data_.target;
    is_new_target_ = false;
  }
  else
  {
    if (!is_got_target_)
    {
      data_ = {};
      return false;
    }
    input.target = prev_data_.target;
    TransformPose(input.target, "base_link");
  }

  if (is_new_humans_)
  {
    input.humans = data_.humans;
    is_new_humans_ = false;
  }
  else
  {
    input.humans = prev_data_.humans;
    for (auto &human : input.humans)
    {
      TransformPose(human, "base_link");
    }
  }
  data_ = {};
  return true;
}
void RosComm::Publish(const SocialOutput &output)
{
  geometry_msgs::msg::PoseStamped wp_msg{};
  std_msgs::msg::Float32 k_msg{};
  std_msgs::msg::Float32 mu_msg{};
  std_msgs::msg::Float32 beta_k_msg{};
  k_msg.data      = output.apf_k;
  mu_msg.data     = output.mu;
  beta_k_msg.data = output.apf_beta_k;   // zero in the current trainer
  wp_msg.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  wp_msg.header.frame_id = "base_link";
  wp_msg.pose.position.x = output.wp_x;
  wp_msg.pose.position.y = output.wp_y;

  apfk_pub_->publish(k_msg);
  mu_pub_->publish(mu_msg);
  apf_betak_pub_->publish(beta_k_msg);
  wp_pub_->publish(wp_msg);

  if (output.status != NavState::RESUME && output.status != NavState::ALMOST)
  {
    is_got_target_ = false;
  }
  rate_->sleep();
}

void RosComm::TargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target_msg)
{
  data_.target.x = target_msg->pose.position.x;
  data_.target.x = target_msg->pose.position.x;
  data_.target.y = target_msg->pose.position.y;
  tf2::Quaternion q(
      target_msg->pose.orientation.x,
      target_msg->pose.orientation.y,
      target_msg->pose.orientation.z,
      target_msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  data_.target.yaw = yaw;
  data_.target.frame = target_msg->header.frame_id;
  is_new_target_ = true;
  is_got_target_ = true;
}

void RosComm::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  const auto &ranges = scan_msg->ranges;
  const std::size_t n = ranges.size();
  if (n == 0U) {
    return;                           // nothing to downsample; keep the last scan
  }
  data_.scan.resize(SOCIAL_STATE_SCAN_SIZE);
  // Same defect as the apfrl callback, and worse here: the old loop ran
  // SCAN_SIZE/SOCIAL_SCAN_STEP = 200 iterations while writing into a 64-element
  // vector, so it overran the destination by 136 elements as well as reading past
  // the end of the message.
  for (std::size_t k = 0U; k < SOCIAL_STATE_SCAN_SIZE; ++k)
  {
    const std::size_t i = std::min(k * n / SOCIAL_STATE_SCAN_SIZE, n - 1U);
    const auto range = ranges[i];
    data_.scan[k] = (range > kMaxRange || std::isnan(range)) ? kMaxRange : range;
  }
  // Only for segway!!!
  (void)std::rotate(data_.scan.begin(), data_.scan.begin() + 64 / 2, data_.scan.end());
  is_new_scan_ = true;
}

void RosComm::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  tf2::Quaternion q(
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z,
      odom_msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  data_.robot_pose.x = odom_msg->pose.pose.position.x;
  data_.robot_pose.y = odom_msg->pose.pose.position.y;
  data_.robot_pose.yaw = yaw;
  is_new_odom_ = true;
}

void RosComm::checkTransform()
{
  if (transform_ready_.load()) {
    return;
  }
  if (tf_buffer_->canTransform("base_link", "odom", tf2::TimePointZero)) {
    try {
      odom_transform_ = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
      transform_ready_.store(true);
      transform_check_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "TF base_link<->odom obtained, planner ready");
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup transient failure: %s", ex.what());
    }
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for base_link<->odom transform...");
  }
}

void RosComm::TransformPose(Pose& pose, std::string target_frame) {
  odom_transform_ = tf_buffer_->lookupTransform(target_frame, pose.frame, tf2::TimePointZero); 
  tf2::Vector3 current_position(pose.x, pose.y, 0.0);
  tf2::Stamped<tf2::Transform> transform;
  tf2::fromMsg(odom_transform_, transform);
  //current_position = odom_transform_(current_position);
  current_position = transform(current_position);
  pose.x = current_position.getX();
  pose.y = current_position.getY();
  pose.frame = target_frame;
}