// ros_comm.cpp — RosComm implementation. See ros_comm.hpp for the design
// contract. The algorithm (ObsGenerator) is held as a member and only ever
// receives plain Input / writes plain Output — no rclcpp leakage either way.

#include "obs_generator/ros_comm.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace obs_generator
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

RosComm::RosComm(const rclcpp::NodeOptions & options)
: rclcpp::Node("obs_generator_laser", options)
{
  declareParameters();
  setupIO();

  RCLCPP_INFO(get_logger(),
              "obs_generator::RosComm up. scan=%s out=%s (algorithm fires at scan rate)",
              scan_topic_.c_str(), obstacle_topic_.c_str());
}

void RosComm::declareParameters()
{
  max_range_      = declare_parameter<double>("max_range",       40.0);
  max_gt_range_   = declare_parameter<double>("max_gt_range",     7.5);
  min_human_coef_ = declare_parameter<double>("min_human_coef",   0.25);
  is_humans_gt_   = declare_parameter<bool>(  "is_humans_gt",     true);
  laser_frame_    = declare_parameter<std::string>("laser_frame", "laser_frame");
  base_frame_     = declare_parameter<std::string>("base_frame",  "base_link");
  scan_topic_     = declare_parameter<std::string>("scan_topic",     "/scan");
  obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "/obstacles");
}

void RosComm::setupIO()
{
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RosComm::scanCallback, this, _1));

  // /apfrl_mu is the 4th action published by social_apfrl_node — it scales
  // the human-matched ellipse enlargement (input.human_coef in MatchToHuman /
  // modelObstaclesAsEllipses). Internal variable name stays `human_coef`
  // since that's what the value semantically *is*; the topic name reflects
  // its origin in the RL policy.
  coef_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/apfrl_mu", 10,
      std::bind(&RosComm::coefCallback, this, _1));

  obs_pub_ = create_publisher<apfrl_interfaces::msg::Obstacles>(
      obstacle_topic_, 1U);

  cmd_srv_ = create_service<apfrl_interfaces::srv::SendCommand>(
      "obs_generator_laser/send_command",
      std::bind(&RosComm::commandCallback, this, _1, _2));

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  transform_check_timer_ = create_wall_timer(
      300ms, std::bind(&RosComm::checkTransform, this));
}

// ── Subscriber callbacks ────────────────────────────────────────────────
// scanCallback parses + latches, then immediately drives tick() so the
// algorithm fires at the lidar's rate — same as the legacy code, no extra
// latency from the migration. The separation from the algorithm is
// structural (scan parsing in callback, math in tick()), not temporal.
//
// coefCallback only latches; the next scan triggers consumption.

void RosComm::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  if (!transform_ready_.load()) {
    return;  // ignore until laser→base TF is known; clamp doesn't make sense first
  }

  LaserParams params;
  params.angle_min       = scan_msg->angle_min;
  params.angle_max       = scan_msg->angle_max;
  params.angle_increment = scan_msg->angle_increment;
  params.range_max       = scan_msg->range_max;
  params.scan_size       = scan_msg->ranges.size();

  std::vector<double> scan(params.scan_size);
  for (size_t k = 0; k < params.scan_size; ++k) {
    const auto r = scan_msg->ranges[k];
    if (r > max_range_ || std::isnan(r) || r <= 0) {
      scan[k] = 2.0 * max_range_;   // legacy "no return" sentinel
    } else {
      scan[k] = r;
    }
  }

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    latched_.scan         = std::move(scan);
    latched_.laser_params = params;
    is_new_scan_.store(true);
  }

  tick();
}

void RosComm::coefCallback(std_msgs::msg::Float32::ConstSharedPtr coef_msg)
{
  std::lock_guard<std::mutex> lk(state_mutex_);
  latched_.human_coef = std::max(static_cast<double>(coef_msg->data),
                                 min_human_coef_);
}

// ── Algorithm tick ──────────────────────────────────────────────────────
// Driven by scanCallback. Pulls latched data, runs algorithm, publishes.

void RosComm::tick()
{
  Input  in {};
  Output out {};
  if (!GetData(in)) {
    return;  // nothing to consume
  }
  algo_.modelObstaclesAsEllipses(in, out.obstacles);
  Publish(out);
}

bool RosComm::GetData(Input & input)
{
  if (!is_new_scan_.exchange(false)) {
    return false;  // nothing new since the previous call
  }
  std::lock_guard<std::mutex> lk(state_mutex_);
  input.scan         = std::move(latched_.scan);
  input.laser_params = latched_.laser_params;
  input.humans       = latched_.humans;     // copy — small, infrequent
  input.human_coef   = latched_.human_coef;

  algo_.setLaserParams(input.laser_params);  // keep algo's cached params in sync
  return true;
}

// ── Publish ─────────────────────────────────────────────────────────────

void RosComm::Publish(const Output & output)
{
  apfrl_interfaces::msg::Obstacles msg;
  msg.header.stamp    = now();
  msg.header.frame_id = base_frame_;

  // Sentinel obstacle (legacy convention — APF planner expects index 0 as
  // a "dummy far-out" marker so downstream loops can skip it cheaply).
  apfrl_interfaces::msg::Obstacle sentinel;
  sentinel.pose.x  = -1.0;
  sentinel.pose.y  = 0.0;
  sentinel.pose.theta = 0.0;
  sentinel.r_a = 40.0;
  sentinel.r_b = 40.0;
  msg.obs.push_back(sentinel);

  // Algorithm worked in laser frame; transform to base before publishing.
  std::vector<Obstacle> in_base = output.obstacles;
  transformObstaclesToBase(in_base, laser_to_base_);

  for (const auto & o : in_base) {
    apfrl_interfaces::msg::Obstacle ob;
    ob.pose.x     = o.x;
    ob.pose.y     = o.y;
    ob.pose.theta = o.yaw;
    ob.r_a        = std::max(o.r_a, 0.1);
    ob.r_b        = std::max(o.r_b, 0.1);
    msg.obs.push_back(ob);
  }
  obs_pub_->publish(msg);
}

void RosComm::transformObstaclesToBase(std::vector<Obstacle> & obstacles,
                                       const tf2::Transform & transform) const
{
  for (auto & o : obstacles) {
    tf2::Vector3 p(o.x, o.y, 0.0);
    p = transform * p;
    o.x = p.getX();
    o.y = p.getY();
    // Yaw is not rotated (laser-aligned ellipses on a flat base assumption,
    // matches the legacy behaviour).
  }
}

// ── TF readiness ────────────────────────────────────────────────────────
// Async wait pattern (same as apf/apfrl). Fires every 300 ms until the
// laser→base transform is available; then caches it and cancels itself.

void RosComm::checkTransform()
{
  if (transform_ready_.load()) return;

  if (!tf_buffer_->canTransform(base_frame_, laser_frame_, tf2::TimePointZero)) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Waiting for transform %s → %s",
                          laser_frame_.c_str(), base_frame_.c_str());
    return;
  }

  try {
    auto t = tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero);
    tf2::fromMsg(t.transform, laser_to_base_);
    transform_ready_.store(true);
    transform_check_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Got %s → %s transform",
                laser_frame_.c_str(), base_frame_.c_str());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Transform lookup failed: %s", ex.what());
  }
}

// ── Service: runtime command (matches legacy SendCommand contract) ─────

void RosComm::commandCallback(
    const std::shared_ptr<apfrl_interfaces::srv::SendCommand::Request> req,
    std::shared_ptr<apfrl_interfaces::srv::SendCommand::Response> res)
{
  RCLCPP_INFO(get_logger(), "Command received: %d", req->command);
  switch (req->command) {
    case 1:
      // Reset: drop any latched human/coef state. Scan refills on next tick.
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        latched_.humans.clear();
        latched_.human_coef = min_human_coef_;
      }
      res->success = true;
      res->message = "obs_generator state reset";
      break;
    default:
      res->success = false;
      res->message = "unknown command";
  }
}

}  // namespace obs_generator

RCLCPP_COMPONENTS_REGISTER_NODE(obs_generator::RosComm)
