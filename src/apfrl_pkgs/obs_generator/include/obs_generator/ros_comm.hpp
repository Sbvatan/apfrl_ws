// ros_comm.hpp — ROS2 wrapper for the obs_generator algorithm.
//
// Pattern (matches docs/Yunus's RosComm spec, ROS2-translated):
//   * RosComm derives from rclcpp::Node and owns *all* I/O — subs, pubs,
//     services, tf, params. Composable + standalone via the same class.
//   * Algorithm is held as a member of type ObsGenerator (rclcpp-free).
//   * Trigger: the scan callback latches data, then calls a private tick():
//       Input  in {};
//       Output out {};
//       if (GetData(in)) { algo_.modelObstaclesAsEllipses(in, out.obstacles); Publish(out); }
//     This preserves the legacy scan-rate processing — no extra latency
//     introduced by the migration. The separation is structural: scan
//     callback only parses the message and latches; algorithm invocation
//     lives in tick(), not in the callback.
//   * GetData() returns true only when a fresh scan arrived since the last
//     consumption.
//
// SOLID:
//   SRP — RosComm = ROS plumbing only; ObsGenerator = math only.
//   ISP — public surface is just GetData/Publish + ctor.
//   DIP — algorithm depends on plain Input/Output structs, never on rclcpp.

#ifndef OBS_GENERATOR_ROS_COMM_HPP_
#define OBS_GENERATOR_ROS_COMM_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <apfrl_interfaces/msg/obstacles.hpp>
#include <apfrl_interfaces/srv/send_command.hpp>

#include "obs_generator/data_structures.h"
#include "obs_generator/obs_generator.h"

namespace obs_generator
{

class RosComm : public rclcpp::Node
{
public:
  /// NodeOptions ctor — required by rclcpp_components for the composable
  /// build target. The standalone main wraps this with the default options.
  explicit RosComm(const rclcpp::NodeOptions & options);

  /// Latch the next algorithm input from the most-recent scan callback.
  /// Returns true when there is fresh data to consume; false when nothing
  /// has arrived since the last call.
  bool GetData(Input & input);

  /// Transform algorithm output to base_frame and publish on the
  /// configured obstacle topic.
  void Publish(const Output & output);

private:
  // ── Lifecycle helpers ────────────────────────────────────────────────
  void declareParameters();
  void setupIO();

  // ── Callbacks ────────────────────────────────────────────────────────
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
  void coefCallback(std_msgs::msg::Float32::ConstSharedPtr coef_msg);
  // tick() = GetData → algo → Publish. Driven by scanCallback so the
  // algorithm fires at the lidar's rate (matches legacy behaviour).
  void tick();
  void checkTransform();
  void commandCallback(
      const std::shared_ptr<apfrl_interfaces::srv::SendCommand::Request> req,
      std::shared_ptr<apfrl_interfaces::srv::SendCommand::Response> res);

  // ── Helpers ──────────────────────────────────────────────────────────
  void transformObstaclesToBase(std::vector<Obstacle> & obstacles,
                                const tf2::Transform & transform) const;

  // ── Parameters ───────────────────────────────────────────────────────
  double      max_range_{40.0};
  double      max_gt_range_{7.5};
  double      min_human_coef_{0.25};
  bool        is_humans_gt_{true};
  std::string laser_frame_{"laser_frame"};
  std::string base_frame_{"base_link"};
  std::string scan_topic_{"/scan"};
  std::string obstacle_topic_{"/obstacles"};

  // ── Subs / pubs / services / timers ──────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       coef_sub_;
  rclcpp::Publisher<apfrl_interfaces::msg::Obstacles>::SharedPtr obs_pub_;
  rclcpp::Service<apfrl_interfaces::srv::SendCommand>::SharedPtr cmd_srv_;
  rclcpp::TimerBase::SharedPtr transform_check_timer_;

  // ── TF (laser → base) ────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
  std::atomic<bool>                            transform_ready_{false};
  tf2::Transform                               laser_to_base_;

  // ── Latched input state (mutated from callbacks, drained by GetData) ─
  // Mutex-protected because scan_sub_ + coef_sub_ may dispatch on different
  // executor threads under MultiThreadedExecutor.
  std::mutex          state_mutex_;
  Input               latched_;
  std::atomic<bool>   is_new_scan_{false};

  // ── Algorithm (pure C++, no rclcpp) ──────────────────────────────────
  ObsGenerator algo_;
};

}  // namespace obs_generator

#endif  // OBS_GENERATOR_ROS_COMM_HPP_
