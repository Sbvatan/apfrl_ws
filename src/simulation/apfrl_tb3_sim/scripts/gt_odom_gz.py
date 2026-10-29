#!/usr/bin/env python3
"""Ground-truth odometry for the TurtleBot3, read straight from gz.

    ros2 run apfrl_tb3_sim gt_odom_gz.py --ros-args -p use_sim_time:=true

Publishes the robot's true pose as /odom_gt and as the odom->base_footprint TF,
so apf_node and apfrl_node work on ground truth rather than on the diff-drive
controller's wheel odometry, as the policy did in training. apfrl_tb3.launch.py
starts it; the controller config turns the controller's own odom TF off (a frame
cannot have two parents). /diff_drive_controller/odom is still published.

odom is the world frame: the pose is the model's world pose, planar (x, y, yaw)
as wheel odometry is. That keeps it consistent across the env's set_pose
teleports, which wheel odometry never sees.

Source: /world/<world>/dynamic_pose/info (gz-transport, not ROS).
"""
import math
import threading

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GtOdomGz(Node):
    """Republish the model's gz world pose as /odom_gt and odom->base_footprint."""

    def __init__(self):
        super().__init__("gt_odom_gz")
        self.model = self.declare_parameter("robot_model", "turtlebot3_burger").value
        world = self.declare_parameter("world_name", "default").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        rate = self.declare_parameter("publish_rate_hz", 50.0).value

        self.pub = self.create_publisher(Odometry, "/odom_gt", 10)
        self.tf = TransformBroadcaster(self)
        self._lock = threading.Lock()
        self._latest = None      # (sec, nanosec, x, y, yaw), written on the gz thread
        self._sent = None        # stamp last published, so TF never repeats one

        topic = f"/world/{world}/dynamic_pose/info"
        self._gz = GzNode()
        if not self._gz.subscribe(Pose_V, topic, self._on_poses):
            raise RuntimeError(f"cannot subscribe to {topic}")
        self.create_timer(1.0 / rate, self._publish)
        self._t0 = self.get_clock().now()
        self._warned = False
        self.get_logger().info(
            f"ground truth for '{self.model}' from {topic} -> /odom_gt and "
            f"{self.odom_frame}->{self.base_frame}")

    def _on_poses(self, msg):
        for p in msg.pose:
            if p.name != self.model:
                continue
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self._latest = (msg.header.stamp.sec, msg.header.stamp.nsec,
                                p.position.x, p.position.y, yaw)
            return

    def _publish(self):
        with self._lock:
            latest = self._latest
        if latest is None:
            if not self._warned and (self.get_clock().now() - self._t0).nanoseconds > 5e9:
                self.get_logger().warn(
                    f"no pose for model '{self.model}' after 5 s: /odom_gt and the "
                    "odom TF are not being published")
                self._warned = True
            return
        sec, nsec, x, y, yaw = latest
        if (sec, nsec) == self._sent:
            return
        self._sent = (sec, nsec)
        # The gz stamp is sim time, the clock every node runs on; it dates the
        # pose itself rather than the moment this timer happened to fire.
        stamp = Time(sec=sec, nanosec=nsec) if (sec or nsec) else self.get_clock().now().to_msg()
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        self.pub.publish(od)   # pose only: apf_node reads nothing else

        tr = TransformStamped()
        tr.header.stamp = stamp
        tr.header.frame_id = self.odom_frame
        tr.child_frame_id = self.base_frame
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        self.tf.sendTransform(tr)


def main():
    rclpy.init()
    node = GtOdomGz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
