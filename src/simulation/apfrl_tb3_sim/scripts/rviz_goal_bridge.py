#!/usr/bin/env python3
"""Forward RViz "2D Goal Pose" clicks to the apfrl navigation action.

    ros2 run apfrl_tb3_sim rviz_goal_bridge.py --ros-args -p use_sim_time:=true

apfrl_node takes its goal only through the /apfrl/navigate_to_pose action
(apfrl_interfaces/action/NavigateToPose). RViz's goal tool publishes a
PoseStamped on /goal_pose, so this node turns each click into an action goal.
A new click replaces the goal being driven to.

The goal is sent in the frame RViz clicked it in, which must be `odom` (the
fixed frame of the provided RViz config). odom is the gz world frame here, so
world coordinates and odom coordinates are the same.
"""
import rclpy
from apfrl_interfaces.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node


class RvizGoalBridge(Node):

    def __init__(self):
        super().__init__('rviz_goal_bridge')
        self.client = ActionClient(self, NavigateToPose, '/apfrl/navigate_to_pose')
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal, 1)
        self.get_logger().info('click "2D Goal Pose" in RViz to send apfrl a goal')

    def _on_goal(self, msg):
        if msg.header.frame_id != 'odom':
            self.get_logger().warn(
                f"goal is in frame '{msg.header.frame_id}', expected 'odom'; "
                'set the RViz fixed frame to odom')
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('/apfrl/navigate_to_pose is not up; is apfrl_node running?')
            return
        goal = NavigateToPose.Goal()
        goal.target_pose = msg
        p = msg.pose.position
        self.get_logger().info(f'goal ({p.x:.2f}, {p.y:.2f})')
        self.client.send_goal_async(goal, feedback_callback=self._on_feedback) \
            .add_done_callback(self._on_response)

    def _on_feedback(self, fb):
        self.get_logger().info(f'distance remaining {fb.feedback.distance_remaining:.2f} m',
                               throttle_duration_sec=2.0)

    def _on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('goal rejected')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self.get_logger().info(
                'goal reached' if f.result().result.success else 'goal not reached'))


def main():
    rclpy.init()
    node = RvizGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
