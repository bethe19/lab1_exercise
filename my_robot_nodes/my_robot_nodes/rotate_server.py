#!/usr/bin/env python3
"""Action server that rotates the robot on the spot for a given angle."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """target - current, wrapped to [-pi, pi] so the robot turns the short way."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__("rotate_server")
        self.declare_parameter("turn_speed", 0.5)

        self.yaw = None

        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10, callback_group=group)
        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def on_odom(self, msg):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal):
        if goal.angle == 0.0 or self.yaw is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        goal_angle = float(goal_handle.request.angle)
        self.get_logger().info(f"Rotating {goal_angle:.2f} rad")

        turn_speed = float(self.get_parameter("turn_speed").value)
        direction = 1.0 if goal_angle > 0.0 else -1.0

        twist = Twist()
        twist.angular.z = direction * turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        turned = 0.0
        last_yaw = self.yaw

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.cmd_vel.publish(Twist())
                goal_handle.canceled()
                result.turned = turned
                self.get_logger().info(f"Canceled after {turned:.2f} rad")
                return result

            if self.yaw is not None and last_yaw is not None:
                delta = angle_difference(self.yaw, last_yaw)
                turned += delta
                last_yaw = self.yaw

            feedback.turned = turned
            goal_handle.publish_feedback(feedback)

            if (direction > 0.0 and turned >= goal_angle - 1e-4) or (direction < 0.0 and turned <= goal_angle + 1e-4):
                break

            self.cmd_vel.publish(twist)
            time.sleep(0.02)

        self.cmd_vel.publish(Twist())
        goal_handle.succeed()
        result.turned = turned
        self.get_logger().info(f"Reached, turned {result.turned:.2f} rad")
        return result


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
