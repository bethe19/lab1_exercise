#!/usr/bin/env python3
"""Action client that drives the robot around a square using DriveDistance and Rotate."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

    def wait_for_servers(self):
        self.get_logger().info("Waiting for action servers (/drive_distance and /rotate)...")
        self.drive_client.wait_for_server()
        self.rotate_client.wait_for_server()
        self.get_logger().info("Action servers are available.")

    def drive_distance(self, distance: float) -> bool:
        goal = DriveDistance.Goal()
        goal.distance = float(distance)

        # Retry sending goal if odometry has not arrived yet at startup
        goal_handle = None
        for _ in range(20):
            send_future = self.drive_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle.accepted:
                break
            self.get_logger().warn("drive_distance goal not accepted yet, waiting...")
            time.sleep(1.0)

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("drive_distance goal rejected by server")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()

        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"drive_distance goal ended with status {result_response.status}")
            return False

        if not result_response.result.reached:
            self.get_logger().warn(
                f"drive_distance goal aborted early after {result_response.result.travelled:.2f} m"
            )
            return False

        self.get_logger().info(f"Drove {result_response.result.travelled:.2f} m")
        return True

    def rotate_angle(self, angle: float) -> bool:
        goal = Rotate.Goal()
        goal.angle = float(angle)

        goal_handle = None
        for _ in range(20):
            send_future = self.rotate_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle.accepted:
                break
            self.get_logger().warn("rotate goal not accepted yet, waiting...")
            time.sleep(1.0)

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("rotate goal rejected by server")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()

        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"rotate goal ended with status {result_response.status}")
            return False

        self.get_logger().info(f"Rotated {result_response.result.turned:.2f} rad")
        return True

    def drive_square(self):
        side_length = float(self.get_parameter("side_length").value)
        self.get_logger().info(f"Starting square of side length {side_length:.2f} m")

        for side in range(1, 5):
            self.get_logger().info(f"--- Side {side}/4: driving straight ---")
            if not self.drive_distance(side_length):
                self.get_logger().error(f"DriveDistance failed on side {side}")
                return False

            self.get_logger().info(f"--- Side {side}/4: turning left quarter turn ---")
            if not self.rotate_angle(math.pi / 2.0):
                self.get_logger().error(f"Rotate failed on side {side}")
                return False

        self.get_logger().info("Square driving complete!")
        return True


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.wait_for_servers()
        node.drive_square()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
