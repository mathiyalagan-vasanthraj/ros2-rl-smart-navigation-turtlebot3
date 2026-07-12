#!/usr/bin/env python3
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as PathMsg


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class PathSaver(Node):
    def __init__(self):
        super().__init__("save_global_path_from_rviz")

        self.start_pose = None
        self.goal_pose = None
        self.sent = False
        self.done = False
        self.request_time = None

        self.out_path = Path("/ws_slam/nav2_rl_project/paths/stage4_global_path.csv")
        self.out_start_goal = Path("/ws_slam/nav2_rl_project/paths/stage4_start_goal.csv")

        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.initial_cb, 10)
        self.create_subscription(PointStamped, "/clicked_point", self.clicked_goal_cb, 10)

        # Publish the received path also, so you can see it in RViz if needed.
        self.path_pub = self.create_publisher(PathMsg, "/rl_saved_global_path", 10)

        self.client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info("Waiting for /compute_path_to_pose action server...")
        self.client.wait_for_server()
        self.get_logger().info("Ready.")
        self.get_logger().info("Use existing RViz.")
        self.get_logger().info("1) Click 2D Pose Estimate at robot start pose.")
        self.get_logger().info("2) Click Publish Point at final goal position.")
        self.get_logger().info("Do NOT click Nav2 Goal.")

    def initial_cb(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "map"
        ps.pose = msg.pose.pose
        self.start_pose = ps

        p = ps.pose.position
        self.get_logger().info(f"Start pose received: x={p.x:.2f}, y={p.y:.2f}")

        self.try_send_request()

    def clicked_goal_cb(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "map"
        ps.pose.position.x = msg.point.x
        ps.pose.position.y = msg.point.y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        self.goal_pose = ps

        self.get_logger().info(f"Goal point received: x={msg.point.x:.2f}, y={msg.point.y:.2f}")

        self.try_send_request()

    def try_send_request(self):
        if self.sent or self.done:
            return

        if self.start_pose is None or self.goal_pose is None:
            return

        self.sent = True
        self.request_time = time.time()

        goal = ComputePathToPose.Goal()
        goal.start = self.start_pose
        goal.goal = self.goal_pose
        goal.use_start = True
        goal.planner_id = ""

        self.get_logger().info("Sending path request to Nav2 planner only...")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("No goal handle received from planner.")
            self.sent = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("Planner rejected request. Choose start/goal in free white area.")
            self.sent = False
            return

        self.get_logger().info("Planner accepted request. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        try:
            result_wrapper = future.result()
            result = result_wrapper.result
            path = result.path
        except Exception as e:
            self.get_logger().error(f"Failed to get result: {e}")
            self.sent = False
            return

        if len(path.poses) == 0:
            self.get_logger().error("Planner returned empty path. Click start/goal in free white area.")
            self.sent = False
            return

        self.save_path(path)

        # Publish it so RViz can show it if you add Path topic /rl_saved_global_path.
        self.path_pub.publish(path)

        self.done = True
        self.get_logger().info("DONE. Path saved. You can press Ctrl+C.")

    def save_path(self, path):
        self.out_path.parent.mkdir(parents=True, exist_ok=True)

        with self.out_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["i", "x", "y", "yaw"])
            for i, pose_stamped in enumerate(path.poses):
                p = pose_stamped.pose.position
                q = pose_stamped.pose.orientation
                writer.writerow([i, p.x, p.y, yaw_from_quat(q)])

        sp = self.start_pose.pose.position
        sq = self.start_pose.pose.orientation
        gp = self.goal_pose.pose.position
        gq = self.goal_pose.pose.orientation

        with self.out_start_goal.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["type", "x", "y", "yaw"])
            writer.writerow(["start", sp.x, sp.y, yaw_from_quat(sq)])
            writer.writerow(["goal", gp.x, gp.y, yaw_from_quat(gq)])

        self.get_logger().info(f"Saved global path: {self.out_path}")
        self.get_logger().info(f"Saved start/goal: {self.out_start_goal}")
        self.get_logger().info(f"Path poses: {len(path.poses)}")

    def timer_cb(self):
        if self.sent and not self.done and self.request_time is not None:
            elapsed = time.time() - self.request_time
            if int(elapsed) in [10, 20, 30, 40, 50, 60]:
                self.get_logger().warn(f"Still waiting for planner result... {elapsed:.0f}s")


def main():
    rclpy.init()
    node = PathSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
