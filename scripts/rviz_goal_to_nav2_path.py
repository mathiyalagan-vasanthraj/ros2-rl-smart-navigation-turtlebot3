#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = PROJECT_ROOT / "results"
PATH_FILE = RESULTS_DIR / "real_global_path.csv"
GOAL_FILE = RESULTS_DIR / "real_goal_pose.csv"


class GoalToNav2Path(Node):
    def __init__(self):
        super().__init__("rviz_goal_to_nav2_path")

        RESULTS_DIR.mkdir(parents=True, exist_ok=True)

        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_publisher = self.create_publisher(
            NavPath,
            "/rl_global_path",
            path_qos,
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10,
        )

        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
        )

        self.request_in_progress = False
        self.last_goal = None

        self.get_logger().info(
            "Listening for RViz goals on /goal_pose."
        )
        self.get_logger().info(
            "Use the RViz toolbar: 2D Goal Pose."
        )
        self.get_logger().info(
            "Only the Nav2 global planner will run."
        )
        self.get_logger().info(
            "This node will not publish /cmd_vel."
        )

    def goal_callback(self, goal_msg: PoseStamped):
        if self.request_in_progress:
            self.get_logger().warning(
                "A planning request is already running."
            )
            return

        if not goal_msg.header.frame_id:
            goal_msg.header.frame_id = "map"

        if goal_msg.header.frame_id != "map":
            self.get_logger().error(
                f"Goal frame must be map, received "
                f"{goal_msg.header.frame_id!r}."
            )
            return

        self.last_goal = goal_msg
        self.save_goal(goal_msg)

        self.get_logger().info(
            "RViz goal received: "
            f"x={goal_msg.pose.position.x:.3f}, "
            f"y={goal_msg.pose.position.y:.3f}"
        )

        if not self.compute_path_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                "Nav2 action /compute_path_to_pose "
                "is unavailable."
            )
            return

        request = ComputePathToPose.Goal()
        request.goal = goal_msg
        request.planner_id = ""
        request.use_start = False

        self.request_in_progress = True

        future = self.compute_path_client.send_goal_async(
            request
        )
        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_in_progress = False
            self.get_logger().error(
                f"Could not send planning request: {exc}"
            )
            return

        if not goal_handle.accepted:
            self.request_in_progress = False
            self.get_logger().error(
                "Nav2 rejected the planning request."
            )
            return

        self.get_logger().info(
            "Nav2 accepted the planning request."
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.path_result_callback
        )

    def path_result_callback(self, future):
        self.request_in_progress = False

        try:
            wrapped_result = future.result()
            path_msg = wrapped_result.result.path
        except Exception as exc:
            self.get_logger().error(
                f"Path calculation failed: {exc}"
            )
            return

        if not path_msg.poses:
            self.get_logger().error(
                "Nav2 returned an empty path."
            )
            return

        path_length = self.calculate_path_length(path_msg)

        self.path_publisher.publish(path_msg)
        self.save_path(path_msg)

        self.get_logger().info(
            "PATH READY: "
            f"poses={len(path_msg.poses)}, "
            f"length={path_length:.3f} m"
        )
        self.get_logger().info(
            "Published: /rl_global_path"
        )
        self.get_logger().info(
            f"Saved: {PATH_FILE}"
        )
        self.get_logger().info(
            "Robot remains stationary."
        )

    @staticmethod
    def calculate_path_length(path_msg: NavPath) -> float:
        total_length = 0.0

        for previous, current in zip(
            path_msg.poses[:-1],
            path_msg.poses[1:],
        ):
            dx = (
                current.pose.position.x
                - previous.pose.position.x
            )
            dy = (
                current.pose.position.y
                - previous.pose.position.y
            )

            total_length += math.hypot(dx, dy)

        return total_length

    @staticmethod
    def save_goal(goal_msg: PoseStamped):
        with GOAL_FILE.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.writer(file)

            writer.writerow(
                [
                    "frame",
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                ]
            )

            writer.writerow(
                [
                    goal_msg.header.frame_id,
                    goal_msg.pose.position.x,
                    goal_msg.pose.position.y,
                    goal_msg.pose.position.z,
                    goal_msg.pose.orientation.x,
                    goal_msg.pose.orientation.y,
                    goal_msg.pose.orientation.z,
                    goal_msg.pose.orientation.w,
                ]
            )

    @staticmethod
    def save_path(path_msg: NavPath):
        with PATH_FILE.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.writer(file)

            writer.writerow(
                [
                    "index",
                    "frame",
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                ]
            )

            for index, pose_stamped in enumerate(
                path_msg.poses
            ):
                pose = pose_stamped.pose

                writer.writerow(
                    [
                        index,
                        path_msg.header.frame_id,
                        pose.position.x,
                        pose.position.y,
                        pose.position.z,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ]
                )


def main():
    rclpy.init()
    node = GoalToNav2Path()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "Planner bridge stopped."
        )
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
