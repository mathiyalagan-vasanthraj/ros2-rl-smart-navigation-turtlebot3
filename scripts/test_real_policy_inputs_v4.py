#!/usr/bin/env python3

import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from stable_baselines3 import A2C


PROJECT_ROOT = Path(__file__).resolve().parents[1]

MODEL_PATH = (
    PROJECT_ROOT
    / "results"
    / "a2c_v4_nav2path_local_controller.zip"
)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class RealPolicyProbe(Node):
    def __init__(self):
        super().__init__("real_policy_probe_v4")

        self.maximum_lidar_range = 3.5
        self.number_of_lidar_bins = 24
        self.lookahead_distance = 0.60

        self.latest_scan = None
        self.latest_odom = None

        self.path = []
        self.path_distances = []
        self.total_path_length = 0.0
        self.goal = None

        self.stop_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # The real LD08 LiDAR publishes BEST_EFFORT data.
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

    def scan_callback(self, message):
        cleaned_ranges = []

        for distance in message.ranges:
            if math.isfinite(distance) and distance > 0.01:
                cleaned_ranges.append(
                    min(float(distance), self.maximum_lidar_range)
                )
            else:
                cleaned_ranges.append(self.maximum_lidar_range)

        self.latest_scan = np.asarray(
            cleaned_ranges,
            dtype=np.float32,
        )

    def odom_callback(self, message):
        self.latest_odom = message

    def stop_robot(self):
        self.stop_publisher.publish(Twist())

    def wait_for_real_sensors(self):
        print("[PROBE] Waiting for real /scan and /odom...")

        deadline = time.monotonic() + 20.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)

            if (
                self.latest_scan is not None
                and self.latest_odom is not None
            ):
                print("[PROBE] Real sensor data received.")
                return

        raise RuntimeError(
            "No real /scan or /odom data received."
        )

    def current_pose(self):
        position = self.latest_odom.pose.pose.position
        orientation = self.latest_odom.pose.pose.orientation

        return (
            float(position.x),
            float(position.y),
            yaw_from_quaternion(orientation),
        )

    def create_virtual_path(self, path_length=1.0):
        start_x, start_y, start_yaw = self.current_pose()

        distances = np.linspace(
            0.0,
            path_length,
            101,
        )

        self.path = [
            (
                start_x + distance * math.cos(start_yaw),
                start_y + distance * math.sin(start_yaw),
            )
            for distance in distances
        ]

        self.path_distances = list(distances)
        self.total_path_length = float(distances[-1])
        self.goal = self.path[-1]

        print(
            "[PROBE] Virtual path created:"
            f" {self.total_path_length:.2f} m"
        )

        print(
            f"[PROBE] Start: ({start_x:.3f}, {start_y:.3f})"
        )

        print(
            f"[PROBE] Goal:  "
            f"({self.goal[0]:.3f}, {self.goal[1]:.3f})"
        )

    def create_lidar_bins(self):
        indices = np.linspace(
            0,
            len(self.latest_scan) - 1,
            self.number_of_lidar_bins,
        ).astype(int)

        values = self.latest_scan[indices]

        values = np.clip(
            values,
            0.0,
            self.maximum_lidar_range,
        )

        return (
            values / self.maximum_lidar_range
        ).astype(np.float32)

    def nearest_path_point(self, robot_x, robot_y):
        nearest_index = 0
        nearest_distance = float("inf")

        for index, point in enumerate(self.path):
            path_x, path_y = point

            distance = math.hypot(
                path_x - robot_x,
                path_y - robot_y,
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        return nearest_index, nearest_distance

    def find_lookahead_point(self, nearest_index):
        target_distance = (
            self.path_distances[nearest_index]
            + self.lookahead_distance
        )

        for index in range(nearest_index, len(self.path)):
            if self.path_distances[index] >= target_distance:
                return self.path[index]

        return self.path[-1]

    def create_observation(self):
        robot_x, robot_y, robot_yaw = self.current_pose()

        nearest_index, cross_track_error = (
            self.nearest_path_point(robot_x, robot_y)
        )

        lookahead_x, lookahead_y = (
            self.find_lookahead_point(nearest_index)
        )

        lookahead_distance = math.hypot(
            lookahead_x - robot_x,
            lookahead_y - robot_y,
        )

        lookahead_angle = wrap_angle(
            math.atan2(
                lookahead_y - robot_y,
                lookahead_x - robot_x,
            )
            - robot_yaw
        )

        goal_distance = math.hypot(
            self.goal[0] - robot_x,
            self.goal[1] - robot_y,
        )

        minimum_scan = float(np.min(self.latest_scan))

        # Gazebo supplied obstacle-centre distance during training.
        # For the real robot, estimate it using LiDAR clearance.
        estimated_dynamic_distance = min(
            minimum_scan + 0.30,
            2.0,
        )

        progress = (
            self.path_distances[nearest_index]
            / max(self.total_path_length, 1e-6)
        )

        navigation_features = np.asarray(
            [
                min(lookahead_distance / 2.0, 1.0),
                math.sin(lookahead_angle),
                math.cos(lookahead_angle),
                min(goal_distance / 5.0, 1.0),
                min(cross_track_error / 1.5, 1.0),
                min(
                    minimum_scan
                    / self.maximum_lidar_range,
                    1.0,
                ),
                min(
                    estimated_dynamic_distance / 2.0,
                    1.0,
                ),
                progress,
            ],
            dtype=np.float32,
        )

        observation = np.concatenate(
            [
                self.create_lidar_bins(),
                navigation_features,
            ]
        ).astype(np.float32)

        observation = np.clip(
            observation,
            -1.0,
            1.0,
        )

        return observation, {
            "goal_distance": goal_distance,
            "cross_track_error": cross_track_error,
            "progress": progress,
            "minimum_scan": minimum_scan,
        }


def main():
    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"Model not found: {MODEL_PATH}"
        )

    rclpy.init()

    node = RealPolicyProbe()

    action_names = {
        0: "forward",
        1: "forward-left",
        2: "forward-right",
        3: "turn-left",
        4: "turn-right",
        5: "slow-forward",
        6: "stop",
    }

    try:
        node.wait_for_real_sensors()

        model = A2C.load(MODEL_PATH)

        print(
            f"[PROBE] Model observation space: "
            f"{model.observation_space.shape}"
        )

        node.create_virtual_path(path_length=1.0)

        print("[PROBE] Robot will NOT move.")
        print("[PROBE] Publishing zero velocity only.")
        print("[PROBE] Testing real observations...")

        for step in range(1, 31):
            rclpy.spin_once(node, timeout_sec=0.10)

            observation, information = (
                node.create_observation()
            )

            action, _ = model.predict(
                observation,
                deterministic=True,
            )

            action = int(
                np.asarray(action).reshape(-1)[0]
            )

            # Safety: never send the model command in this test.
            node.stop_robot()

            print(
                f"[PROBE] step={step:02d} "
                f"obs={observation.shape} "
                f"action={action} "
                f"({action_names[action]}) "
                f"goal={information['goal_distance']:.2f} "
                f"cte={information['cross_track_error']:.3f} "
                f"progress={information['progress']:.2f} "
                f"minimum_scan="
                f"{information['minimum_scan']:.2f}"
            )

            time.sleep(0.20)

        print("")
        print("[PROBE] SUCCESS")
        print(
            "[PROBE] The trained A2C model accepted "
            "real LiDAR and odometry observations."
        )

    except KeyboardInterrupt:
        print("[PROBE] Operator stopped the test.")

    finally:
        for _ in range(10):
            node.stop_robot()
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)

        node.destroy_node()
        rclpy.shutdown()

        print("[PROBE] Zero velocity published.")


if __name__ == "__main__":
    main()
