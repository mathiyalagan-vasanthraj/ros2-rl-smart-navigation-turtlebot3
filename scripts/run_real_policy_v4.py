#!/usr/bin/env python3

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
import rclpy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from stable_baselines3 import A2C, DQN, PPO
from tf2_ros import Buffer, TransformException, TransformListener


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion) -> float:
    """Convert a geometry_msgs quaternion into yaw."""
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def action_to_integer(action) -> int:
    """Convert Stable-Baselines action output into a Python integer."""
    if hasattr(action, "item"):
        return int(action.item())

    return int(action)


class RealPolicyRunner(Node):
    """
    Run a trained A2C, DQN, or PPO local controller on the real TurtleBot3.

    Robot pose is read primarily from TF:

        map -> base_footprint

    /amcl_pose is retained as a fallback.
    """

    ACTIONS = {
        0: (0.22, 0.00, "forward"),
        1: (0.16, 0.75, "forward_left"),
        2: (0.16, -0.75, "forward_right"),
        3: (0.04, 1.20, "turn_left"),
        4: (0.04, -1.20, "turn_right"),
        5: (0.08, 0.00, "slow_forward"),
        6: (0.00, 0.00, "stop"),
    }

    def __init__(self, args):
        super().__init__(f"real_policy_{args.algo}")

        self.args = args
        self.maximum_scan_range = 3.5

        self.path = self.load_path(args.path)
        self.path_distances = self.calculate_path_distances(self.path)
        self.total_path_length = self.path_distances[-1]
        self.goal = self.path[-1]

        self.previous_path_index = 0

        # Front-obstacle recovery state.
        self.recovery_steps = 0
        self.recovery_direction = 0

        self.scan = None
        self.scan_angles = None
        self.robot_pose = None

        self.last_scan_time = 0.0
        self.last_pose_time = 0.0
        self.pose_source = "none"

        # TF listener for map -> base_footprint.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # Real LiDAR subscription.
        self.create_subscription(
            LaserScan,
            args.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # AMCL pose fallback.
        self.create_subscription(
            PoseWithCovarianceStamped,
            args.pose_topic,
            self.pose_callback,
            10,
        )

        # Robot velocity publisher.
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            args.cmd_topic,
            10,
        )

        model_path = Path(args.model)

        if not model_path.exists():
            raise FileNotFoundError(
                f"Model file does not exist: {model_path}"
            )

        model_loader = {
            "a2c": A2C,
            "dqn": DQN,
            "ppo": PPO,
        }[args.algo]

        self.model = model_loader.load(str(model_path))

        log_directory = Path(args.log_dir)
        log_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = time.strftime("%Y%m%d_%H%M%S")

        self.log_path = (
            log_directory
            / f"{args.algo}_real_{timestamp}.csv"
        )

        self.log_file = self.log_path.open(
            "w",
            newline="",
        )

        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow(
            [
                "time",
                "step",
                "x",
                "y",
                "yaw",
                "action",
                "action_name",
                "linear_velocity",
                "angular_velocity",
                "goal_distance",
                "cross_track_error",
                "progress",
                "minimum_scan",
                "front_scan",
                "left_scan",
                "right_scan",
                "pose_source",
                "status",
            ]
        )

    @staticmethod
    def load_path(filename):
        """Load x,y path points from a CSV file."""
        path_file = Path(filename)

        if not path_file.exists():
            raise FileNotFoundError(
                f"Path file does not exist: {path_file}"
            )

        points = []

        with path_file.open(
            "r",
            newline="",
        ) as csv_file:

            reader = csv.DictReader(csv_file)

            if reader.fieldnames is None:
                raise RuntimeError(
                    f"Path CSV has no header: {path_file}"
                )

            if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
                raise RuntimeError(
                    "Path CSV must contain columns named x and y. "
                    f"Found columns: {reader.fieldnames}"
                )

            for row in reader:
                points.append(
                    (
                        float(row["x"]),
                        float(row["y"]),
                    )
                )

        if len(points) < 5:
            raise RuntimeError(
                "Saved path is missing or contains fewer than five points."
            )

        return points

    @staticmethod
    def calculate_path_distances(path):
        """Calculate cumulative distance along the global path."""
        cumulative = [0.0]

        for index in range(
            1,
            len(path),
        ):
            segment_length = math.hypot(
                path[index][0] - path[index - 1][0],
                path[index][1] - path[index - 1][1],
            )

            cumulative.append(
                cumulative[-1] + segment_length
            )

        return cumulative

    def scan_callback(self, message):
        """Receive and sanitize the real LaserScan."""
        if not message.ranges:
            return

        sanitized_ranges = []

        for scan_range in message.ranges:
            if math.isfinite(scan_range) and scan_range > 0.01:
                sanitized_ranges.append(
                    min(
                        float(scan_range),
                        self.maximum_scan_range,
                    )
                )
            else:
                sanitized_ranges.append(
                    self.maximum_scan_range
                )

        self.scan = np.asarray(
            sanitized_ranges,
            dtype=np.float32,
        )

        self.scan_angles = (
            message.angle_min
            + np.arange(
                len(self.scan),
                dtype=np.float32,
            )
            * message.angle_increment
        )

        self.last_scan_time = time.monotonic()

    def pose_callback(self, message):
        """Fallback pose input from /amcl_pose."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        self.robot_pose = (
            float(position.x),
            float(position.y),
            quaternion_to_yaw(orientation),
        )

        self.last_pose_time = time.monotonic()
        self.pose_source = self.args.pose_topic

    def update_pose_from_tf(self) -> bool:
        """Read the latest robot pose from map -> base_footprint."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.global_frame,
                self.args.base_frame,
                Time(),
            )

        except TransformException:
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        self.robot_pose = (
            float(translation.x),
            float(translation.y),
            quaternion_to_yaw(rotation),
        )

        self.last_pose_time = time.monotonic()

        self.pose_source = (
            f"TF "
            f"{self.args.global_frame}"
            f"->{self.args.base_frame}"
        )

        return True

    def wait_for_sensor_data(self):
        """Wait for LiDAR and either TF or /amcl_pose."""
        print(
            f"Waiting for {self.args.scan_topic} and robot pose "
            f"(TF {self.args.global_frame}->{self.args.base_frame} "
            f"or {self.args.pose_topic})...",
            flush=True,
        )

        start_time = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(
                self,
                timeout_sec=0.10,
            )

            # Prefer the live map -> base_footprint TF.
            self.update_pose_from_tf()

            scan_is_available = self.scan is not None
            pose_is_available = self.robot_pose is not None

            if scan_is_available and pose_is_available:
                return

            elapsed = time.monotonic() - start_time

            if elapsed > self.args.sensor_wait_timeout:
                missing_inputs = []

                if not scan_is_available:
                    missing_inputs.append(
                        self.args.scan_topic
                    )

                if not pose_is_available:
                    missing_inputs.append(
                        (
                            f"TF "
                            f"{self.args.global_frame}"
                            f"->{self.args.base_frame} "
                            f"or {self.args.pose_topic}"
                        )
                    )

                raise RuntimeError(
                    "No data from: "
                    + ", ".join(missing_inputs)
                )

    def nearest_path_point(self, x, y):
        """Find the nearest path point without searching far backwards."""
        search_start = max(
            0,
            self.previous_path_index - 10,
        )

        distances = [
            math.hypot(
                path_x - x,
                path_y - y,
            )
            for path_x, path_y in self.path[search_start:]
        ]

        relative_index = int(
            np.argmin(distances)
        )

        absolute_index = (
            search_start + relative_index
        )

        return (
            absolute_index,
            distances[relative_index],
        )

    def lookahead_point(self, path_index):
        """Return a path point at the configured lookahead distance."""
        target_distance = (
            self.path_distances[path_index]
            + self.args.lookahead
        )

        for index in range(
            path_index,
            len(self.path),
        ):
            if self.path_distances[index] >= target_distance:
                return self.path[index]

        return self.path[-1]

    def sector_minimum(self, center_angle, half_width):
        """Return minimum LiDAR distance in an angular sector."""
        angular_difference = np.arctan2(
            np.sin(
                self.scan_angles - center_angle
            ),
            np.cos(
                self.scan_angles - center_angle
            ),
        )

        mask = (
            np.abs(angular_difference)
            <= half_width
        )

        if not np.any(mask):
            return self.maximum_scan_range

        return float(
            np.min(self.scan[mask])
        )

    def create_observation(self):
        """
        Construct the 32-value observation expected by the trained models.

        24 LiDAR samples
        + lookahead distance
        + sin lookahead angle
        + cos lookahead angle
        + goal distance
        + cross-track error
        + minimum scan
        + constant navigation-active value
        + path progress
        """
        x, y, heading = self.robot_pose

        path_index, cross_track_error = (
            self.nearest_path_point(
                x,
                y,
            )
        )

        lookahead_x, lookahead_y = (
            self.lookahead_point(path_index)
        )

        lookahead_distance = math.hypot(
            lookahead_x - x,
            lookahead_y - y,
        )

        lookahead_angle = wrap_angle(
            math.atan2(
                lookahead_y - y,
                lookahead_x - x,
            )
            - heading
        )

        goal_distance = math.hypot(
            self.goal[0] - x,
            self.goal[1] - y,
        )

        minimum_scan = float(
            np.min(self.scan)
        )

        progress = (
            self.path_distances[path_index]
            / max(
                self.total_path_length,
                1.0e-6,
            )
        )

        lidar_indexes = np.linspace(
            0,
            len(self.scan) - 1,
            24,
        ).astype(int)

        lidar_observation = (
            self.scan[lidar_indexes]
            / self.maximum_scan_range
        )

        navigation_observation = np.asarray(
            [
                min(
                    lookahead_distance / 2.0,
                    1.0,
                ),
                math.sin(lookahead_angle),
                math.cos(lookahead_angle),
                min(
                    goal_distance / 5.0,
                    1.0,
                ),
                min(
                    cross_track_error / 1.5,
                    1.0,
                ),
                min(
                    minimum_scan
                    / self.maximum_scan_range,
                    1.0,
                ),
                1.0,
                progress,
            ],
            dtype=np.float32,
        )

        observation = np.concatenate(
            [
                lidar_observation,
                navigation_observation,
            ]
        ).astype(np.float32)

        return (
            observation,
            path_index,
            goal_distance,
            cross_track_error,
            progress,
            minimum_scan,
        )

    def publish_velocity(
        self,
        linear_velocity=0.0,
        angular_velocity=0.0,
    ):
        """Publish a velocity command."""
        command = Twist()

        command.linear.x = float(
            linear_velocity
        )

        command.angular.z = float(
            angular_velocity
        )

        self.cmd_vel_publisher.publish(
            command
        )

    def stop_robot(self):
        """Repeatedly publish zero velocity."""
        for _ in range(10):
            self.publish_velocity(
                0.0,
                0.0,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.01,
            )

            time.sleep(0.03)

    def execute(self):
        """Run the real-robot policy test."""
        self.wait_for_sensor_data()

        (
            observation,
            path_index,
            goal_distance,
            cross_track_error,
            progress,
            minimum_scan,
        ) = self.create_observation()

        self.previous_path_index = path_index

        path_start_distance = math.hypot(
            self.robot_pose[0] - self.path[0][0],
            self.robot_pose[1] - self.path[0][1],
        )

        print("=" * 72)
        print(
            f"REAL TEST {self.args.algo.upper()} "
            f"| dry_run={self.args.dry_run}"
        )
        print(
            f"model: {self.args.model}"
        )
        print(
            f"path:  {self.args.path}"
        )
        print(
            f"path points={len(self.path)}, "
            f"length={self.total_path_length:.3f} m"
        )
        print(
            f"distance from saved path start="
            f"{path_start_distance:.3f} m"
        )
        print(
            f"pose source={self.pose_source}"
        )
        print(
            f"log={self.log_path}"
        )
        print("=" * 72, flush=True)

        if path_start_distance > self.args.max_start_gap:
            raise RuntimeError(
                f"Robot is {path_start_distance:.2f} m "
                f"from the saved path start. "
                f"Return the robot to the path start and "
                f"set 2D Pose Estimate again."
            )

        for seconds_remaining in range(
            self.args.countdown,
            0,
            -1,
        ):
            self.publish_velocity(
                0.0,
                0.0,
            )

            print(
                f"Starting in {seconds_remaining}...",
                flush=True,
            )

            countdown_end = (
                time.monotonic() + 1.0
            )

            while (
                rclpy.ok()
                and time.monotonic() < countdown_end
            ):
                rclpy.spin_once(
                    self,
                    timeout_sec=0.05,
                )

                self.update_pose_from_tf()

        final_status = "max_steps"

        for step in range(
            1,
            self.args.max_steps + 1,
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.01,
            )

            self.update_pose_from_tf()

            current_time = time.monotonic()

            if (
                current_time - self.last_scan_time
                > self.args.sensor_timeout
            ):
                final_status = "stale_scan"
                break

            if (
                current_time - self.last_pose_time
                > self.args.sensor_timeout
            ):
                final_status = "stale_pose"
                break

            (
                observation,
                path_index,
                goal_distance,
                cross_track_error,
                progress,
                minimum_scan,
            ) = self.create_observation()

            self.previous_path_index = max(
                self.previous_path_index,
                path_index,
            )

            prediction = self.model.predict(
                observation,
                deterministic=True,
            )[0]

            action = action_to_integer(
                prediction
            )

            (
                base_linear,
                base_angular,
                action_name,
            ) = self.ACTIONS.get(
                action,
                self.ACTIONS[6],
            )

            linear_velocity = float(
                np.clip(
                    base_linear
                    * self.args.speed_scale,
                    0.0,
                    self.args.max_linear,
                )
            )

            angular_velocity = float(
                np.clip(
                    base_angular
                    * self.args.speed_scale,
                    -self.args.max_angular,
                    self.args.max_angular,
                )
            )

            front_distance = self.sector_minimum(
                0.0,
                math.radians(35.0),
            )

            status = "running"

            left_clearance = self.sector_minimum(
                math.radians(75.0),
                math.radians(35.0),
            )

            right_clearance = self.sector_minimum(
                math.radians(-75.0),
                math.radians(35.0),
            )

            # Hard safety checks override the neural-network action.
            if goal_distance <= self.args.goal_threshold:
                linear_velocity = 0.0
                angular_velocity = 0.0
                status = "goal_reached"

            elif cross_track_error > self.args.max_cte:
                linear_velocity = 0.0
                angular_velocity = 0.0
                status = "off_path_stop"

            elif minimum_scan < self.args.all_stop:
                linear_velocity = 0.0
                angular_velocity = 0.0
                status = "emergency_clearance_stop"

            elif (
                front_distance < self.args.stop_distance
                or (
                    self.recovery_steps > 0
                    and front_distance
                    < self.args.recovery_clearance
                )
            ):
                # Begin or continue a bounded recovery rotation.
                if self.recovery_steps == 0:
                    if left_clearance >= right_clearance:
                        self.recovery_direction = 1
                    else:
                        self.recovery_direction = -1

                self.recovery_steps += 1

                linear_velocity = 0.0

                angular_velocity = float(
                    np.clip(
                        self.recovery_direction
                        * self.args.recovery_angular,
                        -self.args.max_angular,
                        self.args.max_angular,
                    )
                )

                status = "front_recovery"

                # Recovery is bounded. It never rotates indefinitely.
                if (
                    self.recovery_steps
                    >= self.args.max_recovery_steps
                ):
                    linear_velocity = 0.0
                    angular_velocity = 0.0
                    status = "front_safety_stop"

            else:
                # Front sector is clear enough; resume the policy.
                self.recovery_steps = 0
                self.recovery_direction = 0

                if (
                    linear_velocity > 0.0
                    and front_distance
                    < self.args.slow_distance
                ):
                    slowdown_ratio = (
                        (
                            front_distance
                            - self.args.stop_distance
                        )
                        / (
                            self.args.slow_distance
                            - self.args.stop_distance
                        )
                    )

                    linear_velocity *= float(
                        np.clip(
                            slowdown_ratio,
                            0.15,
                            1.0,
                        )
                    )

                    status = "front_slowdown"

            if self.args.dry_run:
                linear_velocity = 0.0
                angular_velocity = 0.0

                if status == "running":
                    status = "dry_run"

            self.publish_velocity(
                linear_velocity,
                angular_velocity,
            )

            x, y, heading = self.robot_pose

            self.csv_writer.writerow(
                [
                    time.time(),
                    step,
                    x,
                    y,
                    heading,
                    action,
                    action_name,
                    linear_velocity,
                    angular_velocity,
                    goal_distance,
                    cross_track_error,
                    progress,
                    minimum_scan,
                    front_distance,
                    left_clearance,
                    right_clearance,
                    self.pose_source,
                    status,
                ]
            )

            self.log_file.flush()

            normal_statuses = {
                "running",
                "dry_run",
                "front_slowdown",
                "front_recovery",
            }

            if (
                step == 1
                or step % 5 == 0
                or status not in normal_statuses
            ):
                print(
                    f"[{self.args.algo}] "
                    f"step={step:03d} "
                    f"action={action}:{action_name} "
                    f"cmd=({linear_velocity:.3f},"
                    f"{angular_velocity:.3f}) "
                    f"goal={goal_distance:.2f} "
                    f"cte={cross_track_error:.2f} "
                    f"progress={progress:.2f} "
                    f"min={minimum_scan:.2f} "
                    f"front={front_distance:.2f} "
                    f"left={left_clearance:.2f} "
                    f"right={right_clearance:.2f} "
                    f"pose={self.pose_source} "
                    f"status={status}",
                    flush=True,
                )

            stop_statuses = {
                "goal_reached",
                "off_path_stop",
                "emergency_clearance_stop",
                "front_safety_stop",
            }

            if status in stop_statuses:
                final_status = status
                break

            step_end = (
                time.monotonic()
                + self.args.step_time
            )

            while (
                rclpy.ok()
                and time.monotonic() < step_end
            ):
                rclpy.spin_once(
                    self,
                    timeout_sec=0.01,
                )

                self.update_pose_from_tf()

        self.stop_robot()

        print(
            f"FINAL STATUS: {final_status}"
        )
        print(
            f"CSV LOG: {self.log_path}"
        )
        print(
            "Robot stopped.",
            flush=True,
        )

    def close(self):
        """Close the log file and guarantee zero velocity."""
        self.stop_robot()

        if not self.log_file.closed:
            self.log_file.close()


def parse_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--algo",
        required=True,
        choices=[
            "a2c",
            "dqn",
            "ppo",
        ],
    )

    parser.add_argument(
        "--model",
        required=True,
    )

    parser.add_argument(
        "--path",
        default=(
            "/ws_slam/nav2_rl_project/"
            "paths/real_global_path_latest.csv"
        ),
    )

    parser.add_argument(
        "--scan-topic",
        default="/scan",
    )

    parser.add_argument(
        "--pose-topic",
        default="/amcl_pose",
    )

    parser.add_argument(
        "--global-frame",
        default="map",
    )

    parser.add_argument(
        "--base-frame",
        default="base_footprint",
    )

    parser.add_argument(
        "--cmd-topic",
        default="/cmd_vel",
    )

    parser.add_argument(
        "--log-dir",
        default=(
            "/ws_slam/nav2_rl_project/"
            "results/real_tests"
        ),
    )

    parser.add_argument(
        "--max-steps",
        type=int,
        default=450,
    )

    parser.add_argument(
        "--step-time",
        type=float,
        default=0.10,
    )

    parser.add_argument(
        "--lookahead",
        type=float,
        default=0.60,
    )

    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.40,
    )

    parser.add_argument(
        "--max-linear",
        type=float,
        default=0.09,
    )

    parser.add_argument(
        "--max-angular",
        type=float,
        default=0.55,
    )

    parser.add_argument(
        "--goal-threshold",
        type=float,
        default=0.30,
    )

    parser.add_argument(
        "--max-cte",
        type=float,
        default=0.85,
    )

    parser.add_argument(
        "--max-start-gap",
        type=float,
        default=0.50,
    )

    parser.add_argument(
        "--stop-distance",
        type=float,
        default=0.28,
    )

    parser.add_argument(
        "--slow-distance",
        type=float,
        default=0.55,
    )

    parser.add_argument(
        "--all-stop",
        type=float,
        default=0.14,
    )

    parser.add_argument(
        "--recovery-angular",
        type=float,
        default=0.30,
    )

    parser.add_argument(
        "--recovery-clearance",
        type=float,
        default=0.50,
    )

    parser.add_argument(
        "--max-recovery-steps",
        type=int,
        default=100,
    )

    parser.add_argument(
        "--sensor-timeout",
        type=float,
        default=1.0,
    )

    parser.add_argument(
        "--sensor-wait-timeout",
        type=float,
        default=30.0,
    )

    parser.add_argument(
        "--countdown",
        type=int,
        default=5,
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
    )

    return parser.parse_args()


def main():
    args = parse_arguments()

    rclpy.init()

    runner = None

    try:
        runner = RealPolicyRunner(args)
        runner.execute()

    except KeyboardInterrupt:
        print(
            "Interrupted. Stopping robot.",
            flush=True,
        )

    except Exception as error:
        print(
            f"REAL TEST ERROR: {error}",
            flush=True,
        )

    finally:
        if runner is not None:
            runner.close()
            runner.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
