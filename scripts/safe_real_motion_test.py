#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


# Conservative first-test limits.
MIN_ALL_AROUND = 0.30       # Never move when anything is closer than 30 cm.
MIN_FRONT_FORWARD = 0.40    # Stop forward motion below 40 cm.
FRONT_HALF_ANGLE_DEG = 25.0
PUBLISH_RATE_HZ = 10.0


class SafeMotionTest(Node):
    def __init__(self):
        super().__init__("safe_real_motion_test")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.latest_scan = None

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_clearances(self):
        if self.latest_scan is None:
            return None, None

        valid_all = []
        valid_front = []

        angle = self.latest_scan.angle_min
        front_limit = math.radians(FRONT_HALF_ANGLE_DEG)

        for distance in self.latest_scan.ranges:
            if (
                math.isfinite(distance)
                and self.latest_scan.range_min < distance
                < self.latest_scan.range_max
            ):
                valid_all.append(distance)

                # Normalize angle to [-pi, pi].
                normalized_angle = math.atan2(
                    math.sin(angle),
                    math.cos(angle),
                )

                if abs(normalized_angle) <= front_limit:
                    valid_front.append(distance)

            angle += self.latest_scan.angle_increment

        minimum_all = min(valid_all) if valid_all else math.inf
        minimum_front = min(valid_front) if valid_front else math.inf

        return minimum_all, minimum_front

    def publish_stop(self):
        stop = Twist()

        # Send several zero commands so the stop is reliably received.
        for _ in range(10):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def wait_for_scan(self, timeout_sec=8.0):
        start = time.monotonic()

        while rclpy.ok() and self.latest_scan is None:
            rclpy.spin_once(self, timeout_sec=0.1)

            if time.monotonic() - start > timeout_sec:
                raise RuntimeError("No /scan received within 8 seconds.")

    def run_action(self, action_name):
        commands = {
            # Approximately 7.5 cm.
            "forward": (0.05, 0.0, 1.5),

            # Approximately 14 degrees.
            "left": (0.0, 0.25, 1.0),
            "right": (0.0, -0.25, 1.0),
        }

        linear_x, angular_z, duration = commands[action_name]

        self.wait_for_scan()

        minimum_all, minimum_front = self.get_clearances()

        self.get_logger().info(
            f"Action={action_name}, "
            f"minimum_all={minimum_all:.2f} m, "
            f"minimum_front={minimum_front:.2f} m"
        )

        if minimum_all < MIN_ALL_AROUND:
            raise RuntimeError(
                f"Motion blocked: obstacle is only {minimum_all:.2f} m away."
            )

        if action_name == "forward" and minimum_front < MIN_FRONT_FORWARD:
            raise RuntimeError(
                f"Forward blocked: front obstacle is "
                f"{minimum_front:.2f} m away."
            )

        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z

        start = time.monotonic()
        period = 1.0 / PUBLISH_RATE_HZ

        while rclpy.ok() and time.monotonic() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.01)

            minimum_all, minimum_front = self.get_clearances()

            if minimum_all < MIN_ALL_AROUND:
                self.get_logger().warning(
                    f"Emergency stop: minimum clearance "
                    f"{minimum_all:.2f} m."
                )
                break

            if (
                action_name == "forward"
                and minimum_front < MIN_FRONT_FORWARD
            ):
                self.get_logger().warning(
                    f"Emergency stop: front clearance "
                    f"{minimum_front:.2f} m."
                )
                break

            self.cmd_pub.publish(command)
            time.sleep(period)

        self.publish_stop()
        self.get_logger().info("Action finished. Zero velocity published.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--action",
        required=True,
        choices=["forward", "left", "right"],
    )
    args = parser.parse_args()

    rclpy.init()
    node = SafeMotionTest()

    try:
        node.run_action(args.action)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted. Stopping robot.")
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
