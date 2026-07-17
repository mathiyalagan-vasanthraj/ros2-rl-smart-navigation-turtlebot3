#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


TAU = 2.0 * math.pi


class ScanNormalizer(Node):
    def __init__(self):
        super().__init__("real_scan_normalizer")

        self.declare_parameter("target_beams", 224)
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_fixed")

        self.target_beams = int(
            self.get_parameter("target_beams").value
        )
        input_topic = str(
            self.get_parameter("input_topic").value
        )
        output_topic = str(
            self.get_parameter("output_topic").value
        )

        if self.target_beams < 2:
            raise ValueError("target_beams must be at least 2")

        self.output_angle_min = 0.0
        self.output_angle_increment = TAU / self.target_beams
        self.output_angle_max = (
            self.output_angle_min
            + (self.target_beams - 1)
            * self.output_angle_increment
        )

        self.publisher = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.message_count = 0

        self.get_logger().info(
            f"Reading {input_topic}; publishing {output_topic} "
            f"with exactly {self.target_beams} beams."
        )

    def scan_callback(self, msg: LaserScan):
        source_count = len(msg.ranges)

        if source_count < 2:
            self.get_logger().warning(
                f"Ignoring scan with only {source_count} readings."
            )
            return

        if abs(msg.angle_increment) < 1e-9:
            self.get_logger().warning(
                "Ignoring scan with invalid angle_increment."
            )
            return

        source_angles = (
            msg.angle_min
            + np.arange(source_count, dtype=np.float64)
            * msg.angle_increment
        ) % TAU

        target_angles = (
            self.output_angle_min
            + np.arange(self.target_beams, dtype=np.float64)
            * self.output_angle_increment
        )

        # Circular angular distance between each fixed output angle
        # and each incoming LiDAR angle.
        differences = np.abs(
            (
                target_angles[:, None]
                - source_angles[None, :]
                + math.pi
            )
            % TAU
            - math.pi
        )

        nearest_indices = np.argmin(differences, axis=1)

        source_ranges = np.asarray(
            msg.ranges,
            dtype=np.float32,
        )

        valid_ranges = (
            np.isfinite(source_ranges)
            & (source_ranges >= msg.range_min)
            & (source_ranges <= msg.range_max)
        )

        source_ranges = np.where(
            valid_ranges,
            source_ranges,
            np.inf,
        )

        output = LaserScan()
        output.header = msg.header

        output.angle_min = self.output_angle_min
        output.angle_max = self.output_angle_max
        output.angle_increment = self.output_angle_increment

        output.scan_time = msg.scan_time

        if msg.scan_time > 0.0:
            output.time_increment = (
                msg.scan_time / self.target_beams
            )
        else:
            output.time_increment = 0.0

        output.range_min = msg.range_min
        output.range_max = msg.range_max

        output.ranges = (
            source_ranges[nearest_indices]
            .astype(np.float32)
            .tolist()
        )

        if len(msg.intensities) == source_count:
            source_intensities = np.asarray(
                msg.intensities,
                dtype=np.float32,
            )
            output.intensities = (
                source_intensities[nearest_indices]
                .astype(np.float32)
                .tolist()
            )
        else:
            output.intensities = []

        self.publisher.publish(output)

        self.message_count += 1

        if self.message_count % 30 == 0:
            self.get_logger().info(
                f"Published scan {self.message_count}: "
                f"input={source_count}, "
                f"output={len(output.ranges)}, "
                f"frame={output.header.frame_id}"
            )


def main():
    rclpy.init()
    node = ScanNormalizer()

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
