#!/usr/bin/env python3

import csv
import math
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from visualization_msgs.msg import Marker, MarkerArray


PATH_FILE = Path(
    "/ws_slam/nav2_rl_project/paths/stage4_global_path.csv"
)

# The saved map/path and Gazebo world coordinates use the same coordinates
# in this project, so the overlay is published in the map frame.
FRAME_ID = "map"


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SavedPathOverlay(Node):
    def __init__(self):
        super().__init__("saved_nav2_path_overlay")

        self.path_points = self.load_path()
        self.robot_pose = None
        self.previous_pose = None
        self.trail = deque(maxlen=3000)

        self.saved_path_pub = self.create_publisher(
            PathMsg, "/saved_nav2_path", 10
        )
        self.robot_trail_pub = self.create_publisher(
            PathMsg, "/robot_trail", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/nav2_rl_markers", 10
        )

        self.create_subscription(
            ModelStates,
            "/model_states",
            self.model_states_callback,
            10,
        )

        self.create_timer(0.2, self.publish_visualization)

        self.get_logger().info("RViz saved-path overlay started.")
        self.get_logger().info(
            f"Loaded {len(self.path_points)} path points."
        )

    def load_path(self):
        points = []

        with PATH_FILE.open(newline="") as file:
            reader = csv.DictReader(file)
            for row in reader:
                points.append((float(row["x"]), float(row["y"])))

        if len(points) < 2:
            raise RuntimeError(f"Invalid saved path: {PATH_FILE}")

        return points

    def model_states_callback(self, message):
        try:
            index = list(message.name).index("burger")
        except ValueError:
            return

        position = message.pose[index].position
        orientation = message.pose[index].orientation

        x = float(position.x)
        y = float(position.y)
        yaw = quaternion_to_yaw(orientation)

        # A large jump means that the evaluation environment teleported
        # the robot back to the start for a new episode.
        if self.previous_pose is not None:
            previous_x, previous_y = self.previous_pose
            jump = math.hypot(x - previous_x, y - previous_y)

            if jump > 0.8:
                self.trail.clear()

        self.previous_pose = (x, y)
        self.robot_pose = (x, y, yaw)
        self.trail.append((x, y))

    def nearest_path_information(self, x, y):
        nearest_index = 0
        cross_track_error = float("inf")

        for index, (path_x, path_y) in enumerate(self.path_points):
            distance = math.hypot(path_x - x, path_y - y)

            if distance < cross_track_error:
                cross_track_error = distance
                nearest_index = index

        denominator = max(len(self.path_points) - 1, 1)
        progress = nearest_index / denominator

        return nearest_index, cross_track_error, progress

    def create_path_message(self, points):
        message = PathMsg()
        message.header.frame_id = FRAME_ID
        message.header.stamp = self.get_clock().now().to_msg()

        for x, y in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.03
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        return message

    def create_sphere(self, marker_id, x, y, red, green, blue):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "start_goal"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.26
        marker.scale.y = 0.26
        marker.scale.z = 0.26

        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        return marker

    def create_robot_arrow(self, x, y, yaw):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot"
        marker.id = 10
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.15
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        marker.scale.x = 0.45
        marker.scale.y = 0.09
        marker.scale.z = 0.09

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_text_marker(self, x, y, text):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "navigation_status"
        marker.id = 20
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.65
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.20

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text

        return marker

    def publish_visualization(self):
        self.saved_path_pub.publish(
            self.create_path_message(self.path_points)
        )

        self.robot_trail_pub.publish(
            self.create_path_message(list(self.trail))
        )

        markers = MarkerArray()

        start_x, start_y = self.path_points[0]
        goal_x, goal_y = self.path_points[-1]

        markers.markers.append(
            self.create_sphere(
                marker_id=1,
                x=start_x,
                y=start_y,
                red=0.0,
                green=1.0,
                blue=0.0,
            )
        )

        markers.markers.append(
            self.create_sphere(
                marker_id=2,
                x=goal_x,
                y=goal_y,
                red=1.0,
                green=0.0,
                blue=0.0,
            )
        )

        if self.robot_pose is not None:
            x, y, yaw = self.robot_pose
            index, cte, progress = self.nearest_path_information(x, y)

            markers.markers.append(
                self.create_robot_arrow(x, y, yaw)
            )

            status_text = (
                f"Path: {index}/{len(self.path_points) - 1}\n"
                f"Progress: {progress * 100.0:.1f}%\n"
                f"CTE: {cte:.2f} m"
            )

            markers.markers.append(
                self.create_text_marker(x, y, status_text)
            )

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = SavedPathOverlay()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
