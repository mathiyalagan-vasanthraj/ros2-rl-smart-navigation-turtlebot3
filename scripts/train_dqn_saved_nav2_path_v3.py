#!/usr/bin/env python3
import argparse
import csv
import math
import time
from pathlib import Path
from collections import deque

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except Exception:
    import gym
    from gym import spaces

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState
from std_srvs.srv import Empty

from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import BaseCallback


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def path_yaw(path, idx):
    idx2 = min(idx + 3, len(path) - 1)
    x1, y1 = path[idx]
    x2, y2 = path[idx2]
    return math.atan2(y2 - y1, x2 - x1)


class DQNStage4Nav2PathEnv(gym.Env):
    def __init__(
        self,
        path_csv="/ws_slam/nav2_rl_project/paths/stage4_global_path.csv",
        start_goal_csv="/ws_slam/nav2_rl_project/paths/stage4_start_goal.csv",
        results_dir="/ws_slam/nav2_rl_project/results",
        max_steps=700,
        step_time=0.10,
        lookahead_dist=0.55,
    ):
        super().__init__()

        self.path_csv = Path(path_csv)
        self.start_goal_csv = Path(start_goal_csv)
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.max_steps = max_steps
        self.step_time = step_time
        self.lookahead_dist = lookahead_dist

        self.max_lidar_range = 3.5
        self.num_lidar_bins = 24

        # Collision settings
        self.wall_collision_threshold = 0.095
        self.dynamic_collision_distance = 0.38
        self.warning_scan_distance = 0.35
        self.warning_dynamic_distance = 0.65
        self.collision_grace_steps = 8

        # Stuck detection
        self.stuck_window = 22
        self.stuck_move_threshold = 0.025

        self.goal_threshold = 0.30

        self.path = self.load_path()
        self.start_pose, self.goal_pose = self.load_start_goal()

        # 24 lidar bins + 7 compact navigation/path features
        obs_size = self.num_lidar_bins + 7
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(obs_size,),
            dtype=np.float32,
        )

        # 6 discrete velocity actions
        self.action_space = spaces.Discrete(6)

        self.node = Node("dqn_stage4_nav2_path_env_v3")

        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.node.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.node.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.node.create_subscription(ModelStates, "/model_states", self.model_cb, 10)

        self.set_state_cli = self.node.create_client(SetEntityState, "/set_entity_state")
        self.unpause_cli = self.node.create_client(Empty, "/unpause_physics")

        self.latest_scan = None
        self.latest_odom = None
        self.latest_models = None

        self.obstacle_names = []

        print("[ENV_V3] Waiting for /set_entity_state...")
        if not self.set_state_cli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("Missing /set_entity_state. Start dynamic Gazebo with ros_state world.")

        self.unpause_cli.wait_for_service(timeout_sec=5.0)

        print("[ENV_V3] Waiting for /scan, /odom, /model_states...")
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if (
                self.latest_scan is not None
                and self.latest_odom is not None
                and self.latest_models is not None
            ):
                break

            if time.time() - t0 > 25.0:
                raise RuntimeError("Timeout waiting for /scan, /odom, /model_states.")

        self.detect_obstacles()

        print("[ENV_V3] Ready.")
        print(f"[ENV_V3] Path points: {len(self.path)}")
        print(f"[ENV_V3] Start pose: {self.start_pose}")
        print(f"[ENV_V3] Goal pose: {self.goal_pose}")
        print(f"[ENV_V3] Dynamic obstacle models: {self.obstacle_names}")

        self.episode_id = 0
        self.step_id = 0
        self.prev_goal_dist = None
        self.prev_path_index = 0
        self.min_clearance_ep = 999.0
        self.min_dynamic_ep = 999.0
        self.pose_history = deque(maxlen=self.stuck_window)

    def load_path(self):
        pts = []
        with self.path_csv.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append((float(row["x"]), float(row["y"])))

        if len(pts) < 5:
            raise RuntimeError("Global path is missing or too short.")

        return pts

    def load_start_goal(self):
        start = None
        goal = None

        with self.start_goal_csv.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose = (float(row["x"]), float(row["y"]), float(row["yaw"]))
                if row["type"] == "start":
                    start = pose
                elif row["type"] == "goal":
                    goal = pose

        if start is None or goal is None:
            raise RuntimeError("Missing start/goal csv.")

        return start, goal

    def scan_cb(self, msg):
        values = []
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.01:
                values.append(min(float(r), self.max_lidar_range))
            else:
                values.append(self.max_lidar_range)

        self.latest_scan = np.array(values, dtype=np.float32)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def model_cb(self, msg):
        self.latest_models = msg

    def detect_obstacles(self):
        names = list(self.latest_models.name)
        self.obstacle_names = [
            n for n in names
            if "obstacle" in n.lower() and "burger" not in n.lower()
        ]

        if len(self.obstacle_names) == 0:
            print("[WARN] No obstacle models found from /model_states.")
            print("[WARN] Available models:", names)

    def unpause(self):
        if self.unpause_cli.service_is_ready():
            req = Empty.Request()
            fut = self.unpause_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=1.0)

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def set_robot_pose(self, x, y, yaw):
        state = EntityState()
        state.name = "burger"
        state.reference_frame = "world"

        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = 0.12

        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)

        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.angular.z = 0.0

        req = SetEntityState.Request()
        req.state = state

        fut = self.set_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)

        self.stop_robot()

    def model_pose(self, name):
        if self.latest_models is None:
            return None

        try:
            i = list(self.latest_models.name).index(name)
        except ValueError:
            return None

        p = self.latest_models.pose[i].position
        q = self.latest_models.pose[i].orientation
        return float(p.x), float(p.y), yaw_from_quat(q)

    def get_robot_pose(self):
        pose = self.model_pose("burger")
        if pose is not None:
            return pose

        # fallback to odom
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        return float(p.x), float(p.y), yaw_from_quat(q)

    def min_scan(self):
        if self.latest_scan is None or len(self.latest_scan) == 0:
            return self.max_lidar_range
        return float(np.min(self.latest_scan))

    def min_dynamic_distance(self):
        rx, ry, _ = self.get_robot_pose()
        best = 999.0

        for name in self.obstacle_names:
            pose = self.model_pose(name)
            if pose is None:
                continue

            ox, oy, _ = pose
            d = math.hypot(ox - rx, oy - ry)
            best = min(best, d)

        return best

    def lidar_bins(self):
        scan = self.latest_scan
        if scan is None or len(scan) == 0:
            return np.ones(self.num_lidar_bins, dtype=np.float32)

        idxs = np.linspace(0, len(scan) - 1, self.num_lidar_bins).astype(int)
        vals = scan[idxs]
        vals = np.clip(vals, 0.0, self.max_lidar_range)
        vals = vals / self.max_lidar_range
        return vals.astype(np.float32)

    def nearest_path_index(self, x, y):
        best_i = 0
        best_d = 999.0

        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d = d
                best_i = i

        return best_i, best_d

    def lookahead_point(self, nearest_i):
        accum = 0.0
        prev = self.path[nearest_i]

        for j in range(nearest_i + 1, len(self.path)):
            cur = self.path[j]
            accum += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            if accum >= self.lookahead_dist:
                return j, cur
            prev = cur

        return len(self.path) - 1, self.path[-1]

    def make_obs(self):
        x, y, yaw = self.get_robot_pose()

        nearest_i, cte = self.nearest_path_index(x, y)
        look_i, look = self.lookahead_point(nearest_i)

        lx, ly = look
        gx, gy, _ = self.goal_pose

        dx_l = lx - x
        dy_l = ly - y

        look_dist = math.hypot(dx_l, dy_l)
        look_angle = wrap_angle(math.atan2(dy_l, dx_l) - yaw)

        goal_dist = math.hypot(gx - x, gy - y)

        min_s = self.min_scan()
        dyn_d = self.min_dynamic_distance()

        obs = np.concatenate([
            self.lidar_bins(),
            np.array([
                min(look_dist / 2.0, 1.0),
                math.sin(look_angle),
                math.cos(look_angle),
                min(goal_dist / 5.0, 1.0),
                min(cte / 1.0, 1.0),
                min(min_s / self.max_lidar_range, 1.0),
                min(dyn_d / 2.0, 1.0),
            ], dtype=np.float32)
        ])

        return obs.astype(np.float32), nearest_i, goal_dist, cte, min_s, dyn_d

    def action_to_cmd(self, action):
        action = int(action)

        if action == 0:
            return 0.22, 0.0
        if action == 1:
            return 0.16, 0.90
        if action == 2:
            return 0.16, -0.90
        if action == 3:
            return 0.00, 1.30
        if action == 4:
            return 0.00, -1.30

        return 0.05, 0.0

    def publish_action(self, action):
        v, w = self.action_to_cmd(action)

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)

        self.cmd_pub.publish(msg)

        return v, w

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.unpause()
        self.stop_robot()

        self.episode_id += 1
        self.step_id = 0
        self.pose_history.clear()

        sx, sy, syaw = self.start_pose
        self.set_robot_pose(sx, sy, syaw)

        # let Gazebo, scan, model_states update after teleport
        t0 = time.time()
        while time.time() - t0 < 1.0:
            self.stop_robot()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        obs, nearest_i, goal_dist, cte, min_s, dyn_d = self.make_obs()

        self.prev_goal_dist = goal_dist
        self.prev_path_index = nearest_i
        self.min_clearance_ep = min_s
        self.min_dynamic_ep = dyn_d

        x, y, _ = self.get_robot_pose()
        self.pose_history.append((x, y))

        print(
            f"[RESET_V3] ep={self.episode_id} "
            f"x={x:.2f} y={y:.2f} "
            f"goal_dist={goal_dist:.2f} cte={cte:.2f} "
            f"min_scan={min_s:.2f} dyn_dist={dyn_d:.2f}",
            flush=True
        )

        return obs, {}

    def stuck_check(self):
        if len(self.pose_history) < self.stuck_window:
            return False

        x0, y0 = self.pose_history[0]
        x1, y1 = self.pose_history[-1]
        moved = math.hypot(x1 - x0, y1 - y0)

        return moved < self.stuck_move_threshold

    def step(self, action):
        self.unpause()
        self.step_id += 1

        v, w = self.publish_action(action)
        time.sleep(self.step_time)

        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0.01)

        x, y, _ = self.get_robot_pose()
        self.pose_history.append((x, y))

        obs, nearest_i, goal_dist, cte, min_s, dyn_d = self.make_obs()

        self.min_clearance_ep = min(self.min_clearance_ep, min_s)
        self.min_dynamic_ep = min(self.min_dynamic_ep, dyn_d)

        progress_goal = self.prev_goal_dist - goal_dist
        progress_path = max(0, nearest_i - self.prev_path_index)
        backward_path = max(0, self.prev_path_index - nearest_i)

        self.prev_goal_dist = goal_dist
        if nearest_i > self.prev_path_index:
            self.prev_path_index = nearest_i

        heading_cos = float(obs[self.num_lidar_bins + 2])

        reward = 0.0

        # Progress rewards
        reward += 14.0 * progress_goal
        reward += 0.12 * progress_path
        reward += 0.06 * heading_cos

        # Path-following penalty
        reward -= 0.70 * cte
        reward -= 0.10 * backward_path

        # Small time penalty
        reward -= 0.03

        # Avoid useless spinning
        if int(action) in [3, 4]:
            reward -= 0.02

        # Encourage forward movement when aligned and safe
        if min_s > 0.65 and dyn_d > 0.75 and cte < 0.25 and int(action) == 0:
            reward += 0.05

        # Wall/static obstacle safety penalty
        if min_s < self.warning_scan_distance:
            danger = (self.warning_scan_distance - min_s) / self.warning_scan_distance
            reward -= 5.0 * (danger ** 2)

        # Dynamic obstacle safety penalty
        if dyn_d < self.warning_dynamic_distance:
            danger = (self.warning_dynamic_distance - dyn_d) / self.warning_dynamic_distance
            reward -= 8.0 * (danger ** 2)

        terminated = False
        truncated = False
        status = "running"
        reason = ""

        # Success
        if goal_dist < self.goal_threshold:
            reward += 350.0
            terminated = True
            status = "success"
            reason = "goal_reached"

        # Collision checks after grace steps
        elif self.step_id > self.collision_grace_steps:
            wall_collision = min_s < self.wall_collision_threshold
            dynamic_collision = dyn_d < self.dynamic_collision_distance
            stuck_collision = (
                self.step_id > self.stuck_window
                and self.stuck_check()
                and (min_s < 0.18 or dyn_d < 0.48)
            )

            if wall_collision or dynamic_collision or stuck_collision:
                reward -= 350.0
                terminated = True
                status = "collision"

                if wall_collision:
                    reason = f"wall_scan_{min_s:.2f}"
                elif dynamic_collision:
                    reason = f"dynamic_dist_{dyn_d:.2f}"
                else:
                    reason = "stuck_blocked"

        # Timeout
        if not terminated and self.step_id >= self.max_steps:
            reward -= 80.0
            truncated = True
            status = "timeout"
            reason = "max_steps"

        info = {
            "status": status,
            "reason": reason,
            "steps": self.step_id,
            "goal_dist": float(goal_dist),
            "cross_track_error": float(cte),
            "min_clearance": float(self.min_clearance_ep),
            "min_dynamic_distance": float(self.min_dynamic_ep),
            "nearest_path_index": int(nearest_i),
        }

        if terminated or truncated:
            self.stop_robot()
            print(
                f"[END_V3] ep={self.episode_id} status={status} reason={reason} "
                f"steps={self.step_id} goal_dist={goal_dist:.2f} "
                f"cte={cte:.2f} min_scan_ep={self.min_clearance_ep:.2f} "
                f"dyn_ep={self.min_dynamic_ep:.2f} path_idx={nearest_i}",
                flush=True
            )

        return obs, float(reward), terminated, truncated, info

    def close(self):
        self.stop_robot()
        self.node.destroy_node()


class MetricsCallback(BaseCallback):
    def __init__(self, results_dir):
        super().__init__()
        self.csv_path = Path(results_dir) / "dqn_saved_nav2_path_v3_train_metrics.csv"
        self.ep = 0

        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "episode",
                "status",
                "reason",
                "steps",
                "goal_dist",
                "cross_track_error",
                "min_clearance",
                "min_dynamic_distance",
                "nearest_path_index"
            ])

    def _on_step(self):
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])

        for done, info in zip(dones, infos):
            if done:
                self.ep += 1

                with self.csv_path.open("a", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        self.ep,
                        info.get("status", ""),
                        info.get("reason", ""),
                        info.get("steps", ""),
                        info.get("goal_dist", ""),
                        info.get("cross_track_error", ""),
                        info.get("min_clearance", ""),
                        info.get("min_dynamic_distance", ""),
                        info.get("nearest_path_index", ""),
                    ])

        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--max_steps", type=int, default=700)
    parser.add_argument("--step_time", type=float, default=0.10)
    parser.add_argument("--lookahead_dist", type=float, default=0.55)
    args = parser.parse_args()

    results_dir = "/ws_slam/nav2_rl_project/results"
    Path(results_dir).mkdir(parents=True, exist_ok=True)

    rclpy.init()

    env = DQNStage4Nav2PathEnv(
        results_dir=results_dir,
        max_steps=args.max_steps,
        step_time=args.step_time,
        lookahead_dist=args.lookahead_dist,
    )

    model = DQN(
        "MlpPolicy",
        env,
        learning_rate=1e-4,
        buffer_size=50000,
        learning_starts=1000,
        batch_size=64,
        gamma=0.99,
        train_freq=4,
        gradient_steps=1,
        target_update_interval=500,
        exploration_fraction=0.35,
        exploration_initial_eps=1.0,
        exploration_final_eps=0.05,
        verbose=1,
        tensorboard_log="/ws_slam/nav2_rl_project/results/tb_logs_v3",
    )

    total_timesteps = args.episodes * args.max_steps

    print("============================================")
    print("DQN V3 TRAINING: NAV2 PATH LOCAL CONTROLLER")
    print(f"Episodes approx: {args.episodes}")
    print(f"Max steps per episode: {args.max_steps}")
    print(f"Total timesteps: {total_timesteps}")
    print("Collision checks:")
    print("  wall collision from /scan")
    print("  dynamic obstacle collision from /model_states")
    print("  stuck collision if robot is blocked")
    print("============================================")

    callback = MetricsCallback(results_dir)

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
        progress_bar=False,
    )

    out = Path(results_dir) / "dqn_saved_nav2_path_v3_local_controller"
    model.save(str(out))

    env.close()

    try:
        rclpy.shutdown()
    except Exception:
        pass

    print(f"[SAVE] DQN V3 model saved to {out}.zip")


if __name__ == "__main__":
    main()
