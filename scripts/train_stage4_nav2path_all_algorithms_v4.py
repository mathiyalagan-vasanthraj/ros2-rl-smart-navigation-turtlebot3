#!/usr/bin/env python3
import argparse
import csv
import math
import pickle
import random
import time
from collections import defaultdict, deque
from pathlib import Path

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

from stable_baselines3 import DQN, PPO, A2C
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
    idx2 = min(idx + 4, len(path) - 1)
    x1, y1 = path[idx]
    x2, y2 = path[idx2]
    return math.atan2(y2 - y1, x2 - x1)


class Stage4Nav2PathLocalControllerEnv(gym.Env):
    """
    RL local controller:
    - Input: /scan + robot pose + saved Nav2 global path
    - Output: /cmd_vel
    - Nav2 is NOT controlling the robot during training.
    """

    def __init__(
        self,
        algo_name="rl",
        path_csv="/ws_slam/nav2_rl_project/paths/stage4_global_path.csv",
        start_goal_csv="/ws_slam/nav2_rl_project/paths/stage4_start_goal.csv",
        results_dir="/ws_slam/nav2_rl_project/results",
        max_steps=700,
        step_time=0.10,
        lookahead_dist=0.60,
    ):
        super().__init__()

        self.algo_name = algo_name
        self.path_csv = Path(path_csv)
        self.start_goal_csv = Path(start_goal_csv)
        self.results_dir = Path(results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.max_steps = int(max_steps)
        self.step_time = float(step_time)
        self.lookahead_dist = float(lookahead_dist)

        self.max_lidar_range = 3.5
        self.num_lidar_bins = 24

        # Collision and safety
        self.wall_collision_threshold = 0.075
        self.dynamic_collision_distance = 0.36
        self.warning_scan_distance = 0.35
        self.warning_dynamic_distance = 0.70
        self.collision_grace_steps = 8

        # Path-following enforcement
        self.off_path_threshold = 1.25
        self.off_path_grace_steps = 25

        # Stuck detection
        self.stuck_window = 22
        self.stuck_move_threshold = 0.025

        self.goal_threshold = 0.30

        self.path = self.load_path()
        self.path_s = self.compute_path_s(self.path)
        self.total_path_length = self.path_s[-1]
        self.start_pose, self.goal_pose = self.load_start_goal()

        # 24 lidar bins + 8 navigation/path features
        obs_size = self.num_lidar_bins + 8
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(obs_size,),
            dtype=np.float32,
        )

        # 7 discrete local-controller actions
        self.action_space = spaces.Discrete(7)

        self.node = Node(f"stage4_nav2path_env_{algo_name}")

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

        self.episode_id = 0
        self.step_id = 0
        self.prev_goal_dist = None
        self.prev_path_index = 0
        self.prev_path_s = 0.0
        self.min_scan_ep = 999.0
        self.min_dyn_ep = 999.0
        self.pose_history = deque(maxlen=self.stuck_window)
        self.v_history = deque(maxlen=self.stuck_window)

        print(f"[ENV_V4:{self.algo_name}] Waiting for /set_entity_state...")
        if not self.set_state_cli.wait_for_service(timeout_sec=25.0):
            raise RuntimeError("Missing /set_entity_state. Start Gazebo dynamic ros_state world.")

        self.unpause_cli.wait_for_service(timeout_sec=5.0)

        print(f"[ENV_V4:{self.algo_name}] Waiting for /scan, /odom, /model_states...")
        t0 = time.time()
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_scan is not None and self.latest_odom is not None and self.latest_models is not None:
                break
            if time.time() - t0 > 25.0:
                raise RuntimeError("Timeout waiting for /scan, /odom, /model_states.")

        self.detect_dynamic_obstacles()

        print(f"[ENV_V4:{self.algo_name}] Ready.")
        print(f"[ENV_V4:{self.algo_name}] Path points: {len(self.path)}")
        print(f"[ENV_V4:{self.algo_name}] Path length: {self.total_path_length:.2f} m")
        print(f"[ENV_V4:{self.algo_name}] Start: {self.start_pose}")
        print(f"[ENV_V4:{self.algo_name}] Goal:  {self.goal_pose}")
        print(f"[ENV_V4:{self.algo_name}] Dynamic obstacles: {self.obstacle_names}")

    def load_path(self):
        pts = []
        with self.path_csv.open() as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append((float(row["x"]), float(row["y"])))
        if len(pts) < 5:
            raise RuntimeError("Saved Nav2 path is missing or too short.")
        return pts

    def compute_path_s(self, path):
        s = [0.0]
        for i in range(1, len(path)):
            d = math.hypot(path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
            s.append(s[-1] + d)
        return s

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
        vals = []
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.01:
                vals.append(min(float(r), self.max_lidar_range))
            else:
                vals.append(self.max_lidar_range)
        self.latest_scan = np.array(vals, dtype=np.float32)

    def odom_cb(self, msg):
        self.latest_odom = msg

    def model_cb(self, msg):
        self.latest_models = msg

    def detect_dynamic_obstacles(self):
        names = list(self.latest_models.name)
        self.obstacle_names = [
            n for n in names
            if "obstacle" in n.lower() and "burger" not in n.lower()
        ]
        if len(self.obstacle_names) == 0:
            print("[WARN] No dynamic obstacle models found.")
            print("[WARN] Models:", names)

    def unpause(self):
        if self.unpause_cli.service_is_ready():
            req = Empty.Request()
            fut = self.unpause_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=1.0)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

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
            best = min(best, math.hypot(ox - rx, oy - ry))
        return best

    def lidar_bins(self):
        scan = self.latest_scan
        if scan is None or len(scan) == 0:
            return np.ones(self.num_lidar_bins, dtype=np.float32)
        idxs = np.linspace(0, len(scan) - 1, self.num_lidar_bins).astype(int)
        vals = scan[idxs]
        vals = np.clip(vals, 0.0, self.max_lidar_range)
        return (vals / self.max_lidar_range).astype(np.float32)

    def nearest_path_index(self, x, y):
        # Monotonic-ish search: do not jump far backward.
        search_start = max(0, self.prev_path_index - 10)
        best_i = search_start
        best_d = 999.0

        for i in range(search_start, len(self.path)):
            px, py = self.path[i]
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d = d
                best_i = i

        return best_i, best_d

    def lookahead_point(self, nearest_i):
        target_s = self.path_s[nearest_i] + self.lookahead_dist
        for j in range(nearest_i, len(self.path)):
            if self.path_s[j] >= target_s:
                return j, self.path[j]
        return len(self.path) - 1, self.path[-1]

    def make_obs(self):
        x, y, yaw = self.get_robot_pose()
        nearest_i, cte = self.nearest_path_index(x, y)
        look_i, look = self.lookahead_point(nearest_i)

        lx, ly = look
        gx, gy, _ = self.goal_pose

        look_dist = math.hypot(lx - x, ly - y)
        look_angle = wrap_angle(math.atan2(ly - y, lx - x) - yaw)
        goal_dist = math.hypot(gx - x, gy - y)

        min_s = self.min_scan()
        dyn_d = self.min_dynamic_distance()
        progress_ratio = self.path_s[nearest_i] / max(self.total_path_length, 1e-6)

        obs = np.concatenate([
            self.lidar_bins(),
            np.array([
                min(look_dist / 2.0, 1.0),
                math.sin(look_angle),
                math.cos(look_angle),
                min(goal_dist / 5.0, 1.0),
                min(cte / 1.5, 1.0),
                min(min_s / self.max_lidar_range, 1.0),
                min(dyn_d / 2.0, 1.0),
                progress_ratio,
            ], dtype=np.float32)
        ])

        return obs.astype(np.float32), nearest_i, goal_dist, cte, min_s, dyn_d, progress_ratio

    def action_to_cmd(self, action):
        action = int(action)

        if action == 0:
            return 0.22, 0.0       # fast forward
        if action == 1:
            return 0.16, 0.75      # forward left
        if action == 2:
            return 0.16, -0.75     # forward right
        if action == 3:
            return 0.04, 1.20      # slow turn left
        if action == 4:
            return 0.04, -1.20     # slow turn right
        if action == 5:
            return 0.08, 0.0       # slow forward
        return 0.0, 0.0            # wait/stop

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
        self.prev_path_index = 0
        self.prev_path_s = 0.0
        self.pose_history.clear()
        self.v_history.clear()

        sx, sy, syaw = self.start_pose
        self.set_robot_pose(sx, sy, syaw)

        # Let Gazebo/model_states/scan settle after teleport.
        t0 = time.time()
        while time.time() - t0 < 1.0:
            self.stop_robot()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        obs, nearest_i, goal_dist, cte, min_s, dyn_d, progress_ratio = self.make_obs()

        self.prev_path_index = nearest_i
        self.prev_path_s = self.path_s[nearest_i]
        self.prev_goal_dist = goal_dist
        self.min_scan_ep = min_s
        self.min_dyn_ep = dyn_d

        x, y, _ = self.get_robot_pose()
        self.pose_history.append((x, y))
        self.v_history.append(0.0)

        print(
            f"[RESET_V4:{self.algo_name}] ep={self.episode_id} "
            f"x={x:.2f} y={y:.2f} goal_dist={goal_dist:.2f} "
            f"cte={cte:.2f} min_scan={min_s:.2f} dyn_dist={dyn_d:.2f} "
            f"path_idx={nearest_i}/{len(self.path)-1}",
            flush=True
        )

        return obs, {}

    def stuck_collision(self, min_s, dyn_d):
        if len(self.pose_history) < self.stuck_window:
            return False

        x0, y0 = self.pose_history[0]
        x1, y1 = self.pose_history[-1]
        moved = math.hypot(x1 - x0, y1 - y0)

        avg_forward_cmd = sum(abs(v) for v in self.v_history) / max(len(self.v_history), 1)

        return (
            moved < self.stuck_move_threshold
            and avg_forward_cmd > 0.07
            and (min_s < 0.18 or dyn_d < 0.50)
        )

    def step(self, action):
        self.unpause()
        self.step_id += 1

        v_cmd, w_cmd = self.publish_action(action)
        time.sleep(self.step_time)

        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0.01)

        x, y, _ = self.get_robot_pose()
        self.pose_history.append((x, y))
        self.v_history.append(v_cmd)

        obs, nearest_i, goal_dist, cte, min_s, dyn_d, progress_ratio = self.make_obs()

        self.min_scan_ep = min(self.min_scan_ep, min_s)
        self.min_dyn_ep = min(self.min_dyn_ep, dyn_d)

        current_s = self.path_s[nearest_i]
        path_delta = current_s - self.prev_path_s
        goal_progress = self.prev_goal_dist - goal_dist

        self.prev_goal_dist = goal_dist
        if current_s > self.prev_path_s:
            self.prev_path_s = current_s
        if nearest_i > self.prev_path_index:
            self.prev_path_index = nearest_i

        heading_cos = float(obs[self.num_lidar_bins + 2])

        reward = 0.0

        # Main path-following rewards
        reward += 18.0 * max(0.0, path_delta)
        reward += 8.0 * goal_progress
        reward += 0.10 * heading_cos

        # Strong path error penalty
        reward -= 1.20 * cte

        # Time/action penalties
        reward -= 0.03
        if int(action) in [3, 4]:
            reward -= 0.03
        if int(action) == 6 and dyn_d > 0.80 and min_s > 0.55:
            reward -= 0.08

        # Encourage clean forward movement only when aligned and safe
        if cte < 0.25 and heading_cos > 0.75 and min_s > 0.55 and dyn_d > 0.80 and int(action) == 0:
            reward += 0.08

        # Safety penalty from scan
        if min_s < self.warning_scan_distance:
            danger = (self.warning_scan_distance - min_s) / self.warning_scan_distance
            reward -= 5.0 * (danger ** 2)

        # Safety penalty from dynamic obstacles
        if dyn_d < self.warning_dynamic_distance:
            danger = (self.warning_dynamic_distance - dyn_d) / self.warning_dynamic_distance
            reward -= 8.0 * (danger ** 2)

        terminated = False
        truncated = False
        status = "running"
        reason = ""

        if goal_dist < self.goal_threshold:
            reward += 400.0
            terminated = True
            status = "success"
            reason = "goal_reached"

        elif self.step_id > self.collision_grace_steps:
            wall_collision = min_s < self.wall_collision_threshold
            dynamic_collision = dyn_d < self.dynamic_collision_distance
            stuck_collision = self.stuck_collision(min_s, dyn_d)

            if wall_collision or dynamic_collision or stuck_collision:
                reward -= 400.0
                terminated = True
                status = "collision"

                if wall_collision:
                    reason = f"wall_scan_{min_s:.2f}"
                elif dynamic_collision:
                    reason = f"dynamic_dist_{dyn_d:.2f}"
                else:
                    reason = "stuck_blocked"

        if (
            not terminated
            and self.step_id > self.off_path_grace_steps
            and cte > self.off_path_threshold
        ):
            reward -= 180.0
            terminated = True
            status = "off_path"
            reason = f"cte_{cte:.2f}"

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
            "min_clearance": float(self.min_scan_ep),
            "min_dynamic_distance": float(self.min_dyn_ep),
            "nearest_path_index": int(nearest_i),
            "progress_ratio": float(progress_ratio),
        }

        if self.step_id % 50 == 0 and not terminated and not truncated:
            print(
                f"[STEP_V4:{self.algo_name}] ep={self.episode_id} step={self.step_id} "
                f"path_idx={nearest_i}/{len(self.path)-1} progress={progress_ratio:.2f} "
                f"goal={goal_dist:.2f} cte={cte:.2f} "
                f"scan={min_s:.2f} dyn={dyn_d:.2f}",
                flush=True
            )

        if terminated or truncated:
            self.stop_robot()
            print(
                f"[END_V4:{self.algo_name}] ep={self.episode_id} status={status} reason={reason} "
                f"steps={self.step_id} goal_dist={goal_dist:.2f} "
                f"cte={cte:.2f} min_scan_ep={self.min_scan_ep:.2f} "
                f"dyn_ep={self.min_dyn_ep:.2f} path_idx={nearest_i}/{len(self.path)-1} "
                f"progress={progress_ratio:.2f}",
                flush=True
            )

        return obs, float(reward), terminated, truncated, info

    def close(self):
        self.stop_robot()
        self.node.destroy_node()


class MetricsCallback(BaseCallback):
    def __init__(self, results_dir, algo_name):
        super().__init__()
        self.csv_path = Path(results_dir) / f"{algo_name}_v4_train_metrics.csv"
        self.ep = 0

        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "episode", "status", "reason", "steps",
                "goal_dist", "cross_track_error",
                "min_clearance", "min_dynamic_distance",
                "nearest_path_index", "progress_ratio"
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
                        info.get("progress_ratio", ""),
                    ])
        return True


def obs_to_q_state(env, obs):
    n = env.num_lidar_bins

    look_sin = obs[n + 1]
    look_cos = obs[n + 2]
    cte = obs[n + 4]
    scan = obs[n + 5]
    dyn = obs[n + 6]
    prog = obs[n + 7]

    angle = math.atan2(float(look_sin), float(look_cos))

    angle_bin = int(np.digitize(angle, [-1.2, -0.4, 0.4, 1.2]))
    cte_bin = int(np.digitize(float(cte), [0.15, 0.35, 0.65, 0.9]))
    scan_bin = int(np.digitize(float(scan), [0.08, 0.15, 0.30, 0.60]))
    dyn_bin = int(np.digitize(float(dyn), [0.20, 0.35, 0.50, 0.80]))
    prog_bin = int(np.digitize(float(prog), [0.20, 0.40, 0.60, 0.80]))

    return (angle_bin, cte_bin, scan_bin, dyn_bin, prog_bin)


def train_qlearning(env, episodes, max_steps, results_dir):
    algo = "qlearning"
    q_path = Path(results_dir) / "qlearning_v4_qtable.pkl"
    metrics_path = Path(results_dir) / "qlearning_v4_train_metrics.csv"

    Q = defaultdict(lambda: np.zeros(env.action_space.n, dtype=np.float32))

    alpha = 0.15
    gamma = 0.98
    epsilon = 1.0
    epsilon_min = 0.05
    epsilon_decay = 0.985

    with metrics_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "episode", "status", "reason", "steps",
            "goal_dist", "cross_track_error", "min_clearance",
            "min_dynamic_distance", "nearest_path_index", "progress_ratio"
        ])

    for ep in range(1, episodes + 1):
        obs, _ = env.reset()
        state = obs_to_q_state(env, obs)

        last_info = {}
        total_reward = 0.0

        for step in range(1, max_steps + 1):
            if random.random() < epsilon:
                action = env.action_space.sample()
            else:
                action = int(np.argmax(Q[state]))

            next_obs, reward, terminated, truncated, info = env.step(action)
            next_state = obs_to_q_state(env, next_obs)

            done = terminated or truncated

            best_next = float(np.max(Q[next_state]))
            Q[state][action] += alpha * (reward + gamma * best_next * (not done) - Q[state][action])

            state = next_state
            total_reward += reward
            last_info = info

            if done:
                break

        epsilon = max(epsilon_min, epsilon * epsilon_decay)

        with metrics_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                ep,
                last_info.get("status", ""),
                last_info.get("reason", ""),
                last_info.get("steps", ""),
                last_info.get("goal_dist", ""),
                last_info.get("cross_track_error", ""),
                last_info.get("min_clearance", ""),
                last_info.get("min_dynamic_distance", ""),
                last_info.get("nearest_path_index", ""),
                last_info.get("progress_ratio", ""),
            ])

        print(
            f"[QLEARNING] ep={ep}/{episodes} reward={total_reward:.2f} "
            f"eps={epsilon:.2f} status={last_info.get('status')} "
            f"reason={last_info.get('reason')}",
            flush=True
        )

    with q_path.open("wb") as f:
        pickle.dump(dict(Q), f)

    print(f"[SAVE] Q-learning table saved to {q_path}")


def train_sb3_algo(algo_name, env, episodes, max_steps, results_dir):
    total_timesteps = int(episodes * max_steps)
    callback = MetricsCallback(results_dir, algo_name)

    if algo_name == "dqn":
        model = DQN(
            "MlpPolicy",
            env,
            learning_rate=1e-4,
            buffer_size=60000,
            learning_starts=1200,
            batch_size=64,
            gamma=0.99,
            train_freq=4,
            gradient_steps=1,
            target_update_interval=500,
            exploration_fraction=0.35,
            exploration_initial_eps=1.0,
            exploration_final_eps=0.05,
            verbose=1,
            tensorboard_log=f"{results_dir}/tb_logs_v4",
        )

    elif algo_name == "ppo":
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=256,
            batch_size=64,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            verbose=1,
            tensorboard_log=f"{results_dir}/tb_logs_v4",
        )

    elif algo_name == "a2c":
        model = A2C(
            "MlpPolicy",
            env,
            learning_rate=7e-4,
            n_steps=32,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.01,
            verbose=1,
            tensorboard_log=f"{results_dir}/tb_logs_v4",
        )

    else:
        raise ValueError(f"Unknown SB3 algorithm: {algo_name}")

    print("============================================")
    print(f"TRAINING {algo_name.upper()} V4")
    print(f"Episodes approx: {episodes}")
    print(f"Max steps per episode: {max_steps}")
    print(f"Total timesteps: {total_timesteps}")
    print("RL follows saved Nav2 path using path-progress reward.")
    print("RL publishes /cmd_vel directly.")
    print("============================================")

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
        progress_bar=False,
    )

    out = Path(results_dir) / f"{algo_name}_v4_nav2path_local_controller"
    model.save(str(out))
    print(f"[SAVE] {algo_name.upper()} model saved to {out}.zip")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algos", type=str, default="dqn")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--max_steps", type=int, default=700)
    parser.add_argument("--step_time", type=float, default=0.10)
    parser.add_argument("--lookahead_dist", type=float, default=0.60)
    args = parser.parse_args()

    results_dir = "/ws_slam/nav2_rl_project/results"
    Path(results_dir).mkdir(parents=True, exist_ok=True)

    algos = [a.strip().lower() for a in args.algos.split(",") if a.strip()]

    rclpy.init()

    try:
        for algo in algos:
            print(f"\n\n========== STARTING {algo.upper()} ==========\n")

            env = Stage4Nav2PathLocalControllerEnv(
                algo_name=algo,
                results_dir=results_dir,
                max_steps=args.max_steps,
                step_time=args.step_time,
                lookahead_dist=args.lookahead_dist,
            )

            if algo in ["dqn", "ppo", "a2c"]:
                train_sb3_algo(algo, env, args.episodes, args.max_steps, results_dir)
            elif algo in ["qlearning", "q"]:
                train_qlearning(env, args.episodes, args.max_steps, results_dir)
            else:
                print(f"[WARN] Unknown algo skipped: {algo}")

            env.close()
            time.sleep(1.0)

    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print("\nALL REQUESTED TRAINING FINISHED.")


if __name__ == "__main__":
    main()
