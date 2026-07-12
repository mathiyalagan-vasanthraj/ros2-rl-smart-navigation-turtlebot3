#!/usr/bin/env python3
import argparse
import csv
import importlib.util
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import BaseCallback


OLD_SCRIPT = "/ws_slam/nav2_rl_project/scripts/train_dqn_saved_nav2_path.py"

spec = importlib.util.spec_from_file_location("old_train_script", OLD_SCRIPT)
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)


def path_yaw(path, idx):
    idx2 = min(idx + 3, len(path) - 1)
    x1, y1 = path[idx]
    x2, y2 = path[idx2]
    return math.atan2(y2 - y1, x2 - x1)


class DQNSavedPathEnvV2(old.DQNSavedPathEnv):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Close-wall start settings
        self.reset_safe_distance = 0.05
        self.warning_distance = 0.35
        self.collision_threshold = 0.105
        self.collision_grace_steps = 12

        # Use ONLY the original saved start path point
        self.reset_candidate_indices = [0]

        print("[V2] Fixed start mode.")
        print("[V2] Robot starts at saved start pose, even if close to wall.")
        print("[V2] Collision check starts after short grace time.")

    def spin_sensors(self, duration=0.6):
        t0 = time.time()
        while time.time() - t0 < duration and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def publish_zero(self, n=3):
        msg = Twist()
        for _ in range(n):
            self.cmd_pub.publish(msg)
            time.sleep(0.03)

    def set_path_pose(self, idx):
        idx = max(0, min(idx, len(self.path) - 1))
        x, y = self.path[idx]
        yaw = path_yaw(self.path, idx)
        self.set_robot_pose(x, y, yaw)
        self.publish_zero()
        self.spin_sensors(0.8)
        return idx, x, y, yaw, self.min_scan()

    def reset(self, seed=None, options=None):
        # Do not call old reset, because old reset immediately ends near wall.
        try:
            self.unpause()
        except Exception:
            pass

        self.stop_robot()

        self.episode_id += 1
        self.step_id = 0
        self.min_clearance_ep = 999.0

        idx, x, y, yaw, min_s = self.set_path_pose(0)

        obs, nearest_i, goal_dist, cte, min_s = self.make_obs()

        self.prev_goal_dist = goal_dist
        self.prev_path_index = nearest_i
        self.min_clearance_ep = min_s

        print(
            f"[RESET_V2] ep={self.episode_id} "
            f"start_idx={idx} x={x:.2f} y={y:.2f} "
            f"goal_dist={goal_dist:.2f} cte={cte:.2f} min_scan={min_s:.2f}",
            flush=True
        )

        return obs, {}

    def step(self, action):
        self.unpause()
        self.step_id += 1

        self.publish_action(action)
        time.sleep(self.step_time)

        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0.01)

        obs, nearest_i, goal_dist, cte, min_s = self.make_obs()
        self.min_clearance_ep = min(self.min_clearance_ep, min_s)

        progress_goal = self.prev_goal_dist - goal_dist
        progress_path = max(0, nearest_i - self.prev_path_index)
        backward_path = max(0, self.prev_path_index - nearest_i)

        self.prev_goal_dist = goal_dist
        if nearest_i > self.prev_path_index:
            self.prev_path_index = nearest_i

        heading_cos = float(obs[self.num_lidar_bins + 2])

        reward = 0.0

        # Good behavior rewards
        reward += 14.0 * progress_goal
        reward += 0.12 * progress_path
        reward += 0.06 * heading_cos

        # Bad behavior penalties
        reward -= 0.70 * cte
        reward -= 0.10 * backward_path
        reward -= 0.03

        if int(action) in [3, 4]:
            reward -= 0.02

        if min_s > 0.65 and cte < 0.25 and int(action) == 0:
            reward += 0.04

        # Obstacle near penalty
        if min_s < self.warning_distance:
            danger = (self.warning_distance - min_s) / self.warning_distance
            reward -= 5.0 * (danger ** 2)

        terminated = False
        truncated = False
        status = "running"

        if goal_dist < self.goal_threshold:
            reward += 300.0
            terminated = True
            status = "success"

        elif self.step_id > self.collision_grace_steps and min_s < self.collision_threshold:
            reward -= 300.0
            terminated = True
            status = "collision"

        elif self.step_id >= self.max_steps:
            reward -= 70.0
            truncated = True
            status = "timeout"

        info = {
            "status": status,
            "steps": self.step_id,
            "goal_dist": float(goal_dist),
            "cross_track_error": float(cte),
            "min_clearance": float(self.min_clearance_ep),
            "nearest_path_index": int(nearest_i),
        }

        if terminated or truncated:
            self.stop_robot()
            print(
                f"[END_V2] ep={self.episode_id} status={status} "
                f"steps={self.step_id} goal_dist={goal_dist:.2f} "
                f"cte={cte:.2f} min_clearance={self.min_clearance_ep:.2f} "
                f"path_idx={nearest_i}",
                flush=True
            )

        return obs, float(reward), terminated, truncated, info


class MetricsCallbackV2(BaseCallback):
    def __init__(self, results_dir):
        super().__init__()
        self.csv_path = Path(results_dir) / "dqn_saved_nav2_path_v2_train_metrics.csv"
        self.ep = 0

        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "episode",
                "status",
                "steps",
                "goal_dist",
                "cross_track_error",
                "min_clearance",
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
                        info.get("steps", ""),
                        info.get("goal_dist", ""),
                        info.get("cross_track_error", ""),
                        info.get("min_clearance", ""),
                        info.get("nearest_path_index", ""),
                    ])
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--max_steps", type=int, default=600)
    parser.add_argument("--step_time", type=float, default=0.10)
    parser.add_argument("--lookahead_dist", type=float, default=0.55)
    args = parser.parse_args()

    results_dir = "/ws_slam/nav2_rl_project/results"
    Path(results_dir).mkdir(parents=True, exist_ok=True)

    rclpy.init()

    env = DQNSavedPathEnvV2(
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
        tensorboard_log="/ws_slam/nav2_rl_project/results/tb_logs_v2",
    )

    total_timesteps = args.episodes * args.max_steps

    print("======================================")
    print("DQN V2 TRAINING: NAV2 PATH LOCAL CONTROLLER")
    print(f"Episodes approx: {args.episodes}")
    print(f"Max steps per episode: {args.max_steps}")
    print(f"Total timesteps: {total_timesteps}")
    print("Collision threshold: 0.105 m")
    print("Collision grace steps: 12")
    print("======================================")

    callback = MetricsCallbackV2(results_dir)

    model.learn(
        total_timesteps=total_timesteps,
        callback=callback,
        progress_bar=False,
    )

    out = Path(results_dir) / "dqn_saved_nav2_path_v2_local_controller"
    model.save(str(out))

    env.close()

    try:
        rclpy.shutdown()
    except Exception:
        pass

    print(f"[SAVE] DQN V2 model saved to {out}.zip")


if __name__ == "__main__":
    main()
