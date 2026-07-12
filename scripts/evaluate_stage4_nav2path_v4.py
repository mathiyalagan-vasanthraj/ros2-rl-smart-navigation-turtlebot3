#!/usr/bin/env python3
import argparse
import csv
import importlib.util
import math
import pickle
import random
from pathlib import Path

import numpy as np
import rclpy
from stable_baselines3 import DQN, PPO, A2C


TRAIN_SCRIPT = "/ws_slam/nav2_rl_project/scripts/train_stage4_nav2path_all_algorithms_v4.py"

spec = importlib.util.spec_from_file_location("train_v4", TRAIN_SCRIPT)
train_v4 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(train_v4)


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


def load_model(algo, env, results_dir):
    results = Path(results_dir)

    if algo == "dqn":
        return DQN.load(results / "dqn_v4_nav2path_local_controller.zip", env=env)

    if algo == "ppo":
        return PPO.load(results / "ppo_v4_nav2path_local_controller.zip", env=env)

    if algo == "a2c":
        return A2C.load(results / "a2c_v4_nav2path_local_controller.zip", env=env)

    if algo == "qlearning":
        with (results / "qlearning_v4_qtable.pkl").open("rb") as f:
            return pickle.load(f)

    raise ValueError(algo)


def evaluate_algo(algo, episodes, max_steps, step_time, lookahead_dist, results_dir):
    print("\n" + "=" * 60)
    print(f"EVALUATING {algo.upper()}")
    print("=" * 60)

    env = train_v4.Stage4Nav2PathLocalControllerEnv(
        algo_name=f"eval_{algo}",
        results_dir=results_dir,
        max_steps=max_steps,
        step_time=step_time,
        lookahead_dist=lookahead_dist,
    )

    model = load_model(algo, env, results_dir)

    out_csv = Path(results_dir) / f"{algo}_v4_evaluation_metrics.csv"

    rows = []

    for ep in range(1, episodes + 1):
        obs, _ = env.reset()

        total_reward = 0.0
        final_info = {}

        for step in range(1, max_steps + 1):
            if algo in ["dqn", "ppo", "a2c"]:
                action, _ = model.predict(obs, deterministic=True)
                action = int(action)

            else:
                state = obs_to_q_state(env, obs)
                if state in model:
                    action = int(np.argmax(model[state]))
                else:
                    action = 0

            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            final_info = info

            if terminated or truncated:
                break

        rows.append({
            "episode": ep,
            "algo": algo,
            "status": final_info.get("status", ""),
            "reason": final_info.get("reason", ""),
            "steps": final_info.get("steps", ""),
            "total_reward": total_reward,
            "goal_dist": final_info.get("goal_dist", ""),
            "cross_track_error": final_info.get("cross_track_error", ""),
            "min_clearance": final_info.get("min_clearance", ""),
            "min_dynamic_distance": final_info.get("min_dynamic_distance", ""),
            "nearest_path_index": final_info.get("nearest_path_index", ""),
            "progress_ratio": final_info.get("progress_ratio", ""),
        })

        print(
            f"[EVAL:{algo}] ep={ep}/{episodes} "
            f"status={final_info.get('status')} "
            f"reason={final_info.get('reason')} "
            f"steps={final_info.get('steps')} "
            f"goal_dist={float(final_info.get('goal_dist', 999)):.2f} "
            f"progress={float(final_info.get('progress_ratio', 0)):.2f} "
            f"reward={total_reward:.2f}",
            flush=True
        )

    env.close()

    with out_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f"[SAVE] Evaluation metrics saved: {out_csv}")

    return rows


def summarize(all_rows, results_dir):
    summary_csv = Path(results_dir) / "v4_evaluation_summary.csv"

    algos = sorted(set(r["algo"] for r in all_rows))
    summary_rows = []

    for algo in algos:
        rows = [r for r in all_rows if r["algo"] == algo]
        n = len(rows)

        success = sum(1 for r in rows if r["status"] == "success")
        collision = sum(1 for r in rows if r["status"] == "collision")
        off_path = sum(1 for r in rows if r["status"] == "off_path")
        timeout = sum(1 for r in rows if r["status"] == "timeout")

        avg_reward = np.mean([float(r["total_reward"]) for r in rows])
        avg_steps = np.mean([float(r["steps"]) for r in rows])
        avg_goal = np.mean([float(r["goal_dist"]) for r in rows])
        best_goal = np.min([float(r["goal_dist"]) for r in rows])
        avg_cte = np.mean([float(r["cross_track_error"]) for r in rows])
        avg_progress = np.mean([float(r["progress_ratio"]) for r in rows])
        best_progress = np.max([float(r["progress_ratio"]) for r in rows])

        summary_rows.append({
            "algo": algo,
            "episodes": n,
            "success_count": success,
            "success_rate_%": round(100.0 * success / n, 2),
            "collision_count": collision,
            "collision_rate_%": round(100.0 * collision / n, 2),
            "off_path_count": off_path,
            "off_path_rate_%": round(100.0 * off_path / n, 2),
            "timeout_count": timeout,
            "timeout_rate_%": round(100.0 * timeout / n, 2),
            "avg_reward": round(avg_reward, 2),
            "avg_steps": round(avg_steps, 2),
            "avg_final_goal_dist": round(avg_goal, 2),
            "best_final_goal_dist": round(best_goal, 2),
            "avg_cross_track_error": round(avg_cte, 2),
            "avg_progress_ratio": round(avg_progress, 2),
            "best_progress_ratio": round(best_progress, 2),
        })

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)

    print("\n" + "=" * 60)
    print("EVALUATION SUMMARY")
    print("=" * 60)

    for r in summary_rows:
        print(r)

    print(f"\n[SAVE] Summary saved: {summary_csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algos", type=str, default="dqn,ppo,a2c,qlearning")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--max_steps", type=int, default=700)
    parser.add_argument("--step_time", type=float, default=0.10)
    parser.add_argument("--lookahead_dist", type=float, default=0.60)
    args = parser.parse_args()

    results_dir = "/ws_slam/nav2_rl_project/results"

    algos = [a.strip().lower() for a in args.algos.split(",") if a.strip()]

    rclpy.init()

    all_rows = []

    try:
        for algo in algos:
            rows = evaluate_algo(
                algo=algo,
                episodes=args.episodes,
                max_steps=args.max_steps,
                step_time=args.step_time,
                lookahead_dist=args.lookahead_dist,
                results_dir=results_dir,
            )
            all_rows.extend(rows)

        summarize(all_rows, results_dir)

    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
