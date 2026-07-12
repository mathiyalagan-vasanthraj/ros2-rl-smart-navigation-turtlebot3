#!/usr/bin/env python3

import argparse
import importlib.util
from pathlib import Path

import rclpy
from stable_baselines3 import A2C, DQN, PPO


TRAINING_SCRIPT = Path(
    "/ws_slam/nav2_rl_project/scripts/"
    "train_stage4_nav2path_all_algorithms_v4.py"
)

RESULTS_DIRECTORY = Path(
    "/ws_slam/nav2_rl_project/results"
)


def load_environment_module():
    specification = importlib.util.spec_from_file_location(
        "training_v4",
        TRAINING_SCRIPT,
    )

    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)

    return module


def load_policy(algorithm, environment):
    if algorithm == "a2c":
        model_path = (
            RESULTS_DIRECTORY /
            "a2c_v4_nav2path_local_controller.zip"
        )
        return A2C.load(model_path, env=environment)

    if algorithm == "dqn":
        model_path = (
            RESULTS_DIRECTORY /
            "dqn_v4_nav2path_local_controller.zip"
        )
        return DQN.load(model_path, env=environment)

    if algorithm == "ppo":
        model_path = (
            RESULTS_DIRECTORY /
            "ppo_v4_nav2path_local_controller.zip"
        )
        return PPO.load(model_path, env=environment)

    raise ValueError(f"Unsupported algorithm: {algorithm}")


def action_to_integer(action):
    if hasattr(action, "item"):
        return int(action.item())

    return int(action)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--algo",
        choices=["a2c", "dqn", "ppo"],
        default="a2c",
    )
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--max_steps", type=int, default=700)
    parser.add_argument("--step_time", type=float, default=0.10)
    parser.add_argument("--lookahead_dist", type=float, default=0.60)

    arguments = parser.parse_args()

    training_module = load_environment_module()

    rclpy.init()

    environment = (
        training_module.Stage4Nav2PathLocalControllerEnv(
            algo_name=f"demo_{arguments.algo}",
            results_dir=str(RESULTS_DIRECTORY),
            max_steps=arguments.max_steps,
            step_time=arguments.step_time,
            lookahead_dist=arguments.lookahead_dist,
        )
    )

    model = load_policy(arguments.algo, environment)

    print("=" * 60)
    print(f"RUNNING SAVED {arguments.algo.upper()} POLICY")
    print("Learning: disabled")
    print("Policy execution: deterministic")
    print("Official evaluation CSV files: not modified")
    print("=" * 60)

    try:
        for episode in range(1, arguments.episodes + 1):
            observation, _ = environment.reset()
            total_reward = 0.0
            final_information = {}

            for _ in range(arguments.max_steps):
                action, _ = model.predict(
                    observation,
                    deterministic=True,
                )

                (
                    observation,
                    reward,
                    terminated,
                    truncated,
                    information,
                ) = environment.step(action_to_integer(action))

                total_reward += float(reward)
                final_information = information

                if terminated or truncated:
                    break

            print(
                f"[DEMO:{arguments.algo}] "
                f"episode={episode}/{arguments.episodes} "
                f"status={final_information.get('status')} "
                f"reason={final_information.get('reason')} "
                f"steps={final_information.get('steps')} "
                f"goal_dist="
                f"{float(final_information.get('goal_dist', 999.0)):.2f} "
                f"cte="
                f"{float(final_information.get('cross_track_error', 999.0)):.2f} "
                f"progress="
                f"{float(final_information.get('progress_ratio', 0.0)):.2f} "
                f"reward={total_reward:.2f}",
                flush=True,
            )

    finally:
        environment.close()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
