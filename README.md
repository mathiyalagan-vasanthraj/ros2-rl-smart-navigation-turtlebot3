# Reinforcement Learning for Smart TurtleBot3 Navigation

Course: Case Study ROS Robot Programming
Project: Reinforcement Learning for Smart Navigation
Robot: TurtleBot3 Burger
Framework: ROS2 Humble, Gazebo, Nav2, Stable-Baselines3, TurtleBot3 hardware
Algorithms: DQN, PPO, A2C, Q-learning

## Course Requirements Coverage

- **AI integration:** DQN, PPO, A2C and tabular Q-learning
- **Deep reinforcement learning:** Stable-Baselines3
- **Gym integration:** Gymnasium-compatible navigation environment
- **ROS 2:** nodes, topics, Gazebo services, Nav2 actions, TF, lifecycle and QoS
- **Simulation:** dynamic Gazebo navigation and 20-episode evaluation
- **Real robot:** DQN, A2C and PPO deployment on TurtleBot3 Burger
- **Deliverables:** ROS 2 package, GitHub repository, metrics, videos,
  presentation and IEEE-format report

See [Project Requirements Mapping](docs/PROJECT_REQUIREMENTS_MAPPING.md).

## Project Overview

This project implements and compares reinforcement-learning-based local navigation controllers for TurtleBot3 in a dynamic Gazebo environment and transfers the trained deep-RL policies to a physical TurtleBot3 Burger.

The system uses a hybrid navigation approach:

- Nav2 is used to generate a global path from a saved SLAM map.
- Reinforcement learning is used as the local controller.
- The RL policy receives LiDAR, odometry, saved-path features, and obstacle information.
- The RL controller publishes velocity commands directly to /cmd_vel.

## System Architecture

Saved SLAM Map
→ Nav2 Global Planner
→ Saved Global Path
→ RL Local Controller
→ /cmd_vel
→ TurtleBot3 in Gazebo

## Environment

| Item | Value |
|---|---|
| Robot | TurtleBot3 Burger |
| Simulator | Gazebo |
| ROS version | ROS2 Humble |
| Environment | TurtleBot3 Stage-4 dynamic world |
| Saved path points | 180 |
| Path length | 4.52 m |
| Start pose | (0.7376, -1.0048, 3.1387) |
| Goal pose | (-2.0694, 1.7654, 0.0) |
| Dynamic obstacles | turtlebot3_dqn_obstacle1, turtlebot3_dqn_obstacle2 |

## Algorithms Compared

1. DQN - Deep Q-Network
2. PPO - Proximal Policy Optimization
3. A2C - Advantage Actor-Critic
4. Q-learning - Tabular baseline

## Training Setup

| Algorithm | Training setup | Saved model |
|---|---|---|
| DQN | 140000 timesteps, maximum 700 steps per episode | results/dqn_v4_nav2path_local_controller.zip |
| PPO | 140000 timesteps, maximum 700 steps per episode | results/ppo_v4_nav2path_local_controller.zip |
| A2C | 140000 timesteps, maximum 700 steps per episode | results/a2c_v4_nav2path_local_controller.zip |
| Q-learning | 200 manual episodes, 700 max steps | results/qlearning_v4_qtable.pkl |

## Final 20-Episode Evaluation Results

| Rank | Algorithm | Success Rate | Collision Rate | Timeout Rate | Avg Reward | Avg Final Distance | Avg Cross-Track Error | Avg Progress |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | A2C | 75% | 25% | 0% | 178.99 | 0.61 m | 0.27 m | 0.89 |
| 2 | DQN | 50% | 10% | 40% | 121.83 | 0.61 m | 0.07 m | 0.86 |
| 3 | PPO | 35% | 65% | 0% | -254.81 | 1.06 m | 0.34 m | 0.79 |
| 4 | Q-learning | 5% | 60% | 30% | -601.90 | 2.14 m | 0.45 m | 0.50 |

## Main Findings

In the final 20-episode simulation evaluation, A2C achieved the
highest success rate at 75%. DQN recorded the lowest collision rate
and the lowest average cross-track error, although 40% of its episodes
ended in timeout. PPO showed aggressive behavior and a high simulation
collision rate. Tabular Q-learning was the weakest baseline in the
continuous dynamic-navigation task.

## Repository Structure

nav2_rl_project/
├── config/
├── docs/
├── maps/
├── media/
├── paths/
├── results/
├── scripts/
├── urdf/
├── worlds/
├── package.xml
├── setup.py
├── requirements.txt
└── README.md

## Important Scripts

### Simulation

- `scripts/train_stage4_nav2path_all_algorithms_v4.py` — final training script
- `scripts/evaluate_stage4_nav2path_v4.py` — final simulation evaluation
- `scripts/save_global_path_from_rviz.py` — saves the Nav2 global path
- `scripts/run_policy_demo_v4.py` — deterministic policy demonstration

### Real TurtleBot3

- `scripts/run_real_policy_v4.py` — executes DQN, A2C or PPO on hardware
- `scripts/run_real_experiment.sh` — records one controlled hardware trial
- `scripts/run_real_round_trip.sh` — executes forward and return trials
- `scripts/run_all_models_full_speed.sh` — validates and runs the deep-RL models
- `scripts/analyze_real_test.py` — computes hardware-test metrics
- `scripts/rviz_goal_to_nav2_path.py` — saves the real Nav2 global path
- `scripts/normalize_real_scan.py` — publishes a normalized LiDAR scan
- `scripts/safe_real_motion_test.py` — performs a controlled motion test
- `scripts/test_real_policy_inputs_v4.py` — validates policy observations

## Important Results

### Final simulation evaluation

- `results/v4_evaluation_summary_20ep_final.csv`
- `results/dqn_v4_evaluation_metrics_20ep_final.csv`
- `results/ppo_v4_evaluation_metrics_20ep_final.csv`
- `results/a2c_v4_evaluation_metrics_20ep_final.csv`
- `results/qlearning_v4_evaluation_metrics_20ep_final.csv`

The earlier five-episode pilot is retained separately as:

- `results/v4_evaluation_summary_5ep.csv`

### Real TurtleBot3

- `results/real_robot_full_speed_summary.csv`
- `results/real_robot_round_trip_summary.csv`
- [Real Hardware Results](docs/REAL_HARDWARE_RESULTS.md)

## Demo Videos

Simulation and real TurtleBot3 demonstrations are stored externally
so that large video files do not increase the Git repository size.

See [Demo Video Links](media/DEMO_VIDEO_LINKS.md).

The real-hardware demonstration is a completed part of this project,
not a future or optional implementation.

## Real TurtleBot3 Hardware Implementation

The DQN, A2C and PPO models were deployed on a physical TurtleBot3 Burger.

### Real architecture

```text
Saved real-room map
→ AMCL localization
→ Nav2 global planner
→ Saved forward or return path
→ RL local controller
→ LiDAR safety and recovery layer
→ /cmd_vel
→ Physical TurtleBot3
```

The Nav2 controller server does not control the robot during the RL trials.
The RL policy publishes directly to `/cmd_vel`.

### Real environment

| Item | Value |
|---|---|
| Robot | TurtleBot3 Burger |
| LiDAR | LDS-02 |
| ROS version | ROS 2 Humble |
| Map | `maps/real_room_v1.yaml` |
| Map resolution | 0.05 m/pixel |
| Forward path points | 132 |
| Forward path length | 3.351 m |
| Localization | AMCL and TF |
| Goal tolerance | 0.30 m |

### Real safety layer

The hardware deployment includes:

- slowdown below 0.55 m front clearance
- recovery rotation below 0.28 m
- emergency all-stop at 0.14 m
- bounded recovery duration
- cross-track-error termination
- pre-run `/cmd_vel` ownership verification

### Full-speed real results

| Algorithm | Forward | Return | Main observation |
|---|---|---|---|
| DQN | Success once; repeat failed | Failed | Precise but inconsistent |
| A2C | Success | Success | Completed but forward path was inefficient |
| PPO | Success | Success | Fastest and most efficient completed round trip |

| Algorithm | Round-trip time | Travel distance | Combined efficiency |
|---|---:|---:|---:|
| A2C | 161.74 s | 14.02 m | 0.478 |
| PPO | 126.68 s | 9.11 m | 0.736 |

Detailed documentation:

- [Real Hardware Implementation](docs/REAL_HARDWARE_IMPLEMENTATION.md)
- [Real Hardware Results](docs/REAL_HARDWARE_RESULTS.md)
- [Simulation-to-Real Workflow](docs/SIM_TO_REAL_WORKFLOW.md)
- [Safety Layer](docs/SAFETY_LAYER.md)
- [ROS 2 Interfaces](docs/ROS2_INTERFACES.md)
- [Reproducibility](docs/REPRODUCIBILITY.md)

## Future Improvements

- Train with domain randomization and randomized sensor noise.
- Train on multiple maps and different start-goal combinations.
- Replace simulation ground-truth obstacle distance with a
  perception-only representation.
- Improve DQN near-goal behavior and reduce timeout episodes.
- Reduce A2C oscillation and unnecessary travel distance.
- Integrate the RL controller as a Nav2 local-planner plugin.
- Add a physical emergency-stop device and contact sensing.
- Repeat hardware trials under controlled conditions to obtain
  statistically meaningful real-world results.

## Authors

- Mathiyalagan Vasantharaj
- Isaac Vivin Moses
