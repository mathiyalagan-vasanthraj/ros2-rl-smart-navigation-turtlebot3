# Reinforcement Learning for Smart TurtleBot3 Navigation

Course: Case Study ROS Robot Programming  
Project: Reinforcement Learning for Smart Navigation  
Robot: TurtleBot3 Burger  
Framework: ROS2 Humble, Gazebo, Nav2, Stable-Baselines3  
Algorithms: DQN, PPO, A2C, Q-learning  

## Project Overview

This project implements and compares reinforcement learning based local navigation controllers for TurtleBot3 in a dynamic Gazebo environment.

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
| DQN | 200 approximate episodes, 700 max steps, 140000 timesteps | results/dqn_v4_nav2path_local_controller.zip |
| PPO | 200 approximate episodes, 700 max steps, 140000 timesteps | results/ppo_v4_nav2path_local_controller.zip |
| A2C | 200 approximate episodes, 700 max steps, 140000 timesteps | results/a2c_v4_nav2path_local_controller.zip |
| Q-learning | 200 manual episodes, 700 max steps | results/qlearning_v4_qtable.pkl |

## Final 20-Episode Evaluation Results

| Rank | Algorithm | Success Rate | Collision Rate | Timeout Rate | Avg Reward | Avg Final Distance | Avg Cross-Track Error | Avg Progress |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | A2C | 75% | 25% | 0% | 178.99 | 0.61 m | 0.27 m | 0.89 |
| 2 | DQN | 50% | 10% | 40% | 121.83 | 0.61 m | 0.07 m | 0.86 |
| 3 | PPO | 35% | 65% | 0% | -254.81 | 1.06 m | 0.34 m | 0.79 |
| 4 | Q-learning | 5% | 60% | 30% | -601.90 | 2.14 m | 0.45 m | 0.50 |

## Main Findings

A2C achieved the best overall result with the highest success rate. DQN was the safest controller because it had the lowest collision rate and lowest cross-track error. PPO showed aggressive behavior but had a high collision rate. Q-learning was the weakest baseline because the discretized state representation was not sufficient for continuous navigation with dynamic obstacles.

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

- scripts/train_stage4_nav2path_all_algorithms_v4.py - final training script
- scripts/evaluate_stage4_nav2path_v4.py - final evaluation script
- scripts/save_global_path_from_rviz.py - saves Nav2 global path

## Important Results

- results/v4_evaluation_summary.csv
- results/dqn_v4_evaluation_metrics.csv
- results/ppo_v4_evaluation_metrics.csv
- results/a2c_v4_evaluation_metrics.csv
- results/qlearning_v4_evaluation_metrics.csv

## Demo Videos

Demo videos should show:

1. Gazebo environment overview
2. A2C successful navigation
3. DQN safe/conservative navigation
4. Final result summary
5. Optional real TurtleBot3 rollout

## Future Improvements

- Improve sim-to-real transfer using domain randomization.
- Replace Gazebo /model_states with LiDAR-only obstacle estimation.
- Train on multiple maps and different start-goal pairs.
- Improve reward shaping to reduce timeout behavior.
- Integrate the RL controller as a Nav2 local planner plugin.
- Add a real-world emergency safety layer.

## Authors

- Mathiyalagan Vasantharaj
- Moses Isaac
