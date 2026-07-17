# Project Requirements Mapping

## Course project

**Project:** Reinforcement Learning for Smart Navigation
**Robot:** TurtleBot3 Burger
**Authors:** Mathiyalagan Vasantharaj and Isaac Vivin Moses

## AI integration

The project uses reinforcement learning for mobile-robot navigation.

- Deep Q-Network (DQN)
- Proximal Policy Optimization (PPO)
- Advantage Actor-Critic (A2C)
- Tabular Q-learning baseline

The deep-RL controllers were implemented using Gymnasium and
Stable-Baselines3. Each controller receives LiDAR and path-relative
navigation information and selects one of seven discrete velocity
actions.

## ROS 2 implementation

The system uses ROS 2 Humble and includes:

- ROS 2 nodes for mapping, localization, planning, path handling,
  RL policy execution, scan normalization and result analysis.
- ROS 2 topics including `/scan`, `/odom`, `/cmd_vel`, `/tf`,
  `/tf_static`, `/amcl_pose`, `/map` and `/rl_global_path`.
- Gazebo services for simulation reset and model-state handling.
- Nav2 path-planning actions through the planner server.
- Nav2 lifecycle-managed localization components.
- Explicit QoS handling for the best-effort LiDAR topic.
- Shell-based experiment automation and ROS bag recording.

## Simulation proof of concept

The minimum simulation requirement was completed in Gazebo.

- Dynamic Stage-4 environment
- Saved SLAM map
- Nav2-generated global path
- Two dynamic TurtleBot obstacle models
- 20-episode final evaluation
- Comparative evaluation of DQN, PPO, A2C and Q-learning

## Real-robot bonus implementation

The trained DQN, A2C and PPO policies were deployed on a physical
TurtleBot3 Burger.

The hardware implementation includes:

- Raspberry Pi and OpenCR TurtleBot3 bringup
- LDS-02 LiDAR
- Saved real-room map
- AMCL localization
- Nav2 global planner
- RL policy publishing directly to `/cmd_vel`
- LiDAR slowdown, recovery rotation and emergency stop
- Automatic CSV, JSON, console-log and ROS-bag recording
- Forward and return-path trials

## Deliverables

- Public GitHub repository
- Well-structured ROS 2 package
- Simulation and real-robot code
- Trained models and Q-table
- Evaluation metrics
- Technical documentation
- Simulation and hardware demonstration videos
- Final presentation
- IEEE-format final report
