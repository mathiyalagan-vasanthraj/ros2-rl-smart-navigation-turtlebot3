# Reproducibility

## Software environment

- Ubuntu 22.04 on the TurtleBot3 Raspberry Pi
- ROS 2 Humble
- TurtleBot3 Burger
- Gazebo Classic for simulation
- Nav2
- SLAM Toolbox
- Gymnasium
- Stable-Baselines3
- Python 3.10

## Model files

- `results/dqn_v4_nav2path_local_controller.zip`
- `results/a2c_v4_nav2path_local_controller.zip`
- `results/ppo_v4_nav2path_local_controller.zip`
- `results/qlearning_v4_qtable.pkl`

## Real robot environment

```bash
source /opt/ros/humble/setup.bash
source /ws_slam/install/setup.bash
source /ws_slam/rl_venv/bin/activate

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Before running a real policy

```bash
ros2 topic info /cmd_vel -v
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_footprint
```

Expected `/cmd_vel` state before the policy starts:

- Publisher count: 0
- Subscriber count: 1

## Example experiment command

```bash
bash scripts/run_real_experiment.sh \
  dqn \
  results/dqn_v4_nav2path_local_controller.zip \
  paths/real_global_path_latest.csv \
  dqn_forward
```

## Example round trip

```bash
bash scripts/run_real_round_trip.sh \
  ppo \
  results/ppo_v4_nav2path_local_controller.zip
```

## Data policy

Summary CSV and JSON files are suitable for Git. Raw ROS bags and
large video files are stored externally and linked from the repository.
