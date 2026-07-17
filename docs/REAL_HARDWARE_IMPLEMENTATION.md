# Real TurtleBot3 Hardware Implementation

## Overview

The simulation-trained DQN, A2C and PPO policies were transferred to
a physical TurtleBot3 Burger. The real robot uses the same discrete
action space as the simulation controllers, while replacing
simulation-only pose and obstacle information with ROS 2 localization
and real LiDAR measurements.

## Hardware

- TurtleBot3 Burger
- Raspberry Pi running Ubuntu 22.04 and ROS 2 Humble
- OpenCR motor controller
- LDS-02 LiDAR
- Laptop running the ROS 2 project in Docker
- Wi-Fi communication using ROS_DOMAIN_ID 30

## Software architecture

```text
TurtleBot3 Raspberry Pi
  ├── turtlebot3_node
  ├── LDS-02 driver
  ├── wheel odometry
  ├── IMU and joint states
  └── /cmd_vel subscriber
               │
               │ ROS 2 DDS network
               ▼
Laptop Docker container
  ├── map_server
  ├── AMCL
  ├── Nav2 planner_server
  ├── RViz
  ├── global-path bridge
  ├── RL policy runner
  ├── LiDAR safety layer
  ├── experiment recorder
  └── result analyzer
```

## Real map

Canonical map files:

- `maps/real_room_v1.yaml`
- `maps/real_room_v1.pgm`

Map configuration:

| Parameter | Value |
|---|---:|
| Resolution | 0.05 m/pixel |
| Origin | [-1.55, -2.85, 0] |
| Occupied threshold | 0.65 |
| Free threshold | 0.25 |
| Mode | trinary |

`real_room_latest.*` is retained as a runtime alias because the
experiment recorder references it when copying map evidence into an
experiment directory.

## Real global path

| Item | Value |
|---|---|
| Forward path | `paths/real_global_path_latest.csv` |
| Return path | `paths/real_global_path_return.csv` |
| Number of points | 132 |
| Planned path length | 3.351 m |
| Approximate start | (0.250, -0.200) |
| Approximate goal | (0.039, -2.367) |
| Goal threshold | 0.30 m |

## Localization and planning

- `map_server` publishes the saved occupancy map.
- AMCL estimates the robot pose in the `map` frame.
- The primary pose source is the `map -> base_footprint` transform.
- `/amcl_pose` is available as a fallback.
- Nav2 `planner_server` calculates the global path.
- The Nav2 controller server does not own `/cmd_vel`.
- The RL policy acts as the local controller and publishes directly
  to `/cmd_vel`.

## Real policy actions

| Action | Linear velocity | Angular velocity |
|---|---:|---:|
| Forward | 0.22 m/s | 0.00 rad/s |
| Forward-left | 0.16 m/s | +0.75 rad/s |
| Forward-right | 0.16 m/s | -0.75 rad/s |
| Turn-left | 0.04 m/s | +1.20 rad/s |
| Turn-right | 0.04 m/s | -1.20 rad/s |
| Slow-forward | 0.08 m/s | 0.00 rad/s |
| Stop | 0.00 m/s | 0.00 rad/s |

## Experiment automation

The experiment pipeline uses:

- `scripts/run_real_policy_v4.py`
- `scripts/run_real_experiment.sh`
- `scripts/run_real_round_trip.sh`
- `scripts/run_all_models_full_speed.sh`
- `scripts/analyze_real_test.py`

Each recorded experiment can contain:

- Policy-step CSV
- Result summary CSV
- Result summary JSON
- Metadata
- Console output
- Initial `/cmd_vel` publisher check
- Copy of the path and map
- ROS bag

Raw ROS bags are intentionally excluded from normal Git history.
