> **Status update:** The planned real TurtleBot3 proof of concept was
> completed. DQN, A2C and PPO were deployed using AMCL localization,
> a saved Nav2 path and a LiDAR safety layer. Actual results are reported
> in [REAL_HARDWARE_RESULTS.md](REAL_HARDWARE_RESULTS.md).

# Real TurtleBot3 Test Plan

## Purpose

The real TurtleBot3 test is a preliminary sim-to-real rollout of the trained reinforcement learning local controllers.

## Algorithms to Test

Only two algorithms are selected for the real robot test:

1. A2C - best overall simulation success rate
2. DQN - safest simulation behavior with the lowest collision rate

## Important Difference Between Simulation and Real Robot

The Gazebo training environment used:

- /scan
- /odom
- /cmd_vel
- /model_states
- /set_entity_state

On the real TurtleBot3, /model_states and /set_entity_state are not available. Therefore, the real robot rollout must use only /scan, /odom, and /cmd_vel. The robot must be manually reset between trials.

## Recommended Real Test Setup

- Flat indoor floor
- 2 to 3 m test path
- One or two soft obstacles
- Low speed
- Emergency stop ready
- Record video from side or top view

## Success Criteria

| Result | Meaning |
|---|---|
| Robot reaches target region | Success |
| Robot avoids obstacle | Safe behavior |
| Robot stops or turns near obstacle | Reactive behavior |
| Robot hits obstacle | Collision |
| Robot spins or gets stuck | Failure |
| Manual emergency stop needed | Safety failure |

## Report Wording

The real-world rollout is a preliminary sim-to-real test. Full real-world robustness requires further tuning, domain randomization, and replacement of Gazebo-specific inputs such as /model_states.
