# Real TurtleBot3 Results

## Scope

These results are preliminary hardware trials. They demonstrate
successful deployment and observed behavior, but they are not large
enough to claim statistically reliable hardware success rates.

## Full-speed trial results

| Algorithm | Leg | Result | Time | Travel | Final goal distance | Mean CTE | Max CTE |
|---|---|---|---:|---:|---:|---:|---:|
| DQN | Forward | Success | 109.14 s | 4.44 m | 0.299 m | 0.125 m | 0.223 m |
| DQN | Return | Failed | 9.73 s | 0.23 m | 1.885 m | 0.237 m | 0.254 m |
| DQN | Forward repeat | Failed | 49.96 s | 2.59 m | 0.792 m | 0.087 m | 0.146 m |
| A2C | Forward | Success | 116.85 s | 9.49 m | 0.296 m | 0.218 m | 0.833 m |
| A2C | Return | Success | 44.90 s | 4.53 m | 0.297 m | 0.151 m | 0.381 m |
| PPO | Forward | Success | 64.30 s | 5.01 m | 0.300 m | 0.209 m | 0.646 m |
| PPO | Return | Success | 62.38 s | 4.10 m | 0.299 m | 0.281 m | 0.610 m |

Machine-readable results are stored in:

- `results/real_robot_full_speed_summary.csv`
- `results/real_robot_round_trip_summary.csv`

## DQN

DQN completed one forward trial with the lowest maximum cross-track
error among the successful forward trials. Its behavior was precise
but not repeatable in the available hardware trials.

The return trial entered recovery behavior and did not complete.
A later forward repeat became dominated by right-turn decisions and
stalled before reaching the goal.

## A2C

A2C completed both forward and return legs. The forward run travelled
9.49 m for a 3.351 m planned path, showing substantial oscillation and
inefficient motion. Its maximum forward cross-track error of 0.833 m
was close to the configured real-world path-deviation limit.

## PPO

PPO completed both forward and return legs. It produced the fastest
completed round trip and the best combined distance efficiency among
the recorded complete round trips.

## Round-trip comparison

| Algorithm | Complete | Total time | Total travel | Combined efficiency |
|---|---:|---:|---:|---:|
| DQN | No | — | — | — |
| A2C | Yes | 161.74 s | 14.02 m | 0.478 |
| PPO | Yes | 126.68 s | 9.11 m | 0.736 |

## Safety interpretation

The final full-speed summaries did not report a LiDAR-based
collision-proxy event. However, LiDAR alone cannot prove that no
physical contact occurred. Recorded video remains the primary
qualitative ground truth because the TurtleBot3 setup did not include
a dedicated contact or bumper sensor in these trials.

## Limitations

- Only a small number of full-speed hardware trials were recorded.
- Start pose and AMCL initialization may vary slightly between trials.
- The real environment did not reproduce the moving Gazebo robots.
- The safety layer sometimes overrode the learned action.
- Battery state and floor friction can influence hardware behavior.
- Q-learning was not deployed with the final Stable-Baselines3
  hardware runner.
