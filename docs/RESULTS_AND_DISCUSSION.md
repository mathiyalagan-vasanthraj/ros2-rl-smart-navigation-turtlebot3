# Results and Discussion

## Final System

The final system uses Nav2 for global path generation and reinforcement learning as a local controller. The RL controller publishes velocity commands directly to /cmd_vel.

The saved path contained 180 path points with a total length of 4.52 m. The environment contained two moving dynamic obstacles.

## Development Progress

Several versions were tested before the final V4 implementation:

- Initial DQN: rejected because the robot reset immediately at step 1 due to overly strict collision detection.
- V2: fixed the immediate reset issue but did not handle dynamic obstacle collision robustly.
- V3: added wall collision, dynamic obstacle collision, and stuck detection.
- V4: final implementation with stronger saved-path following reward, dynamic obstacle collision, stuck detection, and off-path termination.

## Final Evaluation Results

The final evaluation was performed over 20 independent rollout episodes for each trained model without further learning.

| Algorithm | Success Rate | Collision Rate | Off-path Rate | Timeout Rate | Avg Reward | Avg Steps | Avg Final Distance | Avg CTE | Avg Progress |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| A2C | 75% | 25% | 0% | 0% | 178.99 | 198.45 | 0.61 m | 0.27 m | 0.89 |
| DQN | 50% | 10% | 0% | 40% | 121.83 | 476.95 | 0.61 m | 0.07 m | 0.86 |
| PPO | 35% | 65% | 0% | 0% | -254.81 | 235.90 | 1.06 m | 0.34 m | 0.79 |
| Q-learning | 5% | 60% | 5% | 30% | -601.90 | 438.85 | 2.14 m | 0.45 m | 0.50 |

## Interpretation

A2C achieved the best overall performance with the highest success rate and no timeout failures. DQN was the safest controller with the lowest collision rate and lowest cross-track error, but it was more conservative and sometimes timed out near the goal. PPO was more aggressive but produced many collisions. Q-learning was the weakest baseline because the discretized state representation was too limited for continuous navigation with dynamic obstacles.

## Final Ranking

1. A2C - best overall performance
2. DQN - safest and most path-stable
3. PPO - aggressive but unsafe
4. Q-learning - weakest baseline
