# Demo Video Plan

## Required Demo Videos

1. Environment overview
2. A2C simulation demo
3. DQN simulation demo
4. Final result summary
5. Real TurtleBot3 A2C rollout
6. Real TurtleBot3 DQN rollout

## Video 1: Environment Overview

Show:
- Gazebo Stage-4 dynamic world
- TurtleBot3 Burger
- Dynamic obstacles
- LiDAR rays
- Saved project folders and results

Suggested explanation:

This is the dynamic Stage-4 Gazebo environment. Nav2 was used to generate a global path from the saved SLAM map. The reinforcement learning policy works as a local controller and publishes velocity commands to /cmd_vel.

## Video 2: A2C Simulation Demo

Show A2C navigating through the dynamic environment.

Suggested explanation:

A2C achieved the best overall result in the final 20-episode evaluation, with 75 percent success rate and zero timeout failures.

## Video 3: DQN Simulation Demo

Show DQN following the path safely.

Suggested explanation:

DQN achieved the lowest collision rate and lowest cross-track error. It is safer but more conservative than A2C.

## Video 4: Result Summary

Show the evaluation summary CSV.

Suggested explanation:

The final 20-episode evaluation shows A2C as the best overall algorithm, DQN as the safest controller, PPO as aggressive but risky, and Q-learning as the weakest baseline.

## Real TurtleBot3 Videos

Test only:
- A2C
- DQN

The real robot test is a preliminary sim-to-real rollout.
