# Simulation-to-Real Workflow

## Simulation stage

1. Create the dynamic Gazebo environment.
2. Generate a global path using Nav2.
3. Save the path as 180 path points with a length of approximately
   4.52 m.
4. Train DQN, PPO and A2C for 140,000 environment timesteps.
5. Train tabular Q-learning for 200 manually controlled episodes.
6. Evaluate each algorithm for 20 episodes.
7. Save trained models, metrics and demonstration logs.

## Real mapping stage

1. Start the real TurtleBot3 bringup.
2. Publish the LDS-02 scan.
3. Run SLAM Toolbox with `use_sim_time: false`.
4. Drive the robot using teleoperation.
5. Save the real-room map.
6. Validate the map resolution and origin.

## Real localization and path stage

1. Start `map_server`.
2. Start AMCL.
3. Set the initial pose in RViz.
4. Run Nav2 `planner_server`.
5. select a goal in RViz.
6. Save the global planner output as a CSV path.
7. Create a reversed CSV for the return leg.

## Real policy stage

1. Confirm that no other node publishes `/cmd_vel`.
2. Verify `/scan`, TF and AMCL.
3. Verify that the robot is close to the path start.
4. Load a saved Stable-Baselines3 model.
5. Build path-relative observations from the real robot pose and scan.
6. Predict a deterministic discrete action.
7. Apply the real-world safety layer.
8. Publish the final velocity command.
9. Log the policy state and action.
10. Stop when the goal, path limit, safety condition or step limit is
    reached.

## Main transfer differences

| Simulation | Real robot |
|---|---|
| Gazebo `/model_states` | Not available |
| Exact simulated pose | AMCL and TF estimate |
| Controlled reset | Robot manually positioned |
| Dynamic robot distance | LiDAR-only obstacle indication |
| Idealized motion | Wheel slip, latency and friction |
| Simulated sensor noise | Physical LiDAR and odometry noise |

The hardware runner therefore uses a compatible observation structure
while relying on real TF, AMCL and LiDAR data instead of Gazebo
ground-truth model states.
