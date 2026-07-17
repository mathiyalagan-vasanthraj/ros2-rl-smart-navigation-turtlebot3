# Data Dictionary

## Simulation evaluation columns

| Column | Meaning |
|---|---|
| `status` | Success, collision, timeout or off-path |
| `reward` | Total episode reward |
| `steps` | Number of policy steps |
| `goal_dist` | Final Euclidean goal distance |
| `cte` | Final cross-track error |
| `progress` | Fraction of the saved path completed |
| `min_scan` | Minimum LiDAR distance |
| `dynamic_dist` | Minimum simulated dynamic-obstacle distance |

## Real test summary columns

| Column | Meaning |
|---|---|
| `elapsed_s` | Recorded run duration |
| `planned_path_m` | Length of reference global path |
| `travelled_m` | Integrated physical travel distance |
| `path_efficiency` | Planned distance divided by travelled distance |
| `final_goal_distance_m` | Robot distance from goal at termination |
| `path_progress` | Final reference-path progress |
| `mean_cte_m` | Mean cross-track error |
| `max_cte_m` | Maximum cross-track error |
| `min_lidar_m` | Minimum valid LiDAR reading |
| `min_front_clearance_m` | Minimum front-sector clearance |
| `near_collision_steps` | Steps below near-clearance threshold |
| `critical_clearance_steps` | Steps below critical threshold |
| `collision_proxy_events` | LiDAR-based collision indicators |
| `stuck_events` | Detected low-progress intervals |

A collision proxy is not equivalent to confirmed physical contact.
