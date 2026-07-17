# ROS 2 Interfaces

## Main nodes and components

| Component | Role |
|---|---|
| `turtlebot3_node` | Motor interface, odometry and robot state |
| LDS-02 driver | Publishes LiDAR measurements |
| `map_server` | Publishes the saved occupancy map |
| AMCL | Estimates pose in the map frame |
| `planner_server` | Produces the Nav2 global path |
| RViz | Pose initialization, goal selection and visualization |
| Path bridge | Saves and republishes the global path |
| Real RL policy | Selects actions and publishes `/cmd_vel` |
| Scan normalizer | Produces `/scan_fixed` when required |
| Result analyzer | Calculates hardware-test metrics |

## Important topics

| Topic | Message type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Raw physical LiDAR |
| `/scan_fixed` | `sensor_msgs/LaserScan` | Normalized scan |
| `/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Localized pose |
| `/cmd_vel` | `geometry_msgs/Twist` | Final robot command |
| `/map` | `nav_msgs/OccupancyGrid` | Saved real-room map |
| `/rl_global_path` | `nav_msgs/Path` | RL reference path |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms |
| `/imu` | `sensor_msgs/Imu` | IMU measurements |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint states |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery status |

## Actions

Nav2 planning uses the planner server's path-computation action to
calculate a global route from the current pose to the selected goal.

## Services

The simulation environment uses Gazebo services for model reset and
entity-state handling. The simulation environment waits for the
entity-state service before starting each episode.

## TF frames

```text
map
  └── odom
        └── base_footprint
              └── base_link
                    └── sensor frames
```

The real policy primarily queries `map -> base_footprint`.

## QoS

The LDS-02 `/scan` publisher uses sensor-data-style best-effort QoS.
Subscribers must use a compatible QoS profile to receive scans
reliably.

## DDS configuration

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

These values must be consistent on the Raspberry Pi and laptop.
