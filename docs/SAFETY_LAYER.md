# Real-World Safety Layer

## Purpose

A learned policy should not be allowed to command the physical robot
without an independent safety mechanism. The real hardware runner
combines the RL action with LiDAR-based velocity limiting and recovery.

## Final parameters

| Parameter | Value |
|---|---:|
| Speed scale | 1.00 |
| Maximum linear velocity | 0.22 m/s |
| Maximum angular velocity | 1.20 rad/s |
| Slowdown distance | 0.55 m |
| Recovery trigger | 0.28 m |
| Emergency all-stop | 0.14 m |
| Recovery angular velocity | 0.60 rad/s |
| Recovery clearance | 0.50 m |
| Maximum recovery steps | 100 |
| Maximum permitted CTE | 0.85 m |

## Safety modes

### Normal execution

The policy action is converted to a linear and angular velocity and
published to `/cmd_vel`.

### Slowdown

When the front clearance becomes smaller than the slowdown threshold,
forward speed is reduced.

### Recovery

When the front clearance becomes smaller than the recovery threshold:

1. Stop forward motion.
2. Compare the left and right LiDAR sectors.
3. Choose the clearer rotation direction.
4. Latch the direction to prevent alternating turns.
5. Rotate in place.
6. Resume policy control after front clearance reaches 0.50 m.
7. Stop if recovery exceeds the configured step limit.

### Emergency all-stop

At or below 0.14 m, the final command is zero velocity regardless of
the learned action.

### Path-deviation stop

The run is stopped when cross-track error exceeds the configured
real-world limit.

## Emergency terminal command

```bash
pkill -INT -f '[r]un_real_policy_v4.py'

timeout 2 ros2 topic pub -r 20 \
  /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Interpretation

The hardware system is therefore not a pure learned controller. It is
a hybrid system:

```text
RL policy
+ LiDAR slowdown
+ bounded recovery
+ emergency stop
```
