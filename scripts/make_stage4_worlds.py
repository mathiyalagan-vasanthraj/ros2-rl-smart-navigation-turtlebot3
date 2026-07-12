#!/usr/bin/env python3
import re
from pathlib import Path

src = Path("/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_dqn_stage4.world")

dynamic_dst = Path("/ws_slam/nav2_rl_project/worlds/stage4_dynamic_ros_state.world")
static_dst = Path("/ws_slam/nav2_rl_project/worlds/stage4_static_ros_state.world")

text = src.read_text()

state_plugin = """
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <update_rate>50.0</update_rate>
    </plugin>
"""

def add_state_plugin(world_text):
    if "libgazebo_ros_state.so" not in world_text:
        world_text = world_text.replace("</world>", state_plugin + "\n  </world>")
    return world_text

dynamic_text = add_state_plugin(text)
dynamic_dst.write_text(dynamic_text)

static_text = text

# Remove official moving obstacle models for mapping.
for name in ["turtlebot3_dqn_obstacle1", "turtlebot3_dqn_obstacle2"]:
    static_text = re.sub(
        r"\s*<model\s+name=['\"]" + re.escape(name) + r"['\"]>.*?</model>",
        "",
        static_text,
        flags=re.DOTALL,
    )

static_text = add_state_plugin(static_text)
static_dst.write_text(static_text)

print("Created:")
print(dynamic_dst)
print(static_dst)
