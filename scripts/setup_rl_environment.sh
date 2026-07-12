#!/usr/bin/env bash

# Use:
# source scripts/setup_rl_environment.sh

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"

source /opt/ros/humble/setup.bash

if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
fi

if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

if [ ! -f "${WORKSPACE_ROOT}/rl_venv/bin/activate" ]; then
    echo "ERROR: RL virtual environment not found:"
    echo "${WORKSPACE_ROOT}/rl_venv"
    return 1
fi

source "${WORKSPACE_ROOT}/rl_venv/bin/activate"
export TURTLEBOT3_MODEL=burger

echo "ROS2 and RL environment ready."
echo "Project: ${PROJECT_ROOT}"
echo "Python:  $(command -v python3)"
