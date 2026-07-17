#!/usr/bin/env bash

set -Eeo pipefail

if [[ $# -ne 2 ]]; then
    echo
    echo "Usage:"
    echo "  $0 ALGORITHM MODEL_FILE"
    echo
    echo "Example:"
    echo "  $0 dqn results/dqn_v4_nav2path_local_controller.zip"
    exit 1
fi

ALGORITHM="$1"
MODEL_FILE="$2"

PROJECT_DIR="/ws_slam/nav2_rl_project"
FORWARD_PATH="paths/real_global_path_latest.csv"
RETURN_PATH="paths/real_global_path_return.csv"

cd "$PROJECT_DIR"

source /opt/ros/humble/setup.bash
source /ws_slam/install/setup.bash
source /ws_slam/rl_venv/bin/activate

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [[ ! -f "$MODEL_FILE" ]]; then
    echo "Model does not exist: $MODEL_FILE"
    exit 1
fi

if [[ ! -f "$FORWARD_PATH" ]]; then
    echo "Forward path does not exist: $FORWARD_PATH"
    exit 1
fi

if [[ ! -f "$RETURN_PATH" ]]; then
    echo "Return path does not exist: $RETURN_PATH"
    exit 1
fi

ROUND_ID="$(date +%Y%m%d_%H%M%S)"

FORWARD_LABEL="${ALGORITHM}_${ROUND_ID}_forward"
RETURN_LABEL="${ALGORITHM}_${ROUND_ID}_return"

echo
echo "================================================================"
echo "REAL TURTLEBOT3 ROUND TRIP"
echo "Algorithm: $ALGORITHM"
echo "Round ID:  $ROUND_ID"
echo
echo "Leg 1: saved start -> saved goal"
echo "Leg 2: saved goal  -> saved start"
echo "================================================================"
echo

echo "The robot must currently be at the original forward start."
echo "Start the continuous phone video now."
echo
read -r -p "Press ENTER to start the FORWARD leg..."

./scripts/run_real_experiment.sh \
    "$ALGORITHM" \
    "$MODEL_FILE" \
    "$FORWARD_PATH" \
    "$FORWARD_LABEL"

FORWARD_DIRECTORY="$(
    find results/real_experiments \
        -mindepth 1 \
        -maxdepth 1 \
        -type d \
        -name "*_${FORWARD_LABEL}" \
        -printf '%T@ %p\n' \
    | sort -n \
    | tail -1 \
    | cut -d' ' -f2-
)"

if [[ -z "$FORWARD_DIRECTORY" ]]; then
    echo "Could not find the forward experiment directory."
    exit 1
fi

FORWARD_SUMMARY="$FORWARD_DIRECTORY/summary.json"

if [[ ! -f "$FORWARD_SUMMARY" ]]; then
    echo "Forward summary is missing: $FORWARD_SUMMARY"
    exit 1
fi

FORWARD_SUCCESS="$(
    python3 - "$FORWARD_SUMMARY" <<'PY'
import json
import sys

with open(sys.argv[1], "r") as file:
    summary = json.load(file)

print("true" if summary.get("success") else "false")
PY
)"

echo
echo "Forward experiment:"
echo "$FORWARD_DIRECTORY"
echo

if [[ "$FORWARD_SUCCESS" != "true" ]]; then
    echo "Forward leg did not reach the goal."
    echo "The return leg will not start automatically."
    echo
    cat "$FORWARD_SUMMARY"
    exit 2
fi

echo "FORWARD LEG SUCCESSFUL."
echo
echo "Do not carry or manually rotate the robot."
echo "Check that RViz LaserScan still aligns with the map."
echo "The next test will use the reversed saved path."
echo
read -r -p "Press ENTER to start the RETURN leg..."

./scripts/run_real_experiment.sh \
    "$ALGORITHM" \
    "$MODEL_FILE" \
    "$RETURN_PATH" \
    "$RETURN_LABEL"

RETURN_DIRECTORY="$(
    find results/real_experiments \
        -mindepth 1 \
        -maxdepth 1 \
        -type d \
        -name "*_${RETURN_LABEL}" \
        -printf '%T@ %p\n' \
    | sort -n \
    | tail -1 \
    | cut -d' ' -f2-
)"

echo
echo "================================================================"
echo "ROUND TRIP FINISHED"
echo
echo "Forward data:"
echo "$FORWARD_DIRECTORY"
echo
echo "Return data:"
echo "$RETURN_DIRECTORY"
echo "================================================================"
