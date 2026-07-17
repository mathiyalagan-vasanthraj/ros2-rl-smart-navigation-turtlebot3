#!/usr/bin/env bash

set -Eeo pipefail

if [[ $# -ne 4 ]]; then
    echo
    echo "Usage:"
    echo "  $0 ALGORITHM MODEL PATH LABEL"
    echo
    echo "Example:"
    echo "  $0 dqn results/dqn_model.zip paths/forward.csv dqn_forward"
    exit 1
fi

ALGORITHM="$1"
MODEL_FILE="$2"
PATH_FILE="$3"
LABEL="$4"

PROJECT_DIR="/ws_slam/nav2_rl_project"

cd "$PROJECT_DIR"

source /opt/ros/humble/setup.bash
source /ws_slam/install/setup.bash
source /ws_slam/rl_venv/bin/activate

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

case "$ALGORITHM" in
    a2c|dqn|ppo)
        ;;
    *)
        echo "Invalid algorithm: $ALGORITHM"
        exit 1
        ;;
esac

if [[ ! -f "$MODEL_FILE" ]]; then
    echo "Model not found: $MODEL_FILE"
    exit 1
fi

if [[ ! -f "$PATH_FILE" ]]; then
    echo "Path not found: $PATH_FILE"
    exit 1
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RUN_NAME="${TIMESTAMP}_${LABEL}"
RUN_DIR="$PROJECT_DIR/results/real_experiments/$RUN_NAME"

mkdir -p "$RUN_DIR"

BAG_PID=""

stop_robot() {
    set +e

    timeout 1.5 ros2 topic pub -r 20 \
        /cmd_vel \
        geometry_msgs/msg/Twist \
        "{linear: {x: 0.0}, angular: {z: 0.0}}" \
        >/dev/null 2>&1

    if [[ -n "${BAG_PID:-}" ]]; then
        if kill -0 "$BAG_PID" 2>/dev/null; then
            kill -INT "$BAG_PID" 2>/dev/null
            wait "$BAG_PID" 2>/dev/null
        fi
    fi
}

trap stop_robot EXIT INT TERM

echo
echo "=============================================================="
echo "RUN:       $RUN_NAME"
echo "ALGORITHM: $ALGORITHM"
echo "MODEL:     $MODEL_FILE"
echo "PATH:      $PATH_FILE"
echo "OUTPUT:    $RUN_DIR"
echo "=============================================================="
echo

echo "Checking live LaserScan..."

if ! timeout 6 ros2 topic echo /scan --once --qos-reliability best_effort >/dev/null; then
    echo "ERROR: No live /scan data."
    exit 1
fi

echo "LaserScan OK."

echo "Checking localization transform map -> base_footprint..."

TF_CHECK_LOG="$(mktemp)"

set +e
timeout 8 ros2 run tf2_ros tf2_echo \
    map base_footprint \
    > "$TF_CHECK_LOG" 2>&1
TF_CHECK_EXIT=$?
set -e

if grep -q "Translation:" "$TF_CHECK_LOG"; then
    echo "Localization TF OK."

    grep -m1 -A2 "Translation:" "$TF_CHECK_LOG" || true
else
    echo
    echo "ERROR: map -> base_footprint was not received."
    echo "TF check exit code: $TF_CHECK_EXIT"
    echo
    cat "$TF_CHECK_LOG"
    rm -f "$TF_CHECK_LOG"
    exit 1
fi

rm -f "$TF_CHECK_LOG"

echo "Costmap clearing skipped."
echo "The trained RL controller does not require Nav2 costmaps."

cp "$PATH_FILE" "$RUN_DIR/test_path.csv"

cp maps/real_room_latest.yaml \
   "$RUN_DIR/" 2>/dev/null || true

cp maps/real_room_latest.pgm \
   "$RUN_DIR/" 2>/dev/null || true

cp maps/real_room_v1.yaml \
   "$RUN_DIR/" 2>/dev/null || true

cp maps/real_room_v1.pgm \
   "$RUN_DIR/" 2>/dev/null || true

{
    echo "run_name=$RUN_NAME"
    echo "algorithm=$ALGORITHM"
    echo "model=$MODEL_FILE"
    echo "path=$PATH_FILE"
    echo "date=$(date --iso-8601=seconds)"
    echo "ros_domain_id=$ROS_DOMAIN_ID"
    echo "turtlebot3_model=$TURTLEBOT3_MODEL"
    echo "speed_profile=full_trained_action_space"
    echo "speed_scale=1.00"
    echo "max_linear_mps=0.22"
    echo "max_angular_rps=1.20"
    echo "recovery_angular_rps=0.60"
    echo "stop_distance_m=0.28"
    echo "slow_distance_m=0.55"
    echo "emergency_all_stop_m=0.14"
} > "$RUN_DIR/metadata.txt"

ros2 topic info /cmd_vel -v \
    > "$RUN_DIR/cmd_vel_info_before.txt" 2>&1 || true

echo
echo "Starting ROS bag recording..."

ros2 bag record \
    -o "$RUN_DIR/rosbag" \
    /scan \
    /odom \
    /amcl_pose \
    /cmd_vel \
    /tf \
    /tf_static \
    /imu \
    /joint_states \
    /battery_state \
    /rl_global_path \
    /map \
    > "$RUN_DIR/rosbag_console.log" 2>&1 &

BAG_PID=$!

sleep 2

echo
echo "ROS bag PID: $BAG_PID"
echo
echo "Start the physical video recording now."
echo "Keep your hand close to the emergency power switch."
echo
read -r -p "Press ENTER to begin the robot test..."

POLICY_START_TIME="$(date +%s)"

set +e

python3 -u scripts/run_real_policy_v4.py \
    --algo "$ALGORITHM" \
    --model "$MODEL_FILE" \
    --path "$PATH_FILE" \
    --speed-scale 1.00 \
    --max-linear 0.22 \
    --max-angular 1.20 \
    --max-steps 1800 \
    --step-time 0.10 \
    --goal-threshold 0.30 \
    --max-cte 0.85 \
    --max-start-gap 0.50 \
    --stop-distance 0.28 \
    --slow-distance 0.55 \
    --all-stop 0.14 \
    --recovery-angular 0.60 \
    --recovery-clearance 0.50 \
    --max-recovery-steps 100 \
    --countdown 5 \
    --sensor-wait-timeout 30 \
    2>&1 | tee "$RUN_DIR/policy_console.log"

POLICY_EXIT_CODE=${PIPESTATUS[0]}

set -e

POLICY_END_TIME="$(date +%s)"

echo
echo "Policy process finished with exit code: $POLICY_EXIT_CODE"
echo "Stopping robot and ROS bag..."

stop_robot
BAG_PID=""

LATEST_POLICY_LOG="$(
    find "$PROJECT_DIR/results/real_tests" \
        -maxdepth 1 \
        -type f \
        -name "${ALGORITHM}_real_*.csv" \
        -printf '%T@ %p\n' \
        | sort -n \
        | tail -n 1 \
        | cut -d' ' -f2-
)"

if [[ -z "$LATEST_POLICY_LOG" ]]; then
    echo "ERROR: Could not locate the policy CSV log."
    exit 1
fi

POLICY_ROW_COUNT="$(wc -l < "$LATEST_POLICY_LOG")"

if [[ "$POLICY_ROW_COUNT" -le 1 ]]; then
    echo
    echo "ERROR: The policy produced no experiment data."
    echo "Most likely the robot was not located at the selected path start."
    echo
    echo "Policy console:"
    cat "$RUN_DIR/policy_console.log" || true
    echo
    echo "Partial run directory:"
    echo "$RUN_DIR"
    exit 2
fi

cp "$LATEST_POLICY_LOG" \
   "$RUN_DIR/policy_log.csv"

{
    echo "policy_exit_code=$POLICY_EXIT_CODE"
    echo "policy_start_epoch=$POLICY_START_TIME"
    echo "policy_end_epoch=$POLICY_END_TIME"
    echo "source_policy_log=$LATEST_POLICY_LOG"
} >> "$RUN_DIR/metadata.txt"

python3 scripts/analyze_real_test.py \
    --log "$RUN_DIR/policy_log.csv" \
    --path "$PATH_FILE" \
    --output-prefix "$RUN_DIR/summary"

echo
echo "=============================================================="
echo "TEST COMPLETE"
echo "Run directory:"
echo "$RUN_DIR"
echo
echo "Saved:"
echo "  policy_log.csv"
echo "  policy_console.log"
echo "  summary.csv"
echo "  summary.json"
echo "  rosbag/"
echo "  rosbag_console.log"
echo "  metadata.txt"
echo "=============================================================="

exit "$POLICY_EXIT_CODE"
