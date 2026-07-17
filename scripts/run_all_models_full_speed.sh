#!/usr/bin/env bash

set -Eeo pipefail

PROJECT_DIR="/ws_slam/nav2_rl_project"

cd "$PROJECT_DIR"

source /opt/ros/humble/setup.bash
source /ws_slam/install/setup.bash
source /ws_slam/rl_venv/bin/activate

export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ALGORITHMS=(
    "dqn"
    "a2c"
    "ppo"
)

MODELS=(
    "results/dqn_v4_nav2path_local_controller.zip"
    "results/a2c_v4_nav2path_local_controller.zip"
    "results/ppo_v4_nav2path_local_controller.zip"
)

mkdir -p results/full_speed_checks

echo
echo "=============================================================="
echo "REAL TURTLEBOT3 — FULL TRAINED-SPEED TESTS"
echo
echo "Speed scale:       1.00"
echo "Maximum linear:    0.22 m/s"
echo "Maximum angular:   1.20 rad/s"
echo
echo "Order:"
echo "  1. DQN"
echo "  2. A2C"
echo "  3. PPO"
echo "=============================================================="

for index in "${!ALGORITHMS[@]}"; do

    ALGORITHM="${ALGORITHMS[$index]}"
    MODEL="${MODELS[$index]}"
    CHECK_TIME="$(date +%Y%m%d_%H%M%S)"
    CHECK_LOG="results/full_speed_checks/${CHECK_TIME}_${ALGORITHM}_start_check.log"

    echo
    echo "=============================================================="
    echo "NEXT MODEL: ${ALGORITHM^^}"
    echo "MODEL FILE: $MODEL"
    echo "=============================================================="
    echo
    echo "Before continuing:"
    echo "  1. Physically place the robot at the original start."
    echo "  2. Point it along the first forward-path segment."
    echo "  3. Set 2D Pose Estimate in RViz."
    echo "  4. Wait until LaserScan aligns with the map."
    echo "  5. Start a new phone-video recording."
    echo

    read -r -p "Press ENTER after the robot is reset..."

    echo
    echo "Checking /cmd_vel..."

    CMD_INFO="$(ros2 topic info /cmd_vel -v 2>&1 || true)"
    echo "$CMD_INFO"

    if ! echo "$CMD_INFO" | grep -q "Publisher count: 0"; then
        echo
        echo "ERROR: Another node is publishing /cmd_vel."
        echo "Skipping ${ALGORITHM^^} until the old controller is stopped."
        continue
    fi

    echo
    echo "Performing a one-step stationary start check..."

    set +e

    python3 -u scripts/run_real_policy_v4.py \
        --algo "$ALGORITHM" \
        --model "$MODEL" \
        --path paths/real_global_path_latest.csv \
        --dry-run \
        --countdown 1 \
        --max-steps 1 \
        --sensor-wait-timeout 30 \
        2>&1 | tee "$CHECK_LOG"

    CHECK_EXIT=${PIPESTATUS[0]}

    set -e

    if grep -q "REAL TEST ERROR" "$CHECK_LOG"; then
        echo
        echo "START CHECK FAILED FOR ${ALGORITHM^^}."
        echo "Move the physical robot to the real path start and reset AMCL."
        echo "This model was not started."
        continue
    fi

    if ! grep -q "distance from saved path start=" "$CHECK_LOG"; then
        echo
        echo "Could not verify the saved-path start."
        echo "This model was not started."
        continue
    fi

    echo
    echo "Start check passed for ${ALGORITHM^^}."
    echo
    echo "The round-trip test will now begin."
    echo "It will record forward data and, only after success, return data."
    echo

    set +e

    ./scripts/run_real_round_trip.sh \
        "$ALGORITHM" \
        "$MODEL"

    ROUND_EXIT=$?

    set -e

    echo
    echo "${ALGORITHM^^} round-trip process exited with code $ROUND_EXIT."

    if [[ "$ROUND_EXIT" -eq 0 ]]; then
        echo "${ALGORITHM^^} round-trip completed."
    else
        echo "${ALGORITHM^^} did not complete the whole round trip."
        echo "The failed or partial run remains recorded as experimental data."
    fi

    echo
    echo "Stop the current phone recording."
    echo "Reset the robot before proceeding to the next model."
    echo

    read -r -p "Press ENTER to continue to the next model..."
done

echo
echo "=============================================================="
echo "ALL REQUESTED MODELS HAVE BEEN ATTEMPTED"
echo
echo "Results:"
echo "$PROJECT_DIR/results/real_experiments"
echo "=============================================================="
