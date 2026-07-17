#!/usr/bin/env python3

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path


def number(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def rising_events(flags):
    events = 0
    previous = False

    for flag in flags:
        if flag and not previous:
            events += 1
        previous = flag

    return events


def calculate_path_length(path_file):
    with Path(path_file).open("r", newline="") as file:
        rows = list(csv.DictReader(file))

    points = [
        (number(row, "x"), number(row, "y"))
        for row in rows
    ]

    return sum(
        math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
        for index in range(1, len(points))
    )


def calculate_stuck_events(rows, window_seconds=1.0):
    if len(rows) < 2:
        return 0

    flags = [False] * len(rows)

    for start_index in range(len(rows)):
        start_time = number(rows[start_index], "time")
        end_index = start_index

        while (
            end_index < len(rows)
            and number(rows[end_index], "time") - start_time
            < window_seconds
        ):
            end_index += 1

        if end_index >= len(rows):
            continue

        window = rows[start_index:end_index + 1]

        commanded_linear = [
            abs(number(row, "linear_velocity"))
            for row in window
        ]

        average_command = statistics.fmean(commanded_linear)

        start_x = number(rows[start_index], "x")
        start_y = number(rows[start_index], "y")
        end_x = number(rows[end_index], "x")
        end_y = number(rows[end_index], "y")

        displacement = math.hypot(
            end_x - start_x,
            end_y - start_y,
        )

        front_clearance = min(
            number(row, "front_scan", 99.0)
            for row in window
        )

        flags[start_index] = (
            average_command > 0.025
            and displacement < 0.015
            and front_clearance < 0.35
        )

    return rising_events(flags)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--log", required=True)
    parser.add_argument("--path", required=True)
    parser.add_argument("--output-prefix", required=True)

    parser.add_argument(
        "--goal-threshold",
        type=float,
        default=0.30,
    )

    parser.add_argument(
        "--near-clearance",
        type=float,
        default=0.30,
    )

    parser.add_argument(
        "--critical-clearance",
        type=float,
        default=0.16,
    )

    parser.add_argument(
        "--collision-proxy-clearance",
        type=float,
        default=0.10,
    )

    args = parser.parse_args()

    log_file = Path(args.log)

    if not log_file.exists():
        raise SystemExit(f"Missing log file: {log_file}")

    with log_file.open("r", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise SystemExit("Policy log contains no data rows.")

    planned_path_length = calculate_path_length(args.path)

    timestamps = [
        number(row, "time")
        for row in rows
    ]

    positions = [
        (
            number(row, "x"),
            number(row, "y"),
        )
        for row in rows
    ]

    travelled_distance = sum(
        math.hypot(
            positions[index][0] - positions[index - 1][0],
            positions[index][1] - positions[index - 1][1],
        )
        for index in range(1, len(positions))
    )

    elapsed_time = max(
        0.0,
        timestamps[-1] - timestamps[0],
    )

    cross_track_errors = [
        number(row, "cross_track_error")
        for row in rows
    ]

    minimum_scans = [
        number(row, "minimum_scan", 99.0)
        for row in rows
    ]

    front_scans = [
        number(row, "front_scan", 99.0)
        for row in rows
    ]

    goal_distances = [
        number(row, "goal_distance", 99.0)
        for row in rows
    ]

    progresses = [
        number(row, "progress")
        for row in rows
    ]

    linear_commands = [
        number(row, "linear_velocity")
        for row in rows
    ]

    angular_commands = [
        abs(number(row, "angular_velocity"))
        for row in rows
    ]

    statuses = [
        row.get("status", "unknown")
        for row in rows
    ]

    action_names = [
        row.get("action_name", "unknown")
        for row in rows
    ]

    success = (
        "goal_reached" in statuses
        or min(goal_distances) <= args.goal_threshold
    )

    near_collision_flags = [
        minimum_scan < args.near_clearance
        or front_scan < args.near_clearance
        for minimum_scan, front_scan
        in zip(minimum_scans, front_scans)
    ]

    critical_clearance_flags = [
        minimum_scan < args.critical_clearance
        or front_scan < args.critical_clearance
        for minimum_scan, front_scan
        in zip(minimum_scans, front_scans)
    ]

    collision_proxy_flags = [
        minimum_scan < args.collision_proxy_clearance
        or front_scan < args.collision_proxy_clearance
        for minimum_scan, front_scan
        in zip(minimum_scans, front_scans)
    ]

    safety_statuses = {
        "front_safety_stop",
        "emergency_clearance_stop",
        "off_path_stop",
    }

    safety_stop_flags = [
        status in safety_statuses
        for status in statuses
    ]

    final_status = statuses[-1]

    if success:
        final_status = "goal_reached"

    path_efficiency = 0.0

    if travelled_distance > 0.001:
        path_efficiency = (
            planned_path_length
            / travelled_distance
        )

    summary = {
        "log_file": str(log_file),
        "path_file": str(Path(args.path)),
        "steps": len(rows),
        "success": success,
        "final_status": final_status,
        "elapsed_time_seconds": elapsed_time,
        "planned_path_length_m": planned_path_length,
        "travelled_distance_m": travelled_distance,
        "path_efficiency_ratio": path_efficiency,
        "final_goal_distance_m": goal_distances[-1],
        "minimum_goal_distance_m": min(goal_distances),
        "final_progress_ratio": progresses[-1],
        "maximum_progress_ratio": max(progresses),
        "mean_cross_track_error_m": statistics.fmean(
            cross_track_errors
        ),
        "maximum_cross_track_error_m": max(
            cross_track_errors
        ),
        "minimum_lidar_clearance_m": min(
            minimum_scans
        ),
        "minimum_front_clearance_m": min(
            front_scans
        ),
        "near_collision_events": rising_events(
            near_collision_flags
        ),
        "critical_clearance_events": rising_events(
            critical_clearance_flags
        ),
        "collision_proxy_events": rising_events(
            collision_proxy_flags
        ),
        "safety_stop_events": rising_events(
            safety_stop_flags
        ),
        "stuck_events": calculate_stuck_events(rows),
        "average_commanded_linear_mps": statistics.fmean(
            linear_commands
        ),
        "maximum_commanded_linear_mps": max(
            linear_commands
        ),
        "average_absolute_angular_rps": statistics.fmean(
            angular_commands
        ),
        "maximum_absolute_angular_rps": max(
            angular_commands
        ),
        "action_counts": dict(Counter(action_names)),
        "status_counts": dict(Counter(statuses)),
        "final_x": positions[-1][0],
        "final_y": positions[-1][1],
    }

    output_prefix = Path(args.output_prefix)
    output_prefix.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    json_file = output_prefix.with_suffix(".json")
    csv_file = output_prefix.with_suffix(".csv")

    with json_file.open("w") as file:
        json.dump(
            summary,
            file,
            indent=2,
        )

    flat_summary = {
        key: value
        for key, value in summary.items()
        if not isinstance(value, dict)
    }

    flat_summary["action_counts"] = json.dumps(
        summary["action_counts"],
        sort_keys=True,
    )

    flat_summary["status_counts"] = json.dumps(
        summary["status_counts"],
        sort_keys=True,
    )

    with csv_file.open("w", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(flat_summary.keys()),
        )

        writer.writeheader()
        writer.writerow(flat_summary)

    print()
    print("=" * 70)
    print("REAL ROBOT TEST SUMMARY")
    print("=" * 70)
    print(f"Success:                 {summary['success']}")
    print(f"Final status:            {summary['final_status']}")
    print(
        f"Elapsed time:            "
        f"{summary['elapsed_time_seconds']:.2f} s"
    )
    print(
        f"Planned path length:     "
        f"{summary['planned_path_length_m']:.3f} m"
    )
    print(
        f"Travelled distance:      "
        f"{summary['travelled_distance_m']:.3f} m"
    )
    print(
        f"Path efficiency:         "
        f"{summary['path_efficiency_ratio']:.3f}"
    )
    print(
        f"Final goal distance:     "
        f"{summary['final_goal_distance_m']:.3f} m"
    )
    print(
        f"Mean cross-track error:  "
        f"{summary['mean_cross_track_error_m']:.3f} m"
    )
    print(
        f"Maximum cross-track err: "
        f"{summary['maximum_cross_track_error_m']:.3f} m"
    )
    print(
        f"Minimum LiDAR clearance: "
        f"{summary['minimum_lidar_clearance_m']:.3f} m"
    )
    print(
        f"Near-collision events:   "
        f"{summary['near_collision_events']}"
    )
    print(
        f"Collision proxy events:  "
        f"{summary['collision_proxy_events']}"
    )
    print(
        f"Safety-stop events:      "
        f"{summary['safety_stop_events']}"
    )
    print(
        f"Stuck events:            "
        f"{summary['stuck_events']}"
    )
    print(f"JSON: {json_file}")
    print(f"CSV:  {csv_file}")
    print("=" * 70)


if __name__ == "__main__":
    main()
