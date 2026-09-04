#!/usr/bin/env python3
"""Plot wall-gap rosbag data and vehicle-to-reference lateral error."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def load_reference(path: Path) -> np.ndarray:
    wanted = ("path_straight1", "path_arc", "path_straight2")
    grouped = {name: [] for name in wanted}
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if row["segment"] in grouped:
                grouped[row["segment"]].append(
                    (int(row["index"]), float(row["map_x"]), float(row["map_y"])))
    points = []
    for name in wanted:
        for _, x, y in sorted(grouped[name]):
            if not points or math.hypot(x - points[-1][0], y - points[-1][1]) > 1e-9:
                points.append((x, y))
    if len(points) < 2:
        raise RuntimeError(f"reference path has fewer than two points: {path}")
    return np.asarray(points)


def read_bag(bag: Path):
    storage = rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    wanted = {"/parking/slam_pose", "/vehicle/vector", "/wall_gap/state"}
    data = {topic: [] for topic in wanted}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(raw, get_message(types[topic]))
        if topic == "/parking/slam_pose":
            q = msg.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            data[topic].append((stamp_ns * 1e-9, msg.pose.position.x,
                                msg.pose.position.y, yaw))
        elif topic == "/vehicle/vector":
            data[topic].append((stamp_ns * 1e-9, msg.v, msg.str))
        else:
            data[topic].append((stamp_ns * 1e-9, msg.data))
    return data


def project_to_path(positions: np.ndarray, path: np.ndarray):
    starts, vectors = path[:-1], np.diff(path, axis=0)
    lengths2 = np.einsum("ij,ij->i", vectors, vectors)
    segment_lengths = np.sqrt(lengths2)
    cumulative = np.r_[0.0, np.cumsum(segment_lengths)]
    lateral, along, projected = [], [], []
    for point in positions:
        u = np.clip(np.einsum("ij,ij->i", point - starts, vectors) / lengths2, 0, 1)
        candidates = starts + u[:, None] * vectors
        delta = point - candidates
        index = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
        tangent = vectors[index] / segment_lengths[index]
        lateral.append(tangent[0] * delta[index, 1] - tangent[1] * delta[index, 0])
        along.append(cumulative[index] + u[index] * segment_lengths[index])
        projected.append(candidates[index])
    return np.asarray(lateral), np.asarray(along), np.asarray(projected)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path)
    args = parser.parse_args()
    out_dir = args.out_dir or args.bag.parent / f"{args.bag.name}_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)

    path = load_reference(args.reference)
    bag = read_bag(args.bag)
    poses = np.asarray(bag["/parking/slam_pose"], dtype=float)
    if not len(poses):
        raise RuntimeError("bag contains no /parking/slam_pose messages")
    t0 = poses[0, 0]
    time_s, xy, yaw = poses[:, 0] - t0, poses[:, 1:3], poses[:, 3]
    lateral, along, projected = project_to_path(xy, path)

    csv_path = out_dir / "lateral_error.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "vehicle_x_m", "vehicle_y_m", "vehicle_yaw_rad",
                         "path_s_m", "lateral_error_m", "projected_x_m", "projected_y_m"])
        writer.writerows(np.c_[time_s, xy, yaw, along, lateral, projected])

    fig, axes = plt.subplots(3, 1, figsize=(11, 12), constrained_layout=True)
    axes[0].plot(path[:, 0], path[:, 1], "k--", label="reference path")
    scatter = axes[0].scatter(xy[:, 0], xy[:, 1], c=time_s, s=8, cmap="viridis",
                              label="vehicle pose")
    axes[0].axis("equal"); axes[0].grid(True); axes[0].legend()
    axes[0].set(xlabel="map x [m]", ylabel="map y [m]", title="Vehicle trajectory")
    fig.colorbar(scatter, ax=axes[0], label="time [s]")
    axes[1].plot(time_s, lateral * 100, lw=1)
    axes[1].axhline(0, color="k", lw=.7)
    axes[1].grid(True); axes[1].set(xlabel="time [s]", ylabel="lateral error [cm]",
                                    title="Signed lateral error (left of path is positive)")
    axes[2].plot(time_s, along, label="closest path station")
    axes[2].grid(True); axes[2].set(xlabel="time [s]", ylabel="path s [m]",
                                    title="Progress along reference path")
    png_path = out_dir / "wall_gap_analysis.png"
    fig.savefig(png_path, dpi=160)

    rmse = float(np.sqrt(np.mean(lateral ** 2)))
    print(f"poses={len(poses)}, duration={time_s[-1]:.3f}s")
    print(f"lateral RMSE={rmse:.4f}m, mean_abs={np.mean(np.abs(lateral)):.4f}m, "
          f"max_abs={np.max(np.abs(lateral)):.4f}m")
    print(csv_path)
    print(png_path)


if __name__ == "__main__":
    main()
