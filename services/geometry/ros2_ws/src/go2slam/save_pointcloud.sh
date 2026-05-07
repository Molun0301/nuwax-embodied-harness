#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT_DIR="${ROOT_DIR}/output"

source /opt/ros/jazzy/setup.bash
source "${ROOT_DIR}/install/setup.bash"

mkdir -p "${OUTPUT_DIR}"

echo "Waiting for one /go2_built_map PointCloud2 message..."

OUTPUT_DIR="${OUTPUT_DIR}" python3 - <<'PY'
import math
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


OUTPUT_DIR = Path(os.environ["OUTPUT_DIR"])
TOPIC = "/go2_built_map"


def pcd_type_and_size(datatype):
    mapping = {
        PointField.INT8: ("I", 1),
        PointField.UINT8: ("U", 1),
        PointField.INT16: ("I", 2),
        PointField.UINT16: ("U", 2),
        PointField.INT32: ("I", 4),
        PointField.UINT32: ("U", 4),
        PointField.FLOAT32: ("F", 4),
        PointField.FLOAT64: ("F", 8),
    }
    return mapping.get(datatype)


def expanded_fields(msg):
    names = []
    sizes = []
    types = []
    counts = []
    read_names = []

    for field in msg.fields:
        type_info = pcd_type_and_size(field.datatype)
        if type_info is None or field.name == "_":
            continue

        pcd_type, size = type_info
        read_names.append(field.name)
        if field.count <= 1:
            names.append(field.name)
            sizes.append(str(size))
            types.append(pcd_type)
            counts.append("1")
        else:
            for index in range(field.count):
                names.append(f"{field.name}_{index}")
                sizes.append(str(size))
                types.append(pcd_type)
                counts.append("1")

    return read_names, names, sizes, types, counts


def flatten_point(point):
    values = []
    for value in point:
        if isinstance(value, (list, tuple)):
            values.extend(value)
        elif hasattr(value, "shape") and getattr(value, "shape", ()):
            values.extend(value.tolist())
        else:
            values.append(value)
    return values


def format_value(value):
    if isinstance(value, float):
        if not math.isfinite(value):
            return "nan"
        return f"{value:.9g}"
    return str(value)


class OneShotPcdSaver(Node):
    def __init__(self):
        super().__init__("go2_pointcloud_one_shot_saver")
        self.subscription = self.create_subscription(PointCloud2, TOPIC, self.callback, 10)
        self.saved = False

    def callback(self, msg):
        if self.saved:
            return
        self.saved = True

        read_names, names, sizes, types, counts = expanded_fields(msg)
        if not read_names:
            raise RuntimeError("PointCloud2 message has no supported fields")

        points = [flatten_point(point) for point in point_cloud2.read_points(
            msg, field_names=read_names, skip_nans=True)]

        stamp = msg.header.stamp
        filename = OUTPUT_DIR / f"go2_built_map_{stamp.sec}.{stamp.nanosec:09d}.pcd"
        with filename.open("w", encoding="ascii") as pcd:
            pcd.write("# .PCD v0.7 - Point Cloud Data file format\n")
            pcd.write("VERSION 0.7\n")
            pcd.write(f"FIELDS {' '.join(names)}\n")
            pcd.write(f"SIZE {' '.join(sizes)}\n")
            pcd.write(f"TYPE {' '.join(types)}\n")
            pcd.write(f"COUNT {' '.join(counts)}\n")
            pcd.write(f"WIDTH {len(points)}\n")
            pcd.write("HEIGHT 1\n")
            pcd.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            pcd.write(f"POINTS {len(points)}\n")
            pcd.write("DATA ascii\n")
            for point in points:
                pcd.write(" ".join(format_value(value) for value in point) + "\n")

        self.get_logger().info(f"Saved one point cloud: {filename}")
        rclpy.shutdown()


rclpy.init()
node = OneShotPcdSaver()
try:
    rclpy.spin(node)
finally:
    if rclpy.ok():
        rclpy.shutdown()
PY
