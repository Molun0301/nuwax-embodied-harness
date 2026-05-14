#!/usr/bin/env bash
# 获取机器狗在 map 坐标系下的位置，或基于当前位置发送一个简单的 Nav2 目标。
#
# 用法:
#   ./get_go2_map_pose.sh
#   ./get_go2_map_pose.sh --watch
#   ./get_go2_map_pose.sh forward 3
#   ./get_go2_map_pose.sh forward 3 --send
#   ./get_go2_map_pose.sh map-x 3 --send

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SETUP="$ROOT_DIR/install/setup.bash"
GEOMETRY_RUNTIME_ENV_SCRIPT="$ROOT_DIR/../../../source_runtime_env.sh"

detect_ros_setup() {
  local candidates=()
  if [[ -n "${NUWAX_ROS_SETUP_SCRIPT:-}" ]]; then
    candidates+=("${NUWAX_ROS_SETUP_SCRIPT}")
  fi
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    candidates+=("/opt/ros/${ROS_DISTRO}/setup.bash")
  fi
  candidates+=(
    "/opt/ros/jazzy/setup.bash"
    "/opt/ros/humble/setup.bash"
    "/opt/ros/iron/setup.bash"
    "/opt/ros/foxy/setup.bash"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

ROS_SETUP="$(detect_ros_setup || true)"

workspace_stale_source() {
  local setup_file="$1"
  local source_file=""
  local search_roots=()
  [[ -d "${ROOT_DIR}/src" ]] && search_roots+=("${ROOT_DIR}/src")
  if [[ "${#search_roots[@]}" -eq 0 ]]; then
    return 1
  fi
  while IFS= read -r -d '' source_file; do
    if [[ "${source_file}" -nt "${setup_file}" ]]; then
      echo "${source_file}"
      return 0
    fi
  done < <(find "${search_roots[@]}" -type f \
    \( -name 'package.xml' -o -name 'CMakeLists.txt' -o -name '*.launch.py' -o -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.yaml' -o -name '*.xml' \) \
    -print0 2>/dev/null)
  return 1
}

assert_workspace_current() {
  local setup_file="$1"
  local stale_source=""
  stale_source="$(workspace_stale_source "${setup_file}" || true)"
  if [[ -n "${stale_source}" ]]; then
    echo "Workspace install is older than source: ${stale_source}" >&2
    echo "Rebuild first: cd ${ROOT_DIR} && source ${ROS_SETUP} && colcon build --symlink-install" >&2
    exit 1
  fi
}

if [[ -z "${ROS_SETUP}" || ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup. Set NUWAX_ROS_SETUP_SCRIPT or install ROS2 Jazzy/Humble/Iron/Foxy." >&2
  exit 1
fi

if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Missing workspace setup: ${WS_SETUP}" >&2
  echo "Build the workspace first: colcon build" >&2
  exit 1
fi

assert_workspace_current "${WS_SETUP}"

source "${ROS_SETUP}"
source "${WS_SETUP}"
export NUWAX_GEOMETRY_RUNTIME_SOURCE_ROS_ENV=false
export NUWAX_GEOMETRY_RUNTIME_FORCE_CYCLONEDDS=true
source "${GEOMETRY_RUNTIME_ENV_SCRIPT}"

set -u

if [[ "${GO2_SHOW_DDS_WARNINGS:-0}" != "1" ]]; then
  exec 2> >(grep -v "Failed to parse type hash" >&2)
fi

python3 - "$@" <<'PY'
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MapPoseTool(Node):
    def __init__(self, fixed_frame, base_frame):
        super().__init__("go2_map_pose_tool")
        self.fixed_frame = fixed_frame
        self.base_frame = base_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_pose(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    self.base_frame,
                    rclpy.time.Time(seconds=0, nanoseconds=0),
                )
                t = tf.transform.translation
                yaw = yaw_from_quaternion(tf.transform.rotation)
                return t.x, t.y, t.z, yaw
            except TransformException as exc:
                last_error = exc

        raise RuntimeError(
            f"无法获取 TF: {self.fixed_frame} -> {self.base_frame}. "
            f"请确认 Nav2/SLAM/localization 已启动，且 TF 链路正常。最后错误: {last_error}"
        )

    def send_nav_goal(self, x, y, yaw, timeout_sec):
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError as exc:
            raise RuntimeError("缺少 nav2_msgs，无法发送 NavigateToPose action") from exc

        client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("找不到 /navigate_to_pose action server，请确认 bt_navigator 已 active")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.fixed_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = quaternion_from_yaw(yaw)

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Nav2 拒绝了目标点")

        print(f"已发送 Nav2 目标: frame={self.fixed_frame} x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg")


def build_parser():
    parser = argparse.ArgumentParser(
        description="读取 Go2 在 map 坐标系下的位置，并可发送相对目标。"
    )
    parser.add_argument(
        "command",
        nargs="?",
        choices=("pose", "forward", "map-x", "map-y"),
        default="pose",
        help="pose=打印当前位置；forward=按当前朝向前进；map-x/map-y=沿 map 坐标轴移动",
    )
    parser.add_argument("distance", nargs="?", type=float, help="移动距离，单位米")
    parser.add_argument("--send", action="store_true", help="发送 Nav2 NavigateToPose 目标")
    parser.add_argument("--watch", action="store_true", help="持续打印当前位置")
    parser.add_argument("--rate", type=float, default=1.0, help="--watch 打印频率 Hz")
    parser.add_argument("--fixed-frame", default="map", help="默认 map")
    parser.add_argument("--base-frame", default="base_footprint", help="默认 base_footprint")
    parser.add_argument("--timeout", type=float, default=5.0, help="等待 TF/action 的超时时间")
    return parser


def print_pose(x, y, z, yaw, fixed_frame, base_frame):
    print(
        f"{base_frame} in {fixed_frame}: "
        f"x={x:.3f} m, y={y:.3f} m, z={z:.3f} m, "
        f"yaw={math.degrees(yaw):.1f} deg"
    )


def main(argv):
    args = build_parser().parse_args(argv)
    if args.command in ("forward", "map-x", "map-y") and args.distance is None:
        raise SystemExit(f"{args.command} 需要距离参数，例如: ./get_go2_map_pose.sh {args.command} 3")

    rclpy.init()
    node = MapPoseTool(args.fixed_frame, args.base_frame)
    try:
        if args.watch:
            period = 1.0 / max(args.rate, 0.1)
            while rclpy.ok():
                pose = node.lookup_pose(args.timeout)
                print_pose(*pose, args.fixed_frame, args.base_frame)
                time.sleep(period)
            return

        x, y, z, yaw = node.lookup_pose(args.timeout)
        print_pose(x, y, z, yaw, args.fixed_frame, args.base_frame)

        if args.command == "pose":
            return

        if args.command == "forward":
            goal_x = x + args.distance * math.cos(yaw)
            goal_y = y + args.distance * math.sin(yaw)
        elif args.command == "map-x":
            goal_x = x + args.distance
            goal_y = y
        else:
            goal_x = x
            goal_y = y + args.distance

        print(
            f"目标点: frame={args.fixed_frame} "
            f"x={goal_x:.3f} m, y={goal_y:.3f} m, yaw={math.degrees(yaw):.1f} deg"
        )

        if args.send:
            node.send_nav_goal(goal_x, goal_y, yaw, args.timeout)
        else:
            print("仅计算目标点；加 --send 才会发送给 Nav2。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
PY
