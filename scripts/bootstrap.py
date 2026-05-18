#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import annotations

import argparse
import getpass
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import textwrap
import time
import urllib.request
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from drivers.registry import ROBOT_DRIVER_REGISTRY
ROOT_ENV_TEMPLATE = PROJECT_ROOT / ".env.example"
ROOT_ENV_TARGET = PROJECT_ROOT / ".env"
START_SCRIPT = PROJECT_ROOT / "scripts" / "start_gateway.sh"
DEFAULT_SYSTEMD_TEMPLATE = PROJECT_ROOT / "deploy" / "systemd" / "nuwax-host-gateway.service.example"
DEFAULT_SYSTEMD_OUTPUT = PROJECT_ROOT / "deploy" / "systemd" / "nuwax-host-gateway.service"
DEFAULT_SYSTEMD_USER = os.environ.get("SUDO_USER") or os.environ.get("USER") or "unitree"
DEFAULT_EXTERNAL_MCP_TEMPLATE = PROJECT_ROOT / "deploy" / "nuwaclaw" / "external_mcp_servers.host_gateway.example.json"
DEFAULT_PROXY_MCP_TEMPLATE = PROJECT_ROOT / "deploy" / "nuwaclaw" / "mcp_proxy_config.host_gateway.example.json"
DEFAULT_EXTERNAL_MCP_OUTPUT = PROJECT_ROOT / "deploy" / "nuwaclaw" / "external_mcp_servers.host_gateway.json"
DEFAULT_PROXY_MCP_OUTPUT = PROJECT_ROOT / "deploy" / "nuwaclaw" / "mcp_proxy_config.host_gateway.json"
DEFAULT_HEALTH_URL = "http://127.0.0.1:8766/api/health"
DEFAULT_AGENT_TOKEN = "agent-robot-bridge-token"
DEFAULT_ADMIN_TOKEN = "admin-robot-bridge-token"
DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME = "nuwax-client"
DEFAULT_NUWACLAW_CLIENT_VOLUME_NAME = "nuwax_home"
DEFAULT_NUWACLAW_CLIENT_IMAGE = "nuwax-docker-images-registry.cn-hangzhou.cr.aliyuncs.com/nuwax/nuwax-agent-client:latest"
DEFAULT_NUWACLAW_SERVER_HOST = "https://agent.nuwax.com"
DEFAULT_NUWACLAW_AGENT_PORT = 60006
DEFAULT_NUWACLAW_FILE_SERVER_PORT = 60005
DEFAULT_NUWACLAW_WEB_PORT = 6080
DEFAULT_NUWACLAW_WORKSPACE_DIR = "/home/user/nuwaxbot/workspace"
DEFAULT_NUWACLAW_HOST_MOUNT = "none"
DEFAULT_NUWACLAW_PROXY_SCRIPT = (
    Path("/var/lib/docker/volumes")
    / DEFAULT_NUWACLAW_CLIENT_VOLUME_NAME
    / "_data"
    / ".nuwaxbot"
    / "node_modules"
    / "nuwax-mcp-stdio-proxy"
    / "dist"
    / "index.js"
)
DEFAULT_MEMORY_ONLINE_MODEL = "tongyi-embedding-vision-flash-2026-03-06"
DEFAULT_MEMORY_LOCAL_TEXT_MODEL = str(PROJECT_ROOT / "runtime_data" / "models" / "BAAI__bge-m3")
DEFAULT_MEMORY_LOCAL_IMAGE_MODEL = str(PROJECT_ROOT / "runtime_data" / "models" / "openai__clip-vit-base-patch32")
DEFAULT_YOLO_WORLD_MODEL = str(PROJECT_ROOT / "runtime_data" / "models" / "yolov8s-worldv2.pt")
DEFAULT_YOLO_WORLD_ENGINE = str(PROJECT_ROOT / "runtime_data" / "models" / "yolov8s-worldv2.engine")
GEOMETRY_WS_DIR = PROJECT_ROOT / "services" / "geometry" / "ros2_ws"
GO2SLAM_WS_DIR = GEOMETRY_WS_DIR / "src" / "go2slam"
GEOMETRY_APT_BASE_PACKAGES = (
    "build-essential",
    "cmake",
    "git",
    "python3-colcon-common-extensions",
    "python3-pip",
    "python3-rosdep",
    "python3-vcstool",
)
GEOMETRY_ROS_REQUIRED_PACKAGE_SUFFIXES = (
    "navigation2",
    "nav2-bringup",
    "pointcloud-to-laserscan",
    "rmw-cyclonedds-cpp",
    "robot-localization",
    "rtabmap-ros",
    "tf-transformations",
)
GEOMETRY_ROS_OPTIONAL_PACKAGE_SUFFIXES = (
    "nav2-simple-commander",
)
GO2SLAM_APT_BASE_PACKAGES = (
    "libboost-thread-dev",
    "libopencv-dev",
    "libpcl-dev",
)
GO2SLAM_RUNTIME_REQUIRED_PACKAGES = (
    "go2_navigation2",
    "go2_slam",
    "go2_driver",
    "go2_perception",
)
GO2SLAM_SOURCE_STALE_SUFFIXES = (
    ".xml",
    ".yaml",
    ".yml",
    ".py",
    ".cpp",
    ".hpp",
    ".h",
    ".launch.py",
)
GO2SLAM_ROS_REQUIRED_PACKAGE_SUFFIXES = (
    "ament-cmake",
    "ament-cmake-gtest",
    "ament-index-python",
    "ament-lint-auto",
    "ament-lint-common",
    "geometry-msgs",
    "image-geometry",
    "joint-state-publisher",
    "laser-geometry",
    "launch",
    "launch-ros",
    "map-msgs",
    "message-filters",
    "nav-msgs",
    "nav2-amcl",
    "nav2-bringup",
    "nav2-bt-navigator",
    "nav2-controller",
    "nav2-core",
    "nav2-costmap-2d",
    "nav2-lifecycle-manager",
    "nav2-map-server",
    "nav2-msgs",
    "nav2-planner",
    "nav2-smoother",
    "nav2-util",
    "nav2-waypoint-follower",
    "pcl-conversions",
    "pcl-ros",
    "pluginlib",
    "rclcpp",
    "rclcpp-components",
    "rclcpp-lifecycle",
    "rclpy",
    "robot-localization",
    "robot-state-publisher",
    "ros2launch",
    "rosidl-default-generators",
    "rosidl-default-runtime",
    "rviz2",
    "sensor-msgs",
    "sensor-msgs-py",
    "slam-toolbox",
    "std-msgs",
    "tf2",
    "tf2-geometry-msgs",
    "tf2-ros",
    "tf2-sensor-msgs",
    "visualization-msgs",
    "xacro",
)
DEFAULT_ROS2_APT_FALLBACK_URLS = (
    "https://mirrors.ustc.edu.cn/ros2/ubuntu",
    "http://packages.ros.org/ros2/ubuntu",
)


class BootstrapError(RuntimeError):
    """部署编排错误。"""


@dataclass(frozen=True)
class RobotBootstrapMetadata:
    robot_name: str
    env_template: Optional[Path]
    env_target: Optional[Path]
    supports_data_plane: bool
    recommended_tier: str


ROBOT_BOOTSTRAP_METADATA: Dict[str, RobotBootstrapMetadata] = {
    "go2": RobotBootstrapMetadata(
        robot_name="go2",
        env_template=PROJECT_ROOT / "drivers" / "go2" / "data_plane.env.example",
        env_target=PROJECT_ROOT / "drivers" / "go2" / "data_plane.env",
        supports_data_plane=True,
        recommended_tier="full",
    ),
    "g1": RobotBootstrapMetadata(
        robot_name="g1",
        env_template=None,
        env_target=None,
        supports_data_plane=False,
        recommended_tier="lite",
    ),
    "customrobot": RobotBootstrapMetadata(
        robot_name="customrobot",
        env_template=None,
        env_target=None,
        supports_data_plane=False,
        recommended_tier="lite",
    ),
}


def _bool_text(value: bool) -> str:
    return "true" if bool(value) else "false"


def _run(
    command: Sequence[str],
    *,
    cwd: Optional[Path] = None,
    env: Optional[Mapping[str, str]] = None,
    check: bool = True,
    capture_output: bool = False,
    display_command: Optional[str] = None,
) -> subprocess.CompletedProcess[str]:
    printable = display_command or shlex.join(list(command))
    print(f"$ {printable}")
    completed = subprocess.run(
        list(command),
        cwd=str(cwd) if cwd is not None else None,
        env=dict(env) if env is not None else None,
        check=False,
        text=True,
        capture_output=capture_output,
    )
    if check and completed.returncode != 0:
        if capture_output:
            if completed.stdout:
                print(completed.stdout, end="")
            if completed.stderr:
                print(completed.stderr, end="", file=sys.stderr)
        raise BootstrapError(f"命令执行失败: {printable}")
    return completed


def _print_completed_output(completed: subprocess.CompletedProcess[str]) -> None:
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.stderr:
        print(completed.stderr, end="", file=sys.stderr)


def _run_streaming_capture(
    command: Sequence[str],
    *,
    cwd: Optional[Path] = None,
    env: Optional[Mapping[str, str]] = None,
    display_command: Optional[str] = None,
) -> subprocess.CompletedProcess[str]:
    printable = display_command or shlex.join(list(command))
    print(f"$ {printable}")
    process = subprocess.Popen(
        list(command),
        cwd=str(cwd) if cwd is not None else None,
        env=dict(env) if env is not None else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    output_parts: List[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        output_parts.append(line)
        print(line, end="")
    returncode = process.wait()
    return subprocess.CompletedProcess(list(command), returncode, stdout="".join(output_parts), stderr="")


def _run_shell(command: str, *, cwd: Optional[Path] = None, check: bool = True) -> None:
    print(f"$ {command}")
    completed = subprocess.run(command, cwd=str(cwd) if cwd is not None else None, shell=True, text=True)
    if check and completed.returncode != 0:
        raise BootstrapError(f"命令执行失败: {command}")


def _which(name: str) -> Optional[str]:
    return shutil.which(name)


def _file_exists(path: Path) -> bool:
    return path.exists()


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _write_text(path: Path, content: str) -> None:
    _ensure_parent(path)
    path.write_text(content, encoding="utf-8")


def _mask_secret(value: str) -> str:
    normalized = str(value or "").strip()
    if not normalized:
        return "<empty>"
    if len(normalized) <= 8:
        return "*" * len(normalized)
    return f"{normalized[:4]}...{normalized[-4:]}"


def _parse_env_template(content: str) -> List[Tuple[str, Optional[str]]]:
    entries: List[Tuple[str, Optional[str]]] = []
    for raw_line in content.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in raw_line:
            continue
        key, value = raw_line.split("=", 1)
        key = key.strip()
        if not key:
            continue
        entries.append((key, value.strip()))
    return entries


def _render_env_from_template(template_path: Path, overrides: Mapping[str, str]) -> str:
    template = _read_text(template_path)
    lines: List[str] = []
    seen: set[str] = set()
    strip_go2_geometry_notes = (
        overrides.get("NUWAX_GEOMETRY_STACK") == "disabled"
        and overrides.get("NUWAX_GEOMETRY_EXTERNAL_SCRIPT") == ""
    )
    skipping_go2_geometry_note = False
    for raw_line in template.splitlines():
        stripped = raw_line.strip()
        if strip_go2_geometry_notes:
            lowered = raw_line.lower()
            if stripped.startswith("# external_slam_nav2 默认外部 SLAM/Nav2 栈说明"):
                skipping_go2_geometry_note = True
                continue
            if skipping_go2_geometry_note:
                if not stripped:
                    skipping_go2_geometry_note = False
                    lines.append(raw_line)
                continue
            if "go2slam" in lowered or "start_go2_slam_nav2.sh" in lowered or "start_rtabmap_sidecar" in lowered:
                continue
        if not stripped or stripped.startswith("#") or "=" not in raw_line:
            lines.append(raw_line)
            continue
        key, _ = raw_line.split("=", 1)
        normalized_key = key.strip()
        if normalized_key in overrides:
            lines.append(f"{normalized_key}={overrides[normalized_key]}")
            seen.add(normalized_key)
        else:
            lines.append(raw_line)
            seen.add(normalized_key)
    extra_keys = [key for key in overrides if key not in seen]
    if extra_keys:
        if lines and lines[-1] != "":
            lines.append("")
        lines.append("# bootstrap 追加配置")
        for key in extra_keys:
            lines.append(f"{key}={overrides[key]}")
    return "\n".join(lines).rstrip() + "\n"


def _parse_csv(value: str) -> Tuple[str, ...]:
    return tuple(part.strip() for part in str(value or "").split(",") if part.strip())


def _detect_ros_setup() -> Optional[Path]:
    configured = str(os.environ.get("NUWAX_ROS_SETUP_SCRIPT", "")).strip()
    candidates: List[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.extend(
        (
            Path("/opt/ros/jazzy/setup.bash"),
            Path("/opt/ros/humble/setup.bash"),
            Path("/opt/ros/iron/setup.bash"),
            Path("/opt/ros/foxy/setup.bash"),
        )
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def _detect_ros_distro(ros_setup: Optional[Path] = None) -> str:
    env_distro = str(os.environ.get("ROS_DISTRO", "")).strip()
    if env_distro:
        return env_distro
    setup_path = ros_setup or _detect_ros_setup()
    if setup_path is None:
        return ""
    try:
        parts = setup_path.parts
        ros_index = parts.index("ros")
        return parts[ros_index + 1]
    except (ValueError, IndexError):
        return ""


def go2slam_setup_path() -> Path:
    return GO2SLAM_WS_DIR / "install" / "setup.bash"


def find_go2slam_stale_source(setup_path: Path) -> Optional[Path]:
    if not setup_path.exists():
        return None
    roots = [GO2SLAM_WS_DIR / "src"]
    for root in roots:
        if not root.exists():
            continue
        for candidate in root.rglob("*"):
            if not candidate.is_file():
                continue
            name = candidate.name
            if not (name == "CMakeLists.txt" or name.endswith(GO2SLAM_SOURCE_STALE_SUFFIXES)):
                continue
            try:
                if candidate.stat().st_mtime > setup_path.stat().st_mtime:
                    return candidate
            except OSError:
                continue
    return None


def missing_go2slam_runtime_packages(ros_setup: Optional[Path], packages: Sequence[str] = GO2SLAM_RUNTIME_REQUIRED_PACKAGES) -> Tuple[str, ...]:
    setup_path = go2slam_setup_path()
    if ros_setup is None or not setup_path.exists():
        return tuple(packages)
    missing: List[str] = []
    for package_name in packages:
        command = (
            f"source {shlex.quote(str(ros_setup))} "
            f"&& source {shlex.quote(str(setup_path))} "
            f"&& ros2 pkg prefix {shlex.quote(package_name)} >/dev/null"
        )
        result = _run(("bash", "-lc", command), cwd=GO2SLAM_WS_DIR, check=False, capture_output=True)
        if result.returncode != 0:
            missing.append(package_name)
    return tuple(missing)


def _sudo_prefix() -> Tuple[str, ...]:
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        return ()
    if _which("sudo") is None:
        raise BootstrapError("需要 sudo 执行系统依赖安装，但当前环境未找到 sudo。")
    return ("sudo",)


def build_geometry_apt_packages(ros_distro: str) -> Tuple[str, ...]:
    normalized = str(ros_distro or "").strip()
    if not normalized:
        raise BootstrapError("无法识别 ROS2 发行版，不能推导 RTAB-Map/Nav2 apt 包名。")
    ros_packages = tuple(f"ros-{normalized}-{suffix}" for suffix in GEOMETRY_ROS_REQUIRED_PACKAGE_SUFFIXES)
    return (*GEOMETRY_APT_BASE_PACKAGES, *ros_packages)


def build_go2slam_apt_packages(ros_distro: str) -> Tuple[str, ...]:
    normalized = str(ros_distro or "").strip()
    if not normalized:
        raise BootstrapError("无法识别 ROS2 发行版，不能推导 Go2SLAM apt 包名。")
    nav2_recovery_suffix = "nav2-recoveries" if normalized == "foxy" else "nav2-behaviors"
    ros_packages = tuple(f"ros-{normalized}-{suffix}" for suffix in (*GO2SLAM_ROS_REQUIRED_PACKAGE_SUFFIXES, nav2_recovery_suffix))
    return (*GO2SLAM_APT_BASE_PACKAGES, *ros_packages)


def build_geometry_optional_apt_packages(ros_distro: str) -> Tuple[str, ...]:
    normalized = str(ros_distro or "").strip()
    if not normalized:
        raise BootstrapError("无法识别 ROS2 发行版，不能推导 RTAB-Map/Nav2 apt 包名。")
    return tuple(f"ros-{normalized}-{suffix}" for suffix in GEOMETRY_ROS_OPTIONAL_PACKAGE_SUFFIXES)


def apt_output_has_ros2_archive_404(output: str) -> bool:
    normalized = str(output or "").lower()
    return "404" in normalized and "not found" in normalized and "ros2/ubuntu" in normalized and "failed to fetch" in normalized


def build_ros2_apt_fallback_urls() -> Tuple[str, ...]:
    configured = _parse_csv(os.environ.get("NUWAX_ROS2_APT_FALLBACK_URLS", ""))
    if configured:
        return configured
    return DEFAULT_ROS2_APT_FALLBACK_URLS


def replace_ros2_apt_source_url(sudo: Sequence[str], replacement_url: str) -> bool:
    script = r"""
import re
import sys
from pathlib import Path

replacement = sys.argv[1].rstrip("/")
paths = [Path("/etc/apt/sources.list")]
source_dir = Path("/etc/apt/sources.list.d")
if source_dir.exists():
    paths.extend(sorted(source_dir.glob("*.list")))

changed = []
for path in paths:
    if not path.exists():
        continue
    text = path.read_text(encoding="utf-8", errors="replace")
    updated = re.sub(r"https?://[^\s]+/ros2/ubuntu/?", replacement, text)
    if updated == text:
        continue
    backup = Path(str(path) + ".nuwaxbak")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
    path.write_text(updated, encoding="utf-8")
    changed.append(str(path))

if changed:
    print("已更新 ROS2 apt 源: " + ", ".join(changed))
    sys.exit(0)

print("没有找到可更新的 ROS2 apt 源文件。")
sys.exit(3)
"""
    result = _run(
        (*sudo, "python3", "-c", script, replacement_url),
        cwd=PROJECT_ROOT,
        check=False,
        capture_output=True,
        display_command=f"{shlex.join((*sudo, 'python3'))} -c '<replace ROS2 apt source>' {replacement_url}",
    )
    _print_completed_output(result)
    return result.returncode == 0


def filter_available_apt_packages(packages: Sequence[str]) -> Tuple[str, ...]:
    if not packages:
        return ()
    if _which("apt-cache") is None:
        print("未找到 apt-cache，跳过可选 geometry apt 包: " + " ".join(packages))
        return ()

    available: List[str] = []
    skipped: List[str] = []
    for package in packages:
        result = _run(
            ("apt-cache", "show", package),
            cwd=PROJECT_ROOT,
            check=False,
            capture_output=True,
            display_command=f"apt-cache show {package} >/dev/null",
        )
        if result.returncode == 0 and result.stdout.strip():
            available.append(package)
        else:
            skipped.append(package)

    if skipped:
        print("跳过当前 apt 源不存在的可选 geometry 包: " + " ".join(skipped))
    return tuple(available)


def run_geometry_apt_update(sudo: Sequence[str], env: Mapping[str, str]) -> None:
    result = _run((*sudo, "apt-get", "update"), cwd=PROJECT_ROOT, env=env, check=False)
    if result.returncode != 0:
        print(
            "apt-get update 未完全成功，通常是某个无关第三方 apt 源不可用。"
            "将继续尝试安装 geometry 依赖；如果后续 apt-get install 仍失败，请先修复或临时禁用异常 apt 源。"
        )


def install_geometry_apt_packages(planner: "BootstrapPlanner", sudo: Sequence[str], env: Mapping[str, str], packages: Sequence[str]) -> None:
    command = (*sudo, "apt-get", "install", "-y", *packages)
    result = _run_streaming_capture(command, cwd=PROJECT_ROOT, env=env)
    if result.returncode == 0:
        return

    combined_output = "\n".join((result.stdout or "", result.stderr or ""))
    if not planner.repair_ros2_apt_source or not apt_output_has_ros2_archive_404(combined_output):
        raise BootstrapError(f"命令执行失败: {shlex.join(list(command))}")

    print("检测到 ROS2 apt 源包索引与实际归档不同步，准备切换 ROS2 apt 源并重试 geometry 依赖安装。")
    for fallback_url in build_ros2_apt_fallback_urls():
        print(f"尝试将 ROS2 apt 源切换到: {fallback_url}")
        if not replace_ros2_apt_source_url(sudo, fallback_url):
            continue
        _run((*sudo, "apt-get", "clean"), cwd=PROJECT_ROOT, env=env, check=False)
        run_geometry_apt_update(sudo, env)
        retry = _run_streaming_capture(command, cwd=PROJECT_ROOT, env=env)
        if retry.returncode == 0:
            return

    raise BootstrapError(
        "ROS2 apt 源持续返回 404。请手工修复 /etc/apt/sources.list.d 中的 ros2/ubuntu 源，"
        "或通过 NUWAX_ROS2_APT_FALLBACK_URLS 指定可用源后重试。"
    )


def _build_robot_choices() -> Tuple[str, ...]:
    return tuple(sorted(ROBOT_DRIVER_REGISTRY.keys()))


def _read_existing_env(path: Path) -> Dict[str, str]:
    if not path.exists():
        return {}
    values: Dict[str, str] = {}
    for raw_line in _read_text(path).splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if key:
            values[key] = value.strip().strip("\"'")
    return values


class BootstrapPlanner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.install_python = self._resolve_bool("install_python", default=True)
        requested_python_bin = str(args.python_bin or "").strip()
        self.seed_python_bin = sys.executable or "python3"
        self.use_project_venv = not requested_python_bin and self.install_python
        self.python_bin = requested_python_bin or (
            str(PROJECT_ROOT / ".venv" / "bin" / "python") if self.use_project_venv else self.seed_python_bin
        )
        self.repo_root = PROJECT_ROOT
        self.root_template = ROOT_ENV_TEMPLATE
        self.root_target = Path(args.root_env_output).expanduser().resolve() if args.root_env_output else ROOT_ENV_TARGET
        self.root_existing = _read_existing_env(self.root_target)
        self.selected_robot = self._resolve_robot()
        self.robot_meta = ROBOT_BOOTSTRAP_METADATA[self.selected_robot]
        self.tier = self._resolve_tier()
        self.geometry_enabled = self._resolve_bool("geometry", default=self.tier == "full")
        self.memory_mode = self._resolve_choice("memory_mode", ("cloud", "local", "disabled"), default="cloud" if self.tier == "full" else "disabled")
        self.perception_mode = self._resolve_choice("perception_mode", ("local", "hybrid", "cloud-scene", "disabled"), default="hybrid" if self.tier == "full" else "local")
        self.systemd_enabled = self._resolve_bool("systemd", default=False)
        self.relay_enabled = self._resolve_bool("relay", default=False)
        self.data_plane_enabled = self._resolve_bool("data_plane", default=self.selected_robot == "go2" and self.tier == "full")
        self.prepare_memory = self._resolve_bool("prepare_memory", default=self.memory_mode == "local")
        self.prepare_yolo = self._resolve_bool("prepare_yolo", default=self.perception_mode != "disabled")
        self.export_yolo_engine = self._resolve_bool("export_yolo_engine", default=False)
        self.build_geometry = self._resolve_bool("build_geometry", default=self.geometry_enabled)
        self.build_go2slam = self._resolve_bool("build_go2slam", default=self.selected_robot == "go2" and self.build_geometry)
        self.install_geometry_deps = self._resolve_bool("install_geometry_deps", default=self.geometry_enabled or self.build_geometry or self.build_go2slam)
        self.repair_ros2_apt_source = self._resolve_bool("repair_ros2_apt_source", default=True)
        self.start_gateway = self._resolve_bool("start_gateway", default=False)
        self.run_healthcheck = self._resolve_bool("healthcheck", default=False)
        self.mcp_mode = self._resolve_choice("mcp_mode", ("auto", "config-only", "skip"), default="auto")
        self.health_url = str(args.health_url or DEFAULT_HEALTH_URL).strip() or DEFAULT_HEALTH_URL
        self.agent_token = str(args.agent_token or self.root_existing.get("NUWAX_GATEWAY_AGENT_TOKENS", DEFAULT_AGENT_TOKEN)).strip() or DEFAULT_AGENT_TOKEN
        self.admin_token = str(args.admin_token or self.root_existing.get("NUWAX_GATEWAY_ADMIN_TOKENS", DEFAULT_ADMIN_TOKEN)).strip() or DEFAULT_ADMIN_TOKEN
        self.install_nuwaclaw_docker = self._resolve_bool("install_nuwaclaw_docker", default=False)
        self.nuwaclaw_saved_key = str(args.nuwaclaw_saved_key or os.environ.get("NUWAX_SAVED_KEY", "")).strip()
        self.nuwaclaw_host_mount = self._resolve_choice(
            "nuwaclaw_host_mount",
            ("none", "ro", "rw"),
            default=DEFAULT_NUWACLAW_HOST_MOUNT,
        )
        self.systemd_output = Path(args.systemd_output).expanduser().resolve() if args.systemd_output else DEFAULT_SYSTEMD_OUTPUT
        self.systemd_user = str(args.systemd_user or DEFAULT_SYSTEMD_USER).strip() or DEFAULT_SYSTEMD_USER
        self.systemd_group = str(args.systemd_group or self.systemd_user).strip() or self.systemd_user
        self.external_mcp_output = Path(args.external_mcp_output).expanduser().resolve() if args.external_mcp_output else DEFAULT_EXTERNAL_MCP_OUTPUT
        self.proxy_mcp_output = Path(args.proxy_mcp_output).expanduser().resolve() if args.proxy_mcp_output else DEFAULT_PROXY_MCP_OUTPUT
        if args.robot_env_output:
            self.robot_env_output = Path(args.robot_env_output).expanduser().resolve()
        elif args.root_env_output and self.robot_meta.env_target is not None:
            self.robot_env_output = self.root_target.with_name(f"{self.selected_robot}_{self.robot_meta.env_target.name}")
        else:
            self.robot_env_output = self.robot_meta.env_target

    def _resolve_robot(self) -> str:
        value = str(self.args.robot or self.root_existing.get("NUWAX_ACTIVE_ROBOT", "go2")).strip().lower() or "go2"
        if value not in ROBOT_DRIVER_REGISTRY:
            supported = ", ".join(_build_robot_choices())
            raise BootstrapError(f"不支持的机器人类型: {value}。当前支持: {supported}")
        return value

    def _resolve_tier(self) -> str:
        value = str(self.args.tier or "").strip().lower()
        if not value:
            value = ROBOT_BOOTSTRAP_METADATA[self.selected_robot].recommended_tier
        if value not in {"full", "lite"}:
            raise BootstrapError("tier 只支持 full 或 lite。")
        return value

    def _resolve_bool(self, name: str, *, default: bool) -> bool:
        value = getattr(self.args, name, None)
        if value is None:
            return default
        return bool(value)

    def _resolve_choice(self, name: str, choices: Sequence[str], *, default: str) -> str:
        value = str(getattr(self.args, name, "") or "").strip().lower()
        if not value:
            value = default
        if value not in choices:
            allowed = ", ".join(choices)
            raise BootstrapError(f"{name} 只支持: {allowed}")
        return value

    def root_overrides(self) -> Dict[str, str]:
        go2_geometry_enabled = self.selected_robot == "go2" and self.geometry_enabled
        ros_setup = _detect_ros_setup() if go2_geometry_enabled else None
        values: Dict[str, str] = {
            "NUWAX_ACTIVE_ROBOT": self.selected_robot,
            "NUWAX_PYTHON_BIN": self.python_bin,
            "NUWAX_GEOMETRY_ENABLED": _bool_text(go2_geometry_enabled),
            "NUWAX_GEOMETRY_REQUIRE_ROS2": _bool_text(go2_geometry_enabled),
            "NUWAX_ROS_SETUP_SCRIPT": str(ros_setup) if ros_setup is not None else "",
            "NUWAX_GEOMETRY_STACK": "disabled",
            "NUWAX_GEOMETRY_SETUP_SCRIPT": str(PROJECT_ROOT / "services" / "geometry" / "ros2_ws" / "install" / "setup.bash"),
            "NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR": "",
            "NUWAX_GEOMETRY_ODOM_TOPIC": "",
            "NUWAX_GEOMETRY_SECONDARY_ODOM_TOPICS": "",
            "NUWAX_GEOMETRY_IMU_TOPIC": "",
            "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC": "",
            "NUWAX_GEOMETRY_TF_TOPIC": "",
            "NUWAX_GEOMETRY_TF_STATIC_TOPIC": "",
            "NUWAX_GEOMETRY_OCCUPANCY_TOPIC": "",
            "NUWAX_GEOMETRY_COST_MAP_TOPIC": "",
            "NUWAX_GEOMETRY_MAP_FRAME": "",
            "NUWAX_GEOMETRY_ODOM_FRAME": "",
            "NUWAX_GEOMETRY_BASE_FRAME": "",
            "NUWAX_GEOMETRY_LIDAR_FRAME": "",
            "NUWAX_NAV2_MODE": "disabled",
            "NUWAX_NAV2_ACTION_NAME": "",
            "NUWAX_NAV2_LAUNCH_PACKAGE": "",
            "NUWAX_NAV2_LAUNCH_FILE": "",
            "NUWAX_NAV2_GLOBAL_COSTMAP_TOPIC": "",
            "NUWAX_NAV2_LOCAL_COSTMAP_TOPIC": "",
            "NUWAX_NAV2_COSTMAP_SOURCE": "",
            "NUWAX_GEOMETRY_EXTERNAL_SCRIPT": "",
            "NUWAX_GEOMETRY_EXTERNAL_ARGS": "",
            "NUWAX_GEOMETRY_EXTERNAL_PROCESS_NAME": "",
            "NUWAX_GEOMETRY_EXTERNAL_LOG_NAME": "",
            "NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED": "false",
            "NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED": "false",
            "NUWAX_LOG_DIR": str(PROJECT_ROOT),
            "NUWAX_LOG_FILE": str(PROJECT_ROOT / "nuwax_robot_bridge.log"),
            "NUWAX_GATEWAY_ARTIFACT_DIR": str(PROJECT_ROOT / "artifacts"),
            "NUWAX_MAP_CATALOG_ROOT": str(PROJECT_ROOT / "runtime_data" / "maps"),
            "NUWAX_RTABMAP_DATABASE_ROOT": str(PROJECT_ROOT / "runtime_data" / "geometry" / "rtabmap"),
            "NUWAX_MEMORY_DB_PATH": str(PROJECT_ROOT / "runtime_data" / "memory" / "vector_memory.db"),
            "NUWAX_GATEWAY_AGENT_TOKENS": self.agent_token,
            "NUWAX_GATEWAY_ADMIN_TOKENS": self.admin_token,
            "NUWAX_RELAY_ENABLED": _bool_text(self.relay_enabled),
        }
        if self.selected_robot == "go2" and self.geometry_enabled:
            values.update(
                {
                    "NUWAX_GEOMETRY_STACK": "external_slam_nav2",
                    "NUWAX_GEOMETRY_CYCLONEDDS_LIB_DIR": "/usr/local/lib",
                    "NUWAX_GEOMETRY_ODOM_TOPIC": "/odom",
                    "NUWAX_GEOMETRY_SECONDARY_ODOM_TOPICS": "/uslam/localization/odom,/lio_sam_ros2/mapping/odometry,/odom_lio",
                    "NUWAX_GEOMETRY_IMU_TOPIC": "/utlidar/imu",
                    "NUWAX_GEOMETRY_POINT_CLOUD_TOPIC": "/utlidar/cloud_deskewed",
                    "NUWAX_GEOMETRY_TF_TOPIC": "/tf",
                    "NUWAX_GEOMETRY_TF_STATIC_TOPIC": "/tf_static",
                    "NUWAX_GEOMETRY_OCCUPANCY_TOPIC": "/map",
                    "NUWAX_GEOMETRY_COST_MAP_TOPIC": "/global_costmap/costmap",
                    "NUWAX_GEOMETRY_MAP_FRAME": "map",
                    "NUWAX_GEOMETRY_ODOM_FRAME": "odom",
                    "NUWAX_GEOMETRY_BASE_FRAME": "base_footprint",
                    "NUWAX_GEOMETRY_LIDAR_FRAME": "utlidar_lidar",
                    "NUWAX_RTABMAP_LAUNCH_PACKAGE": "rtabmap_launch",
                    "NUWAX_RTABMAP_LAUNCH_FILE": "rtabmap.launch.py",
                    "NUWAX_RTABMAP_OCCUPANCY_PROJECTION": "projected_occupancy",
                    "NUWAX_GEOMETRY_EXTERNAL_SCRIPT": str(PROJECT_ROOT / "services" / "geometry" / "start_rtabmap_sidecar.sh"),
                    "NUWAX_GEOMETRY_EXTERNAL_ARGS": "",
                    "NUWAX_GEOMETRY_EXTERNAL_PROCESS_NAME": "rtabmap_nav2",
                    "NUWAX_GEOMETRY_EXTERNAL_LOG_NAME": "rtabmap_nav2.log",
                    "NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED": "false",
                    "NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED": "false",
                    "NUWAX_NAV2_MODE": "external",
                    "NUWAX_NAV2_ACTION_NAME": "/navigate_to_pose",
                    "NUWAX_NAV2_LAUNCH_PACKAGE": "nav2_bringup",
                    "NUWAX_NAV2_LAUNCH_FILE": "navigation_launch.py",
                    "NUWAX_NAV2_GLOBAL_COSTMAP_TOPIC": "/global_costmap/costmap",
                    "NUWAX_NAV2_LOCAL_COSTMAP_TOPIC": "/local_costmap/costmap",
                    "NUWAX_NAV2_COSTMAP_SOURCE": "nav2_global",
                }
            )

        if self.memory_mode == "cloud":
            values.update(
                {
                    "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": DEFAULT_MEMORY_ONLINE_MODEL,
                    "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "768",
                    "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL": DEFAULT_MEMORY_ONLINE_MODEL,
                    "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM": "768",
                }
            )
        elif self.memory_mode == "local":
            values.update(
                {
                    "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": DEFAULT_MEMORY_LOCAL_TEXT_MODEL,
                    "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "1024",
                    "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL": DEFAULT_MEMORY_LOCAL_IMAGE_MODEL,
                    "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM": "512",
                }
            )
        else:
            values.update(
                {
                    "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": "hashing-v1",
                    "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "256",
                    "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL": "disabled",
                    "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM": "512",
                }
            )

        yolo_world_runtime = "tensorrt" if self.export_yolo_engine else "pytorch"
        yolo_world_int8 = "false"
        if self.perception_mode == "local":
            values.update(
                {
                    "NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND": "yolo_world",
                    "NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND": "simple_scene",
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": "true",
                    "NUWAX_PERCEPTION_OPENAI_ENABLED": "false",
                    "NUWAX_PERCEPTION_STREAM_ENABLED": "true",
                    "NUWAX_PERCEPTION_STREAM_AUTO_START": "false",
                    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL": DEFAULT_YOLO_WORLD_MODEL,
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENGINE_PATH": DEFAULT_YOLO_WORLD_ENGINE,
                    "NUWAX_PERCEPTION_YOLO_WORLD_RUNTIME": yolo_world_runtime,
                    "NUWAX_PERCEPTION_YOLO_WORLD_INT8": yolo_world_int8,
                    "NUWAX_PERCEPTION_TRACKER_BACKEND": "bytetrack",
                }
            )
        elif self.perception_mode == "hybrid":
            values.update(
                {
                    "NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND": "yolo_world",
                    "NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND": "hybrid_scene",
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": "true",
                    "NUWAX_PERCEPTION_OPENAI_ENABLED": "true",
                    "NUWAX_PERCEPTION_STREAM_ENABLED": "true",
                    "NUWAX_PERCEPTION_STREAM_AUTO_START": "false",
                    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL": DEFAULT_YOLO_WORLD_MODEL,
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENGINE_PATH": DEFAULT_YOLO_WORLD_ENGINE,
                    "NUWAX_PERCEPTION_YOLO_WORLD_RUNTIME": yolo_world_runtime,
                    "NUWAX_PERCEPTION_YOLO_WORLD_INT8": yolo_world_int8,
                    "NUWAX_PERCEPTION_TRACKER_BACKEND": "bytetrack",
                }
            )
        elif self.perception_mode == "cloud-scene":
            values.update(
                {
                    "NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND": "yolo_world",
                    "NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND": "openai_scene",
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": "true",
                    "NUWAX_PERCEPTION_OPENAI_ENABLED": "true",
                    "NUWAX_PERCEPTION_STREAM_ENABLED": "true",
                    "NUWAX_PERCEPTION_STREAM_AUTO_START": "false",
                    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL": DEFAULT_YOLO_WORLD_MODEL,
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENGINE_PATH": DEFAULT_YOLO_WORLD_ENGINE,
                    "NUWAX_PERCEPTION_YOLO_WORLD_RUNTIME": yolo_world_runtime,
                    "NUWAX_PERCEPTION_YOLO_WORLD_INT8": yolo_world_int8,
                    "NUWAX_PERCEPTION_TRACKER_BACKEND": "bytetrack",
                }
            )
        else:
            values.update(
                {
                    "NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND": "metadata_detector",
                    "NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND": "simple_scene",
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": "false",
                    "NUWAX_PERCEPTION_OPENAI_ENABLED": "false",
                    "NUWAX_PERCEPTION_STREAM_ENABLED": "false",
                    "NUWAX_PERCEPTION_STREAM_AUTO_START": "false",
                }
            )

        return values

    def robot_env_overrides(self) -> Dict[str, str]:
        if self.selected_robot != "go2":
            return {}
        values = {
            "GO2_DATA_PLANE_ENABLED": _bool_text(self.data_plane_enabled),
            "GO2_DIRECT_DDS_ENABLED": _bool_text(self.data_plane_enabled),
            "GO2_OFFICIAL_BACKEND_ENABLED": _bool_text(self.data_plane_enabled),
        }
        return values

    def summary(self) -> str:
        lines = [
            f"机器人: {self.selected_robot}",
            f"档位: {self.tier}",
            f"geometry: {_bool_text(self.geometry_enabled)}",
            f"memory_mode: {self.memory_mode}",
            f"perception_mode: {self.perception_mode}",
            f"data_plane: {_bool_text(self.data_plane_enabled)}",
            f"build_geometry: {_bool_text(self.build_geometry)}",
            f"build_go2slam: {_bool_text(self.build_go2slam)}",
            f"install_geometry_deps: {_bool_text(self.install_geometry_deps)}",
            f"repair_ros2_apt_source: {_bool_text(self.repair_ros2_apt_source)}",
            f"systemd: {_bool_text(self.systemd_enabled)}",
            f"relay: {_bool_text(self.relay_enabled)}",
            f"mcp_mode: {self.mcp_mode}",
        ]
        if self.install_nuwaclaw_docker:
            lines.extend(
                (
                    f"install_nuwaclaw_docker: {_bool_text(self.install_nuwaclaw_docker)}",
                    f"nuwaclaw_host_mount: {self.nuwaclaw_host_mount}",
                    f"nuwaclaw_saved_key: {_mask_secret(self.nuwaclaw_saved_key)}",
                )
            )
        return "\n".join(lines)


def _check_result(name: str, ok: bool, detail: str) -> str:
    status = "OK" if ok else "MISS"
    return f"[{status}] {name}: {detail}"


def run_checks(planner: BootstrapPlanner) -> int:
    results: List[Tuple[bool, str]] = []
    results.append((_file_exists(ROOT_ENV_TEMPLATE), _check_result("根模板", _file_exists(ROOT_ENV_TEMPLATE), str(ROOT_ENV_TEMPLATE))))
    results.append((_file_exists(START_SCRIPT), _check_result("统一启动入口", _file_exists(START_SCRIPT), str(START_SCRIPT))))
    if planner.use_project_venv:
        seed_name = Path(planner.seed_python_bin).name
        python_ok = _which(seed_name) is not None or Path(planner.seed_python_bin).exists()
        results.append((python_ok, _check_result("Python", python_ok, planner.seed_python_bin)))
        venv_ok = False
        if python_ok:
            venv_result = _run(
                (planner.seed_python_bin, "-c", "import venv, ensurepip"),
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                display_command=f"{planner.seed_python_bin} -c 'import venv, ensurepip'",
            )
            venv_ok = venv_result.returncode == 0
        results.append((venv_ok, _check_result("python venv", venv_ok, f"将创建 {PROJECT_ROOT / '.venv'}")))
    else:
        python_ok = _which(Path(planner.python_bin).name) is not None or Path(planner.python_bin).exists()
        results.append((python_ok, _check_result("Python", python_ok, planner.python_bin)))
        pip_ok = _which("pip3") is not None or _which("pip") is not None
        results.append((pip_ok, _check_result("pip", pip_ok, "pip3 或 pip")))
    results.append((_file_exists(PROJECT_ROOT / "requirements.txt"), _check_result("requirements.txt", _file_exists(PROJECT_ROOT / "requirements.txt"), str(PROJECT_ROOT / "requirements.txt"))))

    if planner.geometry_enabled or planner.build_geometry or planner.build_go2slam:
        ros_setup = _detect_ros_setup()
        results.append((ros_setup is not None, _check_result("ROS2", ros_setup is not None, str(ros_setup) if ros_setup else "未找到 /opt/ros/*/setup.bash")))
        ros_distro = _detect_ros_distro(ros_setup)
        results.append((bool(ros_distro), _check_result("ROS2 发行版", bool(ros_distro), ros_distro or "无法从 ROS_DISTRO 或 /opt/ros/*/setup.bash 推导")))
        colcon_ok = _which("colcon") is not None
        colcon_ready = colcon_ok or planner.install_geometry_deps
        results.append((colcon_ready, _check_result("colcon", colcon_ready, _which("colcon") or "将由 --install-geometry-deps 安装")))
        rosdep_ok = _which("rosdep") is not None
        rosdep_ready = rosdep_ok or planner.install_geometry_deps
        results.append((rosdep_ready, _check_result("rosdep", rosdep_ready, _which("rosdep") or "将由 --install-geometry-deps 安装")))
        root_template_values = _read_existing_env(ROOT_ENV_TEMPLATE)
        cyclone_value = (
            os.environ.get("CYCLONEDDS_URI", "").strip()
            or planner.root_existing.get("CYCLONEDDS_URI", "").strip()
            or root_template_values.get("CYCLONEDDS_URI", "").strip()
        )
        cyclone_ok = bool(cyclone_value)
        results.append((cyclone_ok, _check_result("CycloneDDS 配置", cyclone_ok, "环境变量、.env 或 .env.example 中需存在 CYCLONEDDS_URI")))
        if planner.install_geometry_deps:
            apt_ok = _which("apt-get") is not None
            results.append((apt_ok, _check_result("apt-get", apt_ok, _which("apt-get") or "未找到 apt-get，无法自动安装 RTAB-Map/Nav2")))
            sudo_ok = hasattr(os, "geteuid") and os.geteuid() == 0 or _which("sudo") is not None
            results.append((sudo_ok, _check_result("sudo/root", sudo_ok, "root 或 sudo 用于安装系统包")))
        if planner.build_go2slam or (planner.selected_robot == "go2" and planner.geometry_enabled):
            go2slam_ok = GO2SLAM_WS_DIR.exists()
            results.append((go2slam_ok, _check_result("Go2SLAM 工作区", go2slam_ok, str(GO2SLAM_WS_DIR) if go2slam_ok else "未找到 services/geometry/ros2_ws/src/go2slam")))
            setup_path = go2slam_setup_path()
            setup_ready = setup_path.exists() or planner.build_go2slam
            setup_detail = str(setup_path) if setup_path.exists() else "将由 --build-go2slam 构建生成"
            results.append((setup_ready, _check_result("Go2SLAM install/setup.bash", setup_ready, setup_detail)))
            if setup_path.exists():
                stale_source = find_go2slam_stale_source(setup_path)
                stale_ok = stale_source is None or planner.build_go2slam
                stale_detail = "源码未新于 install" if stale_source is None else f"{stale_source} 新于 install，将重新构建" if planner.build_go2slam else f"{stale_source} 新于 install，请重新构建"
                results.append((stale_ok, _check_result("Go2SLAM 构建新鲜度", stale_ok, stale_detail)))
                missing_runtime = missing_go2slam_runtime_packages(ros_setup) if ros_setup is not None else GO2SLAM_RUNTIME_REQUIRED_PACKAGES
                packages_ok = not missing_runtime or planner.build_go2slam
                packages_detail = "运行包可被当前 ROS2 环境解析" if not missing_runtime else "缺少: " + ", ".join(missing_runtime) + ("；将重新构建" if planner.build_go2slam else "")
                results.append((packages_ok, _check_result("Go2SLAM 运行包", packages_ok, packages_detail)))

    if planner.selected_robot == "go2" and planner.data_plane_enabled:
        go2_template_ok = planner.robot_meta.env_template is not None and planner.robot_meta.env_template.exists()
        results.append((go2_template_ok, _check_result("Go2 数据面模板", go2_template_ok, str(planner.robot_meta.env_template) if planner.robot_meta.env_template else "无")))

    if planner.memory_mode == "local":
        results.append((_file_exists(PROJECT_ROOT / "requirements-memory-models.txt"), _check_result("记忆模型依赖清单", _file_exists(PROJECT_ROOT / "requirements-memory-models.txt"), str(PROJECT_ROOT / "requirements-memory-models.txt"))))

    if planner.mcp_mode != "skip":
        results.append((_file_exists(DEFAULT_EXTERNAL_MCP_TEMPLATE), _check_result("外部 MCP 模板", _file_exists(DEFAULT_EXTERNAL_MCP_TEMPLATE), str(DEFAULT_EXTERNAL_MCP_TEMPLATE))))
        results.append((_file_exists(DEFAULT_PROXY_MCP_TEMPLATE), _check_result("SQLite MCP 模板", _file_exists(DEFAULT_PROXY_MCP_TEMPLATE), str(DEFAULT_PROXY_MCP_TEMPLATE))))

    if planner.run_healthcheck:
        token_ok = bool(planner.agent_token.strip())
        results.append((token_ok, _check_result("健康检查 token", token_ok, "将使用 Authorization: Bearer <agent token> 调用 /api/health")))

    if planner.install_nuwaclaw_docker:
        docker_ok = _which("docker") is not None
        results.append((docker_ok, _check_result("Docker CLI", docker_ok, _which("docker") or "未找到 docker 可执行文件")))
        saved_key_ok = bool(planner.nuwaclaw_saved_key)
        results.append(
            (
                saved_key_ok,
                _check_result(
                    "NuwaxClaw 个人密钥",
                    saved_key_ok,
                    "需提供 --nuwaclaw-saved-key 或环境变量 NUWAX_SAVED_KEY",
                ),
            )
        )

    for _, line in results:
        print(line)
    return 0 if all(ok for ok, _ in results) else 1


def write_root_env(planner: BootstrapPlanner) -> Path:
    content = _render_env_from_template(planner.root_template, planner.root_overrides())
    _write_text(planner.root_target, content)
    print(f"已生成根配置: {planner.root_target}")
    return planner.root_target


def write_robot_env(planner: BootstrapPlanner) -> Optional[Path]:
    if planner.robot_meta.env_template is None or planner.robot_env_output is None:
        print(f"机器人 {planner.selected_robot} 当前没有专属 env 模板，跳过生成机器人专属配置。")
        return None
    content = _render_env_from_template(planner.robot_meta.env_template, planner.robot_env_overrides())
    _write_text(planner.robot_env_output, content)
    print(f"已生成机器人专属配置: {planner.robot_env_output}")
    return planner.robot_env_output


def install_python_dependencies(planner: BootstrapPlanner) -> None:
    if not planner.install_python:
        print("跳过 Python 依赖安装。")
        return
    if planner.use_project_venv:
        venv_dir = PROJECT_ROOT / ".venv"
        venv_python = venv_dir / "bin" / "python"
        if not venv_python.exists():
            try:
                _run((planner.seed_python_bin, "-m", "venv", "--system-site-packages", str(venv_dir)), cwd=PROJECT_ROOT)
            except BootstrapError as exc:
                raise BootstrapError("创建项目虚拟环境失败，请先安装 python3-venv 后重试。") from exc
        planner_python = str(venv_python)
    else:
        planner_python = planner.python_bin
    _run((planner_python, "-m", "pip", "install", "-r", "requirements.txt"), cwd=PROJECT_ROOT)
    if planner.memory_mode == "local":
        _run((planner_python, "-m", "pip", "install", "-r", "requirements-memory-models.txt"), cwd=PROJECT_ROOT)


def install_geometry_system_dependencies(planner: BootstrapPlanner) -> None:
    if not planner.install_geometry_deps:
        print("跳过 geometry 系统依赖安装。")
        return
    if not (planner.geometry_enabled or planner.build_geometry or planner.build_go2slam):
        print("未启用 geometry，跳过 RTAB-Map/Nav2 系统依赖安装。")
        return

    ros_setup = _detect_ros_setup()
    if ros_setup is None:
        raise BootstrapError("未找到 ROS2 安装。请先安装 ROS2 Foxy/Humble/Jazzy，再安装 RTAB-Map/Nav2。")
    ros_distro = _detect_ros_distro(ros_setup)
    package_set = set(build_geometry_apt_packages(ros_distro))
    if planner.build_go2slam:
        package_set.update(build_go2slam_apt_packages(ros_distro))
    required_packages = tuple(sorted(package_set))
    optional_candidates = build_geometry_optional_apt_packages(ros_distro)
    sudo = _sudo_prefix()
    env = dict(os.environ)
    env["DEBIAN_FRONTEND"] = "noninteractive"

    print(f"准备安装 geometry 系统依赖，ROS2 发行版: {ros_distro}")
    run_geometry_apt_update(sudo, env)
    optional_packages = filter_available_apt_packages(optional_candidates)
    packages = (*required_packages, *optional_packages)
    print("将安装: " + " ".join(packages))
    install_geometry_apt_packages(planner, sudo, env, packages)

    if _which("rosdep") is None:
        print("未找到 rosdep，跳过工作区 rosdep 依赖解析。")
        return

    rosdep_sources = Path("/etc/ros/rosdep/sources.list.d/20-default.list")
    if not rosdep_sources.exists():
        init_result = _run((*sudo, "rosdep", "init"), cwd=PROJECT_ROOT, check=False)
        if init_result.returncode != 0:
            print("rosdep init 未成功；如果已经初始化过可忽略，否则请手工执行 sudo rosdep init。")
    _run(("rosdep", "update"), cwd=PROJECT_ROOT)
    _run(
        (
            "rosdep",
            "install",
            "--from-paths",
            "src/vendor",
            "--ignore-src",
            "-r",
            "-y",
            "--rosdistro",
            ros_distro,
        ),
        cwd=GEOMETRY_WS_DIR,
    )
    if planner.build_go2slam:
        assert_go2slam_workspace_ready(ros_setup)
        _run(
            (
                "rosdep",
                "install",
                "--from-paths",
                "src",
                "--ignore-src",
                "-r",
                "-y",
                "--rosdistro",
                ros_distro,
            ),
            cwd=GO2SLAM_WS_DIR,
        )


def assert_go2slam_workspace_ready(ros_setup: Path) -> None:
    if not GO2SLAM_WS_DIR.exists():
        raise BootstrapError(f"未找到 Go2SLAM 工作区: {GO2SLAM_WS_DIR}")
    missing: List[str] = []
    for package_name in ("unitree_go", "unitree_api"):
        command = f"source {shlex.quote(str(ros_setup))} && ros2 pkg prefix {shlex.quote(package_name)} >/dev/null"
        result = _run(("bash", "-lc", command), cwd=GO2SLAM_WS_DIR, check=False, capture_output=True)
        if result.returncode != 0:
            missing.append(package_name)
    if missing:
        raise BootstrapError(
            "Go2SLAM 构建前缺少 Unitree ROS2 接口包: "
            + ", ".join(missing)
            + "。请先按 Unitree 官方说明安装或编译 unitree_go/unitree_api，并确认 `ros2 pkg prefix unitree_go` 与 `ros2 pkg prefix unitree_api` 可用。"
        )


def assert_go2slam_runtime_ready(ros_setup: Path) -> None:
    setup_path = go2slam_setup_path()
    if not setup_path.exists():
        raise BootstrapError(
            f"缺少 Go2SLAM install/setup.bash: {setup_path}。"
            f"请执行 `cd {GO2SLAM_WS_DIR} && source {ros_setup} && colcon build --symlink-install`。"
        )
    stale_source = find_go2slam_stale_source(setup_path)
    if stale_source is not None:
        raise BootstrapError(
            f"Go2SLAM 工作区需要重新编译，源码文件新于 install: {stale_source}。"
            f"请执行 `cd {GO2SLAM_WS_DIR} && source {ros_setup} && colcon build --symlink-install`。"
        )
    missing_runtime = missing_go2slam_runtime_packages(ros_setup)
    if missing_runtime:
        raise BootstrapError(
            "Go2SLAM install 中缺少运行包: "
            + ", ".join(missing_runtime)
            + f"。请执行 `cd {GO2SLAM_WS_DIR} && source {ros_setup} && colcon build --symlink-install`。"
        )


def build_workspaces(planner: BootstrapPlanner) -> None:
    geometry_script = PROJECT_ROOT / "services" / "geometry" / "build_ros2_workspace.sh"
    if planner.build_geometry and geometry_script.exists():
        _run(("bash", str(geometry_script)), cwd=PROJECT_ROOT)
    if planner.build_go2slam:
        ros_setup = _detect_ros_setup()
        if ros_setup is None:
            raise BootstrapError("未找到 ROS2 安装，不能构建 Go2SLAM 工作区。")
        assert_go2slam_workspace_ready(ros_setup)
        _run(
            (
                "bash",
                "-lc",
                "source "
                + shlex.quote(str(ros_setup))
                + " && colcon --log-base log build --symlink-install --parallel-workers 1 --build-base build --install-base install",
            ),
            cwd=GO2SLAM_WS_DIR,
        )
        assert_go2slam_runtime_ready(ros_setup)


def prepare_assets(planner: BootstrapPlanner) -> None:
    if planner.prepare_yolo:
        _run(
            (
                planner.python_bin,
                "scripts/prepare_yolo_world_tensorrt.py",
                "--weights-output",
                DEFAULT_YOLO_WORLD_MODEL,
                "--engine-output",
                DEFAULT_YOLO_WORLD_ENGINE,
                "--download-only",
            ),
            cwd=PROJECT_ROOT,
        )
    if planner.export_yolo_engine:
        _run(
            (
                planner.python_bin,
                "scripts/prepare_yolo_world_tensorrt.py",
                "--weights-output",
                DEFAULT_YOLO_WORLD_MODEL,
                "--engine-output",
                DEFAULT_YOLO_WORLD_ENGINE,
                "--no-int8",
            ),
            cwd=PROJECT_ROOT,
        )
    if planner.prepare_memory:
        _run((planner.python_bin, "scripts/prepare_memory_models.py"), cwd=PROJECT_ROOT)


def _build_nuwaclaw_host_mount(mode: str) -> Optional[str]:
    if mode == "ro":
        return "/:/host:ro"
    if mode == "rw":
        return "/:/host:rw"
    return None


def build_nuwaclaw_docker_run_command(planner: BootstrapPlanner) -> List[str]:
    if not planner.nuwaclaw_saved_key:
        raise BootstrapError("已选择安装 NuwaxClaw Docker 客户端，但未提供个人密钥。")

    command = [
        "docker",
        "run",
        "-d",
        "--name",
        DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME,
        "--restart",
        "always",
        "--add-host",
        "host.docker.internal:host-gateway",
        "-p",
        f"{DEFAULT_NUWACLAW_WEB_PORT}:{DEFAULT_NUWACLAW_WEB_PORT}",
        "-p",
        f"{DEFAULT_NUWACLAW_FILE_SERVER_PORT}:{DEFAULT_NUWACLAW_FILE_SERVER_PORT}",
        "-p",
        f"{DEFAULT_NUWACLAW_AGENT_PORT}:{DEFAULT_NUWACLAW_AGENT_PORT}",
        "-v",
        f"{DEFAULT_NUWACLAW_CLIENT_VOLUME_NAME}:/home/user",
    ]
    host_mount = _build_nuwaclaw_host_mount(planner.nuwaclaw_host_mount)
    if host_mount is not None:
        command.extend(("-v", host_mount))
    command.extend(
        (
            "-e",
            "TZ=Asia/Shanghai",
            "-e",
            f"NUWAX_SERVER_HOST={DEFAULT_NUWACLAW_SERVER_HOST}",
            "-e",
            f"NUWAX_AGENT_PORT={DEFAULT_NUWACLAW_AGENT_PORT}",
            "-e",
            f"NUWAX_FILE_SERVER_PORT={DEFAULT_NUWACLAW_FILE_SERVER_PORT}",
            "-e",
            f"NUWAX_WORKSPACE_DIR={DEFAULT_NUWACLAW_WORKSPACE_DIR}",
            "-e",
            f"NUWAX_SAVED_KEY={planner.nuwaclaw_saved_key}",
            DEFAULT_NUWACLAW_CLIENT_IMAGE,
        )
    )
    return command


def _build_redacted_nuwaclaw_docker_command(command: Sequence[str]) -> str:
    redacted: List[str] = []
    for part in command:
        if part.startswith("NUWAX_SAVED_KEY="):
            redacted.append(f"NUWAX_SAVED_KEY={_mask_secret(part.split('=', 1)[1])}")
        else:
            redacted.append(part)
    return shlex.join(redacted)


def _wait_for_nuwaclaw_client_files(timeout_seconds: float = 60.0) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if DEFAULT_NUWACLAW_PROXY_SCRIPT.exists():
            return True
        time.sleep(2.0)
    return DEFAULT_NUWACLAW_PROXY_SCRIPT.exists()


def install_nuwaclaw_client(planner: BootstrapPlanner) -> None:
    if not planner.install_nuwaclaw_docker:
        print("跳过 NuwaxClaw Docker 客户端安装。")
        return

    if _which("docker") is None:
        raise BootstrapError("未找到 docker，无法执行 NuwaxClaw Docker 客户端安装。")

    if planner.nuwaclaw_host_mount == "rw":
        print("警告: 当前选择把宿主机根目录以读写模式挂载到容器 /host，这会带来宿主机文件被修改的风险。")

    inspect_result = _run(
        ("docker", "container", "inspect", DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME),
        check=False,
        capture_output=True,
        display_command=f"docker container inspect {DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME}",
    )
    if inspect_result.returncode == 0:
        print("检测到已有 nuwax-client 容器，将删除后按当前参数重建。")
        _run(("docker", "rm", "-f", DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME))

    command = build_nuwaclaw_docker_run_command(planner)
    _run(command, display_command=_build_redacted_nuwaclaw_docker_command(command))
    print(
        "已启动 NuwaxClaw Docker 客户端: "
        f"{DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME}，"
        f"端口映射 {DEFAULT_NUWACLAW_WEB_PORT}/{DEFAULT_NUWACLAW_FILE_SERVER_PORT}/{DEFAULT_NUWACLAW_AGENT_PORT}，"
        f"宿主机挂载模式 {planner.nuwaclaw_host_mount}。"
    )

    if planner.mcp_mode == "auto" and not _wait_for_nuwaclaw_client_files(timeout_seconds=30.0):
        print(
            "NuwaxClaw 容器已启动，但持久卷中的 MCP proxy 文件尚未就绪；"
            "如果后续自动安装外部 MCP hook 失败，可稍后重跑 bootstrap 或单独执行 hook 安装。"
        )


def render_mcp_payload(token: str) -> Dict[str, object]:
    return {
        "mcpServers": {
            "nuwax_robot_bridge": {
                "url": "http://host.docker.internal:8766/mcp",
                "headers": {"Authorization": f"Bearer {token}"},
            }
        }
    }


def configure_mcp(planner: BootstrapPlanner) -> None:
    if planner.mcp_mode == "skip":
        print("跳过 MCP 注入。")
        return

    payload = render_mcp_payload(planner.agent_token)
    _write_text(planner.external_mcp_output, json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    _write_text(planner.proxy_mcp_output, json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    print(f"已生成外部 MCP 配置: {planner.external_mcp_output}")
    print(f"已生成 SQLite MCP 声明式配置: {planner.proxy_mcp_output}")

    if planner.mcp_mode == "config-only":
        print("当前为 config-only，只生成配置文件，不自动修改目标环境。")
        return

    hook_script = PROJECT_ROOT / "scripts" / "nuwaclaw_external_mcp_hook.py"
    if hook_script.exists():
        try:
            _run(
                (
                    planner.python_bin,
                    str(hook_script),
                    "--external-config",
                    str(planner.external_mcp_output),
                    "install",
                ),
                cwd=PROJECT_ROOT,
            )
        except BootstrapError as exc:
            print(f"自动安装外部 MCP hook 失败，已保留配置文件供手工执行: {exc}")
    else:
        print("未找到 MCP hook 安装脚本，已仅生成配置文件。")


def write_systemd_example(planner: BootstrapPlanner) -> Path:
    service_content = textwrap.dedent(
        f"""\
        [Unit]
        Description=nuwax_robot_bridge 宿主机网关
        After=network-online.target
        Wants=network-online.target

        [Service]
        Type=simple
        User={planner.systemd_user}
        Group={planner.systemd_group}
        WorkingDirectory={PROJECT_ROOT}
        Environment=PYTHONUNBUFFERED=1
        ExecStart=/bin/bash {START_SCRIPT}
        Restart=always
        RestartSec=3
        TimeoutStopSec=20
        ReadWritePaths=/var/log/nuwax_robot_bridge /var/lib/nuwax_robot_bridge

        [Install]
        WantedBy=multi-user.target
        """
    )
    _write_text(planner.systemd_output, service_content)
    print(f"已生成 systemd 示例: {planner.systemd_output}")
    return planner.systemd_output


def validate_requested_actions(planner: BootstrapPlanner) -> None:
    if not planner.install_nuwaclaw_docker:
        return
    if _which("docker") is None:
        raise BootstrapError("已选择安装 NuwaxClaw Docker 客户端，但当前环境未找到 docker。")
    if not planner.nuwaclaw_saved_key:
        raise BootstrapError("已选择安装 NuwaxClaw Docker 客户端，但未提供个人密钥。")


def run_bootstrap_actions(planner: BootstrapPlanner) -> None:
    validate_requested_actions(planner)
    write_root_env(planner)
    write_robot_env(planner)
    install_python_dependencies(planner)
    install_geometry_system_dependencies(planner)
    build_workspaces(planner)
    prepare_assets(planner)
    install_nuwaclaw_client(planner)
    configure_mcp(planner)
    if planner.systemd_enabled:
        write_systemd_example(planner)
    start_gateway(planner)
    run_healthcheck(planner)


def start_gateway(planner: BootstrapPlanner) -> None:
    if not planner.start_gateway:
        print("跳过自动启动 gateway。")
        return
    if planner.selected_robot == "go2" and planner.geometry_enabled:
        ros_setup = _detect_ros_setup()
        if ros_setup is None:
            raise BootstrapError("未找到 ROS2 安装，不能启动 Go2 geometry 主链。")
        assert_go2slam_runtime_ready(ros_setup)
    command = f"nohup bash {START_SCRIPT} > /tmp/nuwax_bootstrap_gateway.log 2>&1 &"
    _run_shell(command, cwd=PROJECT_ROOT)
    print("已在后台启动 gateway，日志位于 /tmp/nuwax_bootstrap_gateway.log")


def run_healthcheck(planner: BootstrapPlanner) -> None:
    if not planner.run_healthcheck:
        print("跳过健康检查。")
        return
    request = urllib.request.Request(planner.health_url)
    request.add_header("Authorization", f"Bearer {planner.agent_token}")
    try:
        with urllib.request.urlopen(request, timeout=10.0) as response:
            payload = response.read().decode("utf-8", errors="replace")
    except Exception as exc:
        raise BootstrapError(f"健康检查失败: {exc}") from exc
    print(f"健康检查通过: {planner.health_url}")
    print(payload)


def prompt_choice(title: str, choices: Sequence[str], default: str) -> str:
    choice_line = "/".join(choices)
    answer = input(f"{title} [{choice_line}] (默认 {default}): ").strip().lower()
    return answer if answer in choices else default


def prompt_bool(title: str, default: bool) -> bool:
    default_text = "Y/n" if default else "y/N"
    answer = input(f"{title} [{default_text}]: ").strip().lower()
    if not answer:
        return default
    return answer in {"y", "yes", "1", "true", "on"}


def prompt_non_empty(title: str, *, secret: bool = False) -> str:
    while True:
        answer = (getpass.getpass(f"{title}: ") if secret else input(f"{title}: ")).strip()
        if answer:
            return answer
        print("该项不能为空。")


def run_wizard(args: argparse.Namespace) -> int:
    args.robot = prompt_choice("选择机器人", _build_robot_choices(), "go2")
    recommended_tier = ROBOT_BOOTSTRAP_METADATA[args.robot].recommended_tier
    args.tier = prompt_choice("选择部署档位", ("full", "lite"), recommended_tier)
    args.geometry = prompt_bool("启用 geometry 平台侧车", args.tier == "full")
    args.data_plane = prompt_bool("启用 Go2 数据面", args.robot == "go2" and args.tier == "full")
    args.memory_mode = prompt_choice("选择 memory 模式", ("cloud", "local", "disabled"), "cloud" if args.tier == "full" else "disabled")
    args.perception_mode = prompt_choice("选择 perception 模式", ("local", "hybrid", "cloud-scene", "disabled"), "hybrid" if args.tier == "full" else "local")
    args.relay = prompt_bool("启用 relay", False)
    args.systemd = prompt_bool("生成 systemd 示例", True)
    args.install_python = prompt_bool("安装 Python 依赖", True)
    args.build_geometry = prompt_bool("构建 geometry ROS2 工作空间", bool(args.geometry))
    args.build_go2slam = prompt_bool("单独构建 Go2SLAM 工作区", args.robot == "go2" and bool(args.build_geometry))
    args.install_geometry_deps = prompt_bool("安装 RTAB-Map/Nav2 系统依赖", bool(args.geometry))
    args.prepare_yolo = prompt_bool("准备 YOLO-World 权重", args.perception_mode != "disabled")
    args.prepare_memory = prompt_bool("准备本地 memory 模型", args.memory_mode == "local")
    args.mcp_mode = prompt_choice("选择 MCP 注入模式", ("auto", "config-only", "skip"), "auto")
    args.install_nuwaclaw_docker = prompt_bool("安装 NuwaxClaw Docker 客户端", False)
    if args.install_nuwaclaw_docker:
        args.nuwaclaw_host_mount = prompt_choice(
            "选择宿主机根目录挂载模式（rw 有风险）",
            ("none", "ro", "rw"),
            DEFAULT_NUWACLAW_HOST_MOUNT,
        )
        if args.nuwaclaw_host_mount == "rw":
            print("警告: 读写挂载会让容器可以直接修改宿主机根目录，请确认你明确接受该风险。")
        args.nuwaclaw_saved_key = prompt_non_empty(
            "请输入个人密钥（将写入 NUWAX_SAVED_KEY）",
            secret=True,
        )
    planner = BootstrapPlanner(args)
    print("\n当前部署选择：")
    print(planner.summary())
    run_bootstrap_actions(planner)
    return 0


def run_apply(args: argparse.Namespace) -> int:
    planner = BootstrapPlanner(args)
    print("部署参数：")
    print(planner.summary())
    run_bootstrap_actions(planner)
    return 0


def add_boolean_flag(parser: argparse.ArgumentParser, name: str, help_text: str) -> None:
    parser.add_argument(f"--{name.replace('_', '-')}", dest=name, action="store_true", help=help_text)
    parser.add_argument(f"--no-{name.replace('_', '-')}", dest=name, action="store_false", help=f"关闭{help_text}")
    parser.set_defaults(**{name: None})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="nuwax_robot_bridge 一键部署入口。")
    parser.add_argument("--python-bin", default="", help="Python 解释器路径；默认创建并使用项目 .venv。")

    subparsers = parser.add_subparsers(dest="command", required=True)

    check_parser = subparsers.add_parser("check", help="按当前选择执行前置检查。")
    wizard_parser = subparsers.add_parser("wizard", help="交互式收集参数并执行部署动作。")
    apply_parser = subparsers.add_parser("apply", help="执行配置生成与部署动作。")

    for subparser in (check_parser, wizard_parser, apply_parser):
        subparser.add_argument("--robot", choices=_build_robot_choices(), help="机器人类型。")
        subparser.add_argument("--tier", choices=("full", "lite"), help="部署档位。")
        subparser.add_argument("--memory-mode", choices=("cloud", "local", "disabled"), help="记忆模式。")
        subparser.add_argument("--perception-mode", choices=("local", "hybrid", "cloud-scene", "disabled"), help="感知模式。")
        subparser.add_argument("--mcp-mode", choices=("auto", "config-only", "skip"), help="MCP 注入模式。")
        subparser.add_argument("--agent-token", default="", help="健康检查与 MCP 默认使用的 agent token。")
        subparser.add_argument("--admin-token", default="", help="写入 .env 的 admin token。")
        subparser.add_argument("--nuwaclaw-saved-key", default="", help="安装 NuwaxClaw Docker 客户端时写入的个人密钥。")
        subparser.add_argument(
            "--nuwaclaw-host-mount",
            choices=("none", "ro", "rw"),
            default="",
            help="把宿主机根目录挂载到容器 /host 的模式：none / ro / rw。",
        )
        subparser.add_argument("--health-url", default=DEFAULT_HEALTH_URL, help="健康检查 URL。")
        subparser.add_argument("--systemd-output", default="", help="systemd 示例输出路径。")
        subparser.add_argument("--systemd-user", default="", help="systemd 运行用户。")
        subparser.add_argument("--systemd-group", default="", help="systemd 运行组。")
        subparser.add_argument("--root-env-output", default="", help="根 .env 输出路径。")
        subparser.add_argument("--robot-env-output", default="", help="机器人专属 env 输出路径。")
        subparser.add_argument("--external-mcp-output", default="", help="外部 MCP JSON 输出路径。")
        subparser.add_argument("--proxy-mcp-output", default="", help="SQLite MCP JSON 输出路径。")
        add_boolean_flag(subparser, "geometry", "geometry 平台侧车")
        add_boolean_flag(subparser, "data_plane", "Go2 数据面")
        add_boolean_flag(subparser, "systemd", "生成 systemd 示例")
        add_boolean_flag(subparser, "relay", "relay 配置")
        add_boolean_flag(subparser, "install_nuwaclaw_docker", "安装 NuwaxClaw Docker 客户端")
        add_boolean_flag(subparser, "install_python", "安装 Python 依赖")
        add_boolean_flag(subparser, "prepare_memory", "准备本地记忆模型")
        add_boolean_flag(subparser, "prepare_yolo", "准备 YOLO-World 权重")
        add_boolean_flag(subparser, "export_yolo_engine", "导出 YOLO-World TensorRT 引擎")
        add_boolean_flag(subparser, "build_geometry", "构建 geometry ROS2 工作空间")
        add_boolean_flag(subparser, "build_go2slam", "Go2 默认额外构建 services/geometry/ros2_ws/src/go2slam")
        add_boolean_flag(subparser, "install_geometry_deps", "安装 RTAB-Map/Nav2 系统依赖")
        add_boolean_flag(subparser, "repair_ros2_apt_source", "自动修复异常 ROS2 apt 源")
        add_boolean_flag(subparser, "start_gateway", "启动 gateway")
        add_boolean_flag(subparser, "healthcheck", "执行健康检查")

    apply_parser.add_argument("--non-interactive", action="store_true", help="保留兼容参数；当前 apply 默认即为无交互模式。")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "check":
            return run_checks(BootstrapPlanner(args))
        if args.command == "wizard":
            return run_wizard(args)
        if args.command == "apply":
            return run_apply(args)
        parser.error("未知命令")
        return 2
    except BootstrapError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
