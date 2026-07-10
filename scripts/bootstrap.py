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
# 去 ROS2 化（2026-07-03 drop-ros2）：所有 ROS2 工作区常量 / 包后缀 / 运行时检查常量
# 已物理删除。原包含的 GEOMETRY_WS_DIR / GO2SLAM_WS_DIR / GEOMETRY_ROS_* / GO2SLAM_*
# / GO2SLAM_RUNTIME_REQUIRED_PACKAGES / GO2SLAM_SOURCE_STALE_SUFFIXES 等均整体下线；
# geometry runtime 由 ``nuwax-geometry-runtime`` Rust 二进制承载，平台 Python 侧不再
# 推导 ROS2 工作区路径。
# DEFAULT_ROS2_APT_FALLBACK_URLS：
# 已删除；geometry runtime 改由 ``nuwax-geometry-runtime`` 提供，未再需要 ROS2 apt 源。


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
    for raw_line in template.splitlines():
        stripped = raw_line.strip()
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


# 去 ROS2 化（2026-07-03 drop-ros2）：以下 ROS2 探测 / 运行时检查函数已删除
# - _detect_ros_setup / _detect_ros_distro（探测 /opt/ros/*）
# - go2slam_setup_path / find_go2slam_stale_source / missing_go2slam_runtime_packages
# - build_geometry_apt_packages / build_go2slam_apt_packages / build_geometry_optional_apt_packages
# - apt_output_has_ros2_archive_404 / build_ros2_apt_fallback_urls / replace_ros2_apt_source_url
# - filter_available_apt_packages / run_geometry_apt_update / install_geometry_apt_packages
# - install_geometry_system_dependencies / assert_go2slam_workspace_ready /
#   assert_go2slam_runtime_ready / build_workspaces
# 替换说明：geometry runtime 已迁到 ``nuwax-geometry-runtime`` Rust 二进制，平台侧不再
# 承担 ROS2 工作区构建、Nav2 / RTAB-Map ROS 适配、rosdep / colcon / apt-get 依赖解析。


def _sudo_prefix() -> Tuple[str, ...]:
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        return ()
    if _which("sudo") is None:
        raise BootstrapError("需要 sudo 执行系统依赖安装，但当前环境未找到 sudo。")
    return ("sudo",)


# 去 ROS2 化（2026-07-03 drop-ros2）：以下 ROS2 apt 派生 / 源回退 / 安装辅助函数已删除
# - build_geometry_apt_packages / build_go2slam_apt_packages / build_geometry_optional_apt_packages
# - apt_output_has_ros2_archive_404 / build_ros2_apt_fallback_urls / replace_ros2_apt_source_url
# - filter_available_apt_packages / run_geometry_apt_update / install_geometry_apt_packages
# 替换说明：geometry runtime 由 ``nuwax-geometry-runtime`` Rust 二进制承载，平台侧
# 不再推导 ROS2 发行版 / 派生 apt 包名 / 处理 ROS2 apt 源 404 回退；
# installer 的 system 阶段只负责通用基础工具 + nuwax-geometry-runtime 的 OS 依赖。
# 注：原实现引用已删除的 GEOMETRY_ROS_*_PACKAGE_SUFFIXES 常量；本函数组整体下线。


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
        build_go2_rtabnav2 = bool(getattr(args, "build_go2_rtabnav2", False))
        self.build_go2slam = build_go2_rtabnav2 or self._resolve_bool(
            "build_go2slam",
            default=self.selected_robot == "go2" and self.build_geometry,
        )
        self.install_geometry_deps = self._resolve_bool("install_geometry_deps", default=self.geometry_enabled or self.build_geometry or self.build_go2slam)
        self.repair_ros2_apt_source = self._resolve_bool("repair_ros2_apt_source", default=False)
        self.start_gateway = self._resolve_bool("start_gateway", default=False)
        self.run_healthcheck = self._resolve_bool("healthcheck", default=False)
        self.mcp_mode = self._resolve_choice("mcp_mode", ("auto", "config-only", "skip"), default="auto")
        self.health_url = str(args.health_url or DEFAULT_HEALTH_URL).strip() or DEFAULT_HEALTH_URL
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
        values: Dict[str, str] = {
            "NUWAX_ACTIVE_ROBOT": self.selected_robot,
            "NUWAX_PYTHON_BIN": self.python_bin,
            "NUWAX_GEOMETRY_ENABLED": _bool_text(go2_geometry_enabled),
            # 去 ROS2 化（2026-07-03 drop-ros2）：所有 ROS2 env 已删除；geometry runtime
            # 由 ``nuwax-geometry-runtime`` 提供，平台侧不再 source ROS2 / 配置 RMW /
            # 设置 ROS topic 默认值；运行时 I/O 全部走 LCM typed channel 与 native shm。
            "NUWAX_GEOMETRY_RUNTIME_BINARY": str(PROJECT_ROOT / "services" / "geometry" / "nuwax_geometry_runtime" / "target" / "release" / "nuwax-geometry-runtime"),
            "NUWAX_GEOMETRY_RUNTIME_PORT": "50051",
            "NUWAX_GEOMETRY_RUNTIME_READY_FILE": str(PROJECT_ROOT / "runtime_data" / "geometry" / "nuwax_geometry_runtime.ready"),
            "NUWAX_LCM_URL": "udpm://239.255.76.67:7667?ttl=0",
            "NUWAX_LCM_BACKEND": "real-lcm",
            "NUWAX_LCM_CHANNEL_ODOM": "/odom",
            "NUWAX_LCM_CHANNEL_TF": "/tf",
            "NUWAX_LCM_CHANNEL_POINTCLOUD": "/go2/lidar/points",
            "NUWAX_LCM_CHANNEL_OCCUPANCY": "/map/occupancy",
            "NUWAX_LCM_CHANNEL_CMD_VEL": "/cmd_vel",
            # Phase 1 (2026-07-07):nuwax-workspace 已并入本仓 services/nuwax-workspace,
            # 不再写入绝对路径,默认走 build_default_nuwax_workspace_dir() 计算 repo-local。
            "NUWAX_NUWAX_WORKSPACE_DIR": str(PROJECT_ROOT / "services" / "nuwax-workspace"),
            "NUWAX_LOG_DIR": str(PROJECT_ROOT),
            "NUWAX_LOG_FILE": str(PROJECT_ROOT / "nuwax_robot_bridge.log"),
            "NUWAX_GATEWAY_ARTIFACT_DIR": str(PROJECT_ROOT / "artifacts"),
            "NUWAX_MAP_CATALOG_ROOT": str(PROJECT_ROOT / "runtime_data" / "maps"),
            "NUWAX_RTABMAP_DATABASE_ROOT": str(PROJECT_ROOT / "runtime_data" / "geometry" / "rtabmap"),
            "NUWAX_MEMORY_DB_PATH": str(PROJECT_ROOT / "runtime_data" / "memory" / "vector_memory.db"),
            "NUWAX_RELAY_ENABLED": _bool_text(self.relay_enabled),
        }

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

        # 模型路径后缀即推理链路：导出引擎时指向 .engine(TensorRT)，否则 .pt(PyTorch 开放词汇)。
        yolo_world_model = DEFAULT_YOLO_WORLD_ENGINE if self.export_yolo_engine else DEFAULT_YOLO_WORLD_MODEL
        if self.perception_mode == "local":
            values.update(
                {
                    "NUWAX_PERCEPTION_DEFAULT_DETECTOR_BACKEND": "yolo_world",
                    "NUWAX_PERCEPTION_DEFAULT_SCENE_BACKEND": "simple_scene",
                    "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED": "true",
                    "NUWAX_PERCEPTION_OPENAI_ENABLED": "false",
                    "NUWAX_PERCEPTION_STREAM_ENABLED": "true",
                    "NUWAX_PERCEPTION_STREAM_AUTO_START": "false",
                    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL": yolo_world_model,
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
                    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL": yolo_world_model,
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
                    "NUWAX_PERCEPTION_YOLO_WORLD_MODEL": yolo_world_model,
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

        # R-N11 守门:合并 .env 已存在键(用户已设值),跳过空值 / null / 超长 / 注释行
        # 拒绝把空 / 纯空白 / 包含 NUL / > 4096 字符的值注入子进程环境。
        for key, value in self.root_existing.items():
            if key.startswith("#"):
                continue
            if not value or value.isspace() or "\0" in value or len(value) > 4096:
                continue  # R-N11 守门:跳过空值 / null / 超长
            if key not in values:
                values[key] = value

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
        # 去 ROS2 化（2026-07-03 drop-ros2）：原本 6+ 项 ROS2 / colcon / rosdep / CycloneDDS /
        # Go2SLAM 工作区校验整体删除。geometry runtime 由 ``nuwax-geometry-runtime`` 提供，
        # 平台侧不再要求安装 ROS2 / colcon / rosdep / libddsc.so。
        results.append((True, _check_result("ROS2 依赖", True, "已下线；geometry runtime 改由 nuwax-geometry-runtime 承载（LCM typed channel）")))
        runtime_binary = PROJECT_ROOT / "services" / "geometry" / "nuwax_geometry_runtime" / "target" / "release" / "nuwax-geometry-runtime"
        binary_ok = runtime_binary.exists() or bool(planner.install_geometry_deps)
        binary_detail = str(runtime_binary) if runtime_binary.exists() else "将由 nuwax-workspace cargo build 产出"
        results.append((binary_ok, _check_result("nuwax-geometry-runtime 二进制", binary_ok, binary_detail)))

    if planner.selected_robot == "go2" and planner.data_plane_enabled:
        go2_template_ok = planner.robot_meta.env_template is not None and planner.robot_meta.env_template.exists()
        results.append((go2_template_ok, _check_result("Go2 数据面模板", go2_template_ok, str(planner.robot_meta.env_template) if planner.robot_meta.env_template else "无")))

    if planner.memory_mode == "local":
        results.append((_file_exists(PROJECT_ROOT / "requirements-memory-models.txt"), _check_result("记忆模型依赖清单", _file_exists(PROJECT_ROOT / "requirements-memory-models.txt"), str(PROJECT_ROOT / "requirements-memory-models.txt"))))

    if planner.mcp_mode != "skip":
        results.append((_file_exists(DEFAULT_EXTERNAL_MCP_TEMPLATE), _check_result("外部 MCP 模板", _file_exists(DEFAULT_EXTERNAL_MCP_TEMPLATE), str(DEFAULT_EXTERNAL_MCP_TEMPLATE))))
        results.append((_file_exists(DEFAULT_PROXY_MCP_TEMPLATE), _check_result("SQLite MCP 模板", _file_exists(DEFAULT_PROXY_MCP_TEMPLATE), str(DEFAULT_PROXY_MCP_TEMPLATE))))

    if planner.run_healthcheck:
        results.append((True, _check_result("健康检查", True, "将匿名调用 /api/health（网关不鉴权）")))

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


# 去 ROS2 化（2026-07-03 drop-ros2）：以下函数已删除并替换为 no-op 占位
# - install_geometry_system_dependencies: 原负责 apt 安装 ROS2 + 派生包 / rosdep init
#   / rosdep install / Go2SLAM 工作区校验；ROS2 工作空间已物理删除，故无对象可装。
# - assert_go2slam_workspace_ready / assert_go2slam_runtime_ready: Go2SLAM 工作区已
#   物理删除，对应 colcon build / install 校验流程整体下线。
# - build_workspaces: 原负责 ROS2 工作空间脚本 / colcon build；删除后无
#   任何 ROS2 工作空间可构建。
# 替换说明：geometry runtime 由 ``nuwax-geometry-runtime`` Rust 二进制承载，
# 平台侧 ``phases/system.py`` 不再调这些函数；保留同名 ``_no_op`` 仅为防止
# installer-redesign / 老脚本意外 import 时 NameError（执行方案 Phase 0+4）。


def install_geometry_system_dependencies(planner: BootstrapPlanner) -> None:  # noqa: ARG001
    """no-op 占位：原 ROS2 apt 派生 + rosdep + Go2SLAM 校验已下线。"""
    print("跳过 geometry 系统依赖安装（去 ROS2 化：geometry runtime 由 nuwax-geometry-runtime 承载）。")
    return


def build_workspaces(planner: BootstrapPlanner) -> None:  # noqa: ARG001
    """no-op 占位：原 ROS2 工作空间脚本 / colcon build 已下线。"""
    print("跳过 ROS2 工作空间构建（去 ROS2 化：geometry runtime 由 nuwax-geometry-runtime 承载）。")
    return


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


def render_mcp_payload() -> Dict[str, object]:
    return {
        "mcpServers": {
            "nuwax_robot_bridge": {
                "url": "http://host.docker.internal:8766/mcp",
            }
        }
    }


def configure_mcp(planner: BootstrapPlanner) -> None:
    if planner.mcp_mode == "skip":
        print("跳过 MCP 注入。")
        return

    payload = render_mcp_payload()
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
    # 去 ROS2 化（2026-07-03 drop-ros2）：Go2 geometry 不再要求 _detect_ros_setup /
    # assert_go2slam_runtime_ready；geometry runtime 由 ``nuwax-geometry-runtime``
    # 提供，依赖 go2.sh / start_gateway.sh 的 native 启动链。
    command = f"nohup bash {START_SCRIPT} > /tmp/nuwax_bootstrap_gateway.log 2>&1 &"
    _run_shell(command, cwd=PROJECT_ROOT)
    print("已在后台启动 gateway，日志位于 /tmp/nuwax_bootstrap_gateway.log")


def run_healthcheck(planner: BootstrapPlanner) -> None:
    if not planner.run_healthcheck:
        print("跳过健康检查。")
        return
    request = urllib.request.Request(planner.health_url)
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
    # 去 ROS2 化（2026-07-03 drop-ros2）：原 build_geometry / build_go2slam /
    # install_geometry_deps wizard 选项已下线；geometry runtime 由
    # ``nuwax-geometry-runtime`` 提供，无 ROS2 工作空间可构建；默认值置 False 以保留
    # argparse 接口兼容，避免旧 profile 加载崩溃。
    args.build_geometry = False
    args.build_go2slam = False
    args.install_geometry_deps = prompt_bool("安装 nuwax-geometry-runtime OS 依赖", bool(args.geometry))
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
        # 去 ROS2 化（2026-07-03 drop-ros2）：--build-geometry / --build-go2slam /
        # --install-geometry-deps / --repair-ros2-apt-source 已下线；保留为 no-op
        # argparse 接口以兼容老 profile / wizard；help 文本明确已下线。
        add_boolean_flag(
            subparser,
            "build_geometry",
            "[下线] 已删除 ROS2 geometry 工作空间构建",
        )
        add_boolean_flag(
            subparser,
            "build_go2slam",
            "[下线] 已删除 Go2SLAM 工作空间构建",
        )
        subparser.add_argument(
            "--build-go2-rtabnav2",
            action="store_true",
            help="[下线] 已删除（与 --build-go2slam 等价，ROS2 工作空间已物理删除）",
        )
        add_boolean_flag(
            subparser,
            "install_geometry_deps",
            "安装 nuwax-geometry-runtime OS 依赖",
        )
        add_boolean_flag(
            subparser,
            "repair_ros2_apt_source",
            "[下线] ROS2 apt 源修复已删除",
        )
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
