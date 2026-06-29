from __future__ import annotations

import logging
import os
import subprocess
from dataclasses import dataclass
from typing import Any, Dict, Optional, TYPE_CHECKING, Tuple

import httpx

from adapters.base import AdapterConfig
from contracts.geometry import Twist, Vector3
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.puppypi.manifest import build_puppypi_defaults, build_puppypi_manifest
from drivers.puppypi.providers import PuppyPiProviderBundle
from drivers.puppypi.capabilities import PUPPYPI_ACTION_GROUP_BY_NAME

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


LOGGER = logging.getLogger("nuwax_robot_bridge.puppypi.assembly")


@dataclass(frozen=True)
class ProxyHealth:
    ok: bool
    error: Optional[str] = None


class MotionProxyClient:
    """同步 HTTP 客户端：转发 twist 到容器 motion_proxy。"""

    def __init__(self, base_url: str, *, timeout_sec: float = 2.0) -> None:
        self.base_url = str(base_url or "").rstrip("/")
        self.timeout_sec = max(0.1, float(timeout_sec))
        self._client = httpx.Client(timeout=self.timeout_sec)
        self.last_health_ok = False

    def health_check(self) -> ProxyHealth:
        if not self.base_url:
            self.last_health_ok = False
            return ProxyHealth(ok=False, error="PUPPYPI_PROXY_URL 未配置")
        try:
            response = self._client.get(f"{self.base_url}/health")
            response.raise_for_status()
            payload = response.json()
            self.last_health_ok = bool(payload.get("ok"))
            return ProxyHealth(ok=self.last_health_ok)
        except Exception as exc:
            self.last_health_ok = False
            return ProxyHealth(ok=False, error=str(exc))

    def post_twist(self, *, x_mps: float, yaw_rate: float) -> None:
        if not self.base_url:
            raise RuntimeError("PUPPYPI_PROXY_URL 未配置，无法下发运动指令。")
        response = self._client.post(
            f"{self.base_url}/twist",
            json={"x_mps": float(x_mps), "yaw_rate": float(yaw_rate)},
        )
        response.raise_for_status()

    def close(self) -> None:
        self._client.close()


@dataclass
class PuppyPiAssembly(RobotAssemblyBase):
    """幻尔 PuppyPi 薄装配：仅维护 motion_proxy HTTP 客户端。"""

    config: "NuwaxRobotBridgeConfig"
    iface: Optional[str] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None

    def __post_init__(self) -> None:
        self.defaults = build_puppypi_defaults(self.config)
        self.manifest = build_puppypi_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        proxy_url = os.environ.get("PUPPYPI_PROXY_URL", "http://127.0.0.1:8090").strip()
        timeout_sec = float(os.environ.get("PUPPYPI_PROXY_TIMEOUT_SEC", "2.0") or "2.0")
        self.motion_proxy_client = MotionProxyClient(proxy_url, timeout_sec=timeout_sec)
        self.providers = PuppyPiProviderBundle(self)
        self._last_x_mps = 0.0
        self._last_yaw_rate = 0.0

    def record_twist(self, *, x_mps: float, yaw_rate: float) -> None:
        self._last_x_mps = float(x_mps)
        self._last_yaw_rate = float(yaw_rate)

    @property
    def last_twist(self) -> Twist:
        return Twist(
            frame_id=self.defaults.frame_ids.get("base", "world/puppypi/base"),
            linear=Vector3(x=self._last_x_mps, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self._last_yaw_rate),
        )

    def last_twist_metadata(self) -> Dict[str, float]:
        return {"x_mps": self._last_x_mps, "yaw_rate": self._last_yaw_rate}

    def start(self) -> None:
        health = self.motion_proxy_client.health_check()
        if not health.ok:
            LOGGER.warning("motion_proxy 不可达: %s", health.error or "unknown")

    def stop(self) -> None:
        try:
            self.motion_proxy_client.post_twist(x_mps=0.0, yaw_rate=0.0)
        except Exception:
            LOGGER.debug("停止时下发零速度失败。", exc_info=True)
        self.motion_proxy_client.close()

    def execute_action(self, action: str, params: Optional[Dict[str, Any]] = None) -> Tuple[Optional[int], Any]:
        """执行 PuppyPi ROS1 服务或动作组。"""

        params = params or {}
        action_name = str(action or "").strip()
        if not action_name:
            return None, "动作名称不能为空。"

        if action_name == "go_home":
            remote_command = self._build_ros_service_command("/puppy_control/go_home")
            return self._run_remote_ros_command(action_name, remote_command, metadata={"service": "go_home"})

        if action_name in {"set_running", "set_self_balancing", "set_mark_time"}:
            enabled = bool(params.get("enabled", params.get("data", True)))
            remote_command = self._build_ros_service_command(
                f"/puppy_control/{action_name}",
                f'"data: {"true" if enabled else "false"}"',
            )
            return self._run_remote_ros_command(
                action_name,
                remote_command,
                metadata={"service": action_name, "enabled": enabled},
            )

        if action_name == "run_action_group":
            action_group = str(params.get("name", "")).strip()
            if not action_group:
                return None, "run_action_group 需要参数 name。"
            wait = bool(params.get("wait", False))
            remote_command = self._build_run_action_group_command(action_group, wait=wait)
            return self._run_remote_ros_command(
                action_name,
                remote_command,
                metadata={"action_group": action_group, "wait": wait},
            )

        action_group = PUPPYPI_ACTION_GROUP_BY_NAME.get(action_name)
        if not action_group:
            return None, f"不支持的动作: {action_name}"

        wait = bool(params.get("wait", False))
        remote_command = self._build_run_action_group_command(action_group, wait=wait)
        return self._run_remote_ros_command(
            action_name,
            remote_command,
            metadata={"action_group": action_group, "wait": wait},
        )

    def _build_ros_setup_command(self) -> str:
        return os.environ.get(
            "PUPPYPI_ROS1_SETUP",
            "source /opt/ros/noetic/setup.bash && source /home/ubuntu/puppypi/devel/setup.bash",
        ).strip()

    def _build_ros_service_command(self, service_name: str, args: str = "") -> str:
        if args:
            return f"rosservice call {service_name} {args}"
        return f"rosservice call {service_name}"

    def _build_run_action_group_command(self, action_group: str, *, wait: bool) -> str:
        return self._build_ros_service_command(
            "/puppy_control/runActionGroup",
            f'"{{name: {action_group}, wait: {"true" if wait else "false"}}}"',
        )

    def _run_remote_ros_command(
        self,
        action_name: str,
        service_command: str,
        *,
        metadata: Dict[str, Any],
    ) -> Tuple[Optional[int], Any]:
        container = os.environ.get("PUPPYPI_ROS1_CONTAINER", "puppypi").strip() or "puppypi"
        remote_command = f"{self._build_ros_setup_command()}; {service_command}"
        command = ["docker", "exec", container, "bash", "-lc", remote_command]
        timeout_sec = float(os.environ.get("PUPPYPI_ACTION_TIMEOUT_SEC", "30") or "30")

        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=max(5.0, timeout_sec),
                check=False,
            )
        except subprocess.TimeoutExpired:
            return None, f"动作 {action_name} 执行超时。"
        except Exception as exc:
            LOGGER.exception("PuppyPi 动作执行失败 action=%s", action_name)
            return None, str(exc)

        if completed.returncode != 0:
            error_text = (completed.stderr or completed.stdout or "").strip()
            return None, error_text or f"动作 {action_name} 执行失败（exit {completed.returncode}）。"

        output = (completed.stdout or "").strip()
        LOGGER.info("PuppyPi 动作执行成功 action=%s metadata=%s", action_name, metadata)
        return 0, {
            "action": action_name,
            "output": output,
            **metadata,
        }

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=True,
            control_mode="high" if self.motion_proxy_client.last_health_ok else "unknown",
            low_level_ready=False,
            low_level_running=False,
            adapter_count=len(adapter_statuses),
            healthy_adapter_count=sum(1 for status in adapter_statuses if status.is_healthy),
        )


def create_puppypi_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> PuppyPiAssembly:
    del iface
    return PuppyPiAssembly(config=config, adapter_configs=adapter_configs)
