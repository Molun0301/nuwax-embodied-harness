from __future__ import annotations

import base64
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Dict, Optional, Tuple, Any, Callable, TYPE_CHECKING

from adapters.base import AdapterBase, AdapterConfig
from drivers.common import ManagedRobotDeviceDataPlane, RobotAssemblyBase, RobotAssemblyStatus
from drivers.go2.capabilities import GO2_CAPABILITY_DESCRIPTORS_BY_NAME
from drivers.go2.data_plane import Go2DataPlaneRuntime, Go2DeviceDataPlane
from drivers.go2.defaults import GO2_TIMEOUTS, build_go2_defaults
from drivers.go2.manifest import build_go2_manifest
from drivers.go2.providers import Go2ProviderBundle
from drivers.go2.settings import load_go2_data_plane_config

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig


@dataclass
class Go2ClientFactories:
    """Go2 传感与媒体客户端工厂集合。"""

    channel_factory_initialize: Callable[[int, Optional[str]], None]
    video_client_factory: Callable[[], Any]
    vui_client_factory: Callable[[], Any]


def _prepare_go2_sdk_path(config: "NuwaxRobotBridgeConfig") -> None:
    sdk_path = str(getattr(getattr(config, "dds", None), "sdk_path", "") or "").strip()
    if not sdk_path:
        return
    normalized = str(Path(sdk_path).expanduser())
    if normalized not in sys.path:
        sys.path.insert(0, normalized)


def _load_default_factories() -> Go2ClientFactories:
    """懒加载默认 Go2 工厂。"""

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from unitree_sdk2py.go2.vui.vui_client import VuiClient

    def _initialize_channel(domain_id: int, iface: Optional[str]) -> None:
        if iface:
            ChannelFactoryInitialize(domain_id, iface)
        else:
            ChannelFactoryInitialize(domain_id)

    return Go2ClientFactories(
        channel_factory_initialize=_initialize_channel,
        video_client_factory=VideoClient,
        vui_client_factory=VuiClient,
    )


@dataclass
class Go2RobotAssembly(RobotAssemblyBase):
    """Go2 装配入口。"""

    config: "NuwaxRobotBridgeConfig"
    iface: Optional[str] = None
    factories: Optional[Go2ClientFactories] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None
    prebound_adapters: Tuple[AdapterBase[Any, Any], ...] = ()
    data_plane: Optional[ManagedRobotDeviceDataPlane] = None
    data_plane_runtime: Optional[Go2DataPlaneRuntime] = None

    def __post_init__(self) -> None:
        _prepare_go2_sdk_path(self.config)
        self.factories = self.factories or _load_default_factories()
        self.defaults = build_go2_defaults(self.config, self.iface)
        self.manifest = build_go2_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        runtime = self.data_plane_runtime
        if runtime is None and isinstance(self.data_plane, Go2DataPlaneRuntime):
            runtime = self.data_plane
        if runtime is None:
            runtime_candidate = getattr(self.data_plane, "runtime", None)
            if isinstance(runtime_candidate, Go2DataPlaneRuntime):
                runtime = runtime_candidate
        if runtime is None and self.data_plane is None:
            runtime = Go2DataPlaneRuntime(load_go2_data_plane_config(platform_geometry=self.config.geometry))
        self.data_plane_runtime = runtime
        if self.data_plane is None and runtime is not None:
            self.data_plane = Go2DeviceDataPlane(runtime)
        elif isinstance(self.data_plane, Go2DataPlaneRuntime):
            self.data_plane = Go2DeviceDataPlane(self.data_plane)
        self.providers = Go2ProviderBundle(self)
        self.video_client = None
        self.vui_client = None
        self.high_level_initialized = False
        self.current_mode = "high"
        for adapter in self.prebound_adapters:
            self.bind_adapter(adapter)

    def start(self) -> None:
        self.ensure_high_level_clients()
        if self.data_plane is not None:
            self.data_plane.start()
        self.initialize_registered_adapters()

    def stop(self) -> None:
        self.stop_registered_adapters()
        if self.data_plane is not None:
            self.data_plane.stop()

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=self.high_level_initialized,
            control_mode=self.current_mode,
            low_level_ready=False,
            low_level_running=False,
            adapter_count=len(adapter_statuses),
            healthy_adapter_count=sum(1 for status in adapter_statuses if status.is_healthy),
        )

    def ensure_high_level_clients(self) -> None:
        """初始化高层客户端。"""

        if self.high_level_initialized:
            return

        assert self.factories is not None
        self.factories.channel_factory_initialize(0, self.iface or self.config.dds.iface)

        self.video_client = self.factories.video_client_factory()
        self.video_client.SetTimeout(GO2_TIMEOUTS.video_timeout_sec)
        self.video_client.Init()

        self.vui_client = self.factories.vui_client_factory()
        self.vui_client.SetTimeout(GO2_TIMEOUTS.vui_timeout_sec)
        self.vui_client.Init()

        time.sleep(GO2_TIMEOUTS.post_init_stabilize_sec)
        self.high_level_initialized = True

    def _require_high_level(self) -> None:
        self.ensure_high_level_clients()
        if self.video_client is None or self.vui_client is None:
            raise RuntimeError("Go2 图像/VUI 客户端未初始化")

    def capture_image_bytes(self) -> bytes:
        """抓取当前图像字节。"""

        self._require_high_level()
        code, data = self.video_client.GetImageSample()
        if code != 0:
            raise RuntimeError(f"获取图像失败: code={code}")
        return bytes(data)

    def capture_image_payload(self) -> Tuple[int, Dict[str, Any]]:
        image_bytes = self.capture_image_bytes()
        return 0, {
            "image_base64": base64.b64encode(image_bytes).decode("utf-8"),
            "size": len(image_bytes),
        }

    def save_image(self, filename: str) -> Tuple[int, Dict[str, Any]]:
        image_bytes = self.capture_image_bytes()
        output_path = Path(filename)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(image_bytes)
        return 0, {"filename": str(output_path), "size": len(image_bytes)}

    def get_vui_volume_info(self) -> Dict[str, Any]:
        self._require_high_level()
        code, level = self.vui_client.GetVolume()
        if code != 0 or level is None:
            raise RuntimeError(f"读取 Go2 VUI 音量失败: code={code}")

        switch_code, switch_enabled = self.vui_client.GetSwitch()
        result = {
            "backend": "vui",
            "level": int(level),
            "volume": self._vui_level_to_ratio(int(level)),
        }
        if switch_code == 0 and switch_enabled is not None:
            result["switch_enabled"] = bool(switch_enabled)
        return result

    def set_vui_volume_ratio(self, volume: float, auto_enable_switch: bool) -> Dict[str, Any]:
        self._require_high_level()
        target_level = self._ratio_to_vui_level(volume)
        if auto_enable_switch:
            switch_code = self.vui_client.SetSwitch(1)
            if switch_code != 0:
                raise RuntimeError(f"启用 Go2 语音开关失败: code={switch_code}")

        code = self.vui_client.SetVolume(target_level)
        if code != 0:
            raise RuntimeError(f"设置 Go2 VUI 音量失败: code={code}")

        info = self.get_vui_volume_info()
        info["set_level"] = target_level
        return info

    def _vui_level_to_ratio(self, level: int) -> float:
        span = max(1, self.config.volume.vui_max_level - self.config.volume.vui_min_level)
        normalized = (int(level) - self.config.volume.vui_min_level) / span
        return max(0.0, min(1.0, normalized))

    def _ratio_to_vui_level(self, volume: float) -> int:
        normalized = max(0.0, min(1.0, float(volume)))
        span = max(1, self.config.volume.vui_max_level - self.config.volume.vui_min_level)
        return int(round(self.config.volume.vui_min_level + normalized * span))

    def execute_action(self, action: str, params: Optional[Dict[str, Any]] = None) -> Tuple[Optional[int], Any]:
        """执行 Go2 入口负责的动作。"""

        params = params or {}

        if action == "get_image":
            return self.capture_image_payload()
        if action == "save_image":
            return self.save_image(params.get("filename", "/tmp/go2_image.jpg"))

        if action in GO2_CAPABILITY_DESCRIPTORS_BY_NAME:
            return None, (
                f"动作 {action} 不由 Go2 装配入口执行；"
                "运动、模式和关节变更必须通过独立 motion safety guard。"
            )

        return None, f"未知动作: {action}"


def create_go2_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    factories: Optional[Go2ClientFactories] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
    prebound_adapters: Tuple[AdapterBase[Any, Any], ...] = (),
    data_plane: Optional[ManagedRobotDeviceDataPlane] = None,
    data_plane_runtime: Optional[Go2DataPlaneRuntime] = None,
) -> Go2RobotAssembly:
    """创建 Go2 机器人装配入口。"""

    return Go2RobotAssembly(
        config=config,
        iface=iface,
        factories=factories,
        adapter_configs=adapter_configs,
        prebound_adapters=prebound_adapters,
        data_plane=data_plane,
        data_plane_runtime=data_plane_runtime,
    )
