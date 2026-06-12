from __future__ import annotations

import base64
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Tuple, Any, Callable, TYPE_CHECKING

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
    """Go2 运行时工厂集合。"""

    channel_factory_initialize: Callable[[int, Optional[str]], None]
    sport_client_factory: Callable[[], Any]
    video_client_factory: Callable[[], Any]
    vui_client_factory: Callable[[], Any]
    low_level_controller_factory: Callable[[Optional[str]], Any]


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
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from unitree_sdk2py.go2.vui.vui_client import VuiClient

    from drivers.go2.control.low_level_controller import LowLevelController

    def _initialize_channel(domain_id: int, iface: Optional[str]) -> None:
        if iface:
            ChannelFactoryInitialize(domain_id, iface)
        else:
            ChannelFactoryInitialize(domain_id)

    return Go2ClientFactories(
        channel_factory_initialize=_initialize_channel,
        sport_client_factory=SportClient,
        video_client_factory=VideoClient,
        vui_client_factory=VuiClient,
        low_level_controller_factory=LowLevelController,
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
        self.sport_client = None
        self.video_client = None
        self.vui_client = None
        self.low_level_controller = None
        self.high_level_initialized = False
        self.current_mode = "high"
        self._last_motion_stop_routes: List[str] = []
        self._last_motion_stop_errors: List[str] = []
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
        self.switch_mode("high")

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        low_level_running = bool(
            self.low_level_controller is not None and getattr(self.low_level_controller, "is_running", False)
        )
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=self.high_level_initialized,
            control_mode=self.current_mode,
            low_level_ready=self.low_level_controller is not None,
            low_level_running=low_level_running,
            adapter_count=len(adapter_statuses),
            healthy_adapter_count=sum(1 for status in adapter_statuses if status.is_healthy),
        )

    def ensure_high_level_clients(self) -> None:
        """初始化高层客户端。"""

        if self.high_level_initialized:
            return

        assert self.factories is not None
        self.factories.channel_factory_initialize(0, self.iface or self.config.dds.iface)

        self.sport_client = self.factories.sport_client_factory()
        self.sport_client.SetTimeout(GO2_TIMEOUTS.sport_timeout_sec)
        self.sport_client.Init()

        self.video_client = self.factories.video_client_factory()
        self.video_client.SetTimeout(GO2_TIMEOUTS.video_timeout_sec)
        self.video_client.Init()

        self.vui_client = self.factories.vui_client_factory()
        self.vui_client.SetTimeout(GO2_TIMEOUTS.vui_timeout_sec)
        self.vui_client.Init()

        time.sleep(GO2_TIMEOUTS.post_init_stabilize_sec)
        self.high_level_initialized = True

    def ensure_low_level_controller(self) -> None:
        """初始化低层控制器。"""

        assert self.factories is not None
        if self.low_level_controller is not None:
            return

        controller = self.factories.low_level_controller_factory(self.iface or self.config.dds.iface)
        if not controller.init():
            raise RuntimeError("低级别控制器初始化失败")
        controller.start()
        self.low_level_controller = controller

    def switch_mode(self, mode: str) -> Tuple[int, Dict[str, str]]:
        """切换控制模式。"""

        if mode == "low":
            self.ensure_low_level_controller()
            self.current_mode = "low"
            return 0, {"mode": "low"}

        if self.low_level_controller is not None:
            self.low_level_controller.stop()
            self.low_level_controller = None
        self.current_mode = "high"
        return 0, {"mode": "high"}

    def stop_all_motion(self, reason: Optional[str] = None) -> None:
        """执行统一停止。"""

        del reason
        stopped_routes: List[str] = []
        errors: List[str] = []

        data_plane_stop = getattr(self.data_plane, "stop_motion_command", None)
        if callable(data_plane_stop):
            try:
                data_plane_stop()
                stopped_routes.append("data_plane_motion")
            except Exception as exc:
                errors.append(f"data_plane_motion: {exc}")

        if self.low_level_controller is not None:
            try:
                stop_motion = getattr(self.low_level_controller, "stop_motion", None)
                if callable(stop_motion):
                    stop_motion()
                else:
                    self.low_level_controller.stop()
                self.low_level_controller = None
                self.current_mode = "high"
                stopped_routes.append("lowcmd")
            except Exception as exc:
                errors.append(f"lowcmd: {exc}")

        if self.sport_client is not None:
            try:
                code = self.sport_client.StopMove()
                if code not in (0, None):
                    errors.append(f"sport: code={code}")
                else:
                    stopped_routes.append("sport")
            except Exception as exc:
                errors.append(f"sport: {exc}")

        self._last_motion_stop_routes = stopped_routes
        self._last_motion_stop_errors = errors
        if errors:
            raise RuntimeError("Go2 统一停止部分通道失败: " + "; ".join(errors))

    def _require_high_level(self) -> None:
        self.ensure_high_level_clients()
        if self.sport_client is None or self.video_client is None or self.vui_client is None:
            raise RuntimeError("Go2 高层客户端未初始化")

    def _require_high_level_motion(self) -> None:
        if self.current_mode == "low":
            raise RuntimeError("Go2 当前处于低层控制模式，不能下发高层速度命令。")
        self._require_high_level()

    def _require_low_level(self) -> None:
        if self.current_mode != "low" or self.low_level_controller is None:
            raise RuntimeError("请先切换到低级别模式: switch_mode")

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

    def move(self, vx: float, vy: float, vyaw: float) -> int:
        self._require_high_level_motion()
        return self.sport_client.Move(vx, vy, vyaw)

    def move_for(self, vx: float, vy: float, vyaw: float, duration: float) -> int:
        self._require_high_level_motion()
        start_time = time.time()
        while time.time() - start_time < duration:
            code = self.sport_client.Move(vx, vy, vyaw)
            if code != 0:
                return code
            time.sleep(self.defaults.parameters["move_for_interval_sec"])
        self.sport_client.StopMove()
        return 0

    def stop_move(self) -> int:
        self._require_high_level()
        return self.sport_client.StopMove()

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

        no_param_actions = {
            "damp": "Damp",
            "balance_stand": "BalanceStand",
            "stop_move": "StopMove",
            "stand_up": "StandUp",
            "stand_down": "StandDown",
            "recovery_stand": "RecoveryStand",
            "sit": "Sit",
            "rise_sit": "RiseSit",
            "hello": "Hello",
            "stretch": "Stretch",
            "dance1": "Dance1",
            "dance2": "Dance2",
            "front_flip": "FrontFlip",
            "front_jump": "FrontJump",
            "front_pounce": "FrontPounce",
            "left_flip": "LeftFlip",
            "back_flip": "BackFlip",
            "free_walk": "FreeWalk",
        }

        bool_param_actions = {
            "hand_stand": ("HandStand", "flag"),
            "free_jump": ("FreeJump", "flag"),
            "free_bound": ("FreeBound", "flag"),
            "walk_upright": ("WalkUpright", "flag"),
            "cross_step": ("CrossStep", "flag"),
            "switch_joystick": ("SwitchJoystick", "on"),
            "pose": ("Pose", "flag"),
            "classic_walk": ("ClassicWalk", "flag"),
        }

        if action in no_param_actions:
            self._require_high_level()
            method = getattr(self.sport_client, no_param_actions[action])
            return method(), None

        if action in bool_param_actions:
            self._require_high_level()
            method_name, param_name = bool_param_actions[action]
            flag_value = params.get(param_name, True)
            if isinstance(flag_value, str):
                flag_value = flag_value.lower() in ("true", "1", "yes")
            method = getattr(self.sport_client, method_name)
            return method(bool(flag_value)), None

        if action == "move":
            return self.move(
                float(params.get("vx", 0)),
                float(params.get("vy", 0)),
                float(params.get("vyaw", 0)),
            ), None

        if action == "move_for":
            return self.move_for(
                float(params.get("vx", 0)),
                float(params.get("vy", 0)),
                float(params.get("vyaw", 0)),
                float(params.get("duration", 1.0)),
            ), None

        if action == "euler":
            self._require_high_level()
            return self.sport_client.Euler(
                float(params.get("roll", 0)),
                float(params.get("pitch", 0)),
                float(params.get("yaw", 0)),
            ), None

        if action == "speed_level":
            self._require_high_level()
            return self.sport_client.SpeedLevel(int(params.get("level", 1))), None

        if action == "switch_mode":
            return self.switch_mode(params.get("mode", "high"))

        if action == "joint_position":
            self._require_low_level()
            joint = params.get("joint")
            position = params.get("position")
            if not joint or position is None:
                return None, "缺少必要参数: joint, position"
            success = self.low_level_controller.set_joint_position(
                joint,
                float(position),
                params.get("kp", self.config.low_level.default_kp),
                params.get("kd", self.config.low_level.default_kd),
            )
            return (0 if success else -1), None

        if action == "joints_position":
            self._require_low_level()
            joints = params.get("joints", [])
            positions = params.get("positions", [])
            if not joints or not positions or len(joints) != len(positions):
                return None, "参数错误: joints和positions长度需一致"
            joint_positions = {joint: float(position) for joint, position in zip(joints, positions)}
            success = self.low_level_controller.set_joints_position(
                joint_positions,
                params.get("kp", self.config.low_level.default_kp),
                params.get("kd", self.config.low_level.default_kd),
            )
            return (0 if success else -1), None

        if action == "joint_velocity":
            self._require_low_level()
            joint = params.get("joint")
            velocity = params.get("velocity")
            if not joint or velocity is None:
                return None, "缺少必要参数: joint, velocity"
            success = self.low_level_controller.set_joint_velocity(
                joint,
                float(velocity),
                params.get("kd", self.config.low_level.default_kd),
            )
            return (0 if success else -1), None

        if action == "joint_torque":
            self._require_low_level()
            joint = params.get("joint")
            torque = params.get("torque")
            if not joint or torque is None:
                return None, "缺少必要参数: joint, torque"
            success = self.low_level_controller.set_joint_torque(joint, float(torque))
            return (0 if success else -1), None

        if action == "get_joint_state":
            self._require_low_level()
            return 0, self.low_level_controller.get_joint_state(params.get("joint"))

        if action == "get_imu_state":
            self._require_low_level()
            return 0, self.low_level_controller.get_imu_state()

        if action == "pose_preset":
            return None, "pose_preset 当前未绑定 Go2 实机关节标定，已在能力矩阵中标记为不可用。"

        if action in GO2_CAPABILITY_DESCRIPTORS_BY_NAME:
            return None, f"动作 {action} 已声明，但当前装配入口尚未实现。"

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
