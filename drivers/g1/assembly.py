"""G1 机器人装配入口。

按 ralplan Round 3 收敛：start/stop 真实现，get_status 返回 initialized + control_mode="loco"。
- LocoController 接管运动（drivers/g1/control/loco_controller.py）
- ArmController 接管手臂（drivers/g1/control/arm_controller.py）
- ROS SLAM 栈（外部 G1WebRtcSlamScript）本期不实装，按用户授权"ROS 侧先不动"
- 视频/点云/IMU 走 videohub 复路径（与 Go2 范式相同），由 providers/ 抽象层对外
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Dict, Optional, TYPE_CHECKING

from adapters.base import AdapterConfig
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.g1.control.loco_controller import LocoController
from drivers.g1.control.arm_controller import ArmController
from drivers.g1.defaults import build_g1_defaults
from drivers.g1.manifest import build_g1_manifest
from drivers.g1.providers import G1ProviderBundle

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.assembly")

# 默认参数（与 drivers/g1/data_plane.env 一致；env 可覆盖）
DEFAULT_IFACE = "eth0"
DEFAULT_CLIENT_TIMEOUT_SEC = 10.0


@dataclass
class G1RobotAssembly(RobotAssemblyBase):
    """G1 入口装配（本期实现控制+手臂；ROS SLAM 待用户后续任务）。"""

    config: "NuwaxRobotBridgeConfig"
    iface: Optional[str] = None
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None
    # 可注入的控制器工厂（方便测试）
    loco_controller_factory: Optional[object] = None
    arm_controller_factory: Optional[object] = None

    def __post_init__(self) -> None:
        self.defaults = build_g1_defaults(self.config, self.iface)
        self.manifest = build_g1_manifest(self.config)
        self._initialize_adapter_runtime(self.adapter_configs)
        # 控制器实例（init 时创建）
        self.loco_controller: Optional[LocoController] = None
        self.arm_controller: Optional[ArmController] = None
        self.high_level_initialized: bool = False
        self.current_mode: str = "unknown"  # "loco" / "low" / "unknown"
        # 装配状态互斥锁
        self._lock = threading.RLock()
        # Provider bundle（按 Go2 范式：assembly 注入 bundle）
        self.providers = G1ProviderBundle(assembly=self)

    def _create_loco_controller(self) -> LocoController:
        if self.loco_controller_factory is not None:
            return self.loco_controller_factory()
        return LocoController(iface=self.iface, client_timeout_sec=DEFAULT_CLIENT_TIMEOUT_SEC)

    def _create_arm_controller(self) -> ArmController:
        if self.arm_controller_factory is not None:
            return self.arm_controller_factory()
        return ArmController(iface=self.iface, client_timeout_sec=DEFAULT_CLIENT_TIMEOUT_SEC)

    # ------------------ 生命周期 ------------------

    def start(self) -> None:
        """启动 G1 装配：初始化 LocoController + ArmController + adapters。"""
        with self._lock:
            if self.high_level_initialized:
                LOGGER.info("G1 装配已启动，跳过。")
                return
            LOGGER.info("启动 G1 装配（iface=%s）...", self.iface or "default")
            # 1. LocoController
            try:
                self.loco_controller = self._create_loco_controller()
                if not self.loco_controller.init():
                    raise RuntimeError("LocoController.init() 返 False")
            except Exception as exc:
                LOGGER.error("LocoController 启动失败: %s", exc)
                self.loco_controller = None
                self.high_level_initialized = False
                self.current_mode = "unknown"
                raise
            # 2. ArmController（失败仅警告，不阻塞运动）
            try:
                self.arm_controller = self._create_arm_controller()
                if not self.arm_controller.init():
                    LOGGER.warning("ArmController.init() 返 False（仅警告，运动仍可用）")
                    self.arm_controller = None
            except Exception as exc:
                LOGGER.warning("ArmController 启动失败: %s（仅警告）", exc)
                self.arm_controller = None
            # 3. adapters（Go2 范式：先注册，再统一初始化）
            try:
                self.initialize_registered_adapters()
            except Exception as exc:
                LOGGER.warning("adapters 初始化失败: %s（仅警告）", exc)
            self.high_level_initialized = True
            self.current_mode = "loco"
            LOGGER.info("G1 装配启动完成（control_mode=%s）", self.current_mode)

    def stop(self) -> None:
        """优雅停止 G1 装配。"""
        with self._lock:
            if not self.high_level_initialized:
                LOGGER.info("G1 装配未启动，跳过停止。")
                return
            LOGGER.info("停止 G1 装配...")
            try:
                self.stop_registered_adapters()
            except Exception as exc:
                LOGGER.warning("adapters 停止失败: %s", exc)
            if self.arm_controller is not None:
                try:
                    self.arm_controller.stop()
                except Exception as exc:
                    LOGGER.warning("ArmController 停止失败: %s", exc)
                self.arm_controller = None
            if self.loco_controller is not None:
                try:
                    self.loco_controller.stop()
                except Exception as exc:
                    LOGGER.warning("LocoController 停止失败: %s", exc)
                self.loco_controller = None
            self.high_level_initialized = False
            self.current_mode = "unknown"
            LOGGER.info("G1 装配停止完成。")

    def get_status(self) -> RobotAssemblyStatus:
        adapter_statuses = self.get_adapter_health_statuses()
        return RobotAssemblyStatus(
            robot_name=self.manifest.robot_name,
            initialized=self.high_level_initialized,
            control_mode=self.current_mode,
            low_level_ready=False,  # G1 本期不实装关节级 LowCmd 控制
            low_level_running=False,
            adapter_count=len(adapter_statuses),
            healthy_adapter_count=sum(1 for status in adapter_statuses if status.is_healthy),
        )


def create_g1_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> G1RobotAssembly:
    """创建 G1 入口装配。"""
    return G1RobotAssembly(config=config, iface=iface, adapter_configs=adapter_configs)
