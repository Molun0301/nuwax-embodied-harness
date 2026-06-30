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
import os
from typing import Any, Dict, Optional, Tuple, TYPE_CHECKING

from adapters.base import AdapterConfig
from drivers.common.assembly import RobotAssemblyBase, RobotAssemblyStatus
from drivers.g1.control.loco_controller import LocoController
from drivers.g1.control.arm_controller import ArmController
from drivers.g1.control.motion_switcher_controller import (
    MotionSwitcherController,
    SUPPORTED_MODES,
)
from drivers.g1.defaults import build_g1_defaults
from drivers.g1.manifest import build_g1_manifest
from drivers.g1.providers import G1ProviderBundle

if TYPE_CHECKING:
    from core.settings import NuwaxRobotBridgeConfig

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.assembly")

# 默认参数（与 drivers/g1/data_plane.env 一致；env 可覆盖）
DEFAULT_IFACE = "eth0"
DEFAULT_CLIENT_TIMEOUT_SEC = 10.0

# G1 私有能力 → LocoController 无参方法名映射（与 drivers/g1/private_capabilities.py
# 的 ROBOT_PRIVATE_CAPABILITY_SPECS 严格对齐；LocoController 上的同名方法是这层契约的
# 真值源）。9 个 one_shot Loco 动作均不接 params。
_G1_LOCO_NO_PARAM_ACTIONS: Dict[str, str] = {
    "g1_damp": "damp",
    "g1_squat_to_stand": "squat_to_stand",
    "g1_stand_to_squat": "stand_to_squat",
    "g1_zero_torque": "zero_torque",
    "g1_lie_to_stand": "lie_to_stand",
    "g1_low_stand": "low_stand",
    "g1_high_stand": "high_stand",
    "g1_wave_hand": "wave_hand",
    "g1_shake_hand": "shake_hand",
}

# G1 私有能力集合（用于错误信息提示，不参与路由）。
_G1_KNOWN_ACTIONS: frozenset[str] = frozenset(
    list(_G1_LOCO_NO_PARAM_ACTIONS.keys())
    + [
        "g1_move", "g1_arm_action", "g1_play_audio", "g1_get_state",
        # MotionSwitcher 4 个（2026-06-30 实机新增：修高层动作 code=None 根因）
        "g1_check_motion_mode", "g1_select_motion_mode",
        "g1_release_motion_mode", "g1_acquire_high_level",
    ]
)


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

    def _resolved_dds_iface(self) -> Optional[str]:
        """Resolve the DDS interface G1 control clients should bind to.

        G1 deployments keep the device-side DDS interface in
        ``G1_DATA_PLANE_DDS_IFACE`` (see drivers/g1/data_plane.env). The
        platform-wide ``config.dds.iface`` is still accepted for explicit
        overrides and backwards compatibility.
        """

        candidates = (
            self.iface,
            os.environ.get("G1_DATA_PLANE_DDS_IFACE"),
            os.environ.get("G1_DDS_IFACE"),
            getattr(getattr(self.config, "dds", None), "iface", ""),
        )
        for candidate in candidates:
            value = str(candidate or "").strip()
            if value:
                return value
        return None

    def _create_loco_controller(self) -> LocoController:
        if self.loco_controller_factory is not None:
            return self.loco_controller_factory()
        return LocoController(iface=self._resolved_dds_iface(), client_timeout_sec=DEFAULT_CLIENT_TIMEOUT_SEC)

    def _create_arm_controller(self) -> ArmController:
        if self.arm_controller_factory is not None:
            return self.arm_controller_factory()
        return ArmController(iface=self._resolved_dds_iface(), client_timeout_sec=DEFAULT_CLIENT_TIMEOUT_SEC)

    # ------------------ 生命周期 ------------------

    def start(self) -> None:
        """启动 G1 装配：初始化 LocoController + ArmController + adapters。"""
        with self._lock:
            if self.high_level_initialized:
                LOGGER.info("G1 装配已启动，跳过。")
                return
            LOGGER.info("启动 G1 装配（iface=%s）...", self._resolved_dds_iface() or "default")
            # 1. LocoController（内部会触发 motion_switcher 自动抢占 advanced）
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

    # ------------------ 端侧 MCP 分发入口（private_capabilities 的 dispatch="execute_action" 落点） ------------------

    def execute_action(
        self,
        action: str,
        params: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Optional[int], Any]:
        """G1 入口动作分发（被 runtime._build_private_capability_handler 调用）。

        路由表（13 个 g1.* capability 与 drivers/g1/private_capabilities.py 一一对齐）：
        - LocoPrimitive（5，无参）：g1_damp / g1_squat_to_stand / g1_stand_to_squat /
          g1_zero_torque / g1_lie_to_stand → LocoController 同名方法
        - G1Specific（4，无参）：g1_low_stand / g1_high_stand / g1_wave_hand / g1_shake_hand →
          LocoController 同名方法
        - G1Specific（1，带参）：g1_arm_action → ArmController.execute_action(action_seq)
        - Generic：g1_move(vx, vy, vyaw) → LocoController.move；g1_get_state → provider 快照；
          g1_play_audio → 暂留占位（未接 AudioClient）

        Returns:
            (code, result)：code=0/SDK 返回码表示成功；-1 + 错误字符串表示失败。
            与 go2/assembly.py:331 范式严格一致，方便 runtime._build_private_capability_handler 透明透传。
        """

        params = params or {}

        with self._lock:
            if not self.high_level_initialized:
                return (-1, "G1 装配未启动，请先调用 start()。")

            # 1) Loco 一档（9 个无参 one_shot）
            method_name = _G1_LOCO_NO_PARAM_ACTIONS.get(action)
            if method_name is not None:
                loco = self.loco_controller
                if loco is None or not loco.is_running():
                    return (-1, f"LocoController 未初始化，无法执行 {action}。")
                try:
                    code = getattr(loco, method_name)()
                    return (int(code) if code is not None else 0, None)
                except Exception as exc:
                    LOGGER.error("G1 action=%s 抛错: %s", action, exc)
                    return (-1, f"{action} 执行失败: {exc}")

            # 2) 持续移动（streaming）
            if action == "g1_move":
                loco = self.loco_controller
                if loco is None or not loco.is_running():
                    return (-1, "LocoController 未初始化，无法执行 g1_move。")
                try:
                    code = loco.move(
                        float(params.get("vx", 0.0) or 0.0),
                        float(params.get("vy", 0.0) or 0.0),
                        float(params.get("vyaw", 0.0) or 0.0),
                    )
                    return (int(code) if code is not None else 0, None)
                except Exception as exc:
                    LOGGER.error("G1 g1_move 抛错: %s", exc)
                    return (-1, f"g1_move 执行失败: {exc}")

            # 3) 手臂动作（透传 action_seq）
            if action == "g1_arm_action":
                arm = self.arm_controller
                if arm is None or not arm.is_running():
                    return (-1, "ArmController 未初始化，无法执行 g1_arm_action。")
                action_seq = params.get("action_seq")
                if action_seq is None:
                    return (-1, "g1_arm_action 缺少必要参数: action_seq。")
                if not isinstance(action_seq, (list, tuple)):
                    return (-1, "g1_arm_action.action_seq 必须为 list/tuple 形态。")
                try:
                    code = arm.execute_action(action_seq)
                    return (int(code) if code is not None else 0, None)
                except Exception as exc:
                    LOGGER.error("G1 g1_arm_action 抛错: %s", exc)
                    return (-1, f"g1_arm_action 执行失败: {exc}")

            # 4) 状态快照（同步，只读）
            if action == "g1_get_state":
                loco = self.loco_controller
                ready = bool(loco is not None and loco.has_state_frame())
                snapshot = {
                    "ready": ready,
                    "reason": "" if ready else "no frame",
                    "timestamp": 0.0,
                    "control_mode": self.current_mode,
                    "initialized": bool(self.high_level_initialized),
                    "loco_running": bool(loco is not None and loco.is_running()),
                    "arm_running": bool(self.arm_controller is not None and self.arm_controller.is_running()),
                }
                return (0, snapshot)

            # 5) MotionSwitcher 4 个（2026-06-30 新增：修高层动作 code=None 真实根因）
            if action in ("g1_check_motion_mode", "g1_select_motion_mode",
                          "g1_release_motion_mode", "g1_acquire_high_level"):
                return self._dispatch_motion_switcher(action, params)

            # 6) 语音播放 — 暂留占位，待 AudioClient 接入
            if action == "g1_play_audio":
                wav_path = params.get("wav_path")
                if not wav_path or not isinstance(wav_path, str):
                    return (-1, "g1_play_audio 缺少必要参数: wav_path（绝对路径字符串）。")
                LOGGER.warning("g1_play_audio 本期未实装，wav_path=%s 已接收（待 AudioClient 接入）", wav_path)
                return (-1, "g1_play_audio 本期未实装，请接入 G1ProviderBundle.AudioTTSProvider。")

            # 6) 未识别 action
            supported = ", ".join(sorted(_G1_KNOWN_ACTIONS))
            return (-1, f"G1 未实现的 action: {action}。当前支持: {supported}")

    # ------------------ MotionSwitcher 分发（4 个 capability 共享） ------------------

    def _resolve_motion_switcher(self) -> MotionSwitcherController:
        """获取 motion_switcher 控制器：优先用 LocoController 注入的，缺失时惰性创建。"""
        loco = self.loco_controller
        if loco is None or not loco.is_running():
            raise RuntimeError("LocoController 未运行，无法访问 MotionSwitcher。")
        return loco.ensure_motion_switcher()

    def _dispatch_motion_switcher(
        self,
        action: str,
        params: Optional[Dict[str, Any]],
    ) -> Tuple[Optional[int], Any]:
        """g1_check_motion_mode / g1_select_motion_mode / g1_release_motion_mode / g1_acquire_high_level 分发。"""
        params = params or {}
        try:
            ms = self._resolve_motion_switcher()
        except Exception as exc:
            LOGGER.error("MotionSwitcher 不可用: %s", exc)
            return (-1, str(exc))

        if action == "g1_check_motion_mode":
            code, name, msg = ms.check_mode()
            return (code, {
                "name": name,
                "is_high_level_owned": name == "advanced",
                "supported_modes": list(SUPPORTED_MODES),
                "target_mode": getattr(self.loco_controller, "_target_mode", "advanced"),
                "message": msg,
            })

        if action == "g1_select_motion_mode":
            target = params.get("name")
            if not target or not isinstance(target, str):
                return (-1, f"g1_select_motion_mode 缺少必要参数 name（{SUPPORTED_MODES}）。")
            code, name, msg = ms.select_mode(target)
            return (code, {"name": name, "target": target, "message": msg})

        if action == "g1_release_motion_mode":
            code, name, msg = ms.release_mode()
            return (code, {"name": name, "message": msg})

        if action == "g1_acquire_high_level":
            target = params.get("name")
            code, name, msg = ms.acquire_high_level(target)
            return (code, {"name": name, "target": target or getattr(ms, "target_mode", "advanced"),
                           "message": msg})

        return (-1, f"MotionSwitcher 未实现的 action: {action}")


def create_g1_assembly(
    config: "NuwaxRobotBridgeConfig",
    iface: Optional[str] = None,
    adapter_configs: Optional[Dict[str, AdapterConfig]] = None,
) -> G1RobotAssembly:
    """创建 G1 入口装配。"""
    return G1RobotAssembly(config=config, iface=iface, adapter_configs=adapter_configs)
