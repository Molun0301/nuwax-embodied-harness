"""G1 手臂动作控制器（封装 G1ArmActionClient）。

提供官方 Arm Action RPC 的薄封装：``ExecuteAction(int action_id)`` 执行固件内置
手臂互动动作；``ExecuteAction(str action_name)`` 透传 App 示教动作名称（若 SDK/固件支持）。
挥手、握手等互动功能属于 Arm Action 服务，不属于 Loco/FSM 模式切换。

设计要点：
- 本类仅端侧独占（drivers/g1/ 子树内），services / core / gateways 一行不碰。
- 单一 execute_action 入口方便上层（capabilities.py: arm_action）做参数透传。
- 类比 drivers/go2/control/low_level_controller.py:38-65 模板（生命周期 init/start/stop 一致）。
"""

from __future__ import annotations

import importlib
import logging
import time
from typing import Any, Mapping, Optional

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.control.arm_controller")

DEFAULT_IFACE = "eth0"
DEFAULT_CLIENT_TIMEOUT_SEC = 10.0
RELEASE_ARM_ACTION_NAME = "release arm"

# 宇树官方 G1 Arm Action 文档 / unitree_sdk2_python.g1.arm.g1_arm_action_client.action_map
# 的当前动作表。固件动作 ID 可能更新；init() 时若 SDK 暴露 action_map，会以 SDK 为准。
DEFAULT_ARM_ACTION_MAP: dict[str, int] = {
    RELEASE_ARM_ACTION_NAME: 99,
    "恢复初始手臂位姿": 99,
    "two-hand kiss": 11,
    "双手飞吻": 11,
    "left kiss": 12,
    "单手飞吻": 12,
    "right kiss": 13,
    "hands up": 15,
    "平举": 15,
    "clap": 17,
    "鼓掌": 17,
    "high five": 18,
    "击掌": 18,
    "hug": 19,
    "拥抱": 19,
    "heart": 20,
    "双手比心": 20,
    "right heart": 21,
    "右手比心": 21,
    "reject": 22,
    "双手打X": 22,
    "right hand up": 23,
    "右手平举": 23,
    "x-ray": 24,
    "动感光波": 24,
    "face wave": 25,
    "胸前挥手": 25,
    "high wave": 26,
    "高举挥手": 26,
    "shake hand": 27,
    "握手": 27,
}


class ArmController:
    """G1 手臂 Action 控制器。"""

    def __init__(
        self,
        iface: Optional[str] = None,
        *,
        client_timeout_sec: float = DEFAULT_CLIENT_TIMEOUT_SEC,
    ) -> None:
        self.iface = iface
        self.client_timeout_sec = float(client_timeout_sec)
        self.is_initialized = False
        self.arm_client: Any = None
        self._action_module: Any = None
        self._action_idl_cls: Any = None
        self._action_map: dict[str, int] = dict(DEFAULT_ARM_ACTION_MAP)

    def init(self) -> bool:
        """初始化 G1ArmActionClient。"""

        if self.is_initialized:
            LOGGER.warning("ArmController 已初始化")
            return True

        try:
            channel_module = importlib.import_module("unitree_sdk2py.core.channel")
            channel_factory_initialize = getattr(channel_module, "ChannelFactoryInitialize")
            if self.iface:
                channel_factory_initialize(0, self.iface)
            else:
                channel_factory_initialize(0)

            arm_module = importlib.import_module("unitree_sdk2py.g1.arm.g1_arm_action_client")
            G1ArmActionClient = getattr(arm_module, "G1ArmActionClient")
            self.arm_client = G1ArmActionClient()
            self.arm_client.SetTimeout(self.client_timeout_sec)
            self.arm_client.Init()

            sdk_action_map = getattr(arm_module, "action_map", None)
            if isinstance(sdk_action_map, Mapping):
                self._action_map.update(
                    {
                        str(name): int(action_id)
                        for name, action_id in sdk_action_map.items()
                        if isinstance(name, str) and type(action_id) is int
                    }
                )

            # 预加载官方 API 常量；ExecuteAction 本身由 G1ArmActionClient 封装。
            try:
                self._action_module = importlib.import_module("unitree_sdk2py.g1.arm.g1_arm_action_api")
            except Exception as exc:
                LOGGER.warning("arm action API 加载失败（ExecuteAction 仍可用）: %s", exc)
        except Exception as exc:
            LOGGER.error("G1ArmActionClient 初始化失败: %s", exc)
            return False

        self.is_initialized = True
        LOGGER.info(
            "ArmController 初始化成功（iface=%s, timeout=%.1fs）",
            self.iface or "default",
            self.client_timeout_sec,
        )
        return True

    def stop(self) -> None:
        """停止 ArmController。"""
        if not self.is_initialized:
            return
        try:
            self.arm_client = None
        except Exception as exc:
            LOGGER.warning("ArmController stop 失败: %s", exc)
        self.is_initialized = False
        LOGGER.info("ArmController 已停止。")

    def is_running(self) -> bool:
        return self.is_initialized and self.arm_client is not None

    def resolve_action_id(self, action_name: str) -> int:
        """按动作名解析当前可用的预置 Arm Action ID。"""

        normalized = str(action_name or "").strip()
        if not normalized:
            raise ValueError("action_name 不能为空")
        try:
            return int(self._action_map[normalized])
        except KeyError as exc:
            supported = ", ".join(sorted(self._action_map))
            raise ValueError(f"未知 G1 Arm Action 名称: {action_name}。当前支持: {supported}") from exc

    def execute_preset_action(
        self,
        action_name: str,
        *,
        release_after_sec: float | None = None,
    ) -> int:
        """执行固件内置手臂互动动作，并可按官方示例自动恢复手臂位姿。

        Args:
            action_name: SDK ``action_map`` 中的英文动作名，例如 ``face wave`` /
                ``high wave`` / ``shake hand``。
            release_after_sec: 非 None 时，在动作触发成功后等待该秒数，再执行
                ``release arm``（ID 99）。官方 Python 示例对 ``shake hand`` 使用
                约 2 秒保持后恢复。
        """

        action_id = self.resolve_action_id(action_name)
        code = self.execute_action(action_id)
        if code != 0 or release_after_sec is None:
            return code
        if release_after_sec > 0:
            time.sleep(float(release_after_sec))
        release_id = self.resolve_action_id(RELEASE_ARM_ACTION_NAME)
        return self.execute_action(release_id)

    def get_action_list(self) -> tuple[int, Any]:
        """返回固件侧当前可用动作列表（以服务端响应为准）。"""

        if not self.is_running():
            raise RuntimeError("ArmController 未初始化")
        method = getattr(self.arm_client, "GetActionList", None)
        if method is None:
            raise AttributeError("G1ArmActionClient 缺少 GetActionList 方法（SDK 版本不匹配？）")
        result = method()
        if isinstance(result, tuple) and len(result) == 2:
            return int(result[0]) if result[0] is not None else -1, result[1]
        return 0, result

    def execute_action(self, action: Any) -> int:
        """执行 G1 Arm Action。

        Args:
            action: 官方 Arm Action 入参：
                1) ``int``：固件预置动作 ID（如 25=胸前挥手，27=握手，99=恢复手臂）
                2) ``str``：若命中 SDK ``action_map``，先解析为预置 ID；否则按示教动作名透传
                3) 兼容旧调用：长度为 1 的 list/tuple 会解包；其它序列会拒绝，避免把
                   旧的 action_seq 误传给官方 ``ExecuteAction(int)``。

        Returns:
            SDK 返回码（0 表示成功）
        """
        if not self.is_running():
            raise RuntimeError("ArmController 未初始化")
        try:
            method = getattr(self.arm_client, "ExecuteAction", None)
            if method is None:
                raise AttributeError("G1ArmActionClient 缺少 ExecuteAction 方法（SDK 版本不匹配？）")

            resolved_action = self._normalize_action_input(action)
            code = method(resolved_action)
            LOGGER.info("Arm ExecuteAction(action=%r) code=%s", resolved_action, code)
            return int(code) if code is not None else 0
        except Exception as exc:
            LOGGER.error("Arm ExecuteAction 抛错: %s", exc)
            raise

    def _normalize_action_input(self, action: Any) -> int | str:
        if isinstance(action, (list, tuple)):
            if len(action) == 1:
                action = action[0]
            else:
                raise ValueError(
                    "G1 ArmAction 官方接口不是 action_seq 序列；请传 action_id(int) "
                    "或 action_name(str)，例如 action_id=25 / action_name='shake hand'。"
                )
        if type(action) is int:
            return action
        if isinstance(action, str):
            name = action.strip()
            if not name:
                raise ValueError("action_name 不能为空")
            return self._action_map.get(name, name)
        raise TypeError(
            f"G1 ArmAction 入参类型不支持: {type(action).__name__}；"
            "请传 action_id(int) 或 action_name(str)。"
        )


def main() -> None:
    """CLI 入口：python3 -m drivers.g1.control.arm_controller init|status"""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="G1 ArmController CLI 测试入口")
    parser.add_argument("command", choices=["init", "status"], help="仅做 init 自检 / 状态查询（Action 执行需在网关内）")
    parser.add_argument("--iface", default=DEFAULT_IFACE, help=f"DDS 网卡（默认 {DEFAULT_IFACE}）")
    parser.add_argument("--timeout", type=float, default=DEFAULT_CLIENT_TIMEOUT_SEC, help="G1ArmActionClient 超时（秒）")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    controller = ArmController(iface=args.iface, client_timeout_sec=args.timeout)
    if args.command == "init":
        ok = controller.init()
        print(f"init={'OK' if ok else 'FAILED'}")
        sys.exit(0 if ok else 1)
    else:
        print(f"is_initialized={controller.is_initialized} is_running={controller.is_running()}")


if __name__ == "__main__":
    main()
