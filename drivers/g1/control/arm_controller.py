"""G1 手臂动作控制器（封装 G1ArmActionClient）。

提供单一透传入口 `execute_action(action_seq)`，由上层负责拼装 Action 序列。
Action API 形态按宇树 G1 SDK 1.0.1：序列号 + 持续时间 + 关键点序列。

设计要点：
- 本类仅端侧独占（drivers/g1/ 子树内），services / core / gateways 一行不碰。
- 单一 execute_action 入口方便上层（capabilities.py: arm_action）做参数透传。
- 类比 drivers/go2/control/low_level_controller.py:38-65 模板（生命周期 init/start/stop 一致）。
"""

from __future__ import annotations

import importlib
import logging
from typing import Any, List, Optional, Sequence

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.control.arm_controller")

DEFAULT_IFACE = "eth0"
DEFAULT_CLIENT_TIMEOUT_SEC = 10.0


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

            # 预加载 arm_sdk_msg_dds 顶层 API（供 execute_action 拼装 sequence 用）
            try:
                self._action_module = importlib.import_module("unitree_sdk2py.g1.arm.g1_arm_action_api")
            except Exception as exc:
                LOGGER.warning("arm action API 加载失败（execute_action 仍可透传原始序列）: %s", exc)
        except Exception as exc:
            LOGGER.error("G1ArmActionClient 初始化失败: %s", exc)
            return False

        self.is_initialized = True
        LOGGER.info("ArmController 初始化成功（iface=%s, timeout=%.1fs）", self.iface or "default", self.client_timeout_sec)
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

    def execute_action(self, action_seq: Sequence[Any]) -> int:
        """执行手臂 Action 序列（透传）。

        Args:
            action_seq: G1 SDK arm action 序列（具体形态由 SDK 决定，常见为：
                1) 数字 sequence_id（按 SDK 模板拼装）
                2) Sequence dict（如 {"seq": [...]}）
                3) 自定义 action 对象（透传给 SDK）

        Returns:
            SDK 返回码（0 表示成功）
        """
        if not self.is_running():
            raise RuntimeError("ArmController 未初始化")
        try:
            # 当前 SDK 1.0.1 提供 ExecuteAction 模板方法；具体 action 序列由调用方拼装
            method = getattr(self.arm_client, "ExecuteAction", None)
            if method is None:
                raise AttributeError("G1ArmActionClient 缺少 ExecuteAction 方法（SDK 版本不匹配？）")
            code = method(list(action_seq) if not isinstance(action_seq, list) else action_seq)
            LOGGER.info("Arm ExecuteAction(seq_len=%d) code=%s", len(list(action_seq)), code)
            return int(code) if code is not None else 0
        except Exception as exc:
            LOGGER.error("Arm ExecuteAction 抛错: %s", exc)
            raise


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
