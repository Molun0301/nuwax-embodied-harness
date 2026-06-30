"""G1 运动模式切换控制器（封装 MotionSwitcherClient）。

设计要点（修复 G1 端高层动作 code=None 的真实根因）：
- 宇树 G1 主控把"低高级控制"实现为 MotionSwitcherClient 抢占机制：
    * ai        — AI 自主
    * normal    — 手柄 / 遥控
    * advanced  — 高级 SDK（本控制器目标）
    * ai-w      — 轮式 AI
- LocoClient（sport service RPC）和 LowCmd pub（rt/lowcmd）必须在对应 mode 抢占成功后才会被主控响应。
- example/motionSwitcher/motion_switcher_example.py 显示 high-level 模式切换流程：
    MotionSwitcherClient.Init() → SelectMode("advanced")。
- example/g1/low_level/g1_low_level_example.py:90-99 的
    CheckMode → ReleaseMode → 直到空
  是 low-level LowCmd 发布前释放高层/AI 占主的流程，不应作为 G1 high-level
  LocoClient 接管 advanced 的默认路径。
- LocoClient.Start() 内部 = SetFsmId(200) 不切 mode，所以 Start() 成功 ≠ 模式已切。
- acquire_high_level() 业务级封装：SelectMode("advanced") → CheckMode 复核。

入口：
- 仅供 drivers/g1/ 内部使用，services / core / gateways 不允许 import。
- 类比 drivers/g1/control/loco_controller.py 模板。
"""

from __future__ import annotations

import importlib
import logging
import time
from typing import Any, Optional, Tuple

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.control.motion_switcher_controller")

# SDK 支持的 4 个 mode 串（来自 example/motionSwitcher/motion_switcher_example.py:30-33）
SUPPORTED_MODES: tuple[str, ...] = ("ai", "normal", "advanced", "ai-w")
# 默认目标模式（用户在 2026-06-30 实机反馈场景明确指定 advanced）
DEFAULT_TARGET_MODE = "advanced"
# 抢占/查询超时（SDK 默认 5.0s；与 low_level_example.py:91 对齐）
DEFAULT_CLIENT_TIMEOUT_SEC = 5.0
# 抢占时连续 ReleaseMode 的最大次数（防死循环兜底）
DEFAULT_MAX_RELEASE_ROUNDS = 5
# 每次 CheckMode 之间的间隔（low_level_example.py:99 用的是 1.0s）
DEFAULT_RELEASE_INTERVAL_SEC = 1.0
# acquire_high_level 复核等待时间
DEFAULT_ACQUIRE_CONFIRM_SEC = 0.5


class MotionSwitcherController:
    """G1 运动模式切换控制器。"""

    def __init__(
        self,
        iface: Optional[str] = None,
        *,
        client_timeout_sec: float = DEFAULT_CLIENT_TIMEOUT_SEC,
        target_mode: str = DEFAULT_TARGET_MODE,
        acquire_on_init: bool = True,
        max_release_rounds: int = DEFAULT_MAX_RELEASE_ROUNDS,
        release_interval_sec: float = DEFAULT_RELEASE_INTERVAL_SEC,
    ) -> None:
        self.iface = iface
        self.client_timeout_sec = float(client_timeout_sec)
        self.target_mode = self._validate_mode(target_mode)
        self.acquire_on_init = bool(acquire_on_init)
        self.max_release_rounds = int(max_release_rounds)
        self.release_interval_sec = float(release_interval_sec)
        self.is_initialized = False
        self.ms_client: Any = None
        # 缓存最近一次 CheckMode 结果
        self._last_mode_name: str = ""

    # ------------------ 生命周期 ------------------

    def init(self) -> bool:
        """初始化 MotionSwitcherClient + 可选自动抢占 high level。

        返回 True 表示 init 成功；acquire_high_level 失败不影响 init 状态（仅记 warning），
        因为启动期主控可能尚未 enable_motor，调用方可在 g1_select_motion_mode 时再重试。
        """
        if self.is_initialized:
            LOGGER.warning("MotionSwitcherController 已初始化")
            return True

        try:
            channel_module = importlib.import_module("unitree_sdk2py.core.channel")
            channel_factory_initialize = getattr(channel_module, "ChannelFactoryInitialize")
            if self.iface:
                channel_factory_initialize(0, self.iface)
            else:
                channel_factory_initialize(0)

            ms_module = importlib.import_module(
                "unitree_sdk2py.comm.motion_switcher.motion_switcher_client"
            )
            MotionSwitcherClient = getattr(ms_module, "MotionSwitcherClient")
            self.ms_client = MotionSwitcherClient()
            self.ms_client.SetTimeout(self.client_timeout_sec)
            self.ms_client.Init()
            LOGGER.info(
                "MotionSwitcherClient 初始化成功（iface=%s, timeout=%.1fs, target=%s）",
                self.iface or "default", self.client_timeout_sec, self.target_mode,
            )
        except Exception as exc:
            LOGGER.error("MotionSwitcherClient 初始化失败: %s", exc)
            return False

        self.is_initialized = True

        # 启动期自动抢占 advanced；抛异常视为 hard fail，由调用方走补救。
        # 任何 RPC 异常都不能被静默吞掉（实机 2026-06-30 教训：mode=ai 时 Release 抛错被吞了，
        # 导致 LocoClient 后续 RPC 全部 code=None）。
        if self.acquire_on_init:
            try:
                code, current_name, msg = self.acquire_high_level()
            except Exception as exc:  # noqa: BLE001
                LOGGER.error(
                    "MotionSwitcher 启动期自动抢占抛异常（多半是 ReleaseMode 与 ai 模式冲突）：%s；"
                    "请检查 G1 主控是否处于 ai 模式并人工切到 advanced 后再启动网关",
                    exc,
                )
                self.is_initialized = False
                self.ms_client = None
                return False
            if code == 0:
                LOGGER.info(
                    "MotionSwitcher 启动期自动抢占成功（mode=%s → %s）",
                    current_name or "(空)", self.target_mode,
                )
            else:
                LOGGER.error(
                    "MotionSwitcher 启动期自动抢占失败 code=%s current=%s msg=%s；"
                    "hard fail — LocoController 不应继续 Init/Start，否则高层动作仍 code=None",
                    code, current_name, msg,
                )
                self.is_initialized = False
                self.ms_client = None
                return False
        return True

    def stop(self) -> None:
        """停止 MotionSwitcherController。

        注意：**不主动 ReleaseMode**。原因：Release 会让运动立刻失主，可能引发跌倒；
        留给上层 stop_gateway 走完所有动作后的自然 stop 流程，或调用方显式调 release_mode。
        """
        if not self.is_initialized:
            return
        self.ms_client = None
        self.is_initialized = False
        LOGGER.info("MotionSwitcherController 已停止。")

    def is_running(self) -> bool:
        return self.is_initialized and self.ms_client is not None

    # ------------------ 4 个公开方法（对应 MCP 4 个 capability） ------------------

    def check_mode(self) -> Tuple[int, str, str]:
        """查询当前主控模式。

        Returns:
            (code, name, message)
            code=0 + name ∈ SUPPORTED_MODES|"" 时成功；name=="" 表示空闲无人占主。
            code≠0 时 message 含错误描述。
        """
        if not self.is_running():
            return (-1, "", "MotionSwitcherController 未初始化。")
        try:
            code, result = self.ms_client.CheckMode()
        except Exception as exc:
            LOGGER.error("MotionSwitcher CheckMode 抛错: %s", exc)
            return (-1, "", f"CheckMode 抛错: {exc}")
        if code != 0 or result is None:
            return (int(code) if code is not None else -1, "", "CheckMode 失败")
        name = str(result.get("name", "") or "")
        self._last_mode_name = name
        return (0, name, "")

    def select_mode(self, name: str) -> Tuple[int, str, str]:
        """抢占为目标模式。

        Args:
            name: ai | normal | advanced | ai-w
        Returns:
            (code, current_name, message)
        """
        if not self.is_running():
            return (-1, "", "MotionSwitcherController 未初始化。")
        try:
            target = self._validate_mode(name)
        except ValueError as exc:
            return (-1, "", str(exc))
        # SDK 官方 motion_switcher_example.py 直接 SelectMode(name) 在 ai/normal/advanced/ai-w
        # 间切换；ReleaseMode 只用于 low-level 示例在发布 LowCmd 前清空当前占主。
        current_code, current_name, _ = self.check_mode()
        if current_code != 0:
            return (current_code, current_name, "抢占前 CheckMode 失败")
        if current_name == target:
            LOGGER.info("MotionSwitcher 当前已在目标模式 %s，跳过 SelectMode", target)
            return (0, target, "")
        try:
            ret_code, _ = self.ms_client.SelectMode(target)
        except Exception as exc:
            LOGGER.error("MotionSwitcher SelectMode(%s) 抛错: %s", target, exc)
            return (-1, current_name, f"SelectMode 抛错: {exc}")
        if ret_code != 0:
            return (int(ret_code) if ret_code is not None else -1, current_name,
                    f"SelectMode({target}) 失败")
        # 等一小段时间后复核（主控 fsm 切换有 ms 级延迟）
        time.sleep(DEFAULT_ACQUIRE_CONFIRM_SEC)
        confirm_code, confirm_name, _ = self.check_mode()
        if confirm_code == 0 and confirm_name == target:
            LOGGER.info("MotionSwitcher SelectMode(%s) 成功并复核通过", target)
            return (0, target, "")
        return (
            -1 if confirm_code == 0 else confirm_code,
            confirm_name or current_name,
            "SelectMode 返回成功但复核模式不匹配，请检查主控状态",
        )

    def release_mode(self) -> Tuple[int, str, str]:
        """主动释放当前持有的模式（让位给手柄/AI）。"""
        if not self.is_running():
            return (-1, "", "MotionSwitcherController 未初始化。")
        try:
            ret_code, _ = self.ms_client.ReleaseMode()
        except Exception as exc:
            LOGGER.error("MotionSwitcher ReleaseMode 抛错: %s", exc)
            return (-1, "", f"ReleaseMode 抛错: {exc}")
        # 复核（release 后通常 name==""）
        time.sleep(DEFAULT_ACQUIRE_CONFIRM_SEC)
        _, current_name, _ = self.check_mode()
        if ret_code == 0:
            LOGGER.info("MotionSwitcher ReleaseMode 成功（当前 mode=%s）", current_name or "(空)")
            return (0, current_name, "")
        return (int(ret_code) if ret_code is not None else -1, current_name,
                f"ReleaseMode 失败 code={ret_code}")

    def acquire_high_level(
        self,
        target: Optional[str] = None,
    ) -> Tuple[int, str, str]:
        """业务级封装：SelectMode(target) → CheckMode 复核。

        流程对应 example/motionSwitcher/motion_switcher_example.py 的 high-level
        模式切换范式。low-level 示例中的 ReleaseMode 循环只用于 LowCmd 发布前清空
        当前占主，不适合作为 G1 high-level advanced 接管的默认路径。
        默认 target = self.target_mode（advanced）。

        Returns:
            (code, current_name, message)
            code=0 + current_name == target 时真正抢到。
        """
        if not self.is_running():
            return (-1, "", "MotionSwitcherController 未初始化。")
        try:
            target_mode = self._validate_mode(target) if target else self.target_mode
        except ValueError as exc:
            return (-1, "", str(exc))

        sel_code, current_name, sel_msg = self.select_mode(target_mode)
        if sel_code != 0:
            return (sel_code, current_name, sel_msg)
        return (0, current_name, "")

    # ------------------ 内部 ------------------

    @staticmethod
    def _validate_mode(name: Optional[str]) -> str:
        """校验 mode 串是否在 SDK 支持集合内。"""
        if not name or not isinstance(name, str):
            raise ValueError(f"motion mode 必须为非空字符串：{name!r}")
        normalized = name.strip().lower()
        if normalized not in SUPPORTED_MODES:
            raise ValueError(
                f"不支持的 motion mode={name!r}，SDK 仅支持 {SUPPORTED_MODES}"
            )
        return normalized


def main() -> None:
    """CLI 入口：python3 -m drivers.g1.control.motion_switcher_controller check|select|release|acquire [--name advanced]"""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="G1 MotionSwitcherController CLI 测试入口")
    parser.add_argument("command", choices=["check", "select", "release", "acquire"],
                        help="check=查模式；select=抢占指定模式；release=释放；acquire=业务级抢占")
    parser.add_argument("--iface", default=None, help="DDS 网卡（默认沿用 ChannelFactoryInitialize(0)）")
    parser.add_argument("--name", default=None,
                        help="select/acquire 的目标 mode（ai/normal/advanced/ai-w）")
    parser.add_argument("--timeout", type=float, default=DEFAULT_CLIENT_TIMEOUT_SEC,
                        help="MotionSwitcherClient 超时（秒）")
    parser.add_argument("--no-auto-acquire", action="store_true",
                        help="init 时不自动抢占（默认会抢）")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    controller = MotionSwitcherController(
        iface=args.iface,
        client_timeout_sec=args.timeout,
        acquire_on_init=not args.no_auto_acquire,
    )
    if not controller.init():
        LOGGER.error("MotionSwitcherController.init() 失败")
        sys.exit(1)

    try:
        if args.command == "check":
            code, name, msg = controller.check_mode()
            print(f"check: code={code} name={name!r} msg={msg}")
        elif args.command == "select":
            code, name, msg = controller.select_mode(args.name or DEFAULT_TARGET_MODE)
            print(f"select: code={code} name={name!r} msg={msg}")
        elif args.command == "release":
            code, name, msg = controller.release_mode()
            print(f"release: code={code} name={name!r} msg={msg}")
        elif args.command == "acquire":
            code, name, msg = controller.acquire_high_level(args.name)
            print(f"acquire: code={code} name={name!r} msg={msg}")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
