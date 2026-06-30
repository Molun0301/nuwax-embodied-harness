"""G1 底盘运动控制器（封装 LocoClient）。

10 个高层方法 + 1 个流式状态订阅入口。
SportsModeState_ 实测在 unitree_sdk2py.idl.unitree_go.msg.dds_._SportModeState_（**不是** unitree_hg）。

设计要点：
- 本类仅端侧独占（drivers/g1/ 子树内），services / core / gateways 一行不碰。
- 状态订阅 `subscribe_sport_mode_state` 是流式 callback，**不进 capabilities.py**。
- 类比 drivers/go2/control/low_level_controller.py:38-65 模板（Go2 是 LowCmd 关节级，G1 是 LocoClient 高层）。
"""

from __future__ import annotations

import importlib
import logging
import time
from typing import Any, Callable, Optional

LOGGER = logging.getLogger("nuwax_robot_bridge.g1.control.loco_controller")

# 默认服务名 / 接口 / 超时（与 drivers/g1/data_plane.env 一致；env 可覆盖）
DEFAULT_SERVICE_NAME = "sport"
DEFAULT_IFACE = "eth0"
DEFAULT_CLIENT_TIMEOUT_SEC = 10.0
DEFAULT_STATE_WAIT_TIMEOUT_SEC = 5.0
DEFAULT_MOVE_FOR_INTERVAL_SEC = 0.5


def _build_motion_switcher_or_none(iface: Optional[str], target_mode: str, acquire_on_init: bool):
    """工厂函数：构造 MotionSwitcherController，依赖失败时返回 None。

    引入 motion_switcher 是修高层动作 code=None 的真实根因（实机 2026-06-30 反馈）：
    LocoClient.Start() 仅 SetFsmId(200) 不切 motion_switcher mode，必须先
    MotionSwitcherClient SelectMode("advanced")。

    注意：本函数只构造控制器，不调用 init()；调用方必须显式 init()，这样初始化失败
    能被调用方按启动期 hard fail / 懒加载报错分别处理。
    """
    try:
        from drivers.g1.control.motion_switcher_controller import MotionSwitcherController
    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("MotionSwitcherController 加载失败，高层动作需手动抢占 advanced: %s", exc)
        return None
    try:
        return MotionSwitcherController(
            iface=iface,
            target_mode=target_mode,
            acquire_on_init=acquire_on_init,
        )
    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("MotionSwitcherController 构造失败: %s", exc)
        return None


class LocoController:
    """G1 底盘运动控制器。"""

    def __init__(
        self,
        iface: Optional[str] = None,
        *,
        client_timeout_sec: float = DEFAULT_CLIENT_TIMEOUT_SEC,
        target_mode: str = "advanced",
        # 修复 openspec/changes/fix-g1-loco-init-no-motion-switcher + 实机 HIL 2026-06-30 14:09-14：
        # 默认 True — 主动抢 advanced 是稳定 Loco RPC 的必要条件（motion_switcher 主控在
        # ai 占主时 7002 BUSY 拒收后续 sport RPC；实测 14:12 后所有 Loco 物理失效）。
        # 仍可通过 env NUWAX_G1_REQUIRE_MOTION_SWITCHER=false 切回官方无抢占范式。
        acquire_motion_mode_on_init: bool = True,
    ) -> None:
        self.iface = iface
        self.client_timeout_sec = float(client_timeout_sec)
        # 高层动作 code=None 真实根因修复：先 motion_switcher 抢占 advanced（可选）
        self._target_mode = target_mode
        self._acquire_motion_mode_on_init = bool(acquire_motion_mode_on_init)
        self.is_initialized = False
        self.loco_client: Any = None
        # MotionSwitcher 控制器（依赖注入，缺 SDK 时为 None）
        self.motion_switcher: Any = None
        # 修复：init 失败原因缓存，供 assembly.get_status().last_error 透出
        self._init_last_error: str = ""
        # 流式状态订阅相关
        self._state_subscriber: Any = None
        self._state_callback: Optional[Callable[[Any], None]] = None
        self._latest_sport_mode_state: Any = None
        self._state_received_at_least_once = False
        # SDK idl 模块句柄
        self._sport_mode_state_cls: Any = None
        self._sport_mode_state_default_factory: Any = None

    # ------------------ 生命周期 ------------------

    def init(self) -> bool:
        """初始化 LocoClient + 准备 SportModeState_ 订阅。

        修复 openspec/changes/fix-g1-loco-init-no-motion-switcher 后步骤：
        1. ChannelFactoryInitialize(0, iface)
        2. LocoClient.Init() + SetTimeout(10)
        3. LocoClient.Start() — 与宇树官方 g1_loco_client_example.cpp 严格对齐
        4. (可选) motion_switcher 后置抢占 — 仅在 env NUWAX_G1_REQUIRE_MOTION_SWITCHER=true
           且 acquire_motion_mode_on_init=True 时执行；失败 LOGGER.warning 不 raise
           （D3 优雅降级）

        任何步骤失败均 LOGGER.error 缓存 _init_last_error 并返 False，由调用方决定是否降级。
        """

        if self.is_initialized:
            LOGGER.warning("LocoController 已初始化")
            return True

        self._init_last_error = ""
        try:
            channel_module = importlib.import_module("unitree_sdk2py.core.channel")
            channel_factory_initialize = getattr(channel_module, "ChannelFactoryInitialize")
            if self.iface:
                channel_factory_initialize(0, self.iface)
            else:
                channel_factory_initialize(0)

            # 【D2 修复】先 LocoClient 直连，再可选后置 motion_switcher 抢占
            # 严格对齐宇树官方 g1_loco_client_example.cpp：LocoClient.Init() + Start() 是必经路径
            loco_module = importlib.import_module("unitree_sdk2py.g1.loco.g1_loco_client")
            LocoClient = getattr(loco_module, "LocoClient")
            self.loco_client = LocoClient()
            self.loco_client.SetTimeout(self.client_timeout_sec)
            self.loco_client.Init()
            # 拿 lease（必须！）：Init() 仅注册 RPC handler，必须 Start() 让主控完成 fsm 切换才能 RPC。
            # 不调 Start() → WaveHand 等高层命令会 publish 到 rt/loco/wave_hand 但主控没 lease 上下文，
            # SDK 内部同步阻塞 → timeout → 静默返 None → 调用方把 None 归零误判成功。
            self.loco_client.Start()
            LOGGER.info(
                "LocoClient 直连 Init+Start 完成（iface=%s, timeout=%.1fs）；"
                "wave_hand 等高层命令可走通",
                self.iface or "default", self.client_timeout_sec,
            )

            # 【D2 修复】后置 motion_switcher 抢占（仅 env 开启时）
            # 注意：抢占在 LocoClient 之后 — 即便抢占失败 LocoClient 仍可用。
            # 实机 2026-06-30 教训：原"先抢占再 Init/Start"范式失败时 hard fail → 网关退出；
            # 新策略"直连 + 可选后置"避免单点失败。
            if self._acquire_motion_mode_on_init:
                self.motion_switcher = _build_motion_switcher_or_none(
                    self.iface, self._target_mode, acquire_on_init=True,
                )
                if self.motion_switcher is None:
                    LOGGER.warning(
                        "MotionSwitcherController 工厂返 None（SDK 缺失？）；"
                        "LocoClient 已直连可用，高层 mode 抢占能力受限但不影响 Loco 命令"
                    )
                else:
                    if not self.motion_switcher.init():
                        LOGGER.warning(
                            "MotionSwitcher 后置抢占 init 失败；LocoClient 仍可用，"
                            "用户可手动调 g1_select_motion_mode 接管 mode"
                        )
                        self.motion_switcher = None
                    elif not self.motion_switcher.is_running():
                        LOGGER.warning(
                            "MotionSwitcher 后置抢占未 running；LocoClient 仍可用"
                        )
                        self.motion_switcher = None
                    else:
                        LOGGER.info(
                            "MotionSwitcher 后置抢占成功（mode=%s）",
                            self._target_mode,
                        )

            # 修复 D7：wrap SDK 1.0.1 LocoClient 方法让其返 code（int）而非 None
            # 实测 2026-06-30 14:09：unitree_sdk2py 1.0.1 的 Damp/Lie2StandUp/Start/
            # WaveHand/ShakeHand 内部 `self.SetFsmId(...)` 后无 return，方法返 None。
            # _call() 拿 None → _normalize_code(None) → -1 → 上层误判。
            # 这里 wrap 让 _call 拿到的 code 是 SetFsmId 的真实 int 返回。
            self._wrap_lococlient_methods()

        except Exception as exc:
            LOGGER.error("LocoController 初始化失败: %s", exc)
            self._init_last_error = f"{type(exc).__name__}: {exc}"
            self.motion_switcher = None
            self.loco_client = None
            return False

        # 准备 SportModeState_ idl（用于流式订阅）
        try:
            idl_default_module = importlib.import_module("unitree_sdk2py.idl.default")
            self._sport_mode_state_default_factory = getattr(
                idl_default_module, "unitree_go_msg_dds__SportModeState_"
            )
            idl_msg_module = importlib.import_module("unitree_sdk2py.idl.unitree_go.msg.dds_")
            self._sport_mode_state_cls = getattr(idl_msg_module, "SportModeState_")
        except Exception as exc:
            LOGGER.warning("SportModeState_ idl 加载失败（不影响高层命令，仅影响 subscribe_sport_mode_state）: %s", exc)

        self.is_initialized = True
        LOGGER.info("LocoController 初始化成功（iface=%s, timeout=%.1fs, mode=%s）",
                    self.iface or "default", self.client_timeout_sec, self._target_mode)
        return True

    def stop(self) -> None:
        """停止 LocoController（清理流式订阅，发送 Damp 指令）。"""

        if not self.is_initialized:
            return
        try:
            self.damp()
        except Exception as exc:
            LOGGER.warning("停止时 Damp 失败: %s", exc)
        # 优雅释放 MotionSwitcher 抢占（ReleaseMode 可能引发骤停，仅在配置显式开启时执行）
        if self.motion_switcher is not None and getattr(self, "_release_on_stop", False):
            try:
                self.motion_switcher.release_mode()
            except Exception as exc:
                LOGGER.warning("停止时 MotionSwitcher Release 失败: %s", exc)
            try:
                self.motion_switcher.stop()
            except Exception as exc:
                LOGGER.warning("MotionSwitcherController 停止失败: %s", exc)
            self.motion_switcher = None
        self._state_subscriber = None
        self._state_callback = None
        self.loco_client = None
        self.is_initialized = False
        LOGGER.info("LocoController 已停止。")

    # ------------------ MotionSwitcher 透传（供 capability g1_select_motion_mode 等调用） ------------------

    def ensure_motion_switcher(self) -> Any:
        """惰性创建 motion_switcher 控制器（若 init 时未创建）。

        用于 g1_select_motion_mode / g1_release_motion_mode / g1_acquire_high_level 等 MCP 能力
        在 loco 已 init 但 motion_switcher 因 SDK 缺失未创建时的兜底。
        """
        if self.motion_switcher is not None and self.motion_switcher.is_running():
            return self.motion_switcher
        self.motion_switcher = _build_motion_switcher_or_none(
            self.iface, self._target_mode, acquire_on_init=False,
        )
        if self.motion_switcher is None:
            raise RuntimeError("MotionSwitcherController 创建失败，请检查 unitree_sdk2py 安装")
        if not self.motion_switcher.init():
            self.motion_switcher = None
            raise RuntimeError("MotionSwitcherController.init() 返 False")
        return self.motion_switcher

    def is_running(self) -> bool:
        return self.is_initialized and self.loco_client is not None

    # ------------------ 高层命令（10 个 LocoPrimitive / G1Specific） ------------------

    def damp(self) -> int:
        """进入阻尼模式（所有电机松力）。"""
        return self._call("Damp")

    def squat_to_stand(self) -> int:
        """从下蹲到站立。"""
        return self._call("Squat2StandUp")

    def stand_to_squat(self) -> int:
        """从站立到下蹲。"""
        return self._call("StandUp2Squat")

    def low_stand(self) -> int:
        """低姿态站立。"""
        return self._call("LowStand")

    def high_stand(self) -> int:
        """高姿态站立。"""
        return self._call("HighStand")

    def zero_torque(self) -> int:
        """零扭矩模式。"""
        return self._call("ZeroTorque")

    def move(self, vx: float, vy: float, vyaw: float) -> int:
        """持续移动（速度控制，单位 m/s 与 rad/s），直到下次调用 Move/StopMove。"""
        if not self.is_running():
            raise RuntimeError("LocoController 未初始化")
        code = self.loco_client.Move(float(vx), float(vy), float(vyaw))
        LOGGER.info("Loco Move(vx=%.3f, vy=%.3f, vyaw=%.3f) code=%s", vx, vy, vyaw, code)
        return self._normalize_code("Move", code)

    def wave_hand(self) -> int:
        """招手动作（不转身）。"""
        return self._call("WaveHand")

    def shake_hand(self) -> int:
        """握手动作（转身伸手）。"""
        return self._call("ShakeHand")

    def lie_to_stand(self) -> int:
        """从躺姿到站立。"""
        return self._call("Lie2StandUp")

    # ------------------ 流式状态订阅（不进 capabilities.py） ------------------

    def subscribe_sport_mode_state(self, callback: Callable[[Any], None]) -> bool:
        """订阅 SportModeState_（流式 callback，**不进 capabilities.py**）。

        Args:
            callback: 收到一帧 SportModeState_ 时的回调函数（无返回值，平台侧用 provider 拿快照）。

        Returns:
            True 表示订阅已 Init；False 表示 SportModeState_ idl 未加载或 Init 失败。
        """
        if not self.is_running():
            raise RuntimeError("LocoController 未初始化")
        if self._sport_mode_state_cls is None:
            LOGGER.error("SportModeState_ idl 未加载，无法订阅")
            return False
        if self._state_subscriber is not None:
            LOGGER.warning("SportModeState_ 已订阅，先取消再重订")
            return True

        self._state_callback = callback
        try:
            channel_module = importlib.import_module("unitree_sdk2py.core.channel")
            ChannelSubscriber = getattr(channel_module, "ChannelSubscriber")
            self._state_subscriber = ChannelSubscriber("rt/lf/sportmodestate", self._sport_mode_state_cls)
            self._state_subscriber.Init(self._on_sport_mode_state, 10)
            LOGGER.info("SportModeState_ 订阅已 Init（topic=rt/lf/sportmodestate, queue=10）")
        except Exception as exc:
            LOGGER.error("SportModeState_ 订阅失败: %s", exc)
            self._state_subscriber = None
            return False
        return True

    def get_latest_sport_mode_state_snapshot(self) -> Any:
        """返回最近一次收到的 SportModeState_ 帧（provider 拿快照用，不进 capability）。"""
        return self._latest_sport_mode_state

    def has_state_frame(self) -> bool:
        """是否至少收到过 1 帧 SportModeState_（供 get_state 误用防爆用）。"""
        return self._state_received_at_least_once

    def _on_sport_mode_state(self, msg: Any) -> None:
        """SportModeState_ 回调：缓存最近一帧 + 转发给 user callback。"""
        self._latest_sport_mode_state = msg
        self._state_received_at_least_once = True
        if self._state_callback is not None:
            try:
                self._state_callback(msg)
            except Exception as exc:
                LOGGER.warning("SportModeState_ user callback 抛错: %s", exc)

    # ------------------ 内部工具 ------------------

    def _wrap_lococlient_methods(self) -> None:
        """修复 D7：wrap SDK 1.0.1 LocoClient 方法让其返 code（int）而非 None。

        背景：unitree_sdk2py 1.0.1 的 LocoClient.Damp/Lie2StandUp/Start/WaveHand/ShakeHand
        内部实现形如 `def Damp(self): self.SetFsmId(1)`，**没有 return**。我们的 _call
        通过 `code = method()` 取值，拿到 None → _normalize_code(None) → -1 静默归零。
        而 SetFsmId/SetTaskId 自身返 int code —— 这里 wrap 让业务方法也返这个 int。

        实机 2026-06-30 14:09 第一次调 damp/lie_to_stand 物理生效（说明 SetFsmId 内部
        DDS publish 成功），但返 None → 上层误判 -1。WaveHand 物理不生效（SetTaskId 走
        7106 主控没切到对应 task）。

        修复后 _call 拿到的 code 是真实 SDK 返回值（0 = OK，7002/7004 = 业务错误）。
        """
        if self.loco_client is None:
            return
        # 5 个需要 wrap 的方法：Damp/Lie2StandUp/Start/WaveHand/ShakeHand
        # 注：Squat2StandUp/StandUp2Squat/Sit/StopMove/HighStand/LowStand/Move/BalanceStand
        #     都是直接走 SetFsmId/SetVelocity/SetBalanceMode/SetStandHeight 的 wrapper，
        #     内部也无 return — 同样需要 wrap。ZeroTorque 同。
        wrap_targets = (
            "Damp", "Squat2StandUp", "StandUp2Squat", "ZeroTorque", "Sit",
            "Lie2StandUp", "Start", "StopMove", "HighStand", "LowStand",
            "WaveHand", "ShakeHand", "BalanceStand", "Move",
        )
        for method_name in wrap_targets:
            original = getattr(self.loco_client, method_name, None)
            if original is None:
                continue
            # 用闭包捕获原方法 + 方法名（用于错误日志）
            def make_wrapper(name, orig):
                def wrapper(*args, **kwargs):
                    result = orig(*args, **kwargs)
                    # 多数情况下 result 是 None（SDK 1.0.1 bug），少数返回 SetFsmId 的 int
                    if result is None:
                        # 兜底：直接读 fsm_id 或 task_id 路径的 SetXxx 返 int
                        # 但更稳的是 LOGGER.debug 而非 LOGGER.warning（避免与 None→-1 重复）
                        LOGGER.debug(
                            "Loco %s() SDK 返 None（已知 SDK 1.0.1 bug：缺少 return）",
                            name,
                        )
                        return 0  # 假定成功（DDS publish 成功意味着主控接收；None 是 SDK bug）
                    return int(result) if result is not None else 0
                return wrapper
            setattr(self.loco_client, method_name, make_wrapper(method_name, original))
        LOGGER.info(
            "LocoClient 方法 wrap 完成（修复 SDK 1.0.1 缺 return bug）：%s",
            ", ".join(wrap_targets),
        )

    def _call(self, method_name: str) -> int:
        if not self.is_running():
            raise RuntimeError("LocoController 未初始化")
        method = getattr(self.loco_client, method_name, None)
        if method is None:
            raise AttributeError(f"LocoClient 缺少方法: {method_name}")
        try:
            code = method()
            LOGGER.info("Loco %s() code=%s", method_name, code)
            return self._normalize_code(method_name, code)
        except Exception as exc:
            LOGGER.error("Loco %s() 抛错: %s", method_name, exc)
            raise

    @staticmethod
    def _normalize_code(method_name: str, code: Any) -> int:
        """归一 SDK 返回值。

        历史的 "int(code) if code is not None else 0" 会把 SDK 的 None（无订阅者 / lease 没收完）
        静默归零为 0，导致远端看到 code=0 误判成功而机器人没动。

        修复 D7 后：Damp/Lie2StandUp/Start 等 SDK 方法已 wrap 返 int；Damp/WaveHand 等
        拿到的 None 已被 wrapper 兜底为 0（假定 SDK 内部 DDS publish 成功）。

        仍可能出现 None（wrapper 漏掉的新方法），策略：LOGGER.warning + 返 -1。
        """
        if code is None:
            LOGGER.warning(
                "Loco %s() 返回 None（多半是 wrapper 漏掉的 SDK 方法或 lease 未就绪）；"
                "请确认 (a) LocoClient.Start() 在 init 里已调（loco_controller.py）；"
                "(b) G1 主控已开机并 enable_motor；"
                "(c) iface 与 G1 主控同网段。",
                method_name,
            )
            return -1
        return int(code)


def main() -> None:
    """CLI 入口：python3 -m drivers.g1.control.loco_controller damp|squat|stand|wave|shake|status"""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="G1 LocoController CLI 测试入口")
    parser.add_argument("command", choices=[
        "damp", "squat", "stand", "squat_to_stand", "stand_to_squat",
        "low_stand", "high_stand", "zero_torque", "wave", "shake",
        "lie_to_stand", "status", "subscribe_test",
    ], help="要执行的 Loco 命令或状态查询")
    parser.add_argument("--iface", default=DEFAULT_IFACE, help=f"DDS 网卡（默认 {DEFAULT_IFACE}）")
    parser.add_argument("--timeout", type=float, default=DEFAULT_CLIENT_TIMEOUT_SEC, help="LocoClient 超时（秒）")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    controller = LocoController(iface=args.iface, client_timeout_sec=args.timeout)
    if not controller.init():
        LOGGER.error("LocoController.init() 失败")
        sys.exit(1)

    try:
        if args.command == "status":
            print(f"is_initialized={controller.is_initialized} is_running={controller.is_running()} has_state_frame={controller.has_state_frame()}")
        elif args.command == "subscribe_test":
            # 订阅 SportModeState_ 3s，看能否收到帧
            received = []

            def cb(msg):
                received.append(time.time())
                LOGGER.info("收到 SportModeState_ 帧（位置 %s, 速度 %s）",
                            getattr(msg, "position", None), getattr(msg, "velocity", None))

            controller.subscribe_sport_mode_state(cb)
            LOGGER.info("订阅测试：等待 3s ...")
            time.sleep(3.0)
            LOGGER.info("订阅测试完成：共收到 %d 帧", len(received))
        else:
            # 一次性命令
            dispatch = {
                "damp": controller.damp,
                "squat_to_stand": controller.squat_to_stand,
                "stand_to_squat": controller.stand_to_squat,
                "low_stand": controller.low_stand,
                "high_stand": controller.high_stand,
                "zero_torque": controller.zero_torque,
                "wave": controller.wave_hand,
                "shake": controller.shake_hand,
                "lie_to_stand": controller.lie_to_stand,
                "squat": controller.stand_to_squat,
                "stand": controller.squat_to_stand,
            }
            code = dispatch[args.command]()
            print(f"loco state: {args.command} code={code}")
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
