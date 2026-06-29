#!/usr/bin/env python3
"""按 NUWAX_ACTIVE_ROBOT 准备端侧 SDK 视频后端（网关启动前调用）。

G1：重启本机 videohub_pc4，并通过 SDK 确认 DDS videohub 可响应。
Go2：videohub 在机载端，仅做 SDK 连通性探测（需正确 DDS 网卡）。
"""
from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

LOGGER = logging.getLogger("nuwax_robot_bridge.ensure_video_backend")

G1_VIDEHUB_BIN = Path("/unitree/module/video_hub_pc4/videohub_pc4")
G1_VIDEHUB_CYCLONEDDS = "/unitree/module/video_hub_pc4/cyclonedds.xml"
G1_CAMERA_CONFIG = Path("/unitree/etc/rgb_camera/CAMAERA_CONFIG")
G1_MSCLI_BIN = Path("/unitree/sbin/mscli")
G1_VIDEHUB_SERVICE_NAME = "video_hub_pc4"
DEFAULT_GEOMETRY_CYCLONEDDS = str(Path.home() / "cyclonedds_ws" / "cyclonedds.xml")


def _active_robot() -> str:
    return str(os.environ.get("NUWAX_ACTIVE_ROBOT", "") or "").strip().lower()


def _sudo_password() -> str:
    return str(os.environ.get("UNITREE_SUDO_PASSWORD", "123") or "123")


def _run_sudo_shell(script: str) -> bool:
    password = _sudo_password()
    try:
        proc = subprocess.run(
            ["sudo", "-S", "bash", "-c", script],
            input=f"{password}\n",
            text=True,
            capture_output=True,
            timeout=30,
        )
        if proc.returncode != 0:
            LOGGER.warning("sudo 命令失败: %s", (proc.stderr or proc.stdout or "").strip())
            return False
        return True
    except (subprocess.TimeoutExpired, OSError) as exc:
        LOGGER.warning("sudo 执行异常: %s", exc)
        return False


def _camera_devices_present() -> bool:
    return any(Path(f"/dev/video{i}").exists() for i in range(8)) or Path("/dev/media0").exists()


def _run_mscli(*args: str, timeout_sec: float = 10.0) -> tuple[int, str]:
    """直接调 unitree master_service 自带的 mscli（非 sudo）。"""
    if not G1_MSCLI_BIN.is_file():
        return 127, ""
    try:
        proc = subprocess.run(
            [str(G1_MSCLI_BIN), *args],
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
        return proc.returncode, (proc.stdout or "")
    except (subprocess.TimeoutExpired, OSError) as exc:
        LOGGER.debug("mscli 执行异常: %s", exc)
        return 126, ""


def _g1_videohub_alive() -> bool:
    """通过 mscli getservice 查询 videohub 状态。stdout 含 "status:0" 视为 running。"""
    rc, stdout = _run_mscli("getservice", G1_VIDEHUB_SERVICE_NAME)
    if rc != 0:
        return False
    return "status:0" in stdout


def _g1_kill_stale_videohub_via_mscli() -> None:
    """让 master_service 停掉当前 fork 出来的 videohub（如果有）。"""
    rc, _ = _run_mscli("stopservice", G1_VIDEHUB_SERVICE_NAME, timeout_sec=5.0)
    LOGGER.debug("mscli stopservice 返回 rc=%s", rc)


def _g1_kill_stale_videohub_via_pidfile() -> None:
    """兜底：杀 start-stop-daemon 历史路径留下的孤儿进程。"""
    script = (
        "/unitree/sbin/start-stop-daemon "
        "--stop --pidfile=/unitree/var/run/videohub_pc4.pid "
        f"--exec {G1_VIDEHUB_BIN} 2>/dev/null || true"
    )
    _run_sudo_shell(script)


def _g1_init_camera_config() -> None:
    """确保 /unitree/etc/rgb_camera/CAMAERA_CONFIG 存在（unitree 上游拼写是事实）。"""
    script = f"""
mkdir -p {G1_CAMERA_CONFIG.parent}
if [[ ! -f {G1_CAMERA_CONFIG} ]]; then
  printf '%s\\n' '{{' '  "Backlight Compensation": 0,' '  "Exposure": 0,' '  "Gain": 0' '}}' > {G1_CAMERA_CONFIG}
fi
"""
    _run_sudo_shell(script)


def ensure_g1_videohub_process() -> bool:
    """G1 端启动 videohub_pc4 并等待 ready。

    走 master_service 自带的 mscli 控制通道（而非 start-stop-daemon），
    让 videohub 进程纳入 master_service 的 fork 树：
    - 进程崩溃后 master_service 5 秒内自动重启（与 ota_pipe 同守护等级）
    - start-stop-daemon 路径会派生 PPid=1 的孤儿进程，master_service 看不到，
      进程崩了不会自动重启，是 2026-06-29 实机视频恢复不稳的根因。

    Returns:
        True 表示 mscli 通道可用且 videohub 进入 status:0；False 表示退化路径。
    """
    if not G1_VIDEHUB_BIN.is_file():
        LOGGER.info("未找到 %s，跳过 G1 videohub 重启。", G1_VIDEHUB_BIN)
        return False
    if not G1_MSCLI_BIN.is_file():
        LOGGER.warning(
            "未找到 %s，无法通过 master_service 通道拉起 videohub；"
            "退化到 streamer 重试链路。",
            G1_MSCLI_BIN,
        )
        return False
    if not _camera_devices_present():
        LOGGER.warning(
            "未检测到 /dev/video* 或 /dev/media0；请确认 RealSense/相机已连接后再启动网关。"
        )

    # 1) 前置：CAMAERA_CONFIG 初始化（master_service 启 videohub 前要读到该文件）
    _g1_init_camera_config()

    # 2) 双清残留：先让 master_service 停自己 fork 的，再用 pidfile 杀孤儿
    _g1_kill_stale_videohub_via_mscli()
    _g1_kill_stale_videohub_via_pidfile()

    # 3) 让 master_service fork 新进程
    rc, _ = _run_mscli("startservice", G1_VIDEHUB_SERVICE_NAME, timeout_sec=10.0)
    if rc != 0:
        LOGGER.warning("mscli startservice 返回 rc=%s", rc)
        return False

    # 4) 轮询直到 status:0，最多 10 秒
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if _g1_videohub_alive():
            LOGGER.info("已通过 master_service (mscli) 拉起 videohub_pc4。")
            return True
        time.sleep(0.5)

    LOGGER.warning("mscli startservice 后 videohub_pc4 10s 内未就绪。")
    return False


def _prepare_g1_sdk_env() -> None:
    from drivers.g1.settings import load_g1_data_plane_config
    from drivers.g1 import video_streamer as g1_vs

    config = load_g1_data_plane_config()
    g1_vs._prepare_g1_sdk_environment(config)


def _prepare_go2_sdk_env() -> None:
    from drivers.go2.settings import load_go2_data_plane_config
    from drivers.go2 import video_streamer as go2_vs

    config = load_go2_data_plane_config()
    go2_vs._prepare_go2_sdk_environment(config)


def _sdk_probe_first_frame(robot: str, *, timeout_sec: float) -> bool:
    if robot == "g1":
        _prepare_g1_sdk_env()
        from drivers.g1.settings import load_g1_data_plane_config
        from drivers.g1 import video_streamer as g1_vs

        config = load_g1_data_plane_config()
        deadline = time.monotonic() + max(1.0, timeout_sec)
        last_code = None
        while time.monotonic() < deadline:
            try:
                client = g1_vs._create_video_client(config)
                code, data = client.GetImageSample()
                last_code = code
                if code == 0 and data and g1_vs._extract_jpeg_payload(data):
                    LOGGER.info("G1 SDK VideoClient 探测成功。")
                    return True
            except Exception as exc:
                LOGGER.debug("G1 SDK 探测重试: %s", exc)
            time.sleep(0.5)
        LOGGER.warning("G1 SDK 在 %.0fs 内未取到首帧（最后 code=%s）。", timeout_sec, last_code)
        return False

    if robot == "go2":
        _prepare_go2_sdk_env()
        from drivers.go2.settings import load_go2_data_plane_config
        from drivers.go2 import video_streamer as go2_vs

        config = load_go2_data_plane_config()
        deadline = time.monotonic() + max(1.0, timeout_sec)
        last_code = None
        while time.monotonic() < deadline:
            try:
                client = go2_vs._create_video_client(config)
                code, data = client.GetImageSample()
                last_code = code
                if code == 0 and data and go2_vs._extract_jpeg_payload(data):
                    LOGGER.info("Go2 SDK VideoClient 探测成功。")
                    return True
            except Exception as exc:
                LOGGER.debug("Go2 SDK 探测重试: %s", exc)
            time.sleep(0.5)
        LOGGER.warning("Go2 SDK 在 %.0fs 内未取到首帧（最后 code=%s）。", timeout_sec, last_code)
        return False

    LOGGER.info("机器人 %s 无端侧 SDK 视频后端准备逻辑。", robot)
    return True


def _g1_enable_videohub_service() -> None:
    try:
        _prepare_g1_sdk_env()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
        from drivers.g1.settings import load_g1_data_plane_config

        config = load_g1_data_plane_config()
        iface = str(config.dds_iface or "").strip()
        if iface:
            ChannelFactoryInitialize(0, iface)
        else:
            ChannelFactoryInitialize(0)
        rsc = RobotStateClient()
        rsc.SetTimeout(5.0)
        rsc.Init()
        code = rsc.ServiceSwitch("video_hub_pc4", True)
        if code == 0:
            LOGGER.info("已通过 SDK 启用 video_hub_pc4 服务。")
        else:
            LOGGER.debug("ServiceSwitch video_hub_pc4 返回 code=%s（可能已在运行）。", code)
    except Exception as exc:
        LOGGER.debug("ServiceSwitch 跳过: %s", exc)


def ensure_for_robot(robot: str, *, timeout_sec: float, skip_probe: bool) -> int:
    robot = (robot or "").strip().lower()
    if not robot:
        LOGGER.error("未设置 NUWAX_ACTIVE_ROBOT。")
        return 1

    if robot == "g1":
        ensure_g1_videohub_process()
        _g1_enable_videohub_service()
    elif robot == "go2":
        LOGGER.info("Go2 视频走机载 videohub，跳过本机 videohub 重启。")
    else:
        LOGGER.info("机器人 %s 无端侧 SDK 视频准备步骤。", robot)
        return 0

    if skip_probe:
        return 0
    if _sdk_probe_first_frame(robot, timeout_sec=timeout_sec):
        return 0
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="准备机器人 SDK 视频后端")
    parser.add_argument("--robot", default=_active_robot(), help="机器人名称 g1/go2/...")
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=float(os.environ.get("NUWAX_VIDEO_BACKEND_READY_TIMEOUT_SEC", "30")),
    )
    parser.add_argument("--skip-probe", action="store_true", help="仅准备进程，不做 SDK 取帧探测")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s:%(name)s:%(message)s",
    )
    return ensure_for_robot(args.robot, timeout_sec=args.timeout_sec, skip_probe=args.skip_probe)


if __name__ == "__main__":
    raise SystemExit(main())