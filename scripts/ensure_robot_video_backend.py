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


def ensure_g1_videohub_process() -> bool:
    if not G1_VIDEHUB_BIN.is_file():
        LOGGER.info("未找到 %s，跳过 G1 videohub 重启。", G1_VIDEHUB_BIN)
        return False
    if not _camera_devices_present():
        LOGGER.warning(
            "未检测到 /dev/video* 或 /dev/media0；请确认 RealSense/相机已连接后再启动网关。"
        )
    script = f"""
mkdir -p {G1_CAMERA_CONFIG.parent}
if [[ ! -f {G1_CAMERA_CONFIG} ]]; then
  printf '%s\\n' '{{' '  "Backlight Compensation": 0,' '  "Exposure": 0,' '  "Gain": 0' '}}' > {G1_CAMERA_CONFIG}
fi
/unitree/sbin/start-stop-daemon --stop --pidfile=/unitree/var/run/videohub_pc4.pid --exec {G1_VIDEHUB_BIN} 2>/dev/null || true
sleep 1
export CYCLONEDDS_URI={G1_VIDEHUB_CYCLONEDDS}
/unitree/sbin/start-stop-daemon --start --background --make-pidfile --pidfile=/unitree/var/run/videohub_pc4.pid --exec {G1_VIDEHUB_BIN}
"""
    if not _run_sudo_shell(script):
        return False
    time.sleep(2)
    return True


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