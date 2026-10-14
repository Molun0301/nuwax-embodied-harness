#!/usr/bin/env python3
"""只读取本机总线地址，并在干净环境中准备或校验对应路由。"""
from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Optional


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=0"
LEGACY_DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=1"
DEFAULT_GEOMETRY_ENV = Path("/var/lib/nuwax_robot_bridge/geometry-owner.env")
DEFAULT_POINTCLOUD_ENV = Path("/var/lib/nuwax/gateway/pointcloud3d.env")
DEFAULT_USER_ENV = Path("/var/lib/nuwax_robot_bridge/settings.env")


class _CheckerCancelled(BaseException):
    """将命令行取消送入拥有方清理，不把它误当作检查器正常退出。"""

    def __init__(self, signum: int):
        self.signum = signum


def _run_checker(command, *, env, capture_output=False, timeout=30):
    """受管运行整个检查器进程组，超时与可捕获取消均先回收再返回。"""
    process = None
    pending_signal = None
    previous_handlers = {}

    def cancel(signum, _frame):
        nonlocal pending_signal
        # 信号只记状态，创建和清理均不能被异步异常打断。
        if pending_signal is None:
            pending_signal = signum

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, cancel)
        if pending_signal is not None:
            raise _CheckerCancelled(pending_signal)
        process = subprocess.Popen(
            command, env=env, stdout=subprocess.PIPE if capture_output else None,
            text=True, start_new_session=True,
        )
        deadline = time.monotonic() + timeout
        while True:
            if pending_signal is not None:
                raise _CheckerCancelled(pending_signal)
            try:
                stdout, _ = process.communicate(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
            except subprocess.TimeoutExpired as exc:
                if time.monotonic() >= deadline:
                    raise subprocess.TimeoutExpired(command, timeout, output=exc.output, stderr=exc.stderr)
                continue
            if pending_signal is not None:
                raise _CheckerCancelled(pending_signal)
            return subprocess.CompletedProcess(command, process.returncode, stdout)
    except BaseException:
        if process is not None:
            # 重复取消不得抢先结束拥有方清理；检查器无须在失败后继续执行网络命令。
            for signum in previous_handlers:
                signal.signal(signum, signal.SIG_IGN)
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.communicate()
        raise
    finally:
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)


def migrated_root_lcm_url(value: Optional[str]) -> str:
    """只迁移根文件的缺省和已声明旧默认；用户覆盖文件不经过迁移。"""
    value = (value or "").strip()
    return DEFAULT_LCM_URL if not value or value == LEGACY_DEFAULT_LCM_URL else value


def _configuration_parser():
    """按文件导入纯标准库解析器，避免触发core包的应用依赖。"""
    spec = importlib.util.spec_from_file_location(
        "_nuwax_lcm_config_values", REPO_ROOT / "core" / "config_values.py"
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("无法加载共同配置解析器")
    values_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(values_module)
    return values_module


def lcm_url_from_env_text(source: str) -> Optional[str]:
    """根环境迁移沿共同语法读取引号、注释及最后一个同名条目。"""
    parse_line = _configuration_parser().parse_env_line
    value = None
    for line in source.splitlines():
        parsed = parse_line(line)
        if parsed is not None and parsed[0] == "NUWAX_LCM_URL":
            value = parsed[1]
    return value


def resolve_lcm_url(
    config_dir: Path,
    *,
    go2_env: Optional[Path] = None,
    geometry_env: Path = DEFAULT_GEOMETRY_ENV,
    pointcloud_env: Path = DEFAULT_POINTCLOUD_ENV,
    user_env: Path = DEFAULT_USER_ENV,
) -> str:
    """按网关服务文件顺序解析地址，每次新读，不执行文件内容或继承业务环境。"""
    values_module = _configuration_parser()
    value = DEFAULT_LCM_URL
    active = REPO_ROOT / "runtime_data/settings/active.env"
    if active.is_symlink():
        location = values_module.configuration_location(root=REPO_ROOT, environment={})
        if location is None or not location["path"].is_file():
            raise ValueError("活动配置方案不存在")
        paths = (location["path"],)
    else:
        paths = (config_dir / "nuwax.env",
                 go2_env if go2_env is not None else config_dir / "go2-data-plane.env",
                 geometry_env, pointcloud_env, user_env)
    for path in paths:
        values = values_module.read_env_values(Path(path))
        if "NUWAX_LCM_URL" in values:
            value = values["NUWAX_LCM_URL"].strip() or DEFAULT_LCM_URL
    return value


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=Path("/etc/nuwax"))
    parser.add_argument("--go2-env", type=Path)
    parser.add_argument("--geometry-env", type=Path, default=DEFAULT_GEOMETRY_ENV)
    parser.add_argument("--pointcloud-env", type=Path, default=DEFAULT_POINTCLOUD_ENV)
    parser.add_argument("--user-env", type=Path, default=DEFAULT_USER_ENV)
    parser.add_argument("--print-url", action="store_true")
    args = parser.parse_args(argv)
    try:
        url = resolve_lcm_url(
            args.config_dir, go2_env=args.go2_env, geometry_env=args.geometry_env,
            pointcloud_env=args.pointcloud_env, user_env=args.user_env,
        )
        # 配置中的PATH、BASH_ENV、LD_PRELOAD等不进入特权命令。
        environment = {
            "PATH": "/usr/sbin:/usr/bin:/sbin:/bin",
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
        }
        result = _run_checker(
            ["/bin/bash", str(REPO_ROOT / "scripts/systemd/check_lcm_multicast.sh"),
             "--validate-config" if args.print_url else "--prepare", url],
            env=environment, capture_output=args.print_url,
        )
        if result.returncode:
            return result.returncode
        if args.print_url:
            print(url)
        return 0
    except _CheckerCancelled as exc:
        print("本机总线配置准备已取消", file=sys.stderr)
        return 128 + exc.signum
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"本机总线配置准备失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
