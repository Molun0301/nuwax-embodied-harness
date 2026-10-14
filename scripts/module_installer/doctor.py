"""只读检查安装前提和已有产物，不安装软件、不启动或连接机器人。"""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

from .manager import InstallError, _verify_prefix, check_prerequisites


def diagnose(request, hardware, current: dict) -> dict:
    checks = []

    def record(name: str, status: str, message: str) -> None:
        checks.append({"name": name, "status": status, "message": message})

    try:
        python = check_prerequisites(request, hardware)
        record("prerequisites", "ok", "系统、账号、安装位置与所选设备前提通过；解释器：" + python)
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        record("prerequisites", "error", str(exc))

    available = hardware.free_disk_bytes / 1024 ** 3
    record("disk", "warning" if available < 4 else "ok",
           f"安装磁盘可用 {available:.1f} GiB（吉字节）；原生编译和模型下载需额外空间，4 GiB 仅为基础提醒。")
    if not shutil.which("apt-get") or not shutil.which("dpkg-query"):
        record("packages", "error", "找不到 apt-get/dpkg-query（系统软件包工具），请使用完整的 Debian 或 Ubuntu 环境。")
    else:
        from .catalog import module_system_packages

        missing = []
        for package in module_system_packages(request.modules, request.robot):
            names = (package, "libglib2.0-0t64") if package == "libglib2.0-0" else (package,)
            installed = False
            for name in names:
                result = subprocess.run(["dpkg-query", "-W", "-f=${Status}", name],
                                        capture_output=True, text=True, timeout=10, check=False)
                if result.returncode == 0 and result.stdout.strip() == "install ok installed":
                    installed = True
                    break
            if not installed:
                missing.append(package)
        if missing:
            status = "warning" if shutil.which("sudo") else "error"
            record("packages", status, "安装时需要 sudo（管理员提权）补齐：" + ", ".join(missing))
        else:
            record("packages", "ok", "所选模块的系统软件包已安装。")

    if current:
        if current.get("source_root") != str(request.root):
            record("source", "error", "当前安装属于其它源码目录，请在原目录操作或另选 --install-dir。")
        try:
            _verify_prefix(Path(current["prefix"]), current["modules"])
            if not os.access(Path(current["prefix"]) / "venv/bin/python", os.X_OK):
                raise InstallError("当前安装的 Python 解释器不可执行。")
            record("installation", "ok", "当前安装的必要文件完整；功能就绪仍需启动后检查。")
        except (OSError, RuntimeError) as exc:
            record("installation", "error", str(exc) + "；停止网关后运行 install --repair 修复。")
    else:
        record("installation", "warning", "尚未安装；完成安装后再运行本检查核对产物。")
    record("network", "warning", "未联网探测下载源；安装需要系统软件源、Python包源及所选源码/模型源可达。")
    return {"ok": not any(item["status"] == "error" for item in checks), "checks": checks}
