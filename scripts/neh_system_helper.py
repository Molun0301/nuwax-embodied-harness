#!/usr/bin/python3 -I
"""端侧管理的固定特权操作；安装后本文件及总线检查器必须由root持有。"""
from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


STATE = Path("/var/lib/neh-system-helper")
CHECKER = Path("/usr/local/libexec/neh-check-lcm-multicast.sh")
# 包名来自现有模块配方；不允许来源参数、包文件、版本表达式或任意包名。
PACKAGES = frozenset({
    "ca-certificates", "curl", "git", "python3", "python3-venv", "python3-dev",
    "python3-pip", "build-essential", "pkg-config", "liblcm-dev", "liblcm-bin",
    "libgl1", "libglib2.0-0", "libglib2.0-0t64", "libglib2.0-dev", "libssl-dev", "cmake",
    "protobuf-compiler", "libpcl-dev", "libeigen3-dev", "libgoogle-glog-dev",
    "libgflags-dev", "libboost-dev", "libboost-serialization-dev",
    "libboost-filesystem-dev", "libboost-thread-dev", "libboost-system-dev",
    "libboost-chrono-dev", "libboost-atomic-dev", "python3-pyaudio",
    "portaudio19-dev", "patch", "patchelf", "iproute2", "util-linux", "openssl",
})
# 顺序对应原系统部署stop_stack，先撤销设置触发器，避免迁入期间重新拉起旧发布。
UNITS = (
    "nuwax-settings-apply.path", "nuwax-settings-apply.service",
    "nuwax-go2-video-streamer.service", "nuwax-host-gateway.service",
    "nuwax-runtime-activation.path", "nuwax-runtime-activation.service",
    "nuwax-person-follow.service", "nuwax-geometry-runtime.service",
    "nuwax-nav-stack.service", "nuwax-motion-safety-guard.service",
    "nuwax-lcm-network.service",
)
ENV = {"PATH": "/usr/sbin:/usr/bin:/sbin:/bin", "LANG": "C.UTF-8",
       "DEBIAN_FRONTEND": "noninteractive"}

# 配方使用稳定逻辑名称，由本机适配为发行版包名；没有映射时明确失败。
_COMMON_RPM = {
    "python3-venv": ("python3",), "python3-dev": ("python3-devel",),
    "build-essential": ("gcc", "gcc-c++", "make"), "pkg-config": ("pkgconf-pkg-config",),
    "liblcm-dev": ("lcm-devel",), "liblcm-bin": ("lcm",), "libgl1": ("libglvnd-glx",),
    "libglib2.0-0": ("glib2",), "libglib2.0-0t64": ("glib2",), "libglib2.0-dev": ("glib2-devel",), "libssl-dev": ("openssl-devel",),
    "libpcl-dev": ("pcl-devel",), "libeigen3-dev": ("eigen3-devel",),
    "libgoogle-glog-dev": ("glog-devel",), "libgflags-dev": ("gflags-devel",),
    "portaudio19-dev": ("portaudio-devel",), "iproute2": ("iproute",),
}
_ARCH = {
    "python3": ("python",), "python3-venv": ("python",), "python3-dev": ("python",),
    "python3-pip": ("python-pip",), "build-essential": ("base-devel",), "pkg-config": ("pkgconf",),
    "liblcm-dev": ("lcm",), "liblcm-bin": ("lcm",), "libgl1": ("libglvnd",),
    "libglib2.0-0": ("glib2",), "libglib2.0-0t64": ("glib2",), "libglib2.0-dev": ("glib2",), "libssl-dev": ("openssl",),
    "libpcl-dev": ("pcl",), "libeigen3-dev": ("eigen",), "libgoogle-glog-dev": ("glog",),
    "libgflags-dev": ("gflags",), "python3-pyaudio": ("python-pyaudio",),
    "portaudio19-dev": ("portaudio",), "protobuf-compiler": ("protobuf",),
}
_ALPINE = {
    "python3-venv": ("python3",), "python3-dev": ("python3-dev",), "python3-pip": ("py3-pip",),
    "build-essential": ("build-base",), "pkg-config": ("pkgconf",), "liblcm-dev": ("lcm-dev",),
    "liblcm-bin": ("lcm",), "libgl1": ("mesa-gl",), "libglib2.0-0": ("glib",),
    "libglib2.0-0t64": ("glib",), "libglib2.0-dev": ("glib-dev",), "libssl-dev": ("openssl-dev",), "libpcl-dev": ("pcl-dev",),
    "libeigen3-dev": ("eigen-dev",), "libgoogle-glog-dev": ("glog-dev",),
    "libgflags-dev": ("gflags-dev",), "python3-pyaudio": ("py3-pyaudio",),
    "portaudio19-dev": ("portaudio-dev",), "protobuf-compiler": ("protobuf",),
}
_UNIVERSAL = frozenset({"ca-certificates", "curl", "git", "python3", "python3-pip", "cmake", "patch", "patchelf", "openssl", "util-linux", "iproute2", "protobuf-compiler", "python3-pyaudio"})


def run(argv, *, check=True, timeout=1800):
    """只执行调用点构造的固定命令，拒绝继承调用者的程序路径与包管理配置。"""
    return subprocess.run(argv, env=ENV, check=check, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)


def body(stream):
    raw = stream.read(65537)
    if len(raw) > 65536:
        raise ValueError("请求过大")
    value = json.loads(raw) if raw.strip() else {}
    if not isinstance(value, dict):
        raise ValueError("请求必须为对象")
    return value


def validate_packages(payload):
    if set(payload) != {"packages"} or not isinstance(payload["packages"], list):
        raise ValueError("系统依赖请求只能包含packages列表")
    values = payload["packages"]
    if len(values) > len(PACKAGES) or any(not isinstance(p, str) or p not in PACKAGES for p in values):
        raise ValueError("请求包含未允许的系统依赖")
    return sorted(set(values))


def package_mapping(manager, packages):
    if manager == "apt-get":
        return list(packages)
    mapping = dict(_COMMON_RPM) if manager in {"dnf", "yum", "zypper"} else dict(_ARCH if manager == "pacman" else _ALPINE)
    if manager == "zypper":
        mapping.update({"libgl1": ("libGL1",), "libglib2.0-0": ("libglib-2_0-0",),
                        "libglib2.0-0t64": ("libglib-2_0-0",), "pkg-config": ("pkg-config",),
                        "iproute2": ("iproute2",)})
    result = []
    for package in packages:
        if package.startswith("libboost-"):
            result.append("boost" if manager == "pacman" else "boost-dev" if manager == "apk" else "boost-devel")
        elif package in mapping:
            result.extend(mapping[package])
        elif package in _UNIVERSAL:
            result.append(package)
        else:
            raise RuntimeError("此发行版尚未提供该依赖的安装映射：" + package)
    return sorted(set(result))


def dependency_plan(payload):
    """无特权、无网络的只读查询，管理服务和特权安装共用。"""
    logical = validate_packages(payload)
    if not logical:
        return {"manager": None, "packages": [], "missing": []}
    manager = next((name for name in ("apt-get", "dnf", "yum", "pacman", "zypper", "apk")
                    if shutil.which(name, path=ENV["PATH"])), None)
    if manager is None:
        raise RuntimeError("此设备尚无匹配的系统依赖适配器；未执行包管理命令")
    packages = package_mapping(manager, logical)
    missing = []
    resolved = []
    for package in packages:
        if manager == "apt-get":
            command = ["dpkg-query", "-W", "-f=${Status}", "--", package]
        elif manager in {"dnf", "yum", "zypper"}:
            command = ["rpm", "-q", "--", package]
        elif manager == "pacman":
            command = ["pacman", "-Q", "--", package]
        else:
            command = ["apk", "info", "-e", "--", package]
        result = run(command, check=False, timeout=30)
        installed = not result.returncode and (manager != "apt-get" or result.stdout.strip() == "install ok installed")
        if manager == "apt-get" and package == "libglib2.0-0" and not installed:
            alias = "libglib2.0-0t64"
            checked = run(["dpkg-query", "-W", "-f=${Status}", "--", alias], check=False, timeout=30)
            alias_installed = not checked.returncode and checked.stdout.strip() == "install ok installed"
            available = alias_installed or "Package: " + alias in run(["apt-cache", "show", "--", alias], check=False, timeout=30).stdout
            if available:
                package, installed = alias, alias_installed
        resolved.append(package)
        if not installed:
            missing.append(package)
    return {"manager": manager, "packages": resolved, "missing": missing}


def install_dependencies(payload):
    plan = dependency_plan(payload)
    missing, manager = plan["missing"], plan["manager"]
    if missing:
        if manager == "apt-get":
            run([manager, "update"])
            # 空镜像可能尚无包索引；刷新后重新解析t64等实际包名。
            missing = dependency_plan(payload)["missing"]
            if not missing:
                return {"installed": [], "message": "系统依赖检查完成"}
            command = [manager, "install", "--yes", "--no-install-recommends", "--"]
        elif manager in {"dnf", "yum"}:
            command = [manager, "install", "--assumeyes", "--"]
        elif manager == "pacman":
            command = [manager, "-S", "--needed", "--noconfirm", "--"]
        elif manager == "zypper":
            command = [manager, "--non-interactive", "install", "--no-recommends", "--"]
        else:
            command = [manager, "add", "--no-cache", "--"]
        run(command + missing)
    return {"installed": missing, "message": "系统依赖检查完成"}


def atomic_json(path, value):
    fd, name = tempfile.mkstemp(prefix=".state-", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def legacy_state():
    units = {}
    for unit in UNITS:
        result = run(["systemctl", "show", unit, "--property=LoadState,ActiveState,UnitFileState"], timeout=30)
        fields = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        if fields.get("LoadState") not in {"not-found", "masked"}:
            units[unit] = {"active": fields.get("ActiveState") in {"active", "activating", "reloading"},
                           "enabled": fields.get("UnitFileState", "disabled")}
    return units


def restore_legacy():
    snapshot = STATE / "legacy.json"
    if not snapshot.is_file():
        return {"restored": False, "message": "没有待恢复的迁入事务"}
    state = json.loads(snapshot.read_text(encoding="utf-8"))
    if not isinstance(state, dict) or set(state) - set(UNITS):
        raise RuntimeError("旧部署恢复记录无效")
    for unit in reversed(UNITS):
        previous = state.get(unit)
        if not previous:
            continue
        enabled = previous.get("enabled")
        if enabled in {"enabled", "enabled-runtime"}:
            command = ["systemctl", "enable"]
            if enabled == "enabled-runtime":
                command.append("--runtime")
            run(command + [unit], timeout=60)
        if previous.get("active"):
            run(["systemctl", "start", unit], timeout=120)
    snapshot.unlink()
    return {"restored": True}


def stop_legacy():
    snapshot = STATE / "legacy.json"
    if snapshot.exists():
        # 恢复中断事务时保留第一次现场事实，不把已经停掉的状态覆盖进去。
        state = json.loads(snapshot.read_text(encoding="utf-8"))
    else:
        state = legacy_state()
        atomic_json(snapshot, state)
    for unit in UNITS:
        if unit not in state:
            continue
        run(["systemctl", "stop", unit], timeout=120)
        enabled = state[unit].get("enabled")
        if enabled in {"enabled", "enabled-runtime"}:
            command = ["systemctl", "disable"]
            if enabled == "enabled-runtime":
                command.append("--runtime")
            run(command + [unit], timeout=60)
    return {"stopped": list(state), "recoverable": True}


def dispatch(action, payload):
    if action == "install-dependencies":
        return install_dependencies(payload)
    if action == "prepare-network":
        if set(payload) != {"url"} or not isinstance(payload["url"], str) or len(payload["url"]) > 1024:
            raise ValueError("网络准备请求只能包含url字符串")
        info = CHECKER.stat()
        if info.st_uid != 0 or info.st_mode & 0o022 or CHECKER.is_symlink():
            raise RuntimeError("网络检查器必须是root持有且普通用户不可写的文件")
        result = run(["/bin/bash", str(CHECKER), "--prepare", payload["url"]], timeout=60)
        return {"prepared": True, "message": result.stdout.strip()}
    if payload:
        raise ValueError("此操作不接受额外参数")
    if action == "restart-manager":
        value = os.environ.get("SUDO_UID", "")
        if not value.isdigit() or int(value) <= 0:
            raise ValueError("管理服务重启必须由已授权的普通用户经sudo调用")
        # 用户编号由sudo自身提供，不接受请求指定其它用户的服务。
        unit = "neh-manager-%s.service" % int(value)
        run(["systemctl", "--no-block", "restart", unit], timeout=30)
        return {"scheduled": True}
    if action == "stop-legacy":
        return stop_legacy()
    if action == "restore-legacy":
        return restore_legacy()
    if action == "commit-legacy":
        path = STATE / "legacy.json"
        if path.exists():
            path.unlink()
        return {"committed": True}
    raise ValueError("不支持的特权操作")


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    try:
        if len(args) != 1:
            raise ValueError("必须指定一个固定特权操作")
        if os.geteuid() != 0:
            raise PermissionError("请通过已安装的固定特权入口调用")
        payload = body(sys.stdin)
        STATE.mkdir(mode=0o700, parents=True, exist_ok=True)
        if STATE.is_symlink() or STATE.stat().st_uid != 0:
            raise RuntimeError("特权状态目录不可信")
        STATE.chmod(0o700)
        with (STATE / "operation.lock").open("a") as handle:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = dispatch(args[0], payload)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        # 不把完整进程输入或调用者环境打印到日志。
        message = "系统依赖或服务操作失败"
        if isinstance(exc, subprocess.CalledProcessError):
            message += ": " + (exc.stdout or "").strip()[-3000:]
        else:
            message = str(exc)
        print(json.dumps({"error": message}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
