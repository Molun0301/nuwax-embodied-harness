"""在独立环境完成目标安装，再原子切换；失败保留上一有效安装。"""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence

from scripts.deploy_installer.config import atomic_write, fsync_directory

from .builds import build_modules, describe_builds, prune_build_cache
from .catalog import module_imports, module_requirements, module_system_packages, resolve_modules
from .runtime import read_existing, read_settings, write_runtime


class InstallError(RuntimeError):
    """向使用者展示明确的安装失败原因。"""


@dataclass(frozen=True)
class InstallRequest:
    root: Path
    home: Path
    modules: tuple
    robot: str
    jobs: int
    python: str = ""
    wheel_dir: Optional[Path] = None
    repair: bool = False


def read_current(home: Path) -> dict:
    current = home / "current"
    if not current.is_symlink():
        if current.exists():
            raise InstallError("安装入口 current 不是受管链接，拒绝覆盖。")
        return {}
    prefix = current.resolve()
    if prefix.parent != (home / "generations").resolve():
        raise InstallError("安装状态指向受管目录以外，拒绝继续。")
    try:
        state = json.loads((prefix / "manifest.json").read_text(encoding="utf-8"))
        if not isinstance(state, dict):
            raise TypeError("安装记录必须是对象")
        if state.get("schema") != 1 or state.get("complete") is not True:
            raise ValueError("未完成的安装记录")
        if tuple(state["modules"]) != resolve_modules(state["modules"]):
            raise ValueError("模块依赖记录不完整")
        if state["robot"] not in ("customrobot", "go2"):
            raise ValueError("未知机器人装配")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise InstallError(f"安装状态损坏：{exc}；请检查 {prefix}") from exc
    state["prefix"] = str(prefix)
    return state


def existing_config(root: Path, current: Mapping) -> dict:
    if not current:
        return read_existing(root)
    return read_settings(Path(current["prefix"]) / "settings.env")


def choose_python(requested: str = "") -> str:
    candidates = [requested] if requested else [sys.executable, "/usr/bin/python3"] + [
        shutil.which(f"python3.{minor}") or "" for minor in (12, 11, 10, 9, 8)
    ]
    for candidate in dict.fromkeys(candidates):
        if not candidate:
            continue
        try:
            result = subprocess.run(
                [candidate, "-c", "import sys; sys.exit(not ((3,8) <= sys.version_info[:2] <= (3,12)))"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10, check=False,
            )
            if result.returncode == 0:
                return os.path.abspath(shutil.which(candidate) or candidate)
        except (OSError, subprocess.SubprocessError):
            continue
    raise InstallError("当前依赖支持 Python（编程语言）3.8—3.12；请安装其中一种，再用 --python 指定解释器。")


def source_fingerprint(root: Path, modules: Sequence[str], robot: str) -> str:
    """纳入实际源码与配方，避免同一提交里的修改被误判为无需更新。"""
    digest = hashlib.sha256()
    try:
        result = subprocess.run(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=str(root), capture_output=True, check=False,
        )
        paths = [root / os.fsdecode(name) for name in result.stdout.split(b"\0") if name] if result.returncode == 0 else []
    except OSError:
        paths = []
    if not paths:
        # 下载源码压缩包的用户未必装有 git；排除数据和构建目录后直接读源码。
        excluded = {".git", ".venv", "target", "result", "build", "__pycache__", "runtime_data", "docs", "artifacts", "tests"}
        for directory, subdirs, files in os.walk(root):
            subdirs[:] = [name for name in subdirs if name not in excluded and not name.startswith(".")]
            paths.extend(Path(directory) / name for name in files if not name.startswith("."))
    # 新增安装器文件在提交之前同样参与身份。
    for directory in (root / "scripts/module_installer", root / "requirements/modules"):
        paths.extend(directory.glob("*.py"))
        paths.extend(directory.glob("*.in"))
    for path in sorted(set(paths)):
        if path.is_file() and not path.is_symlink():
            digest.update(str(path.relative_to(root)).encode())
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
    digest.update(json.dumps([list(modules), robot]).encode())
    return digest.hexdigest()


def make_plan(request: InstallRequest, hardware, current: Mapping) -> dict:
    selected = resolve_modules(request.modules)
    old = set(current.get("modules", []))
    return {
        "modules": list(selected), "robot": request.robot,
        "add": sorted(set(selected) - old), "remove": sorted(old - set(selected)),
        "requirements": list(module_requirements(selected, request.robot)),
        "system_packages": list(module_system_packages(selected, request.robot)),
        "builds": list(describe_builds(selected)), "jobs": request.jobs,
        "install_dir": str(request.home),
        "models": [name for name in ("perception", "memory") if name in selected],
        "jetson_vendor_python": hardware.is_jetson and bool(set(selected) & {"perception", "memory"}),
    }


class CommandRunner:
    def __init__(self, log: Path, root: Path) -> None:
        self.log = log
        self.root = root

    def __call__(self, argv, *, cwd=None, env=None) -> None:
        merged = dict(os.environ)
        # 使用自己的环境，避免当前开发虚拟环境污染编译和依赖探测。
        merged.pop("PYTHONPATH", None)
        merged.pop("PYTHONHOME", None)
        merged.update({"PIP_DISABLE_PIP_VERSION_CHECK": "1", "PYTHONUNBUFFERED": "1", "PYTHONNOUSERSITE": "1"})
        if env:
            merged.update(env)
        with self.log.open("a", encoding="utf-8") as output:
            output.write("\n$ " + shlex.join([str(item) for item in argv]) + "\n")
            output.flush()
            process = subprocess.Popen(
                [str(item) for item in argv], cwd=str(cwd or self.root), env=merged,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
            )
            try:
                while True:
                    try:
                        code = process.wait(timeout=20)
                        break
                    except subprocess.TimeoutExpired:
                        print(f"  仍在处理，详细进度：{self.log}", flush=True)
            except BaseException:
                # 构建器会派生编译器；取消必须覆盖整个进程组再清理安装目录。
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
                raise
        if code:
            raise InstallError(f"{argv[0]} 执行失败（退出码 {code}）。详细日志：{self.log}")


@contextmanager
def installation_lock(home: Path):
    home.mkdir(parents=True, exist_ok=True)
    with (home / "install.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise InstallError("另一个安装任务正在执行，请等它完成后重试。") from exc
        yield


@contextmanager
def idle_gateway(root: Path):
    """与真实启动器共用锁，整个安装过程禁止旧环境开始运行。"""
    path = root / "runtime_data/gateway/start_gateway.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise InstallError("网关正在运行，请先执行 ./install.sh stop，再修改安装模块。") from exc
        yield


def _system_dependencies(packages: Sequence[str], run) -> None:
    missing = []
    for package in packages:
        status = subprocess.run(
            ["dpkg-query", "-W", "-f=${Status}", package], capture_output=True, text=True, check=False,
        )
        if package == "libglib2.0-0" and (status.returncode or status.stdout.strip() != "install ok installed"):
            # 新版发行版由时间类型迁移后的包提供同一运行库。
            status = subprocess.run(
                ["dpkg-query", "-W", "-f=${Status}", "libglib2.0-0t64"], capture_output=True, text=True, check=False,
            )
        if status.returncode or status.stdout.strip() != "install ok installed":
            missing.append(package)
    if missing:
        prefix = [] if os.geteuid() == 0 else ["sudo", "--"]
        if prefix:
            if not shutil.which("sudo"):
                raise InstallError("缺少系统依赖且找不到 sudo（提权工具）：" + ", ".join(missing))
            # 密码提示直接连接终端，不隐藏到安装日志。
            subprocess.run(["sudo", "-v"], check=True)
        run(prefix + ["apt-get", "update"])
        run(prefix + ["apt-get", "install", "-y", "--no-install-recommends"] + missing)


def _probe_imports(python: str, names: Sequence[str]) -> bool:
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment["PYTHONNOUSERSITE"] = "1"
    result = subprocess.run(
        [python, "-c", "import importlib; [importlib.import_module(n) for n in " + repr(list(names)) + "]"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=45, check=False, env=environment,
    )
    return result.returncode == 0


def _vendor_constraints(python: str, target: Path, names: Sequence[str]) -> None:
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment["PYTHONNOUSERSITE"] = "1"
    code = f"from importlib.metadata import version; print('\\n'.join(n+'=='+version(n) for n in {list(names)!r}))"
    result = subprocess.run([python, "-c", code], capture_output=True, text=True, check=True, timeout=45, env=environment)
    atomic_write(target, result.stdout, mode=0o600)


def _python_environment(request: InstallRequest, hardware, prefix: Path, seed: str, run) -> None:
    models = bool(set(request.modules) & {"perception", "memory"})
    vendor = hardware.is_jetson and models
    device_system = request.robot == "go2" and _probe_imports(seed, ("unitree_sdk2py", "cyclonedds"))
    venv_args = [seed, "-m", "venv"]
    if vendor or device_system:
        venv_args.append("--system-site-packages")
    run(venv_args + [str(prefix / "venv")])
    python = str(prefix / "venv/bin/python")
    pip = [python, "-m", "pip", "install"]
    run(pip + ["--upgrade", "pip", "setuptools", "wheel"])
    constraints = prefix / "constraints.txt"
    if models:
        if not vendor:
            torch_args = ["torch", "torchvision"]
            if not hardware.nvidia_gpu:
                torch_args += ["--index-url", "https://download.pytorch.org/whl/cpu"]
            run(pip + torch_args)
        _vendor_constraints(python, constraints, ("torch", "torchvision"))
        pip += ["--constraint", str(constraints)]
    if request.robot == "go2" and not device_system:
        cache = request.home / "cache/device-wheels"
        cache.mkdir(parents=True, exist_ok=True)
        for distribution in ("cyclonedds", "unitree_sdk2py"):
            wheels = sorted(request.wheel_dir.glob(distribution + "-*.whl")) if request.wheel_dir else []
            if len(wheels) != 1:
                raise InstallError(f"Go2 需要唯一的 {distribution} 安装包，请用 --wheel-dir 指定设备软件包目录。")
            cached = cache / wheels[0].name
            if cached.resolve() != wheels[0].resolve():
                shutil.copyfile(wheels[0], cached)
            run(pip + ["--no-deps", str(cached)])
    requirements = module_requirements(request.modules, request.robot)
    arguments = []
    for requirement in requirements:
        arguments += ["-r", str(request.root / requirement)]
    run(pip + arguments)
    run([python, "-m", "pip", "check"])
    run([python, "-c", "import importlib; [importlib.import_module(n) for n in " + repr(list(module_imports(request.modules, request.robot))) + "]"])


def _prepare_models(request: InstallRequest, prefix: Path, run) -> None:
    python = str(prefix / "venv/bin/python")
    output = request.root / "runtime_data/models"
    values = read_settings(prefix / "settings.env")
    if "perception" in request.modules:
        configured = Path(values["NUWAX_PERCEPTION_YOLO_WORLD_MODEL"])
        target = configured if configured.is_absolute() else request.root / configured
        if not target.is_file():
            if target.resolve() != (output / "yolov8s-worldv2.pt").resolve():
                raise InstallError(f"自定义视觉模型不存在：{target}；请准备该文件或在配置中使用默认模型。")
            run([python, "scripts/prepare_yolo_world_tensorrt.py", "--weights-output",
                 str(target), "--download-only"])
    if "memory" in request.modules:
        arguments = []
        for kind, directory in (("text", "BAAI__bge-m3"), ("image", "openai__clip-vit-base-patch32")):
            configured = Path(values[f"NUWAX_MEMORY_{kind.upper()}_EMBEDDING_MODEL"])
            target = configured if configured.is_absolute() else request.root / configured
            if target.resolve() != (output / directory).resolve():
                if not target.is_dir():
                    raise InstallError(f"自定义{kind}记忆模型目录不存在：{target}；安装器不会为它下载另一个默认模型。")
                arguments.append("--skip-" + kind)
        if len(arguments) < 2:
            run([python, "scripts/prepare_memory_models.py", "--output-root", str(output)] + arguments)


def _verify_prefix(prefix: Path, modules: Sequence[str]) -> None:
    needed = ["venv/bin/python", "start.sh", "settings.env"]
    if "geometry" in modules:
        needed += ["bin/nuwax-geometry-runtime", "bin/nuwax-mapping-cli"]
    if "navigation" in modules:
        needed += ["bin/nuwax-nav-stack"]
    if "follow" in modules:
        needed += ["bin/nuwax-person-follow"]
    needed += [f"bin/{name}_native" for name in ("fastlio2", "pointlio") if name in modules]
    missing = [name for name in needed if not (prefix / name).is_file()]
    if missing:
        raise InstallError("安装产物不完整：" + ", ".join(missing))


def apply(request: InstallRequest, hardware) -> dict:
    if sys.platform != "linux" or hardware.os_id not in ("ubuntu", "debian", "raspbian", "linuxmint"):
        raise InstallError("当前自动安装支持 Debian（操作系统）及 Ubuntu（操作系统）系列；其它系统可以查看 --dry-run 计划。")
    if request.jobs < 1:
        raise InstallError("编译并行数必须大于零。")
    if os.geteuid() == 0:
        raise InstallError("请用普通用户运行 ./install.sh；只有缺少的系统依赖由安装器自动提权安装。")
    if request.robot not in ("customrobot", "go2"):
        raise InstallError("请选择标准机器人接入 customrobot 或 Go2。")
    seed = choose_python(request.python)
    models = bool(set(request.modules) & {"perception", "memory"})
    if hardware.is_jetson and models and not _probe_imports(seed, ("torch", "torchvision")):
        raise InstallError("检测到 Jetson（嵌入式计算平台）；本地模型需要已安装且可导入的厂商 torch/torchvision（张量与视觉库）。可先安装其它模块。")
    if request.robot == "go2" and not _probe_imports(seed, ("unitree_sdk2py", "cyclonedds")):
        for name in ("unitree_sdk2py", "cyclonedds"):
            wheels = list(request.wheel_dir.glob(name + "-*.whl")) if request.wheel_dir else []
            if len(wheels) != 1:
                raise InstallError(f"Go2 设备软件尚未就绪：需要 {name}；请通过 --wheel-dir 提供设备安装包，或用 --robot customrobot 安装标准接入。")
    source_identity = source_fingerprint(request.root, request.modules, request.robot)
    fingerprint = hashlib.sha256((source_identity + seed).encode()).hexdigest()
    with installation_lock(request.home), idle_gateway(request.root):
        current = read_current(request.home)
        if current and current.get("source_root") != str(request.root):
            raise InstallError("此安装目录属于另一个源码工作区，请使用独立 --install-dir。")
        if current.get("fingerprint") == fingerprint and not request.repair:
            _verify_prefix(Path(current["prefix"]), request.modules)
            print("所选模块已经安装，无需重复处理。使用 --repair 可重新安装。")
            return current
        previous = Path(current["prefix"]) if current else None
        identifier = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
        prefix = request.home / "generations" / identifier
        prefix.mkdir(parents=True)
        logs = request.home / "logs"
        logs.mkdir(exist_ok=True)
        run = CommandRunner(logs / (identifier + ".log"), request.root)
        temporary = request.home / (".current-" + identifier)
        try:
            print("[1/5] 安装所选模块的系统依赖", flush=True)
            _system_dependencies(module_system_packages(request.modules, request.robot), run)
            print("[2/5] 创建独立 Python 环境", flush=True)
            _python_environment(request, hardware, prefix, seed, run)
            print(f"[3/5] 构建所选组件（并行 {request.jobs}）", flush=True)
            binaries = build_modules(request.root, prefix, request.home / "cache", request.modules, request.jobs, run)
            print("[4/5] 准备所选模型和启动配置", flush=True)
            write_runtime(request.root, prefix, request.modules, request.robot, existing_config(request.root, current))
            _prepare_models(request, prefix, run)
            _verify_prefix(prefix, request.modules)
            print("[5/5] 校验完成，切换当前安装", flush=True)
            state = {"schema": 1, "complete": True, "modules": list(request.modules),
                     "robot": request.robot, "source_root": str(request.root), "fingerprint": fingerprint,
                     "python": seed, "binaries": binaries, "installed_at": identifier}
            atomic_write(prefix / "manifest.json", json.dumps(state, ensure_ascii=False, indent=2) + "\n", mode=0o600)
            temporary.symlink_to(prefix.relative_to(request.home))
            os.replace(str(temporary), str(request.home / "current"))
            fsync_directory(request.home)
        except BaseException:
            # 信号可能发生在切换成功之后；已生效的环境不能按未完成产物删除。
            if (request.home / "current").resolve() != prefix:
                shutil.rmtree(prefix)
            if temporary.is_symlink():
                temporary.unlink()
            raise
        if previous:
            # 默认数据在安装目录之外；用户自行放进旧代际的数据同样不能删。
            references = []
            for key, value in read_settings(prefix / "settings.env").items():
                path = Path(value)
                if value and path.is_absolute() and (path.resolve() == previous or previous in path.resolve().parents):
                    references.append(key)
            if references:
                print("新安装已生效；旧目录仍被自定义配置引用，已保留：" + "、".join(references))
            else:
                try:
                    shutil.rmtree(previous)
                except OSError as exc:
                    print(f"新安装已生效，旧安装清理未完成：{exc}")
        try:
            prune_build_cache(request.home / "cache", set(current.get("modules", ())) - set(request.modules), request.modules)
        except (OSError, RuntimeError) as exc:
            print(f"新安装已生效，已退出模块的缓存清理未完成：{exc}")
        return read_current(request.home)
