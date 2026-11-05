"""在独立环境完成目标安装，再原子切换；失败保留上一有效安装。"""

from __future__ import annotations

import fcntl
import json
import os
import re
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import time
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence

from scripts.device_manager.common import atomic_write, fsync_directory
from scripts.neh_system_helper import dependency_plan

from .builds import build_modules, describe_builds, prune_build_cache
from .catalog import ROBOT_REQUIREMENTS, module_imports, module_requirements, module_system_packages, resolve_modules
from .runtime import read_existing, read_settings, write_profile, write_runtime, _VALUES

# Increment when dependency preparation changes without a requirements change.
DEPENDENCY_RECIPE_VERSION = 1


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
    keep_previous: bool = False
    noninteractive: bool = False
    service_mode: str = ""
    lio_backend: str = ""


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
        if state["robot"] not in ROBOT_REQUIREMENTS:
            raise ValueError("未知机器人装配")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise InstallError(f"安装状态损坏：{exc}；请检查 {prefix}") from exc
    state["prefix"] = str(prefix)
    return state


def existing_config(root: Path, current: Mapping) -> dict:
    return read_existing(root)


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


def _file_metadata(path: Path) -> list:
    info = path.stat()
    return [info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_mode]


def source_inputs(root: Path, home: Path) -> list:
    """Record source file metadata without reading or hashing source contents.

    This is a local incremental-install hint, not content authentication.
    Metadata-only changes may conservatively request another installation.
    """
    outputs = tuple((home / name).absolute() for name in ("generations", "cache", "logs", "current", "install.lock"))
    outputs += tuple((root / "drivers").glob("*/configs"))
    try:
        result = subprocess.run(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=str(root), capture_output=True, check=False,
        )
        paths = [root / os.fsdecode(name) for name in result.stdout.split(b"\0") if name] if result.returncode == 0 else []
    except OSError:
        paths = []
    if not paths:
        # Preserve the source-export scope; runtime/build products are not inputs.
        excluded = {".git", ".venv", "target", "result", "build", "__pycache__", "runtime_data", "docs", "artifacts", "tests"}
        for directory, subdirs, files in os.walk(root):
            subdirs[:] = [name for name in subdirs if name not in excluded and not name.startswith(".")
                          and (Path(directory) / name).absolute() not in outputs]
            paths.extend(Path(directory) / name for name in files if not name.startswith("."))
    for directory in (root / "scripts/module_installer", root / "requirements/modules"):
        paths.extend(directory.glob("*.py"))
        paths.extend(directory.glob("*.in"))
    records = []
    for path in sorted(set(paths)):
        absolute = path.absolute()
        if any(absolute == output or output in absolute.parents for output in outputs):
            continue
        try:
            info = path.lstat()
        except FileNotFoundError:
            continue  # A deleted tracked source no longer appears in the record.
        if stat.S_ISREG(info.st_mode):
            records.append([str(path.relative_to(root)), info.st_size, info.st_mtime_ns, info.st_ctime_ns, info.st_mode])
    return records


def installation_inputs(request: InstallRequest, python: str) -> dict:
    wheels = []
    if request.robot == "go2" and request.wheel_dir is not None:
        for distribution in ("cyclonedds", "unitree_sdk2py"):
            for path in sorted(request.wheel_dir.glob(distribution + "-*.whl")):
                wheels.append([str(path.resolve()), *_file_metadata(path)])
    interpreter = Path(python).resolve()
    return {
        "version": 1,
        "source_files": source_inputs(request.root, request.home),
        "modules": list(request.modules), "robot": request.robot,
        "python": [python, str(interpreter), *_file_metadata(interpreter)],
        "device_wheels": wheels,
        "lio_backend": request.lio_backend,
    }


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
        "service_mode": request.service_mode or current.get("deployment", {}).get("lifecycle", {}).get("mode", "foreground"),
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
def idle_gateway(home: Path):
    """与真实启动器共用锁，整个安装过程禁止旧环境开始运行。"""
    path = home / "run/business.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise InstallError("网关正在运行，请先执行 ./install.sh stop，再修改安装模块。") from exc
        yield


def _system_dependencies(packages: Sequence[str], run, *, noninteractive: bool = False) -> None:
    payload = {"packages": list(packages)}
    missing = dependency_plan(payload)["missing"]
    if missing and noninteractive:
        raise InstallError("自动更新缺少系统依赖，请由管理员先安装：" + ", ".join(missing))
    if missing:
        prefix = [] if os.geteuid() == 0 else ["sudo", "--"]
        if prefix:
            if not shutil.which("sudo"):
                raise InstallError("缺少系统依赖且找不到 sudo（提权工具）：" + ", ".join(missing))
            # 密码提示直接连接终端，不隐藏到安装日志。
            subprocess.run(["sudo", "-v"], check=True)
        # 沿用命令执行器的日志与进程组回收，只容许刷新命令的非零退出。
        with run.log.open("a+", encoding="utf-8") as output:
            output.seek(0, os.SEEK_END)
            start = output.tell()
            try:
                run(prefix + ["apt-get", "update"])
            except InstallError:
                pass
            output.seek(start)
            refresh_diagnostic = output.read()
        # 与特权助手共用刷新后的包名解析，包含发行版的t64映射。
        missing = dependency_plan(payload)["missing"]
        if missing:
            try:
                run(prefix + ["apt-get", "install", "-y", "--no-install-recommends"] + missing)
            except InstallError as exc:
                raise InstallError("系统依赖安装失败（" + "、".join(missing) + "）：" + str(exc)
                                   + "\n包列表刷新诊断：\n" + refresh_diagnostic) from exc


_JETSON_VENDOR_PACKAGES = ("torch", "torchvision")


def jetson_vendor_packages(python: str) -> dict:
    """定位Jetson厂商torch/torchvision：先查系统目录，再查当前用户目录。

    返回 {"site": 所在用户目录或空串, "versions": {...}, "entries": [...]}；空串表示位于系统目录，
    entries 为两个发行包在用户目录中的顶层文件与元数据目录。
    """
    code = (
        "import importlib, importlib.metadata as m, json, os, site\n"
        f"names = {list(_JETSON_VENDOR_PACKAGES)!r}\n"
        "modules = [importlib.import_module(n) for n in names]\n"
        "user = os.path.realpath(site.getusersitepackages()) if site.ENABLE_USER_SITE else ''\n"
        "inside = [bool(user) and os.path.realpath(x.__file__).startswith(user + os.sep) for x in modules]\n"
        "entries = sorted({f.parts[0] for n in names for f in (m.distribution(n).files or []) if f.parts and f.parts[0] != '..'})\n"
        "print(json.dumps({'versions': {n: m.version(n) for n in names}, 'user_site': user, 'inside': inside, 'entries': entries}))"
    )
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    reason = ""
    for user_site in (False, True):
        if user_site:
            environment.pop("PYTHONNOUSERSITE", None)
        else:
            environment["PYTHONNOUSERSITE"] = "1"
        result = subprocess.run([python, "-c", code], capture_output=True, text=True, timeout=45, check=False, env=environment)
        if result.returncode:
            reason = (result.stderr.strip().splitlines() or ["无法导入"])[-1]
            continue
        found = json.loads(result.stdout.strip().splitlines()[-1])
        if not user_site:
            return {"site": "", "versions": found["versions"]}
        if all(found["inside"]) and found["entries"]:
            return {"site": found["user_site"], "versions": found["versions"], "entries": found["entries"]}
        if all(found["inside"]):
            reason = "用户目录中的 torch/torchvision 缺少安装记录（RECORD），无法单独接入"
            continue
        reason = "torch与torchvision分别位于系统目录和用户目录，请安装到同一位置"
    raise InstallError(
        "检测到 Jetson（嵌入式计算平台）；本地视觉与语义记忆需要与 JetPack（厂商软件栈）匹配的 torch/torchvision（张量与视觉库），"
        f"{python} 在系统目录和当前用户目录都不能使用：{reason}。请先安装厂商 torch/torchvision，或先安装其它模块。"
    )


def _vendor_constraints(python: str, target: Path, names: Sequence[str]) -> None:
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment["PYTHONNOUSERSITE"] = "1"
    code = f"from importlib.metadata import version; print('\\n'.join(n+'=='+version(n) for n in {list(names)!r}))"
    result = subprocess.run([python, "-c", code], capture_output=True, text=True, check=True, timeout=45, env=environment)
    atomic_write(target, (result.stdout).encode("utf-8"), mode=0o600)


def _check_dependencies(python: str) -> None:
    """pip check只追究本环境与厂商包；Jetson环境可见的系统包自身缺依赖时只记录。"""
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment.update({"PYTHONNOUSERSITE": "1", "PIP_DISABLE_PIP_VERSION_CHECK": "1"})
    result = subprocess.run([python, "-m", "pip", "check"], capture_output=True, text=True, timeout=300, check=False, env=environment)
    if not result.returncode:
        return
    code = (
        "import importlib.metadata as m, json, os, sys, sysconfig\n"
        "roots = {os.path.realpath(sysconfig.get_paths()[k]) for k in ('purelib', 'platlib')}\n"
        "pth = os.path.join(sysconfig.get_paths()['purelib'], 'nuwax-jetson-vendor.pth')\n"
        "roots |= {os.path.realpath(line.strip()) for line in open(pth) if line.strip()} if os.path.exists(pth) else set()\n"
        "print(json.dumps(sorted({d.metadata['Name'] for d in m.distributions() if os.path.realpath(str(d.locate_file(''))) in roots})))"
    )
    names = json.loads(subprocess.check_output([python, "-c", code], text=True, env=environment))
    normalize = lambda name: re.sub(r"[-_.]+", "-", name).lower()
    owned = {normalize(name) for name in list(names) + list(_JETSON_VENDOR_PACKAGES)}
    problems = [line for line in result.stdout.splitlines() if line.strip() and normalize(line.split()[0]) in owned]
    if problems:
        raise InstallError("Python 依赖不一致：" + "；".join(problems))
    print("运行环境外的已有包存在依赖问题，不影响本环境：" + "；".join(line for line in result.stdout.splitlines() if line.strip()), flush=True)


def _preload_bundled_openmp(python: str) -> None:
    """aarch64上glibc 2.31的静态TLS余量很小，torch、cv2等加载后再加载轮子自带的libgomp会失败；让本环境的进程启动时先加载。"""
    purelib = Path(subprocess.check_output([python, "-c", "import sysconfig; print(sysconfig.get_paths()['purelib'])"], text=True).strip())
    libraries = sorted(str(path) for path in purelib.glob("*.libs/libgomp-*.so*"))
    if libraries:
        # 局部加载，不进入全局符号表；torch仍使用它链接的系统libgomp。
        line = "import ctypes; " + "; ".join(f"ctypes.CDLL({library!r})" for library in libraries) + "\n"
        atomic_write(purelib / "nuwax-preload-openmp.pth", line.encode("utf-8"), mode=0o644)


def _python_environment(request: InstallRequest, hardware, prefix: Path, seed: str, run) -> None:
    models = bool(set(request.modules) & {"perception", "memory"})
    vendor = jetson_vendor_packages(seed) if hardware.is_jetson and models else None
    venv_args = [seed, "-m", "venv"]
    if vendor is not None:
        venv_args.append("--system-site-packages")
    run(venv_args + [str(prefix / "venv")])
    python = str(prefix / "venv/bin/python")
    if vendor and vendor["site"]:
        # 只接入两个厂商发行包本身，其余依赖仍装进本环境，不借用用户目录里的其它包。
        exposed = prefix / "jetson-vendor"
        shutil.rmtree(exposed, ignore_errors=True)
        exposed.mkdir()
        for entry in vendor["entries"]:
            (exposed / entry).symlink_to(Path(vendor["site"]) / entry)
        purelib = subprocess.check_output([python, "-c", "import sysconfig; print(sysconfig.get_paths()['purelib'])"], text=True).strip()
        atomic_write(Path(purelib) / "nuwax-jetson-vendor.pth", (str(exposed) + "\n").encode("utf-8"), mode=0o644)
    pip = [python, "-m", "pip", "install"]
    tools = ["pip==25.0.1", "setuptools==75.3.2", "wheel==0.45.1"] if request.robot == "go2" else ["pip", "setuptools", "wheel"]
    run(pip + ["--upgrade"] + tools)
    constraints = prefix / "constraints.txt"
    if models:
        if vendor is None:
            torch_args = ["torch", "torchvision"]
            if not hardware.nvidia_gpu:
                torch_args += ["--index-url", "https://download.pytorch.org/whl/cpu"]
            run(pip + torch_args)
        _vendor_constraints(python, constraints, ("torch", "torchvision"))
        pip += ["--constraint", str(constraints)]
    if request.robot == "go2" and request.wheel_dir is not None:
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
    if request.robot == "go2" and request.wheel_dir is None:
        run([python, str(request.root / "scripts/go2_dependencies.py"), "--root", str(request.root),
             "--prefix", str(prefix), "--cache", str(request.home / "cache/go2")])
    if hardware.architecture == "aarch64":
        _preload_bundled_openmp(python)
    _check_dependencies(python)
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
        # YOLOWorld设置词表会使用OpenAI原生CLIP；安装时预取同一默认缓存，首次推理不再隐式下载。
        run([python, "-c", "import os; from clip.clip import _download, _MODELS; "
             "_download(_MODELS['ViT-B/32'], os.path.expanduser('~/.cache/clip'))"])
    if "memory" in request.modules:
        arguments = []
        for kind, directory in (("text", "BAAI__bge-small-zh-v1.5"), ("image", "openai__clip-vit-base-patch32")):
            configured = Path(values[f"NUWAX_MEMORY_{kind.upper()}_EMBEDDING_MODEL"])
            target = configured if configured.is_absolute() else request.root / configured
            if target.resolve() != (output / directory).resolve():
                if not target.is_dir():
                    raise InstallError(f"自定义{kind}记忆模型目录不存在：{target}；安装器不会为它下载另一个默认模型。")
                other_kind = "image" if kind == "text" else "text"
                run([python, "scripts/prepare_memory_models.py", "--validate-only",
                     "--" + kind + "-output-dir", str(target), "--skip-" + other_kind])
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
    if "slam2d" in modules:
        needed += ["bin/nuwax-slam-2d-worker", "lib/libnuwax_slam_toolbox.so.1", "lib/libnuwax_slamtec.so.1"]
    missing = [name for name in needed if not (prefix / name).is_file()]
    if missing:
        raise InstallError("安装产物不完整：" + ", ".join(missing))


def check_prerequisites(request: InstallRequest, hardware, *, system_packages_managed: bool = False) -> str:
    """安装与只读诊断共用准入检查，返回实际可用的解释器。"""
    if sys.platform != "linux":
        raise InstallError("当前自动安装支持 Linux。")
    if not system_packages_managed and hardware.os_id not in ("ubuntu", "debian", "raspbian", "linuxmint"):
        raise InstallError("当前自动安装支持 Debian（操作系统）及 Ubuntu（操作系统）系列；其它系统可以查看 --dry-run 计划。")
    if request.jobs < 1:
        raise InstallError("编译并行数必须大于零。")
    if os.geteuid() == 0:
        raise InstallError("请用普通用户运行 ./install.sh；只有缺少的系统依赖由安装器自动提权安装。")
    if request.robot not in ROBOT_REQUIREMENTS:
        raise InstallError("请选择已注册机器人接入；自定义硬件使用 customrobot。")
    if hardware.architecture not in ("x86_64", "aarch64"):
        raise InstallError("当前自动安装面向 x86_64 与 aarch64（64位ARM架构）；请在受支持的64位主机上安装。")
    ancestor = request.home
    while not ancestor.exists() and ancestor != ancestor.parent:
        ancestor = ancestor.parent
    if not ancestor.is_dir() or not os.access(ancestor, os.W_OK | os.X_OK):
        raise InstallError(f"安装位置不可写：{request.home}；请用 --install-dir 选择当前用户可写的目录。")
    seed = choose_python(request.python)
    if hardware.is_jetson and set(request.modules) & {"perception", "memory"}:
        jetson_vendor_packages(seed)
    if request.robot == "go2" and request.wheel_dir is not None:
        for name in ("unitree_sdk2py", "cyclonedds"):
            wheels = list(request.wheel_dir.glob(name + "-*.whl")) if request.wheel_dir else []
            if len(wheels) != 1:
                raise InstallError(f"显式指定的Go2设备轮子目录缺少唯一{name}；不指定该目录时自动使用官方来源。")
    elif request.robot == "go2":
        from scripts.go2_dependencies import load_recipe
        load_recipe(request.root)
    return seed




def apply(request: InstallRequest, hardware) -> dict:
    seed = check_prerequisites(request, hardware)
    from .lifecycle import MODES, deployment_descriptor, inspect_installation
    if request.service_mode and request.service_mode not in MODES:
        raise InstallError("未知生命周期模式")
    with installation_lock(request.home), idle_gateway(request.home):
        current = read_current(request.home)
        if current and current.get("source_root") != str(request.root):
            raise InstallError("此安装目录属于另一个源码工作区，请使用独立 --install-dir。")
        inputs = installation_inputs(request, seed)
        registration = inspect_installation(request.home)["lifecycle"] if current else None
        mode = request.service_mode or (registration["mode"] if registration else "foreground")
        service_name = registration["service_name"] if registration else None
        if current.get("installation_inputs") == inputs and not request.repair and (not request.service_mode or request.service_mode == registration["mode"]):
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
            _system_dependencies(module_system_packages(request.modules, request.robot), run, noninteractive=request.noninteractive)
            print("[2/5] 创建独立 Python 环境", flush=True)
            _python_environment(request, hardware, prefix, seed, run)
            print(f"[3/5] 构建所选组件（并行 {request.jobs}）", flush=True)
            binaries = build_modules(request.root, prefix, request.home / "cache", request.modules, request.jobs, run)
            print("[4/5] 准备所选模型和启动配置", flush=True)
            configuration = existing_config(request.root, current)
            if request.lio_backend:
                configuration["NUWAX_GEOMETRY_LIO_BACKEND"] = request.lio_backend
            write_runtime(request.root, prefix, request.modules, request.robot, configuration)
            _prepare_models(request, prefix, run)
            _verify_prefix(prefix, request.modules)
            print("[5/5] 校验完成，切换当前安装", flush=True)
            state = {"schema": 1, "complete": True, "modules": list(request.modules),
                     "robot": request.robot, "source_root": str(request.root), "installation_inputs": inputs,
                     "python": seed, "binaries": binaries, "installed_at": identifier}
            if service_name is None:
                service_name = "neh-install-" + str(uuid.uuid4()) + ".service"
            location = _VALUES.configuration_location(root=request.root, environment={}, robot=request.robot)
            write_profile(prefix, location["path"].parent, location["profile"], request.root / "runtime_data/settings")
            state["deployment"] = deployment_descriptor(request.root, request.home, prefix, request.modules,
                                                        request.robot, mode, service_name=service_name)
            atomic_write(prefix / "manifest.json", (json.dumps(state, ensure_ascii=False, indent=2) + "\n").encode("utf-8"), mode=0o600)
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
        if previous and not request.keep_previous:
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
