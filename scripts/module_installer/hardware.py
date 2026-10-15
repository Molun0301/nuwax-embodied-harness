"""只读探测宿主资源并给出可解释的保守安装建议。"""
from __future__ import annotations

import math
import os
import platform
import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Mapping, Optional

from .catalog import MODULES, ROBOT_REQUIREMENTS, resolve_modules

GIB = 1024 ** 3


@dataclass(frozen=True)
class HardwareInfo:
    os_id: str = "unknown"
    version_id: str = "unknown"
    architecture: str = "unknown"
    cpu_count: int = 1
    cpu_limit: Optional[float] = None
    memory_bytes: int = 0
    memory_available_bytes: int = 0
    memory_total_bytes: int = 0
    free_disk_bytes: int = 0
    nvidia_gpu: bool = False
    gpu_memory_bytes: int = 0
    is_jetson: bool = False
    model: str = ""
    build_jobs: int = 1
    evidence: tuple[str, ...] = ()
    camera_detected: bool = False

    @property
    def os_version(self) -> str:
        return self.version_id

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class Recommendation:
    modules: tuple[str, ...]
    robot: str
    reasons: tuple[str, ...]
    warnings: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip().strip("\0")
    except OSError:
        return ""


def _positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except (ValueError, TypeError):
        return 0
    return max(0, parsed)


def _cgroup_paths(filesystem_root: Path, controller: str) -> tuple[Path, ...]:
    """同时覆盖容器挂载根和进程所在组，并检查父组较小的限制。"""
    root = filesystem_root / "sys/fs/cgroup"
    candidates = [root, root / controller]
    for line in _read(filesystem_root / "proc/self/cgroup").splitlines():
        fields = line.split(":", 2)
        if len(fields) != 3:
            continue
        controllers, relative = fields[1], Path(fields[2].lstrip("/"))
        if ".." in relative.parts or (controllers and controller not in controllers.split(",")):
            continue
        bases = (root,) if not controllers else (root / controllers, root / controller)
        for base in bases:
            current = base / relative
            while current == base or base in current.parents:
                candidates.append(current)
                if current == base:
                    break
                current = current.parent
    return tuple(dict.fromkeys(candidates))


def _resource_limits(filesystem_root: Path) -> tuple[int, int, Optional[float], tuple[str, ...]]:
    memory_limits = []
    memory_remaining = []
    cpu_limits = []
    evidence = []
    for directory in _cgroup_paths(filesystem_root, "memory"):
        for limit_name, used_name in (("memory.max", "memory.current"), ("memory.limit_in_bytes", "memory.usage_in_bytes")):
            limit = _positive_int(_read(directory / limit_name))
            # 第一版内存控制组以接近有符号整数上限的数值表示无限制。
            if limit and limit < 2 ** 60:
                memory_limits.append(limit)
                used = _positive_int(_read(directory / used_name))
                memory_remaining.append(max(0, limit - used))
                evidence.append(f"{directory / limit_name}={limit}")
    for directory in _cgroup_paths(filesystem_root, "cpu"):
        fields = _read(directory / "cpu.max").split()
        if len(fields) == 2:
            quota, period = _positive_int(fields[0]), _positive_int(fields[1])
            if quota and period:
                cpu_limits.append(quota / period)
                evidence.append("{}={}".format(directory / "cpu.max", " ".join(fields)))
        quota = _positive_int(_read(directory / "cpu.cfs_quota_us"))
        period = _positive_int(_read(directory / "cpu.cfs_period_us"))
        if quota and period:
            cpu_limits.append(quota / period)
            evidence.append(f"{directory}: {quota}/{period}")
    return (min(memory_limits) if memory_limits else 0, min(memory_remaining) if memory_remaining else -1,
            min(cpu_limits) if cpu_limits else None, tuple(evidence))


def detect_hardware(root: Path, *, filesystem_root: Path = Path("/"), run_commands: bool = True) -> HardwareInfo:
    """root 为目标安装所在目录；探测不会联网、安装软件或导入推理框架。"""
    root = Path(root).expanduser().resolve()
    filesystem_root = Path(filesystem_root)
    evidence = []
    os_release = {}
    for line in _read(filesystem_root / "etc/os-release").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            os_release[key] = value.strip('"').strip("'")
    os_id = os_release.get("ID", platform.system().lower())
    version_id = os_release.get("VERSION_ID", "unknown")
    architecture = {"arm64": "aarch64", "amd64": "x86_64"}.get(platform.machine().lower(), platform.machine().lower())
    evidence.append(f"系统 {os_id} {version_id}，架构 {architecture}；来源 /etc/os-release 与系统查询。")
    cpu_count = max(1, os.cpu_count() or 1)
    if hasattr(os, "sched_getaffinity"):
        try:
            cpu_count = max(1, len(os.sched_getaffinity(0)))
            evidence.append(f"进程可使用 {cpu_count} 个处理器；来源 sched_getaffinity。")
        except OSError:
            pass
    memory = {}
    for line in _read(filesystem_root / "proc/meminfo").splitlines():
        key, separator, value = line.partition(":")
        if separator and value.split():
            memory[key] = _positive_int(value.split()[0]) * 1024
    total = memory.get("MemTotal", 0)
    available = memory.get("MemAvailable", memory.get("MemFree", total))
    if total:
        evidence.append("系统物理内存 %.1f GiB；来源 /proc/meminfo。" % (total / GIB))
    memory_limit, remaining, cpu_limit, cgroup_evidence = _resource_limits(filesystem_root)
    evidence.extend(cgroup_evidence)
    effective_memory = min(total, memory_limit) if total and memory_limit else total or memory_limit
    if remaining >= 0:
        available = min(available, remaining) if total else remaining
    if cpu_limit is not None:
        cpu_count = min(cpu_count, max(1, math.ceil(cpu_limit)))
    probe = root
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    try:
        free_disk = shutil.disk_usage(probe).free
        evidence.append(f"安装盘可用 {free_disk / GIB:.1f} GiB；来源 {probe}。")
    except OSError:
        free_disk = 0
        evidence.append("无法读取目标安装盘剩余空间。")
    model = _read(filesystem_root / "proc/device-tree/model") or _read(filesystem_root / "sys/firmware/devicetree/base/model")
    tegra = _read(filesystem_root / "etc/nv_tegra_release")
    is_jetson = bool(tegra or "jetson" in model.lower())
    nvidia_gpu = is_jetson
    if model:
        evidence.append("设备树型号：" + model)
    if tegra:
        evidence.append("检测到 Jetson 厂商系统标识 /etc/nv_tegra_release。")
    gpu_root = filesystem_root / "proc/driver/nvidia/gpus"
    try:
        if gpu_root.is_dir() and any(gpu_root.iterdir()):
            nvidia_gpu = True
            evidence.append("NVIDIA 驱动已登记图形设备；来源 /proc/driver/nvidia/gpus。")
        for vendor_file in (filesystem_root / "sys/class/drm").glob("card*/device/vendor"):
            if _read(vendor_file).lower() == "0x10de":
                nvidia_gpu = True
                evidence.append("NVIDIA 图形设备厂商标识：" + str(vendor_file))
    except OSError:
        pass
    camera_devices = []
    try:
        for device in (filesystem_root / "sys/class/video4linux").glob("video*"):
            if device.is_dir():
                camera_devices.append(str(device))
        for device in (filesystem_root / "dev").glob("video*"):
            if device.is_char_device():
                camera_devices.append(str(device))
    except OSError:
        pass
    camera_detected = bool(camera_devices)
    if camera_detected:
        camera_names = "、".join(camera_devices[:4])
        evidence.append(f"检测到摄像头设备节点：{camera_names}；实际图像数据仍待启动验证。")
    gpu_memory = 0
    # 型号存在与推理环境可用是不同事实；这里仅报告设备和显存。
    if run_commands and filesystem_root == Path("/") and shutil.which("nvidia-smi"):
        try:
            result = subprocess.run(
                ["nvidia-smi", "--query-gpu=memory.total", "--format=csv,noheader,nounits"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=3, check=False,
            )
            capacities = [_positive_int(line.strip()) * 1024 ** 2 for line in result.stdout.splitlines()]
            if result.returncode == 0 and any(capacities):
                nvidia_gpu = True
                gpu_memory = max(capacities)
                evidence.append("单卡最大显存 %.1f GiB；来源 nvidia-smi 只读查询。" % (gpu_memory / GIB))
        except (OSError, subprocess.SubprocessError):
            evidence.append("NVIDIA 显存查询不可用；未据此判定推理环境可用。")
    cpu_jobs = cpu_count
    # 依据当前可用内存收敛原生编译并发，未知资源时退到单进程。
    memory_jobs = max(1, available // (2 * GIB)) if effective_memory else 1
    jobs = min(cpu_jobs, memory_jobs)
    evidence.append(f"自动编译并行 {jobs}：可用处理器 {cpu_jobs}，当前可用内存 {available / GIB:.1f} GiB（每任务预算 2 GiB）。")
    return HardwareInfo(os_id, version_id, architecture, cpu_count, cpu_limit, effective_memory,
                        available, total, free_disk, nvidia_gpu, gpu_memory, is_jetson, model, jobs, tuple(evidence), camera_detected)


def resolve_build_jobs(root: Path, requested: Optional[int] = None, *,
                       hardware: Optional[HardwareInfo] = None,
                       environment: Optional[Mapping[str, str]] = None) -> int:
    """One budget for standalone builders and Cargo's shared native jobserver."""
    environment = os.environ if environment is None else environment
    source = "--jobs"
    if requested is None:
        for source in ("CARGO_BUILD_JOBS", "CMAKE_BUILD_PARALLEL_LEVEL"):
            value = environment.get(source, "").strip()
            if value:
                try:
                    requested = int(value)
                except ValueError as exc:
                    raise ValueError(source + " 必须是正整数") from exc
                break
    if requested is not None:
        if isinstance(requested, bool) or not isinstance(requested, int) or requested < 1:
            raise ValueError(source + " 必须是正整数")
        return requested
    return (hardware or detect_hardware(root, run_commands=False)).build_jobs


def _enabled(env: Mapping[str, str], key: str) -> bool:
    return str(env.get(key, "")).strip().lower() in ("1", "true", "yes", "on")


def recommend_modules(hardware: HardwareInfo, existing_env: Optional[Mapping[str, str]] = None,
                      installed_modules: Optional[Iterable[str]] = None) -> Recommendation:
    """保留明确安装意图；首次依据资源、设备节点和既有配置推荐大模块。"""
    env = existing_env or {}
    reasons = []
    warnings = []
    robot = str(env.get("NUWAX_ACTIVE_ROBOT", "")).strip().lower()
    if robot not in ROBOT_REQUIREMENTS:
        robot = "customrobot"
        reasons.append("未发现明确机器人配置，使用通用标准数据接入；计算板型号不能证明机器人型号。")
    else:
        reasons.append("沿用现有机器人配置：" + robot)
    if robot in ("g1", "raspi_lidar_robot"):
        warnings.append("该机器人当前接入实现仍为骨架；安装依赖不表示设备功能已经实现。")
    if installed_modules is not None:
        modules = resolve_modules(installed_modules)
        reasons.append("保留上次成功安装的模块；硬件变化只提供建议，不自动增加或移除功能。")
        return Recommendation(modules, robot, tuple(reasons), tuple(warnings))
    selected = {"core"}
    reasons.append("基础网关始终安装，不安装本地推理模型也可运行。")
    geometry_configured = _enabled(env, "NUWAX_GEOMETRY_ENABLED")
    # 仅参数模板中的话题名不能证明设备在线；主动保存的启用开关作为用户已有意图。
    if geometry_configured:
        if hardware.memory_bytes >= 4 * GIB and hardware.cpu_count >= 2:
            selected.add("geometry")
            reasons.append("已有几何启用配置，且可用资源上限满足保守推荐条件。")
            backend = str(env.get("NUWAX_GEOMETRY_LIO_BACKEND", "")).strip().lower()
            if backend in ("fastlio2", "pointlio"):
                selected.add(backend)
                reasons.append("沿用明确配置的定位后端：" + MODULES[backend].title)
            elif backend not in ("", "device", "disabled", "none"):
                warnings.append("现有定位后端无法对应安装模块，请检查 NUWAX_GEOMETRY_LIO_BACKEND。")
        else:
            warnings.append("已有几何配置，但资源较少或未知，默认先装基础网关；仍可手动选择几何模块。")
    else:
        reasons.append("未发现明确几何启用配置，不推测雷达存在或自动安装定位算法。")
    visual_configured = _enabled(env, "NUWAX_PERCEPTION_YOLO_WORLD_ENABLED")
    sufficient_visual = hardware.memory_bytes >= 8 * GIB and hardware.cpu_count >= 4
    if (visual_configured or hardware.camera_detected) and sufficient_visual:
        selected.add("perception")
        if visual_configured:
            reasons.append("已有本地视觉启用配置且内存充足，推荐保留视觉检测。")
        else:
            reasons.append("检测到摄像头，建议本地视觉；实际图像数据和模型推理仍待启动验证。")
        if not hardware.nvidia_gpu:
            warnings.append("未检测到 NVIDIA 图形设备，本地视觉可能使用处理器推理；实际延迟须运行测量。")
    elif visual_configured or hardware.camera_detected:
        warnings.append("检测到摄像头或已有本地视觉配置，但处理器或内存资源较少、未知，默认不安装推理框架和视觉模型。")
    # 不按显卡档次猜用途；导航、跟随和大语义模型需要用户明确选择。
    reasons.append("导航、人物跟随与本地语义模型按用途选装，高配设备也不会自动全装。")
    if hardware.is_jetson and "perception" in selected:
        warnings.append("Jetson 视觉安装需要现有厂商 torch/torchvision 可用；硬件检测不代替导入验证。")
    if hardware.free_disk_bytes < 4 * GIB:
        warnings.append("目标磁盘空间不足或未知，请在执行安装前确认可用空间。")
    return Recommendation(resolve_modules(selected), robot, tuple(reasons), tuple(warnings))
