"""安装大模块及其真实依赖；只使用标准库，安装前即可导入。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


class ModuleError(ValueError):
    """模块选择不满足依赖约束。"""


@dataclass(frozen=True)
class ModuleSpec:
    key: str
    title: str
    description: str
    requires: tuple[str, ...] = ()
    conflicts: tuple[str, ...] = ()
    requirements: tuple[str, ...] = ()
    imports: tuple[str, ...] = ()


MODULES: dict[str, ModuleSpec] = {
    "core": ModuleSpec(
        "core", "基础网关", "控制台、机器人接口、标准数据接入；不安装推理模型。",
        requirements=("requirements/modules/core.in",),
        imports=("fastapi", "uvicorn", "websockets", "numpy", "cv2", "grpc", "google.protobuf", "jsonschema", "psutil", "lcm"),
    ),
    "geometry": ModuleSpec(
        "geometry", "几何建图", "定位与地图运行时；可使用设备已有里程计。", ("core",),
    ),
    "fastlio2": ModuleSpec(
        "fastlio2", "FastLIO2 激光惯性定位", "安装原生 FastLIO2，需要真实点云、惯性输入及标定。",
        ("geometry",), ("pointlio",),
    ),
    "pointlio": ModuleSpec(
        "pointlio", "PointLIO 激光惯性定位", "安装原生 PointLIO，与 FastLIO2 选择一个即可。",
        ("geometry",), ("fastlio2",),
    ),
    "navigation": ModuleSpec(
        "navigation", "导航探索", "导航独立程序与运动守卫；运行仍需要真实执行器及标定。", ("geometry",),
    ),
    "perception": ModuleSpec(
        "perception", "本地视觉感知", "目标检测与跟踪，安装本地推理依赖和视觉模型。", ("core",),
        requirements=("requirements/modules/perception.in",), imports=("torch", "torchvision", "ultralytics", "clip"),
    ),
    "follow": ModuleSpec(
        "follow", "人物跟随", "视觉目标跟随程序，自动补齐视觉感知与导航。", ("perception", "navigation"),
    ),
    "memory": ModuleSpec(
        "memory", "本地语义记忆", "本地文本与图像向量模型；普通地图和已有记忆数据不随卸载删除。", ("core",),
        requirements=("requirements/modules/memory.in",), imports=("torch", "sentence_transformers", "transformers"),
    ),
}

ROBOT_REQUIREMENTS = {
    "customrobot": (),
    "go2": ("requirements/robot-go2.in",),
    "g1": ("requirements/robot-g1.in",),
    "raspi_lidar_robot": ("requirements/robot-raspi.in",),
}
ROBOT_IMPORTS = {
    "customrobot": (),
    "go2": ("unitree_sdk2py", "cyclonedds", "unitree_webrtc_connect"),
    "g1": ("aiortc", "aiohttp"),
    "raspi_lidar_robot": ("serial",),
}


def _keys(values: Iterable[str]) -> tuple[str, ...]:
    if isinstance(values, str):
        values = values.split(",")
    result = tuple(dict.fromkeys(str(value).strip().lower() for value in values if str(value).strip()))
    unknown = tuple(key for key in result if key not in MODULES)
    if unknown:
        raise ModuleError("未知安装模块：" + "、".join(unknown))
    return result


def resolve_modules(selected: Iterable[str]) -> tuple[str, ...]:
    """补齐依赖，按目录顺序返回；不会静默替换互斥后端。"""
    wanted = set(_keys(selected)) | {"core"}
    pending = list(wanted)
    while pending:
        for dependency in MODULES[pending.pop()].requires:
            if dependency not in wanted:
                wanted.add(dependency)
                pending.append(dependency)
    for key, spec in MODULES.items():
        if key in wanted:
            for conflict in spec.conflicts:
                if conflict in wanted:
                    raise ModuleError(f"{spec.title} 与 {MODULES[conflict].title} 不能同时安装，请选择一个定位后端。")
    return tuple(key for key in MODULES if key in wanted)


def remove_modules(installed: Iterable[str], removed: Iterable[str], *, cascade: bool = False) -> tuple[str, ...]:
    """删除模块；依赖者默认报错，显式递归模式可一并移除。"""
    current = set(resolve_modules(installed))
    deleting = set(_keys(removed))
    if "core" in deleting:
        raise ModuleError("基础网关是必选模块；模块修改不能删除基础安装。")
    while True:
        dependants = {key for key in current - deleting if set(MODULES[key].requires) & deleting}
        if not dependants:
            break
        if not cascade:
            names = "、".join(MODULES[key].title for key in MODULES if key in dependants)
            raise ModuleError(f"仍有模块依赖待移除内容：{names}。请同时移除这些模块，或选择递归移除。")
        deleting.update(dependants)
    return tuple(key for key in MODULES if key in current - deleting)


def replace_lio(modules: Iterable[str], backend: str) -> tuple[str, ...]:
    """明确切换定位后端；device 表示使用设备里程计，不安装原生算法。"""
    backend = str(backend).strip().lower()
    if backend not in ("fastlio2", "pointlio", "device"):
        raise ModuleError("定位后端只支持 fastlio2、pointlio 或 device（设备里程计）。")
    selected = set(_keys(modules)) - {"fastlio2", "pointlio"}
    if backend != "device":
        selected.add(backend)
    return resolve_modules(selected)


def _robot(robot: str) -> str:
    if robot not in ROBOT_REQUIREMENTS:
        raise ModuleError("未知机器人类型：" + str(robot))
    return robot


def module_requirements(modules: Iterable[str], robot: str = "customrobot") -> tuple[str, ...]:
    paths = tuple(path for key in resolve_modules(modules) for path in MODULES[key].requirements)
    return tuple(dict.fromkeys(paths + ROBOT_REQUIREMENTS[_robot(robot)]))


def module_imports(modules: Iterable[str], robot: str = "customrobot") -> tuple[str, ...]:
    imports = tuple(name for key in resolve_modules(modules) for name in MODULES[key].imports)
    return tuple(dict.fromkeys(imports + ROBOT_IMPORTS[_robot(robot)]))


def module_system_packages(modules: Iterable[str], robot: str = "customrobot") -> tuple[str, ...]:
    selected = set(resolve_modules(modules))
    _robot(robot)
    # lcm 的 Python 绑定需要编译；opencv-python 的桌面构建需要这些共享库。
    packages = ["ca-certificates", "python3-venv", "python3-dev", "python3-pip", "build-essential", "pkg-config", "liblcm-dev", "libgl1", "libglib2.0-0"]
    if "geometry" in selected:
        packages.extend(("curl", "git", "cmake", "protobuf-compiler", "libssl-dev", "liblcm-bin", "libpcl-dev"))
    if selected & {"fastlio2", "pointlio"}:
        packages.extend(("cmake", "git", "patch", "liblcm-bin", "libeigen3-dev", "libpcl-dev"))
    if "pointlio" in selected:
        packages.append("libgoogle-glog-dev")
    if "perception" in selected:
        packages.append("git")
    if robot == "go2":
        packages.extend(("python3-pyaudio", "portaudio19-dev"))
    return tuple(dict.fromkeys(packages))
