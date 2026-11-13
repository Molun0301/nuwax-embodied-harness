"""业务配置的模式、方案及合成文件；设备管理程序负责写入。"""
from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
from functools import lru_cache
import json
import logging
import math
import os
from pathlib import Path
import re
import tempfile

try:
    import tomllib
except ImportError:
    import tomli as tomllib

CODE_ROOT = Path(__file__).resolve().parents[1]
_SNAPSHOT = ContextVar("nuwax_config", default=None)
_RUNTIME = None
SECTION_TITLES = {
    "robot": "机器人", "transport": "通信", "gateway": "网关", "perception": "感知",
    "memory": "记忆", "semantic_map": "语义地图", "geometry": "定位与建图",
    "navigation": "导航", "motion_guard": "运动守卫", "person_follow": "人物跟随",
    "go2": "Go2 驱动", "logging": "日志", "relay": "容器中继",
    "unitree_sim": "仿真设备", "manipulation": "操作设备", "map_cloud": "地名查询",
}
PATH_KEYS = {
    "gateway.artifact_dir", "geometry.map_catalog_root", "geometry.mapping_recording_dir",
    "geometry.raw_scan_directory", "geometry.runtime_data_dir", "logging.dir", "logging.file",
}
MODEL_KEYS = {"perception.yolo_world_model", "memory.text_embedding_model", "memory.image_embedding_model"}


def install_home():
    return Path(os.environ.get("NUWAX_INSTALL_HOME", str(CODE_ROOT))).expanduser().resolve()


def read_toml(path):
    with Path(path).open("rb") as handle:
        return tomllib.load(handle)


def atomic_write(path, data):
    """在目标目录完成序列化后的原子替换，不生成备份。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix="." + path.name, dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, str(path))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_toml(path, values):
    import tomli_w
    atomic_write(path, tomli_w.dumps(values).encode("utf-8"))


@lru_cache(maxsize=None)
def schema(code_root=CODE_ROOT):
    return read_toml(Path(code_root) / "core/config_schema.toml")


def schema_default(name):
    section, key = name.split(".", 1)
    return deepcopy(schema()[section][key]["default"])


def specifications(code_root=CODE_ROOT):
    return {section + "." + key: spec for section, entries in schema(code_root).items()
            for key, spec in entries.items()}


def defaults(code_root=CODE_ROOT):
    return {section: {key: deepcopy(spec["default"]) for key, spec in entries.items()}
            for section, entries in schema(code_root).items()}


def flatten(values):
    return {section + "." + key: value for section, entries in values.items() for key, value in entries.items()}


def nested(values):
    result = {}
    for name, value in values.items():
        section, key = name.split(".", 1)
        result.setdefault(section, {})[key] = value
    return result


def validate_value(name, value, spec):
    """保存边界按模式校验原生类型、范围、枚举和数组。"""
    if isinstance(value, str) and value in spec.get("special_values", ()):
        return value
    kind = {"boolean": "bool", "integer": "int"}.get(spec["type"], spec["type"])
    valid = {"string": isinstance(value, str), "bool": type(value) is bool,
             "int": type(value) is int, "float": type(value) in (int, float),
             "array": isinstance(value, list)}[kind]
    if not valid:
        raise ValueError(name + " 类型必须为 " + kind)
    if kind in ("int", "float"):
        if not math.isfinite(value):
            raise ValueError(name + " 必须为有限数值")
        for bound, invalid in (("min", value < spec.get("min", -math.inf)),
                               ("max", value > spec.get("max", math.inf))):
            if invalid or (spec.get(bound + "_exclusive") and value == spec[bound]):
                raise ValueError(name + " 超出模式范围")
        if kind == "float":
            value = float(value)
    if "choices" in spec and value not in spec["choices"]:
        raise ValueError(name + " 不在可选值中")
    if kind == "array":
        if "length" in spec and len(value) != spec["length"]:
            raise ValueError(name + " 数组长度错误")
        value = [validate_value(name, item, {"type": spec["items"]}) for item in value]
    return value


def optional_number(value):
    return None if value == "" else value


def validate_updates(values, code_root=CODE_ROOT):
    specs = specifications(code_root)
    result = {}
    for name, value in values.items():
        if name not in specs:
            raise ValueError("未知配置键：" + name)
        if isinstance(value, str):
            value = value.strip()
        value = validate_value(name, value, specs[name])
        if name == "geometry.pgo_options":
            from contracts.pgo_options import normalize_pgo_options
            value = normalize_pgo_options(value)
        elif name == "geometry.pointlio_options":
            from contracts.pointlio_options import normalize_pointlio_options
            value = normalize_pointlio_options(value)
        elif name == "geometry.slam2d_config" and value:
            parsed = json.loads(value)
            if not isinstance(parsed, dict):
                raise ValueError(name + " 必须为对象")
            value = json.dumps(parsed, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        elif name.startswith("gateway.pointcloud3d_") and (name.endswith("_color") or name.endswith("_colors")):
            colors = value if isinstance(value, list) else [value]
            if any(not re.fullmatch(r"#[0-9a-fA-F]{6}", item) for item in colors):
                raise ValueError(name + " 必须为六位十六进制颜色")
            value = [item.lower() for item in colors] if isinstance(value, list) else value.lower()
        elif name.startswith("robot.") and name.split(".", 1)[1] == "model_joint_positions_json":
            from core.robot_geometry import normalize_robot_geometry_setting
            value = normalize_robot_geometry_setting("model_joint_positions_json", value)
        result[name] = value
    return result


def validate_business_values(values, code_root=CODE_ROOT):
    """业务解释器在保存边界复用已有结构化输入契约。"""
    result = validate_updates(values, code_root)
    if "navigation.facility_providers_json" in result:
        from contracts.building_navigation import normalize_facility_provider_configuration
        result["navigation.facility_providers_json"] = normalize_facility_provider_configuration(result["navigation.facility_providers_json"])
    if "navigation.capabilities_json" in result:
        from contracts.navigation_capabilities import parse_navigation_capabilities
        profile = parse_navigation_capabilities(result["navigation.capabilities_json"])
        result["navigation.capabilities_json"] = "" if profile is None else profile.model_dump_json()
    return result


def resolve_paths(values, home):
    result = deepcopy(values)
    for name in PATH_KEYS | MODEL_KEYS:
        section, key = name.split(".", 1)
        raw = result[section][key]
        if not raw:
            continue
        path = Path(raw).expanduser()
        if name in MODEL_KEYS and not raw.startswith(("runtime_data/", "data/")):
            continue
        if not path.is_absolute():
            parts = path.parts[1:] if path.parts[0] in ("runtime_data", "data") else path.parts
            path = Path(home) / "data" / Path(*parts)
        result[section][key] = str(path)
    model = result["robot"]["model_path"]
    if model and not Path(model).is_absolute():
        result["robot"]["model_path"] = str(Path(home) / model)
    if not result["gateway"]["public_base_url"]:
        result["gateway"]["public_base_url"] = "http://127.0.0.1:" + str(result["gateway"]["port"])
    return result


def derived_values(values, code_root=CODE_ROOT):
    from core.robot_geometry import RobotGeometryConfig, resolve_robot_geometry
    robot, go2 = values["robot"], values["go2"]
    from runpy import run_path
    motion = run_path(str(Path(code_root) / "drivers" / robot["driver"] / "navigation_capabilities.py"))
    profile = {"lio_3d": "pct-scan-3d", "slam_2d": "simple-2d"}[values["geometry"]["pipeline"]]
    linear, yaw, model = (robot[key] for key in (
        "actuator_max_linear_mps", "actuator_max_yaw_rps", "actuator_motion_model"))
    if any(value != "" for value in (linear, yaw, model)):
        if any(value == "" for value in (linear, yaw, model)):
            raise ValueError("基本运动声明须同时填写平面速度、偏航速度和运动形式")
    elif robot["driver"] == "go2":
        linear, yaw, model = go2["official_max_linear_velocity_mps"], go2["official_max_yaw_rate_rps"], "omnidirectional"
    else:
        linear, yaw, model = 0.0, 0.0, ""
    geometry = resolve_robot_geometry(RobotGeometryConfig.from_values(robot),
                                      robot_name=values["unitree_sim"]["model"] if robot["driver"] == "unitree_sim" else robot["driver"],
                                      base_dir=code_root)
    return {"nav_profile": profile,
            "min_executable_linear_mps": motion["MIN_EXECUTABLE_LINEAR_MPS"],
            "min_executable_angular_rps": motion["MIN_EXECUTABLE_ANGULAR_RPS"],
            "actuator_max_linear_mps": float(linear),
            "actuator_max_yaw_rps": float(yaw), "actuator_motion_model": model if profile == "pct-scan-3d" else "",
            "robot_geometry_json": geometry.to_runtime_json()}


def read_config(home=None, code_root=CODE_ROOT):
    """进程只读合成文件；所需字段由实际使用处访问。"""
    return read_toml(Path(home or install_home()) / "run/config.toml")


def runtime_config():
    global _RUNTIME
    snapshot = _SNAPSHOT.get()
    if snapshot is not None:
        return snapshot
    if _RUNTIME is None:
        _RUNTIME = read_config()
    return _RUNTIME


def value(name):
    section, key = name.split(".", 1)
    return runtime_config()[section][key]


@contextmanager
def config_snapshot(values):
    """预览与热加载使用显式快照，不修改进程环境。"""
    token = _SNAPSHOT.set(values)
    try:
        yield values
    finally:
        _SNAPSHOT.reset(token)


def motion_guard_socket(home=None):
    relative = (CODE_ROOT / "services/navigation/proto/motion_guard_socket_relative.txt").read_text().strip()
    return str(Path(home or install_home()) / relative)


def binary_path(name):
    current = install_home() / "current"
    if current.is_symlink():
        return str(current / "bin" / name)
    directory = (CODE_ROOT / "services/geometry/nuwax_geometry_runtime/target/release"
                 if name == "nuwax-geometry-runtime" else CODE_ROOT / "services/nuwax-workspace/target/release")
    return str(directory / name)


class ConfigStore:
    """设备管理持有的方案操作；调用方在管理事务锁内串行写入。"""

    def __init__(self, home, code_root=CODE_ROOT, port=None):
        self.home, self.code_root, self.asset_root = Path(home), CODE_ROOT, Path(code_root)
        if port is None:
            marker = self.home / "config/manager-home.json"
            manager = Path(json.loads(marker.read_text())["manager_dir"]) if marker.exists() else self.home
            settings = manager / "config/settings.json"
            saved = json.loads(settings.read_text()) if settings.exists() else {}
            port = saved.get("gateway_port", schema(self.code_root)["gateway"]["port"]["default"])
        self.port = port
        self.directory = self.home / "config/profiles"
        self.selector = self.home / "config/active_profile"

    def profile_path(self, name):
        if not isinstance(name, str) or not re.fullmatch(r"[\w\-\u4e00-\u9fff][\w\- \u4e00-\u9fff]{0,63}", name) or name != name.strip():
            raise ValueError("配置名称限1—64个中文、字母、数字、空格、下划线或短横线")
        return self.directory / (name + ".toml")

    def active(self):
        return self.selector.read_text(encoding="utf-8").strip()

    def profile_values(self, name, *, startup=False):
        values = flatten(read_toml(self.profile_path(name)))
        specs = specifications(self.code_root)
        for key in tuple(values):
            if key not in specs:
                if not startup:
                    raise ValueError("未知配置键：" + key)
                logging.getLogger("nuwax_robot_bridge.config_io").warning("忽略方案中的未知配置键：%s", key)
                del values[key]
        return validate_business_values(values, self.code_root)

    def effective(self, name=None, *, startup=False):
        values = flatten(defaults(self.code_root))
        values.update(self.profile_values(name or self.active(), startup=startup))
        if self.port is not None:
            values["gateway.port"] = self.port
        return nested(values)

    def _composed_values(self, values):
        values = resolve_paths(values, self.home)
        record = self.home / "current/installed.json"
        if record.exists() and "memory" not in json.loads(record.read_text())["modules"]:
            values["memory"].update(text_embedding_model="lexical-only", text_embedding_dim=256,
                                    image_embedding_model="disabled", image_embedding_dim=512)
        values["derived"] = derived_values(values, self.asset_root)
        return values

    def compose(self):
        values = self._composed_values(self.effective(startup=True))
        write_toml(self.home / "run/config.toml", values)
        return values

    def write_profile(self, name, values):
        self._write_profile(name, validate_business_values(values, self.code_root))

    def _write_profile(self, name, checked):
        specs = specifications(self.code_root)
        base_defaults = defaults(self.code_root)
        base_defaults["gateway"]["port"] = self.port
        resolved_defaults = flatten(resolve_paths(base_defaults, self.home))
        changed = {key: val for key, val in checked.items()
                   if val != specs[key]["default"] and val != resolved_defaults[key] and key != "gateway.port"}
        write_toml(self.profile_path(name), nested(changed))

    def create(self, name, source=None):
        path = self.profile_path(name)
        if path.exists():
            raise ValueError("配置方案已存在：" + name)
        self._write_profile(name, self.profile_values(source) if source else {})
        return self.catalog(name)

    def save(self, updates, name=None):
        if not isinstance(updates, dict):
            raise ValueError("配置 values 必须是对象。")
        if "gateway.port" in updates:
            raise ValueError("业务端口请通过设备管理设置修改。")
        name = name or self.active()
        before = self.profile_values(name)
        values = dict(before)
        specs = specifications(self.code_root)
        updates = {key: val for key, val in updates.items()
                   if not (specs.get(key, {}).get("sensitive") and isinstance(val, str) and val.strip() in ("", "********"))}
        updates = validate_business_values(updates, self.code_root)
        values.update(updates)
        changed = sorted(key for key, val in updates.items() if before.get(key, specs[key]["default"]) != val)
        active = name == self.active()
        if active:
            effective = flatten(defaults(self.code_root))
            effective.update(values)
            effective["gateway.port"] = self.port
            composed = self._composed_values(nested(effective))
        self._write_profile(name, values)
        if active:
            write_toml(self.home / "run/config.toml", composed)
        hot = [key for key in changed if specs[key]["apply"] == "hot"]
        restart = [key for key in changed if specs[key]["apply"] == "restart"]
        return {"saved": changed, "hot_keys": hot, "restart_keys": restart,
                "restart_required": active and bool(restart), "is_active": active}

    def activate(self, name):
        values = self._composed_values(self.effective(name))
        atomic_write(self.selector, (name + "\n").encode("utf-8"))
        write_toml(self.home / "run/config.toml", values)
        return values

    def delete(self, name):
        if name == self.active():
            raise ValueError("请先切换到其它方案再删除当前方案")
        self.profile_path(name).unlink()
        return self.catalog()

    def catalog(self, name=None):
        return {"profiles": sorted(path.stem for path in self.directory.glob("*.toml")),
                "selected_profile": self.active() if self.selector.exists() else "",
                "profile": name or (self.active() if self.selector.exists() else "")}

    def install(self, name, values):
        checked = validate_business_values(values, self.code_root)
        effective = flatten(defaults(self.code_root))
        effective.update(checked)
        effective["gateway.port"] = self.port
        composed = self._composed_values(nested(effective))
        self._write_profile(name, checked)
        atomic_write(self.selector, (name + "\n").encode("utf-8"))
        return composed

    def request(self, operation, **parameters):
        """管理探针在本业务版本解释器中执行配置操作。"""
        if operation == "runtime":
            from scripts.module_installer.runtime import write_runtime
            return write_runtime(self.code_root, Path(parameters["install_prefix"]), parameters["modules"],
                                 parameters["robot"], parameters["values"])
        if operation == "schema":
            return self.settings_schema(parameters.get("profile"))
        if operation == "save":
            values = parameters["values"]
            if parameters.get("text_values"):
                specs = specifications(self.code_root)
                values = dict(values)
                for key, raw in values.items():
                    spec = specs.get(key)
                    if spec and spec["type"] != "string" and raw not in spec.get("special_values", ()):
                        values[key] = json.loads(raw)
            return self.save(values, parameters.get("profile"))
        if operation == "create":
            return self.create(parameters["name"], parameters.get("source"))
        if operation == "delete":
            return self.delete(parameters["name"])
        if operation == "activate":
            self.activate(parameters["name"])
            return self.catalog()
        if operation == "install":
            return self.install(parameters["name"], parameters["values"])
        if operation == "existing":
            from scripts.import_legacy_config import import_legacy
            report = import_legacy(self.home)
            values = self.profile_values(self.active()) if self.selector.exists() else {}
            return {"values": values, "import": report}
        if operation == "snapshot":
            name = self.active()
            return {"profile": name, "values": self.profile_values(name)}
        if operation == "read":
            return flatten(read_config(self.home))
        if operation == "effective":
            return flatten(self.effective())
        if operation == "existing_values":
            return flatten(self.effective()) if self.selector.exists() else {}
        if operation == "prepare":
            values = flatten(defaults(self.code_root))
            values.update(parameters["values"])
            return flatten(resolve_paths(nested(values), self.home))
        if operation == "secrets":
            specs = specifications(self.code_root)
            values = parameters["values"] if "values" in parameters else flatten(read_config(self.home))
            return [val for key, val in values.items()
                    if specs.get(key, {}).get("sensitive") and isinstance(val, str) and len(val) >= 4]
        if operation == "catalog":
            return self.catalog()
        raise ValueError("未知配置操作：" + operation)

    def settings_schema(self, name=None):
        name = name or self.active()
        values = flatten(self.effective(name))
        sections = []
        for section, entries in schema(self.code_root).items():
            fields = []
            for key, spec in entries.items():
                full = section + "." + key
                kind = {"string": "str", "integer": "int", "boolean": "bool", "array": "csv"}.get(spec["type"], spec["type"])
                displayed = "" if spec.get("sensitive") else values[full]
                if isinstance(displayed, list):
                    displayed = ",".join(displayed)
                label = spec["description"].split("。", 1)[0]
                fields.append(dict(spec, key=full, zh_name=label, label=label, type=kind,
                                   sensitive=spec.get("sensitive", False), value=displayed,
                                   default_value=spec["default"], apply_mode=spec["apply"],
                                   is_set=bool(values[full]) if spec.get("sensitive") else False,
                                   apply_scope="保存后即时生效" if spec["apply"] == "hot" else "下次业务启动生效",
                                   target_path=str(self.profile_path(name)), readonly=full == "gateway.port"))
            for item in fields:
                if item["key"] in ("geometry.pgo_options", "geometry.pointlio_options"):
                    filename = item["key"].split(".", 1)[1] + ".json"
                    item["options_schema"] = json.loads((self.code_root / "contracts" / filename).read_text())["parameters"]
            sections.append({"section_id": section, "section_title": SECTION_TITLES[section], "fields": fields})
        context = self.catalog(name)
        context.update(is_active=name == self.active(), robot=values["robot.driver"])
        return {"sections": sections, "profile_context": context}
