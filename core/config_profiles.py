"""机器人配置方案的持久化；读写和切换共用目录锁与版本检查。"""
from __future__ import annotations

import hashlib
import os
import uuid
from pathlib import Path

from core.config_values import configuration_location, configuration_sources, configuration_values, read_env_values, env_assignment

_PATH_KEYS = frozenset({
    "NUWAX_BOOTSTRAP_ENV_FILE", "GO2_DATA_PLANE_ENV_FILE", "NUWAX_USER_CONFIG_ENV_FILE",
    "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE", "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE",
    "NUWAX_GEOMETRY_EXTRA_ENV_FILES", "NUWAX_SETTINGS_LAUNCHER",
})


class ConfigurationConflict(ValueError):
    """页面读取之后文件已变化，保留客户端编辑以供刷新合并。"""


def read_profile(location, *, root):
    return configuration_values(root=root, robot=location["robot"], profile=location["profile"])


def revision(location, *, root):
    # 尚未迁移的默认方案也绑定原来源版本，避免迁移时覆盖另一页面的新值。
    digest = hashlib.sha256()
    for path in configuration_sources(root=root, robot=location["robot"], profile=location["profile"]):
        digest.update(str(path).encode())
        digest.update(path.read_bytes() if path.is_file() else b"")
    return digest.hexdigest()


def check_revision(location, expected, *, root):
    if expected is not None and expected != revision(location, root=root):
        raise ConfigurationConflict("配置已被其它页面修改，请刷新后重新保存；本页输入仍保留。")


def render_values(values):
    lines = ["# 机器人独立配置；通过控制台保存、另存或选择方案。"]
    lines.extend(env_assignment(key, str(value)) for key, value in sorted(values.items())
                 if key not in _PATH_KEYS and not key.startswith("_NUWAX_"))
    return "\n".join(lines) + "\n"


def ensure_profile(location, *, root):
    from core.config_io import _atomic_write

    if not location["path"].exists():
        _atomic_write(location["path"], render_values(read_profile(location, root=root)))


def profile_catalog(*, root, robot=None, profile=None):
    location = configuration_location(root=root, robot=robot, profile=profile)
    if location is None:
        return None
    directory = root / "drivers" / location["robot"]
    names = {path.stem for path in (directory / "configs").glob("*.env") if path.is_file() and not path.is_symlink()}
    names.add(location["profile"])
    selected = configuration_location(root=root, robot=location["robot"])
    robots = sorted(path.name for path in (root / "drivers").iterdir()
                    if path.is_dir() and not path.is_symlink() and (path / "manifest.py").is_file()
                    and (path / "assembly.py").is_file())
    active = configuration_location(root=root)
    return {"robot": location["robot"], "profile": location["profile"],
            "profiles": sorted(names), "robots": robots, "selected_profile": selected["profile"],
            "selected_robot": active["robot"] if active else "",
            "path": str(location["path"]), "revision": revision(location, root=root),
            "migration_required": not location["path"].exists()}


def create_profile(*, root, robot, source, name, expected_revision=None):
    from core.config_io import _atomic_write, _target_lock

    location = configuration_location(root=root, robot=robot, profile=source)
    target = configuration_location(root=root, robot=robot, profile=name)
    with _target_lock(root / "runtime_data" / "settings" / "transaction"):
        check_revision(location, expected_revision, root=root)
        if target["path"].exists() or target["profile"] == source:
            raise ValueError("同名配置已经存在，请使用其它名称")
        _atomic_write(target["path"], render_values(read_profile(location, root=root)))
    return profile_catalog(root=root, robot=robot, profile=name)


def activate_profile(*, root, robot, profile, expected_revision=None):
    from core import config_io as io

    location = configuration_location(root=root, robot=robot, profile=profile)
    with io._target_lock(root / "runtime_data" / "settings" / "transaction"):
        check_revision(location, expected_revision, root=root)
        values = read_profile(location, root=root)
        # 手工编辑的方案仍须经过同一类型与算法契约校验；失效时不改活动选择。
        for key, value in values.items():
            field = io.R.get_field(key)
            if field is not None and (io._validate(key, value, field.type) is None
                                     or (field.choices and value.strip() not in field.choices)):
                raise ValueError("方案中的配置无效：" + key)
        if "NUWAX_GEOMETRY_PIPELINE" in values:
            from contracts.navigation_pipeline import resolve_navigation_profile
            expected = resolve_navigation_profile(values["NUWAX_GEOMETRY_PIPELINE"])
            if values.get("NUWAX_NAV_PROFILE", expected) != expected:
                raise ValueError("配置方案中的导航与建图执行链不一致")
        ensure_profile(location, root=root)
        io._atomic_write(location["selector"], profile + "\n")
        active = root / "runtime_data" / "settings" / "active.env"
        active.parent.mkdir(parents=True, exist_ok=True)
        temporary = active.with_name(".active-" + uuid.uuid4().hex)
        try:
            temporary.symlink_to(os.path.relpath(location["path"], active.parent))
            os.replace(temporary, active)
        finally:
            if temporary.is_symlink():
                temporary.unlink()
    return profile_catalog(root=root, robot=robot, profile=profile)
