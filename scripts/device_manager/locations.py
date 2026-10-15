"""Stable manager entry and the first-install payload directory, without moving files."""
from __future__ import annotations

import os
from pathlib import Path

from scripts.neh_bootstrap import INSTALL_DIRECTORY_NAME

from .common import ManagerError, atomic_json, read_json


def manager_home(home):
    from scripts.neh_bootstrap import BootstrapError, resolve_manager_home
    try:
        return resolve_manager_home(home)
    except BootstrapError as exc:
        raise ManagerError("install_location_invalid", str(exc), 409) from exc


def installation_home(home):
    value = read_json(home / "config/install-location.json", {}).get("install_dir")
    if value is None:
        return home
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise ManagerError("install_location_invalid", "安装位置记录无效。", 500)
    return Path(value).resolve()


def directory_path(value):
    if not isinstance(value, str) or not value.strip() or any(ord(char) < 32 for char in value):
        raise ManagerError("invalid_install_dir", "请输入设备上的完整目录路径。")
    try:
        path = Path(value.strip()).expanduser()
        if not path.is_absolute():
            raise ValueError()
        return path.resolve()
    except (OSError, RuntimeError, ValueError) as exc:
        raise ManagerError("invalid_install_dir", "请使用绝对路径或以~/开头的目录路径。") from exc


def selected_directory(value):
    """A Web folder selection is a parent; selecting the NEH folder itself reuses it."""
    parent = directory_path(value)
    if parent.exists() and not parent.is_dir():
        raise ManagerError("invalid_install_dir", "所选路径不是目录。")
    name = Path(os.path.normpath(value.strip())).name
    if name == INSTALL_DIRECTORY_NAME or parent.name == INSTALL_DIRECTORY_NAME:
        return parent
    return directory_path(str(parent / INSTALL_DIRECTORY_NAME))


def prepare_directory(engine, value):
    target = directory_path(value)
    manager = engine.manager_home
    if target != manager and (manager in target.parents or target in manager.parents):
        raise ManagerError("invalid_install_dir", "业务安装目录不能包含管理入口，也不能位于管理入口内部。")
    owner = {"manager_dir": str(manager), "instance_id": engine.instance_id}
    try:
        if target.exists():
            if not target.is_dir():
                raise ManagerError("invalid_install_dir", "所选路径不是目录。")
            marker = target / "config/manager-home.json"
            if target != manager and any(target.iterdir()) and (not marker.is_file() or read_json(marker, {}) != owner):
                raise ManagerError("install_dir_not_empty", "目标文件夹已包含其他内容，无法作为NEH安装目录：" + str(target) + "。请另选父目录，已有文件不会被修改。", 409)
            if target != engine.home and ((target / "current").exists() or (target / "current").is_symlink()):
                raise ManagerError("install_dir_installed", "目标目录已有安装，不能通过选择位置接管或迁移。", 409)
        else:
            target.mkdir(parents=True, mode=0o700)
        if target.stat().st_uid != os.getuid() or target.stat().st_mode & 0o022 or not os.access(str(target), os.W_OK | os.X_OK):
            raise ManagerError("install_dir_permissions", "目录须属于当前设备用户且可写，并且不能允许其他用户写入。", 403)
        if target != manager:
            atomic_json(target / "config/manager-home.json", owner)
    except OSError as exc:
        raise ManagerError("install_dir_unwritable", "无法写入所选目录，请选择当前设备用户可写的位置。", 403) from exc
    return target


def browse_directories(engine, value=None):
    path = directory_path(value) if value else engine.home.parent
    if not value:
        while not path.exists() and path != path.parent:
            path = path.parent
    try:
        if not path.is_dir() or not os.access(str(path), os.R_OK | os.X_OK):
            raise ManagerError("directory_unavailable", "目录不存在或当前设备用户无权浏览。", 403)
        directories = [{"name": child.name, "path": str(child)} for child in path.iterdir()
                       if not child.name.startswith(".") and child.is_dir() and os.access(str(child), os.R_OK | os.X_OK)]
    except OSError as exc:
        raise ManagerError("directory_unavailable", "当前设备用户无权浏览此目录。", 403) from exc
    selection = value if value else str(path)
    return {"path": str(path), "selection_path": selection, "parent": str(path.parent) if path != path.parent else None,
            "directories": sorted(directories, key=lambda item: item["name"].casefold()),
            "writable": os.access(str(path), os.W_OK | os.X_OK), "install_dir": str(selected_directory(selection))}
