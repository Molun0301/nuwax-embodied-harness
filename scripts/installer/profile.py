# -*- coding: utf-8 -*-
"""
installer profile 持久化与 schema 守门。

设计目标（installer-redesign / task 3）：
- JSON profile 文件统一存放在 ``~/.config/nuwax/installer/<robot>.json`` 默认目录；
  CLI ``--profile`` 支持绝对路径覆盖（plan 6.15 / AC13）。
- 写文件强制带三元组 ``{_schema_version, created_at, installer_version}``（评审 Architect 增强 2）。
- 读取时校验 ``_schema_version``：缺失或 major 版本不兼容 → 抛
  ``InstallerError(code=6, ...)``，**禁止** silent fallback（plan RK17 + iter2 P1-5 PASS）。
- 内置 ``_KNOWN_FLAGS`` 集合：枚举 ``scripts/bootstrap.py:597`` ``BootstrapPlanner`` 字段 +
  ``scripts/bootstrap.py:1522-1536`` ``add_boolean_flag`` 调用清单 + 3 个 choice 字段
  （memory_mode / perception_mode / mcp_mode），作为 CI 守门（评审增强 2）。

本文件 **不** 依赖第三方库；仅依赖标准库 ``json`` / ``os`` / ``pathlib`` / ``datetime`` /
``tempfile`` + 项目内 ``scripts.installer.errors``（task 1 已定义 ``InstallerError`` /
``InstallerErrorCode``，code=6 = profile schema 不兼容）。
"""

from __future__ import annotations

import json
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Union

# 复用 task 1 errors.py 中定义的错误类型；不依赖时降级为占位类，便于单文件独立编译。
try:
    from scripts.installer.errors import InstallerError, InstallerErrorCode
except Exception:  # pragma: no cover - 兼容 task 1 尚未落地的过渡期
    class InstallerErrorCode(object):  # type: ignore[no-redef]
        PROFILE_SCHEMA_INCOMPATIBLE = 6

    class InstallerError(Exception):  # type: ignore[no-redef]
        def __init__(self, code: int, message: str, *, remediation: str = "") -> None:
            super().__init__(message)
            self.code = code
            self.remediation = remediation


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

#: profile schema 当前版本（major.minor.patch）。major 变更 = 不向后兼容，禁止 silent fallback。
SCHEMA_VERSION: str = "1.0.0"

#: 当前支持的 schema major 版本白名单。读取时若 major 不在集合内 → InstallerError(6)。
SUPPORTED_MAJOR_VERSIONS = frozenset({1})

#: installer 自身版本号（写入 profile 的 installer_version 字段，与 _schema_version 配合做兼容门）。
INSTALLER_VERSION: str = "0.1.0"

#: profile 默认根目录（plan 6.15 / AC13 硬编码断言）。
DEFAULT_PROFILE_DIR: str = os.path.join("~", ".config", "nuwax", "installer")

#: 文件名模板，``<robot>`` 占位符在 ``default_profile_path()`` 中替换。
DEFAULT_PROFILE_FILENAME_TEMPLATE: str = "{robot}.json"


# ---------------------------------------------------------------------------
# _KNOWN_FLAGS：BootstrapPlanner 字段守门
# ---------------------------------------------------------------------------
#
# 来源：``scripts/bootstrap.py:597 BootstrapPlanner.__init__`` + ``scripts/bootstrap.py:1522-1536``
# ``add_boolean_flag`` 调用清单 + 3 个 choice 字段。
#
# 该集合同时承担两个职责：
# 1. **写文件校验**：save() 断言所有 keys 都在集合内（schema 漂移守门）。
# 2. **CI 守门**：CI 步骤 ``from scripts.installer.profile import _KNOWN_FLAGS`` 后比对
#    ``set(BootstrapPlanner.__init__ 字段) == _KNOWN_FLAGS``，缺一个就 fail。
#
# 当前共 14 个 boolean flag：
#   geometry / data_plane / systemd / relay / install_nuwaclaw_docker / install_python /
#   prepare_memory / prepare_yolo / export_yolo_engine / build_geometry / build_go2slam /
#   install_geometry_deps / start_gateway / healthcheck
# 加上 memory_mode / perception_mode / mcp_mode / nuwaclaw_host_mount / tier / robot / health_url
# 共 21 个；本集合按此 21 字段收敛。

_KNOWN_FLAGS = frozenset(
    {
        # === robot / tier ===
        "robot",
        "tier",
        # === 14 个 boolean flag ===
        "geometry",
        "data_plane",
        "systemd",
        "relay",
        "install_nuwaclaw_docker",
        "install_python",
        "prepare_memory",
        "prepare_yolo",
        "export_yolo_engine",
        "build_geometry",
        "build_go2slam",
        "install_geometry_deps",
        "start_gateway",
        "healthcheck",
        # === 4 个 choice 字段 ===
        "memory_mode",
        "perception_mode",
        "mcp_mode",
        "nuwaclaw_host_mount",
        # === 字符串 / 路径类（bootstrap.py:630-642 字段）===
        "health_url",
    }
)

_PROFILE_HEADER_FIELDS = frozenset(
    {"_schema_version", "created_at", "installer_version"}
)


# ---------------------------------------------------------------------------
# 路径解析
# ---------------------------------------------------------------------------

def default_profile_dir() -> str:
    """返回 profile 默认根目录（绝对路径），plan 6.15 / AC13 硬编码断言目标。

    Returns:
        ``~/.config/nuwax/installer/`` 绝对路径（末尾含 ``/``，便于 os.path.join）。
    """
    return os.path.expanduser(DEFAULT_PROFILE_DIR) + os.sep


def default_profile_path(robot: str) -> str:
    """返回指定 robot 的默认 profile 路径。``<robot>.json``，存放在默认目录。

    Args:
        robot: 机器人 slug（go2 / r1 / ...），与 ``scripts/bootstrap.py:651`` 同源。

    Returns:
        绝对路径字符串。
    """
    if not robot or not robot.strip():
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message="robot 不能为空，无法解析默认 profile 路径",
            remediation="调用 default_profile_path(robot) 时传入非空 robot slug",
        )
    return os.path.join(default_profile_dir(), DEFAULT_PROFILE_FILENAME_TEMPLATE.format(robot=robot.strip().lower()))


# ---------------------------------------------------------------------------
# 校验
# ---------------------------------------------------------------------------

def _split_semver(version: str) -> tuple:
    """解析 "1.2.3" → (1, 2, 3)；解析失败抛 InstallerError(6)。"""
    parts = version.strip().split(".")
    if len(parts) < 1 or not parts[0].isdigit():
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message=f"无法解析 _schema_version: {version!r}",
            remediation="确保 profile._schema_version 是 semver 字符串（如 '1.0.0'）",
        )
    try:
        return tuple(int(p) for p in parts)
    except ValueError as exc:
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message=f"无法解析 _schema_version: {version!r}（{exc}）",
            remediation="确保 profile._schema_version 是 semver 字符串（如 '1.0.0'）",
        ) from exc


def _utc_now_iso() -> str:
    """返回 UTC ISO8601 时间戳字符串，精度到秒（``2026-06-27T08:00:00+00:00``）。"""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S%z")


def _validate_profile(data: Dict[str, Any]) -> None:
    """profile 读取校验（内部）。

    规则（plan RK17 + spec ``profile 持久化与 schema 守门`` Scenario）：
    - ``_schema_version`` 缺失 → InstallerError(6)
    - ``_schema_version`` major 不在 ``SUPPORTED_MAJOR_VERSIONS`` → InstallerError(6)
    - **不**做字段级 default 填补（**禁止** silent fallback）
    """
    if not isinstance(data, dict):
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message=f"profile 顶层必须是 JSON object，实际为 {type(data).__name__}",
            remediation="重新运行 wizard 生成新 profile（python3 install.py wizard --save-profile <path>）",
        )
    if "_schema_version" not in data:
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message="profile 缺少 _schema_version 字段",
            remediation="重新运行 wizard 生成新 profile（python3 install.py wizard --save-profile <path>）",
        )
    raw_version = data["_schema_version"]
    if not isinstance(raw_version, str) or not raw_version.strip():
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message=f"profile._schema_version 必须是字符串，实际为 {type(raw_version).__name__}",
            remediation="重新运行 wizard 生成新 profile（python3 install.py wizard --save-profile <path>）",
        )
    parsed = _split_semver(raw_version)
    if parsed[0] not in SUPPORTED_MAJOR_VERSIONS:
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message=(
                f"profile schema 不兼容：major 版本 {parsed[0]} 不在 "
                f"{sorted(SUPPORTED_MAJOR_VERSIONS)} 之内（当前文件版本 {raw_version}，"
                f"installer 支持 {SCHEMA_VERSION}）"
            ),
            remediation=(
                "请重新运行 wizard 生成新 profile：python3 install.py wizard --save-profile <path>。"
                "若需要降级到旧 installer，请用旧版本重建 profile。"
            ),
        )

    unknown_keys = sorted(set(data) - _KNOWN_FLAGS - _PROFILE_HEADER_FIELDS)
    if unknown_keys:
        raise InstallerError(
            code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
            message=f"profile 包含未知业务字段 {unknown_keys}，拒绝静默忽略",
            remediation=(
                "删除失效字段后重试，或重新运行 wizard 生成当前 schema 的 profile"
            ),
        )


# ---------------------------------------------------------------------------
# Profile 类（核心 API）
# ---------------------------------------------------------------------------

class Profile:
    """profile 读写门面。

    用法::

        p = Profile()
        p.save("/tmp/go2.json", {"robot": "go2", "tier": "full", ...})
        data = p.load("/tmp/go2.json")  # 校验 _schema_version；不兼容抛 InstallerError(6)
    """

    #: 当前实例的 schema 版本（写入时使用）
    schema_version: str = SCHEMA_VERSION

    #: 当前实例的 installer 版本（写入时使用）
    installer_version: str = INSTALLER_VERSION

    def save(
        self,
        path: Union[str, os.PathLike],
        fields: Optional[Dict[str, Any]] = None,
        *,
        ensure_dir: bool = True,
    ) -> str:
        """写入 profile 到指定路径；强制带三元组 header。

        Args:
            path: 输出路径。``~`` 会自动 expanduser。
            fields: 业务字段（如 ``{"robot": "go2", "tier": "full"}``）。未传入则只写 header。
            ensure_dir: True 时自动创建父目录（默认 True，符合"wizard 保存即用"UX）。

        Returns:
            写入的绝对路径字符串。

        Raises:
            InstallerError: 当 fields 包含 ``_KNOWN_FLAGS`` 之外的 key 时（schema 漂移守门）。
        """
        if fields is None:
            fields = {}

        if not isinstance(fields, dict):
            raise InstallerError(
                code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
                message=f"fields 必须是 dict，实际为 {type(fields).__name__}",
                remediation="调用 Profile().save(path, fields) 时传入 dict",
            )

        # schema 守门：所有 key 必须在 _KNOWN_FLAGS 内（_schema_version / created_at /
        # installer_version 三个 header 字段由本类自己注入，不参与此校验）。
        unknown_keys = sorted(set(fields.keys()) - _KNOWN_FLAGS)
        if unknown_keys:
            raise InstallerError(
                code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
                message=(
                    f"profile 包含 _KNOWN_FLAGS 之外的字段 {unknown_keys}，"
                    f"疑似 schema 漂移；拒绝写入"
                ),
                remediation=(
                    "检查 BootstrapPlanner 字段是否新增；若是，请在 "
                    "scripts/installer/profile.py 的 _KNOWN_FLAGS 集合中追加新 key"
                ),
            )

        target = Path(os.path.expanduser(str(path)))
        if ensure_dir:
            target.parent.mkdir(parents=True, exist_ok=True)

        payload: Dict[str, Any] = {
            "_schema_version": self.schema_version,
            "created_at": _utc_now_iso(),
            "installer_version": self.installer_version,
        }
        payload.update(fields)

        # 原子写：先写到临时文件再 rename（与 state.py 的 os.replace 模式一致）。
        fd, tmp_path = tempfile.mkstemp(
            prefix=".profile-", suffix=".json", dir=str(target.parent) or None
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2, sort_keys=True)
                fh.write("\n")
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp_path, target)
        except Exception:
            # 清理临时文件
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise

        return str(target.resolve())

    def load(self, path: Union[str, os.PathLike]) -> Dict[str, Any]:
        """读取并校验 profile。

        Args:
            path: 输入路径。``~`` 会自动 expanduser。

        Returns:
            解析后的 dict（含 header 三元组 + 业务字段）。

        Raises:
            InstallerError(code=6): 缺失/不兼容 _schema_version / 解析失败 / 顶层非 dict。
            FileNotFoundError: 文件不存在（不吞，让调用方决定是否回退到 wizard）。
        """
        target = Path(os.path.expanduser(str(path)))
        if not target.exists():
            raise FileNotFoundError(f"profile 文件不存在: {target}")

        with target.open("r", encoding="utf-8") as fh:
            try:
                data = json.load(fh)
            except json.JSONDecodeError as exc:
                raise InstallerError(
                    code=InstallerErrorCode.PROFILE_SCHEMA_INCOMPATIBLE,
                    message=f"profile 不是合法 JSON: {exc.msg}（line {exc.lineno} col {exc.colno}）",
                    remediation="重新运行 wizard 生成新 profile（python3 install.py wizard --save-profile <path>）",
                ) from exc

        _validate_profile(data)
        return data

    # 便捷方法 ----------------------------------------------------------

    def save_for_robot(
        self,
        robot: str,
        fields: Dict[str, Any],
        *,
        base_dir: Optional[str] = None,
    ) -> str:
        """保存 profile 到默认目录 ``<base_dir || default_profile_dir>/<robot>.json``。

        ``robot`` 字段若不在 ``fields`` 中会自动注入，便于后续 ``load_profile()`` 校验。
        """
        merged = dict(fields)
        merged.setdefault("robot", robot.strip().lower())
        target_dir = os.path.expanduser(base_dir) if base_dir else default_profile_dir()
        target_path = os.path.join(target_dir, DEFAULT_PROFILE_FILENAME_TEMPLATE.format(robot=merged["robot"]))
        return self.save(target_path, merged)


# ---------------------------------------------------------------------------
# 顶层便捷函数（planner.py / cli.py 直接调用）
# ---------------------------------------------------------------------------

def save_profile(
    path: Union[str, os.PathLike],
    fields: Dict[str, Any],
    *,
    ensure_dir: bool = True,
) -> str:
    """顶层便捷函数：等价于 ``Profile().save(path, fields, ensure_dir=...)``。"""
    return Profile().save(path, fields, ensure_dir=ensure_dir)


def load_profile(path: Union[str, os.PathLike]) -> Dict[str, Any]:
    """顶层便捷函数：等价于 ``Profile().load(path)``。"""
    return Profile().load(path)


__all__ = [
    "SCHEMA_VERSION",
    "SUPPORTED_MAJOR_VERSIONS",
    "INSTALLER_VERSION",
    "DEFAULT_PROFILE_DIR",
    "_KNOWN_FLAGS",
    "InstallerErrorCode",
    "InstallerError",
    "Profile",
    "default_profile_dir",
    "default_profile_path",
    "save_profile",
    "load_profile",
]
