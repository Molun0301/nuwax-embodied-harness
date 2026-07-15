"""installer wizard 末尾 banner 输出。

任务范围（task 4 / installer-shared-lock）：

本模块**仅**承担 wizard 完成后的"下一步提示 banner"输出契约（plan 4.5 +
spec.md "health phase 区分 gateway 来源" Scenario + R14）：

    echo "下一步: bash scripts/start_gateway.sh start 启动 gateway"

完整 ``InstallPlanner``（参数互斥校验 / 默认值 / 摘要）由 task 1
``installer-shell-dispatcher`` 落地的骨架补全；本任务**不**重写 task 1 范围。
为防止 task 1 落地时与本文件的 banner 输出契约冲突，banner 内容集中到
``WIZARD_NEXT_STEP_BANNER`` 常量 + ``print_wizard_next_step_banner()`` 函数，
task 1 接入 wizard 完成流程时调用此函数即可。
"""

from __future__ import annotations

import os
import platform
from dataclasses import asdict, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

# wizard 完成后向用户输出的下一步提示（plan 4.5 / R14 + spec Scenario "wizard 末尾 banner"）。
# 强制写死字符串内容，不允许从配置/i18n 注入（installer 边界内，避免 banner 漂移）。
WIZARD_NEXT_STEP_BANNER: str = (
    "下一步: bash scripts/start_gateway.sh start 启动 gateway"
)


def wizard_next_step_banner() -> str:
    """返回 wizard 完成后的下一步提示字符串。

    Returns:
        固定 banner 文本（见模块顶部 ``WIZARD_NEXT_STEP_BANNER``）。

    Note:
        纯函数，便于 task 1 wizard 完成流程在 dry-run / 非交互 / 交互三模式下
        一致调用；调用方负责 ``print(..., flush=True)``。
    """
    return WIZARD_NEXT_STEP_BANNER


def print_wizard_next_step_banner() -> None:
    """向 stdout 输出 wizard 下一步提示 banner（行尾换行）。"""
    print(WIZARD_NEXT_STEP_BANNER, flush=True)


# ---------------------------------------------------------------------------
# task 7 (installer-yolo-engine-default) 增量：YOLO-World TensorRT engine 默认映射
# ---------------------------------------------------------------------------
#
# 设计目标（plan RK10 + spec Requirement "YOLO-World TensorRT 引擎默认导出"）：
# 1. tier=full 默认 export_yolo_engine=True（必需：Jetson 实时推理依赖 engine）；
# 2. tier=lite 默认 export_yolo_engine=False（仅下载 .pt，节省 5-10 分钟）；
# 3. x86_64 dev 机无 GPU（torch.cuda.is_available() == False） → 自动 fallback False，
#    并在 wizard / summary 标注"x86_64 + 无 GPU → 跳过 engine 导出"。
#
# 向后兼容（plan 4.3 + 评审要点 3）：
# - **禁止**直接改 ``scripts/bootstrap.py:622`` 的 ``default=False``；
# - ``scripts/bootstrap.py`` 行为零变化；installer 在自身 dataclass 内部做默认映射。
# - 详细根因：memory ``yolo-world-engine-zero-detection-defect.md``
#   （YOLOWorld 类无法加载 TensorRT .engine，需走 .pt 路径）。

# tier → 默认值映射表（AC10 显式要求）
TIER_TO_EXPORT_YOLO_ENGINE: Dict[str, bool] = {
    "full": True,
    "lite": False,
}

# x86_64 dev 机架构集合（fallback 触发条件之一）
HOST_ARCH_X86_64: Tuple[str, ...] = ("x86_64", "amd64")


def _detect_gpu_available() -> bool:
    """探测当前主机是否有可用 CUDA GPU。

    返回 True 当且仅当：
      1. ``torch`` 模块可导入；
      2. ``torch.cuda.is_available()`` 为 True。

    任一条件不满足 → 返回 False（x86_64 dev 机、Jetson 未装 torch、虚拟环境）
    都属于"无 GPU"语境。``torch`` 在 Jetson Orin 实机默认预装（PyTorch wheel），
    但 ``import torch`` 在无 wheel 的开发机可能抛 ``ImportError``，属于预期路径。
    """
    try:
        import torch  # type: ignore[import-not-found]
    except ImportError:
        return False
    try:
        return bool(torch.cuda.is_available())
    except Exception:
        # 部分 torch 编译版本在没有 CUDA 驱动时会抛 RuntimeError
        return False


def _detect_host_arch() -> str:
    """返回当前主机架构（小写）；优先用 ``platform.machine()``。"""
    try:
        return platform.machine().lower() or "unknown"
    except Exception:
        return "unknown"


def _resolve_export_yolo_engine_default(tier: str) -> tuple[bool, str]:
    """根据 tier + 主机 GPU 探测决定 export_yolo_engine 默认值。

    Returns:
        ``(value, note)`` 二元组：
        - ``value``：bool 默认值（True=导 engine，False=仅下 .pt）；
        - ``note``：中文解释字符串，写入 wizard / summary 用。

    Note:
        **不**写回任何 InstallPlanner 字段；调用方（``InstallPlanner.apply_tier_defaults``）
        自行决定是否覆盖（保留用户显式选择）。
    """
    tier_norm = str(tier or "").strip().lower() or "lite"
    gpu_available = _detect_gpu_available()
    host_arch = _detect_host_arch()

    base_default = TIER_TO_EXPORT_YOLO_ENGINE.get(tier_norm, False)
    if tier_norm == "full" and not gpu_available:
        # x86_64 dev 机无 GPU fallback（D5 缓解 + AC10 隐含语义）
        if host_arch in HOST_ARCH_X86_64:
            note = "x86_64 + 无 GPU → 跳过 engine 导出"
        else:
            note = f"{host_arch} + 无 CUDA GPU → 跳过 engine 导出"
        return False, note
    if tier_norm == "full":
        return True, "tier full + GPU 可用 → 默认导出 engine"
    return False, "tier lite → 默认不导出 engine"


# ---------------------------------------------------------------------------
# task 3 (installer-profile-version) 增量：profile 读写薄包装
# ---------------------------------------------------------------------------
#
# 设计：planner.py 既是 wizard banner 的家（task 4），也是 task 3 profile 读写 API 的家。
# 写文件强制走 ``scripts/installer/profile.py`` 的 ``Profile`` 门面，schema 守门
# （_schema_version / _KNOWN_FLAGS）由 profile.py 单点维护，planner 仅做薄包装。
#
# ``InstallPlanner`` 是 dataclass：字段**有意对齐** ``scripts/bootstrap.py:597 BootstrapPlanner.__init__``
# + ``scripts/bootstrap.py:1522-1536`` ``add_boolean_flag`` 调用清单 + 3 个 choice 字段
# （memory_mode / perception_mode / mcp_mode）+ nuwaclaw_host_mount + tier / robot / health_url。
# 任何新增字段必须同步更新 ``scripts/installer/profile.py`` 的 ``_KNOWN_FLAGS`` 集合，
# 否则 ``save_profile()`` 抛 ``InstallerError(code=6, ...)``。
#
# 注意：保持与现有 banner 输出契约**完全向后兼容**（task 4 不破坏）。

from scripts.installer.profile import (  # noqa: E402  (task 3 增量，依赖 task 3 已落地的 profile.py)
    Profile,
    _KNOWN_FLAGS,
    default_profile_path,
    save_profile as _save_profile_to_path,
    load_profile as _load_profile_from_path,
)


def save_profile(
    path: Union[str, os.PathLike],
    planner: "InstallPlanner",
    *,
    ensure_dir: bool = True,
) -> str:
    """把 ``InstallPlanner`` 序列化为 profile JSON 写入 ``path``（薄包装）。

    写文件强制走 ``scripts.installer.profile.Profile.save()``，自动注入
    ``_schema_version`` / ``created_at`` / ``installer_version`` 三元组（评审 Architect 增强 2）。
    ``Profile.save()`` 会用 ``_KNOWN_FLAGS`` 做 schema 守门——planner 新增字段若
    未在 ``_KNOWN_FLAGS`` 中登记 → 抛 ``InstallerError(code=6)``，**禁止** silent fallback
    （plan RK17 + iter2 P1-5 PASS）。

    Args:
        path: 输出路径（支持 ``~``）。
        planner: ``InstallPlanner`` 实例。
        ensure_dir: 是否自动创建父目录（默认 True，wizard UX）。

    Returns:
        写入的绝对路径字符串。
    """
    fields: Dict[str, Any] = asdict(planner)
    return _save_profile_to_path(path, fields, ensure_dir=ensure_dir)


def load_profile(
    path: Union[str, os.PathLike],
    *,
    planner_cls: type = None,
) -> "InstallPlanner":
    """从 profile JSON 反序列化为 ``InstallPlanner``（薄包装）。

    Args:
        path: 输入路径（支持 ``~``）。
        planner_cls: 目标 dataclass 类型（默认 ``InstallPlanner``，便于单测注入 mock）。

    Returns:
        填充好字段的 ``InstallPlanner`` 实例。header 三元组
        （``_schema_version`` / ``created_at`` / ``installer_version``）不进入 planner；
        未知业务字段由 ``Profile.load()`` 在构造 planner 前拒绝。

    Raises:
        InstallerError(code=6): ``_schema_version`` 缺失、major 不兼容或含未知业务字段。
        FileNotFoundError: 文件不存在。
    """
    if planner_cls is None:
        planner_cls = InstallPlanner
    data = _load_profile_from_path(path)
    fields = {k: v for k, v in data.items() if k in _KNOWN_FLAGS}
    return planner_cls(**fields)


def profile_path_for(robot: str) -> str:
    """返回 ``robot`` 的默认 profile 路径（薄包装 ``default_profile_path``）。"""
    return default_profile_path(robot)


# ---------------------------------------------------------------------------
# InstallPlanner：最小 dataclass 骨架
# ---------------------------------------------------------------------------
#
# 字段集合**严格**收敛到 ``profile._KNOWN_FLAGS`` 集合；任何新增字段必须同步
# 更新 ``scripts/installer/profile.py`` 的 ``_KNOWN_FLAGS``（plan RK17 + iter2 P1-5 PASS）。
# task 1 后续填充默认值 / summary / 参数互斥校验逻辑时，请勿引入新字段而不更新守门集合。

from dataclasses import dataclass  # noqa: E402  (放后面避免与 banner 段混在一起)


# ---------------------------------------------------------------------------
# task 1 增量：tier 默认值映射 + argparse bool 解析辅助
# ---------------------------------------------------------------------------
#
# 这两个常量/函数不属于 ``InstallPlanner`` 数据类本身：TIER_DEFAULTS 是模块级
# 字典（profile / planner / phase 决策三处复用），coerce_bool_flag 是给 cli.py
# 解析 ``--no-xxx`` / ``--xxx`` 双标记用的纯函数。
#
# 注意：``coerce_bool_flag`` 不引入新字段，仅做类型归一化，避免 cli.py 重复
# 同样的 ``in {"1", "true", ...}`` 逻辑。

TIER_DEFAULTS: Dict[str, bool] = {"full": True, "lite": False}


def coerce_bool_flag(raw: Any, default: Optional[bool] = None) -> Optional[bool]:
    """把 argparse 的 ``None`` / ``True`` / ``False`` / 字符串统一为 ``True|False|None``。

    Args:
        raw: argparse ``add_boolean_flag`` 产生的值（可能为 ``None`` / bool / 字符串）。
        default: ``raw is None`` 时返回的默认值；缺省返回 ``None``。

    Returns:
        归一化后的布尔值（缺省时返回 ``default``）。
    """
    if raw is None:
        return default
    if isinstance(raw, bool):
        return raw
    text = str(raw).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    return default


@dataclass
class RuntimeOptions:
    """installer 运行时选项（task 1 增量，不进入 ``InstallPlanner``）。

    这些字段不参与 profile 序列化（plan RK17 + ``profile._KNOWN_FLAGS`` 守门）：
    * ``dry_run``：所有副作用仅打印命令不执行。
    * ``offline``：拒绝任何网络入口（apt / pip / docker / curl / wget）。
    * ``non_interactive``：跳过 wizard 提问，全部走默认值。
    * ``profile_path``：wizard 保存 / apply 读取的 profile JSON 路径。
    * ``phase_overrides``：``--phase-override <phase>=<exit-code>`` 解析结果。
    * ``save_profile_path``：``--save-profile`` 显式指定输出路径；空串表示不保存。
    """

    dry_run: bool = False
    offline: bool = False
    non_interactive: bool = False
    profile_path: str = ""
    save_profile_path: str = ""
    phase_overrides: Dict[str, Any] = field(default_factory=dict)


@dataclass
class InstallPlanner:
    """installer 部署规划数据类（task 3 最小骨架，task 1 后续扩展）。

    字段含义对照 ``scripts/bootstrap.py:597 BootstrapPlanner.__init__`` + ``add_boolean_flag``
    调用清单 + 3 个 choice（memory_mode / perception_mode / mcp_mode）。
    """

    # === robot / tier ===
    robot: str = "go2"
    tier: str = "full"

    # === 14 个 boolean flag ===
    geometry: bool = True
    data_plane: bool = True
    systemd: bool = False
    relay: bool = False
    install_nuwaclaw_docker: bool = False
    install_python: bool = True
    prepare_memory: bool = False
    prepare_yolo: bool = True
    export_yolo_engine: bool = False
    build_geometry: bool = True
    build_go2slam: bool = True
    install_geometry_deps: bool = True
    start_gateway: bool = False
    healthcheck: bool = False

    # === 4 个 choice 字段 ===
    memory_mode: str = "cloud"
    perception_mode: str = "hybrid"
    mcp_mode: str = "auto"
    nuwaclaw_host_mount: str = "none"

    # === 字符串 / 路径类 ===
    health_url: str = "http://127.0.0.1:8766/api/health"

    # === task 1 增量：默认值 / 联动 / 校验 方法（不新增字段以兼容 profile._KNOWN_FLAGS 守门）===
    # 字段集合**严格**收敛到上面的 21 项；任何新增字段必须同步更新
    # ``scripts/installer/profile.py`` 的 ``_KNOWN_FLAGS`` 集合（plan RK17）。
    # task 1 需要的 dry_run / offline / non_interactive / profile_path / phase_overrides
    # 等运行时标志由 cli.py 维护为模块级 dict（RuntimeOptions），不进入 InstallPlanner。

    def apply_tier_defaults(self) -> tuple[bool, str]:
        """根据 ``tier`` + GPU 探测写入 ``export_yolo_engine`` 默认值（task 7 增量）。

        返回 ``(value, note)`` 元组；note 是中文解释，写入 wizard / summary 用，
        **不**进入 InstallPlanner 字段（避免破坏 profile._KNOWN_FLAGS schema 守门，
        plan RK17 + iter2 P1-5 PASS）。

        规则（plan RK10 + spec Requirement "YOLO-World TensorRT 引擎默认导出"）：
        - tier=full + 有 GPU（torch.cuda.is_available()） → True
        - tier=full + 无 GPU（x86_64 dev 机） → False + 标注 fallback 原因
        - tier=lite → False + 标注"tier lite → 默认不导出 engine"

        当且仅当 export_yolo_engine 仍为 dataclass 默认 False 时才重算；profile
        显式覆盖或 wizard 显式给 True 时保留用户选择。

        调用顺序：构造 dataclass 之后、validate 之前。
        与 plan AC10 对齐：tier=full → True；tier=lite → False。
        """
        value, note = _resolve_export_yolo_engine_default(self.tier)
        if not self.export_yolo_engine:
            self.export_yolo_engine = value
        return self.export_yolo_engine, note

    def validate_mutex(self) -> list:
        """参数互斥校验；返回 warning 列表（不抛错，由 cli 决定打印）。

        规则（plan AC15）：
        * ``--robot go2`` 时默认 ``data_plane=true`` + ``geometry=true``；若显式关闭则 warning。
        * 非 go2 时强制 ``data_plane=false`` 并 warning（不支持 data_plane）。
        * 非 go2 时 ``geometry`` 保留用户选择（可能仍为 True）。
        """
        warnings: list = []
        if self.robot == "go2":
            if not self.data_plane:
                warnings.append(
                    "已显式 --no-data-plane；将覆盖 --robot go2 默认 data_plane=true。"
                )
            if not self.geometry:
                warnings.append(
                    "已显式 --no-geometry；将覆盖 --robot go2 默认 geometry=true。"
                )
        else:
            if self.data_plane:
                self.data_plane = False
                warnings.append(
                    f"机器人 {self.robot} 不支持 data_plane，已强制设置为 false。"
                )
        return warnings

    def summary(self) -> str:
        """多行中文摘要，便于 wizard 末尾 / --dry-run 展示。"""
        lines = [
            f"机器人: {self.robot}    档位: {self.tier}",
            f"data_plane: {self.data_plane}    geometry: {self.geometry}",
            f"memory_mode: {self.memory_mode}    perception_mode: {self.perception_mode}",
            f"mcp_mode: {self.mcp_mode}    nuwaclaw_host_mount: {self.nuwaclaw_host_mount}",
            (
                f"export_yolo_engine: {self.export_yolo_engine}    "
                f"prepare_yolo: {self.prepare_yolo}    prepare_memory: {self.prepare_memory}"
            ),
            (
                f"build_geometry: {self.build_geometry}    "
                f"build_go2slam: {self.build_go2slam}    "
                f"install_geometry_deps: {self.install_geometry_deps}"
            ),
            (
                f"systemd: {self.systemd}    relay: {self.relay}    "
                f"install_nuwaclaw_docker: {self.install_nuwaclaw_docker}"
            ),
        ]
        return "\n".join(lines)


__all__ = [
    "WIZARD_NEXT_STEP_BANNER",
    "wizard_next_step_banner",
    "print_wizard_next_step_banner",
    "InstallPlanner",
    "RuntimeOptions",
    "TIER_DEFAULTS",
    "coerce_bool_flag",
    "save_profile",
    "load_profile",
    "profile_path_for",
    # task 7 (installer-yolo-engine-default) 增量
    "TIER_TO_EXPORT_YOLO_ENGINE",
    "HOST_ARCH_X86_64",
    "_resolve_export_yolo_engine_default",
    "_detect_gpu_available",
    "_detect_host_arch",
]
