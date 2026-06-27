"""installer 阶段 6：YOLO-World 模型 + TensorRT 引擎。

对应 plan 4.1 L116 + RK10 + spec Requirement "YOLO-World TensorRT 引擎默认导出"。
本文件**仅**负责：
1. 读取 `planner.export_yolo_engine`（由 `planner.py` 决定 tier 映射默认）；
2. 调 `scripts.bootstrap.prepare_assets(planner)` 触发 .pt 下载 + TensorRT 导出；
3. dry-run 时打印 "+ <cmd>" 列表，不实际执行。

设计要点：
- **禁止**修改 `scripts/bootstrap.py`（plan 4.3 + 评审要点 3）；
- **禁止**直接传 `--export-yolo-engine` CLI flag 给 `prepare_assets`：
  bootstrap.py:1151-1177 的 prepare_assets 已经按 `planner.export_yolo_engine` 布尔字段
  分支（True 时调 `--no-int8` 走 engine 导出，False 时只 `--download-only`）。
  本 phase 的任务是**为 installer 决定 planner.export_yolo_engine 默认值**，
  之后把 planner 喂给 bootstrap.prepare_assets 即可。
- x86_64 dev 机 fallback：planner.py 已自动把 `export_yolo_engine` 设为 False，
  本 phase 只需打印 note + 调 bootstrap.prepare_assets。

根因记忆：memory `yolo-world-engine-zero-detection-defect.md` —— YOLOWorld 类
无法加载 TensorRT .engine（AttributeError），需走 .pt 路径。本 phase 不解决
engine 加载问题（那是 services/perception 代码范围），仅控制**安装阶段**
是否生成 engine 产物。

字段约束：planner 字段集合**严格**收敛到 ``profile._KNOWN_FLAGS``（plan RK17），
因此 ``export_yolo_engine_note`` / ``gpu_available`` / ``host_arch`` 等运行态信息
**不**进入 InstallPlanner 字段；本 phase 每次 run() / dry_run() 时通过
``InstallPlanner.apply_tier_defaults()`` 返回的 (value, note) 实时获取。
"""

from __future__ import annotations

import sys
from typing import List, Tuple

from .base import Phase
from scripts.installer.planner import _resolve_export_yolo_engine_default


# ---------- phase 实现 ----------


def _resolve_planner_note(planner) -> str:
    """从 planner 拿 export_yolo_engine 的中文 note。

    优先读 ``InstallPlanner._yolo_engine_note``（run() 提前调用
    ``apply_tier_defaults()`` 缓存）；无则实时重算。
    """
    cached = getattr(planner, "_yolo_engine_note", None)
    if cached:
        return cached
    _value, note = _resolve_export_yolo_engine_default(planner.tier)
    return note


class ModelsPhase(Phase):
    """阶段 6：YOLO-World 模型准备 + 可选 TensorRT engine 导出。"""

    name = "models"
    order = 6
    requires: Tuple[str, ...] = ("python_env", "config_env")
    retryable = True

    def run(self, planner, ctx) -> int:
        """执行阶段 6；副作用全部委托给 scripts.bootstrap.prepare_assets。

        planner.export_yolo_engine 的值由 scripts/installer/planner.py:InstallPlanner
        按 tier + GPU 探测自动决定（task 7 主体逻辑在 planner._resolve_export_yolo_engine_default）。
        """
        note = _resolve_planner_note(planner)
        ctx.info(f"[phase: {self.name}] 启动：tier={planner.tier} "
                 f"export_yolo_engine={bool(planner.export_yolo_engine)} "
                 f"({note})")

        # 1. 引导 import scripts.bootstrap（避免模块名冲突）
        try:
            from scripts import bootstrap as _bootstrap  # type: ignore[import-not-found]
        except Exception as exc:  # pragma: no cover - 环境故障
            ctx.error(f"[phase: {self.name}] 无法 import scripts.bootstrap: {exc}")
            return 1

        # 2. 构造 BootstrapPlanner；export_yolo_engine / prepare_yolo 按 installer planner 走
        bootstrap_planner = _bootstrap.BootstrapPlanner.__new__(_bootstrap.BootstrapPlanner)
        # 只填 prepare_assets 真正读到的字段（bootstrap.py:1151-1179 用到的）
        bootstrap_planner.prepare_yolo = True
        bootstrap_planner.export_yolo_engine = bool(planner.export_yolo_engine)
        bootstrap_planner.prepare_memory = False  # 记忆模型由 memory phase 单独跑
        bootstrap_planner.python_bin = sys.executable

        # 3. dry-run：只打印命令不执行
        if getattr(planner, "dry_run", False):
            ctx.info(f"+ [phase: {self.name}] dry-run：以下命令将执行但本模式不实际跑")
            ctx.info(
                f"+ {sys.executable} scripts/prepare_yolo_world_tensorrt.py "
                f"--weights-output {_bootstrap.DEFAULT_YOLO_WORLD_MODEL} "
                f"--engine-output {_bootstrap.DEFAULT_YOLO_WORLD_ENGINE} "
                + ("--no-int8" if planner.export_yolo_engine else "--download-only")
            )
            if note:
                ctx.info(f"+ note: {note}")
            return 0

        # 4. 实际执行（委托给 bootstrap 纯函数）
        try:
            _bootstrap.prepare_assets(bootstrap_planner)
        except Exception as exc:  # 透传 BootstrapError 等
            ctx.error(
                f"[phase: {self.name}] prepare_assets 失败: {exc}。\n"
                f"  建议: 1) 确认 tier={planner.tier!r} 与 export_yolo_engine={planner.export_yolo_engine} 匹配；\n"
                f"        2) x86_64 dev 机无 GPU 时 planner 已自动 fallback False，"
                f"如仍失败请检查 .omc/plans/installer-redesign.md RK10。"
            )
            return 1

        ctx.success(
            f"[phase: {self.name}] 完成：tier={planner.tier} "
            f"export_yolo_engine={planner.export_yolo_engine}"
        )
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
        """dry-run 输出（与 run() 中 dry-run 分支等价）。"""
        try:
            from scripts import bootstrap as _bootstrap  # type: ignore[import-not-found]
        except Exception:
            return [f"+ [phase: {self.name}] 无法 import scripts.bootstrap（dry-run）"]

        cmd_suffix = "--no-int8" if planner.export_yolo_engine else "--download-only"
        note = _resolve_planner_note(planner)
        return [
            f"+ {sys.executable} scripts/prepare_yolo_world_tensorrt.py "
            f"--weights-output {_bootstrap.DEFAULT_YOLO_WORLD_MODEL} "
            f"--engine-output {_bootstrap.DEFAULT_YOLO_WORLD_ENGINE} {cmd_suffix}",
            f"+ note: {note or '(no note)'}",
        ]


__all__ = ["ModelsPhase"]
