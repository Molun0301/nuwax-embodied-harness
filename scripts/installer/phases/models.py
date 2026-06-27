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
- x86_64 dev 机 fallback：planner.py 已自动把 `export_yolo_engine` 设为 False。
- 默认经 `default_registry.register` 注册到 phase 注册表，与 cli/apply 集成路径对齐。
"""

from __future__ import annotations

import sys
from typing import List, Tuple

from scripts.installer.phases.base import Phase
from scripts.installer.planner import _resolve_export_yolo_engine_default


def _resolve_planner_note(planner) -> str:
    """从 planner 拿 export_yolo_engine 的中文 note。"""
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
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        """执行阶段 6；副作用全部委托给 scripts.bootstrap.prepare_assets。"""
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        note = _resolve_planner_note(planner)
        ctx.guard_network(self.name)
        ctx.info(
            f"[phase: {self.name}] 启动：tier={planner.tier} "
            f"export_yolo_engine={bool(planner.export_yolo_engine)} "
            f"({note})"
        )

        # 1. 引导 import scripts.bootstrap
        try:
            from scripts import bootstrap as _bootstrap  # type: ignore[import-not-found]
        except Exception as exc:  # pragma: no cover - 环境故障
            ctx.error(f"[phase: {self.name}] 无法 import scripts.bootstrap: {exc}")
            return 1

        # 2. 构造 BootstrapPlanner；只填 prepare_assets 真正读到的字段
        bootstrap_planner = _bootstrap.BootstrapPlanner.__new__(_bootstrap.BootstrapPlanner)
        bootstrap_planner.prepare_yolo = True
        bootstrap_planner.export_yolo_engine = bool(planner.export_yolo_engine)
        bootstrap_planner.prepare_memory = bool(getattr(planner, "prepare_memory", False))
        bootstrap_planner.python_bin = sys.executable
        # BootstrapPlanner.__init__ 中的 args / install_python 等需要兜底
        import argparse as _argparse
        _ns = _argparse.Namespace(
            python_bin=sys.executable,
            root_env_output="",
            robot_env_output="",
            systemd_output="",
            systemd_user="",
            systemd_group="",
            external_mcp_output="",
            proxy_mcp_output="",
            robot=planner.robot,
            tier=planner.tier,
            health_url=getattr(planner, "health_url", "http://127.0.0.1:8766/api/health"),
            nuwaclaw_saved_key="",
            nuwaclaw_host_mount=getattr(planner, "nuwaclaw_host_mount", "none"),
            # 15 个 boolean flag 全部交给 BootstrapPlanner._resolve_bool 走默认
            install_python=None, geometry=None, data_plane=None, systemd=None, relay=None,
            install_nuwaclaw_docker=None, prepare_memory=None, prepare_yolo=None,
            export_yolo_engine=None, build_geometry=None, build_go2slam=None,
            install_geometry_deps=None, repair_ros2_apt_source=None,
            start_gateway=None, healthcheck=None,
            memory_mode=getattr(planner, "memory_mode", "cloud"),
            perception_mode=getattr(planner, "perception_mode", "hybrid"),
            mcp_mode=getattr(planner, "mcp_mode", "auto"),
        )
        bootstrap_planner.args = _ns
        # 真正调用 prepare_assets
        try:
            _bootstrap.prepare_assets(bootstrap_planner)
        except Exception as exc:
            ctx.error(
                f"[phase: {self.name}] prepare_assets 失败: {exc}。"
                f"建议: 1) 确认 tier={planner.tier!r} 与 export_yolo_engine={planner.export_yolo_engine} 匹配；"
                f"2) x86_64 dev 机无 GPU 时 planner 已自动 fallback False。"
            )
            return 1

        ctx.success(
            f"[phase: {self.name}] 完成：tier={planner.tier} "
            f"export_yolo_engine={planner.export_yolo_engine}"
        )
        return 0

    def dry_run(self, planner, ctx) -> List[str]:
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


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name="models", cls=ModelsPhase)


__all__ = ["ModelsPhase"]