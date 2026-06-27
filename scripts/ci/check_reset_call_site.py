#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""reset_loop_closure_state 调用点 AST 守门脚本（commit 1 §1.5）。

目的：强制 ``reset_loop_closure_state`` 仅在 session 边界事件入口调用，
避免普通业务逻辑误调清空闭环检测缓存。

实现：
- 用 ``ast`` 模块扫描目标目录（默认 ``services/geometry skills gateways``）；
- 匹配所有 ``ast.Call`` 节点 + ``ast.Attribute`` 属性名 ``reset_loop_closure_state``；
- 通过 ``ast.FunctionDef.name`` 或 ``ast.AsyncFunctionDef.name`` 判断所在
  函数是否在白名单（whitelist）；
- 命中非白名单函数即报 ``file:line - reset_loop_closure_state 不在 session 边界白名单``；
- exit 0 = 通过；exit 1 = 违规。

白名单（spec 运行时核心 §外部公开入口 / saved_map_activate / explore_start /
mapping_start 三类边界事件 + commit 1 set_session_mode 内部联动）：
- saved_map_activate
- explore_start
- mapping_start
- load_map_version_into_runtime（map_workspace.py:411-417 入口）
- _load_map_version_into_runtime（私有包装）
- start_external_mapping（runtime.py 路径）
- start_platform_exploration（runtime.py 路径）
- _reset_exploration_completion_state（runtime.py:5599 已有合法调用）
- set_session_mode（commit 1 §1.3 内部联动）
- on_localization_session_started（commit 3b 公开方法）

用法：
    python3 scripts/ci/check_reset_call_site.py services/geometry skills gateways
"""

from __future__ import annotations

import ast
import os
import sys
from pathlib import Path
from typing import List, Set, Tuple


# session 边界事件白名单（commit 1 §1.5 + spec 运行时核心）
SESSION_BOUNDARY_FUNCTIONS: Set[str] = {
    # spec 显式三类边界
    "saved_map_activate",
    "explore_start",
    "mapping_start",
    # skills/map_workspace.py:411-417 入口（commit 2 §2.5）
    "load_map_version_into_runtime",
    "_load_map_version_into_runtime",
    # runtime.py 会话入口
    "start_external_mapping",
    "start_platform_exploration",
    # runtime.py:5599 已有合法 reset 调用（commit 1 §1.2 硬契约的隐性边界）
    "_reset_exploration_completion_state",
    # commit 1 §1.3 set_session_mode 内部联动（reset_loop_closure_state 由
    # set_session_mode 在 normalized != "explore" 时调用）
    "set_session_mode",
    # commit 3b 公开入口（cherry-pick 触发后才会用到）
    "on_localization_session_started",
}


def _collect_enclosing_function(tree: ast.AST, target_lineno: int) -> str:
    """返回 target_lineno 所在的最内层 ``FunctionDef``/``AsyncFunctionDef`` 名字。

    若不在任何函数内（模块级调用），返回 ``"<module>"``。
    """

    best_name = "<module>"
    best_lineno = -1
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            # 用函数定义起点 <= target_lineno 判定外层关系；同函数可能多次匹配取最深
            if node.lineno <= target_lineno and node.lineno > best_lineno:
                best_name = node.name
                best_lineno = node.lineno
    return best_name


def _scan_reset_calls(tree: ast.AST) -> List[Tuple[int, str, str]]:
    """扫描树中所有 ``reset_loop_closure_state`` 调用点。

    返回 ``(lineno, enclosing_func_name, callee_qualified)`` 列表。
    ``callee_qualified`` 用于诊断输出：``self.reset_loop_closure_state`` 或
    ``observer.reset_loop_closure_state`` 等。
    """

    hits: List[Tuple[int, str, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        callee = None
        # 形式 1：self.reset_loop_closure_state(...) / observer.reset_loop_closure_state(...)
        if isinstance(func, ast.Attribute) and func.attr == "reset_loop_closure_state":
            callee = func.attr
        # 形式 2：模块级裸调 reset_loop_closure_state(...)（不允许，白名单只允许属性访问）
        elif isinstance(func, ast.Name) and func.id == "reset_loop_closure_state":
            callee = func.id
        if callee is None:
            continue
        enclosing = _collect_enclosing_function(tree, node.lineno)
        hits.append((node.lineno, enclosing, callee))
    return hits


def _check_file(path: Path) -> List[Tuple[int, str]]:
    """检查单个 .py 文件；返回 ``(lineno, message)`` 错误列表。"""

    try:
        source = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return []
    try:
        tree = ast.parse(source, feature_version=(3, 8))
    except SyntaxError as exc:
        return [(exc.lineno or 0, "解析失败：{}".format(exc))]
    errors: List[Tuple[int, str]] = []
    for lineno, enclosing, callee in _scan_reset_calls(tree):
        if enclosing in SESSION_BOUNDARY_FUNCTIONS:
            continue
        errors.append((
            lineno,
            "{}::{} - reset_loop_closure_state 不在 session 边界白名单（whitelist={}）".format(
                enclosing, callee, sorted(SESSION_BOUNDARY_FUNCTIONS),
            ),
        ))
    return errors


def main(argv: List[str]) -> int:
    if not argv:
        print("usage: check_reset_call_site.py <dir1> [dir2 ...]", file=sys.stderr)
        return 2
    total = 0
    for arg in argv:
        root = Path(arg)
        if not root.exists():
            print("skip missing: {}".format(root), file=sys.stderr)
            continue
        files = sorted(root.rglob("*.py")) if root.is_dir() else [root]
        for f in files:
            for ln, msg in _check_file(f):
                rel = f.relative_to(Path.cwd()) if str(f).startswith(str(Path.cwd())) else f
                print("{}:{} - {}".format(rel, ln, msg))
                total += 1
    if total:
        print("\ncheck_reset_call_site: {} 处违规（reset_loop_closure_state 必须在 session 边界调用）".format(total), file=sys.stderr)
        return 1
    print("check_reset_call_site: 0 违规（reset_loop_closure_state 全部在 session 边界白名单内）")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))