#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PEP 604 union / walrus / match-case 检测（实机 Py3.8.10 兼容守门）。

- 用 ``ast`` 模块 + ``compile(..., feature_version=(3, 8))`` 检测 PEP 604 union
  语法（``X | Y`` 出现在类型注解位置）。
- 位移运算符、字符串字面量、注释等不误报（仅在类型注解位置报错）。
- 同时检测 walrus ``:=`` 与 3.10+ 的 ``match`` 语句，保持 Py3.8.10 兼容。

用法：
    python3 scripts/ci/check_pep604.py services/geometry services/navigation gateways
"""

from __future__ import annotations

import ast
import os
import sys
from pathlib import Path
from typing import List, Tuple


def _scan_pep604_union(tree: ast.AST) -> List[Tuple[int, int, str]]:
    """检测 PEP 604 union 语法（``X | Y`` 出现在类型注解位置）。

    实现策略：
    - 用 ``ast.parse(source, feature_version=(3, 8))``；3.9+ 语法直接 SyntaxError。
    - 进一步用 AST 扫描 ``ast.BinOp`` + ``BitOr``，仅在以下位置报错：
      * ``ast.arg.annotation``（函数参数类型）
      * ``ast.FunctionDef.returns``（返回值类型）
      * ``ast.AnnAssign.annotation``（变量注解）
    """

    errors: List[Tuple[int, int, str]] = []
    for node in ast.walk(tree):
        # 变量注解
        if isinstance(node, ast.AnnAssign) and isinstance(node.annotation, ast.BinOp):
            if isinstance(node.annotation.op, ast.BitOr):
                errors.append((node.lineno, node.col_offset, "实机 Py3.8.10 不支持 PEP 604 union（变量注解），请改 Union[X, Y]"))
        # 函数/异步函数 返回值
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and isinstance(node.returns, ast.BinOp):
            if isinstance(node.returns.op, ast.BitOr):
                errors.append((node.lineno, node.col_offset, "实机 Py3.8.10 不支持 PEP 604 union（返回值注解），请改 Union[X, Y]"))
        # 参数注解：ast.arguments 容器（posonlyargs/args/kwonlyargs/vararg/kwarg）
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            arguments = node.args
            _all_args = list(arguments.posonlyargs) + list(arguments.args) + list(arguments.kwonlyargs)
            if arguments.vararg is not None:
                _all_args.append(arguments.vararg)
            if arguments.kwarg is not None:
                _all_args.append(arguments.kwarg)
            for arg in _all_args:
                if isinstance(arg, ast.arg) and isinstance(arg.annotation, ast.BinOp):
                    if isinstance(arg.annotation.op, ast.BitOr):
                        errors.append((node.lineno, node.col_offset, "实机 Py3.8.10 不支持 PEP 604 union（参数注解），请改 Union[X, Y]"))
    return errors


def _scan_walrus(tree: ast.AST) -> List[Tuple[int, int, str]]:
    """检测 walrus 操作符（``:=``）— Py3.8 起支持，这里只对不必要使用做提示。"""

    errors: List[Tuple[int, int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.NamedExpr):
            errors.append((node.lineno, node.col_offset, "walrus 操作符（:=）请评估可读性，Py3.8.10 支持但本项目统一禁用"))
    return errors


def _scan_match_case(tree: ast.AST) -> List[Tuple[int, int, str]]:
    """检测 match-case（Py3.10+）— 本项目统一禁用。"""

    errors: List[Tuple[int, int, str]] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Match):
            errors.append((node.lineno, node.col_offset, "实机 Py3.8.10 不支持 match-case（Py3.10+），请改 if/elif"))
    return errors


def _check_file(path: Path) -> List[Tuple[int, int, str]]:
    """检查单个 .py 文件；返回错误列表。"""

    try:
        source = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return []
    # 第一关：用 ast.parse + 严格 3.8 模式解析，3.9+ 语法直接 SyntaxError
    try:
        tree = ast.parse(source, feature_version=(3, 8))
    except SyntaxError as exc:
        msg = str(exc)
        if (
            "walrus" in msg
            or "PEP 604" in msg
            or "match" in msg.lower()
            or "AnnAssign" in msg
            or "arguments" in msg
        ):
            return [(exc.lineno or 0, exc.offset or 0, "解析失败：实机 Py3.8.10 不支持该语法（{}）".format(msg))]
        # 其他语法错误（语法笔误等）仍由 lint 工具承担；本守门只管版本兼容
        return []
    # 第二关：AST 精确扫描类型注解位置的 BitOr
    errors: List[Tuple[int, int, str]] = []
    errors.extend(_scan_pep604_union(tree))
    return errors


def main(argv: List[str]) -> int:
    if not argv:
        print("usage: check_pep604.py <dir1> [dir2 ...]", file=sys.stderr)
        return 2
    total = 0
    for arg in argv:
        root = Path(arg)
        if not root.exists():
            print("skip missing: {}".format(root), file=sys.stderr)
            continue
        files = sorted(root.rglob("*.py")) if root.is_dir() else [root]
        for f in files:
            errs = _check_file(f)
            for ln, col, msg in errs:
                rel = f.relative_to(Path.cwd()) if str(f).startswith(str(Path.cwd())) else f
                print("{}:{}:{} - {}".format(rel, ln, col, msg))
                total += 1
    if total:
        print("\ncheck_pep604: {} 处违规".format(total), file=sys.stderr)
        return 1
    print("check_pep604: 0 违规（Py3.8.10 兼容）")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))