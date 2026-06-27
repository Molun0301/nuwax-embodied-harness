#!/usr/bin/env python3
"""
docs_guard.py — README / 部署文档 token 引用守门脚本。

任务来源：installer-redesign / task 6（installer-docs-guard）。
对应 plan 4.1 L128（scripts/installer/docs_guard.py 守门脚本）与
specs/installer/spec.md "Requirement: 文档 token 引用清理与守门"。

用途：
- 扫描 README.md 与 docs/公开文档/ 下所有 .md / .rst / .txt 文件，
  检测是否仍残留已废弃的鉴权 token 字面量：
      agent-robot-bridge-token
      admin-robot-bridge-token
- 命中即 exit 2 + 中文报错"检测到已废弃鉴权 token 引用，请清理"。
- 默认无输出（命中才输出），CI 守门友好。

CLI：
    python3 scripts/installer/docs_guard.py
        # 默认扫描 README.md + docs/公开文档/
    python3 scripts/installer/docs_guard.py --paths <额外路径>
        # 扩展扫描路径（可重复）
    python3 scripts/installer/docs_guard.py --strict
        # 命中即 exit 2（默认即此行为；保留显式开关便于将来区分 warning）

退出码：
    0  仓库干净（无任何命中）
    2  命中废弃 token（CI 失败）
    64 参数错误（argparse 自动）

设计原则：
- 复用 Python 自带 pathlib + re，不引第三方依赖（与 installer 包其他模块一致）。
- 守门不阻断 wizard 流程（wizard 末尾只 "report but not abort"），但 CI 步骤必跑。
- 替换失败时宁可漏报也不误报：只匹配完整 token 字面量，不做模糊 token 探测。
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# 默认扫描根：README.md（仓库根）+ docs/公开文档/ 整目录
DEFAULT_TARGETS: tuple[Path, ...] = (
    Path("README.md"),
    Path("docs/公开文档"),
)

# 守门模式：精确匹配两个已废弃 token 字面量
# 不做前后缀宽松匹配（避免误伤同名前缀的合法配置项）
DEPRECATED_TOKEN_PATTERN = re.compile(
    r"\b(?:agent|admin)-robot-bridge-token\b"
)

# 待扫描的文件后缀
SCAN_SUFFIXES: tuple[str, ...] = (".md", ".rst", ".txt")


def iter_target_files(targets: list[Path]) -> list[Path]:
    """把传入的 target 列表展开为待扫描文件清单。

    - 若是文件路径：直接加入
    - 若是目录路径：递归收集所有匹配 SCAN_SUFFIXES 的文件
    - 路径不存在：跳过（不抛错，便于 CI 在缺失目录时也能跑过）
    """
    collected: list[Path] = []
    for target in targets:
        if not target.exists():
            # 路径不存在直接跳过（README.md / docs/公开文档/ 可能在 sub-shell 路径下）
            continue
        if target.is_file():
            collected.append(target)
        elif target.is_dir():
            for path in target.rglob("*"):
                if path.is_file() and path.suffix.lower() in SCAN_SUFFIXES:
                    collected.append(path)
    # 去重 + 排序（确定性输出，便于 CI 日志）
    return sorted(set(collected))


def scan_file(path: Path) -> list[tuple[int, str]]:
    """扫描单个文件，返回命中 (行号, 行内容) 列表。

    使用 utf-8 编码容错读取；遇到二进制 / 解码错误时返回空列表（不抛错）。
    """
    hits: list[tuple[int, str]] = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return hits
    for lineno, line in enumerate(text.splitlines(), start=1):
        if DEPRECATED_TOKEN_PATTERN.search(line):
            hits.append((lineno, line.rstrip("\n")))
    return hits


def main(argv: list[str] | None = None) -> int:
    """CLI 入口。返回进程退出码。"""
    parser = argparse.ArgumentParser(
        prog="docs_guard",
        description="文档 token 引用守门：检测 README/部署文档中是否残留已废弃鉴权 token",
    )
    parser.add_argument(
        "--paths",
        action="append",
        default=[],
        metavar="PATH",
        help="扩展扫描路径（文件或目录），可重复；与默认目标合并",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        default=True,
        help="命中即 exit 2（默认行为，保留显式开关）",
    )
    args = parser.parse_args(argv)

    # 合并默认 target + 用户 --paths（用户在项目根运行时）
    extra_paths = [Path(p) for p in args.paths]
    targets: list[Path] = list(DEFAULT_TARGETS) + extra_paths

    files = iter_target_files(targets)
    if not files:
        # 没有可扫描文件 = 静默通过（不输出，避免污染 CI 日志）
        return 0

    all_hits: list[tuple[Path, int, str]] = []
    for f in files:
        for lineno, line in scan_file(f):
            all_hits.append((f, lineno, line))

    if not all_hits:
        # 干净：exit 0，无输出
        return 0

    # 命中：exit 2，中文报错
    print("检测到已废弃鉴权 token 引用，请清理：", file=sys.stderr)
    for f, lineno, line in all_hits:
        # 相对路径优先，便于阅读
        rel = f
        try:
            rel = f.relative_to(Path.cwd())
        except ValueError:
            pass
        print(f"  {rel}:{lineno}: {line.strip()}", file=sys.stderr)
    print(
        "\n修复指引：删除 curl 示例中的 -H 'Authorization: Bearer <token>' 行；"
        "env 示例中 NUWAX_GATEWAY_*_TOKENS 直接删除。\n"
        "参见 memory config-layering-and-env-mechanics.md（网关已完全移除鉴权）。",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
