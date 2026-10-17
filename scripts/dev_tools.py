#!/usr/bin/env python3
"""可复用开发工具：LIO（激光惯性里程计）建图对比、导航测试台。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/diagnostics"))


def doctor():
    from navigation_simulator import default_binary, worker_environment
    binary = default_binary()
    checks = {"python": sys.version.split()[0], "lio_viewer": (ROOT / "gateways/http/static/lio-compare/index.html").is_file(),
              "navigation_binary": str(binary), "navigation_binary_exists": binary.is_file(),
              "cargo": shutil.which("cargo"), "local_build_environment": (ROOT / ".nuwax-test-env/env.sh").is_file()}
    try:
        checks["robot_geometry"] = bool(json.loads(worker_environment()["NUWAX_ROBOT_GEOMETRY_JSON"]))
    except Exception as exc:
        checks["robot_geometry_error"] = str(exc)
    print(json.dumps(checks, ensure_ascii=False, indent=2))
    print("LIO对比仅需Python标准库和支持三维渲染的浏览器；导航首次使用先执行 nav build。")
    return 0 if checks["lio_viewer"] and checks.get("robot_geometry") else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog="用法：lio --help；nav build；nav serve --help；nav run --help；doctor。详细说明见docs/公开文档/开发工具使用指南.md。")
    parser.add_argument("tool", choices=("list", "doctor", "lio", "nav"), nargs="?", default="list")
    # 子命令的帮助交给对应工具，避免顶层提前吞掉--help。
    args = parser.parse_args(sys.argv[1:2])
    rest = sys.argv[2:]
    if args.tool == "list":
        print("lio  建图效果对比：导入点云与轨迹、迁移历史清单、三维并排查看、导出指标\nnav  导航测试台：构建、交互选点、内置/实录场景、自动执行与结果导出\ndoctor  检查工具与本机依赖")
        return 0
    if args.tool == "doctor":
        return doctor()
    if args.tool == "lio":
        from lio_compare import main as lio_main
        return lio_main(rest)
    if not rest or rest[0] in {"-h", "--help"}:
        print("导航测试台：nav build [--build-only]；nav serve [参数]；nav run [--scene straight --case straight-seven --timeout 120]\nserve/run支持--binary、--data-dir、--cloud、--poses；用 nav serve --help 查看全部参数。")
        return 0
    operation, *parameters = rest
    if operation == "build":
        if parameters == ["--help"] or parameters == ["-h"]:
            print("nav build：锁定依赖构建当前导航测试台；复用项目本机环境或PATH中的Cargo。构建完成后使用nav serve或nav run。")
            return 0
        command = ["bash", str(ROOT / "scripts/diagnostics/start_navigation_simulator.sh"), "--build-only", *parameters]
        return subprocess.call(command, cwd=ROOT)
    if operation not in {"serve", "run"}:
        parser.error("nav操作必须为build、serve或run")
    from navigation_simulator import main as nav_main
    return nav_main((["--run"] if operation == "run" else []) + parameters)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"工具执行失败：{exc}", file=sys.stderr)
        raise SystemExit(1)
