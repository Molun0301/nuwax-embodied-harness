"""运行python3 -m scripts.product_release构建并生成版本目录。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .builder import build_manager, build_runtime, create_catalog
from .validation import ReleaseError, host_platform, verify_archive


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="在目标构建机产生NEH轻量业务包及安装下载配方")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("platform", help="输出实际构建平台，供目标断言使用")
    build = commands.add_parser("build", help="编译自有程序并固定下载配方，不把依赖或模型打入业务包")
    manager = commands.add_parser("manager", help="生成独立管理包")
    for command in (build, manager):
        command.add_argument("--root", type=Path, default=Path.cwd())
        command.add_argument("--output", type=Path, required=True)
        command.add_argument("--version", required=True)
        command.add_argument("--source", choices=("git", "workspace"), default="git")
    build.add_argument("--modules", default="core", help="逗号分隔模块，自动补齐依赖")
    build.add_argument("--robot", choices=("customrobot", "go2"), default="customrobot")
    build.add_argument("--dependency-cache", "--wheelhouse", dest="wheelhouse", type=Path, help="发布端已验证轮子或独立资产缓存，只生成下载配方，不进入业务包")
    build.add_argument("--payload", type=Path, help="已有bin/lib等原生产物，仍须校验架构和动态依赖")
    build.add_argument("--constraints", type=Path, help="自动解析模型依赖时须固定torch/torchvision；完整下载配方已固定版本时可省略")
    build.add_argument("--target", type=Path, help="目标平台JSON，必须与platform命令输出完全一致")
    build.add_argument("--jobs", type=int)
    build.add_argument("--install-recipe", type=Path, help="完整固定下载配方JSON，包含python/native/models")
    build.add_argument("--model-recipe", type=Path, help="固定模型修订/URL/摘要/大小的列表JSON，禁止传模型目录")
    build.add_argument("--local-dependencies", action="store_true", help="仅workspace验收：把已有依赖缓存复制为同级独立HTTP下载资产")
    verify = commands.add_parser("verify", help="流式核对归档和全部文件摘要，不执行包内程序")
    verify.add_argument("archive", type=Path)
    verify.add_argument("--manager", action="store_true")
    catalog = commands.add_parser("catalog", help="为已校验的业务包和管理包生成catalog.json")
    catalog.add_argument("archives", type=Path, nargs="*")
    catalog.add_argument("--manager", type=Path)
    catalog.add_argument("--output", type=Path, required=True)
    catalog.add_argument("--base-url", required=True)
    catalog.add_argument("--channel", choices=("stable", "preview"), default="stable")
    catalog.add_argument("--notes", default="")
    catalog.add_argument("--previous", type=Path, help="合并既有版本目录，不覆盖已发布目标")
    args = parser.parse_args(argv)
    try:
        if args.command == "platform":
            result = host_platform()
        elif args.command == "verify":
            result = verify_archive(args.archive, manifest_name="manager-manifest.json" if args.manager else "manifest.json")
            result = {"version": result["version"], "files": len(result["files"]), "verified": True}
        elif args.command == "manager":
            result = build_manager(args.root, args.output, version=args.version, workspace=args.source == "workspace")
            result = {"output": str(args.output), "version": result["version"], "files": len(result["files"])}
        elif args.command == "build":
            result = build_runtime(args.root, args.output, version=args.version, modules=args.modules.split(","), robot=args.robot, workspace=args.source == "workspace", wheelhouse=args.wheelhouse, payload=args.payload, constraints=args.constraints, target=json.loads(args.target.read_text()) if args.target else None, jobs=args.jobs, install_recipe=args.install_recipe, model_recipe=args.model_recipe, local_dependencies=args.local_dependencies)
            result = {"output": str(args.output), "version": result["version"], "platform": result["platform"], "modules": result["modules"], "files": len(result["files"])}
        else:
            if not args.archives and args.manager is None:
                raise ReleaseError("至少提供一个业务包或管理包")
            result = create_catalog(args.archives, args.output, base_url=args.base_url, channel=args.channel, notes=args.notes, manager=args.manager, previous=args.previous)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ReleaseError, ValueError, OSError) as exc:
        parser.exit(1, "发布失败：" + str(exc) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
