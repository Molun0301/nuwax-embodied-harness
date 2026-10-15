"""自动发布配置和工作流入口；所有写远端操作留在专用发布任务。"""
from __future__ import annotations

import argparse
import json
import os
import platform
import re
import subprocess
from pathlib import Path

from .builder import write_json
from .github_release import GitHubRelease
from .pipeline import assemble_release, build_group, group_system_packages, make_plan
from .validation import ReleaseError, read_json, safe_path


def configured_request(config: dict, *, event: str, ref: str, inputs: dict) -> dict:
    if config.get("schema_version") != 1 or set(config) - {"schema_version", "version", "channel", "targets", "build_jobs", "notes", "input_root"}:
        raise ReleaseError("发布配置版本或字段无效")
    version = config.get("version")
    targets = config.get("targets")
    channel = config.get("channel", "stable")
    publish = True
    if event == "workflow_dispatch":
        version = inputs.get("version") or version
        if inputs.get("targets"):
            targets = [value.strip() for value in inputs["targets"].split(",")]
        if inputs.get("channel") in ("stable", "preview"):
            channel = inputs["channel"]
        publish = inputs.get("publish", True) in (True, "true")
    elif event == "push" and ref.startswith("refs/tags/v"):
        version = ref[len("refs/tags/v"):]
        channel = "preview" if "-" in version else "stable"
    elif event != "push" or ref != "refs/heads/main":
        raise ReleaseError("自动发布只接受main或v版本标签")
    if not isinstance(version, str) or not re.fullmatch(r"\d+\.\d+\.\d+(?:-[A-Za-z0-9.-]+)?", version):
        raise ReleaseError("正式版本必须为主.次.修订，可带预发布后缀")
    if channel not in ("stable", "preview") or (channel == "stable" and "-" in version):
        raise ReleaseError("正式/预览渠道与版本号不一致")
    if not isinstance(targets, list) or not targets or any(not isinstance(value, str) for value in targets):
        raise ReleaseError("发布配置必须给出完整目标列表")
    jobs = config.get("build_jobs")
    if jobs is not None and (not isinstance(jobs, int) or isinstance(jobs, bool) or jobs < 1):
        raise ReleaseError("构建并行数须为正整数；省略时按构建机资源自动选择")
    input_root = config.get("input_root", ".github/release-inputs")
    safe_path(input_root)
    if not isinstance(config.get("notes", ""), str):
        raise ReleaseError("发布说明必须是文本")
    plan = make_plan(version, ",".join(targets))
    return {"plan": plan, "channel": channel, "publish": publish, "build_jobs": jobs, "notes": config.get("notes", ""), "input_root": config.get("input_root", ".github/release-inputs")}


def workflow_output(values: dict) -> None:
    output = os.environ.get("GITHUB_OUTPUT")
    if output:
        with open(output, "a", encoding="utf-8") as handle:
            for key, value in values.items():
                text = json.dumps(value, separators=(",", ":")) if isinstance(value, (dict, list, bool)) else str(value)
                if "\n" in text:
                    handle.write(key + "<<NEH_OUTPUT\n" + text + "\nNEH_OUTPUT\n")
                else:
                    handle.write(key + "=" + text + "\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="NEH正式自动发布入口")
    commands = parser.add_subparsers(dest="command", required=True)
    select = commands.add_parser("select")
    select.add_argument("--config", type=Path, default=Path(".github/release.json"))
    select.add_argument("--output", type=Path, required=True)
    select.add_argument("--event", default=os.environ.get("GITHUB_EVENT_NAME", "workflow_dispatch"))
    select.add_argument("--ref", default=os.environ.get("GITHUB_REF", "refs/heads/main"))
    select.add_argument("--event-file", type=Path)
    for name in ("packages", "build", "container", "assemble", "publish"):
        command = commands.add_parser(name)
        command.add_argument("--request", type=Path, required=True)
        command.add_argument("--root", type=Path, default=Path.cwd())
        if name in ("packages", "build", "container"):
            command.add_argument("--group", required=True)
        if name in ("build", "container", "assemble"):
            command.add_argument("--output", type=Path, required=True)
        if name in ("build", "container"):
            command.add_argument("--cache", type=Path)
        if name == "assemble":
            command.add_argument("--downloads", type=Path, required=True)
        if name == "publish":
            command.add_argument("--directory", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "select":
            event_file = args.event_file or (Path(os.environ["GITHUB_EVENT_PATH"]) if os.environ.get("GITHUB_EVENT_PATH") else None)
            event_data = read_json(event_file.read_bytes()) if event_file else {}
            request = configured_request(read_json(args.config.read_bytes()), event=args.event, ref=args.ref, inputs=event_data.get("inputs", {}))
            revision = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
            request["revision"] = revision
            request["repository"] = os.environ.get("GITHUB_REPOSITORY", "")
            already = False
            if request["repository"]:
                already = GitHubRelease(request["repository"]).check_published(request["plan"]["version"], revision, request["plan"]["requests"])
            write_json(args.output, request)
            workflow_output({"version": request["plan"]["version"], "matrix": request["plan"]["build_matrix"], "publish": request["publish"], "already_published": already})
            print(json.dumps({"version": request["plan"]["version"], "targets": len(request["plan"]["requests"]), "build_groups": len(request["plan"]["build_matrix"]["include"]), "already_published": already}, ensure_ascii=False))
            return 0
        request = read_json(args.request.read_bytes())
        plan = request["plan"]
        if plan != make_plan(plan["version"], ",".join(plan["requests"])):
            raise ReleaseError("受传递的发布计划被修改")
        if args.command == "packages":
            print(" ".join(group_system_packages(plan, args.group)))
        elif args.command == "container":
            group = next(item for item in plan["build_matrix"]["include"] if item["id"] == args.group)
            if group["container"] != "go2-ubuntu20":
                raise ReleaseError("该目标没有已定义的容器环境")
            if platform.system() != "Linux" or platform.machine().lower() not in ("aarch64", "arm64"):
                raise ReleaseError("Go2容器必须在真正的Linux ARM64执行器构建，禁止把x86产物错标")
            if os.geteuid() == 0:
                raise ReleaseError("容器发布应由普通执行器用户发起")
            temporary = Path(os.environ["RUNNER_TEMP"]).resolve()
            workspace = args.root.resolve()
            if temporary not in args.output.resolve().parents or (args.cache and temporary not in args.cache.resolve().parents):
                raise ReleaseError("容器输出和缓存必须位于执行器临时目录")
            image = "neh-go2-release:" + request["revision"][:12]
            packages = " ".join(group_system_packages(plan, args.group))
            subprocess.run(["docker", "build", "--file", str(workspace / ".github/containers/go2-ubuntu20.Dockerfile"), "--build-arg", "NEH_PACKAGES=" + packages, "--build-arg", "NEH_UID=" + str(os.getuid()), "--build-arg", "NEH_GID=" + str(os.getgid()), "--tag", image, str(workspace / ".github/containers")], check=True)
            pip_cache = Path.home() / ".cache/pip"
            pip_cache.mkdir(parents=True, exist_ok=True)
            command = ["docker", "run", "--rm", "--init", "--workdir", str(workspace), "--volume", str(workspace) + ":" + str(workspace) + ":ro", "--volume", str(temporary) + ":" + str(temporary), "--volume", str(pip_cache) + ":/home/nehbuild/.cache/pip"]
            for key in ("GITHUB_OUTPUT", "RUNNER_TEMP", "PYTHONNOUSERSITE", "PYTHONPATH", "PYTHONHOME", "PIP_INDEX_URL", "PIP_EXTRA_INDEX_URL", "PIP_CONFIG_FILE", "LD_LIBRARY_PATH", "PKG_CONFIG_PATH", "CMAKE_PREFIX_PATH", "NEH_RELEASE_STREAM_LOG", "GIT_OPTIONAL_LOCKS"):
                if key in os.environ:
                    command += ["--env", key + "=" + os.environ[key]]
            command += [image, "/usr/bin/python3", "-m", "scripts.product_release.ci", "build", "--request", str(args.request.resolve()), "--group", args.group, "--root", str(workspace), "--output", str(args.output.resolve())]
            if args.cache:
                command += ["--cache", str(args.cache.resolve())]
            subprocess.run(command, check=True)
        elif args.command == "build":
            inputs = args.root / request["input_root"]
            receipts = build_group(plan, args.group, args.root, args.output, jobs=request["build_jobs"], input_root=inputs if inputs.is_dir() else None, cache=args.cache)
            files = [str(path) for directory in args.output.iterdir() if directory.is_dir() for path in sorted(directory.iterdir()) if path.is_file()]
            workflow_output({"files": "\n".join(files)})
            print(json.dumps({"completed_targets": len(receipts)}, ensure_ascii=False))
        elif args.command == "assemble":
            github = GitHubRelease(request["repository"])
            previous = github.previous_catalog(args.output.parent / "previous-catalog.json", current_tag="v" + plan["version"])
            catalog = assemble_release(plan, args.downloads, args.output, args.root, base_url="https://github.com/" + request["repository"] + "/releases/download/v" + plan["version"], channel=request["channel"], previous=previous, notes=request["notes"])
            print(json.dumps({"releases": len(catalog["releases"]), "version": plan["version"]}, ensure_ascii=False))
        else:
            url = GitHubRelease(request["repository"]).publish(args.directory, version=plan["version"], revision=request["revision"], channel=request["channel"], notes=request["notes"])
            print(url)
            if os.environ.get("GITHUB_STEP_SUMMARY"):
                with open(os.environ["GITHUB_STEP_SUMMARY"], "a", encoding="utf-8") as handle:
                    handle.write("已发布 [NEH " + plan["version"] + "](" + url + ")；业务包不携带模型或第三方依赖。\n")
        return 0
    except (ReleaseError, OSError, ValueError, KeyError) as exc:
        parser.exit(1, "自动发布失败：" + str(exc) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
