"""受控发布矩阵与全量产物聚合；未完成任何声明目标时禁止发布。"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
from pathlib import Path
from typing import Optional

from scripts.module_installer.catalog import MODULES, module_requirements, module_system_packages, resolve_modules
from .builder import Runner, build_manager, build_modules, build_runtime, create_catalog, native_runner, prepare_wheels, snapshot, write_json
from .validation import ReleaseError, digest, host_platform, read_json, verify_archive
from .recipes import canonical_name, copy_asset, python_recipe, relative_assets

TARGETS = {
    "ubuntu24-x86_64": {"runner": "ubuntu-24.04", "architecture": "x86_64", "python": "3.12", "os_versions": ["24.04"], "robot": "customrobot"},
    "ubuntu24-aarch64": {"runner": "ubuntu-24.04-arm", "architecture": "aarch64", "python": "3.12", "os_versions": ["24.04"], "robot": "customrobot"},
    "go2-jetson": {"runner": "ubuntu-24.04-arm", "architecture": "aarch64", "python": "3.8", "os_versions": ["20.04"], "robot": "go2", "container": "go2-ubuntu20"},
}
PRESETS = {
    "core": ("core",),
    "geometry": ("geometry",),
    "map2d": ("slam2d",),
    "fastlio2": ("fastlio2",),
    "pointlio": ("pointlio",),
    "map2d-navigation": ("slam2d", "navigation"),
    "fastlio2-navigation": ("fastlio2", "navigation"),
    "pointlio-navigation": ("pointlio", "navigation"),
    "navigation": ("navigation",),
    "perception": ("perception",),
    "memory": ("memory",),
    "follow": ("follow",),
    "vision-memory": ("perception", "memory"),
    "navigation-memory": ("navigation", "memory"),
    "full-map2d": ("slam2d", "follow", "memory"),
    "full-fastlio2": ("fastlio2", "follow", "memory"),
    "full-pointlio": ("pointlio", "follow", "memory"),
}


def build_groups(entries: list) -> list:
    """同一架构的轻量模块共用编译；最终模块选择仍分别执行互斥校验。"""
    groups = []
    for entry in entries:
        isolated = bool(set(entry["modules"]) & {"perception", "memory"})
        dependencies = module_requirements(entry["modules"], entry["robot"])
        group = next((item for item in groups if not isolated and item["shared"] and item["target"] == entry["target"] and tuple(item["requirements"]) == dependencies), None)
        if group is None:
            group = {"id": entry["target"] + "--build-" + str(len(groups) + 1), "target": entry["target"], "runner": entry["runner"], "platform": entry["platform"], "robot": entry["robot"], "entries": [], "modules": [], "requirements": list(dependencies), "shared": not isolated, "container": entry.get("container", "")}
            groups.append(group)
        group["entries"].append(entry["id"])
        selected = set(group["modules"]) | set(entry["modules"])
        group["modules"] = [key for key in MODULES if key in selected]
    return groups


def group_system_packages(plan: dict, group_id: str) -> list:
    group = next((item for item in plan["build_matrix"]["include"] if item["id"] == group_id), None)
    if group is None:
        raise ReleaseError("构建计划没有该组：" + group_id)
    entries = [entry for entry in plan["matrix"]["include"] if entry["id"] in group["entries"]]
    packages = {package for entry in entries for package in module_system_packages(entry["modules"], entry["robot"])}
    packages.update({"python3-setuptools", "python3-wheel", "libclang-dev", "libboost-all-dev", "libeigen3-dev", "git", "curl", "binutils"})
    return sorted("libglib2.0-0t64" if name == "libglib2.0-0" and group["platform"]["os_versions"] == ["24.04"] else name for name in packages)


def build_group(plan: dict, group_id: str, root: Path, output: Path, *, jobs: int = 2, input_root: Optional[Path] = None, cache: Optional[Path] = None) -> list:
    group = next((item for item in plan["build_matrix"]["include"] if item["id"] == group_id), None)
    if group is None:
        raise ReleaseError("构建计划没有该组：" + group_id)
    actual = host_platform()
    if any(actual.get(key) != value for key, value in group["platform"].items()):
        raise ReleaseError("真实系统/架构/解释器与构建组不符")
    entries = [entry for entry in plan["matrix"]["include"] if entry["id"] in group["entries"]]
    if not group["shared"]:
        return [build_target(plan, entry["id"], root, output / entry["id"], input_root=input_root) for entry in entries]
    work = output.with_name(output.name + ".work")
    if output.exists() or work.exists():
        raise ReleaseError("构建组输出已存在，禁止覆盖")
    work.mkdir(parents=True)
    output.mkdir(parents=True)
    run = Runner(work / "build.log")
    frozen = work / "source"
    snapshot(root, frozen, workspace=False)
    dependency_stage = work / "dependencies"
    dependency_stage.mkdir()
    common_modules = resolve_modules(set(group["modules"]) - {"fastlio2", "pointlio", "slam2d"})
    if any(module_requirements(entry["modules"], entry["robot"]) != module_requirements(common_modules, group["robot"]) for entry in entries):
        raise ReleaseError("构建组的Python依赖并不相同，必须拆组")
    records = prepare_wheels(frozen, dependency_stage, work, common_modules, group["robot"], run, None, None)
    assets = work / "assets"
    assets.mkdir()
    registry = json.loads((frozen / "third_party/python-source-artifacts.json").read_text(encoding="utf-8"))
    approved = {canonical_name(item["distribution"]): item for item in registry["artifacts"]}
    python, origins = python_recipe(records, dependency_stage / "wheelhouse", assets, approved_sources=approved)
    install = work / "install.json"
    install_data = {"python": python, "native": [], "models": []}
    if group["robot"] == "go2":
        from scripts.go2_dependencies import load_recipe
        install_data["go2"] = load_recipe(frozen)
    write_json(install, install_data)
    native = work / "native"
    compiler_cache = cache or work / "cache"
    compiler = native_runner(run, work / "verification/venv", jobs) if "geometry" in group["modules"] else run
    # 双前端互斥是安装规则；构建组分别编译它们，随后按各自清单打包。
    for frontend in ("pointlio", "fastlio2"):
        if frontend in group["modules"]:
            build_modules(frozen, native, compiler_cache, (frontend,), jobs, compiler)
    rust_modules = set(group["modules"]) - {"pointlio", "fastlio2"}
    build_modules(frozen, native, compiler_cache, rust_modules, jobs, compiler)
    if "slam2d" in rust_modules and any("geometry" in entry["modules"] and "slam2d" not in entry["modules"] for entry in entries):
        build_modules(frozen, native, compiler_cache, ("geometry",), jobs, compiler)
    smoke = frozen / "scripts/diagnostics/offline_platform_smoke.py"
    if smoke.is_file():
        run([str(work / "verification/venv/bin/python"), str(smoke), "--output-dir", str(work / "smoke")], cwd=frozen)
    result = []
    for entry in entries:
        destination = output / entry["id"]
        destination.mkdir()
        archive = destination / entry["filename"]
        manifest = build_runtime(root, archive, version=plan["version"], modules=entry["modules"], robot=entry["robot"], payload=native, wheelhouse=assets, install_recipe=install)
        write_json(archive.with_name(archive.name + ".dependency-origins.json"), {"version": plan["version"], "python": origins, "native": manifest["install"]["native"]})
        receipt = {"schema_version": 1, "target_id": entry["id"], "version": manifest["version"], "revision": manifest["revision"], "filename": archive.name, "sha256": digest(archive), "size": archive.stat().st_size, "assets": relative_assets(manifest["install"])}
        write_json(destination / "receipt.json", receipt)
        result.append(receipt)
    write_json(work / "result.json", {"status": "succeeded", "targets": [entry["target_id"] for entry in result]})
    return result


def make_plan(version: str, targets: str) -> dict:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,95}", version):
        raise ReleaseError("版本格式无效")
    requests = [part.strip() for part in targets.split(",")]
    if not requests or len(requests) > 64 or any(not item for item in requests) or len(requests) != len(set(requests)):
        raise ReleaseError("目标列表须为1—64个不重复的目标:模块预设")
    matrix, combinations = [], set()
    for request in requests:
        if request.count(":") != 1:
            raise ReleaseError("目标格式须为目标:模块预设：" + request)
        target_name, preset = request.split(":")
        if target_name not in TARGETS or preset not in PRESETS:
            raise ReleaseError("未知目标或模块预设：" + request)
        target = TARGETS[target_name]
        modules = list(resolve_modules(PRESETS[preset]))
        identity = (target["architecture"], target["python"], tuple(target["os_versions"]), target["robot"], tuple(modules))
        if identity in combinations:
            raise ReleaseError("目标列表包含相同平台与模块组合，不能用执行器别名重复发布：" + request)
        combinations.add(identity)
        key = target_name + "--" + preset
        matrix.append({
            "id": key, "target": target_name, "preset": preset, "runner": target["runner"],
            "robot": target["robot"], "modules": modules,
            "platform": {"system": "linux", "architecture": target["architecture"], "python": target["python"], "os_ids": ["ubuntu"], "os_versions": target["os_versions"]},
            "filename": "neh-" + version + "-" + key + ".tar.gz", "container": target.get("container", ""),
        })
    return {"schema_version": 1, "product": "neh", "version": version, "requests": requests, "matrix": {"include": matrix}, "build_matrix": {"include": build_groups(matrix)}}


def load_plan(path: Path) -> dict:
    plan = read_json(path.read_bytes())
    try:
        expected = make_plan(plan["version"], ",".join(plan["requests"]))
    except (KeyError, TypeError) as exc:
        raise ReleaseError("发布计划无效") from exc
    if plan != expected:
        raise ReleaseError("发布计划与受控目标/模块预设不一致")
    return plan


def build_target(plan: dict, target_id: str, root: Path, output: Path, *, input_root: Optional[Path] = None) -> dict:
    entries = [entry for entry in plan["matrix"]["include"] if entry["id"] == target_id]
    if len(entries) != 1:
        raise ReleaseError("本次计划没有唯一目标：" + target_id)
    entry = entries[0]
    actual = host_platform()
    if any(actual.get(key) != value for key, value in entry["platform"].items()):
        raise ReleaseError("执行器实际平台与所选发布目标不符")
    inputs = input_root / entry["target"] / entry["preset"] if input_root else None
    recipe = inputs / "install.json" if inputs and (inputs / "install.json").is_file() else None
    wheels = inputs / "dependency-cache" if inputs and (inputs / "dependency-cache").is_dir() else None
    if recipe and inputs and (inputs / "assets").is_dir():
        wheels = inputs / "assets"
    constraints = inputs / "constraints.txt" if inputs and (inputs / "constraints.txt").is_file() else None
    models = inputs / "models.json" if inputs and (inputs / "models.json").is_file() else None
    output.mkdir(parents=True, exist_ok=True)
    archive = output / entry["filename"]
    manifest = build_runtime(root, archive, version=plan["version"], modules=entry["modules"], robot=entry["robot"], wheelhouse=wheels, constraints=constraints, install_recipe=recipe, model_recipe=models)
    receipt = {"schema_version": 1, "target_id": target_id, "version": manifest["version"], "revision": manifest["revision"], "filename": archive.name, "sha256": digest(archive), "size": archive.stat().st_size, "assets": relative_assets(manifest["install"])}
    write_json(output / "receipt.json", receipt)
    return receipt


def collect_artifacts(plan: dict, downloads: Path, *, revision: str) -> list:
    """精确对照计划中的所有产物和回执，缺一个目标也不返回可发布集合。"""
    expected = {entry["filename"]: entry for entry in plan["matrix"]["include"]}
    found = {}
    for archive in downloads.rglob("*.tar.gz"):
        if archive.is_symlink() or not archive.is_file() or archive.name in found:
            raise ReleaseError("聚合目录包含链接或重复产物：" + archive.name)
        found[archive.name] = archive
    if set(found) != set(expected):
        raise ReleaseError("发布目标产物不齐全：缺失=%s，额外=%s" % (sorted(set(expected) - set(found)), sorted(set(found) - set(expected))))
    collected = []
    for name, entry in expected.items():
        archive = found[name]
        manifest = verify_archive(archive)
        if manifest.get("version") != plan["version"] or manifest.get("revision") != revision:
            raise ReleaseError("目标产物版本或源码修订与本次发布不符：" + name)
        if manifest.get("source_mode") != "git" or manifest.get("source_dirty") is not False:
            raise ReleaseError("聚合发布只接受干净Git来源：" + name)
        if manifest["modules"] != entry["modules"] or manifest["robot"] != entry["robot"] or any(manifest["platform"].get(key) != value for key, value in entry["platform"].items()):
            raise ReleaseError("目标产物的模块/平台与发布计划不符：" + name)
        receipt_path = archive.parent / "receipt.json"
        if receipt_path.is_symlink() or not receipt_path.is_file():
            raise ReleaseError("目标产物缺少构建成功回执：" + name)
        receipt = read_json(receipt_path.read_bytes())
        assets = relative_assets(manifest["install"])
        for item in assets:
            path = archive.parent / item["path"]
            if path.is_symlink() or not path.is_file() or path.stat().st_size != item["size"] or digest(path) != item["sha256"]:
                raise ReleaseError("独立依赖附件缺失或摘要不符：" + item["path"])
        wanted = {"schema_version": 1, "target_id": entry["id"], "version": plan["version"], "revision": revision, "filename": name, "sha256": digest(archive), "size": archive.stat().st_size, "assets": assets}
        if receipt != wanted:
            raise ReleaseError("目标构建回执与归档不符：" + name)
        collected.append(archive)
    return collected


def assemble_release(plan: dict, downloads: Path, output: Path, root: Path, *, base_url: str, channel: str, previous: Optional[Path] = None, notes: str = "") -> dict:
    revision = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    archives = collect_artifacts(plan, downloads, revision=revision)
    if output.exists():
        raise ReleaseError("聚合输出目录已存在，禁止覆盖：" + str(output))
    output.mkdir(parents=True)
    copied = []
    for archive in archives:
        destination = output / archive.name
        shutil.copyfile(archive, destination)
        manifest = verify_archive(destination)
        write_json(destination.with_name(destination.name + ".manifest.json"), manifest)
        copied.append(destination)
        for item in relative_assets(manifest["install"]):
            copy_asset(archive.parent / item["path"], output / item["path"])
        origins = archive.with_name(archive.name + ".dependency-origins.json")
        if origins.is_file():
            copy_asset(origins, output / origins.name)
    manager = output / "manager.tar.gz"
    build_manager(root, manager, version=plan["version"])
    for source_name, target_name in (("get-neh.sh", "get-neh.sh"), ("scripts/neh_bootstrap.py", "neh_bootstrap.py")):
        source = root / source_name
        if source.is_symlink() or not source.is_file():
            raise ReleaseError("发布缺少引导入口：" + source_name)
        shutil.copyfile(source, output / target_name)
    catalog = create_catalog(copied, output / "catalog.json", manager=manager, base_url=base_url, channel=channel, previous=previous, notes=notes)
    write_json(output / "release-plan.json", plan)
    assets = list(output.iterdir())
    if len(assets) + 1 > 1000 or any(path.stat().st_size >= 2 * 1024 ** 3 for path in assets if path.is_file()):
        raise ReleaseError("发布附件超过GitHub单文件或数量限制；依赖/模型应使用独立上游下载源")
    write_json(output / "release-index.json", {"version": plan["version"], "revision": revision, "targets": len(copied), "files": [{"path": path.name, "sha256": digest(path), "size": path.stat().st_size} for path in sorted(output.iterdir()) if path.is_file()]})
    return catalog


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="受控的多目标构建及全量发布聚合")
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan", help="列出目标:模块预设，生成完整构建矩阵")
    plan.add_argument("--version", required=True)
    plan.add_argument("--targets", required=True)
    plan.add_argument("--output", type=Path, required=True)
    commands.add_parser("presets", help="列出所有受控目标和模块预设")
    build = commands.add_parser("build", help="只构建计划中指定的一个目标")
    build.add_argument("--target-id", required=True)
    build.add_argument("--input-root", type=Path)
    assemble = commands.add_parser("assemble", help="所有目标齐全且同版时一次生成发布目录")
    assemble.add_argument("--downloads", type=Path, required=True)
    assemble.add_argument("--base-url", required=True)
    assemble.add_argument("--channel", choices=("stable", "preview"), default="stable")
    for command in (build, assemble):
        command.add_argument("--plan", type=Path, required=True)
        command.add_argument("--root", type=Path, default=Path.cwd())
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "presets":
            result = {"targets": TARGETS, "presets": {key: list(resolve_modules(values)) for key, values in PRESETS.items()}}
        elif args.command == "plan":
            result = make_plan(args.version, args.targets)
            write_json(args.output, result)
        elif args.command == "build":
            result = build_target(load_plan(args.plan), args.target_id, args.root, args.output, input_root=args.input_root)
        else:
            result = assemble_release(load_plan(args.plan), args.downloads, args.output, args.root, base_url=args.base_url, channel=args.channel)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ReleaseError, OSError, ValueError) as exc:
        parser.exit(1, "发布流水线失败：" + str(exc) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
