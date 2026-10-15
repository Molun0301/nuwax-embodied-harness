"""端侧源码构建薄桥：调用共同安装器，封存依赖并导出独立运行候选。"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".new")
    temporary.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n")
    temporary.chmod(0o600)
    os.replace(temporary, path)


def _read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def _runner(root, home, candidate, *, library_prefix=None):
    def run(argv, *, cwd=None, env=None):
        environment = dict(os.environ, PIP_CACHE_DIR=str(home / "cache/python"), PYTHONNOUSERSITE="1", PIP_DISABLE_PIP_VERSION_CHECK="1")
        environment.pop("PYTHONPATH", None)
        environment.pop("PYTHONHOME", None)
        environment.update(env or {})
        if library_prefix is not None:
            environment["LD_LIBRARY_PATH"] = str(library_prefix / "lib") + ":" + str(library_prefix / "lib64")
        if (candidate / "venv").exists():
            environment["PATH"] = str((candidate / "venv/bin").resolve()) + os.pathsep + environment.get("PATH", "")
        if (candidate / "venv/bin/lcm-gen").is_file():
            environment["LCM_GEN_EXECUTABLE"] = str((candidate / "venv/bin/lcm-gen").resolve())
        # 不另建进程组，编译器和安装器后代由外层工作进程守护统一回收。
        subprocess.run([str(value) for value in argv], cwd=cwd or root, env=environment, check=True)
    return run


def _dependency_inputs(request, hardware, seed):
    from scripts.module_installer.catalog import module_requirements
    files = list(module_requirements(request.modules, request.robot))
    files.extend(("scripts/module_installer/manager.py", "scripts/module_installer/catalog.py"))
    if request.robot == "go2":
        files.extend(("scripts/go2_dependencies.py", "third_party/go2-online-sources.json"))
    records = []
    for name in sorted(set(files)):
        path = request.root / name
        records.append({"path": name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    interpreter = Path(seed).resolve()
    identity = subprocess.check_output([seed, "-c", "import sys,sysconfig,json; print(json.dumps({'python':list(sys.version_info[:3]),'platform':sysconfig.get_platform()}))"], text=True)
    vendor = {}
    if hardware.is_jetson and set(request.modules) & {"perception", "memory"}:
        code = "import importlib.metadata as m,json; print(json.dumps({name:m.version(name) for name in ('torch','torchvision')}))"
        vendor = json.loads(subprocess.check_output([seed, "-c", code], text=True))
    return {"schema": 1, "files": records, "bridge_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "robot": request.robot, "interpreter": str(interpreter),
            "interpreter_size": interpreter.stat().st_size, "interpreter_mtime_ns": interpreter.stat().st_mtime_ns,
            "target": json.loads(identity), "is_jetson": hardware.is_jetson, "nvidia_gpu": hardware.nvidia_gpu, "vendor": vendor}


def prepare_dependencies(request, hardware, candidate, run):
    from scripts.module_installer.manager import _python_environment, check_prerequisites
    seed = check_prerequisites(request, hardware)
    inputs = _dependency_inputs(request, hardware, seed)
    key = hashlib.sha256(json.dumps(inputs, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    prefix = request.home / "python-environments" / key
    if prefix.is_symlink():
        raise RuntimeError("依赖环境不能指向其它目录")
    marker = prefix / "complete.json"
    reused = _read_json(marker).get("inputs") == inputs and (prefix / "venv/bin/python").is_file()
    if not reused:
        # 只有未完成且从未发布的目录可清理；完整环境不会在更新时原地改写。
        if marker.exists():
            raise RuntimeError("已封存依赖环境不完整，请保留现场并使用新的依赖输入重新准备")
        if prefix.exists():
            shutil.rmtree(prefix)
        prefix.mkdir(parents=True)
        try:
            _python_environment(request, hardware, prefix, seed, run)
            frozen = subprocess.check_output([str(prefix / "venv/bin/python"), "-m", "pip", "freeze", "--all"], text=True)
            _write_json(marker, {"inputs": inputs, "packages": frozen.splitlines()})
        except BaseException:
            shutil.rmtree(prefix)
            raise
    link = candidate / "venv"
    if link.exists() or link.is_symlink():
        raise RuntimeError("候选Python环境入口已存在")
    link.symlink_to(prefix / "venv", target_is_directory=True)
    return {"dependency_key": key, "dependency_environment": str(prefix), "dependencies_reused": reused}


def _copy_runtime(prefix, candidate, binaries):
    for name, original in binaries.items():
        path = Path(original)
        if Path(name).name != name or not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError("构建程序清单无效：" + str(name))
        target = candidate / "bin" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
    for directory in ("lib", "lib64"):
        source = prefix / directory
        if not source.is_dir():
            continue
        for path in source.rglob("*"):
            if not path.is_file() or not (path.name.endswith(".so") or ".so." in path.name):
                continue
            target = candidate / directory / path.relative_to(source)
            target.parent.mkdir(parents=True, exist_ok=True)
            # 库别名也成为候选普通文件，不保留指向可变构建目录的链接。
            shutil.copy2(path.resolve(strict=True), target)
    if (prefix / "share/nuwax-slam2d").is_dir() and "nuwax-slam-2d-worker" in binaries:
        shutil.copytree(prefix / "share/nuwax-slam2d", candidate / "share/nuwax-slam2d", symlinks=False)


def verify_native(candidate, mutable, run):
    files = [path for name in ("bin", "lib", "lib64") if (candidate / name).exists() for path in (candidate / name).rglob("*") if path.is_file()]
    environment = dict(os.environ, LD_LIBRARY_PATH=str(candidate / "lib") + ":" + str(candidate / "lib64"))
    checked = []
    for path in files:
        with path.open("rb") as handle:
            if handle.read(4) != b"\x7fELF":
                continue
        dynamic = subprocess.run(["readelf", "-d", str(path)], text=True, capture_output=True, check=True).stdout
        paths = re.findall(r"\((?:RUNPATH|RPATH)\).*\[([^\]]+)\]", dynamic)
        if any(str(mutable) in value for value in paths):
            if not shutil.which("patchelf"):
                raise RuntimeError("候选库仍引用构建缓存，需要安装patchelf后重试：" + path.name)
            relative = "$ORIGIN/../lib:$ORIGIN/../lib64" if path.parent == candidate / "bin" else "$ORIGIN:$ORIGIN/../lib:$ORIGIN/../lib64"
            run(["patchelf", "--set-rpath", relative, str(path)])
        output = subprocess.run(["ldd", str(path)], env=environment, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if "not found" in output.stdout or str(mutable) in output.stdout or (output.returncode and "not a dynamic executable" not in output.stdout and "statically linked" not in output.stdout):
            raise RuntimeError("候选动态库未独立闭合：" + path.name + "\n" + output.stdout)
        checked.append(str(path.relative_to(candidate)))
    return checked


def execute(payload):
    root, home, candidate = (Path(payload[name]).resolve() for name in ("root", "home", "candidate"))
    sys.path.insert(0, str(root))
    from scripts.module_installer.catalog import module_imports, module_system_packages, resolve_modules
    from scripts.module_installer.hardware import detect_hardware
    from scripts.module_installer.manager import InstallRequest, _prepare_models, _verify_prefix
    from scripts.module_installer.builds import build_modules
    modules = resolve_modules(payload["modules"])
    hardware = detect_hardware(home)
    request = InstallRequest(root=root, home=home, modules=modules, robot=payload["robot"], jobs=hardware.build_jobs, python=sys.executable)
    operation = payload["operation"]
    run = _runner(root, home, candidate, library_prefix=Path(payload["build_prefix"]) if operation == "build" else None)
    if operation == "metadata":
        packages = list(module_system_packages(modules, request.robot))
        if set(modules) & {"geometry", "slam2d", "fastlio2", "pointlio", "navigation", "follow"}:
            packages.append("patchelf")
        return {"modules": list(modules), "system_packages": list(dict.fromkeys(packages)), "build_jobs": hardware.build_jobs}
    if operation == "dependencies":
        return prepare_dependencies(request, hardware, candidate, run)
    if operation == "build":
        prefix, cache = Path(payload["build_prefix"]), Path(payload["build_cache"])
        binaries = build_modules(root, prefix, cache, modules, request.jobs, run)
        _copy_runtime(prefix, candidate, binaries)
        checked = verify_native(candidate, home / "cache/source", run)
        return {"binaries": sorted(binaries), "native_files": checked}
    if operation == "models":
        # 默认模型指向持久数据根；不在可变Git工作区中写模型或运行配置。
        model_request = InstallRequest(root=candidate / "source", home=home, modules=modules, robot=request.robot, jobs=request.jobs)
        _prepare_models(model_request, candidate, _runner(candidate / "source", home, candidate))
        return {"models_prepared": True}
    if operation == "verify":
        _verify_prefix(candidate, modules)
        python = str(candidate / "venv/bin/python")
        run([python, "-m", "pip", "check"])
        run([python, "-c", "import importlib; [importlib.import_module(n) for n in " + repr(list(module_imports(modules, request.robot))) + "]"], cwd=candidate / "source")
        return {"verified": True}
    raise ValueError("未知源码准备步骤")


def main():
    result = execute(json.load(sys.stdin))
    print("NEH_SOURCE_RESULT=" + json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
