"""在真实目标构建机生成完整源码、离线轮子及原生程序。"""
from __future__ import annotations

import datetime
import gzip
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from email.parser import BytesParser
from pathlib import Path
from typing import Optional

from scripts.module_installer.builds import build_modules
from scripts.module_installer.catalog import module_imports, module_requirements, resolve_modules
from .validation import ReleaseError, digest, elf_architecture, elf_interpreter, host_platform, safe_path, validate_platform, verify_archive
from .recipes import canonical_name, copy_asset, dependency_bytes, python_recipe, relative_assets, validate_install

_REQUIRED_SOURCE = (
    "scripts/start_gateway.sh", "scripts/module_installer/runtime.py", "scripts/module_installer/catalog.py",
    "core/config_values.py", "core/settings.py", "gateways/runtime.py", ".env.example",
)
_NATIVE = {"geometry": "nuwax-geometry-runtime", "navigation": "nuwax-nav-stack", "follow": "nuwax-person-follow", "slam2d": "nuwax-slam-2d-worker", "fastlio2": "fastlio2_native", "pointlio": "pointlio_native"}
_BASE_LIBRARIES = re.compile(r"^(ld-linux.*|lib(c|m|pthread|dl|rt|resolv|util|anl|nss_[^.]+)\.so(?:\..*)?)$")


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".new")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


class Runner:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)

    def __call__(self, argv, *, cwd=None, env=None):
        # 凭据只走环境，日志不记录环境或带令牌的下载参数。
        environment = {**os.environ, **(env or {})}
        environment.pop("PYTHONPATH", None)
        environment.pop("PYTHONHOME", None)
        environment["PYTHONNOUSERSITE"] = "1"
        with self.path.open("a", encoding="utf-8") as log:
            log.write("\n运行：" + " ".join(str(arg) for arg in argv) + "\n")
            log.flush()
            if os.environ.get("NEH_RELEASE_STREAM_LOG") == "1":
                process = subprocess.Popen([str(arg) for arg in argv], cwd=cwd, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
                for line in process.stdout:
                    log.write(line)
                    print(line, end="", flush=True)
                returncode = process.wait()
            else:
                result = subprocess.run([str(arg) for arg in argv], cwd=cwd, env=environment, stdout=log, stderr=subprocess.STDOUT, check=False)
                returncode = result.returncode
        if returncode:
            raise ReleaseError("构建命令退出码%s，详细原因保留于%s" % (returncode, self.path))

    def capture(self, argv, *, cwd=None, environment=None, timeout=1800, cancelled=None):
        """适配共享安装器需要的stdout返回值，保留真实失败日志。"""
        env = {**os.environ, **(environment or {})}
        env.pop("PYTHONPATH", None)
        env.pop("PYTHONHOME", None)
        env["PYTHONNOUSERSITE"] = "1"
        if cancelled:
            cancelled()
        result = subprocess.run([str(arg) for arg in argv], cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
        with self.path.open("a", encoding="utf-8") as log:
            log.write("\n共享安装步骤：" + " ".join(str(arg) for arg in argv) + "\n" + result.stdout)
        if os.environ.get("NEH_RELEASE_STREAM_LOG") == "1":
            print(result.stdout, end="", flush=True)
        if result.returncode:
            raise ReleaseError("官方依赖安装失败，退出码%s；原因保留于%s" % (result.returncode, self.path))
        return result.stdout


def _git(root: Path, *args: str) -> bytes:
    environment = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
    environment.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull, GIT_TERMINAL_PROMPT="0")
    result = subprocess.run(["git", "-c", "core.hooksPath=/dev/null", "-c", "core.fsmonitor=false", "-C", str(root), *args], env=environment, capture_output=True, check=False)
    if result.returncode:
        raise ReleaseError("无法读取Git源码：" + result.stderr.decode("utf-8", errors="replace"))
    return result.stdout


def source_allowed(relative: str) -> bool:
    safe_path(relative)
    from scripts.neh_source import source_allowed as allowed_source
    return allowed_source(relative)


def _copy_regular(source: Path, target: Path) -> None:
    if source.is_symlink() or not source.is_file():
        raise ReleaseError("输入包含链接或非普通文件：" + str(source))
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
    target.chmod(0o755 if source.stat().st_mode & 0o111 else 0o644)


def copy_tree(source: Path, target: Path) -> None:
    if source.is_symlink() or not source.is_dir():
        raise ReleaseError("输入目录缺失或是链接：" + str(source))
    for path in sorted(source.rglob("*")):
        if path.is_symlink():
            raise ReleaseError("输入包含链接：" + str(path))
        if path.is_file():
            _copy_regular(path, target / path.relative_to(source))
        elif not path.is_dir():
            raise ReleaseError("输入包含特殊文件：" + str(path))


def snapshot(root: Path, target: Path, *, workspace: bool) -> dict:
    revision = _git(root, "rev-parse", "HEAD").decode().strip()
    dirty = bool(_git(root, "status", "--porcelain", "--untracked-files=normal").strip())
    if dirty and not workspace:
        raise ReleaseError("正式发布要求干净源码；本地验收包含未提交修改时显式使用--source workspace")
    index = _git(root, "ls-files", "--stage", "-z")
    if any(record.startswith(b"160000 ") for record in index.split(b"\0")):
        raise ReleaseError("源码包含子模块，须先提供完整可发布快照")
    args = ["ls-files", "-z", "--cached"]
    if workspace:
        args.extend(["--others", "--exclude-standard"])
    names = sorted(set(name.decode("utf-8") for name in _git(root, *args).split(b"\0") if name))
    for relative in names:
        if not source_allowed(relative):
            continue
        source = root / relative
        if not source.exists() and workspace:
            continue
        # 不允许任何父级链接把快照读取引出仓库。
        if any((root / Path(*Path(relative).parts[:index])).is_symlink() for index in range(1, len(Path(relative).parts) + 1)):
            raise ReleaseError("源码路径包含链接：" + relative)
        _copy_regular(source, target / relative)
    missing = [name for name in _REQUIRED_SOURCE if not (target / name).is_file()]
    if missing:
        raise ReleaseError("完整源码缺少运行入口：" + ", ".join(missing))
    return {"revision": revision, "source_mode": "workspace" if workspace else "git", "source_dirty": dirty}


def wheel_records(directory: Path) -> list:
    # 使用当前pip自带的标准标签算法，不以文件名包含架构字符串代替ABI检查。
    code = "from pip._vendor.packaging.tags import sys_tags; import json; print(json.dumps([str(t) for t in sys_tags()]))"
    tags = set(json.loads(subprocess.check_output([sys.executable, "-c", code], text=True)))
    records = []
    found = set()
    if not directory.is_dir():
        raise ReleaseError("缺少离线轮子目录")
    for path in sorted(directory.iterdir()):
        if path.is_symlink() or not path.is_file() or path.suffix != ".whl":
            raise ReleaseError("轮子目录只允许普通.whl文件：" + path.name)
        parts = path.stem.rsplit("-", 3)
        if len(parts) != 4:
            raise ReleaseError("轮子文件名无效：" + path.name)
        compatible = {"-".join((python, abi, plat)) for python in parts[-3].split(".") for abi in parts[-2].split(".") for plat in parts[-1].split(".")}
        if not compatible.intersection(tags):
            raise ReleaseError("轮子Python/ABI/平台标签不兼容：" + path.name)
        try:
            with zipfile.ZipFile(path) as wheel:
                names = wheel.namelist()
                if len(names) > 100000 or sum(entry.file_size for entry in wheel.infolist()) > 40 * 1024 ** 3:
                    raise ReleaseError("轮子超过文件数量或解压体积限制：" + path.name)
                if len(names) != len(set(names)):
                    raise ReleaseError("轮子包含重复文件：" + path.name)
                for name in names:
                    safe_path(name.rstrip("/"))
                    entry = wheel.getinfo(name)
                    if (entry.external_attr >> 16) & 0o170000 == 0o120000:
                        raise ReleaseError("轮子包含符号链接：" + name)
                    if not name.endswith("/"):
                        with wheel.open(name) as member:
                            header = member.read(20)
                        if header.startswith(b"\x7fELF"):
                            architecture = host_platform()["architecture"]
                            machine = int.from_bytes(header[18:20], "little" if header[5] == 1 else "big") if len(header) >= 20 else 0
                            if header[4] != 2 or {62: "x86_64", 183: "aarch64"}.get(machine) != architecture or parts[-1] == "any":
                                raise ReleaseError("轮子内原生文件架构或平台标记不符：" + name)
                # setuptools等官方轮子带有内部依赖元数据；发行身份只来自根目录dist-info。
                metadata_names = [name for name in names if name.count("/") == 1 and name.endswith(".dist-info/METADATA")]
                if len(metadata_names) != 1:
                    raise ReleaseError("轮子缺少唯一元数据：" + path.name)
                if wheel.getinfo(metadata_names[0]).file_size > 4 * 1024 ** 2:
                    raise ReleaseError("轮子元数据超过大小限制：" + path.name)
                metadata = BytesParser().parsebytes(wheel.read(metadata_names[0]))
                name, version = metadata.get("Name"), metadata.get("Version")
        except zipfile.BadZipFile as exc:
            raise ReleaseError("轮子损坏：" + path.name) from exc
        if not name or not re.fullmatch(r"[A-Za-z0-9_.-]+", name) or not version or not re.fullmatch(r"[A-Za-z0-9_.+!-]+", version):
            raise ReleaseError("轮子名称或版本无效：" + path.name)
        canonical = re.sub(r"[-_.]+", "-", name).lower()
        if canonical in found:
            raise ReleaseError("轮子目录包含同一发行包多个版本：" + name)
        found.add(canonical)
        records.append({"name": name, "version": version, "filename": path.name, "sha256": digest(path)})
    if not records:
        raise ReleaseError("轮子目录为空")
    return records


def _vendor_gate(root: Path, modules: tuple, robot: str, wheelhouse: Optional[Path]) -> None:
    if robot == "go2":
        from scripts.go2_dependencies import load_recipe
        load_recipe(root)
    needed = {"clip"} if "perception" in modules else set()
    if not needed:
        return
    data = json.loads((root / "third_party/python-source-artifacts.json").read_text(encoding="utf-8"))
    for name in sorted(needed):
        record = next((item for item in data["artifacts"] if item["distribution"] == name), None)
        expected = record.get("integrity", {}).get("wheel_sha256") if record else None
        if not record or record.get("status") != "ready" or not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise ReleaseError("厂商或固定源码轮子尚无批准的摘要：%s；%s" % (name, record.get("reason", "缺少来源档案") if record else "缺少来源档案"))
        source = record.get("source", {})
        if not re.fullmatch(r"[0-9a-f]{40}", str(source.get("commit", ""))) or not re.fullmatch(r"[0-9a-f]{64}", str(record.get("integrity", {}).get("source_sha256", ""))):
            raise ReleaseError("已批准轮子缺少固定源码修订及摘要：" + name)
        target = host_platform()
        if any(record.get("build", {}).get(key) != target[key] for key in ("architecture", "python")):
            raise ReleaseError("已批准轮子的构建目标与本次发布不符：" + name)
        if wheelhouse is None:
            raise ReleaseError("%s须通过--wheelhouse提供完整已批准离线轮子" % name)
        records = wheel_records(wheelhouse)
        actual = next((item for item in records if re.sub(r"[-_.]+", "-", item["name"]).lower() == name), None)
        if not actual or actual["sha256"] != expected or actual["version"] != record.get("version"):
            raise ReleaseError("已提供%s轮子不符合批准的摘要" % name)


def _verify_go2_dependencies(root: Path, prefix: Path, cache: Path, run: Runner) -> None:
    """按设备相同路径验证官方下载安装，生成的本机适配缓存绝不发布。"""
    import time
    import urllib.request
    from scripts.go2_dependencies import install, load_recipe
    recipe = load_recipe(root)
    def fetch(record):
        destination = cache / "downloads" / record["sha256"] / record["filename"]
        if destination.is_file() and destination.stat().st_size == record["size"] and digest(destination) == record["sha256"]:
            return destination
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(destination.name + ".partial")
        for attempt in range(3):
            try:
                count = 0
                with urllib.request.urlopen(record["url"], timeout=60) as response, temporary.open("wb") as output:
                    if not response.geturl().startswith("https://"):
                        raise ReleaseError("官方依赖下载不能降级到HTTP")
                    while True:
                        chunk = response.read(1024 * 1024)
                        if not chunk:
                            break
                        count += len(chunk)
                        if count > record["size"]:
                            raise ReleaseError("官方依赖下载超过声明大小")
                        output.write(chunk)
                if count != record["size"] or digest(temporary) != record["sha256"]:
                    raise ReleaseError("官方依赖下载摘要不符")
                temporary.replace(destination)
                return destination
            except Exception:
                temporary.unlink(missing_ok=True)
                if attempt == 2:
                    raise
                time.sleep(attempt + 1)
    environment = dict(os.environ)
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    first = install(prefix, cache / "adaptations", recipe, fetch=fetch, run=run.capture, environment=environment, progress=lambda message: print(message, flush=True))
    second = install(prefix, cache / "adaptations", recipe, fetch=fetch, run=run.capture, environment=environment, progress=lambda message: print(message, flush=True))
    if not second["reused"]:
        raise ReleaseError("Go2官方依赖的二次安装未复用已核验缓存")
    write_json(cache / "verification.json", {"first": first, "second": second})


def prepare_wheels(root: Path, stage: Path, work: Path, modules: tuple, robot: str, run: Runner, wheelhouse: Optional[Path], constraints: Optional[Path]) -> list:
    _vendor_gate(root, modules, robot, wheelhouse)
    destination = stage / "wheelhouse"
    if set(modules) & {"perception", "memory"} and constraints is None:
        raise ReleaseError("模型模块发布必须提供--constraints，固定torch及torchvision；不能隐式升级厂商GPU依赖")
    if set(modules) & {"perception", "memory"}:
        content = constraints.read_text(encoding="utf-8")
        for name in ("torch", "torchvision"):
            if not re.search(r"^" + name + r"\s*==\s*[A-Za-z0-9_.+!-]+\s*(?:#.*)?$", content, re.MULTILINE):
                raise ReleaseError("模型依赖约束缺少精确版本：" + name)
    if wheelhouse is not None:
        copy_tree(wheelhouse, destination)
    else:
        destination.mkdir()
        command = [sys.executable, "-m", "pip", "wheel", "--disable-pip-version-check", "--wheel-dir", str(destination), "pip==25.0.1"]
        for recipe in module_requirements(modules, robot):
            command.extend(["-r", str(root / recipe)])
        if constraints:
            command.extend(["-c", str(constraints)])
        run(command, cwd=root)
    records = wheel_records(destination)
    requirements = "".join("%s==%s\n" % (item["name"], item["version"]) for item in records)
    (stage / "requirements.txt").write_text(requirements, encoding="utf-8")
    verify_env = work / "verification/venv"
    run([sys.executable, "-m", "venv", str(verify_env)])
    python = str(verify_env / "bin/python")
    # 先升级安装器再解析其余轮子，避免Python3.8默认pip拒绝PEP600平台标签。
    installer = next((item for item in records if canonical_name(item["name"]) == "pip"), None)
    if installer:
        run([python, "-m", "pip", "install", "--disable-pip-version-check", "--no-index", "--no-deps", str(destination / installer["filename"])])
    run([python, "-m", "pip", "install", "--disable-pip-version-check", "--no-index", "--only-binary=:all:", "--find-links", str(destination), "-r", str(stage / "requirements.txt")])
    if robot == "go2":
        _verify_go2_dependencies(root, work / "verification", work / "go2-cache", run)
    run([python, "-m", "pip", "check"])
    recipes = [str(root / path) for path in module_requirements(modules, robot)]
    if constraints:
        recipes.append(str(constraints.resolve()))
    check = """
import importlib.metadata as metadata, pathlib, re
from pip._vendor.packaging.requirements import Requirement
for filename in %r:
    for line in pathlib.Path(filename).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('git+'):
            continue
        if line.startswith('-'):
            raise RuntimeError('发布依赖配方不允许嵌套命令选项: ' + line)
        requirement = Requirement(line)
        if requirement.marker and not requirement.marker.evaluate():
            continue
        installed = metadata.version(requirement.name)
        if installed not in requirement.specifier:
            raise RuntimeError('轮子不满足依赖配方: ' + line + ', actual=' + installed)
""" % recipes
    run([python, "-c", check])
    run([python, "-c", "import importlib; [importlib.import_module(n) for n in " + repr(list(module_imports(modules, robot))) + "]"])
    return records


def native_runner(run: Runner, venv: Path, jobs: int):
    """原生协议生成器复用已核验的LCM轮子，避免系统包的旧版本改变线协议。"""
    generator = venv / "bin/lcm-gen"
    if not generator.is_file():
        raise ReleaseError("已验证依赖中缺少lcm-gen，无法构建固定版本的原生协议")
    version = subprocess.check_output([str(generator), "--version"], text=True).strip()
    if version != "lcm-gen 1.5.2":
        raise ReleaseError("发布构建需要lcm-gen 1.5.2，实际为：" + version)

    def invoke(argv, *, cwd=None, env=None):
        run(argv, cwd=cwd, env={**(env or {}), "LCM_GEN_EXECUTABLE": str(generator), "CARGO_BUILD_JOBS": str(jobs)})

    return invoke


def native_closure(payload: Path, modules: tuple, architecture: str, run: Runner) -> list:
    """只验证发布端依赖构建缓存；完整闭包不能作为轻量业务包的payload。"""
    payload.mkdir(parents=True, exist_ok=True)
    needed = [_NATIVE[name] for name in modules if name in _NATIVE]
    if "geometry" in modules and "slam2d" not in modules:
        needed.append("nuwax-mapping-cli")
    if "slam2d" in modules:
        for library in ("libnuwax_slam_toolbox.so.1", "libnuwax_slamtec.so.1"):
            if not (payload / "lib" / library).is_file():
                raise ReleaseError("二维模块缺少原生共享库：" + library)
    for name in needed:
        path = payload / "bin" / name
        if not path.is_file() or not os.access(path, os.X_OK):
            raise ReleaseError("所选模块缺少可执行原生程序：" + name)
    pending = [path for path in payload.rglob("*") if path.is_file()]
    seen = set()
    while pending:
        path = pending.pop()
        if path.is_symlink():
            raise ReleaseError("原生产物不能包含符号链接：" + str(path))
        with path.open("rb") as handle:
            is_elf = handle.read(4) == b"\x7fELF"
        if not is_elf:
            if path.parent == payload / "bin":
                raise ReleaseError("原生程序不是ELF：" + str(path))
            continue
        if elf_architecture(path) != architecture:
            raise ReleaseError("原生产物架构与目标不符：" + str(path))
        interpreter = elf_interpreter(path)
        supported = {"x86_64": {"/lib64/ld-linux-x86-64.so.2", "/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2"}, "aarch64": {"/lib/ld-linux-aarch64.so.1", "/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1"}}
        if interpreter and interpreter not in supported[architecture]:
            raise ReleaseError("原生产物依赖构建机私有装载器，不能直接发布：" + interpreter)
        if path.name in seen:
            continue
        seen.add(path.name)
        env = {**os.environ, "LD_LIBRARY_PATH": str(payload / "lib") + ":" + str(payload / "lib64")}
        result = subprocess.run(["ldd", str(path)], capture_output=True, text=True, env=env)
        output = result.stdout + result.stderr
        if "not found" in output or (result.returncode and "not a dynamic executable" not in output and "statically linked" not in output):
            raise ReleaseError("原生动态依赖不完整：%s\n%s" % (path, output.strip()))
        for line in output.splitlines():
            match = re.match(r"\s*(\S+)\s+=>\s+(/.+?)\s+\(", line)
            if not match:
                continue
            name, resolved = match.groups()
            if _BASE_LIBRARIES.fullmatch(name):
                continue
            if name.startswith(("libcuda.", "libnvidia", "libnv", "libcudnn", "libcublas")):
                raise ReleaseError("厂商GPU运行库须另行审核目标环境，不自动打入通用包：" + name)
            source = Path(resolved).resolve()
            target = payload / "lib" / name
            if target.exists():
                if digest(target) != digest(source):
                    raise ReleaseError("原生闭包出现同名不同内容的共享库：" + name)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
                target.chmod(0o755)
                pending.append(target)
    return sorted(seen)


def _materialize_build_links(payload: Path) -> None:
    # 构建工具生成的SONAME链接转换为普通文件，归档从不携带链接。
    for path in sorted(payload.rglob("*")):
        if path.is_symlink():
            target = path.resolve()
            if not target.is_file() or payload.resolve() not in target.parents:
                raise ReleaseError("构建产物链接指向产物目录之外：" + str(path))
            data = target.read_bytes()
            path.unlink()
            path.write_bytes(data)
            path.chmod(0o755)


def pack(stage: Path, output: Path, manifest: dict, manifest_name: str = "manifest.json") -> dict:
    if output.exists():
        raise ReleaseError("发布文件已存在，禁止覆盖版本：" + str(output))
    files = []
    for path in sorted(stage.rglob("*")):
        if path.is_symlink():
            raise ReleaseError("发布内容含链接：" + str(path))
        if path.is_file():
            relative = path.relative_to(stage).as_posix()
            if relative == manifest_name:
                raise ReleaseError("输入预置了保留的清单文件")
            files.append({"path": relative, "size": path.stat().st_size, "sha256": digest(path)})
    manifest = {**manifest, "files": files}
    write_json(stage / manifest_name, manifest)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".partial")
    try:
        with temporary.open("xb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed, tarfile.open(fileobj=compressed, mode="w|", format=tarfile.PAX_FORMAT) as archive:
            for path in sorted(stage.rglob("*")):
                if not path.is_file():
                    continue
                info = archive.gettarinfo(str(path), arcname=path.relative_to(stage).as_posix())
                info.uid = info.gid = info.mtime = 0
                info.uname = info.gname = ""
                info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                with path.open("rb") as source:
                    archive.addfile(info, source)
        verify_archive(temporary, manifest_name=manifest_name)
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()
    return manifest


def _version(version: str) -> None:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,95}", version):
        raise ReleaseError("版本只允许字母、数字、点、下划线、加减号，最长96字符")


def _check_python_recipe(root: Path, modules: tuple, robot: str, install: dict, constraints: Optional[Path]) -> None:
    from pip._vendor.packaging.requirements import Requirement
    from pip._vendor.packaging.tags import sys_tags
    available = {canonical_name(item["name"]): item["version"] for item in install["python"]}
    recipes = [root / name for name in module_requirements(modules, robot)]
    if constraints:
        recipes.append(constraints)
    for recipe in recipes:
        for raw in recipe.read_text(encoding="utf-8").splitlines():
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("git+"):
                continue
            if line.startswith("-"):
                raise ReleaseError("配方不允许嵌套命令选项")
            requirement = Requirement(line)
            if requirement.marker and not requirement.marker.evaluate():
                continue
            actual = available.get(canonical_name(requirement.name))
            if actual is None or actual not in requirement.specifier:
                raise ReleaseError("下载清单缺少配方要求的精确依赖：" + line)
    tags = {str(tag) for tag in sys_tags()}
    for item in install["python"]:
        pieces = item["filename"][:-4].rsplit("-", 3)
        if len(pieces) != 4 or not {"-".join((py, abi, plat)) for py in pieces[-3].split(".") for abi in pieces[-2].split(".") for plat in pieces[-1].split(".")}.intersection(tags):
            raise ReleaseError("下载轮子的解释器/架构标签不兼容：" + item["filename"])
    if set(modules) & {"perception", "memory"} and not {"torch", "torchvision"}.issubset(available):
        raise ReleaseError("模型模块配方必须固定torch与torchvision精确版本")
    if robot == "go2":
        from scripts.go2_dependencies import load_recipe, validate_recipe
        if install.get("go2") != load_recipe(root):
            raise ReleaseError("Go2官方安装配方与本次源码注册表不一致")
        validate_recipe(install["go2"])
        if set(available) & {"unitree-sdk2py", "cyclonedds"}:
            raise ReleaseError("Go2设备依赖应按官方来源安装，不能发布自建厂商轮子")
    required_vendor = {"clip"} if "perception" in modules else set()
    if required_vendor:
        registry = json.loads((root / "third_party/python-source-artifacts.json").read_text(encoding="utf-8"))
        for name in required_vendor:
            approved = next((item for item in registry["artifacts"] if canonical_name(item["distribution"]) == name), None)
            chosen = next((item for item in install["python"] if canonical_name(item["name"]) == name), None)
            if not approved or approved.get("status") != "ready" or not chosen or chosen["version"] != approved["version"] or chosen["sha256"] != approved.get("integrity", {}).get("wheel_sha256"):
                raise ReleaseError("下载配方包含未批准的厂商或固定源码轮子：" + name)


def _own_payload(source: Path, destination: Path, modules: tuple, architecture: str, assets: Path, native: list) -> list:
    """仅复制自有程序；第三方共享库成为独立下载资产或系统依赖。"""
    needed = ["bin/" + _NATIVE[name] for name in modules if name in _NATIVE]
    if "geometry" in modules and "slam2d" not in modules:
        needed.append("bin/nuwax-mapping-cli")
    if "slam2d" in modules:
        needed += ["lib/libnuwax_slam_toolbox.so.1", "lib/libnuwax_slamtec.so.1"]
    libraries = {item["target"].split("/", 1)[1]: item for item in native}
    inspected = []
    for relative in needed:
        original = source / relative
        path = original.resolve()
        if not path.is_file() or source.resolve() not in path.parents:
            raise ReleaseError("构建缺少自有程序或链接越界：" + relative)
        if elf_architecture(path) != architecture:
            raise ReleaseError("自有程序实际架构不符：" + relative)
        loader = elf_interpreter(path)
        supported_loaders = {"x86_64": {"/lib64/ld-linux-x86-64.so.2", "/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2"}, "aarch64": {"/lib/ld-linux-aarch64.so.1", "/lib/aarch64-linux-gnu/ld-linux-aarch64.so.1"}}
        if loader and loader not in supported_loaders[architecture]:
            raise ReleaseError("自有程序使用非目标系统装载器：" + loader)
        _copy_regular(path, destination / relative)
        (destination / relative).chmod(0o755)
        result = subprocess.run(["ldd", str(path)], capture_output=True, text=True, env={**os.environ, "LD_LIBRARY_PATH": str(source / "lib") + ":" + str(source / "lib64")})
        output = result.stdout + result.stderr
        missing = re.findall(r"^\s*(\S+)\s+=>\s+not found", output, re.MULTILINE)
        if any(name not in libraries for name in missing) or ("not found" in output and not missing) or (result.returncode and not missing and "not a dynamic executable" not in output and "statically linked" not in output):
            raise ReleaseError("自有程序存在未声明原生依赖：%s\n%s" % (relative, output))
        for line in output.splitlines():
            match = re.match(r"\s*(\S+)\s+=>\s+(/.+?)\s+\(", line)
            if not match:
                continue
            name, resolved = match.groups()
            if name.startswith("libnuwax") and name not in {Path(item).name for item in needed if item.startswith("lib/")}:
                raise ReleaseError("构建出现未登记的自有共享库：" + name)
            if _BASE_LIBRARIES.fullmatch(name) or name.startswith("libnuwax"):
                continue
            library = Path(resolved).resolve()
            if str(library).startswith(("/usr/lib/", "/lib/", "/lib64/")):
                continue
            if name.startswith(("libcuda.", "libnvidia", "libnv", "libcudnn", "libcublas")) and name not in libraries:
                raise ReleaseError("厂商GPU运行库须有已批准的独立下载配方：" + name)
            checksum = digest(library)
            if name in libraries:
                if libraries[name]["sha256"] != checksum:
                    raise ReleaseError("构建所用原生依赖与下载配方摘要不符：" + name)
                continue
            filename = name + "." + checksum[:16] + ".download"
            copy_asset(library, assets / filename)
            item = {"name": name, "version": "sha256-" + checksum, "target": "lib/" + name, "url": filename, "sha256": checksum, "size": library.stat().st_size}
            native.append(item)
            libraries[name] = item
        inspected.append(relative)
    return inspected


def build_runtime(root: Path, output: Path, *, version: str, modules, robot: str = "customrobot", workspace: bool = False, wheelhouse: Optional[Path] = None, payload: Optional[Path] = None, constraints: Optional[Path] = None, target: Optional[dict] = None, jobs: Optional[int] = None, install_recipe: Optional[Path] = None, model_recipe: Optional[Path] = None, local_dependencies: bool = False) -> dict:
    _version(version)
    root, output = root.resolve(), output.resolve()
    from scripts.module_installer.hardware import resolve_build_jobs
    jobs = resolve_build_jobs(root, jobs)
    target = target or host_platform()
    validate_platform(target)
    if robot not in ("customrobot", "go2"):
        raise ReleaseError("当前正式启动器仅支持customrobot和go2")
    if robot == "go2" and target["architecture"] != "aarch64":
        raise ReleaseError("Go2端侧发布必须使用实际aarch64目标构建机")
    selected = resolve_modules(modules)
    if set(selected) & {"perception", "memory"} and install_recipe is None and model_recipe is None:
        raise ReleaseError("模型模块须先提供固定下载配方--model-recipe或--install-recipe；不能把模型文件装入主包")
    if local_dependencies and not workspace:
        raise ReleaseError("全部依赖放到本机下载源仅用于workspace验收；正式发布优先使用上游地址")
    output.parent.mkdir(parents=True, exist_ok=True)
    work = output.parent / (output.name + ".build")
    if work.exists() or output.exists():
        raise ReleaseError("输出或构建目录已存在，禁止覆盖：" + str(work))
    work.mkdir()
    run = Runner(work / "build.log")
    stage = work / "stage"
    stage.mkdir()
    try:
        provenance = snapshot(root, stage / "source", workspace=workspace)
        frozen = stage / "source"
        origins = []
        if install_recipe:
            install = json.loads(install_recipe.read_text(encoding="utf-8"))
        else:
            dependency_stage = work / "dependency-preparation"
            dependency_stage.mkdir()
            records = prepare_wheels(frozen, dependency_stage, work, selected, robot, run, wheelhouse, constraints)
            registry = json.loads((frozen / "third_party/python-source-artifacts.json").read_text(encoding="utf-8"))
            approved = {canonical_name(item["distribution"]): item for item in registry["artifacts"]}
            python, origins = python_recipe(records, dependency_stage / "wheelhouse", output.parent, local_assets=local_dependencies, approved_sources=approved)
            install = {"python": python, "native": [], "models": []}
            if robot == "go2":
                from scripts.go2_dependencies import load_recipe
                install["go2"] = load_recipe(frozen)
        if model_recipe:
            install["models"] = json.loads(model_recipe.read_text(encoding="utf-8"))
        validate_install(install, selected)
        _check_python_recipe(frozen, selected, robot, install, constraints)
        (stage / "requirements.txt").write_text("".join("%s==%s\n" % (item["name"], item["version"]) for item in install["python"]), encoding="utf-8")
        build_prefix = payload.resolve() if payload else work / "native-build"
        if payload is None:
            compiler = native_runner(run, work / "verification/venv", jobs) if "geometry" in selected and not install_recipe else run
            build_modules(frozen, build_prefix, work / "cache", selected, jobs, compiler)
        own = _own_payload(build_prefix, stage / "payload", selected, target["architecture"], output.parent, install["native"])
        validate_install(install, selected)
        # 显式配方的同级资产必须已提供并匹配；不会在发布中偷偷缺少文件。
        for item in relative_assets(install):
            asset = output.parent / item["path"]
            if not asset.is_file() and wheelhouse and (wheelhouse / item["path"]).is_file():
                copy_asset(wheelhouse / item["path"], asset)
            if asset.is_symlink() or not asset.is_file() or asset.stat().st_size != item["size"] or digest(asset) != item["sha256"]:
                raise ReleaseError("独立依赖资产缺失或摘要不符：" + item["path"])
        glib = subprocess.run(["dpkg-query", "-W", "-f=${Status}", "libglib2.0-0t64"], capture_output=True, text=True)
        system_packages = ["python3-venv", "libgl1", "libglib2.0-0t64" if glib.returncode == 0 and glib.stdout.strip() == "install ok installed" else "libglib2.0-0"]
        if "geometry" in selected:
            system_packages += ["liblcm-dev", "libpcl-dev", "libssl-dev"]
        if "pointlio" in selected:
            system_packages.append("libgoogle-glog-dev")
        if robot == "go2":
            system_packages += ["python3-pyaudio", "python3-dev", "build-essential"]
        manifest = pack(stage, output, {"schema_version": 2, "product": "neh", "version": version, **provenance, "platform": target, "robot": robot, "modules": list(selected), "data_schema": 1, "system_packages": system_packages, "install": install, "own_native": own})
        write_json(output.with_name(output.name + ".manifest.json"), manifest)
        write_json(output.with_name(output.name + ".dependency-origins.json"), {"version": version, "python": origins, "native": install["native"]})
        write_json(work / "result.json", {"status": "succeeded", "output": str(output), "sha256": digest(output), "size": output.stat().st_size, "dependency_bytes": dependency_bytes(install), "model_bytes": sum(item["size"] for model in install["models"] for item in model["files"])})
        shutil.rmtree(stage)
        for name in ("verify-venv", "verification", "dependency-preparation"):
            if (work / name).exists():
                shutil.rmtree(work / name)
        return manifest
    except Exception as exc:
        write_json(work / "result.json", {"status": "failed", "error": str(exc)})
        raise


def build_manager(root: Path, output: Path, *, version: str, workspace: bool = False) -> dict:
    _version(version)
    root = root.resolve()
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="neh-manager-", dir=str(output.parent)) as directory:
        stage = Path(directory)
        # 管理包单独筛选已知安装源码，绝不打包业务配置和用户状态。
        revision = _git(root, "rev-parse", "HEAD").decode().strip()
        dirty = bool(_git(root, "status", "--porcelain", "--untracked-files=normal").strip())
        if not workspace and dirty:
            raise ReleaseError("管理包正式发布要求干净源码；本地验证可使用--source workspace")
        for relative in ("scripts/device_manager", "scripts/module_installer", "scripts/deploy_installer", "scripts/product_release"):
            source = root / relative
            if not source.is_dir():
                raise ReleaseError("管理包缺少源码：" + relative)
            for path in sorted(source.rglob("*")):
                if path.is_file() and source_allowed(path.relative_to(root).as_posix()):
                    _copy_regular(path, stage / path.relative_to(root))
        for relative in ("scripts/__init__.py", "core/config_values.py", "core/config_catalog.json", "core/config_descriptions.json", "scripts/neh_bootstrap.py", "scripts/neh_system_helper.py", "scripts/neh_source.py", "scripts/neh_source_build.py", "scripts/go2_dependencies.py", "scripts/systemd/check_lcm_multicast.sh", "scripts/systemd/prepare_lcm_network.py"):
            if (root / relative).is_file():
                _copy_regular(root / relative, stage / relative)
        for relative in ("scripts/device_manager/__main__.py", "scripts/device_manager/static/index.html", "scripts/module_installer/catalog.py", "scripts/module_installer/builds.py", "scripts/module_installer/manager.py", "scripts/deploy_installer/config.py", "scripts/product_release/builder.py", "core/config_values.py", "scripts/neh_bootstrap.py", "scripts/neh_system_helper.py", "scripts/neh_source.py", "scripts/neh_source_build.py", "scripts/go2_dependencies.py", "scripts/systemd/check_lcm_multicast.sh", "scripts/systemd/prepare_lcm_network.py"):
            if not (stage / relative).is_file():
                raise ReleaseError("管理包缺少运行文件：" + relative)
        initializer = stage / "scripts/device_manager/__init__.py"
        content, count = re.subn(r"^__version__\s*=\s*[^\n]+$", "__version__ = " + repr(version), initializer.read_text(encoding="utf-8"), flags=re.MULTILINE)
        if count != 1:
            raise ReleaseError("管理源码缺少唯一__version__赋值，无法写入发布版本")
        initializer.write_text(content, encoding="utf-8")
        manifest = pack(stage, output, {"schema_version": 1, "product": "neh-manager", "kind": "manager", "installation_layout": 2, "version": version, "revision": revision, "source_mode": "workspace" if workspace else "git", "source_dirty": dirty, "python": {"minimum": "3.8", "maximum_exclusive": "3.13"}}, "manager-manifest.json")
        write_json(output.with_name(output.name + ".manifest.json"), manifest)
        return manifest


def create_catalog(archives: list, output: Path, *, base_url: str, channel: str = "stable", notes: str = "", manager: Optional[Path] = None, previous: Optional[Path] = None) -> dict:
    from urllib.parse import quote, urlsplit
    url = urlsplit(base_url)
    if url.scheme != "https" and not (url.scheme == "http" and url.hostname in ("127.0.0.1", "localhost", "::1")):
        raise ReleaseError("版本下载地址必须使用HTTPS；本地验收可使用回环HTTP")
    if url.username or url.password or url.query or url.fragment:
        raise ReleaseError("版本地址不能含凭据、查询参数或片段")
    if channel not in ("stable", "preview"):
        raise ReleaseError("发布渠道无效")
    def check_source(manifest):
        if manifest.get("source_mode") != "git" or manifest.get("source_dirty") is not False:
            if channel != "preview" or url.scheme != "http" or url.hostname not in ("127.0.0.1", "localhost", "::1"):
                raise ReleaseError("工作区或未确认来源的包只能用于回环地址的preview验收，不能进入正式版本目录")
    catalog = json.loads(previous.read_text()) if previous else {"schema_version": 1, "product": "neh", "releases": []}
    if catalog.get("schema_version") != 1 or catalog.get("product") != "neh" or not isinstance(catalog.get("releases"), list):
        raise ReleaseError("原版本目录格式无效")
    names = set()
    for archive in archives:
        if archive.name in names:
            raise ReleaseError("下载目录存在同名产物：" + archive.name)
        names.add(archive.name)
        manifest = verify_archive(archive)
        check_source(manifest)
        for item in relative_assets(manifest["install"]):
            asset = archive.parent / item["path"]
            if asset.is_symlink() or not asset.is_file() or asset.stat().st_size != item["size"] or digest(asset) != item["sha256"]:
                raise ReleaseError("版本目录缺少经校验的独立依赖附件：" + item["path"])
        release = next((entry for entry in catalog["releases"] if entry["version"] == manifest["version"] and entry["channel"] == channel), None)
        if release is None:
            release = {"version": manifest["version"], "revision": manifest["revision"], "channel": channel, "notes": notes, "published_at": datetime.datetime.now(datetime.timezone.utc).isoformat(), "artifacts": []}
            catalog["releases"].insert(0, release)
        if release["revision"] != manifest["revision"]:
            raise ReleaseError("同一发布版本不能混用不同源码修订")
        install = manifest["install"]
        artifact = {"kind": "runtime", "url": base_url.rstrip("/") + "/" + quote(archive.name), "sha256": digest(archive), "size": archive.stat().st_size, "platform": manifest["platform"], "robot": manifest["robot"], "modules": manifest["modules"], "dependency_bytes": dependency_bytes(install), "model_bytes": sum(item["size"] for model in install["models"] for item in model["files"])}
        if any(all(entry.get(key) == artifact[key] for key in ("platform", "robot", "modules")) for entry in release["artifacts"]):
            raise ReleaseError("该版本已有相同目标及模块集合，禁止覆盖")
        release["artifacts"].append(artifact)
    if manager:
        manifest = verify_archive(manager, manifest_name="manager-manifest.json")
        check_source(manifest)
        catalog["manager"] = {"version": manifest["version"], "url": base_url.rstrip("/") + "/" + quote(manager.name), "sha256": digest(manager), "size": manager.stat().st_size, "python": manifest["python"]}
    write_json(output, catalog)
    return catalog
