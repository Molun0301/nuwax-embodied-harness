#!/usr/bin/env python3
"""Go2现有工作副本的远端更新事务；可作为独立文件传输执行。"""

import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import pwd
import re
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time


RECEIPT = "runtime_data/go2-update/deployed.json"
INSTALLER = "scripts/install_motion_guard_deployment.sh"
LOCKS = ("/run/lock/nuwax-go2-update.lock", "/run/nuwax-installer/installer.lock")
BINARIES = ("bin/nuwax-geometry-runtime", "bin/nuwax-nav-stack", "bin/nuwax-person-follow")
_LOG_DIRECTORY = None


class UpdateError(RuntimeError):
    pass


def safe_relative(value):
    if not isinstance(value, str) or not value or "\x00" in value:
        raise UpdateError("清单路径无效")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in value.split("/")):
        raise UpdateError("清单路径必须是规范相对路径：" + value)
    return value


def private_path(value):
    parts = PurePosixPath(value).parts
    return (parts[0] in (".git", "runtime_data", ".nuwax-test-env")
            or any((part.startswith(".env") and part != ".env.example") or part.startswith(".active_")
                   for part in parts)
            or (parts[0] == "drivers" and ("configs" in parts or ".settings.lock" in parts)))


def is_source(value):
    # 与common.is_source一致；远端入口保持可单文件自举。
    parts = PurePosixPath(safe_relative(value)).parts
    return not (private_path(value)
                or parts[0] in (".gitnexus", "bin", "tests", "docs")
                or any(part in ("target", "__pycache__", ".venv", "result") for part in parts))


def checked_path(root, relative, missing=True):
    """逐级拒绝符号链接，避免特权阶段沿现场链接写到其它目录。"""
    safe_relative(relative)
    current = Path(root)
    for part in PurePosixPath(relative).parts:
        current = current / part
        try:
            info = current.lstat()
        except FileNotFoundError:
            if missing:
                continue
            raise UpdateError("文件不存在：" + str(current))
        if stat.S_ISLNK(info.st_mode):
            raise UpdateError("拒绝符号链接：" + str(current))
        if current != Path(root) / relative and not stat.S_ISDIR(info.st_mode):
            raise UpdateError("路径父级不是目录：" + str(current))
    return current


def checked_root(value):
    path = Path(os.path.abspath(str(value)))
    checked_path(Path("/"), str(path).lstrip("/"), missing=False)
    if not path.is_dir():
        raise UpdateError("目录不存在：" + str(path))
    return path


def digest(path):
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    with os.fdopen(os.open(str(path), flags), "rb") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise UpdateError("仅允许普通文件：" + str(path))
        result = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
        return result.hexdigest()


def inventory(root, paths):
    result = {}
    for relative in sorted(set(paths)):
        path = checked_path(root, relative)
        if not path.exists():
            continue
        if not path.is_file():
            raise UpdateError("受控路径不是普通文件：" + relative)
        result[relative] = {"sha256": digest(path), "mode": 0o755 if path.stat().st_mode & 0o111 else 0o644}
    return result


def run(args, env=None, input_text=None, timeout=120):
    process = subprocess.run([str(item) for item in args], input=input_text,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             universal_newlines=True, env=env, timeout=timeout)
    if _LOG_DIRECTORY is not None:
        directory = checked_path(_LOG_DIRECTORY, "diagnostics")
        directory.mkdir(mode=0o700, exist_ok=True)
        directory.chmod(0o700)
        descriptor, name = tempfile.mkstemp(prefix="command-", suffix=".json", dir=str(directory))
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump({"command": [str(item) for item in args], "returncode": process.returncode,
                       "stdout": process.stdout, "stderr": process.stderr}, stream, ensure_ascii=False)
    if process.returncode:
        # 外部服务输出可能含现场设置，只记录命令身份和退出码。
        raise UpdateError("命令失败（退出码{}）：{}".format(process.returncode, args[0]))
    return process.stdout


def read_json(path):
    if path.is_symlink() or not path.is_file():
        raise UpdateError("JSON输入不是普通文件：" + str(path))
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=".update-", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(name, 0o644)
        os.replace(name, str(path))
    finally:
        if os.path.exists(name):
            os.unlink(name)


def receipt(root):
    path = checked_path(root, RECEIPT)
    return read_json(path) if path.exists() else {}


def source_inventory(root):
    tracked = run(["git", "-c", "safe.directory=" + str(root), "-C", root, "ls-files", "-z"]).split("\x00")
    previous = receipt(root).get("files", {})
    paths = [name for name in tracked if name and is_source(name)]
    paths.extend(name for name in previous if is_source(name))
    return inventory(root, paths)


def runtime_paths(root):
    units = ("nuwax-host-gateway.service", "nuwax-motion-safety-guard.service",
             "nuwax-go2-video-streamer.service")
    settings = {}
    unit_text = ""
    for unit in units:
        environment = run(["systemctl", "show", "--property=Environment", "--value", unit])
        for item in shlex.split(environment):
            key, separator, value = item.partition("=")
            if separator and key in ("NUWAX_PYTHON_BIN", "NUWAX_GO2_SDK_PATH", "PYTHONPATH"):
                settings[key] = value
        unit_text += run(["systemctl", "cat", unit]) + "\n"
    python_bin = settings.get("NUWAX_PYTHON_BIN")
    if not python_bin:
        match = re.search(r"\s(/[^\s;]+/python(?:3(?:\.\d+)?)?)\s+-m\s+(?:drivers\.go2|services\.navigation)", unit_text)
        python_bin = match.group(1) if match else "/usr/bin/python3"
    sdk = settings.get("NUWAX_GO2_SDK_PATH")
    if not sdk:
        paths = settings.get("PYTHONPATH", "").split(":")
        sdk = next((item for item in paths if item and (Path(item) / "unitree_sdk2py").is_dir()), None)
    if not sdk:
        sdk = "/home/unitree/unitree_sdk2_python"
    if not Path(python_bin).is_absolute() or not os.access(python_bin, os.X_OK):
        raise UpdateError("已安装服务的Python解释器不存在或不可执行")
    if not Path(sdk).is_absolute() or not (Path(sdk) / "unitree_sdk2py").is_dir():
        raise UpdateError("已安装服务的宇树开发包目录不可验证")
    version = run([python_bin, "-I", "-c", "import sys; print('.'.join(map(str,sys.version_info[:3])))"])
    runtime_user = run(["systemctl", "show", "--property=User", "--value", units[0]]).strip()
    runtime_user = runtime_user or "root"
    pwd.getpwnam(runtime_user)
    return {"python_bin": python_bin, "sdk_path": sdk, "python_version": version.strip(),
            "python_path": settings.get("PYTHONPATH", str(root) + os.pathsep + sdk),
            "runtime_user": runtime_user}


def probe(root):
    root = checked_root(root)
    result = runtime_paths(root)
    artifact_paths = set(BINARIES) | {"bin/nuwax-mapping-cli"} | set(receipt(root).get("artifacts", {}))
    native = root / "services/nuwax-workspace/crates/nuwax-lio-backends/native"
    for backend in ("fastlio2", "pointlio"):
        for directory in (native / backend / "cpp/result/bin", native / backend / "cpp/result/lib"):
            if directory.exists():
                for path in directory.rglob("*"):
                    if path.is_file() and not path.is_symlink():
                        artifact_paths.add(str(path.relative_to(root)))
    result.update({"files": source_inventory(root),
                   "artifacts": inventory(root, artifact_paths),
                   "revision": run(["git", "-C", root, "rev-parse", "HEAD"]).strip(),
                   "has_receipt": bool(receipt(root)), "root": str(root),
                   "deployed_revision": receipt(root).get("revision")})
    # 分别取工作副本和索引差异，避免解析带空格的porcelain路径。
    names = run(["git", "-C", root, "diff", "--name-only", "-z", "HEAD"]).split("\x00")
    result["dirty_tracked"] = sorted(name for name in names if name and is_source(name))
    previous = receipt(root)
    if previous:
        old_files = previous.get("files", {})
        result["protected_changes"] = sorted(name for name in set(old_files) | set(result["files"])
                                               if old_files.get(name) != result["files"].get(name))
    else:
        result["protected_changes"] = result["dirty_tracked"]
    return result


def normalize_map(value):
    if not isinstance(value, dict):
        raise UpdateError("清单文件集合必须是对象")
    result = {}
    for relative, metadata in value.items():
        safe_relative(relative)
        if private_path(relative):
            raise UpdateError("清单不能包含现场私有路径：" + relative)
        if not isinstance(metadata, dict) or not re.fullmatch(r"[0-9a-f]{64}", str(metadata.get("sha256", ""))):
            raise UpdateError("摘要无效：" + relative)
        mode = metadata.get("mode")
        if not isinstance(mode, int) or stat.S_IMODE(mode) not in (0o644, 0o755):
            raise UpdateError("文件权限必须为0644或0755：" + relative)
        result[relative] = {"sha256": metadata["sha256"], "mode": stat.S_IMODE(mode)}
    return result


def manifest_at(stage):
    manifest = read_json(checked_path(stage, "manifest.json", missing=False))
    for key in ("files", "artifacts", "expected"):
        manifest[key] = normalize_map(manifest[key])
        if key != "artifacts" and any(not is_source(name) for name in manifest[key]):
            raise UpdateError("源码清单包含构建缓存或非部署源码")
    if set(manifest["files"]) & set(manifest["artifacts"]):
        raise UpdateError("源码与产物安装路径重叠")
    if INSTALLER not in manifest["files"]:
        raise UpdateError("源码快照缺少正式服务管理入口")
    if not isinstance(manifest.get("revision"), str) or not manifest["revision"]:
        raise UpdateError("缺少源码版本")
    if not isinstance(manifest.get("force_restart", False), bool):
        raise UpdateError("force_restart必须是布尔值")
    return manifest


def validate_payload(stage, manifest):
    for directory, key in (("source", "files"), ("payload", "artifacts")):
        base = checked_root(stage / directory)
        found = []
        for parent, dirs, files in os.walk(str(base), followlinks=False):
            for name in dirs + files:
                path = Path(parent) / name
                if path.is_symlink():
                    raise UpdateError("暂存目录不能包含符号链接：" + str(path))
            found.extend(str((Path(parent) / name).relative_to(base)) for name in files)
        if inventory(base, found) != manifest[key]:
            raise UpdateError("暂存{}文件集合、摘要或权限不匹配".format(directory))
    for relative in manifest["artifacts"]:
        path = checked_path(stage / "payload", relative, missing=False)
        with path.open("rb") as stream:
            header = stream.read(20)
        if header[:4] != b"\x7fELF" or len(header) != 20 or header[4:6] != b"\x02\x01" or header[18:20] != b"\xb7\x00":
            raise UpdateError("产物不是小端64位ARM架构：" + relative)
        dependencies = run(["ldd", path])
        if "not found" in dependencies:
            raise UpdateError("产物动态依赖缺失：" + relative)
        if Path(relative).name == "nuwax-nav-stack":
            if run([path, "--launcher-contract"], timeout=5).strip() != "nuwax-nav-stack-internal-ready-v1":
                raise UpdateError("导航启动契约不匹配")


def preflight(root, stage, manifest):
    current = source_inventory(root)
    if current != manifest["expected"]:
        raise UpdateError("远端源码在探测后发生变化，拒绝覆盖")
    for relative in set(manifest["files"]) - set(current):
        path = checked_path(root, relative)
        if path.exists():
            raise UpdateError("新增源码与非受控文件冲突：" + relative)
    for relative in manifest["artifacts"]:
        destination = checked_path(root, relative)
        if destination.exists() and not destination.is_file():
            raise UpdateError("产物安装路径被目录或特殊文件占用：" + relative)
    runtime = runtime_paths(root)
    for key in ("python_bin", "sdk_path"):
        if manifest.get(key) != runtime[key]:
            raise UpdateError("目标运行环境在探测后发生变化：" + key)
    validate_payload(stage, manifest)
    program = ("import pathlib,sys; root=pathlib.Path(sys.argv[1]); "
               "[compile(p.read_bytes(),str(p),'exec') for p in root.rglob('*.py')]")
    run([manifest["python_bin"], "-I", "-B", "-c", program, stage / "source"])
    environment = os.environ.copy()
    environment["PYTHONPATH"] = runtime["python_path"]
    # 语法检查不执行源码；依赖检查以实际服务用户及已解析的PYTHONPATH执行。
    run(["runuser", "-u", runtime["runtime_user"], "--", manifest["python_bin"], "-s", "-B", "-c",
         "import numpy,lcm,pydantic,fastapi,uvicorn,unitree_sdk2py"], env=environment)
    return current


@contextlib.contextmanager
def deployment_locks(paths=LOCKS):
    descriptors = []
    try:
        for name in paths:
            path = Path(name)
            checked_path(Path("/"), str(path).lstrip("/"))
            path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            descriptor = os.open(str(path), os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0), 0o600)
            descriptors.append(descriptor)
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_nlink != 1:
                raise UpdateError("部署锁身份无效：" + str(path))
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise UpdateError("另一个部署事务正在运行：" + str(path))
            os.ftruncate(descriptor, 0)
            os.write(descriptor, (str(os.getpid()) + "\n").encode())
        yield
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def service(root, stage, manifest, action):
    environment = os.environ.copy()
    environment.update({"NUWAX_PYTHON_BIN": manifest["python_bin"],
                        "NUWAX_GO2_SDK_PATH": manifest["sdk_path"],
                        "NUWAX_BIN_DIR": str(root / "bin"),
                        "NUWAX_REPO_ROOT_OVERRIDE": str(root), "NUWAX_RENDER_ROOT": str(root),
                        "NUWAX_RENDER_BIN_DIR": str(root / "bin"),
                        "NUWAX_RENDER_PYTHON_BIN": manifest["python_bin"],
                        "NUWAX_RENDER_GO2_SDK_PATH": manifest["sdk_path"],
                        "NUWAX_CONFIG_DIR": "/etc/nuwax", "NUWAX_RENDER_CONFIG_DIR": "/etc/nuwax"})
    if stage is not None:
        environment["NUWAX_SYSTEMD_BACKUP_DIR"] = str(stage / "rollback" / "systemd")
    run(["bash", checked_path(root, INSTALLER, missing=False), action], env=environment, timeout=240)
    return {"action": action, "completed": True}


def copy_atomic(source, destination, mode, owner=None):
    pending = []
    parent = destination.parent
    while not parent.exists():
        pending.append(parent)
        parent = parent.parent
    for directory in reversed(pending):
        directory.mkdir()
        if owner is not None:
            os.chown(directory, *owner)
    descriptor, temporary = tempfile.mkstemp(prefix=".go2-update-", dir=str(destination.parent))
    try:
        with os.fdopen(descriptor, "wb") as output:
            with os.fdopen(os.open(str(source), os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)), "rb") as stream:
                shutil.copyfileobj(stream, output)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, mode)
        if owner is not None:
            os.chown(temporary, *owner)
        os.replace(temporary, str(destination))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def backup_files(root, stage, paths):
    result = {}
    for relative in sorted(paths):
        path = checked_path(root, relative)
        if path.exists():
            info = path.stat()
            target = checked_path(stage, "rollback/files/" + relative)
            copy_atomic(path, target, stat.S_IMODE(info.st_mode))
            result[relative] = {"existed": True, "mode": stat.S_IMODE(info.st_mode),
                                "uid": info.st_uid, "gid": info.st_gid}
        else:
            result[relative] = {"existed": False}
    write_json(stage / "rollback" / "files.json", result)
    return result


def restore_files(root, stage, backups):
    for relative, metadata in backups.items():
        path = checked_path(root, relative)
        if metadata["existed"]:
            source = checked_path(stage, "rollback/files/" + relative, missing=False)
            copy_atomic(source, path, metadata["mode"], (metadata["uid"], metadata["gid"]))
        elif path.exists():
            path.unlink()


def verify(root, manifest=None, check_services=True):
    root = checked_root(root)
    manifest = manifest or receipt(root)
    if not manifest:
        raise UpdateError("缺少待核验的部署清单")
    if check_services:
        service(root, None, manifest, "verify-runtime")
    for key in ("files", "artifacts"):
        if inventory(root, manifest[key]) != manifest[key]:
            raise UpdateError("安装后{}摘要或权限不匹配".format(key))
    runtime = []
    for relative in BINARIES:
        unit = Path(relative).name + ".service"
        state = run(["systemctl", "show", "--property=ActiveState", "--value", unit]).strip()
        entry = {"unit": unit, "state": state, "activated": state == "active"}
        if state not in ("active", "inactive", ""):
            raise UpdateError("服务状态异常：{} {}".format(unit, state))
        runtime.append(entry)
    return {"verified": True, "runtime": runtime,
            "note": "未激活的按需服务未进行运行验收；未触发导航或探索"}


def install(root, stage):
    global _LOG_DIRECTORY
    root, stage = checked_root(root), checked_root(stage)
    _LOG_DIRECTORY = stage
    if os.geteuid() != 0:
        raise UpdateError("安装需要根权限")
    manifest = manifest_at(stage)
    report = {"revision": manifest["revision"], "started": time.time(), "success": False}
    with deployment_locks():
        current = preflight(root, stage, manifest)
        source_changes = {name for name in set(current) | set(manifest["files"])
                          if current.get(name) != manifest["files"].get(name)}
        old_artifacts = inventory(root, manifest["artifacts"])
        artifact_changes = {name for name in manifest["artifacts"]
                            if old_artifacts.get(name) != manifest["artifacts"][name]}
        changed = source_changes | artifact_changes
        if not changed and not manifest.get("force_restart", False):
            report.update(verify(root, manifest))
            report.update({"success": True, "changed": [], "restarted": False})
            write_json(checked_path(root, RECEIPT), dict(manifest, completed=time.time()))
            write_json(checked_path(stage, "report.json"), report)
            return report
        rollback = checked_path(stage, "rollback")
        if rollback.exists():
            raise UpdateError("暂存目录已有回滚记录，请使用新的批次目录")
        rollback.mkdir(mode=0o700)
        backups = backup_files(root, stage, changed)
        stopped = False
        handlers = {}

        def interrupted(signum, frame):
            raise UpdateError("部署收到中断信号{}，开始恢复".format(signum))

        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.signal(signum, interrupted)
            service(root, stage, manifest, "snapshot-systemd")
            stopped = True
            service(root, stage, manifest, "stop")
            for relative in sorted(source_changes):
                destination = checked_path(root, relative)
                if relative not in manifest["files"]:
                    destination.unlink()
                else:
                    metadata = manifest["files"][relative]
                    source = checked_path(stage / "source", relative, missing=False)
                    previous = backups[relative]
                    root_owner = root.stat()
                    owner = ((previous["uid"], previous["gid"]) if previous["existed"]
                             else (root_owner.st_uid, root_owner.st_gid))
                    copy_atomic(source, destination, metadata["mode"], owner)
            for relative in sorted(artifact_changes):
                metadata = manifest["artifacts"][relative]
                source = checked_path(stage / "payload", relative, missing=False)
                copy_atomic(source, checked_path(root, relative), metadata["mode"])
            service(root, stage, manifest, "install")
            service(root, stage, manifest, "start")
            service(root, stage, manifest, "verify-runtime")
            report.update(verify(root, manifest, check_services=False))
            write_json(checked_path(root, RECEIPT), dict(manifest, completed=time.time()))
            report.update({"success": True, "changed": sorted(changed), "restarted": True})
        except BaseException as error:
            for signum in handlers:
                signal.signal(signum, signal.SIG_IGN)
            report["error"] = str(error)
            failures = []
            if stopped:
                # 先收回可能已经启动的新版进程，再恢复文件和服务定义。
                for action in (lambda: service(root, stage, manifest, "stop"),
                               lambda: restore_files(root, stage, backups),
                               lambda: service(root, stage, manifest, "restore-systemd"),
                               lambda: service(root, stage, manifest, "start"),
                               lambda: service(root, stage, manifest, "verify-runtime")):
                    try:
                        action()
                    except BaseException as restore_error:
                        failures.append(str(restore_error))
            report["rollback_errors"] = failures
            report["rolled_back"] = stopped and not failures
            raise UpdateError("部署失败：{}；{}".format(error, "恢复失败：" + "; ".join(failures) if failures else "已恢复旧部署" if stopped else "尚未停止服务"))
        finally:
            for signum, handler in handlers.items():
                signal.signal(signum, handler)
            report["finished"] = time.time()
            write_json(checked_path(stage, "report.json"), report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Go2远端增量部署事务")
    parser.add_argument("command", choices=("probe", "preflight", "install", "verify"))
    parser.add_argument("--root", required=True)
    parser.add_argument("--stage")
    args = parser.parse_args(argv)
    try:
        root = checked_root(args.root)
        if args.command == "probe":
            result = probe(root)
        elif args.command == "verify":
            result = verify(root, manifest_at(checked_root(args.stage)) if args.stage else None)
        else:
            if not args.stage:
                raise UpdateError("此命令必须提供--stage")
            stage = checked_root(args.stage)
            if args.command == "install":
                result = install(root, stage)
            else:
                with deployment_locks():
                    preflight(root, stage, manifest_at(stage))
                result = {"preflight": True}
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        return 0
    except (UpdateError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        if args.stage:
            try:
                report_path = checked_path(checked_root(args.stage), "report.json")
                if not report_path.exists():
                    write_json(report_path, {"success": False, "error": str(error), "finished": time.time()})
            except (UpdateError, OSError):
                pass
        print("远端更新失败：" + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
