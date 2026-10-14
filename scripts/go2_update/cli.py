"""更新编排：固定源码、增量双端构建、传输和远端安装。"""
import argparse
import fcntl
import json
import os
import queue
import shlex
import signal
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
from pathlib import Path

from .common import changed_files, digest, git, inventory, is_source, read_json, select_components, sync_snapshot, write_json

ROOT = Path(__file__).resolve().parents[2]


class Connection:
    def __init__(self, args):
        self.host = args.host
        self.password = args.password_file.expanduser()
        self.ssh = ["ssh", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=3", "-o", "StrictHostKeyChecking=yes"]
        if self.password.is_file():
            if self.password.stat().st_mode & 0o077:
                raise RuntimeError("连接口令文件必须仅本人可读：" + str(self.password))
            self.ssh = ["sshpass", "-f", str(self.password), *self.ssh]
        else:
            self.ssh += ["-o", "BatchMode=yes"]

    def command(self, argv):
        return [*self.ssh, self.host, shlex.join([str(a) for a in argv])]

    def run(self, argv, sudo=False, capture=False):
        if sudo:
            argv = ["sudo", "-S", "-p", "", *argv] if self.password.is_file() else ["sudo", "-n", *argv]
        with self.password.open("rb") if sudo and self.password.is_file() else open(os.devnull, "rb") as stream:
            return subprocess.run(self.command(argv), stdin=stream, stdout=subprocess.PIPE if capture else None, text=True, check=True).stdout

    def sync(self, source, destination, delete=False):
        command = ["rsync", "-rc", "--perms", "--omit-dir-times", "--protect-args", "--exclude=__pycache__/", "--exclude=*.pyc", "-e", shlex.join(self.ssh)]
        if delete:
            command.append("--delete")
        # 不传输源时间戳，变更文件取得本次时间；未变化文件由校验和跳过。
        subprocess.run([*command, str(source) + "/", self.host + ":" + destination + "/"], check=True)


def snapshot(args, cache):
    repo = args.repo.resolve()
    archive_repo = repo
    with tempfile.TemporaryDirectory(prefix="snapshot-", dir=str(cache)) as temporary:
        exported = Path(temporary)
        if args.source == "remote":
            branch = args.branch or git(repo, "symbolic-ref", "--short", "HEAD").decode().strip()
            git(repo, "check-ref-format", "refs/heads/" + branch)
            print("获取远端分支：" + args.remote + "/" + branch, flush=True)
            # 独立对象库获取远端版本，不改其他任务使用的FETCH_HEAD与分支引用。
            archive_repo = cache / "upstream.git"
            if not archive_repo.exists():
                subprocess.run(["git", "init", "--bare", "--quiet", str(archive_repo)], check=True)
                common = Path(git(repo, "rev-parse", "--git-common-dir").decode().strip())
                objects = ((repo / common) if not common.is_absolute() else common) / "objects"
                (archive_repo / "objects/info/alternates").write_text(str(objects.resolve()) + "\n", encoding="utf-8")
            remote_url = git(repo, "remote", "get-url", args.remote).decode().strip()
            if ":" not in remote_url and not remote_url.startswith("/"):
                remote_url = str((repo / remote_url).resolve())
            git(archive_repo, "fetch", "--no-tags", remote_url, "refs/heads/" + branch)
            revision = git(archive_repo, "rev-parse", "FETCH_HEAD").decode().strip()
        else:
            revision = git(repo, "rev-parse", "HEAD").decode().strip()
        if args.source == "workspace":
            paths = git(repo, "ls-files", "-z", "--cached", "--others", "--exclude-standard").decode().split("\0")
            files = inventory(repo, filter(None, paths))
            sync_snapshot(repo, exported, files, {})
            if inventory(repo, files) != files:
                raise RuntimeError("快照期间工作区发生变化，请重试")
            revision += "+workspace"
        else:
            archive = exported / "source.tar"
            with archive.open("wb") as stream:
                subprocess.run(["git", "-C", str(archive_repo), "archive", revision], stdout=stream, check=True)
            with tarfile.open(archive) as stream:
                members = []
                for entry in stream.getmembers():
                    if entry.isfile() and is_source(entry.name):
                        members.append(entry)
                    elif entry.issym() and is_source(entry.name):
                        raise RuntimeError("源码快照包含符号链接：" + entry.name)
                stream.extractall(exported, members=members)
            archive.unlink()
            files = inventory(exported, [entry.name for entry in members])
        source = cache / "source"
        # 核对缓存实际内容，避免中断留下新文件而仍沿用旧清单。
        actual = inventory(source, [str(path.relative_to(source)) for path in source.rglob("*") if path.is_file() or path.is_symlink()]) if source.exists() else {}
        sync_snapshot(exported, source, files, actual)
        write_json(cache / "source.json", files)
        return revision, files


def await_line(process, expected, timeout=20):
    try:
        line = process.messages.get(timeout=timeout)
    except queue.Empty:
        raise RuntimeError("等待远端构建协调超时")
    if line != expected:
        raise RuntimeError("远端构建协调失败：" + line)


def execute(args):
    cache = args.cache.resolve()
    cache.mkdir(parents=True, exist_ok=True)
    with (cache / "update.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("另一个Go2更新正在使用本机构建缓存")
        return update(args, cache)


def update(args, cache):
    connection = Connection(args)
    remote_base = args.root.rstrip("/") + "/runtime_data/go2-update"
    run_id = time.strftime("%Y%m%d-%H%M%S") + "-" + str(os.getpid())
    run_dir = cache / "runs" / run_id
    run_dir.mkdir(parents=True)
    remote_stage = remote_base + "/runs/" + run_id
    tool = remote_base + "/tools/" + run_id
    connection.run(["mkdir", "-p", tool + "/scripts/go2_update", remote_stage])
    connection.sync(ROOT / "scripts/go2_update", tool + "/scripts/go2_update")
    remote_py = tool + "/scripts/go2_update/remote.py"
    if args.command == "verify":
        connection.run(["python3", remote_py, "verify", "--root", args.root], sudo=True)
        return 0
    revision, files = snapshot(args, cache)
    probe = json.loads(connection.run(["python3", remote_py, "probe", "--root", args.root], capture=True))
    expected = probe["files"]
    accepted_target = None
    if args.expected_target:
        accepted_target = read_json(args.expected_target)
        if (accepted_target.get("host") != args.host
                or accepted_target.get("root") != args.root
                or accepted_target.get("files") != expected):
            raise RuntimeError("实机源码与已核对的预期清单不一致，拒绝覆盖")
    changed = changed_files(expected, files)
    for name in probe.get("protected_changes", probe.get("dirty_tracked", [])):
        if args.command != "build" and name in changed and accepted_target is None:
            raise RuntimeError("Go2有尚未纳入本次源码的修改：" + name + "；请先同步或使用 --source workspace")
    state = read_json(cache / "built.json", {})
    build_changes = changed_files(state.get("files", {}), files)
    environment_file = args.cross_env.resolve()
    environment_inputs = [environment_file, environment_file.parent / "env.sh", environment_file.parent / "cross/ubuntu20.04-arm64/env.sh", environment_file.parent / "cross/ubuntu20.04-arm64/工具链.cmake"]
    build_config = {str(path): digest(path) for path in environment_inputs if path.is_file()}
    build_config["recipe"] = digest(Path(__file__).with_name("build.py"))
    cached_artifacts = state.get("artifacts", {})
    damaged_cache = any(not (cache / "build/payload" / name).is_file() or digest(cache / "build/payload" / name) != item["sha256"] for name, item in cached_artifacts.items())
    components = select_components(build_changes, first=not state or damaged_cache or state.get("config") != build_config or args.rebuild)
    # 原生产依赖档案仍待锁定；变更依赖必须先由现有安装流程准备，不能静默覆盖厂商库。
    dependency_changes = [p for p in changed if p.startswith("requirements") or p in {"pyproject.toml", "setup.py", "setup.cfg"}]
    plan = {"revision": revision, "changed": changed, "components": components, "dependency_changes": dependency_changes, "host": args.host, "root": args.root, "run": str(run_dir)}
    write_json(run_dir / "plan.json", plan)
    print(json.dumps({**plan, "changed": changed[:40], "changed_count": len(changed)}, ensure_ascii=False, indent=2), flush=True)
    if args.command == "plan":
        return 0
    if dependency_changes and args.command == "deploy" and not args.dependencies_ready:
        raise RuntimeError("Python依赖配方已改变，请先准备并验证目标依赖，再带 --dependencies-ready 重试：" + ", ".join(dependency_changes))
    artifact_drift = any(probe.get("artifacts", {}).get(name) != value for name, value in state.get("artifacts", {}).items())
    if not changed and not components and not artifact_drift and not args.force:
        print("源码与上次成功构建均无变化，跳过构建和重启。")
        return 0
    coordinator = None
    local = None
    try:
        # 独立协调锁覆盖源码同步、双端构建和安装；断线由远端监督进程收回其编译子树。
        remote_command = ["flock", "-n", remote_base + "/coord.lock", "python3", "-m", "scripts.go2_update.build", "--root", remote_base + "/source", "--cache", remote_base + "/cache", "--components", ",".join(components), "--jobs", str(args.remote_jobs), "--supervise"]
        shell = "cd " + shlex.quote(tool) + " && exec " + shlex.join(remote_command)
        coordinator = subprocess.Popen(connection.command(["bash", "-c", shell]), stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        coordinator.messages = queue.Queue()
        def receive():
            for line in coordinator.stdout:
                coordinator.messages.put(line.strip())
            coordinator.messages.put("")
        threading.Thread(target=receive, daemon=True).start()
        await_line(coordinator, "LOCKED")
        connection.sync(cache / "source", remote_base + "/source", delete=True)
        coordinator.stdin.write("start\n")
        coordinator.stdin.flush()
        await_line(coordinator, "BUILD_STARTED")
        print("双端构建已启动；开发机构建日志：" + str(run_dir / "build.log"), flush=True)
        if not environment_file.is_file():
            raise RuntimeError("缺少交叉环境：" + str(environment_file))
        command = [sys.executable, "-m", "scripts.go2_update.build", "--root", str(cache / "source"), "--cache", str(cache / "build"), "--components", ",".join(components), "--jobs", str(args.jobs), "--cross"]
        shell = "source " + shlex.quote(str(environment_file)) + " && exec " + shlex.join(command)
        with (run_dir / "build.log").open("w") as stream:
            local = subprocess.Popen(["bash", "-c", shell], cwd=str(ROOT), stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            while local.poll() is None:
                try:
                    local.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    print("构建仍在进行，日志：" + str(run_dir / "build.log"), flush=True)
            if local.returncode:
                raise RuntimeError("开发机交叉构建失败，请查看 " + str(run_dir / "build.log"))
        artifacts = {**state.get("artifacts", {}), **read_json(cache / "build/artifacts.json", {})}
        # 缓存清单不代表文件永远存在；上传前再次核对。
        for name, item in artifacts.items():
            if digest(cache / "build/payload" / name) != item["sha256"]:
                raise RuntimeError("构建缓存产物损坏：" + name)
        write_json(cache / "built.json", {"files": files, "artifacts": artifacts, "config": build_config})
        coordinator.stdin.write("cancel-build\n")
        coordinator.stdin.flush()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            try:
                line = coordinator.messages.get(timeout=1)
                if line.startswith("BUILD_STOPPED="):
                    write_json(run_dir / "remote-build.json", {"status": line, "log": remote_base + "/cache/build.log"})
                    break
                if not line:
                    raise RuntimeError("远端构建协调连接已断开")
            except queue.Empty:
                continue
        else:
            raise RuntimeError("远端重复构建未确认停止，终止本次部署")
        if args.command == "build":
            print("交叉构建完成，尚未安装；产物：" + str(cache / "build/payload"))
            return 0
        manifest = {"revision": revision, "files": files, "artifacts": artifacts, "expected": expected, "python_bin": probe["python_bin"], "sdk_path": probe["sdk_path"], "force_restart": args.force}
        write_json(run_dir / "manifest.json", manifest)
        connection.sync(cache / "source", remote_stage + "/source")
        connection.sync(cache / "build/payload", remote_stage + "/payload")
        connection.sync(run_dir, remote_stage)
        print("产物传输完成，开始安装与服务验证。", flush=True)
        connection.run(["python3", remote_py, "install", "--root", args.root, "--stage", remote_stage], sudo=True)
        write_json(run_dir / "result.json", {"status": "success", "revision": revision, "remote_stage": remote_stage})
        print("Go2更新完成：" + revision, flush=True)
        return 0
    finally:
        if local is not None and local.poll() is None:
            os.killpg(local.pid, signal.SIGTERM)
            try:
                local.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(local.pid, signal.SIGKILL)
                local.wait()
        if coordinator is not None:
            if coordinator.stdin:
                try:
                    coordinator.stdin.close()
                except BrokenPipeError:
                    pass
            try:
                coordinator.wait(timeout=15)
            except subprocess.TimeoutExpired:
                coordinator.terminate()
                coordinator.wait(timeout=10)


def main(argv=None):
    parser = argparse.ArgumentParser(description="开发机驱动Go2增量更新：固定提交、双端缓存构建、校验传输及失败恢复。")
    parser.add_argument("command", choices=("plan", "build", "deploy", "verify"), help="查看计划／仅双端构建／构建并部署／核验实机")
    parser.add_argument("--host", default="宇树Go2有线", help="已有SSH别名或 user@host")
    parser.add_argument("--root", default="/home/unitree/nuwax-robot-bridge", help="Go2已有源码部署目录")
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--source", choices=("remote", "head", "workspace"), default="remote", help="默认获取远端当前分支；head固定本机提交；workspace包含未提交文件")
    parser.add_argument("--remote", default="origin")
    parser.add_argument("--branch", help="默认当前分支")
    parser.add_argument("--cache", type=Path, default=ROOT / "runtime_data/go2-update", help="持久源码与构建缓存")
    parser.add_argument("--cross-env", type=Path, default=ROOT / ".nuwax-test-env/env-arm64.sh")
    parser.add_argument("--password-file", type=Path, default=Path.home() / ".config/nuwax/go2.password", help="可选0600口令文件；不存在则使用SSH密钥与sudo免密")
    parser.add_argument("--jobs", type=int, default=max(1, min(8, os.cpu_count() or 2)))
    parser.add_argument("--remote-jobs", type=int, default=2)
    parser.add_argument("--force", action="store_true", help="源码无变化时仍重装已校验产物")
    parser.add_argument("--rebuild", action="store_true", help="重新调用全部构建配方，保留编译缓存；升级外部工具链或系统库后使用")
    parser.add_argument("--dependencies-ready", action="store_true", help="依赖配方变化时，声明已另行安装并验证目标Python依赖")
    parser.add_argument("--expected-target", type=Path, help="已核对的实机源码清单，含host、root和完整files；用于首次迁移已有修改，任何漂移仍拒绝")
    args = parser.parse_args(argv)
    if args.jobs < 1 or args.remote_jobs < 1 or not args.root.startswith("/") or args.root == "/":
        parser.error("构建并行数须为正整数，远端目录须为非根绝对路径")
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        return execute(args)
    except KeyboardInterrupt:
        print("更新已中断，本次构建正在回收。", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print("Go2更新失败：" + str(error), file=sys.stderr)
        return 1
