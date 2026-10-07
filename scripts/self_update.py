"""受管模块安装的源码更新事务；不接收网页提交的命令、URL或路径。"""

from __future__ import annotations

import argparse
import fcntl
import ipaddress
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
import uuid
from contextlib import contextmanager
from pathlib import Path

BUSY = {"queued", "fetching", "stopping", "building", "starting", "verifying", "rolling_back"}
SHA = re.compile(r"[0-9a-f]{40}\Z")


def read_json(path, default=None):
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("更新状态必须为对象")  # noqa: TRY004 - 磁盘JSON格式错误
        return value
    except FileNotFoundError:
        return dict(default or {})


def fsync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def lock(root, name="transaction"):
    directory = root / "runtime_data/updates"
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / (name + ".lock")).open("a") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("已有更新操作正在执行") from exc
        yield


def git(root, *args):
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(
        ["git", "-c", "core.hooksPath=/dev/null", "-C", str(root), *args],
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    if result.returncode:
        # 远端错误可能含带凭据的URL，不能传入公共状态或HTTP响应。
        raise RuntimeError("Git 操作失败：" + args[0] + "；请在部署账户下检查仓库权限、网络及分支")
    return result.stdout.strip()


def source_info(root):
    branch = git(root, "symbolic-ref", "--quiet", "--short", "HEAD")
    remote = git(root, "config", "--get", "branch." + branch + ".remote")
    merge = git(root, "config", "--get", "branch." + branch + ".merge")
    if remote == "." or remote.startswith("-") or not merge.startswith("refs/heads/"):
        raise RuntimeError("更新需要指向远端分支的 upstream 配置")
    git(root, "check-ref-format", merge)
    return {"branch": branch, "remote": remote, "merge_ref": merge, "current_revision": git(root, "rev-parse", "HEAD")}


def check(root):
    with lock(root, "check"):
        info = source_info(root)
        lines = git(root, "ls-remote", "--refs", info["remote"], info["merge_ref"]).splitlines()
        matches = [line.split()[0] for line in lines if len(line.split()) == 2 and line.split()[1] == info["merge_ref"]]
        if len(matches) != 1 or not SHA.fullmatch(matches[0]):
            raise RuntimeError("远端分支不存在或版本响应无效")
        info.update(
            latest_revision=matches[0], available=matches[0] != info["current_revision"], checked_at=time.time()
        )
        write_json(root / "runtime_data/updates/check.json", info)
        return info


def install_home(root):
    configured = os.environ.get("NUWAX_UPDATE_INSTALL_DIR") or os.environ.get("NUWAX_INSTALL_HOME", "")
    if configured:
        home = Path(configured).expanduser().resolve()
    else:
        launcher = Path(os.environ.get("NUWAX_SETTINGS_LAUNCHER", "/nonexistent"))
        home = (
            launcher.resolve().parents[2]
            if launcher.is_file() and launcher.parent.parent.name == "generations"
            else root / "runtime_data/installation"
        )
    from scripts.module_installer.lifecycle import inspect_installation
    from scripts.module_installer.manager import read_current

    current = read_current(home)
    if not current or Path(current.get("source_root", "")).resolve() != root.resolve():
        raise RuntimeError("当前源码尚未登记标准安装，请先使用 ./install.sh 安装并从该安装启动")
    descriptor = inspect_installation(home, environment=os.environ)
    # A foreground child cannot escape an unknown service owner's cgroup.
    # Managed user services use a separate transient updater unit instead.
    try:
        cgroups = Path("/proc/self/cgroup").read_text().splitlines()
    except OSError:
        cgroups = []
    owners = {
        component
        for line in cgroups
        for component in line.rsplit(":", 1)[-1].split("/")
        if component.endswith(".service") and not re.fullmatch(r"user@\d+\.service", component)
    }
    if descriptor["lifecycle"]["mode"] == "foreground" and owners:
        raise RuntimeError("运行所有者与安装记录不符：服务托管应使用标准 systemd-user 生命周期")
    if descriptor["lifecycle"]["mode"] == "systemd-user":
        service = descriptor["lifecycle"]["service_name"]
        worker_pattern = re.escape(service[:-8]) + r"-update-[0-9a-f]{32}\.service"
        if service not in owners and not any(re.fullmatch(worker_pattern, owner) for owner in owners):
            raise RuntimeError("运行所有者与安装记录不符：请通过 ./install.sh start 启动已登记的用户服务")
    return home, dict(current, deployment=descriptor)


def process_identity(pid):
    try:
        fields = Path("/proc", str(pid), "stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return {"boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(), "start_ticks": fields[19]}
    except (OSError, IndexError, ValueError):
        return None


def listener_url(host, port):
    host = str(host)
    if host in {"0.0.0.0", "localhost"}:
        host = "127.0.0.1"
    elif host == "::":
        host = "::1"
    address = ipaddress.ip_address(host)
    port = int(port)
    if not 1 <= port <= 65535:
        raise ValueError("网关端口配置无效")
    return "http://" + ("[" + host + "]" if address.version == 6 else host) + ":" + str(port)


def state(root):
    value = read_json(root / "runtime_data/updates/state.json", {"state": "idle"})
    if value.get("state") in BUSY and time.time() - value.get("updated_at", value.get("created_at", 0)) > 60:
        identity = process_identity(value.get("worker_pid"))
        if not identity or identity != value.get("worker_identity"):
            return dict(value, state="recovery_required", message="更新进程已中断，请检查更新日志和记录的恢复版本")
    return value


def phase(root, status, name, message, **extra):
    status.update(state=name, message=message, updated_at=time.time(), **extra)
    write_json(root / "runtime_data/updates/state.json", status)
    print(json.dumps({"event": "system.update", **status}, ensure_ascii=False), flush=True)


def require_clean(root):
    if git(root, "status", "--porcelain", "--untracked-files=normal"):
        raise RuntimeError("源码存在本地修改或未跟踪文件，拒绝覆盖；请先提交或备份处理")


def queue(root, expected, probe_url=None):
    with lock(root):
        if state(root).get("state") in BUSY | {"recovery_required"}:
            raise RuntimeError("已有更新事务，请先查看其状态")
        require_clean(root)
        home, current = install_home(root)
        info = source_info(root)
        known = read_json(root / "runtime_data/updates/check.json")
        if (
            not isinstance(expected, str)
            or not SHA.fullmatch(expected)
            or known.get("latest_revision") != expected
            or not known.get("available")
        ):
            raise RuntimeError("请先检查更新并确认检测到的版本")
        if known.get("current_revision") != info["current_revision"]:
            raise RuntimeError("当前版本已变化，请重新检查更新")
        status = {
            "request_id": uuid.uuid4().hex,
            "state": "queued",
            "created_at": time.time(),
            "previous_revision": info["current_revision"],
            "target_revision": expected,
            "install_home": str(home),
            "probe_url": probe_url or "http://127.0.0.1:8766",
            "previous_prefix": current["prefix"],
            "deployment": current.get("deployment"),
            "message": "已受理更新，等待后台执行",
        }
        write_json(root / "runtime_data/updates/state.json", status)
        return status


def dispatch(root, status):
    directory = root / "runtime_data/updates"
    # 固定当前执行脚本；更新源码不会替换正在执行的协调器。
    runner = directory / ("runner-" + status["request_id"] + ".py")
    shutil.copyfile(__file__, runner)
    environment = dict(os.environ, PYTHONPATH=str(root), PYTHONUNBUFFERED="1")
    environment.pop("PYTHONHOME", None)
    environment["NUWAX_UPDATE_INSTALL_DIR"] = status["install_home"]
    with (directory / "worker.log").open("ab") as output:
        try:
            from scripts.module_installer.lifecycle import spawn_updater

            spawn_updater(
                status["deployment"],
                [sys.executable, str(runner), "--root", str(root), "--request", status["request_id"]],
                environment=environment,
                log=output,
            )
        except Exception:
            runner.unlink(missing_ok=True)
            phase(root, status, "failed", "无法启动独立更新进程，当前服务未修改")
            raise


def command(argv, root, log, timeout=7200):
    env = dict(os.environ)
    for name in ("PYTHONHOME", "VIRTUAL_ENV", "_NUWAX_CONFIG_FILE_KEYS", "NUWAX_GATEWAY_SUPERVISOR_TITLED"):
        env.pop(name, None)
    env["PYTHONPATH"] = str(root)
    with log.open("ab") as output:
        process = subprocess.Popen(
            [str(item) for item in argv],
            cwd=str(root),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            code = process.wait(timeout=timeout)
        except BaseException:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                # 子安装器的信号处理器要先清理它派生的独立构建进程组。
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise
    if code:
        raise RuntimeError("更新步骤失败，退出码 " + str(code) + "；详见更新构建日志")


def stop(root, prefix, process=None, descriptor=None):
    from scripts.module_installer.cli import _stop

    if descriptor is None:
        _stop({"prefix": str(prefix)}, root)
    else:
        from scripts.module_installer.lifecycle import stop_current

        stop_current(dict(descriptor, prefix=str(prefix)))
    if process is not None:
        process.wait(timeout=5)


def start(root, launcher, log, descriptor=None):
    if descriptor is not None:
        from scripts.module_installer.lifecycle import start_current

        with log.open("ab") as output:
            return start_current(descriptor, log=output)
    env = dict(os.environ)
    for name in env.get("_NUWAX_CONFIG_FILE_KEYS", "").split(":"):
        env.pop(name, None)
    for name in ("PYTHONHOME", "VIRTUAL_ENV", "_NUWAX_CONFIG_FILE_KEYS", "NUWAX_GATEWAY_SUPERVISOR_TITLED"):
        env.pop(name, None)
    with log.open("ab") as output:
        return subprocess.Popen(
            ["bash", str(launcher)],
            cwd=str(root),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            close_fds=True,
        )


def verify(url, revision, process, timeout=120):
    # 只访问受控本机地址，不经过系统代理；核对新进程实际加载的源码身份。
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError("新网关启动后退出")
        try:
            with opener.open(url + "/api/updates", timeout=3) as response:
                payload = json.load(response)
            if payload.get("running_revision") == revision:
                return
        except (OSError, ValueError):
            pass
        time.sleep(2)
    raise RuntimeError("新网关未在期限内返回目标版本")


def cleanup_cache(home):
    cache = home / "cache"
    if cache.is_symlink():
        raise RuntimeError("缓存根为符号链接，拒绝清理")
    # 构建产物、固定源码及依赖供后续增量构建复用；退出模块由安装器按选择清理。
    # 更新验证后只回收临时下载，不清空可复用缓存或其路径/所有者记录。
    path = cache / "downloads"
    if path.is_symlink():
        path.unlink()
    elif path.is_dir():
        shutil.rmtree(path)


def mark_generation(prefix, revision):
    manifest = read_json(prefix / "manifest.json")
    manifest.update(updater_managed=True, source_revision=revision)
    write_json(prefix / "manifest.json", manifest)


def prune_generations(root, home, protected):
    from scripts.module_installer.runtime import read_existing, read_settings

    references = list(read_existing(root).values())
    for prefix in protected:
        references.extend(read_settings(prefix / "settings.env").values())
    generations = home / "generations"
    if generations.is_symlink():
        raise RuntimeError("代际目录是符号链接，拒绝清理")
    known = {"venv", "bin", "lib", "lib64", "share", "start.sh", "settings.env", "manifest.json", "constraints.txt"}
    for candidate in generations.iterdir():
        if candidate in protected or candidate.is_symlink() or not candidate.is_dir():
            continue
        try:
            manifest = read_json(candidate / "manifest.json")
        except (OSError, ValueError):
            continue  # 无法验证归属的旧目录保留
        if manifest.get("updater_managed") is not True or manifest.get("source_root") != str(root):
            continue
        if {p.name for p in candidate.iterdir()} - known:
            continue  # 非安装器产物可能是现场数据
        referenced = False
        for value in references:
            if not isinstance(value, str) or not value:
                continue
            try:
                path = Path(value).expanduser()
                if path.is_absolute() and (
                    path.resolve() == candidate.resolve() or candidate.resolve() in path.resolve().parents
                ):
                    referenced = True
                    break
            except (OSError, ValueError, RuntimeError):
                referenced = True  # 不可解析的引用不猜测其归属
                break
        if referenced:
            continue
        shutil.rmtree(candidate)


def execute(root, request_id):
    root = root.resolve()
    with lock(root):
        status = state(root)
        if status.get("request_id") != request_id or status.get("state") != "queued":
            raise RuntimeError("更新事务身份或阶段不匹配")
        status["worker_pid"] = os.getpid()
        status["worker_identity"] = process_identity(os.getpid())
        probe_url = status.get("probe_url", "http://127.0.0.1:8766")
        previous = status["previous_revision"]
        target = status["target_revision"]
        home, current = install_home(root)
        descriptor = status.get("deployment") or current.get("deployment")
        old_prefix = Path(current["prefix"])
        log = root / "runtime_data/updates/build.log"
        stopped = False
        changed = False
        new_process = None
        try:
            require_clean(root)
            if source_info(root)["current_revision"] != previous:
                raise RuntimeError("源码在排队后已变化")
            phase(root, status, "fetching", "正在获取并验证远端版本")
            info = source_info(root)
            git(root, "fetch", "--no-tags", "--no-recurse-submodules", info["remote"], info["merge_ref"])
            if git(root, "rev-parse", "FETCH_HEAD") != target:
                raise RuntimeError("远端版本已改变，请重新检查更新")
            git(root, "merge-base", "--is-ancestor", previous, target)
            # 停止前完成可检查前提。新版本构建依赖可能不同；不允许后台sudo交互。

            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(probe_url + "/api/updates/readiness", timeout=5) as response:
                readiness = json.load(response)
            if readiness.get("ready") is not True:
                raise RuntimeError("停止前安全复核失败：" + str(readiness.get("reason", "状态未知")))
            phase(root, status, "stopping", "正在停止网关及其管理的运行组件")
            stopped = True
            stop(root, old_prefix, descriptor=descriptor)
            require_clean(root)
            git(root, "merge", "--ff-only", target)
            changed = True
            phase(root, status, "building", "正在构建与安装已选模块，期间服务暂时离线")
            # 独立解释器读取新版本安装器。旧代际留到新进程验证通过后再回收。
            code = """from pathlib import Path
from scripts.module_installer.manager import InstallRequest,apply,read_current
from scripts.module_installer.hardware import detect_hardware
import os, signal

def cancel_install(signum, frame):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, cancel_install)
root=Path(os.environ['NEH_UPDATE_ROOT']);home=Path(os.environ['NUWAX_UPDATE_INSTALL_DIR'])
current=read_current(home);hardware=detect_hardware(root)
request=InstallRequest(root,home,tuple(current['modules']),current['robot'],min(2,hardware.build_jobs),python=current['python'],wheel_dir=home/'cache/device-wheels',repair=True,keep_previous=True,noninteractive=True)
apply(request,hardware)
"""
            os.environ["NEH_UPDATE_ROOT"] = str(root)
            command([current["python"], "-c", code], root, log)
            from scripts.module_installer.manager import read_current

            new = read_current(home)
            mark_generation(Path(new["prefix"]), target)
            phase(root, status, "starting", "构建完成，正在启动新版本")
            new_process = start(root, home / "current/start.sh", log, descriptor=descriptor)

            phase(root, status, "verifying", "正在核对网关实际运行版本")
            verify(probe_url, target, new_process)
            warning = ""
            try:
                cleanup_cache(home)
                prune_generations(root, home, {old_prefix, Path(new["prefix"])})
            except (OSError, RuntimeError, ValueError):
                warning = "更新成功，但部分临时下载或旧代际清理失败，请查看日志"
            # 保留一代恢复环境，不属于临时缓存；避免删去配置引用的旧文件。
            phase(
                root,
                status,
                "succeeded",
                warning or "更新完成，临时下载已清理，增量构建缓存已保留",
                cleanup_warning=warning,
                rollback_prefix=str(old_prefix),
            )
        except Exception as error:  # noqa: BLE001 - 事务失败必须进入恢复流程
            message = str(error)
            if stopped:
                phase(root, status, "rolling_back", "更新失败，正在恢复原版本")
                try:
                    if new_process is not None or descriptor is not None:
                        stop(root, (home / "current").resolve(), new_process, descriptor=descriptor)
                    if changed:
                        require_clean(root)
                        git(root, "reset", "--keep", previous)
                    temporary = home / (".rollback-" + request_id)
                    temporary.symlink_to(old_prefix.relative_to(home))
                    os.replace(temporary, home / "current")
                    fsync_directory(home)
                    recovery = start(root, old_prefix / "start.sh", log, descriptor=descriptor)

                    verify(probe_url, previous, recovery)
                    phase(root, status, "rolled_back", "更新失败，已恢复并验证原版本：" + message)
                except Exception:  # noqa: BLE001 - 恢复失败保留明确人工处理状态
                    phase(
                        root,
                        status,
                        "recovery_required",
                        "自动恢复未完成，请检查构建日志并从记录的源码/安装版本恢复",
                        failure=message,
                    )
            else:
                phase(root, status, "failed", message)
        finally:
            (root / "runtime_data/updates" / ("runner-" + request_id + ".py")).unlink(missing_ok=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--request", required=True)
    args = parser.parse_args()

    def interrupted(signum, frame):
        raise RuntimeError("更新进程收到中断信号，尝试安全恢复")

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    execute(args.root, args.request)
