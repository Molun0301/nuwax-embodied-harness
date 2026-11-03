"""沿原网关拥有方管理进程；配置通过当前版本的共同实现读写。"""
from __future__ import annotations

import ast
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from . import PACKAGE_ROOT
from .common import ManagerError, atomic_json, process_alive, process_identity, read_json

PROBE_CODE = r'''
import json, sys
from pathlib import Path
from dataclasses import asdict
request = json.load(sys.stdin)
if request["operation"] == "runtime":
    from scripts.module_installer.runtime import write_runtime
    write_runtime(Path.cwd(), Path(request["install_prefix"]), request["modules"], request["robot"], request["values"])
    result = {"ok": True}
else:
    from core import config_io
    if request["operation"] in ("validate", "validate_existing", "normalize_profile"):
        from core.config_registry import get_field
        values = {}
        for key, value in request["values"].items():
            field = get_field(key)
            raw = str(value).lower() if isinstance(value, bool) else str(value)
            if field is None:
                if request["operation"] in ("validate_existing", "normalize_profile"):
                    values[key] = raw
                    continue
                raise ValueError("未知配置键：" + key)
            if request["operation"] in ("validate_existing", "normalize_profile") and not raw.strip():
                continue
            if field.type == "int" and request["operation"] in ("validate_existing", "normalize_profile"):
                try:
                    number = float(raw)
                    if number.is_integer():
                        raw = str(int(number))
                except (ValueError, OverflowError):
                    pass
            normalized = config_io._validate(key, raw, field.type)
            if normalized is None:
                raise ValueError("配置值类型不合法：" + key)
            if field.sensitive and raw.strip() in ("", "********"):
                continue
            values[key] = normalized
        result = values
    else:
        raise ValueError("配置读写必须使用受管配置拥有方")
print("NEH_RESULT=" + json.dumps(result, ensure_ascii=False))
'''


def base_environment():
    """业务只继承用户身份与语言，程序和配置由安装提供。"""
    environment = {key: value for key, value in os.environ.items()
                   if key in ("HOME", "USER", "LOGNAME", "LANG") or key.startswith("LC_")}
    environment["PATH"] = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    return environment


def clean_environment(prefix=None, *, environment=None):
    environment = os.environ if environment is None else environment
    environment = {key: value for key, value in environment.items()
                   if not key.startswith(("NUWAX_", "GO2_", "PYTHON", "VIRTUAL_ENV", "CONDA")) and key not in ("LD_PRELOAD", "LD_LIBRARY_PATH")}
    environment["PYTHONNOUSERSITE"] = "1"
    if prefix:
        environment["LD_LIBRARY_PATH"] = str(prefix / "lib") + ":" + str(prefix / "lib64")
        environment["NUWAX_ROBOT_PROFILE_MODE"] = "1"
        environment["NUWAX_MANAGED_HOME"] = str(prefix.parent.parent)
        environment["NEH_MANAGER_LOG_DIR"] = str(prefix.parent.parent / "logs")
    return environment



def redactor(home=None):
    """隐藏配置密钥和网址中的认证部分；进入网页的诊断与详细日志共用。"""
    secrets = []
    if home:
        from scripts.module_installer.runtime import read_settings
        active = Path(home) / "data/settings/active.env"
        secrets = [value for key, value in read_settings(active).items()
                   if re.search(r"PASSWORD|TOKEN|SECRET|API_KEY|AUTHORIZATION", key, re.I) and len(value) >= 4]

    def redact(text):
        for value in secrets:
            text = text.replace(value, "[已隐藏]")
        text = re.sub(r'(?i)Bearer\s+[^\s,;]+', 'Bearer [已隐藏]', text)
        text = re.sub(r'(?i)(["\']?(?:password|token|secret|api[_-]?key|authorization)["\']?\s*[:=]\s*)[^\s,;]+', r'\1[已隐藏]', text)
        text = re.sub(r'(https?://)[^/@\s]+:[^/@\s]+@', r'\1[已隐藏]@', text)
        return re.sub(r'(https?://[^\s?]+)\?[^\s]+', r'\1?[已隐藏]', text)
    return redact


def safe_diagnostic(text, home=None):
    """只向网页返回有限诊断，隐藏配置密钥和网址中的认证部分。"""
    return "\n".join(redactor(home)(text[-6000:]).splitlines()[-20:])[-4000:]


_PROTOCOL_LINES = ("NEH_RESULT=", "NEH_SOURCE_RESULT=")


def job_output_path(home=None, job_id=None):
    """当前任务的详细日志：受管命令的逐行输出（已脱敏）与阶段标记。"""
    home = home or os.environ.get("NEH_MANAGER_WORKER_HOME")
    job_id = job_id or os.environ.get("NEH_MANAGER_WORKER_JOB")
    return Path(home) / "logs" / (job_id + ".output.log") if home and job_id else None


def append_job_output(text, home=None, job_id=None):
    path = job_output_path(home, job_id)
    if path:
        with path.open("a", encoding="utf-8") as handle:
            handle.write(text if text.endswith("\n") else text + "\n")


def failure_reason(text):
    """从失败命令的输出中取出可直接展示的原因。"""
    lines = [line.strip() for line in text.splitlines() if line.strip() and not line.startswith(_PROTOCOL_LINES)]
    if not lines:
        return ""
    last = lines[-1]
    try:
        # 系统助手以一行JSON报告错误，取其中包管理器输出的最后一行。
        parsed = json.loads(last)
        if isinstance(parsed, dict) and isinstance(parsed.get("error"), str):
            inner = [line.strip() for line in parsed["error"].splitlines() if line.strip()]
            return inner[-1] if inner else parsed["error"]
    except ValueError:
        pass
    match = re.match(r"^([\w.]+(?:Error|Exception)):\s*(.*)$", last)
    if not match:
        return last[-500:]
    kind, message = match.groups()
    if not kind.endswith("CalledProcessError"):
        return message[-500:]
    # 子命令失败：Python回溯只说明哪条命令失败，原因在回溯之前的输出里。
    command = re.search(r"Command '(\[.*\])' returned", message)
    try:
        words = [str(word) for word in ast.literal_eval(command.group(1))] if command else []
    except (ValueError, SyntaxError):
        words = []
    shown = " ".join([Path(words[0]).name] + [word for word in words[1:4] if not word.startswith("/") and " " not in word]) if words else "子命令"
    start = max(index for index, line in enumerate(lines) if line.startswith("Traceback"))
    before = [line for line in lines[:start] if not line.startswith("WARNING")]
    errors = [line for line in before if re.search(r"(?i)\berror\b|错误", line) and not re.match(r"^make(\[\d+\])?: \*\*\*", line)]
    detail = errors[0] if errors else (before[-1] if before else "")
    return (shown + " 失败：" + detail)[-500:] if detail else message[-500:]


def runtime_diagnostic(home):
    path = Path(home) / "logs/runtime.log"
    try:
        with path.open("rb") as handle:
            handle.seek(max(0, path.stat().st_size - 16384))
            return {"diagnostic": safe_diagnostic(handle.read().decode("utf-8", errors="replace"), home)}
    except OSError:
        return None


def run_command(argv, *, cwd=None, environment=None, payload=None, timeout=600, cancelled=lambda: None):
    # 单独解释器在exec前设置父进程退出信号，避免在线程服务中运行preexec_fn。
    guard = PACKAGE_ROOT / "scripts/device_manager/subprocess_guard.py"
    process = subprocess.Popen([sys.executable, str(guard), str(os.getpid())] + list(argv), cwd=cwd, env=environment, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    command_state = None
    if os.environ.get("NEH_MANAGER_WORKER_HOME") and os.environ.get("NEH_MANAGER_WORKER_JOB"):
        command_state = Path(os.environ["NEH_MANAGER_WORKER_HOME"]) / "jobs" / (os.environ["NEH_MANAGER_WORKER_JOB"] + ".command")
        atomic_json(command_state, process_identity(process.pid))
    data = payload.encode("utf-8") if isinstance(payload, str) else payload
    deadline = time.monotonic() + timeout
    # 任务进程中逐行写入详细日志，网页据此实时显示；内部协议行可能带配置值，不写入。
    log_path = job_output_path()
    redact = redactor((environment or {}).get("NUWAX_MANAGED_HOME") or os.environ.get("NEH_MANAGER_WORKER_HOME"))
    chunks = []

    def pump():
        log = log_path.open("a", encoding="utf-8") if log_path else None
        try:
            if log:
                log.write(redact("$ " + shlex.join(str(item) for item in argv)) + "\n")
                log.flush()
            for raw in iter(process.stdout.readline, b""):
                chunks.append(raw)
                line = raw.decode("utf-8", errors="replace")
                if log and not line.startswith(_PROTOCOL_LINES):
                    log.write(redact(line if line.endswith("\n") else line + "\n"))
                    log.flush()
        finally:
            if log:
                log.close()

    def feed():
        try:
            if data:
                process.stdin.write(data)
        except BrokenPipeError:
            pass
        finally:
            try:
                process.stdin.close()
            except BrokenPipeError:
                pass

    reader = threading.Thread(target=pump, daemon=True)
    reader.start()
    threading.Thread(target=feed, daemon=True).start()
    try:
        while reader.is_alive() or process.poll() is None:
            cancelled()
            if time.monotonic() > deadline:
                raise ManagerError("command_timeout", "安装或拥有方操作超时，正在恢复。", 500)
            reader.join(0.25)
        process.wait()
        output = b"".join(chunks)
        if process.returncode:
            # 配置和第三方进程可能输出密钥；未脱敏的原始输出只留在本地文件，接口只给脱敏后的原因与诊断。
            details = None
            if environment and environment.get("NEH_MANAGER_LOG_DIR"):
                directory = Path(environment["NEH_MANAGER_LOG_DIR"])
                directory.mkdir(parents=True, exist_ok=True)
                from .common import atomic_write
                path = directory / ("command-" + str(process.pid) + ".log")
                atomic_write(path, output)
                details = {"log_file": str(path), "diagnostic": safe_diagnostic(output.decode("utf-8", errors="replace"), environment.get("NUWAX_MANAGED_HOME"))}
            reason = redact(failure_reason(output.decode("utf-8", errors="replace")))
            raise ManagerError("command_failed", reason or "受管步骤失败（退出码 {}）：{}".format(process.returncode, Path(argv[0]).name), 500, details)
        return output.decode("utf-8", errors="replace")
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        if command_state:
            command_state.unlink(missing_ok=True)


def probe(prefix, operation, **kwargs):
    request = dict(kwargs, operation=operation)
    output = run_command([str(prefix / "venv/bin/python"), "-c", PROBE_CODE], cwd=prefix / "source",
                         environment=clean_environment(prefix), payload=json.dumps(request), timeout=120)
    for line in reversed(output.splitlines()):
        if line.startswith("NEH_RESULT="):
            return json.loads(line[len("NEH_RESULT="):])
    raise ManagerError("config_unavailable", "版本解释器没有返回完整配置结果。", 503)


def system_helper(action, payload=None):
    if not Path("/usr/local/libexec/neh-system-helper").is_file():
        raise ManagerError("system_setup_required", "系统依赖准备入口尚未安装，请由设备管理员完成引导器的系统集成步骤。", 409)
    environment = None
    if os.environ.get("NEH_MANAGER_WORKER_HOME"):
        # 助手失败输出（如包管理器报错）与其它受管步骤一样落盘并进入诊断。
        environment = dict(os.environ, NEH_MANAGER_LOG_DIR=str(Path(os.environ["NEH_MANAGER_WORKER_HOME"]) / "logs"))
    return run_command(["sudo", "-n", "/usr/local/libexec/neh-system-helper", action],
                       environment=environment, payload=json.dumps(payload or {}), timeout=1800)


class Runtime:
    def __init__(self, home, manager_home=None):
        self.home = home
        self.manager_home = manager_home or home
        self.state_path = home / "config/runtime.json"

    def port(self):
        return read_json(self.manager_home / "config/settings.json", {}).get("gateway_port", 8766)

    def _pid(self):
        try:
            return int((self.home / "data/gateway/gateway.pid").read_text().strip())
        except (OSError, ValueError):
            return 0

    def _supervisor(self):
        pid = self._pid()
        identity = process_identity(pid)
        if not identity:
            return None
        record = read_json(self.state_path, {})
        if (record.get("process") or {}).get("pid") == pid:
            return identity if record["process"] == identity else None
        prefix = Path(record["prefix"]) if record.get("prefix") else (self.home / "current").resolve()
        try:
            if Path("/proc/{}/cwd".format(pid)).resolve() == prefix / "source":
                return identity
        except OSError:
            pass
        return None

    def _foreign_owners(self, allowed_roots=()):
        markers = (b"nuwax-geometry-runtime", b"nuwax-nav-stack", b"nuwax-person-follow", b"drivers.go2.video_streamer")
        for directory in Path("/proc").iterdir():
            if not directory.name.isdigit() or int(directory.name) == os.getpid():
                continue
            try:
                command = (directory / "cmdline").read_bytes().split(b"\0")
                arguments = [part for part in command if part]
                # 只匹配可执行文件及Python模块参数，避免把终端中的检索命令当作拥有方。
                relevant = arguments[:1] + [arguments[index + 1] for index, item in enumerate(arguments[:-1]) if item == b"-m"]
                if not any(marker in item for marker in markers for item in relevant):
                    continue
                source = (directory / "cwd").resolve()
                if self.home not in source.parents and not any(source == root or root in source.parents for root in allowed_roots):
                    return True
            except (OSError, PermissionError):
                continue
        return False

    def has_owner(self):
        record = read_json(self.state_path, {})
        return process_alive(record.get("process")) or bool(self._supervisor())

    def update_readiness(self):
        if not self.has_owner():
            # 已停止的受管拥有方没有可被本次更新中断的任务。
            return {"ready": True}
        try:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            try:
                response = opener.open("http://127.0.0.1:{}/api/updates/readiness".format(self.port()), timeout=3)
            except urllib.error.URLError as exc:
                if isinstance(exc.reason, ConnectionRefusedError):
                    return {"ready": True}
                raise
            except ConnectionRefusedError:
                return {"ready": True}
            with response as response:
                result = json.loads(response.read(65537))
            if isinstance(result, dict) and type(result.get("ready")) is bool:
                return result
        except (OSError, ValueError):
            pass
        return {"ready": False, "reason": "无法读取新鲜的网关空闲状态，稍后重查"}

    def status(self):
        record = read_json(self.state_path, {})
        owned = process_alive(record.get("process"))
        supervisor = self._supervisor()
        running = owned or bool(supervisor)
        healthy = False
        if running:
            try:
                with urllib.request.build_opener(urllib.request.ProxyHandler({})).open("http://127.0.0.1:{}/api/health".format(self.port()), timeout=2) as response:
                    result = json.loads(response.read(2 * 1024 * 1024))
                    healthy = response.status == 200 and isinstance(result, dict)
            except (OSError, ValueError):
                pass
        result = {"running": running, "healthy": healthy, "console_url": "/console"}
        if running and not healthy:
            result["error"] = "网关进程存在，HTTP健康接口尚未响应；算法和设备就绪请查看控制台。"
        return result

    def start(self, prefix):
        if self._foreign_owners():
            raise ManagerError("foreign_owner", "检测到其它安装的业务拥有方，请先迁入或通过原拥有方停止，拒绝调用全局清理。", 409)
        if self.status()["running"]:
            raise ManagerError("already_running", "业务拥有方已运行，拒绝并行创建。", 409)
        from scripts.module_installer.runtime import _VALUES
        values = _VALUES.configuration_values(root=prefix / "source", environment={
            "NUWAX_MANAGED_HOME": str(self.home), "NUWAX_ROBOT_PROFILE_MODE": "1"})
        if str(values.get("NUWAX_REAL_LCM_REQUIRED", "")).lower() in ("true", "1", "yes", "on") and Path("/usr/local/libexec/neh-system-helper").is_file():
            system_helper("prepare-network", {"url": values.get("NUWAX_LCM_URL", "udpm://239.255.76.67:7667?ttl=0")})
        log_path = self.home / "logs/runtime.log"
        descriptor = os.open(str(log_path), os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o600)
        with os.fdopen(descriptor, "ab") as log:
            process = subprocess.Popen(["bash", str(prefix / "start.sh"), "start"], cwd=prefix / "source",
                                       env=clean_environment(prefix, environment=base_environment()),
                                       stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=log, start_new_session=True)
        atomic_json(self.state_path, {"process": process_identity(process.pid), "prefix": str(prefix)})
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise ManagerError("start_failed", "原网关拥有方提前退出。", 500, runtime_diagnostic(self.home))
            if self.status()["healthy"]:
                return
            time.sleep(0.4)
        raise ManagerError("health_timeout", "网关健康接口未在启动预算内响应。", 500, runtime_diagnostic(self.home))

    def stop(self, prefix):
        record = read_json(self.state_path, {})
        original = record.get("process") or self._supervisor()
        # 兼容旧发布启动器把运行锁持有到退出的实现：先请求原supervisor执行自身关闭trap。
        # 只向已核验身份的根拥有方发信号，不直接操作算法工作进程。
        if process_alive(original):
            try:
                os.kill(original["pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 180
            while process_alive(original) and time.monotonic() < deadline:
                time.sleep(0.2)
            if process_alive(original):
                raise ManagerError("stop_incomplete", "原拥有方未在关闭预算内退出，拒绝切换版本。", 500)
        run_command(["bash", str(prefix / "start.sh"), "stop"], cwd=prefix / "source",
                    environment=clean_environment(prefix), timeout=180)
        deadline = time.monotonic() + 20
        while process_alive(original) and time.monotonic() < deadline:
            time.sleep(0.2)
        if process_alive(original) or process_identity(self._pid()):
            raise ManagerError("stop_incomplete", "原拥有方尚未完全退出，拒绝切换版本。", 500)
        atomic_json(self.state_path, {})
