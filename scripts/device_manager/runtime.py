"""通过业务监督者管理启停；配置通过当前版本的共同实现读写。"""
from __future__ import annotations

import ast
import json
import os
import re
import shlex
import signal
import socket
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
import json,sys
from pathlib import Path
from core.config import validate_business_values
request = json.load(sys.stdin)
try:
    if request["operation"] == "runtime":
        from scripts.module_installer.runtime import write_runtime
        result = write_runtime(Path.cwd(), Path(request["install_prefix"]), request["modules"], request["robot"], request["values"])
    else:
        result = validate_business_values(request["values"])
except (ValueError, TypeError, OverflowError) as error:
    print("NEH_CONFIG_ERROR=" + json.dumps(str(error), ensure_ascii=False))
else:
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
        environment["NUWAX_INSTALL_HOME"] = str(prefix.parent.parent)
        environment["NEH_MANAGER_LOG_DIR"] = str(prefix.parent.parent / "logs")
    return environment



def redactor(home=None):
    """隐藏配置密钥和网址中的认证部分；进入网页的诊断与详细日志共用。"""
    secrets = []
    if home:
        from scripts.module_installer.runtime import read_settings, _CONFIG
        active = Path(home) / "run/config.toml"
        specs = _CONFIG.specifications()
        if active.is_file():
            secrets = [value for key, value in read_settings(active).items()
                       if specs.get(key, {}).get("sensitive") and isinstance(value, str) and len(value) >= 4]


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


_PROTOCOL_LINES = ("NEH_RESULT=", "NEH_CONFIG_ERROR=", "NEH_SOURCE_RESULT=")


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
    redact = redactor((environment or {}).get("NUWAX_INSTALL_HOME") or os.environ.get("NEH_MANAGER_WORKER_HOME"))
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
                details = {"log_file": str(path), "diagnostic": safe_diagnostic(output.decode("utf-8", errors="replace"), environment.get("NUWAX_INSTALL_HOME"))}
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
        if line.startswith("NEH_CONFIG_ERROR="):
            raise ManagerError("invalid_config", json.loads(line[len("NEH_CONFIG_ERROR="):]), 400)
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
        from scripts.module_installer.runtime import _CONFIG
        return read_json(self.manager_home / "config/settings.json", {}).get("gateway_port", _CONFIG.schema_default("gateway.port"))

    def _components(self):
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(2)
            connection.connect(str(self.home / "run/supervisor.sock"))
            connection.sendall(b'{"op":"status"}\n')
            with connection.makefile("rb") as response:
                return json.loads(response.readline())

    def has_owner(self):
        try:
            self._components()
            return True
        except OSError:
            return False

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
        running = self.has_owner()
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
        if self.status()["running"]:
            raise ManagerError("already_running", "业务拥有方已运行，拒绝并行创建。", 409)
        from scripts.module_installer.runtime import _CONFIG
        values = _CONFIG.ConfigStore(self.home, prefix / "source", self.port()).effective()
        if values["transport"]["real_lcm_required"] and Path("/usr/local/libexec/neh-system-helper").is_file():
            system_helper("prepare-network", {"url": values["transport"]["lcm_url"]})
        log_path = self.home / "logs/runtime.log"
        descriptor = os.open(str(log_path), os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o600)
        with os.fdopen(descriptor, "ab") as log:
            process = subprocess.Popen([str(prefix / "venv/bin/python"), "-m", "scripts.supervisor"], cwd=prefix / "source",
                                       env=dict(clean_environment(prefix, environment=base_environment()),
                                                NUWAX_INSTALL_HOME=str(self.home)),
                                       stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=log, start_new_session=True)
        atomic_json(self.state_path, {"prefix": str(prefix)})
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise ManagerError("start_failed", "业务监督者提前退出。", 500, runtime_diagnostic(self.home))
            if self.status()["healthy"]:
                return
            time.sleep(0.4)
        raise ManagerError("health_timeout", "网关健康接口未在启动预算内响应。", 500, runtime_diagnostic(self.home))

    def stop(self, prefix):
        try:
            pid = self._components()["pid"]
        except OSError:
            self.state_path.unlink(missing_ok=True)
            return
        if pid:
            try:
                os.kill(pid, signal.SIGTERM)
                deadline = time.monotonic() + 60
                while time.monotonic() < deadline:
                    try:
                        state = (Path("/proc") / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()[0]
                    except FileNotFoundError:
                        break
                    if state == "Z":
                        break
                    time.sleep(0.2)
                else:
                    os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.state_path.unlink(missing_ok=True)
