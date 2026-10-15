#!/usr/bin/env python3
"""安装独立端侧管理入口；首次引导与管理器版本替换共用此程序。"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path, PurePosixPath
import pwd
import re
import signal
import shlex
import socket
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import uuid
import urllib.error
import urllib.parse
import urllib.request
import webbrowser


DEFAULT_CATALOG = "https://github.com/Molun0301/nuwax-embodied-harness/releases/latest/download/catalog.json"
DEFAULT_REPOSITORY = "https://github.com/Molun0301/nuwax-embodied-harness.git"
DEFAULT_BRANCH = "main"
MAX_MANAGER_BYTES = 128 * 1024 * 1024
MAX_MANAGER_UNPACKED = 512 * 1024 * 1024


class BootstrapError(RuntimeError):
    pass


def version_tuple(value):
    if not isinstance(value, str) or not re.fullmatch(r"\d+(\.\d+){1,2}", value):
        raise BootstrapError("管理包解释器版本声明无效")
    return tuple(int(part) for part in value.split("."))


def valid_url(value):
    parsed = urllib.parse.urlsplit(value)
    if parsed.username or parsed.password or parsed.fragment:
        raise BootstrapError("下载地址不能包含凭据或片段")
    if parsed.scheme == "https" and parsed.hostname:
        return value
    if parsed.scheme == "http":
        try:
            loopback = parsed.hostname == "localhost" or ipaddress.ip_address(parsed.hostname or "").is_loopback
        except ValueError:
            loopback = False
        if loopback:
            return value
    raise BootstrapError("远端下载必须使用HTTPS；本地验收允许回环HTTP")


class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        valid_url(newurl)
        redirect = super().redirect_request(req, fp, code, msg, headers, newurl)
        if redirect and urllib.parse.urlsplit(req.full_url).netloc != urllib.parse.urlsplit(newurl).netloc:
            redirect.remove_header("Authorization")
        return redirect


def fetch(url, target, *, token="", limit=MAX_MANAGER_BYTES, expected_size=None):
    valid_url(url)
    if token:
        url = github_asset_url(url, token)
    headers = {"User-Agent": "NEH-bootstrap/1", "Accept": "application/octet-stream"}
    if token:
        headers["Authorization"] = "Bearer " + token
    request = urllib.request.Request(url, headers=headers)
    total = 0
    digest = hashlib.sha256()
    try:
        with urllib.request.build_opener(SafeRedirect()).open(request, timeout=45) as response, target.open("wb") as output:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                total += len(chunk)
                if total > limit:
                    raise BootstrapError("下载超过管理包大小上限")
                digest.update(chunk)
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())
    except urllib.error.HTTPError as exc:
        raise BootstrapError("下载失败（HTTP %s）；请核对版本已发布及私有仓库访问权限" % exc.code) from exc
    if expected_size is not None and total != expected_size:
        raise BootstrapError("下载大小与版本清单不符")
    return digest.hexdigest()


def github_asset_url(url, token):
    """私有仓库的版本附件通过正式资产接口下载，令牌不进入网址。"""
    parsed = urllib.parse.urlsplit(url)
    parts = parsed.path.strip("/").split("/")
    if parsed.hostname != "github.com" or len(parts) < 6 or parts[2] != "releases":
        return url
    repository = "/".join(parts[:2])
    if parts[3:5] == ["latest", "download"]:
        release_path = "latest"
        filename = urllib.parse.unquote("/".join(parts[5:]))
    elif parts[3] == "download" and len(parts) >= 6:
        release_path = "tags/" + parts[4]
        filename = urllib.parse.unquote("/".join(parts[5:]))
    else:
        return url
    request = urllib.request.Request("https://api.github.com/repos/" + repository + "/releases/" + release_path,
                                     headers={"Authorization": "Bearer " + token, "Accept": "application/vnd.github+json",
                                              "User-Agent": "NEH-bootstrap/1"})
    with urllib.request.build_opener(SafeRedirect()).open(request, timeout=30) as response:
        payload = response.read(8 * 1024 * 1024 + 1)
    if len(payload) > 8 * 1024 * 1024:
        raise BootstrapError("版本资产目录过大")
    metadata = json.loads(payload)
    for asset in metadata.get("assets", []):
        if asset.get("name") == filename:
            candidate = asset.get("url", "")
            if urllib.parse.urlsplit(candidate).netloc != "api.github.com":
                raise BootstrapError("版本资产地址不可信")
            return candidate
    raise BootstrapError("已发布版本缺少附件：" + filename)


def matching_token(source, target, token):
    origin = urllib.parse.urlsplit(source).netloc
    destination = urllib.parse.urlsplit(target).netloc
    return token if origin == destination or (origin == "github.com" and destination == "api.github.com") else ""


def digest_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def unpack_manager(archive, destination):
    """手动展开已验证普通文件，不使用可写出目录外的extractall。"""
    with tarfile.open(str(archive), "r:gz") as bundle:
        members = []
        seen = set()
        total = 0
        for member in bundle:
            path = PurePosixPath(member.name)
            if (not member.name or path.is_absolute() or ".." in path.parts or "\\" in member.name
                    or path.as_posix() != member.name.rstrip("/") or member.name.rstrip("/") in seen
                    or not (member.isfile() or member.isdir())):
                raise BootstrapError("管理包包含非法路径、重复文件或链接")
            seen.add(member.name.rstrip("/"))
            total += member.size
            if total > MAX_MANAGER_UNPACKED or len(seen) > 20000:
                raise BootstrapError("管理包展开后超过限制")
            members.append(member)
        try:
            entry = bundle.getmember("manager-manifest.json")
        except KeyError as exc:
            raise BootstrapError("管理包缺少manager-manifest.json") from exc
        if entry.size > 8 * 1024 * 1024:
            raise BootstrapError("管理包清单过大")
        reader = bundle.extractfile(entry)
        if reader is None:
            raise BootstrapError("管理包清单无法读取")
        with reader:
            manifest = json.load(reader)
        if manifest.get("schema_version") != 1 or manifest.get("product") != "neh-manager":
            raise BootstrapError("不是受支持的NEH管理包")
        python = manifest.get("python", {})
        if not (version_tuple(python.get("minimum")) <= sys.version_info[:2]
                < version_tuple(python.get("maximum_exclusive"))):
            raise BootstrapError("此管理包不支持当前Python解释器")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,95}", str(manifest.get("version", ""))):
            raise BootstrapError("管理版本名称无效")
        listed = manifest.get("files")
        if not isinstance(listed, list):
            raise BootstrapError("管理包文件清单无效")
        expected = {}
        for item in listed:
            if not isinstance(item, dict) or not isinstance(item.get("path"), str) or item["path"] in expected:
                raise BootstrapError("管理包文件清单包含重复或无效项")
            expected[item["path"]] = item
        regular = {m.name for m in members if m.isfile() and m.name != "manager-manifest.json"}
        if set(expected) != regular:
            raise BootstrapError("管理包有未声明或缺失的文件")
        required = {"scripts/device_manager/__main__.py", "scripts/neh_system_helper.py",
                    "scripts/systemd/check_lcm_multicast.sh"}
        if not required.issubset(regular):
            raise BootstrapError("管理包缺少启动或系统集成程序")
        for member in members:
            target = destination / member.name
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            source = bundle.extractfile(member)
            if source is None:
                raise BootstrapError("无法读取管理包文件")
            digest = hashlib.sha256()
            with source, target.open("xb") as output:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    digest.update(chunk)
                    output.write(chunk)
            target.chmod(0o755 if member.mode & 0o111 else 0o644)
            if member.name != "manager-manifest.json":
                item = expected[member.name]
                if member.size != item.get("size") or digest.hexdigest() != item.get("sha256"):
                    raise BootstrapError("管理包文件校验失败：" + member.name)
    return manifest


def switch_link(path, target):
    temporary = path.with_name("." + path.name + "-" + str(os.getpid()))
    if path.exists() and not path.is_symlink():
        raise BootstrapError("安装位置不是受管链接：" + str(path))
    try:
        temporary.symlink_to(os.path.relpath(str(target), str(path.parent)))
        os.replace(str(temporary), str(path))
    finally:
        if temporary.is_symlink():
            temporary.unlink()


def systemd_quote(value):
    if any(char in value for char in ("\n", "\r", "\x00")):
        raise BootstrapError("系统服务路径包含非法字符")
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%").replace("$", "$$") + '"'


def service_text(home, python, host, port, user=None):
    root = home / "manager/current"
    # WorkingDirectory是单个路径值，不采用ExecStart的参数引号语法。
    systemd_quote(str(root))
    working_directory = str(root).replace("%", "%%")
    unit = ["[Unit]", "Description=NEH 端侧管理服务", "After=network-online.target", "Wants=network-online.target",
            "", "[Service]", "Type=simple", "Environment=PYTHONNOUSERSITE=1", "WorkingDirectory=" + working_directory,
            "ExecStart=" + " ".join(systemd_quote(str(item)) for item in (
                python, "-m", "scripts.device_manager", "--home", home, "serve", "--host", host, "--port", port)),
            # 管理服务重启不得结束持久任务和原业务拥有方；它们由显式任务与自己的账本回收。
            "Restart=on-failure", "RestartSec=3", "TimeoutStopSec=45", "KillMode=process", "UMask=0077"]
    if user:
        unit.extend(["User=" + user, "Group=" + str(pwd.getpwnam(user).pw_gid)])
    unit += ["", "[Install]", "WantedBy=" + ("multi-user.target" if user else "default.target"), ""]
    return "\n".join(unit)


def manager_url(host, port):
    local = {"0.0.0.0": "127.0.0.1", "::": "::1"}.get(host, host)
    return "http://%s:%s/" % ("[" + local + "]" if ":" in local else local, port)


def manager_session(host, port):
    # 本机身份探测不经过用户代理，也不跟随其它服务的重定向。
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *arguments, **options):
            return None
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    with opener.open(manager_url(host, port) + "api/manager/session", timeout=1) as response:
        return json.loads(response.read(64 * 1024))


def choose_manager_port(home, host, requested=None):
    """默认安装复用本安装端口，或选择空闲端口；不占用业务网关端口。"""
    def saved(name):
        path = home / "config" / name
        try:
            value = json.loads(path.read_text())
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}
    gateway_port = saved("settings.json").get("gateway_port", 8766)
    remembered = saved("manager-listen.json").get("port")
    identity = home / "config/instance-id"
    instance_id = identity.read_text().strip() if identity.is_file() else ""
    candidates = [requested] if requested is not None else [remembered, *range(8765, 8866)]
    for port in dict.fromkeys(candidates):
        if type(port) is not int or not 1 <= port <= 65535 or port == gateway_port:
            continue
        try:
            with socket.socket(socket.AF_INET6 if ":" in host else socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((host, port))
            if requested is None and port != (remembered or 8765):
                print("默认管理端口已被占用，自动使用端口%s。" % port, flush=True)
            return port
        except OSError:
            if instance_id:
                try:
                    if manager_session(host, port).get("instance_id") == instance_id:
                        return port
                except (OSError, ValueError, AttributeError):
                    pass
    raise BootstrapError("指定管理端口不可用或与业务端口冲突；省略--port可自动选择空闲端口" if requested is not None
                         else "未找到可用管理端口，请检查8765至8865的监听情况")


def wait_for_manager(host, port, instance_id, manager_version):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        try:
            actual = manager_session(host, port)
            if actual.get("instance_id") == instance_id and actual.get("manager_version") == manager_version:
                return manager_url(host, port)
        except (OSError, ValueError, AttributeError):
            pass
        time.sleep(0.2)
    raise BootstrapError("管理服务未通过实例和版本核验；请查看logs/manager.log或系统服务日志")


def activate_service(args, python, path, command, name, user=None):
    """启动失败先停止新服务并恢复服务文件，外层随后回退管理程序链接。"""
    previous = path.read_bytes() if path.is_file() and not args.new_install else None
    enabled = subprocess.run(command + ["is-enabled", "--quiet", name], check=False).returncode == 0 if previous else False
    try:
        path.write_text(service_text(args.home, python, args.host, args.port, user), encoding="utf-8")
        path.chmod(0o644)
        subprocess.run(command + ["daemon-reload"], check=True)
        subprocess.run(command + ["enable", name], check=True)
        subprocess.run(command + ["restart", name], check=True)
        return wait_for_manager(args.host, args.port, args.expected_instance, args.expected_version)
    except BaseException:
        subprocess.run(command + ["stop", name], check=True)
        if previous is None:
            subprocess.run(command + ["disable", name], check=True)
            path.unlink(missing_ok=True)
        else:
            path.write_bytes(previous)
            subprocess.run(command + ["enable" if enabled else "disable", name], check=True)
        subprocess.run(command + ["daemon-reload"], check=True)
        raise


def provision_system(args):
    """此入口仅在终端明确安装时经sudo调用；网页无权调用本程序。"""
    if os.geteuid() != 0 or not args.user or not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_.-]*\$?", args.user):
        raise BootstrapError("系统服务安装需要有效普通用户及管理员权限")
    if args.service == "system" and (args.port is None or not args.expected_instance or not args.expected_version):
        raise BootstrapError("系统服务安装缺少已选择端口或待验证的管理身份")
    account = pwd.getpwnam(args.user)
    if account.pw_uid == 0:
        raise BootstrapError("管理服务必须使用普通用户")
    home = args.home.resolve()
    root = home / "manager/current"
    helper = root / "scripts/neh_system_helper.py"
    checker = root / "scripts/systemd/check_lcm_multicast.sh"
    for path in (helper, checker):
        if not path.is_file():
            raise BootstrapError("管理包缺少特权集成文件")
    libexec = Path("/usr/local/libexec")
    libexec.mkdir(parents=True, exist_ok=True)
    for source, name in ((helper, "neh-system-helper"), (checker, "neh-check-lcm-multicast.sh")):
        target = libexec / name
        descriptor, temporary = tempfile.mkstemp(prefix=".neh-", dir=str(libexec))
        try:
            with os.fdopen(descriptor, "wb") as output, source.open("rb") as input_file:
                shutil.copyfileobj(input_file, output)
            os.chmod(temporary, 0o755)
            os.chown(temporary, 0, 0)
            os.replace(temporary, str(target))
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    sudoers = Path("/etc/sudoers.d/neh-manager-" + str(account.pw_uid))
    sudoers.parent.mkdir(parents=True, exist_ok=True)
    text = args.user + " ALL=(root) NOPASSWD: /usr/local/libexec/neh-system-helper\n"
    descriptor, temporary = tempfile.mkstemp(prefix=".neh-", dir=str(sudoers.parent))
    try:
        with os.fdopen(descriptor, "w") as handle:
            handle.write(text)
        os.chmod(temporary, 0o440)
        subprocess.run(["visudo", "-cf", temporary], check=True, stdout=subprocess.DEVNULL)
        os.replace(temporary, str(sudoers))
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    if args.service == "system":
        name = "neh-manager-%s.service" % account.pw_uid
        target = Path("/etc/systemd/system") / name
        activate_service(args, args.python, target, ["systemctl"], name, args.user)
    return 0


def cli_command(home, python, *arguments):
    return [str(python), "-m", "scripts.device_manager", "--home", str(home), *arguments]


def python_environment():
    environment = dict(os.environ)
    environment.pop("PYTHONHOME", None)
    environment.pop("PYTHONPATH", None)
    environment["PYTHONNOUSERSITE"] = "1"
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    return environment


def stop_local_manager(home):
    path = home / "manager/bootstrap.pid"
    if not path.is_file():
        return
    stopped = None
    try:
        pid = int(path.read_text().strip())
        proc = Path("/proc") / str(pid)
        arguments = (proc / "cmdline").read_bytes().split(b"\0")
        root = (proc / "cwd").resolve()
        if (pid <= 1 or proc.stat().st_uid != os.getuid() or b"scripts.device_manager" not in arguments
                or b"--home" not in arguments or os.fsencode(str(home)) not in arguments
                or home / "manager/versions" not in root.parents):
            raise BootstrapError("既有后台进程身份不匹配，拒绝发送停止信号")
        os.kill(pid, signal.SIGTERM)
        stopped = {"command": [os.fsdecode(value) for value in arguments if value], "cwd": root}
        deadline = time.monotonic() + 15
        while proc.exists() and time.monotonic() < deadline:
            try:
                if (proc / "stat").read_text().rsplit(")", 1)[1].split()[0] == "Z":
                    break
            except FileNotFoundError:
                break
            time.sleep(0.1)
        else:
            if proc.exists():
                raise BootstrapError("既有管理进程未结束，请查看管理日志")
    except (FileNotFoundError, ProcessLookupError):
        pass
    path.unlink(missing_ok=True)
    return stopped


def start_local_manager(home, command, root):
    log_path = home / "logs/manager.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("ab") as output:
        process = subprocess.Popen(command, cwd=str(root), env=python_environment(), stdout=output,
                                   stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    (home / "manager/bootstrap.pid").write_text(str(process.pid), encoding="utf-8")
    return process


def launch(args, python, instance_id, manager_version):
    home = args.home.resolve()
    root = home / "manager/current"
    args.expected_instance, args.expected_version = instance_id, manager_version
    if args.service == "system":
        command = ["sudo", str(Path(sys.executable).resolve()), "-I", str(Path(__file__).resolve()), "--provision-system", "--user", pwd.getpwuid(os.getuid()).pw_name,
                   "--home", str(home), "--python", str(python), "--host", args.host, "--port", str(args.port), "--service", "system",
                   "--expected-instance", instance_id, "--expected-version", manager_version]
        if args.new_install:
            command.append("--new-install")
        subprocess.run(command, check=True)
        return manager_url(args.host, args.port)
    if args.service == "user":
        units = Path.home() / ".config/systemd/user"
        units.mkdir(parents=True, exist_ok=True)
        url = activate_service(args, python, units / "neh-manager.service", ["systemctl", "--user"], "neh-manager.service")
        print("用户服务已安装；开机无人登录运行需要系统管理员为该用户启用持续运行。")
        return url
    args.stopped_local_manager = stop_local_manager(home)
    process = start_local_manager(home, cli_command(home, python, "serve", "--host", args.host, "--port", str(args.port)), root)
    try:
        return wait_for_manager(args.host, args.port, instance_id, manager_version)
    except BaseException:
        process.poll()
        stop_local_manager(home)
        raise


def read_token(path):
    if path is None:
        return ""
    if not path.is_file() or path.stat().st_mode & 0o077:
        raise BootstrapError("下载凭据文件须仅当前用户可读（权限0600）")
    value = path.read_text(encoding="utf-8").strip()
    if not value or "\n" in value or len(value) > 4096:
        raise BootstrapError("下载凭据文件无效")
    return value


def device_urls(port):
    """只枚举本机地址，直接给无头安装者可复制的网址。"""
    result = []
    try:
        process = subprocess.run(["ip", "-json", "address", "show", "scope", "global"],
                                 capture_output=True, text=True, timeout=3, check=True)
        for interface in json.loads(process.stdout):
            if interface.get("operstate") == "DOWN":
                continue
            for info in interface.get("addr_info", []):
                address = ipaddress.ip_address(info["local"])
                if address.is_loopback or address.is_link_local:
                    continue
                host = "[" + str(address) + "]" if address.version == 6 else str(address)
                result.append("http://%s:%s/" % (host, port))
    except (OSError, ValueError, KeyError, subprocess.SubprocessError):
        pass
    return list(dict.fromkeys(result))


def source_manager(args, destination, token):
    """在持久源码缓存锁内固定提交并组装管理源码，服务只运行独立版本目录。"""
    root = str(Path(__file__).resolve().parents[1])
    if root not in sys.path:
        sys.path.insert(0, root)
    from scripts.neh_source import SourceError, source_checkout
    if not shutil.which("git"):
        raise BootstrapError("源码安装需要git；请使用get-neh.sh自动准备，或先安装git")
    try:
        with source_checkout(args.home, args.repository, args.branch, args.revision, token=token,
                             token_origin=args.repository, allow_local=args.allow_local_source,
                             progress=lambda message: print(message, flush=True)) as source:
            archive = destination / "manager.tar.gz"
            print("从固定源码组装管理程序：" + source["version"], flush=True)
            subprocess.run([sys.executable, "-B", "-m", "scripts.product_release", "manager",
                            "--root", str(source["root"]), "--output", str(archive), "--version", source["version"]],
                           cwd=str(source["root"]), env=python_environment(), check=True, timeout=180)
            return archive, {"version": source["version"], "revision": source["revision"]}
    except SourceError as exc:
        raise BootstrapError(str(exc)) from exc


def install(args):
    if sys.platform != "linux":
        raise BootstrapError("当前端侧安装入口支持Linux；其它系统尚未完成验证")
    if os.geteuid() == 0:
        raise BootstrapError("请用普通用户运行引导安装，系统服务步骤会单独请求管理员权限")
    home = args.home.expanduser().resolve()
    args.home = home
    home.mkdir(parents=True, exist_ok=True, mode=0o700)
    if home.stat().st_uid != os.getuid() or home.stat().st_mode & 0o022:
        raise BootstrapError("安装目录必须归当前用户持有且其它用户不可写")
    manager = home / "manager"
    versions = manager / "versions"
    versions.mkdir(parents=True, exist_ok=True)
    token = read_token(args.token_file)
    with (manager / "bootstrap.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        args.port = choose_manager_port(home, args.host, args.port)
        with tempfile.TemporaryDirectory(prefix=".prepare-", dir=str(versions)) as temporary:
            stage = Path(temporary)
            metadata = None
            if args.manager_package:
                archive = args.manager_package.expanduser().resolve()
                if not archive.is_file() or archive.stat().st_size > MAX_MANAGER_BYTES:
                    raise BootstrapError("本地管理包不存在或超过大小限制")
                digest = digest_file(archive)
                if args.sha256 and digest != args.sha256:
                    raise BootstrapError("本地管理包摘要不符")
            elif args.catalog:
                catalog_path = stage / "catalog.json"
                if args.catalog.startswith(("https://", "http://")):
                    fetch(args.catalog, catalog_path, token=token, limit=8 * 1024 * 1024)
                else:
                    shutil.copyfile(Path(args.catalog).expanduser(), catalog_path)
                catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
                if catalog.get("schema_version") != 1 or catalog.get("product") != "neh":
                    raise BootstrapError("版本目录格式不受支持")
                metadata = catalog.get("manager")
                if not isinstance(metadata, dict) or not re.fullmatch(r"[0-9a-f]{64}", str(metadata.get("sha256", ""))):
                    raise BootstrapError("版本目录尚未提供可校验的管理包")
                if not isinstance(metadata.get("size"), int) or not 0 < metadata["size"] <= MAX_MANAGER_BYTES:
                    raise BootstrapError("管理包大小声明无效")
                archive = stage / "manager.tar.gz"
                url = urllib.parse.urljoin(args.catalog, metadata["url"])
                digest = fetch(url, archive, token=matching_token(args.catalog, url, token), expected_size=metadata["size"])
                if digest != metadata["sha256"]:
                    raise BootstrapError("管理包下载完整性校验失败")
            else:
                archive, metadata = source_manager(args, stage, token)
                digest = digest_file(archive)
            unpacked = stage / "unpacked"
            unpacked.mkdir()
            manifest = unpack_manager(archive, unpacked)
            if metadata and manifest["version"] != metadata.get("version"):
                raise BootstrapError("下载管理包与目录版本不一致")
            if metadata and metadata.get("revision") and manifest.get("revision") != metadata["revision"]:
                raise BootstrapError("管理包源码修订与本次固定提交不一致")
            python = Path(sys.executable).resolve()
            subprocess.run([str(python), "-m", "scripts.device_manager", "--help"], cwd=str(unpacked),
                           env=python_environment(), check=True, stdout=subprocess.DEVNULL, timeout=30)
            target = versions / (manifest["version"] + "-" + digest[:16])
            if target.exists():
                intact = all((target / item["path"]).is_file() and not (target / item["path"]).is_symlink()
                             and digest_file(target / item["path"]) == item["sha256"] for item in manifest["files"])
                if not intact:
                    target = versions / (manifest["version"] + "-" + digest[:16] + "-" + uuid.uuid4().hex[:8])
                    os.replace(str(unpacked), str(target))
            else:
                os.replace(str(unpacked), str(target))
            previous = (manager / "current").resolve() if (manager / "current").is_symlink() else None
            previous_state = {}
            if previous:
                state = subprocess.run(cli_command(home, python, "status"), cwd=str(previous), text=True,
                                       env=python_environment(), capture_output=True, check=True, timeout=30)
                previous_state = json.loads(state.stdout)
                if previous_state.get("active_job"):
                    raise BootstrapError("当前有管理任务正在执行，请完成后再更新管理程序")
            args.new_install = previous is None
            service_command = ["systemctl", "--user"] if args.service == "user" else ["systemctl"]
            service_name = "neh-manager.service" if args.service == "user" else "neh-manager-%s.service" % os.getuid()
            was_active = bool(previous and args.service != "none" and subprocess.run(
                service_command + ["is-active", "--quiet", service_name], check=False).returncode == 0)
            # 备份只存在于权限0700的准备目录内；保留文件权限及链接，不输出凭据。
            saved_files = {}
            for relative in ("config/settings.json", "config/credentials.json", "config/manager-listen.json", "bin/neh"):
                path = home / relative
                backup = stage / "rollback" / relative
                if path.exists() or path.is_symlink():
                    backup.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                    shutil.copy2(str(path), str(backup), follow_symlinks=False)
                    saved_files[path] = backup
                else:
                    saved_files[path] = None
            args.stopped_local_manager = None
            switch_link(manager / "current", target)
            binary = home / "bin/neh"
            try:
                binary.parent.mkdir(parents=True, exist_ok=True)
                launcher = stage / "neh-launcher"
                launcher.write_text("#!/bin/sh\nunset PYTHONHOME PYTHONPATH\nexport PYTHONNOUSERSITE=1\ncd " + shlex.quote(str(manager / "current")) + " || exit 1\nexec "
                                    + " ".join(shlex.quote(part) for part in cli_command(home, python)) + ' "$@"\n', encoding="utf-8")
                launcher.chmod(0o755)
                os.replace(str(launcher), str(binary))
                if not args.catalog and not args.manager_package:
                    settings = cli_command(home, python, "settings", "--source-repository", args.repository,
                                           "--source-branch", args.branch)
                    if args.allow_local_source:
                        settings.append("--allow-local-source")
                    subprocess.run(settings, cwd=str(target), env=python_environment(), check=True)
                elif args.catalog and args.catalog.startswith(("https://", "http://")):
                    subprocess.run(cli_command(home, python, "settings", "--feed-url", args.catalog, "--channel", args.channel),
                                   cwd=str(target), env=python_environment(), check=True)
                if token:
                    subprocess.run(cli_command(home, python, "settings", "--token-stdin"), input=token,
                                   text=True, cwd=str(target), env=python_environment(), check=True, stdout=subprocess.DEVNULL)
                state = subprocess.run(cli_command(home, python, "status"), cwd=str(target), text=True,
                                       env=python_environment(), capture_output=True, check=True, timeout=30)
                instance_id = json.loads(state.stdout).get("instance_id")
                if not instance_id:
                    raise BootstrapError("管理包缺少受管实例身份，无法核对实际启动程序")
                # 监听记录与设置、启动器一起提交；启动失败时恢复旧值。
                listen_path = home / "config/manager-listen.json"
                temporary = stage / "manager-listen.json"
                temporary.write_text(json.dumps({"host": args.host, "port": args.port}) + "\n")
                temporary.chmod(0o600)
                os.replace(str(temporary), str(listen_path))
                url = launch(args, python, instance_id, manifest["version"])
            except BaseException:
                for path, backup in saved_files.items():
                    if backup is None:
                        path.unlink(missing_ok=True)
                    else:
                        os.replace(str(backup), str(path))
                if previous:
                    switch_link(manager / "current", previous)
                    if was_active:
                        if args.service == "user":
                            subprocess.run(service_command + ["restart", service_name], check=True)
                        else:
                            subprocess.run(["sudo", "-n", "/usr/local/libexec/neh-system-helper", "restart-manager"],
                                           input="{}", text=True, check=True)
                    elif args.stopped_local_manager:
                        stopped = args.stopped_local_manager
                        command = stopped["command"]
                        start_local_manager(home, command, stopped["cwd"])
                        wait_for_manager(command[command.index("--host") + 1], int(command[command.index("--port") + 1]),
                                         previous_state["instance_id"], previous_state["manager_version"])
                else:
                    (manager / "current").unlink()
                raise
            # 服务已通过核验才登记可清理版本。复用管理事务锁，清理失败不回退已启动服务。
            try:
                with (home / "config/transaction.lock").open("a") as state_lock:
                    fcntl.flock(state_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    installed = stage / "installed.json"
                    installed.write_text(json.dumps({"complete": True, "product": "neh-manager", "version": manifest["version"]}) + "\n")
                    installed.chmod(0o600)
                    os.replace(str(installed), str(target / "installed.json"))
                    if previous and previous != target:
                        switch_link(manager / "previous", previous)
                    cleanup = subprocess.run([str(python), "-c",
                        "import json, sys; from pathlib import Path; from scripts.device_manager.storage import prune_storage; "
                        "print(json.dumps(prune_storage(Path(sys.argv[1]), package_root=Path.cwd())))", str(home)],
                        cwd=str(target), env=python_environment(), capture_output=True, text=True, check=True)
                    cleaned = json.loads(cleanup.stdout)
                    print("旧版本与缓存清理：移除%s项。" % len(cleaned["removed"]), flush=True)
                    for warning in cleaned["warnings"]:
                        print("清理提示：" + warning["error"], flush=True)
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                print("管理服务已启动，旧版本与缓存清理未完成（%s）；可在空闲时重试。" % type(exc).__name__, flush=True)
    print("NEH管理服务已就绪：" + url)
    if args.host in {"0.0.0.0", "::"}:
        addresses = device_urls(args.port)
        for address in addresses:
            print("设备网络地址：" + address)
        if not addresses:
            print("未检测到外部网络地址；连接设备网络后使用该设备地址和端口%s访问。" % args.port)
    print("命令行入口：" + str(binary))
    if args.configure:
        command = cli_command(home, python, "apply", "--file", str(args.configure.resolve()), "--wait")
        if args.catalog and not args.catalog.startswith(("https://", "http://")):
            command.extend(["--catalog", str(Path(args.catalog).resolve())])
        subprocess.run(command, cwd=str(manager / "current"), env=python_environment(), check=True)
    elif args.cli:
        subprocess.run(cli_command(home, python, "wizard"), cwd=str(manager / "current"), env=python_environment(), check=True)
    elif not args.no_browser and not os.environ.get("SSH_CONNECTION") and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        webbrowser.open(url)
    return 0


def parser():
    result = argparse.ArgumentParser(description="NEH端侧安装：一条命令准备网页和命令行管理入口")
    result.add_argument("--home", type=Path, default=Path.home() / ".local/share/neh")
    result.add_argument("--repository", default=DEFAULT_REPOSITORY, help="源码仓库HTTPS地址")
    result.add_argument("--branch", default=DEFAULT_BRANCH, help="跟踪的源码分支")
    result.add_argument("--revision", help="固定完整源码提交；省略时查询分支当前提交")
    result.add_argument("--allow-local-source", action="store_true", help=argparse.SUPPRESS)
    result.add_argument("--catalog", help="显式兼容旧版本目录；默认直接从Git源码安装")
    result.add_argument("--manager-package", type=Path, help="使用本地管理包")
    result.add_argument("--sha256", help="本地管理包预期摘要")
    result.add_argument("--token-file", type=Path, help="私有下载凭据文件，权限0600")
    result.add_argument("--host", default="0.0.0.0")
    result.add_argument("--port", type=int, help="管理端口；默认复用既有端口或自动选择空闲端口")
    result.add_argument("--channel", choices=("stable", "preview"), default="stable")
    result.add_argument("--service", choices=("system", "user", "none"), default="system",
                        help="默认安装普通用户运行的系统服务；none仅启动本次后台进程")
    result.add_argument("--cli", action="store_true", help="继续进入纯命令行安装向导")
    result.add_argument("--configure", type=Path, help="读取安装配置JSON并等待结果，适合预安装")
    result.add_argument("--no-browser", action="store_true")
    result.add_argument("--provision-system", action="store_true", help=argparse.SUPPRESS)
    result.add_argument("--expected-instance", help=argparse.SUPPRESS)
    result.add_argument("--expected-version", help=argparse.SUPPRESS)
    result.add_argument("--new-install", action="store_true", help=argparse.SUPPRESS)
    result.add_argument("--user", help=argparse.SUPPRESS)
    result.add_argument("--python", default=sys.executable, help=argparse.SUPPRESS)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.port is not None and not 1 <= args.port <= 65535:
            raise BootstrapError("端口必须在1至65535之间")
        if args.cli and args.configure:
            raise BootstrapError("交互向导与无人值守配置不能同时指定")
        return provision_system(args) if args.provision_system else install(args)
    except (BootstrapError, OSError, ValueError, tarfile.TarError, subprocess.SubprocessError) as exc:
        print("安装未完成：" + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
