"""独立设备网页、同源接口和透明业务代理，全部使用标准库。"""
from __future__ import annotations

import http.client
import json
import mimetypes
import os
import secrets
import select
import socket
import threading
import time
import urllib.parse
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import MANAGER_DEVELOPMENT, MANAGER_VERSION
from .auth import Authentication
from .common import ManagerError, atomic_json, process_identity, read_json

MAX_BODY = 2 * 1024 * 1024
HOP_HEADERS = {"connection", "keep-alive", "proxy-authenticate", "proxy-authorization", "te", "trailer", "transfer-encoding", "upgrade"}


class LimitedReader:
    """只消费当前HTTP请求体，避免把下一个请求交给上游或整体放入内存。"""
    def __init__(self, source, length):
        self.source, self.remaining = source, length

    def read(self, size=-1):
        if self.remaining == 0:
            return b""
        data = self.source.read(min(self.remaining, 64 * 1024 if size < 0 else size))
        if not data:
            raise ConnectionResetError("请求体提前结束")
        self.remaining -= len(data)
        return data


class ManagerServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, engine, static_root=None):
        self.address_family = socket.AF_INET6 if ":" in address[0] else socket.AF_INET
        self.engine = engine
        self.authentication = Authentication(engine)
        self.static_root = Path(static_root or Path(__file__).parent / "static").resolve()
        self.sessions, self.attempts = {}, {}
        self.session_lock = threading.Lock()
        self.allowed_hosts = {"localhost", "127.0.0.1", "::1", socket.gethostname().lower()}
        try:
            self.allowed_hosts.update(entry[4][0] for entry in socket.getaddrinfo(socket.gethostname(), None))
        except OSError:
            pass
        # 内核路由查询只枚举本机地址，不根据请求提供的主机名进行域名解析。
        import subprocess
        try:
            output = subprocess.run(["ip", "-json", "address", "show"], capture_output=True, text=True, timeout=3)
            self.allowed_hosts.update(info["local"] for interface in json.loads(output.stdout) for info in interface.get("addr_info", []))
        except (OSError, ValueError, KeyError, subprocess.SubprocessError):
            pass
        if address[0] not in ("0.0.0.0", "::"):
            self.allowed_hosts.add(address[0].lower())
        self.slots = threading.BoundedSemaphore(64)
        self.last_recovery = 0
        self.recovery_thread = None
        super().__init__(address, Handler)

    def _recover_background(self):
        try:
            self.engine.recover_if_needed()
        except ManagerError:
            # 状态接口会返回损坏记录的真实错误；监听线程继续提供管理入口。
            pass

    def service_actions(self):
        if time.monotonic() - self.last_recovery >= 5 and not (self.recovery_thread and self.recovery_thread.is_alive()):
            self.last_recovery = time.monotonic()
            self.recovery_thread = threading.Thread(target=self._recover_background, daemon=True)
            self.recovery_thread.start()
        from .runtime import clean_environment
        request_path = self.engine.home / "config/restart-manager.json"
        request = read_json(request_path, {})
        if not request or request.get("instance_id") != self.engine.instance_id:
            return
        job = self.engine.job(request.get("job_id", ""))
        if job["status"] != "succeeded":
            return
        target = (self.engine.home / "manager/current").resolve()
        if target.parent != self.engine.home / "manager/versions":
            return
        manifest = read_json(target / "manager-manifest.json", {})
        if manifest.get("version") != request.get("version"):
            return
        request_path.unlink()
        environment = clean_environment()
        environment["PYTHONPATH"] = str(target)
        import sys
        arguments = [sys.executable, "-m", "scripts.device_manager", "--home", str(self.engine.home), "serve", "--host", self.server_address[0], "--port", str(self.server_port)]
        self.server_close()
        os.chdir(str(target))
        os.execve(sys.executable, arguments, environment)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            request.close()
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "NEH"
    sys_version = ""

    def setup(self):
        super().setup()
        self.connection.settimeout(30)

    def log_message(self, format, *args):
        # 默认请求日志可能带凭据查询参数；管理器只持久化具名任务日志。
        pass

    def _same_origin(self, required=False):
        host = self.headers.get("Host", "")
        parsed = urllib.parse.urlsplit("http://" + host)
        if not parsed.hostname or parsed.hostname.lower() not in self.server.allowed_hosts or parsed.username or parsed.password:
            raise ManagerError("host_denied", "请求主机不是本设备的受支持访问地址。", 403)
        origin = self.headers.get("Origin")
        if origin:
            incoming = urllib.parse.urlsplit(origin)
            if incoming.scheme not in ("http", "https") or incoming.netloc.lower() != host.lower() or incoming.path not in ("", "/"):
                raise ManagerError("origin_denied", "拒绝跨站请求。", 403)
        elif required:
            raise ManagerError("origin_required", "该请求必须携带同源Origin。", 403)
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            raise ManagerError("origin_denied", "拒绝跨站请求。", 403)

    def _session(self, create=False):
        cookie = SimpleCookie()
        try:
            cookie.load(self.headers.get("Cookie", ""))
        except Exception:
            pass
        token = cookie.get("neh_session")
        key = token.value if token else ""
        auth = self.server.authentication
        epoch = auth.epoch()
        with self.server.session_lock:
            now = time.time()
            expired = [sid for sid, state in self.server.sessions.items() if state["expires"] < now or state["epoch"] != epoch]
            for sid in expired:
                self.server.sessions.pop(sid, None)
            session = self.server.sessions.get(key)
            if session is None and create:
                if len(self.server.sessions) >= 512:
                    self.server.sessions.pop(next(iter(self.server.sessions)))
                key = secrets.token_urlsafe(32)
                session = {"authenticated": not auth.enabled(), "csrf_token": secrets.token_urlsafe(32), "expires": now + 86400, "epoch": auth.epoch()}
                self.server.sessions[key] = session
                self._new_cookie = key
            return key, session

    def _rotate_session(self, old_key):
        key = secrets.token_urlsafe(32)
        session = {"authenticated": True, "csrf_token": secrets.token_urlsafe(32), "expires": time.time() + 86400, "epoch": self.server.authentication.epoch()}
        with self.server.session_lock:
            self.server.sessions.pop(old_key, None)
            self.server.sessions[key] = session
        self._new_cookie = key
        return session

    def _authorize(self, mutation=False, public=False, business=False):
        self._same_origin()
        key, session = self._session(create=public)
        if not public and not (business and not self.server.authentication.enabled()) and (not session or (self.server.authentication.enabled() and not session["authenticated"])):
            raise ManagerError("authentication_required", "请先登录设备管理台。", 401)
        if mutation:
            self._same_origin(required=True)
            if not session or not secrets.compare_digest(self.headers.get("X-NEH-CSRF", ""), session["csrf_token"]):
                raise ManagerError("csrf_invalid", "请求校验失效，请刷新页面后重试。", 403)
        return key, session

    def _json_body(self):
        if self.headers.get("Transfer-Encoding"):
            raise ManagerError("invalid_body", "不接受分块管理请求。")
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise ManagerError("invalid_body", "请求长度无效。")
        if not 0 <= length <= MAX_BODY:
            raise ManagerError("request_too_large", "请求体过大。", 413)
        if length and self.headers.get_content_type() != "application/json":
            raise ManagerError("invalid_body", "管理请求必须使用JSON。", 415)
        try:
            value = json.loads(self.rfile.read(length) or b"{}")
        except (ValueError, UnicodeError):
            raise ManagerError("invalid_body", "JSON请求体无效。")
        if not isinstance(value, dict):
            raise ManagerError("invalid_body", "请求体必须是对象。")
        return value

    def _send(self, status, value, content_type="application/json; charset=utf-8"):
        data = json.dumps(value, ensure_ascii=False).encode() if not isinstance(value, bytes) else value
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "same-origin")
        self.send_header("X-Frame-Options", "SAMEORIGIN")
        if status >= 400:
            # 鉴权或长度检查可能先于正文读取；关闭连接，避免残留正文被当成下一条请求。
            self.close_connection = True
            self.send_header("Connection", "close")
        if getattr(self, "_new_cookie", None):
            self.send_header("Set-Cookie", "neh_session={}; HttpOnly; SameSite=Strict; Path=/; Max-Age=86400{}".format(self._new_cookie, "; Secure" if self.headers.get("Origin", "").startswith("https://") else ""))
            self._new_cookie = None
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    def do_GET(self):
        self._dispatch()

    def do_HEAD(self):
        self._dispatch()

    def do_POST(self):
        self._dispatch()

    def do_PUT(self):
        self._dispatch()

    def do_DELETE(self):
        self._dispatch()

    def _dispatch(self):
        try:
            self._same_origin()
            parsed = urllib.parse.urlsplit(self.path)
            path = parsed.path
            if path.startswith("/api/manager/"):
                self._manager(path[len("/api/manager/" ):], urllib.parse.parse_qs(parsed.query))
            elif path in ("/", "/index.html", "/app.css", "/app.js") or path.startswith("/manager-assets/"):
                if self.command not in ("GET", "HEAD"):
                    raise ManagerError("method_not_allowed", "不支持的请求方法。", 405)
                filename = "index.html" if path == "/" else path[len("/manager-assets/"): ] if path.startswith("/manager-assets/") else path[1:]
                target = (self.server.static_root / urllib.parse.unquote(filename)).resolve()
                if self.server.static_root not in target.parents or not target.is_file():
                    raise ManagerError("not_found", "页面资源不存在。", 404)
                self._send(200, target.read_bytes(), mimetypes.guess_type(str(target))[0] or "application/octet-stream")
            else:
                self._authorize(business=True)
                if not self.server.engine.runtime.has_owner():
                    raise ManagerError("gateway_unavailable", "本安装的业务拥有方未运行，请从管理台启动。", 503)
                if self.command not in ("GET", "HEAD"):
                    if self.headers.get("Origin"):
                        self._same_origin(required=True)
                    elif not self.path.startswith(("/mcp", "/api/")) or self.headers.get_content_type() != "application/json":
                        raise ManagerError("origin_required", "业务写请求需要同源浏览器或JSON机器客户端。", 403)
                if self.headers.get("Upgrade", "").lower() == "websocket":
                    self._same_origin(required=True)
                    self._websocket()
                else:
                    self._proxy()
        except ManagerError as exc:
            self._send(exc.status, {"error": exc.as_dict()})
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            self.close_connection = True
        except Exception:
            self._send(500, {"error": {"code": "internal_error", "message": "管理请求失败，请查看设备本地状态。"}})

    def _manager(self, path, query):
        engine, auth = self.server.engine, self.server.authentication
        mutation = self.command not in ("GET", "HEAD")
        public = path in ("session", "login")
        key, session = self._authorize(mutation=mutation, public=public)
        body = self._json_body() if mutation else {}
        if path == "session" and not mutation:
            result = {"manager_version": MANAGER_VERSION, "manager_development": MANAGER_DEVELOPMENT, "instance_id": engine.instance_id, "password_enabled": auth.enabled(), "authenticated": session["authenticated"], "csrf_token": session["csrf_token"]}
        elif path == "login" and self.command == "POST":
            now, address = time.time(), self.client_address[0]
            with self.server.session_lock:
                failures = [t for t in self.server.attempts.get(address, []) if t > now - 60]
                if len(failures) >= 5:
                    raise ManagerError("login_limited", "登录尝试过多，请稍后重试。", 429)
                self.server.attempts[address] = failures + [now]
            if not auth.verify(body.get("password", "")):
                raise ManagerError("wrong_password", "密码不正确。", 401)
            session = self._rotate_session(key)
            self.server.attempts.pop(address, None)
            result = {"authenticated": True, "csrf_token": session["csrf_token"]}
        elif path == "logout" and self.command == "POST":
            self.server.sessions.pop(key, None)
            result = {"authenticated": False}
        elif path == "status" and not mutation:
            result = engine.status()
        elif path == "hardware" and not mutation:
            result = engine.hardware()
        elif path == "migration-sources" and not mutation:
            result = engine.migration_sources()
        elif path == "modules" and not mutation:
            result = engine.modules()
        elif path == "releases" and not mutation:
            result = engine.releases(refresh=query.get("refresh") == ["1"])
        elif path == "plan" and self.command == "POST":
            if body.get("action") in ("install", "migrate") and not engine.current():
                configuration = body.get("configuration", {})
                if not isinstance(configuration, dict):
                    raise ManagerError("invalid_config", "预配置必须是对象。")
                if not configuration.get("NUWAX_GATEWAY_PUBLIC_BASE_URL"):
                    configuration = dict(configuration, NUWAX_GATEWAY_PUBLIC_BASE_URL=self.headers["Origin"].rstrip("/"))
                    body = dict(body, configuration=configuration)
            result = engine.plan(body)
        elif path == "jobs" and self.command == "POST":
            result = engine.submit(body.get("plan_id", ""))
        elif path == "jobs" and not mutation:
            engine.recover_if_needed()
            result = {"jobs": engine.jobs()}
        elif path.startswith("jobs/"):
            parts = path.split("/")
            if len(parts) == 2 and not mutation:
                engine.recover_if_needed()
                result = engine.job(parts[1])
            elif len(parts) == 3 and parts[2] == "cancel" and self.command == "POST":
                result = engine.cancel(parts[1])
            else:
                raise ManagerError("not_found", "任务接口不存在。", 404)
        elif path == "settings" and self.command == "PUT":
            result = engine.save_settings(body)
        elif path == "settings" and not mutation:
            result = engine.settings()
        elif path == "source-credentials" and self.command == "POST":
            token = body.get("token", "")
            if not isinstance(token, str) or len(token) > 4096 or "\n" in token:
                raise ManagerError("invalid_token", "发布源令牌格式无效。")
            engine.set_token(token)
            result = {"has_download_token": engine.settings()["has_download_token"]}
        elif path == "password" and self.command == "POST":
            result = auth.change(body.get("password"), body.get("current_password", ""))
            session = self._rotate_session(key)
            result["csrf_token"] = session["csrf_token"]
        elif path == "config" and self.command == "PUT":
            result = engine.save_config(body.get("values"), body.get("revision"))
        elif path == "config" and not mutation:
            result = engine.config()
        else:
            raise ManagerError("not_found", "管理接口不存在或方法不正确。", 404)
        self._send(200, result)

    def _upstream_headers(self):
        headers = {key: value for key, value in self.headers.items() if key.lower() not in HOP_HEADERS | {"host", "cookie", "x-neh-csrf", "x-forwarded-for", "x-forwarded-host", "x-forwarded-proto"}}
        headers["Host"] = self.headers["Host"]
        headers["X-Forwarded-Host"] = self.headers["Host"]
        headers["X-Forwarded-For"] = self.client_address[0]
        origin = self.headers.get("Origin", "")
        scheme = urllib.parse.urlsplit(origin).scheme if origin else "http"
        headers["X-Forwarded-Proto"] = scheme if scheme in ("http", "https") else "http"
        return headers

    def _proxy(self):
        if not self.path.startswith(("/console", "/static/", "/api/", "/streams", "/artifacts/", "/mcp", "/debug")):
            raise ManagerError("not_found", "页面不存在。", 404)
        if self.headers.get("Transfer-Encoding"):
            raise ManagerError("invalid_body", "不支持分块上传。", 400)
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            raise ManagerError("invalid_body", "请求长度无效。")
        if not 0 <= length <= 1024 * 1024 * 1024:
            raise ManagerError("request_too_large", "业务请求体过大。", 413)
        upstream = http.client.HTTPConnection("127.0.0.1", self.server.engine.runtime.port(), timeout=30)
        started = False
        try:
            upstream.request(self.command, self.path, body=LimitedReader(self.rfile, length) if length else None, headers=self._upstream_headers())
            response = upstream.getresponse()
            started = True
            self.send_response(response.status)
            for key, value in response.getheaders():
                if key.lower() not in HOP_HEADERS | {"set-cookie"}:
                    self.send_header(key, value)
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            if self.command != "HEAD":
                while True:
                    data = response.read1(64 * 1024)
                    if not data:
                        break
                    self.wfile.write(data)
                    self.wfile.flush()
        except OSError as exc:
            if started:
                self.close_connection = True
                return
            raise ManagerError("gateway_unavailable", "业务服务尚未启动或暂不可达，请在管理台查看任务。", 503) from exc
        finally:
            upstream.close()

    def _websocket(self):
        if not self.path.startswith(("/streams", "/api/", "/ws")) or self.command != "GET":
            raise ManagerError("not_found", "实时接口不存在。", 404)
        try:
            upstream = socket.create_connection(("127.0.0.1", self.server.engine.runtime.port()), timeout=10)
        except OSError as exc:
            raise ManagerError("gateway_unavailable", "业务实时服务不可达。", 503) from exc
        try:
            headers = self._upstream_headers()
            headers.update({"Host": self.headers["Host"], "Connection": "Upgrade", "Upgrade": "websocket"})
            request = "GET {} HTTP/1.1\r\n{}\r\n\r\n".format(self.path, "\r\n".join("{}: {}".format(k, v) for k, v in headers.items()))
            upstream.sendall(request.encode("latin-1"))
            self.connection.settimeout(None)
            upstream.settimeout(None)
            # 原始双向隧道保留业务实时协议；管理器不解码或重建业务帧。
            sockets = [self.connection, upstream]
            auth_checked = time.monotonic()
            while True:
                if time.monotonic() - auth_checked >= 5:
                    auth_checked = time.monotonic()
                    if self.server.authentication.enabled() and not self._session()[1]:
                        return
                ready, _, errors = select.select(sockets, [], sockets, 30)
                if errors:
                    break
                if not ready:
                    if self.server.authentication.enabled() and not self._session()[1]:
                        break
                    continue
                for source in ready:
                    data = source.recv(64 * 1024)
                    if not data:
                        return
                    target = upstream if source is self.connection else self.connection
                    target.settimeout(30)
                    target.sendall(data)
                    target.settimeout(None)
        finally:
            upstream.close()
            self.close_connection = True


def serve(engine, host="127.0.0.1", port=8765):
    try:
        engine.recover_if_needed()
        if engine.settings()["auto_start"] and engine.current() and not engine.active_job() and not engine.runtime.status()["running"]:
            plan = engine.plan({"action": "start"}, local_request=True)
            if not plan["blockers"]:
                engine.submit(plan["id"])
    except (ManagerError, ValueError, OSError) as exc:
        # 业务记录或配置异常不能阻止独立管理页启动；接口继续提供实际诊断。
        print("业务自动启动预检失败，管理入口继续运行：" + getattr(exc, "code", type(exc).__name__), flush=True)
    server = ManagerServer((host, port), engine)
    address = "[" + host + "]" if ":" in host else host
    public_url = "http://{}:{}".format(address, server.server_port) if host not in ("0.0.0.0", "::") else ""
    atomic_json(engine.home / "config/server.json", {"process": process_identity(os.getpid()), "host": host, "port": server.server_port, "public_url": public_url})
    print("NEH管理台：http://{}:{}".format(address, server.server_port), flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
