"""发布目录、目标环境与严格归档校验；不执行清单传入的命令。"""
from __future__ import annotations

import ipaddress
import urllib.error
import urllib.parse
import urllib.request

from .common import ManagerError


def validate_url(url, allow_file=False):
    parsed = urllib.parse.urlsplit(str(url))
    if allow_file and parsed.scheme in ("", "file"):
        if parsed.netloc not in ("", "localhost"):
            raise ManagerError("invalid_url", "本地文件必须位于本机。")
        return parsed
    if parsed.username or parsed.password or parsed.fragment:
        raise ManagerError("invalid_url", "发布地址不允许内嵌密码或片段。")
    if parsed.scheme == "https" and parsed.hostname:
        return parsed
    if parsed.scheme == "http":
        try:
            loopback = ipaddress.ip_address(parsed.hostname).is_loopback
        except ValueError:
            loopback = parsed.hostname == "localhost"
        if loopback:
            return parsed
    raise ManagerError("invalid_url", "远端发布源必须使用HTTPS；HTTP仅允许本机回环验收。")


class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, msg, headers, newurl):
        validate_url(newurl)
        result = super().redirect_request(request, fp, code, msg, headers, newurl)
        old, new = urllib.parse.urlsplit(request.full_url), urllib.parse.urlsplit(newurl)
        if (old.scheme, old.hostname, old.port) != (new.scheme, new.hostname, new.port):
            result.remove_header("Authorization")
        return result


def open_source(url, *, allow_file=False):
    parsed = validate_url(url, allow_file=allow_file)
    if parsed.scheme in ("", "file"):
        return open(urllib.request.url2pathname(parsed.path), "rb")
    headers = {"User-Agent": "NEH-Device-Manager/1", "Accept": "application/octet-stream"}
    request = urllib.request.Request(url, headers=headers)
    try:
        return urllib.request.build_opener(urllib.request.ProxyHandler({}) if parsed.scheme == "http" else urllib.request.ProxyHandler(), SafeRedirect()).open(request, timeout=30)
    except (OSError, urllib.error.URLError) as exc:
        raise ManagerError("download_failed", "无法读取公开发布源，请检查网络和地址。", 502) from exc
