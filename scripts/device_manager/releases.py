"""发布目录、目标环境与严格归档校验；不执行清单传入的命令。"""
from __future__ import annotations

import hashlib
import ipaddress
import json
import os
import platform
import re
import shutil
import sys
import tarfile
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path, PurePosixPath

from .common import ManagerError, digest_file, identifier

MAX_DOWNLOAD = 32 * 1024 ** 3
MAX_EXTRACTED = 40 * 1024 ** 3
MAX_CATALOG = 8 * 1024 ** 2
MAX_MANIFEST = 16 * 1024 ** 2


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


def open_source(url, *, allow_file=False, token="", token_origin=""):
    parsed = validate_url(url, allow_file=allow_file)
    if parsed.scheme in ("", "file"):
        return open(urllib.request.url2pathname(parsed.path), "rb")
    target = urllib.parse.urlsplit(token_origin)
    trusted = token and (parsed.scheme, parsed.hostname, parsed.port) == (target.scheme, target.hostname, target.port)
    if token and not trusted and target.hostname == "github.com" and parsed.scheme == "https" and parsed.netloc == "api.github.com":
        parts = target.path.strip("/").split("/")
        trusted = len(parts) >= 2 and parsed.path.startswith("/repos/{}/{}/releases/assets/".format(parts[0], parts[1]))
    if trusted and parsed.hostname == "github.com":
        from scripts.neh_bootstrap import github_asset_url
        try:
            url = github_asset_url(url, token)
        except Exception as exc:
            raise ManagerError("download_failed", "无法读取私有GitHub发布资产。", 502) from exc
    headers = {"User-Agent": "NEH-Device-Manager/1", "Accept": "application/octet-stream"}
    if trusted:
        headers["Authorization"] = "Bearer " + token
    request = urllib.request.Request(url, headers=headers)
    try:
        return urllib.request.build_opener(urllib.request.ProxyHandler({}) if parsed.scheme == "http" else urllib.request.ProxyHandler(), SafeRedirect()).open(request, timeout=30)
    except (OSError, urllib.error.URLError) as exc:
        raise ManagerError("download_failed", "无法读取发布源，请检查网络、地址和访问凭据。", 502) from exc


def load_catalog(url, **kwargs):
    with open_source(url, **kwargs) as response:
        raw = response.read(MAX_CATALOG + 1)
    if len(raw) > MAX_CATALOG:
        raise ManagerError("catalog_invalid", "版本目录超过大小限制。")
    try:
        catalog = json.loads(raw)
        if catalog.get("schema_version") != 1 or catalog.get("product") != "neh" or not isinstance(catalog.get("releases"), list):
            raise ValueError()
        if len(catalog["releases"]) > 1000:
            raise ValueError()
        for release in catalog["releases"]:
            identifier(release["version"])
            if release.get("channel") not in ("stable", "preview") or not isinstance(release.get("artifacts"), list):
                raise ValueError()
            for artifact in release["artifacts"]:
                if artifact.get("kind") != "runtime":
                    continue
                if not re.fullmatch(r"[0-9a-fA-F]{64}", artifact.get("sha256", "")):
                    raise ValueError()
                if not isinstance(artifact.get("size"), int) or not 0 < artifact["size"] <= MAX_DOWNLOAD:
                    raise ValueError()
                for key in ("dependency_bytes", "model_bytes"):
                    if key in artifact and (type(artifact[key]) is not int or artifact[key] < 0):
                        raise ValueError()
                if "parts" in artifact:
                    raise ValueError()
                artifact["url"] = urllib.parse.urljoin(url if ":" in url else Path(url).resolve().as_uri(), artifact["url"])
                validate_url(artifact["url"], allow_file=kwargs.get("allow_file", False))
        return catalog
    except (ValueError, KeyError, TypeError, AttributeError) as exc:
        raise ManagerError("catalog_invalid", "版本目录结构或摘要无效。") from exc


def version_tuple(value):
    try:
        return tuple(int(part) for part in str(value).split("."))
    except ValueError:
        return ()


def platform_issues(target, hardware):
    issues = []
    if target.get("system") != "linux" or platform.system() != "Linux":
        issues.append("仅支持Linux目标")
    architecture = {"arm64": "aarch64", "amd64": "x86_64"}.get(target.get("architecture"), target.get("architecture"))
    if architecture != hardware.get("architecture"):
        issues.append("处理器架构不匹配")
    if hardware.get("os_id") not in target.get("os_ids", []):
        issues.append("发行版不匹配")
    if not target.get("os_versions") or hardware.get("version_id") not in target["os_versions"]:
        issues.append("发行版版本未经此包验证")
    libc = target.get("libc", {})
    host_name, host_version = platform.libc_ver()
    if not libc.get("minimum") or libc.get("name") != host_name or version_tuple(host_version) < version_tuple(libc.get("minimum")):
        issues.append("系统C库版本不兼容或缺少声明")
    desired = str(target.get("python", ""))
    if not re.fullmatch(r"3\.(8|9|10|11|12)", desired):
        issues.append("发布包Python版本不在支持范围")
    elif desired != "{}.{}".format(*sys.version_info[:2]) and not shutil.which("python" + desired):
        issues.append("缺少发布包需要的Python " + desired)
    return issues


def download(artifact, destination, *, progress, cancelled, **kwargs):
    size = artifact["size"]
    temporary = destination.with_suffix(".partial")
    received, digest = 0, hashlib.sha256()
    try:
        with open_source(artifact["url"], **kwargs) as response, open(temporary, "wb") as output:
            while True:
                cancelled()
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                received += len(chunk)
                if received > size or received > MAX_DOWNLOAD:
                    raise ManagerError("size_mismatch", "下载文件超过清单声明大小。")
                digest.update(chunk)
                output.write(chunk)
                progress(received, size)
            output.flush()
            os.fsync(output.fileno())
        if received != size or digest.hexdigest() != artifact["sha256"].lower():
            raise ManagerError("digest_mismatch", "下载文件大小或SHA256摘要不符，已拒绝安装。")
        os.replace(str(temporary), str(destination))
    finally:
        temporary.unlink(missing_ok=True)


def safe_member_name(name):
    path = PurePosixPath(name)
    if not name or "\\" in name or name.startswith("/") or any(part in ("", ".", "..") for part in name.split("/")):
        raise ManagerError("archive_unsafe", "发布包包含不安全路径。")
    if path.parts[0] not in ("manifest.json", "source", "payload", "requirements.txt"):
        raise ManagerError("archive_unsafe", "发布包包含未约定目录。")
    if path.suffix.lower() in (".whl", ".pt", ".pth", ".onnx", ".safetensors"):
        raise ManagerError("archive_unsafe", "业务包不得携带第三方依赖或模型文件。")
    return path


def unpack(archive, destination, expected, *, cancelled=lambda: None):
    """先核对完整目录与长度，再逐文件写入新建候选目录。"""
    try:
        with tarfile.open(archive, "r:gz") as package:
            members, names, total = [], set(), 0
            for member in package:
                cancelled()
                name = member.name.rstrip("/") if member.isdir() else member.name
                safe_member_name(name)
                if name in names or not (member.isfile() or member.isdir()):
                    raise ManagerError("archive_unsafe", "发布包包含重复路径、链接或特殊文件。")
                names.add(name)
                total += member.size
                if total > MAX_EXTRACTED or len(names) > 100000:
                    raise ManagerError("archive_unsafe", "发布包超过解压限制。")
                members.append(member)
            manifests = [entry for entry in members if entry.name == "manifest.json" and entry.isfile()]
            if len(manifests) != 1 or manifests[0].size > MAX_MANIFEST:
                raise ManagerError("manifest_invalid", "缺少有效发布清单。")
            manifest = json.load(package.extractfile(manifests[0]))
            if manifest.get("schema_version") != 2 or manifest.get("product") != "neh" or manifest.get("data_schema") != 1:
                raise ManagerError("manifest_invalid", "发布清单或数据格式不兼容，保留现有版本。")
            for key in ("version", "revision", "platform", "robot", "modules"):
                if key not in manifest or manifest[key] != expected[key]:
                    raise ManagerError("manifest_mismatch", "发布清单与固定计划不一致：" + key)
            declared = {}
            for entry in manifest["files"]:
                name = str(safe_member_name(entry["path"]))
                if name == "manifest.json" or name in declared or not isinstance(entry["size"], int) or entry["size"] < 0:
                    raise ValueError()
                if not re.fullmatch(r"[0-9a-fA-F]{64}", entry["sha256"]):
                    raise ValueError()
                declared[name] = entry
            actual = {entry.name for entry in members if entry.isfile()} - {"manifest.json"}
            if actual != set(declared):
                raise ManagerError("manifest_mismatch", "发布包文件与清单不一致。")
            if shutil.disk_usage(destination.parent).free < total * 2 + 256 * 1024 ** 2:
                raise ManagerError("disk_space", "磁盘空间不足以解压并建立候选环境。")
            destination.mkdir(mode=0o700)
            for member in members:
                cancelled()
                target = destination / member.name
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                    continue
                if member.name != "manifest.json" and member.size != declared[member.name]["size"]:
                    raise ManagerError("manifest_mismatch", "文件大小与清单不一致。")
                target.parent.mkdir(parents=True, exist_ok=True)
                digest = hashlib.sha256()
                with package.extractfile(member) as source, open(target, "xb") as output:
                    while True:
                        chunk = source.read(1024 * 1024)
                        if not chunk:
                            break
                        cancelled()
                        output.write(chunk)
                        digest.update(chunk)
                target.chmod(0o755 if member.mode & 0o111 else 0o644)
                if member.name != "manifest.json" and digest.hexdigest() != declared[member.name]["sha256"].lower():
                    raise ManagerError("digest_mismatch", "发布包文件摘要不符：" + member.name)
            return manifest
    except (tarfile.TarError, OSError, ValueError, KeyError, TypeError) as exc:
        if destination.exists():
            shutil.rmtree(destination)
        raise ManagerError("archive_invalid", "发布归档无法完整校验和解包。") from exc
    except BaseException:
        if destination.exists():
            shutil.rmtree(destination)
        raise


def github_catalog_url(url, channel, token="", token_origin=""):
    """固定GitHub发布源按通道查询正式API；通用目录仍由目录自身管理通道。"""
    parsed = urllib.parse.urlsplit(url)
    parts = parsed.path.strip("/").split("/")
    if parsed.hostname != "github.com" or len(parts) != 6 or parts[2:] != ["releases", "latest", "download", "catalog.json"]:
        return url, False
    source = urllib.parse.urlsplit(token_origin)
    trusted_token = token if (parsed.scheme, parsed.hostname, parsed.port) == (source.scheme, source.hostname, source.port) else ""
    endpoint = "https://api.github.com/repos/{}/{}/releases?per_page=100".format(parts[0], parts[1])
    with open_source(endpoint, token=trusted_token, token_origin="https://api.github.com") as response:
        raw = response.read(MAX_CATALOG + 1)
    if len(raw) > MAX_CATALOG:
        raise ManagerError("catalog_invalid", "GitHub版本目录响应超过限制。")
    try:
        releases = json.loads(raw)
        for release in releases:
            if release.get("draft") or (channel == "stable" and release.get("prerelease")):
                continue
            for asset in release.get("assets", []):
                if asset.get("name") == "catalog.json":
                    candidate = urllib.parse.urlsplit(asset.get("url", ""))
                    expected = "/repos/{}/{}/releases/assets/".format(parts[0], parts[1])
                    if candidate.scheme != "https" or candidate.netloc != "api.github.com" or not candidate.path.startswith(expected):
                        raise ManagerError("catalog_invalid", "GitHub返回了不受信任的目录资产地址。")
                    return candidate.geturl(), True
    except (ValueError, KeyError, TypeError, AttributeError) as exc:
        raise ManagerError("catalog_invalid", "GitHub版本目录响应无效。") from exc
    raise ManagerError("catalog_missing", "当前通道没有已发布的catalog.json附件。", 404)
