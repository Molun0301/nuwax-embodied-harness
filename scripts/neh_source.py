"""端侧固定源码修订、Git增量获取与稳定构建工作区；只依赖标准库。"""
from __future__ import annotations

from contextlib import contextmanager
import base64
import fcntl
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import signal
import subprocess
import time
from urllib.parse import unquote, urlsplit


DEFAULT_REPOSITORY = "https://github.com/Molun0301/nuwax-embodied-harness.git"
DEFAULT_BRANCH = "main"
_EXCLUDED_DIRS = {".git", ".gitnexus", ".config", ".ssh", ".venv", "venv", "__pycache__", ".pytest_cache", ".mypy_cache", ".ruff_cache", "node_modules", "target", "build", "dist", "result", "runtime_data", "tests", "test-results", "configs", ".idea", ".vscode"}


class SourceError(RuntimeError):
    """源码查询、获取或缓存所有权不符合实际条件。"""


def source_version(revision):
    if not re.fullmatch(r"[0-9a-f]{40}", str(revision)):
        raise SourceError("源码版本必须是完整Git提交")
    return "git-" + revision[:12]


def validate_repository(repository, *, allow_local=False):
    value = str(repository).strip()
    parsed = urlsplit(value)
    if allow_local and parsed.scheme in ("", "file"):
        if parsed.netloc not in ("", "localhost"):
            raise SourceError("本地源码地址不能指向其它主机")
        path = Path(unquote(parsed.path) if parsed.scheme == "file" else value).expanduser().resolve()
        if not path.is_dir():
            raise SourceError("本地源码仓库不存在")
        return path.as_uri()
    if (parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password
            or parsed.query or parsed.fragment or not parsed.path.strip("/")
            or any(ord(char) < 32 for char in value)):
        raise SourceError("源码仓库必须使用不含凭据的HTTPS地址")
    return value.rstrip("/")


def source_allowed(relative):
    """源码导出和历史管理包组装共用同一排除规则。"""
    path = PurePosixPath(relative)
    if (not path.parts or path.is_absolute() or path.as_posix() != relative
            or any(part in (".", "..") for part in path.parts) or "\\" in relative):
        raise SourceError("源码路径必须位于仓库内")
    if any(part in _EXCLUDED_DIRS for part in path.parts):
        return False
    if relative.startswith(("docs/开发文档/", ".github/")) or path.name == "AGENTS.md":
        return False
    name = path.name.lower()
    if name.endswith((".whl", ".download", ".deb", ".rpm", ".pt", ".pth", ".onnx", ".safetensors", ".gguf", ".ckpt", ".h5", ".hdf5", ".engine", ".trt", ".bin", ".dll", ".dylib", ".a", ".o")) or re.search(r"\.so(?:\.\d+)*$", name):
        return False
    if name.endswith((".pyc", ".pyo", ".pem", ".key", ".p12", ".pfx")) or name in ("credentials.json", "token.json", ".netrc", ".git-credentials", ".pypirc", ".npmrc", "pip.conf", "id_rsa", "id_ed25519"):
        return False
    return not ((name.startswith(".env") or name.endswith(".env")) and not name.endswith((".example", ".template")))


def _environment(repository, token, token_origin):
    environment = {key: value for key, value in os.environ.items()
                   if not key.startswith("GIT_") and key not in ("GH_TOKEN", "GITHUB_TOKEN", "GH_ENTERPRISE_TOKEN", "GITHUB_ENTERPRISE_TOKEN")}
    environment.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_SYSTEM=os.devnull,
                       GIT_CONFIG_GLOBAL=os.devnull, GIT_TERMINAL_PROMPT="0", GIT_LFS_SKIP_SMUDGE="1",
                       GIT_ASKPASS="/bin/false", SSH_ASKPASS="/bin/false", NEH_SOURCE_REPOSITORY=repository)
    source, origin = urlsplit(repository), urlsplit(token_origin)
    # Git2.25不支持COUNT/KEY/VALUE；沿其自身-c实现使用带引号的PARAMETERS环境。
    # GLOBAL=/dev/null只对新Git生效；旧Git仍读取全局文件，受控项由_git显式覆盖。
    parameters = ["http.extraHeader=", "http.sslVerify=true", "http.followRedirects=false",
                  "protocol.allow=never", "protocol.https.allow=always",
                  "protocol.file.allow=" + ("always" if source.scheme == "file" else "never")]
    source_path = source.path.rstrip("/")
    if source_path.endswith(".git"):
        source_path = source_path[:-4]
    origin_path = origin.path.rstrip("/")
    if origin_path.endswith(".git"):
        origin_path = origin_path[:-4]
    trusted = (token and source.scheme == origin.scheme == "https"
               and source.netloc == origin.netloc
               and origin_path == source_path)
    if trusted:
        authorization = base64.b64encode(("x-access-token:" + token).encode("utf-8")).decode("ascii")
        parameters.extend(["http." + repository + ".extraHeader=",
                           "http." + repository + ".extraHeader=Authorization: Basic " + authorization])
    # 与Git2.25的sq_quote_buf一致；凭据始终不进入进程参数或仓库配置文件。
    environment["GIT_CONFIG_PARAMETERS"] = " ".join("'" + value.replace("'", "'\\''").replace("!", "'\\!'") + "'" for value in parameters)
    return environment


def _failure_message(stderr, environment):
    """Git诊断可能引用环境配置；隐藏本次授权及其编码后再交给上层。"""
    message = stderr.strip()[-2000:]
    parameters = (environment or {}).get("GIT_CONFIG_PARAMETERS", "")
    for encoded in re.findall(r"Authorization: Basic ([A-Za-z0-9+/=]+)", parameters):
        message = message.replace(encoded, "[已隐藏]")
        try:
            token = base64.b64decode(encoded).decode("utf-8").partition(":")[2]
        except (ValueError, UnicodeError):
            continue
        if token:
            message = message.replace(token, "[已隐藏]")
    return "Git操作失败：" + message


def _execute(argv, *, environment=None, run=None, cancelled=None, timeout=900):
    if cancelled:
        cancelled()
    if run:
        return run(argv, environment=environment, timeout=timeout, cancelled=cancelled or (lambda: None))
    try:
        process = subprocess.Popen(argv, env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, start_new_session=True)
    except OSError as exc:
        raise SourceError("无法启动Git，请确认已安装git") from exc
    started = time.monotonic()
    try:
        while True:
            if cancelled:
                cancelled()
            if time.monotonic() - started > timeout:
                raise SourceError("源码获取超时，请检查网络后重试")
            try:
                stdout, stderr = process.communicate(timeout=0.2)
                break
            except subprocess.TimeoutExpired:
                continue
        if process.returncode:
            raise SourceError(_failure_message(stderr, environment))
        return stdout
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            except ProcessLookupError:
                process.wait()


def _git(args, *, root=None, environment=None, run=None, cancelled=None):
    # 不重设HOME或改写用户配置；这些选项在Git2.25已受支持，压过全局与仓库设置。
    command = ["git"]
    for option in ("credential.helper=", "core.hooksPath=" + os.devnull, "core.fsmonitor=false",
                   "core.attributesFile=" + os.devnull, "core.autocrlf=false", "core.quotePath=false",
                   "init.templateDir=", "gc.auto=0", "color.ui=false", "diff.external="):
        command += ["-c", option]
    if root:
        command += ["-C", str(root)]
    repository = (environment or {}).get("NEH_SOURCE_REPOSITORY", "")
    if args and args[0] in ("fetch", "ls-remote") and "--get-url" not in args and repository:
        requested = "origin" if args[0] == "fetch" else repository
        resolved = _execute(command + ["ls-remote", "--get-url", requested], environment=environment,
                            run=run, cancelled=cancelled).strip()
        if resolved.rstrip("/") != repository.rstrip("/"):
            raise SourceError("Git来源被本机配置重写，请核对URL重写规则后重试")
    return _execute(command + list(args), environment=environment, run=run, cancelled=cancelled)


def _branch(branch):
    value = str(branch)
    if (not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]{0,254}", value)
            or ".." in value or "//" in value or value.endswith(("/", ".", ".lock"))):
        raise SourceError("源码分支名称无效")
    return value


def remote_revision(repository=DEFAULT_REPOSITORY, branch=DEFAULT_BRANCH, *, token="", token_origin="", run=None, allow_local=False):
    repository, branch = validate_repository(repository, allow_local=allow_local), _branch(branch)
    reference = "refs/heads/" + branch
    output = _git(["ls-remote", "--exit-code", "--refs", repository, reference],
                  environment=_environment(repository, token, token_origin), run=run)
    matches = [line.split()[0] for line in output.splitlines() if line.split()[1:] == [reference]]
    if len(matches) != 1:
        raise SourceError("源码仓库未返回唯一的目标分支")
    return {"repository": repository, "branch": branch, "revision": matches[0], "version": source_version(matches[0])}


@contextmanager
def source_checkout(home, repository=DEFAULT_REPOSITORY, branch=DEFAULT_BRANCH, revision=None, *, token="", token_origin="", run=None, progress=None, cancelled=None, allow_local=False):
    repository, branch = validate_repository(repository, allow_local=allow_local), _branch(branch)
    area = Path(home).expanduser().resolve() / "cache/source"
    area.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (area / "source.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise SourceError("另一个安装任务正在使用源码和构建缓存") from exc
        environment = _environment(repository, token, token_origin)
        root = area / "checkout"
        if root.is_symlink() or (root.exists() and not root.is_dir()):
            raise SourceError("源码缓存目录类型异常")
        if not (root / ".git").is_dir():
            if root.exists() and any(root.iterdir()):
                raise SourceError("源码缓存含有未登记文件，不能覆盖")
            _git(["init", "--quiet", str(root)], environment=environment, run=run, cancelled=cancelled)
        def git(args):
            return _git(args, root=root, environment=environment, run=run, cancelled=cancelled)
        if git(["status", "--porcelain", "--untracked-files=normal"]).strip():
            raise SourceError("源码构建工作区有未提交修改，未覆盖这些文件")
        git(["config", "remote.origin.url", repository])
        git(["config", "remote.origin.fetch", "+refs/heads/*:refs/remotes/origin/*"])
        previous = git(["rev-parse", "--verify", "--quiet", "HEAD"]).strip() if git(["branch", "--all"]).strip() else None
        if revision is None:
            revision = remote_revision(repository, branch, token=token, token_origin=token_origin,
                                       run=run, allow_local=allow_local)["revision"]
        version = source_version(revision)
        if progress:
            progress("增量获取源码提交 " + revision[:12])
        if previous != revision:
            git(["fetch", "--no-tags", "--recurse-submodules=no", "--depth=1", "origin", revision])
            git(["checkout", "--detach", "--quiet", revision])
        actual = git(["rev-parse", "HEAD"]).strip()
        if actual != revision:
            raise SourceError("检出的源码提交与计划不一致")
        changed = git(["diff", "--name-only", previous, revision, "--"]).splitlines() if previous else git(["ls-files"]).splitlines()
        build_cache, build_prefix = area / "build", area / "prefix"
        build_cache.mkdir(exist_ok=True, mode=0o700)
        build_prefix.mkdir(exist_ok=True, mode=0o700)
        record = {"repository": repository, "branch": branch, "revision": revision, "version": version}
        temporary = area / "revision.json.new"
        temporary.write_text(json.dumps(record, ensure_ascii=False) + "\n", encoding="utf-8")
        os.replace(str(temporary), str(area / "revision.json"))
        yield dict(record, root=root, previous_revision=previous, changed_files=changed,
                   build_cache=build_cache, build_prefix=build_prefix)


def export_source(root, revision, destination):
    root, destination = Path(root).resolve(), Path(destination)
    source_version(revision)
    environment = _environment("", "", "")
    if _git(["rev-parse", "HEAD"], root=root, environment=environment).strip() != revision:
        raise SourceError("源码导出修订与构建工作区不一致")
    if _git(["status", "--porcelain", "--untracked-files=normal"], root=root, environment=environment).strip():
        raise SourceError("不能从有未提交修改的工作区导出运行源码")
    if destination.exists() or destination.is_symlink():
        raise SourceError("候选源码目录已经存在，不能覆盖")
    destination.mkdir(parents=True, mode=0o700)
    for relative in _git(["ls-files", "-z"], root=root, environment=environment).split("\0"):
        if not relative or not source_allowed(relative):
            continue
        source, target = root / relative, destination / relative
        if source.is_symlink() or not source.is_file():
            raise SourceError("源码包含链接或非普通文件：" + relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        target.chmod(0o755 if source.stat().st_mode & 0o111 else 0o644)
    return {"revision": revision, "version": source_version(revision)}
