#!/usr/bin/env bash
# 固定同一源码提交准备引导程序；设备缓存源码后自行组装管理入口。
set -euo pipefail
umask 077

if [[ ${EUID} -eq 0 ]]; then
  echo '请以普通用户执行安装；系统服务会单独请求管理员权限。' >&2
  exit 1
fi
NEH_BOOTSTRAP_PACKAGES=()
command -v python3 >/dev/null 2>&1 || NEH_BOOTSTRAP_PACKAGES+=(python3)
command -v git >/dev/null 2>&1 || NEH_BOOTSTRAP_PACKAGES+=(git)
if (( ${#NEH_BOOTSTRAP_PACKAGES[@]} )); then
  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install --yes "${NEH_BOOTSTRAP_PACKAGES[@]}" ca-certificates
  elif command -v dnf >/dev/null 2>&1; then
    sudo dnf install --assumeyes "${NEH_BOOTSTRAP_PACKAGES[@]}" ca-certificates
  elif command -v yum >/dev/null 2>&1; then
    sudo yum install --assumeyes "${NEH_BOOTSTRAP_PACKAGES[@]}" ca-certificates
  elif command -v pacman >/dev/null 2>&1; then
    sudo pacman -S --needed --noconfirm python git ca-certificates
  elif command -v zypper >/dev/null 2>&1; then
    sudo zypper --non-interactive install "${NEH_BOOTSTRAP_PACKAGES[@]}" ca-certificates
  elif command -v apk >/dev/null 2>&1; then
    sudo apk add --no-cache "${NEH_BOOTSTRAP_PACKAGES[@]}" ca-certificates
  else
    echo '请先安装git及Python 3.8至3.12，再执行此入口。' >&2
    exit 1
  fi
fi

NEH_BOOTSTRAP_TEMP=$(mktemp -d -t neh-bootstrap.XXXXXXXX)
trap 'rm -rf -- "${NEH_BOOTSTRAP_TEMP}"' EXIT
export NEH_BOOTSTRAP_STAGE="$NEH_BOOTSTRAP_TEMP"
export NEH_BOOTSTRAP_REPOSITORY='https://github.com/Molun0301/nuwax-embodied-harness.git'
export NEH_BOOTSTRAP_BRANCH=main
export NEH_BOOTSTRAP_REVISION=''
export NEH_BOOTSTRAP_TOKEN_FILE=''
export NEH_BOOTSTRAP_ALLOW_LOCAL=0
NEH_BOOTSTRAP_ARGS=("$@")
for ((neh_i=0; neh_i < ${#NEH_BOOTSTRAP_ARGS[@]}; neh_i++)); do
  case ${NEH_BOOTSTRAP_ARGS[neh_i]} in
    --token-file) export NEH_BOOTSTRAP_TOKEN_FILE="${NEH_BOOTSTRAP_ARGS[neh_i+1]:-}" ;;
    --token-file=*) export NEH_BOOTSTRAP_TOKEN_FILE="${NEH_BOOTSTRAP_ARGS[neh_i]#--token-file=}" ;;
    --repository) export NEH_BOOTSTRAP_REPOSITORY="${NEH_BOOTSTRAP_ARGS[neh_i+1]:-}" ;;
    --repository=*) export NEH_BOOTSTRAP_REPOSITORY="${NEH_BOOTSTRAP_ARGS[neh_i]#--repository=}" ;;
    --branch) export NEH_BOOTSTRAP_BRANCH="${NEH_BOOTSTRAP_ARGS[neh_i+1]:-}" ;;
    --branch=*) export NEH_BOOTSTRAP_BRANCH="${NEH_BOOTSTRAP_ARGS[neh_i]#--branch=}" ;;
    --revision) export NEH_BOOTSTRAP_REVISION="${NEH_BOOTSTRAP_ARGS[neh_i+1]:-}" ;;
    --revision=*) export NEH_BOOTSTRAP_REVISION="${NEH_BOOTSTRAP_ARGS[neh_i]#--revision=}" ;;
    --allow-local-source) export NEH_BOOTSTRAP_ALLOW_LOCAL=1 ;;
  esac
 done
unset PYTHONHOME PYTHONPATH
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
python3 - <<'PY'
import errno, http.client, json, os, pathlib, re, shutil, socket, ssl, subprocess, sys, tempfile, time
import urllib.error, urllib.parse, urllib.request
if not (3, 8) <= sys.version_info[:2] < (3, 13):
    raise SystemExit("引导安装需要Python 3.8至3.12")
repository = os.environ["NEH_BOOTSTRAP_REPOSITORY"]
branch = os.environ["NEH_BOOTSTRAP_BRANCH"]
revision = os.environ["NEH_BOOTSTRAP_REVISION"]
stage = pathlib.Path(os.environ["NEH_BOOTSTRAP_STAGE"])
parsed = urllib.parse.urlsplit(repository)
if parsed.username or parsed.password or parsed.query or parsed.fragment or any(ord(c) < 32 for c in repository):
    raise SystemExit("源码地址不能包含凭据、查询参数或控制字符")
if revision and not re.fullmatch(r"[0-9a-f]{40,64}", revision):
    raise SystemExit("源码修订必须是完整提交标识")
environment = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
environment.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull, GIT_TERMINAL_PROMPT="0")
if subprocess.run(["git", "check-ref-format", "--branch", branch], env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode:
    raise SystemExit("源码分支名称无效")
token = ""
token_file = os.environ["NEH_BOOTSTRAP_TOKEN_FILE"]
if token_file:
    path = pathlib.Path(token_file).expanduser()
    if not path.is_file() or path.stat().st_mode & 0o077:
        raise SystemExit("下载凭据文件须设置为0600权限")
    token = path.read_text().strip()
    if not token or "\n" in token or len(token) > 4096:
        raise SystemExit("下载凭据文件无效")

class Redirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urllib.parse.urlsplit(newurl).scheme != "https":
            raise ValueError("拒绝不安全重定向")
        result = super().redirect_request(req, fp, code, msg, headers, newurl)
        if result and urllib.parse.urlsplit(req.full_url).netloc != urllib.parse.urlsplit(newurl).netloc:
            result.remove_header("Authorization")
        return result

opener = urllib.request.build_opener(Redirect())
MAX_DOWNLOAD = 2 * 1024 * 1024
DOWNLOAD_ATTEMPTS = 3
DOWNLOAD_TIMEOUT = 45
RETRY_HTTP = {408, 429, 500, 502, 503, 504}
# 私有入口已经用gh取得本脚本；直接复用相同传输及已有授权。
# 不要求匿名入口安装gh，也不依赖用户持久保存的gh登录配置。
gh = shutil.which("gh") if token else None

class DownloadError(Exception):
    def __init__(self, reason, retry=False):
        self.reason, self.retry = reason, retry

def http_failure(status):
    if status in (401, 403, 404):
        return DownloadError("HTTP %s；请核对仓库、分支及私有仓库访问权限" % status)
    return DownloadError("HTTP %s" % status, status in RETRY_HTTP)

def network_failure(exc):
    reason = exc.reason if isinstance(exc, urllib.error.URLError) else exc
    if isinstance(reason, ssl.SSLCertVerificationError):
        return DownloadError("TLS证书验证失败；请检查设备时间与CA证书，不要关闭证书验证")
    transient = isinstance(reason, (ssl.SSLEOFError, TimeoutError, ConnectionError,
                                   http.client.IncompleteRead, http.client.RemoteDisconnected))
    transient = transient or (isinstance(reason, OSError) and reason.errno in
                              (errno.ETIMEDOUT, errno.ECONNRESET, errno.ECONNABORTED,
                               errno.ECONNREFUSED, errno.ENETUNREACH, errno.EHOSTUNREACH))
    transient = transient or (isinstance(reason, socket.gaierror) and reason.errno == socket.EAI_AGAIN)
    # 只显示已知类别，底层异常可能包含代理凭据、令牌或响应内容。
    return DownloadError("TLS或网络连接中断" if transient else "TLS或网络请求失败", transient)

def gh_download(url, accept):
    if urllib.parse.urlsplit(url).netloc != "api.github.com" or not url.startswith("https://"):
        raise DownloadError("拒绝非GitHub官方API下载地址")
    gh_environment = dict(environment)
    for key in ("GH_DEBUG", "DEBUG", "GH_FORCE_TTY", "CLICOLOR_FORCE", "GITHUB_TOKEN", "GH_ENTERPRISE_TOKEN", "GITHUB_ENTERPRISE_TOKEN"):
        gh_environment.pop(key, None)
    gh_environment.update(GH_TOKEN=token, GH_HOST="github.com", GH_PROMPT_DISABLED="1", GH_PAGER="cat", NO_COLOR="1", GH_NO_UPDATE_NOTIFIER="1")
    # 令牌只放在本次子进程环境中，不进入参数、日志或新的登录配置。
    with tempfile.TemporaryFile() as output:
        try:
            result = subprocess.run([gh, "api", "--hostname", "github.com", "--method", "GET", "--include",
                                     "--header", "Accept: " + accept, url], env=gh_environment,
                                    stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.PIPE,
                                    timeout=DOWNLOAD_TIMEOUT)
        except subprocess.TimeoutExpired:
            raise DownloadError("GitHub请求超时", True) from None
        output.seek(0)
        status_line = output.readline(8193)
        status_match = re.match(br"HTTP/\S+ (\d{3})\b", status_line)
        status = int(status_match.group(1)) if status_match else None
        if status is not None and not 200 <= status < 300:
            raise http_failure(status)
        if result.returncode:
            error = result.stderr.lower()
            if b"certificate" in error or b"x509:" in error:
                raise DownloadError("TLS证书验证失败；请检查设备时间与CA证书，不要关闭证书验证")
            transient = result.returncode == 1 and any(part in error for part in (
                b"eof", b"timeout", b"timed out", b"connection reset", b"connection refused",
                b"error connecting to", b"temporary failure", b"network is unreachable",
                b"context deadline exceeded", b"stream error", b"goaway"))
            raise DownloadError("GitHub连接中断" if transient else "GitHub CLI请求失败；请检查已有登录与网络", transient)
        if status is None:
            raise DownloadError("GitHub API响应缺少有效状态")
        header_bytes = len(status_line)
        while True:
            line = output.readline(8193)
            header_bytes += len(line)
            if not line or len(line) > 8192 or header_bytes > 65536:
                raise DownloadError("GitHub API响应头无效")
            if line in (b"\n", b"\r\n"):
                break
        return output.read(MAX_DOWNLOAD + 1)

def download(url, accept):
    headers = {"User-Agent": "NEH-source-bootstrap/1", "Accept": accept}
    if token:
        headers["Authorization"] = "Bearer " + token
    for attempt in range(1, DOWNLOAD_ATTEMPTS + 1):
        try:
            if gh:
                content = gh_download(url, accept)
            else:
                with opener.open(urllib.request.Request(url, headers=headers), timeout=DOWNLOAD_TIMEOUT) as response:
                    content = response.read(MAX_DOWNLOAD + 1)
                    # read(limit)可能静默返回被截断的Content-Length响应。
                    if len(content) <= MAX_DOWNLOAD and getattr(response, "length", 0):
                        raise http.client.IncompleteRead(content)
            if len(content) > MAX_DOWNLOAD:
                raise DownloadError("引导源码超过大小限制")
            return content
        except urllib.error.HTTPError as exc:
            failure = http_failure(exc.code)
            if exc.fp is not None:
                exc.close()
        except (urllib.error.URLError, OSError, http.client.HTTPException) as exc:
            failure = network_failure(exc)
        except DownloadError as exc:
            failure = exc
        if not failure.retry or attempt == DOWNLOAD_ATTEMPTS:
            raise SystemExit("无法获取安装源码（%s；尝试%d/%d次）。请确认网络后重新运行原安装命令。" %
                             (failure.reason, attempt, DOWNLOAD_ATTEMPTS)) from None
        print("安装源码下载暂时失败（%s）；%d秒后重试（%d/%d）" %
              (failure.reason, attempt, attempt + 1, DOWNLOAD_ATTEMPTS), file=sys.stderr, flush=True)
        time.sleep(attempt)

local = os.environ["NEH_BOOTSTRAP_ALLOW_LOCAL"] == "1" and parsed.scheme in ("", "file")
if local:
    if token or parsed.netloc:
        raise SystemExit("本机源码验收不能附带远端凭据或主机名")
    root = pathlib.Path(urllib.parse.unquote(parsed.path) if parsed.scheme else repository).expanduser().resolve(strict=True)
    def git(*args):
        return subprocess.check_output(["git", "-c", "core.hooksPath=/dev/null", "-c", "core.fsmonitor=false", "-C", str(root), *args], env=environment)
    revision = git("rev-parse", "--verify", (revision or "refs/heads/" + branch) + "^{commit}").decode().strip()
else:
    parts = re.sub(r"\.git$", "", parsed.path.strip("/")).split("/")
    if parsed.scheme != "https" or parsed.netloc != "github.com" or len(parts) != 2 or any(not re.fullmatch(r"[A-Za-z0-9_.-]+", part) for part in parts):
        raise SystemExit("首段源码下载需要不含凭据的GitHub HTTPS仓库地址")
    api = "https://api.github.com/repos/" + "/".join(parts)
    if not revision:
        metadata = json.loads(download(api + "/commits/" + urllib.parse.quote(branch, safe=""), "application/vnd.github+json"))
        revision = metadata.get("sha", "")
if not re.fullmatch(r"[0-9a-f]{40,64}", revision):
    raise SystemExit("远端没有返回有效完整源码提交")
for relative in ("scripts/neh_bootstrap.py", "scripts/neh_source.py"):
    content = git("show", revision + ":" + relative) if local else download(api + "/contents/" + relative + "?ref=" + revision, "application/vnd.github.raw+json")
    compile(content, relative, "exec")
    target = stage / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(content)
(stage / "revision").write_text(revision)
print("已固定安装源码：" + revision[:12], flush=True)
PY
NEH_BOOTSTRAP_FIXED_REVISION=$(cat "$NEH_BOOTSTRAP_TEMP/revision")
if [[ " ${NEH_BOOTSTRAP_ARGS[*]} " == *' --cli '* ]] && ( : </dev/tty ) 2>/dev/null; then
  python3 "$NEH_BOOTSTRAP_TEMP/scripts/neh_bootstrap.py" "${NEH_BOOTSTRAP_ARGS[@]}" --revision "$NEH_BOOTSTRAP_FIXED_REVISION" </dev/tty
else
  python3 "$NEH_BOOTSTRAP_TEMP/scripts/neh_bootstrap.py" "${NEH_BOOTSTRAP_ARGS[@]}" --revision "$NEH_BOOTSTRAP_FIXED_REVISION"
fi
