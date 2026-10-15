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
import json, os, pathlib, re, subprocess, sys, urllib.error, urllib.parse, urllib.request
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
def download(url, accept):
    headers = {"User-Agent": "NEH-source-bootstrap/1", "Accept": accept}
    if token:
        headers["Authorization"] = "Bearer " + token
    try:
        with opener.open(urllib.request.Request(url, headers=headers), timeout=45) as response:
            content = response.read(2 * 1024 * 1024 + 1)
    except urllib.error.HTTPError as exc:
        raise SystemExit("无法获取安装源码（HTTP %s）；请核对私有仓库访问权限" % exc.code) from exc
    if len(content) > 2 * 1024 * 1024:
        raise SystemExit("引导源码超过大小限制")
    return content

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
