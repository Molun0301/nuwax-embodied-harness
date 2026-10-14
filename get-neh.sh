#!/usr/bin/env bash
# 首次安装可下载此脚本执行；已克隆源码可直接 ./install.sh product。
set -euo pipefail
umask 077

if [[ ${EUID} -eq 0 ]]; then
  echo '请以普通用户执行安装；系统服务会单独请求管理员权限。' >&2
  exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install --yes python3
  elif command -v dnf >/dev/null 2>&1; then
    sudo dnf install --assumeyes python3
  elif command -v yum >/dev/null 2>&1; then
    sudo yum install --assumeyes python3
  elif command -v pacman >/dev/null 2>&1; then
    sudo pacman -S --needed --noconfirm python
  elif command -v zypper >/dev/null 2>&1; then
    sudo zypper --non-interactive install python3
  elif command -v apk >/dev/null 2>&1; then
    sudo apk add --no-cache python3
  else
    echo '此系统尚无引导依赖适配器，请先安装Python 3.8至3.12。' >&2
    exit 1
  fi
fi

NEH_BOOTSTRAP_TEMP=$(mktemp -d -t neh-bootstrap.XXXXXXXX)
trap 'rm -rf -- "${NEH_BOOTSTRAP_TEMP}"' EXIT
export NEH_BOOTSTRAP_DOWNLOAD="${NEH_BOOTSTRAP_URL:-https://github.com/Molun0301/nuwax-embodied-harness/releases/latest/download/neh_bootstrap.py}"
export NEH_BOOTSTRAP_TARGET="${NEH_BOOTSTRAP_TEMP}/neh_bootstrap.py"

# 只为引导下载读取凭据文件路径，不在进程参数或网址中放入凭据正文。
NEH_BOOTSTRAP_ARGS=("$@")
for ((neh_i=0; neh_i < ${#NEH_BOOTSTRAP_ARGS[@]}; neh_i++)); do
  if [[ ${NEH_BOOTSTRAP_ARGS[neh_i]} == --token-file ]]; then
    export NEH_BOOTSTRAP_TOKEN_FILE="${NEH_BOOTSTRAP_ARGS[neh_i+1]:-}"
  elif [[ ${NEH_BOOTSTRAP_ARGS[neh_i]} == --token-file=* ]]; then
    export NEH_BOOTSTRAP_TOKEN_FILE="${NEH_BOOTSTRAP_ARGS[neh_i]#--token-file=}"
  fi
done
python3 - <<'PY'
import json, os, pathlib, sys, urllib.parse, urllib.request
if not (3, 8) <= sys.version_info[:2] < (3, 13):
    raise SystemExit("引导安装需要Python 3.8至3.12")
url = os.environ["NEH_BOOTSTRAP_DOWNLOAD"]
parsed = urllib.parse.urlsplit(url)
if parsed.scheme != "https" or parsed.username or parsed.password:
    raise SystemExit("引导下载必须使用不含凭据的HTTPS地址")
token = ""
if os.environ.get("NEH_BOOTSTRAP_TOKEN_FILE"):
    path = pathlib.Path(os.environ["NEH_BOOTSTRAP_TOKEN_FILE"]).expanduser()
    if path.stat().st_mode & 0o077:
        raise SystemExit("下载凭据文件须设置为0600权限")
    token = path.read_text().strip()
class Redirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urllib.parse.urlsplit(newurl).scheme != "https":
            raise ValueError("拒绝不安全重定向")
        result = super().redirect_request(req, fp, code, msg, headers, newurl)
        if result and urllib.parse.urlsplit(req.full_url).netloc != urllib.parse.urlsplit(newurl).netloc:
            result.remove_header("Authorization")
        return result
opener = urllib.request.build_opener(Redirect())
headers = {"User-Agent": "NEH-bootstrap/1"}
if token:
    headers["Authorization"] = "Bearer " + token
parts = parsed.path.strip("/").split("/")
if token and parsed.hostname == "github.com" and len(parts) >= 6 and parts[2] == "releases":
    endpoint = "latest" if parts[3:5] == ["latest", "download"] else "tags/" + parts[4]
    metadata_url = "https://api.github.com/repos/" + "/".join(parts[:2]) + "/releases/" + endpoint
    with opener.open(urllib.request.Request(metadata_url, headers=headers), timeout=30) as response:
        metadata = json.load(response)
    matches = [asset for asset in metadata.get("assets", []) if asset.get("name") == parts[-1]]
    if not matches or urllib.parse.urlsplit(matches[0].get("url", "")).netloc != "api.github.com":
        raise SystemExit("发布版本缺少引导程序附件")
    url = matches[0]["url"]
headers["Accept"] = "application/octet-stream"
with opener.open(urllib.request.Request(url, headers=headers), timeout=45) as response:
    data = response.read(2 * 1024 * 1024 + 1)
if len(data) > 2 * 1024 * 1024:
    raise SystemExit("引导程序超过大小限制")
pathlib.Path(os.environ["NEH_BOOTSTRAP_TARGET"]).write_bytes(data)
PY
if [[ " ${NEH_BOOTSTRAP_ARGS[*]} " == *' --cli '* && -r /dev/tty ]]; then
  python3 "${NEH_BOOTSTRAP_TARGET}" "${NEH_BOOTSTRAP_ARGS[@]}" </dev/tty
else
  python3 "${NEH_BOOTSTRAP_TARGET}" "${NEH_BOOTSTRAP_ARGS[@]}"
fi
