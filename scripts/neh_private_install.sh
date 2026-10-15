#!/usr/bin/env bash
# 私有测试期的首段引导：准备下载授权后交给现有产品安装器。
set -euo pipefail
umask 077
if [[ ${EUID} -eq 0 ]]; then
  echo "请以普通用户运行，安装服务时会自动请求管理员权限。" >&2
  exit 1
fi
if [[ $(uname -s) != Linux ]]; then
  echo "此安装入口仅支持Linux。" >&2
  exit 1
fi
NEH_ENTRY_TEMP=$(mktemp -d -t neh-entry.XXXXXXXX)
trap "rm -rf -- \"$NEH_ENTRY_TEMP\"" EXIT
if ! command -v gh >/dev/null 2>&1; then
  if ! command -v curl >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install --yes curl ca-certificates
  fi
  case "$(uname -m)" in
    x86_64) NEH_GH_ARCH=amd64; NEH_GH_SHA=bb766f710eef8ede859c18578c72c327597cd4c8a85b06001b1f3843c6019386 ;;
    aarch64|arm64) NEH_GH_ARCH=arm64; NEH_GH_SHA=7862c86c72f43df3a2d93ddde6f473285b4e2af61b494849846827e513ef6484 ;;
    *) echo "当前安装入口支持Linux x86_64和ARM64。" >&2; exit 1 ;;
  esac
  curl --fail --silent --show-error --location --retry 3 "https://github.com/cli/cli/releases/download/v2.102.0/gh_2.102.0_linux_${NEH_GH_ARCH}.tar.gz" -o "$NEH_ENTRY_TEMP/gh.tar.gz"
  printf "%s  %s\n" "$NEH_GH_SHA" "$NEH_ENTRY_TEMP/gh.tar.gz" | sha256sum --check --status
  tar -xzf "$NEH_ENTRY_TEMP/gh.tar.gz" -C "$NEH_ENTRY_TEMP" --strip-components=2 "gh_2.102.0_linux_${NEH_GH_ARCH}/bin/gh"
  export PATH="$NEH_ENTRY_TEMP:$PATH"
fi
if ! gh auth status --hostname github.com >/dev/null 2>&1; then
  # 本次登录隔离保存；安装器只把必要的下载授权写入本设备的受限配置。
  export GH_CONFIG_DIR="$NEH_ENTRY_TEMP/github"
  unset GH_TOKEN GITHUB_TOKEN GH_ENTERPRISE_TOKEN GITHUB_ENTERPRISE_TOKEN
  gh auth login --hostname github.com --web --git-protocol https --skip-ssh-key
fi
gh auth token --hostname github.com > "$NEH_ENTRY_TEMP/token"
if ! gh api "repos/Molun0301/nuwax-embodied-harness/contents/get-neh.sh?ref=main" --header "Accept: application/vnd.github.raw+json" > "$NEH_ENTRY_TEMP/get-neh.sh"; then
  echo "当前GitHub账户无法读取测试仓库，请使用已获得测试权限的账户。" >&2
  exit 1
fi
unset GH_TOKEN GITHUB_TOKEN GH_ENTERPRISE_TOKEN GITHUB_ENTERPRISE_TOKEN GH_CONFIG_DIR
bash "$NEH_ENTRY_TEMP/get-neh.sh" --token-file "$NEH_ENTRY_TEMP/token" "$@"
