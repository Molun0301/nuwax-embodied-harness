#!/usr/bin/env bash
# scripts/ci/no_external_nuwax_workspace_path.sh
#
# 目的：禁止 dimos 绝对路径 `/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace`
#       出现在源码、Cargo.toml、installer、.env 等真实生效的源文件里。
#
# 对应执行方案：
#   docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md
#   §5 Phase 0 任务 2（防回归门禁）
#   §6 DoD-2：`rg -n '/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace' services core contracts scripts .env.example` 0 命中
#   §12.7 DoD-3：扩展范围到 deploy/
#
# 扫描范围：
#   - 目录：services core contracts scripts deploy
#   - 单文件：.env.example
#   - 扩展名过滤：.py .toml .sh .md .yaml .yml .env（避免误伤二进制）
#
# 白名单（合理保留）：
#   - 本任务目录自身的 .md / 文档归档：
#       docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/
#   - 历史归档文档目录：
#       docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/
#       docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/
#   - IMPORT_MANIFEST 文件中的 provenance / phash / source 字段（用于追溯，
#     但必须在 services/nuwax-workspace/IMPORT_MANIFEST* 范围内）。
#
# 退出码：
#   0  - OK：未命中（或者命中全部位于白名单）
#   1  - FAIL：命中且非白名单，列出 file:line:content
#
# 当前预期：FAIL（这是预期的，因为 .env.example / Cargo.toml / contracts /
#          installer/preflight.py 仍存在 dimos 绝对路径）。Phase 1 完成
#          路径切换后预期 PASS。

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# 被禁绝对路径（用 fixed-string 匹配，避免 grep -E 误伤）
FORBIDDEN_PATH='/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace'

# 任务目录白名单前缀（注意：路径中含中文，必须双引号）
WHITELIST_PREFIXES=(
  "docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/"
  "docs/开发文档/建图导航探索重构/2026-07-01_彻底去ROS2化-执行方案/"
  "docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/"
)

# IMPORT_MANIFEST 边界（命中路径前缀 + 字段语义）
IMPORT_MANIFEST_GLOB='services/nuwax-workspace/IMPORT_MANIFEST*'

# 扫描范围
SCAN_PATHS=(
  "services"
  "core"
  "contracts"
  "scripts"
  "deploy"
  ".env.example"
)

# 扩展名过滤（仅扫描可读源/配置）
EXTS_PATTERN='\.(py|toml|sh|md|yaml|yml|env)$'

# 把参数扩展为 git ls-files 可消费的 pathspec（仅 git 已跟踪 + 工作区存在文件）
# 用 git ls-files + others --exclude-standard 覆盖 tracked + untracked（untracked 不纳入 CI）
# 简化决策：本脚本仅扫已跟踪文件（git ls-files），与 no_ros2_residual.sh 保持一致。
# untracked 落地的源文件由 lane 提交后再纳入扫描。

# --- 收集候选文件 ---
mapfile -t CANDIDATES < <(
  git ls-files \
    -- "${SCAN_PATHS[@]}" 2>/dev/null \
    | grep -E "${EXTS_PATTERN}" \
    | grep -vE '^scripts/ci/no_external_nuwax_workspace_path\.sh$' \
    || true
)

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "[no_external_nuwax_workspace_path] PASS: 扫描范围内无可检查文件"
  exit 0
fi

# --- 命中检查（grep -F 固定字符串，file:line:content 格式）---
HITS=$(grep -nF "${FORBIDDEN_PATH}" "${CANDIDATES[@]}" 2>/dev/null || true)

if [[ -z "${HITS}" ]]; then
  echo "[no_external_nuwax_workspace_path] OK: ${#CANDIDATES[@]} 个文件 0 命中"
  exit 0
fi

# --- 白名单过滤 ---
FILTERED=""
while IFS= read -r hit; do
  [[ -z "$hit" ]] && continue
  file="${hit%%:*}"  # file:line:content → file
  # 前缀白名单
  skip=0
  for prefix in "${WHITELIST_PREFIXES[@]}"; do
    if [[ "$file" == "$prefix"* ]]; then
      skip=1
      break
    fi
  done
  if [[ "$skip" -eq 1 ]]; then
    continue
  fi
  # IMPORT_MANIFEST 文件允许命中（用于记录 provenance / source / sha256 等元数据）
  for f in $IMPORT_MANIFEST_GLOB; do
    if [[ "$file" == $f ]]; then
      skip=1
      break
    fi
  done
  if [[ "$skip" -eq 1 ]]; then
    continue
  fi
  FILTERED+="${hit}"$'\n'
done <<< "${HITS}"

FILTERED="${FILTERED%$'\n'}"  # 去尾换行

if [[ -z "${FILTERED}" ]]; then
  echo "[no_external_nuwax_workspace_path] OK: 所有命中位于白名单（历史 docs 或 IMPORT_MANIFEST）"
  exit 0
fi

echo "[no_external_nuwax_workspace_path] FAIL: 命中 ${FORBIDDEN_PATH}" >&2
echo "${FILTERED}" >&2
echo "" >&2
echo "如需放行（仅限历史归档），请把对应目录追加到本脚本的 WHITELIST_PREFIXES 并在 docs/ 留根因备忘。" >&2
exit 1