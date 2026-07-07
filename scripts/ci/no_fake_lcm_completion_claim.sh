#!/usr/bin/env bash
# scripts/ci/no_fake_lcm_completion_claim.sh
#
# 目的：禁止 docs/开发文档/建图导航探索重构/ 下任何 .md 把 real-LCM fail-fast
#       标为"PASS / 完成 / 已实现 / 接入"。防止验收报告用 fake-LCM 内通或
#       启动期 placeholder 拒绝冒充 real-LCM 通信完成。
#
# 对应执行方案：
#   docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md
#   §5 Phase 0 任务 3（防虚假完成门禁）
#   §6 DoD-6：`smoke_real_lcm_roundtrip.sh` PASS（不得用 fail-fast 当 PASS）
#   §6 DoD-7：lcm-logger 观测真实 channel（不得用 fake backend 内存事件当 PASS）
#
# 检测模式（滑动窗口 ±3 行）：
#   - 同时包含 "real-lcm" 与 "PASS"
#   - 紧邻（±3 行内）包含 "完成 / 已实现 / 接入 / 通过 / ok / 验证 / 接入成功 / 集成"
#     中任何一个
#
# 豁免：
#   - 当前任务目录自身 `2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/`
#     下的 plan / phase 描述、todolist、feedback 等"正在执行中"的文档
#   - 历史归档目录（`2026-07-01_彻底去ROS2化-执行方案/`、
#     `2026-07-06_建图导航探索库完整接入执行方案/`）
#
# 退出码：
#   0  - OK：未命中虚假完成表述
#   1  - FAIL：命中，列出 file:line + 上下文
#
# 当前预期：PASS（因为没有最终验收报告写完成）。

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

DOCS_DIR="docs/开发文档/建图导航探索重构"

if [[ ! -d "$DOCS_DIR" ]]; then
  echo "[no_fake_lcm_completion_claim] OK: 扫描目录不存在，跳过"
  exit 0
fi

# 豁免目录（当前正在执行的任务目录 + 历史归档目录）
EXCLUDE_DIRS=(
  "2026-07-07_建图导航探索库真接入与real-lcm通信实施方案"
  "2026-07-01_彻底去ROS2化-执行方案"
  "2026-07-06_建图导航探索库完整接入执行方案"
)

# 1) 先用 ripgrep（若可用）或 grep -F 收集所有含 "real-lcm" 的命中行（file:line:content）
REAL_LCM_HITS_FILE="$(mktemp)"
trap 'rm -f "${REAL_LCM_HITS_FILE}" "${REAL_LCM_HITS_FILE}.ctx"' EXIT

if command -v rg >/dev/null 2>&1; then
  rg -n --no-heading --color=never "real-lcm" "$DOCS_DIR" --glob '*.md' \
    > "${REAL_LCM_HITS_FILE}" 2>/dev/null || true
else
  # git grep fallback（仅扫 git 已跟踪的 .md）
  git grep -nF -- "real-lcm" -- 'docs/开发文档/建图导航探索重构/*.md' 'docs/开发文档/建图导航探索重构/**/*.md' \
    > "${REAL_LCM_HITS_FILE}" 2>/dev/null || true
fi

if [[ ! -s "${REAL_LCM_HITS_FILE}" ]]; then
  echo "[no_fake_lcm_completion_claim] OK: 扫描范围内无 real-lcm 关键字引用"
  exit 0
fi

# 2) 检查每条命中是否在 ±3 行内同时出现 PASS + 完成类关键词
#    取出命中文件，做上下文扫描
FAKE_FINDS_FILE="${REAL_LCM_HITS_FILE}.ctx"
: > "${FAKE_FINDS_FILE}"

# 完成类关键词（中文 + 英文）
COMPLETION_KEYWORDS_REGEX='完成|已实现|已接入|接入完成|接入成功|集成成功|实现完毕|通过验证|验证通过|已通过|已实现完成|ok|OK|Passed|PASSED'

while IFS= read -r hit; do
  [[ -z "$hit" ]] && continue

  # 解析 file:line:content
  file="${hit%%:*}"
  rest="${hit#*:}"
  line="${rest%%:*}"

  # 豁免目录
  skip=0
  for exdir in "${EXCLUDE_DIRS[@]}"; do
    if [[ "$file" == *"$exdir"* ]]; then
      skip=1
      break
    fi
  done
  [[ "$skip" -eq 1 ]] && continue

  # 取该文件全文（md）扫描 ±3 行窗口
  if [[ ! -f "$file" ]]; then
    continue
  fi

  # 用 awk 在 file 中查找 line 附近 3 行是否有 completion 关键字
  # 输出文件:命中行:window_text
  context=$(awk -v target_line="$line" -v win=3 '
    NR >= target_line - win && NR <= target_line + win {
      print NR": "$0
    }
  ' "$file")

  if echo "$context" | grep -qE "${COMPLETION_KEYWORDS_REGEX}"; then
    echo "FAKE-LCM-COMPLETION: $file:$line" >> "${FAKE_FINDS_FILE}"
    echo "$context" | sed 's/^/  | /' >> "${FAKE_FINDS_FILE}"
    echo "" >> "${FAKE_FINDS_FILE}"
  fi
done < "${REAL_LCM_HITS_FILE}"

if [[ ! -s "${FAKE_FINDS_FILE}" ]]; then
  echo "[no_fake_lcm_completion_claim] OK: real-lcm 引用全部位于豁免目录或无虚假完成表述"
  exit 0
fi

echo "[no_fake_lcm_completion_claim] FAIL: 检测到 real-lcm 虚假完成表述" >&2
cat "${FAKE_FINDS_FILE}" >&2
echo "" >&2
echo "real-lcm 通信真实完成必须由 smoke_real_lcm_roundtrip.sh + lcm-logger 共同证明，" >&2
echo "不得用启动期 fail-fast 或 fake backend 内存事件冒充 PASS。" >&2
exit 1