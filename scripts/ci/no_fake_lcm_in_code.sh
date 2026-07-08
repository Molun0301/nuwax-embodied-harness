#!/usr/bin/env bash
# scripts/ci/no_fake_lcm_in_code.sh
#
# 目的:代码层硬门 — 禁止仓库生产代码出现任何 fake-lcm 引用。
# 2026-07-08 fake-lcm 物理删除(v3 F2 决策)后,本门守"fake-lcm 没有存在的意义"硬约束。
#
# 对应执行方案:
#   docs/开发文档/建图导航探索重构/补丁文档/2026-07-08_彻底根除fake-lcm接入执行方案/00_最终执行方案.md
#
# 扫描模式(全仓 Rust + Python + TOML,排除 dev-only 区域):
#   FakeLcmBackend / new_fake() / MockLcmBackend / LcmBackendKind::Fake
#   fake-lcm-test / "fake-lcm" / 'fake-lcm' / mock://
#
# 退出码:
#   0  - OK:全仓无 fake-lcm 引用
#   1  - FAIL:命中,列出 file:line + 上下文
#
# 豁免目录(纯历史归档,内含已弃用 fake 描述):
#   - docs/开发文档/  (按 CLAUDE.md 约定不入仓)
#   - vendor_history/  (上游历史)
#   - migration/_deprecated/  (已 git mv 弃用)
#   - target/ .git/  (构建/版本控制)

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# 优先 ripgrep,失败 fallback git grep
USE_RG=0
if command -v rg >/dev/null 2>&1; then
  USE_RG=1
fi

# 扫描模式 — 仅 token 标识符(cargo feature / 类名 / 工厂函数)
# 字符串字面量("fake-lcm" / 'fake-lcm' / 'mock://')**不**在扫描范围:
# v3 决策要求 from_env / _normalize_lcm_backend / _resolve_backend 看到
# "fake-lcm" / "mock://" 必须 reject(字符串是拒绝触发器),不应被 gate 误伤。
PATTERNS=(
  "FakeLcmBackend"
  "new_fake\\(\\)"
  "MockLcmBackend"
  "LcmBackendKind::Fake"
  "fake-lcm-test"
)

# 收集所有命中
HITS_FILE="$(mktemp)"
trap 'rm -f "$HITS_FILE"' EXIT

build_rg_cmd() {
  local pattern="$1"
  rg -n --no-heading --color=never \
    -g '*.rs' -g '*.py' -g '*.toml' \
    -g '!docs/**' -g '!target/**' -g '!.git/**' \
    -g '!vendor_history/**' -g '!_deprecated/**' \
    -g '!**/_deprecated/**' -g '!**/migration/_deprecated/**' \
    "$pattern" services core contracts drivers scripts gateways pyproject.toml 2>/dev/null
}

build_git_grep_cmd() {
  local pattern="$1"
  git grep -nIE -- \
    "$pattern" \
    -- 'services/*.rs' 'services/*.py' 'services/*.toml' \
    'core/*.rs' 'core/*.py' 'core/*.toml' \
    'contracts/*.rs' 'contracts/*.py' 'contracts/*.toml' \
    'drivers/*.rs' 'drivers/*.py' 'drivers/*.toml' \
    'scripts/*.rs' 'scripts/*.py' 'scripts/*.toml' \
    'scripts/ci/*.rs' 'scripts/ci/*.py' 'scripts/ci/*.toml' \
    'gateways/*.rs' 'gateways/*.py' 'gateways/*.toml' \
    'pyproject.toml' 2>/dev/null \
    | grep -vE '(docs/开发文档/|/_deprecated/|/vendor_history/|/target/)' \
    || true
}

for pattern in "${PATTERNS[@]}"; do
  if [[ "$USE_RG" -eq 1 ]]; then
    build_rg_cmd "$pattern" >> "$HITS_FILE" || true
  else
    build_git_grep_cmd "$pattern" >> "$HITS_FILE" || true
  fi
done

# 去重 + 排序
sort -u "$HITS_FILE" -o "$HITS_FILE"

# 过滤注释行:doc-comment 提及"fake-lcm 已物理删除"是 v3 决策的合法说明,不应 gate fail
# Rust 注释起始: //, /*, *, ///
# Python/TOML 注释起始: #
# 过滤规则:行 trimmed 起始字符是注释标记则忽略
FILTERED_HITS="$(mktemp)"
trap 'rm -f "$HITS_FILE" "$FILTERED_HITS"' EXIT

while IFS= read -r hit; do
  [[ -z "$hit" ]] && continue
  # 提取 content(去掉 file:line: 前缀)
  content="${hit#*:*:}"
  trimmed="$(echo "$content" | sed -E 's/^[[:space:]]+//')"
  # 注释/docstring 起始字符 / 缩进 list item
  case "$trimmed" in
    "//"*|"///"*|"//!"*|"/*"*|"*") continue ;;
    "#"*) continue ;;
    "- "*) continue ;;  # 缩进 list item 视为 docstring 一部分
  esac
  # docstring 中间行 / 字符串字面量 / 含 v3 决策标记 → 内容是合规说明
  # 豁免信号:本行含以下任一关键字
  if echo "$content" | grep -qE '2026-07-08 fake-lcm 物理删除|2026-07-08 fake-lcm 根除|2026-07-08 v3 F2|fake-lcm 已物理删除|fake-lcm backend 已物理删除|docstring|d""|r"""|"""' ; then
    continue
  fi
  echo "$hit" >> "$FILTERED_HITS"
done < "$HITS_FILE"

if [[ ! -s "$FILTERED_HITS" ]]; then
  echo "[no_fake_lcm_in_code] OK: 全仓代码层无 fake-lcm 引用(注释/docstring 提及已豁免)"
  exit 0
fi

# 命中 — 列 file:line + 上下文
echo "[no_fake_lcm_in_code] FAIL: 检测到 fake-lcm 代码层引用(v3 F2 决策:fake-lcm 没有存在的意义)" >&2
echo "" >&2
echo "命中清单(已去重 + 排除注释/docstring):" >&2
cat "$FILTERED_HITS" >&2
echo "" >&2
echo "如确需保留(legacy 兼容),请用以下豁免手段之一:" >&2
echo "  1. 把文件物理迁入 _deprecated/ 子目录(本 gate 自动豁免)" >&2
echo "  2. 删除对应代码行(本任务期望的做法)" >&2
echo "" >&2
echo "禁止:cfg gate / compile_error! 守门 / 兼容分支(v3 已明确禁止)。" >&2
exit 1
