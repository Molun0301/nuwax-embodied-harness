#!/usr/bin/env bash
# =============================================================================
# vendor_dimos_crates.sh
# -----------------------------------------------------------------------------
# 状态:DEPRECATED(2026-07-07,Phase 1 起) — 不再被调用。
#
# 替代脚本:
#   scripts/import_nuwax_workspace_snapshot.sh
#   该脚本会把 dimos 整棵 nuwax-workspace(含 48+ 个 crate / Cargo.toml /
#   Cargo.lock / .cargo/config.toml / lcmtypes)落到本仓
#   services/nuwax-workspace/,入仓作为 tracked snapshot。
#   services/geometry/nuwax_geometry_runtime/Cargo.toml 中的 13 条
#   path dep 也已在 Phase 1 切到 repo-local 相对路径
#   ../../nuwax-workspace/crates/<name>,不再需要 vendor/<name> 兜底。
#
# 历史背景:
#   Phase 0 / Phase 8 期间,13 个 path dep 指向开发机绝对路径
#   /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/<name>,
#   目标机 aarch64 + Ubuntu 20.04 没有该路径;本脚本当时把 crate 复制到
#   services/geometry/nuwax_geometry_runtime/vendor/<name>/ 兜底。
#
# 保留原因:
#   - 历史 commit 可读性:旧 commit hash 仍可回放;
#   - 应急:如果新脚本 import_nuwax_workspace_snapshot.sh 失效,可临时
#     走回本脚本对单一 crate 做 vendor(不推荐,会有 target/ 路径漂移)。
#
# 不删:
#   见 .gitignore 注释:已不再忽略 vendor/vendor-config,改忽略
#   services/nuwax-workspace/{target,.omc,.omx}。
# -----------------------------------------------------------------------------
# 目的:
#   把 services/geometry/nuwax_geometry_runtime/Cargo.toml 中 13 个
#   path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/<name>"
#   风格的 nuwax-* path dep **就地 vendor 化**:复制 crate 源码到本仓的
#   services/geometry/nuwax_geometry_runtime/vendor/<name>/,并把 Cargo.toml
#   的 path 改写为相对路径 vendor/<name>。
#
# 为什么需要这个脚本:
#   dev 机的 13 个 path dep 全部指向开发机绝对路径
#   /home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/...
#   目标机(aarch64 + Ubuntu 20.04 + Jetson)上根本没有这条路径。
#   不 vendor = 目标机 cargo build 必失败(aarch64 编译期/链接期都无法跑通)。
#
# 上游替代方案 [patch.git] / crates.io 发布:
#   当前 dimos nuwax-workspace 还未发布到 crates.io,也没稳定的 git tag。
#   故本期采用"vendor 静态快照"兜底。
#
# 行为契约:
#   - idempotent:重跑只更新源 hash 不一致的 crate;源一致跳过。
#   - 复制 .git 之外的全部文件(包括 Cargo.toml / src/ / tests/ / examples/ 等)。
#   - sed 改写 Cargo.toml 是幂等的(已经 vendor 化的也是 vendor/<name>)。
#   - 自动写 vendor-config.json(manifest:13 个 crate 源 + commit hash + 时间)。
#   - 自动写 .cargo/config.toml(cargo vendor 模式需要这个文件存在)。
#   - 执行后 NUWAX_LCM_BACKEND 默认仍是 fake-lcm(本脚本不动 LCM backend)。
#
# 不做:
#   - 不实际跑 cargo vendor(避免删除 .cargo/config.toml 中我们设置的 offline
#     配置;Cargo.toml 仍以 Cargo.lock 生成时为准)。
#   - 不 commit 任何 vendor/ 内容(产物路径已在 .gitignore 排除)。
#   - 不动 .env / .env.example / Web 配置面板路径。
#
# 退出码:
#   0  成功;    非 0  表示源目录不可达 / Cargo.toml 解析失败 / 复制失败等。
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# 路径常量
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUNTIME_DIR="$REPO_ROOT/services/geometry/nuwax_geometry_runtime"
VENDOR_DIR="$RUNTIME_DIR/vendor"
CARGO_TOML="$RUNTIME_DIR/Cargo.toml"
CARGO_CONFIG_DIR="$RUNTIME_DIR/.cargo"
CARGO_CONFIG_TOML="$CARGO_CONFIG_DIR/config.toml"
VENDOR_CONFIG_JSON="$RUNTIME_DIR/vendor-config.json"

# 上游 nuwax-workspace 绝对路径(dev 机)
UPSTREAM_ROOT="/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace"

# 颜色输出(只在 tty + 没被 CI 强制时打开)
if [[ -t 1 ]] && [[ -z "${NO_COLOR:-}" ]]; then
  C_RED='\033[0;31m'; C_GREEN='\033[0;32m'; C_YEL='\033[0;33m'; C_RST='\033[0m'
else
  C_RED=''; C_GREEN=''; C_YEL=''; C_RST=''
fi

log()  { printf '%b[vendor]%b %s\n' "$C_GREEN" "$C_RST" "$*"; }
warn() { printf '%b[vendor]%b %s\n' "$C_YEL" "$C_RST" "$*" >&2; }
die()  { printf '%b[vendor]%b %s\n' "$C_RED" "$C_RST" "$*" >&2; exit 1; }

# -----------------------------------------------------------------------------
# 0. 预检
# -----------------------------------------------------------------------------
[[ -f "$CARGO_TOML" ]] || die "Cargo.toml not found: $CARGO_TOML"
[[ -d "$UPSTREAM_ROOT/crates" ]] || die "upstream nuwax-workspace not found: $UPSTREAM_ROOT/crates"

mkdir -p "$VENDOR_DIR" "$CARGO_CONFIG_DIR"

# -----------------------------------------------------------------------------
# 1. 从 Cargo.toml 解析 path deps,提取 <name>
#    匹配规则:
#      ^nuwax-XXX = { path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-XXX" [, ...] }
#    也兼容 vendor 化后已存在的相对路径 vendor/nuwax-XXX(以幂等)
# -----------------------------------------------------------------------------
log "扫描 Cargo.toml 的 nuwax-* path deps ..."
mapfile -t NUWAX_DEPS < <(
  awk -v prefix="nuwax-" '
    $0 ~ "^" prefix "[-a-z0-9_]*[[:space:]]*=" {
      # 取键名
      match($0, /^[[:space:]]*(nuwax-[-a-z0-9_]+)/, m)
      key = m[1]
      if (key == "") next
      # 取 path 值(绝对或 vendor 相对)
      match($0, /path[[:space:]]*=[[:space:]]*"([^"]+)"/, p)
      path = p[1]
      if (path == "") next
      # 绝对路径 → 期望 /.../crates/<name>
      # vendor 路径 → 期望 vendor/<name>
      if (path ~ /\/crates\//) {
        # .../crates/nuwax-foo 形式
        n = split(path, parts, "/")
        leaf = parts[n]
      } else if (path ~ /^vendor\//) {
        n = split(path, parts, "/")
        leaf = parts[2]
      } else {
        next
      }
      # 强校验:key 与 leaf 一致,避免 sed match 错位
      if (leaf == key) print key
    }
  ' "$CARGO_TOML"
)

# awk 在 GNU 与 BSD 行为略不同,这里兜底再校验一次
if [[ "${#NUWAX_DEPS[@]}" -eq 0 ]]; then
  warn "awk 未匹到任何 nuwax-* path dep;回退 grep 兜底"
  mapfile -t NUWAX_DEPS < <(
    grep -oE 'nuwax-[a-z0-9_-]+' "$CARGO_TOML" | sort -u | head -50
  )
fi

if [[ "${#NUWAX_DEPS[@]}" -eq 0 ]]; then
  die "Cargo.toml 中找不到任何 nuwax-* path dep,请检查文件"
fi

log "识别到 ${#NUWAX_DEPS[@]} 个 nuwax-* path dep:"
printf '  - %s\n' "${NUWAX_DEPS[@]}"

# -----------------------------------------------------------------------------
# 2. 取上游 commit hash (rev-parse HEAD);失败兜底写 "unknown"
# -----------------------------------------------------------------------------
UPSTREAM_COMMIT="$(git -C "$UPSTREAM_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
log "upstream HEAD = $UPSTREAM_COMMIT"

NOW_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

# -----------------------------------------------------------------------------
# 3. 对每个 crate:复制源 + 比对,只有 src hash 变了才覆盖
# -----------------------------------------------------------------------------
copy_crate() {
  local name="$1"
  local src="$UPSTREAM_ROOT/crates/$name"
  local dst="$VENDOR_DIR/$name"

  [[ -d "$src" ]] || { warn "upstream 不存在,跳过: $name"; return 0; }

  # 计算源 hash(对全部内容的 size + sha256 拼)
  local src_hash
  src_hash="$(
    (cd "$src" && find . -type f -not -path './.git/*' -print0 \
       | LC_ALL=C sort -z \
       | xargs -0 sha256sum 2>/dev/null) \
      | sha256sum | awk '{print $1}'
  )"

  # 计算现 vendor hash
  local dst_hash=""
  if [[ -d "$dst" ]]; then
    dst_hash="$(
      (cd "$dst" && find . -type f -print0 \
         | LC_ALL=C sort -z \
         | xargs -0 sha256sum 2>/dev/null) \
        | sha256sum | awk '{print $1}'
    )" || dst_hash=""
  fi

  if [[ "$src_hash" == "$dst_hash" ]] && [[ -n "$dst_hash" ]]; then
    printf '  = %s (unchanged)\n' "$name"
    return 0
  fi

  rm -rf "$dst"
  mkdir -p "$dst"
  # 不复制 .git / target(若有)
  (cd "$src" && tar --exclude='.git' --exclude='target' -cf - .) \
    | (cd "$dst" && tar -xf -)
  printf '  + %s (updated, src=%s)\n' "$name" "${src_hash:0:12}"
}

log "开始同步 crate 源码到 vendor/ ..."
for name in "${NUWAX_DEPS[@]}"; do
  copy_crate "$name"
done

# -----------------------------------------------------------------------------
# 4. sed 改写 Cargo.toml:绝对路径 → vendor/<name>
# -----------------------------------------------------------------------------
log "改写 Cargo.toml 的 path = ..."
for name in "${NUWAX_DEPS[@]}"; do
  # 兼容两种写法:绝对路径 .../crates/<name> 与 已 vendor 化的 vendor/<name>
  # 优先把绝对路径替换为 vendor 路径
  sed -i "s|path = \"/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/${name}\"|path = \"vendor/${name}\"|g" \
    "$CARGO_TOML"
done

# 兜底:任何残留的 nuwax-workspace 绝对路径都替换成 vendor/<leaf>
if grep -q 'nuwax-workspace' "$CARGO_TOML"; then
  warn "Cargo.toml 中仍有 nuwax-workspace 字样,做最后一次兜底替换"
  sed -i 's|path = "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/\(nuwax-[a-z0-9_-]*\)"|path = "vendor/\1"|g' \
    "$CARGO_TOML"
fi

# -----------------------------------------------------------------------------
# 5. 写 vendor-config.json (manifest)
# -----------------------------------------------------------------------------
log "写 vendor-config.json ..."

python3 - "$UPSTREAM_COMMIT" "$NOW_UTC" "$VENDOR_CONFIG_JSON" "$VENDOR_DIR" \
  <<'PYEOF'
import json, os, sys, hashlib, datetime

upstream_commit, now_utc, manifest_path, vendor_dir = sys.argv[1:5]
vendor_dir = os.path.abspath(vendor_dir)

entries = []
for name in sorted(os.listdir(vendor_dir)):
    crate_dir = os.path.join(vendor_dir, name)
    if not os.path.isdir(crate_dir):
        continue
    size = 0
    file_count = 0
    h = hashlib.sha256()
    for root, _, files in os.walk(crate_dir):
        for fn in files:
            fp = os.path.join(root, fn)
            try:
                with open(fp, "rb") as f:
                    data = f.read()
                h.update(data)
                size += len(data)
                file_count += 1
            except OSError:
                pass
    digest = h.hexdigest()
    entries.append({
        "name": name,
        "path": f"vendor/{name}",
        "src_sha256": digest,
        "file_count": file_count,
        "size_bytes": size,
    })

manifest = {
    "schema": "nuwax-vendor-manifest/1.0",
    "generated_at_utc": now_utc,
    "upstream_repo": "/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace",
    "upstream_commit": upstream_commit,
    "target_machine": "aarch64 + Ubuntu 20.04 (Jetson)",
    "default_lcm_backend": "fake-lcm",
    "vendored_by": "scripts/vendor_dimos_crates.sh",
    "crates": entries,
}

with open(manifest_path, "w") as f:
    json.dump(manifest, f, indent=2, ensure_ascii=False)
    f.write("\n")
PYEOF

# -----------------------------------------------------------------------------
# 6. 写 .cargo/config.toml
#    此文件让 cargo 知道 vendor/ 是本地 source(可选,主要是为 cargo --offline
#    提供 source replacement 入口)。本文件**入 git**,所以写得最小、可解释。
# -----------------------------------------------------------------------------
log "写 .cargo/config.toml ..."
mkdir -p "$CARGO_CONFIG_DIR"
cat > "$CARGO_CONFIG_TOML" <<'EOF'
# 此文件由 scripts/vendor_dimos_crates.sh 生成,**入 git**。
# vendor 目录本身已在 .gitignore 排除,但此文件记录"vendor 模式已启用"。
#
# 之所以需要这个文件:
#   - cargo build --offline 时,如果 Cargo.toml 引用了本地 path deps,
#     且 vendor/ 也存在,cargo 会优先使用 vendor/ 中的源码。
#   - 配置 source replacement 把所有上游 crate 重定向到本地 vendor,
#     这样即使将来切回 [patch.git] / crates.io,也会有明确占位语义。
#
# 注意:不要在这里写 [source.<crate>] = { replace_with = "vendored-sources" },
# 那是 cargo vendor 的产物,本脚本不替 cargo vendor。

[net]
offline = false
EOF

# -----------------------------------------------------------------------------
# 7. 收口汇总
# -----------------------------------------------------------------------------
log "vendor 化完成。"
log "  vendor 目录: $VENDOR_DIR ($(du -sh "$VENDOR_DIR" 2>/dev/null | awk '{print $1}'))"
log "  manifest   : $VENDOR_CONFIG_JSON"
log "  cargo conf : $CARGO_CONFIG_TOML"
log ""
log "下一步(合并 PR 前必做):"
log "  1. 执行 \`cargo check --manifest-path $RUNTIME_DIR/Cargo.toml --locked\`"
log "  2. 执行 \`cargo test --manifest-path $RUNTIME_DIR/Cargo.toml --locked\`"
log "  3. 在目标机上同跑一遍确认 aarch64 编译通过"
log ""
log "若 dimos upstream \`nuwax-lcm-sys\` / \`nuwax-lcm\` wrapper 落地,"
log "切 NUWAX_LCM_BACKEND=real-lcm + 重新跑本脚本更新 vendor 即可。"

exit 0
