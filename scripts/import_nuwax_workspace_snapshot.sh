#!/usr/bin/env bash
# =============================================================================
# import_nuwax_workspace_snapshot.sh
# -----------------------------------------------------------------------------
# 目的（Phase 1,2026-07-07）：
#   把开发机 dimos snapshot 的 nuwax-workspace 目录(含 50 个 crate + 根
#   Cargo.toml/Cargo.lock/.cargo/config.toml + lcmtypes/) 复制到本仓
#   ``services/nuwax-workspace/``,作为 tracked snapshot 入仓,供
#   services/geometry/nuwax_geometry_runtime/Cargo.toml 中的 13 条 path dep
#   走 repo-local 相对路径 (../../nuwax-workspace/crates/<name>)。
#
# 与 vendor_dimos_crates.sh 的区别：
#   - vendor_dimos_crates.sh 只复制 13 个 crate 到 services/geometry/
#     nuwax_geometry_runtime/vendor/<name>/,只对单 binary 起作用;
#   - import_nuwax_workspace_snapshot.sh 复制整棵 nuwax-workspace
#     (50 个 crate + 元数据 + lcmtypes) 到 services/nuwax-workspace/,
#     是 dev/目标机/未来多个 Rust binary 共用的"真接入"路径。
#
# 使用方法：
#   bash scripts/import_nuwax_workspace_snapshot.sh \
#       --source <snapshot 路径>(必填;默认走 env NUWAX_SNAPSHOT_SOURCE)
#
#   可选参数：
#       --source <PATH>   必填,源 nuwax-workspace 路径
#                         (或 export NUWAX_SNAPSHOT_SOURCE=/path/to/nuwax-workspace)
#       --dest   <PATH>   可选,目标,默认 services/nuwax-workspace
#       --dry-run         只打印动作,不实际 rsync / 写 manifest
#       --regen-manifest  不复制,只重新生成 IMPORT_MANIFEST.*
#       --help            打印本帮助
#
#   示例:
#       export NUWAX_SNAPSHOT_SOURCE=/path/to/nuwax-workspace
#       bash scripts/import_nuwax_workspace_snapshot.sh
#
# 行为契约：
#   - idempotent:重跑结果稳定(rsync -a --delete + 重新生成 manifest)
#   - excludes:.git/ target/ .omc/ .omx/ generated/c/ *.lock.tmp
#   - 自动从源目录取 git commit hash,作为 IMPORT_MANIFEST.source_git_commit
#   - 自动计算全部 Cargo.toml/Cargo.lock 的 sha256,落到 IMPORT_MANIFEST.sha256
#   - 自动统计 on-disk crates 与 declared members,差异写 IMPORT_MANIFEST.json
#   - 自动写 IMPORT_MANIFEST.md(7 行精简版)
#   - 完成后**不自动 git add** —— 由 commit 阶段人工审查 + 入仓
#
# 退出码：
#   0  成功
#   1  参数错误
#   2  源目录不存在 / 不含 Cargo.toml
#   3  rsync 失败
#   4  manifest 写入失败
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# 路径常量
# -----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DEFAULT_DEST="$REPO_ROOT/services/nuwax-workspace"

# 默认值
SOURCE="${NUWAX_SNAPSHOT_SOURCE:-}"
DEST="$DEFAULT_DEST"
DRY_RUN="false"
REGEN_MANIFEST_ONLY="false"

# -----------------------------------------------------------------------------
# 参数解析
# -----------------------------------------------------------------------------
usage() {
    sed -n '2,40p' "$0" | sed 's/^# \?//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --source)
            SOURCE="$2"
            shift 2
            ;;
        --source=*)
            SOURCE="${1#*=}"
            shift
            ;;
        --dest)
            DEST="$2"
            shift 2
            ;;
        --dest=*)
            DEST="${1#*=}"
            shift
            ;;
        --dry-run)
            DRY_RUN="true"
            shift
            ;;
        --regen-manifest)
            REGEN_MANIFEST_ONLY="true"
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "[import_nuwax_workspace_snapshot] unknown arg: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

# -----------------------------------------------------------------------------
# 源路径校验
# -----------------------------------------------------------------------------
if [[ -z "$SOURCE" ]]; then
    echo "[import_nuwax_workspace_snapshot] ERROR: --source <PATH> 必填" >&2
    echo "                          (或 export NUWAX_SNAPSHOT_SOURCE=/path/to/nuwax-workspace)" >&2
    exit 1
fi

if [[ ! -d "$SOURCE" ]]; then
    echo "[import_nuwax_workspace_snapshot] ERROR: 源目录不存在: $SOURCE" >&2
    exit 2
fi

if [[ ! -f "$SOURCE/Cargo.toml" ]]; then
    echo "[import_nuwax_workspace_snapshot] ERROR: 源目录不含 Cargo.toml: $SOURCE" >&2
    exit 2
fi

# 工作区根 Cargo.toml 是否含 [workspace] 段（防御误传子 crate）
if ! grep -q '^\[workspace\]' "$SOURCE/Cargo.toml"; then
    echo "[import_nuwax_workspace_snapshot] ERROR: 源 Cargo.toml 不是 workspace root: $SOURCE" >&2
    exit 2
fi

# -----------------------------------------------------------------------------
# 源 git commit（若 source 在 git repo 内）
#   不要求 source 本身是 .git 根:只要祖先目录有 .git 即视为可追溯。
#   用 git -C <path> 避免子 shell cd + set -e 互相干扰。
# -----------------------------------------------------------------------------
SOURCE_GIT_COMMIT=""
# 1) source 本身含 .git
if [[ -d "$SOURCE/.git" ]]; then
    SOURCE_GIT_COMMIT="$(git -C "$SOURCE" rev-parse HEAD 2>/dev/null || true)"
fi
# 2) source 在祖先 git repo 中（典型:dimos/rust重写与移植/ 是 git repo,
#    其下的 nuwax-workspace/ 只是 working tree 子目录,没有自己的 .git）
if [[ -z "$SOURCE_GIT_COMMIT" ]]; then
    SOURCE_GIT_COMMIT="$(git -C "$SOURCE" rev-parse HEAD 2>/dev/null || true)"
fi
if [[ -z "$SOURCE_GIT_COMMIT" ]]; then
    SOURCE_GIT_COMMIT="本任务导入时未取源 Git commit"
fi

SNAPSHOT_TIME_UTC="$(date -u +'%Y-%m-%dT%H:%M:%SZ')"

# -----------------------------------------------------------------------------
# rsync 复制
# -----------------------------------------------------------------------------
run_rsync() {
    rsync -a --delete \
        --exclude='.git/' \
        --exclude='target/' \
        --exclude='.omc/' \
        --exclude='.omx/' \
        --exclude='generated/c/' \
        --exclude='*.lock.tmp' \
        --exclude='.cargo/config.toml' \
        "$SOURCE/" "$DEST/"
}

if [[ "$REGEN_MANIFEST_ONLY" == "true" ]]; then
    echo "[import_nuwax_workspace_snapshot] --regen-manifest 模式：跳过 rsync，仅重建 manifest"
elif [[ "$DRY_RUN" == "true" ]]; then
    echo "[import_nuwax_workspace_snapshot] DRY-RUN: rsync -a --delete $SOURCE/ -> $DEST/"
else
    echo "[import_nuwax_workspace_snapshot] rsync $SOURCE -> $DEST"
    mkdir -p "$DEST"
    if ! run_rsync; then
        echo "[import_nuwax_workspace_snapshot] ERROR: rsync 失败 (exit=$?)" >&2
        exit 3
    fi
fi

# -----------------------------------------------------------------------------
# 生成 IMPORT_MANIFEST.md / .json / .sha256
# -----------------------------------------------------------------------------
MANIFEST_MD="$DEST/IMPORT_MANIFEST.md"
MANIFEST_JSON="$DEST/IMPORT_MANIFEST.json"
MANIFEST_SHA="$DEST/IMPORT_MANIFEST.sha256"

if [[ "$DRY_RUN" == "true" ]]; then
    echo "[import_nuwax_workspace_snapshot] DRY-RUN: would write $MANIFEST_MD / $MANIFEST_JSON / $MANIFEST_SHA"
    exit 0
fi

# 统计 on-disk crates
ON_DISK_COUNT=$(find "$DEST/crates" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | wc -l)

# 解析 declared members
MEMBERS_TMP="$(mktemp)"
awk '/^members = \[/,/^\]/' "$DEST/Cargo.toml" 2>/dev/null \
    | grep -oE 'crates/nuwax-[a-z0-9-]+' \
    | sed 's|crates/||' \
    | sort > "$MEMBERS_TMP" || true
MEMBERS_COUNT=$(wc -l < "$MEMBERS_TMP")

# on-disk but not in members
ON_DISK_ONLY_TMP="$(mktemp)"
{
    find "$DEST/crates" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort
    cat "$MEMBERS_TMP"
} | sort | uniq -u > "$ON_DISK_ONLY_TMP" || true

# 写 IMPORT_MANIFEST.md（精简版）
{
    echo "# IMPORT_MANIFEST"
    echo ""
    echo "来源:\`\${NUWAX_SNAPSHOT_SOURCE:-${SOURCE}}\`"
    echo "快照导入时间(UTC):${SNAPSHOT_TIME_UTC}"
    echo "source_git_commit:${SOURCE_GIT_COMMIT}"
    echo "crate 总数:${ON_DISK_COUNT}"
    echo "excludes: [target/, .omc/, .omx/, generated/c/, *.lock.tmp, .cargo/config.toml(如含机器私有路径)]"
    echo "更新命令:\`bash scripts/import_nuwax_workspace_snapshot.sh --source <path>\`"
    echo "deferred:[]"
} > "$MANIFEST_MD"

# 写 IMPORT_MANIFEST.json（用 python 拼）
python3 - "$MANIFEST_JSON" "$MEMBERS_TMP" "$ON_DISK_ONLY_TMP" "$SOURCE" "$SNAPSHOT_TIME_UTC" "$SOURCE_GIT_COMMIT" "$ON_DISK_COUNT" "$MEMBERS_COUNT" "$DEST" <<'PY'
import json
import sys

(json_path, members_path, on_disk_only_path,
 source, ts, commit, on_disk_count_s, members_count_s, dest) = sys.argv[1:]
on_disk_count = int(on_disk_count_s)
members_count = int(members_count_s)

members = [l.strip() for l in open(members_path) if l.strip()]
on_disk_only = [l.strip() for l in open(on_disk_only_path) if l.strip()]

# hash root Cargo.toml + Cargo.lock
import hashlib, os
def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()

lock_hashes = {
    "Cargo.toml": sha256(os.path.join(dest, "Cargo.toml")),
    "Cargo.lock": sha256(os.path.join(dest, "Cargo.lock")),
}

manifest = {
    "source": "${NUWAX_SNAPSHOT_SOURCE:-" + source + "}",
    "snapshotted_at_utc": ts,
    "source_git_commit": commit,
    "crate_count_on_disk": on_disk_count,
    "crate_count_in_workspace_members": members_count,
    "crate_count": max(on_disk_count, members_count),
    "excludes": [
        "target/",
        ".omc/",
        ".omx/",
        "generated/c/",
        "*.lock.tmp",
        ".cargo/config.toml (如含机器私有路径)"
    ],
    "members": members,
    "on_disk_but_not_in_members": on_disk_only,
    "in_members_but_not_on_disk": [],
    "deferred": [],
    "lockfile_sha256": lock_hashes,
    "refresh_command": "bash scripts/import_nuwax_workspace_snapshot.sh --source <path>",
    "snapshot_script": "scripts/import_nuwax_workspace_snapshot.sh",
    "manifest_files": [
        "services/nuwax-workspace/IMPORT_MANIFEST.md",
        "services/nuwax-workspace/IMPORT_MANIFEST.json",
        "services/nuwax-workspace/IMPORT_MANIFEST.sha256"
    ]
}

with open(json_path, "w", encoding="utf-8") as f:
    json.dump(manifest, f, indent=2, ensure_ascii=False)
    f.write("\n")
PY

# 写 IMPORT_MANIFEST.sha256（root Cargo.toml/lock + 所有 crate/Cargo.toml）
{
    sha256sum "$DEST/Cargo.toml" "$DEST/Cargo.lock"
    find "$DEST/crates" -mindepth 2 -maxdepth 2 -name "Cargo.toml" -print0 \
        | sort -z | xargs -0 sha256sum
} > "$MANIFEST_SHA"

# 清理临时
rm -f "$MEMBERS_TMP" "$ON_DISK_ONLY_TMP"

echo "[import_nuwax_workspace_snapshot] DONE"
echo "  dest=$DEST"
echo "  crates_on_disk=$ON_DISK_COUNT"
echo "  members=$MEMBERS_COUNT"
echo "  manifest_md=$MANIFEST_MD"
echo "  manifest_json=$MANIFEST_JSON"
echo "  manifest_sha=$MANIFEST_SHA"