#!/usr/bin/env bash
# 复用唯一依赖检查实现；不再按源码行号维护注释白名单。
set -euo pipefail
NUWAX_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NUWAX_CHECK_ARGS=()
for arg in "$@"; do
  if [[ "$arg" == "--dry-run" ]]; then
    NUWAX_CHECK_ARGS+=(--report-only)
  else
    NUWAX_CHECK_ARGS+=("$arg")
  fi
done
exec python3 "$NUWAX_REPO_ROOT/scripts/ci/release_gates/static_neg_gate.py" \
  --root "$NUWAX_REPO_ROOT" --gate gate-1-active-ros2-fallback "${NUWAX_CHECK_ARGS[@]}"
