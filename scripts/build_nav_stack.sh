#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

exec cargo build \
  --manifest-path "${REPO_ROOT}/services/nuwax-workspace/Cargo.toml" \
  --release \
  -p nuwax-nav-stack \
  --features simple-2d,replanning-astar
