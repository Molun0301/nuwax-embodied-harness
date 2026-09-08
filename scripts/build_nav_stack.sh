#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cargo +1.88.0 build \
  --manifest-path "${REPO_ROOT}/services/nuwax-workspace/Cargo.toml" \
  --release \
  -p nuwax-nav-stack \
  --features simple-2d,replanning-astar,mls

BINARY="${REPO_ROOT}/services/nuwax-workspace/target/release/nuwax-nav-stack"
EXPECTED_CONTRACT="nuwax-nav-stack-internal-ready-v1"
ACTUAL_CONTRACT="$("${BINARY}" --launcher-contract)"
[[ "${ACTUAL_CONTRACT}" == "${EXPECTED_CONTRACT}" ]] || {
  echo "nuwax-nav-stack build produced an incompatible launcher contract: ${ACTUAL_CONTRACT:-<empty>}" >&2
  exit 1
}
