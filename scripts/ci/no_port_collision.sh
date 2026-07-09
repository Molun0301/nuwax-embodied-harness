#!/usr/bin/env bash
# scripts/ci/no_port_collision.sh
# 2025-07-08 v5:R8-M-2 独立 CI gate,7667 硬编码只允许在 port_utils.py
set -uo pipefail
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"
HITS=$(rg -n '\b7667\b' --glob '*.py' --glob '*.sh' services/ gateways/ scripts/ core/ 2>/dev/null | grep -v 'port_utils.py' | grep -v 'no_port_collision.sh' | grep -v 'V0_preflight.sh' || true)
if [[ -n "${HITS}" ]]; then
    echo "FAIL: 7667 硬编码残留(除 port_utils.py / no_port_collision.sh / V0_preflight.sh):"
    echo "${HITS}"
    exit 1
fi
echo "OK: 无 7667 硬编码残留"
exit 0