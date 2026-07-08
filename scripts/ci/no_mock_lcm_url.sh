#!/usr/bin/env bash
# scripts/ci/no_mock_lcm_url.sh
# 2026-07-08 v5:R8-M-2 独立 CI gate,只允许 _resolve_backend reject 信息含 'mock://'
set -uo pipefail
REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"
HITS=$(rg -n 'mock://' --glob '*.py' services/ gateways/ core/ 2>/dev/null | grep -v '_resolve_backend' | grep -v 'test_lcm_motion_command_bridge.py' | grep -v 'feedback' || true)
if [[ -n "${HITS}" ]]; then
    echo "FAIL: mock:// 硬编码残留(除 _resolve_backend reject 信息外):"
    echo "${HITS}"
    exit 1
fi
echo "OK: 无 mock:// 硬编码残留"
exit 0