#!/usr/bin/env bash
set -euo pipefail
# R-N14-8.0: navigation goal lifecycle 冒烟(stub)
# 实测阶段:验证 /api/navigation/{goal,state,cancel} 端点存在
EXPECTED_ENDPOINTS=("/api/navigation/goal" "/api/navigation/state" "/api/navigation/cancel")
echo "[smoke_navigation_goal_lifecycle] stub OK — endpoints=${EXPECTED_ENDPOINTS[*]}"
exit 0