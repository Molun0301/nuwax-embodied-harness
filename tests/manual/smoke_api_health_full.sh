#!/usr/bin/env bash
set -euo pipefail
# R-N14-8.0: /api/health/full 端到端冒烟(stub)
EXPECTED_SCHEMA_FIELDS=("geometry_runtime" "lcm_backend" "connected_channels" "nuwax_nav_stack")
echo "[smoke_api_health_full] stub OK — schema=${EXPECTED_SCHEMA_FIELDS[*]}"
exit 0