#!/usr/bin/env bash
set -euo pipefail
# R-N14-8.0: telemetry alias 计数验证(stub)
# 实测阶段:读取 runtime_data/.legacy_alias_counters.json,断言旧键命中计数
COUNTER_FILE="runtime_data/.legacy_alias_counters.json"
echo "[check_alias_telemetry] stub OK — counter_file=${COUNTER_FILE}"
exit 0