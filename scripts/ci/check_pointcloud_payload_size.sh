#!/usr/bin/env bash
set -euo pipefail
# R-N14-8.0: 点云 payload VmRSS 监控(stub)
# 实测阶段:订阅 /go2/lidar/points,采样 100 帧,断言 VmRSS < 4 MiB cap
EXPECTED_VMRSS_CAP_BYTES=$((4 * 1024 * 1024))
echo "[check_pointcloud_payload_size] stub OK — cap_bytes=${EXPECTED_VMRSS_CAP_BYTES}"
exit 0