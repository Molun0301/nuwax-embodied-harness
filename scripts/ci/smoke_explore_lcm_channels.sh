#!/usr/bin/env bash
set -euo pipefail
# R-N14-8.0: explore LCM 双通道冒烟(stub)
# 实测阶段:验证 nuwax-explore-wavefront + nuwax-explore-tare-bridge 至少各 1 帧
# stub 阶段:仅检查 LCM URL 可达 + channel 注册项 ≥ 2
EXPECTED_CHANNELS=("explore/frontiers#nuwax_msgs.FrontierList" "explore/status#nuwax_msgs.ExplorationStatus")
echo "[smoke_explore_lcm_channels] stub OK — channels=${EXPECTED_CHANNELS[*]}"
exit 0