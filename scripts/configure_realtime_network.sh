#!/usr/bin/env bash
set -euo pipefail

# 高频惯性和点云共用组播接收器，内核缓冲须能容纳一次集中到达的消息。
# 保留宿主机已有的更大配置；修改默认值只影响之后创建的套接字。
config_path="${1:-/etc/sysctl.d/90-nuwax-realtime.conf}"
minimum_bytes=8388608
receive_max="$(sysctl -n net.core.rmem_max)"
receive_default="$(sysctl -n net.core.rmem_default)"
(( receive_max < minimum_bytes )) && receive_max="${minimum_bytes}"
(( receive_default < minimum_bytes )) && receive_default="${minimum_bytes}"

install -d -m 0755 "$(dirname "${config_path}")"
printf '# 机器人实时通信接收缓冲，由项目安装器维护。\nnet.core.rmem_max=%s\nnet.core.rmem_default=%s\n' \
  "${receive_max}" "${receive_default}" > "${config_path}"
chmod 0644 "${config_path}"
sysctl -p "${config_path}"
