#!/usr/bin/env bash
# PuppyPi Phase 1 路 A：在 puppypi_ros2 容器内启动 motion_proxy + video_proxy。
# 前提：宿主机已 start_gateway.sh，且 /tmp/nuwax_video.sock 已存在。

set -euo pipefail

CONTAINER_NAME="${PUPPYPI_ROS2_CONTAINER:-puppypi_ros2}"
IMAGE_TOPIC="${PUPPYPI_IMAGE_TOPIC:?请设置 PUPPYPI_IMAGE_TOPIC，例如 /usb_cam/image_raw}"
UDS_PATH="${NUWAX_VIDEO_UDS_PATH:-/tmp/nuwax_video.sock}"
PROXY_DIR="${PUPPYPI_PLATFORM_BRIDGE_DIR:-/opt/nuwax/platform_bridge}"

if [[ ! -S "${UDS_PATH}" ]]; then
  echo "错误: ${UDS_PATH} 不存在。请先启动宿主机网关（Rust 桥）。" >&2
  exit 1
fi

echo "[puppypi] 同步 platform_bridge 到容器..."
docker exec "${CONTAINER_NAME}" mkdir -p "${PROXY_DIR}" 2>/dev/null || true
docker cp "$(dirname "$0")/video_proxy.py" "${CONTAINER_NAME}:${PROXY_DIR}/video_proxy.py"
docker cp "$(dirname "$0")/motion_proxy.py" "${CONTAINER_NAME}:${PROXY_DIR}/motion_proxy.py"
docker cp "$(dirname "$0")/video_uds_sender.py" "${CONTAINER_NAME}:${PROXY_DIR}/video_uds_sender.py"

echo "[puppypi] 启动 motion_proxy :8090..."
docker exec -d "${CONTAINER_NAME}" bash -lc \
  "PUPPYPI_PROXY_PORT=8090 python3 ${PROXY_DIR}/motion_proxy.py"

echo "[puppypi] 启动 video_proxy -> ${UDS_PATH}..."
docker exec -d "${CONTAINER_NAME}" bash -lc \
  "PUPPYPI_IMAGE_TOPIC=${IMAGE_TOPIC} NUWAX_VIDEO_UDS_PATH=${UDS_PATH} python3 ${PROXY_DIR}/video_proxy.py"

echo "[puppypi] proxy 已后台启动。检查:"
echo "  docker exec ${CONTAINER_NAME} pgrep -af 'motion_proxy|video_proxy'"
echo "  curl http://127.0.0.1:8766/api/health"
