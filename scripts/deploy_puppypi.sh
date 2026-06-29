#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${PUPPYPI_ROS2_CONTAINER:-puppypi_ros2}"
NUWAX_SAVED_KEY="${NUWAX_SAVED_KEY:-300234dc675e457e9db7eef0aa906a76}"
MAP_NAME="${PUPPYPI_MAP:-map_01}"

echo "[deploy] repo: ${REPO_ROOT}"
echo "[deploy] container: ${CONTAINER_NAME}"

if ! docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "容器 ${CONTAINER_NAME} 不存在" >&2
  exit 1
fi

docker start "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "[deploy] 同步 nuwax_puppypi_adapter 到容器..."
docker exec "${CONTAINER_NAME}" mkdir -p /home/ubuntu/ros2_ws/src
docker cp "${REPO_ROOT}/deploy/puppypi/ros2_ws/src/nuwax_puppypi_adapter" \
  "${CONTAINER_NAME}:/home/ubuntu/ros2_ws/src/nuwax_puppypi_adapter"

echo "[deploy] 容器内 colcon build..."
docker exec "${CONTAINER_NAME}" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  cd /home/ubuntu/ros2_ws
  colcon build --packages-select nuwax_puppypi_adapter --symlink-install
'

echo "[deploy] 宿主机 Python 虚拟环境（无 unitree_sdk2py）..."
if [[ ! -d "${REPO_ROOT}/.venv" ]]; then
  python3 -m venv "${REPO_ROOT}/.venv"
fi
"${REPO_ROOT}/.venv/bin/python" -m pip install --upgrade pip
"${REPO_ROOT}/.venv/bin/python" -m pip install -r "${REPO_ROOT}/requirements-puppypi.txt"

mkdir -p "${REPO_ROOT}/runtime_data/memory" "${REPO_ROOT}/artifacts" "${REPO_ROOT}/runtime_data/maps"

echo "[deploy] 生成 MCP 配置..."
"${REPO_ROOT}/.venv/bin/python" "${REPO_ROOT}/scripts/bootstrap.py" apply --non-interactive \
  --robot customrobot \
  --tier lite \
  --no-geometry \
  --no-data-plane \
  --memory-mode disabled \
  --perception-mode disabled \
  --mcp-mode config-only \
  --no-install-python \
  --no-prepare-yolo \
  --no-build-geometry \
  --no-install-geometry-deps

if [[ -n "${NUWAX_SAVED_KEY}" ]]; then
  echo "[deploy] 安装 NuwaxClaw Docker 客户端..."
  "${REPO_ROOT}/.venv/bin/python" "${REPO_ROOT}/scripts/bootstrap.py" apply --non-interactive \
    --robot customrobot \
    --tier lite \
    --no-geometry \
    --no-data-plane \
    --memory-mode disabled \
    --perception-mode disabled \
    --mcp-mode skip \
    --no-install-python \
    --install-nuwaclaw-docker \
    --nuwaclaw-host-mount ro \
    --nuwaclaw-saved-key "${NUWAX_SAVED_KEY}"
fi

cat > "${REPO_ROOT}/.env" <<EOF
NUWAX_ACTIVE_ROBOT=customrobot
NUWAX_PYTHON_BIN=${REPO_ROOT}/.venv/bin/python
NUWAX_GEOMETRY_ENABLED=false
NUWAX_GEOMETRY_REQUIRE_ROS2=false
NUWAX_GATEWAY_HOST=0.0.0.0
NUWAX_GATEWAY_PORT=8766
NUWAX_GATEWAY_PUBLIC_BASE_URL=http://host.docker.internal:8766
NUWAX_PERCEPTION_STREAM_ENABLED=false
NUWAX_PERCEPTION_YOLO_WORLD_ENABLED=false
NUWAX_MEMORY_DB_PATH=${REPO_ROOT}/runtime_data/memory/vector_memory.db
NUWAX_LOG_DIR=${REPO_ROOT}
NUWAX_LOG_FILE=${REPO_ROOT}/nuwax_robot_bridge.log
NUWAX_GATEWAY_ARTIFACT_DIR=${REPO_ROOT}/artifacts
NUWAX_MAP_CATALOG_ROOT=${REPO_ROOT}/runtime_data/maps
EOF

echo
echo "部署完成。"
echo "启动整套栈: bash ${REPO_ROOT}/scripts/start_puppypi_stack.sh map:=${MAP_NAME}"
echo "仅启动网关: bash ${REPO_ROOT}/scripts/start_gateway.sh"
