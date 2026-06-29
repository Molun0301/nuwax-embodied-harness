#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${PUPPYPI_ROS2_CONTAINER:-puppypi_ros2}"
MAP_NAME="map_01"
ENABLE_CMD_VEL_ADAPTER="${ENABLE_CMD_VEL_ADAPTER:-false}"

for arg in "$@"; do
  case "${arg}" in
    map:=*) MAP_NAME="${arg#map:=}" ;;
  esac
done

docker start "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "[stack] 1/4 启动 PuppyPi 导航 (map=${MAP_NAME})..."
docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  if [[ -x /home/ubuntu/.stop_ros.sh ]]; then
    /home/ubuntu/.stop_ros.sh || true
  fi
  ros2 launch navigation navigation.launch.py map:=${MAP_NAME}
"

sleep 12

echo "[stack] 2/4 启动 nuwax MQTT bridge..."
docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  ros2 run nuwax_bridge_py mqtt_bridge_node
"

echo "[stack] 3/4 启动 PuppyPi action bridge..."
docker exec -d "${CONTAINER_NAME}" bash -lc "
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  ros2 run nuwax_puppypi_adapter action_bridge
"

if [[ \"${ENABLE_CMD_VEL_ADAPTER}\" == \"true\" ]]; then
  echo "[stack] 可选: 启动 cmd_vel -> velocity 适配器..."
  docker exec -d "${CONTAINER_NAME}" bash -lc "
    source /opt/ros/humble/setup.bash
    source /home/ubuntu/ros2_ws/install/setup.bash
    ros2 launch nuwax_puppypi_adapter cmd_vel_adapter.launch.py
  "
else
  echo "[stack] 跳过 cmd_vel 适配器（官方 navigation.launch 已接 puppy_control）。"
  echo "       如需手动接管 /cmd_vel，请设置 ENABLE_CMD_VEL_ADAPTER=true。"
fi

echo "[stack] 4/4 启动宿主机 nuwax 网关..."
if [[ -f \"${REPO_ROOT}/.env\" ]]; then
  nohup bash \"${REPO_ROOT}/scripts/start_gateway.sh\" > /tmp/nuwax_puppypi_gateway.log 2>&1 &
else
  echo \"未找到 ${REPO_ROOT}/.env，请先运行 scripts/deploy_puppypi.sh\" >&2
  exit 1
fi

sleep 3
echo
echo "栈已启动。检查:"
echo "  curl http://127.0.0.1:8766/api/health"
echo "  docker exec ${CONTAINER_NAME} bash -lc 'source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 node list'"
echo "网关日志: /tmp/nuwax_puppypi_gateway.log"
