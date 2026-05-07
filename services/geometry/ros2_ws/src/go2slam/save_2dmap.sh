#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="${ROOT_DIR}/maps"

source /opt/ros/jazzy/setup.bash
source "${ROOT_DIR}/install/setup.bash"

mkdir -p "${MAP_DIR}"

echo "Saving /map to ${MAP_DIR}/map.yaml and ${MAP_DIR}/map.pgm"

ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/map"
