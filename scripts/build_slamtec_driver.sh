#!/usr/bin/env bash
# 构建无ROS依赖的思岚雷达驱动动态库。
set -euo pipefail
NUWAX_SLAMTEC_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
NUWAX_SLAMTEC_BUILD="$NUWAX_SLAMTEC_REPO/runtime_data/build_dependencies/slamtec/native-build"
NUWAX_SLAMTEC_ARGS=()
if [[ -n "${CMAKE_TOOLCHAIN_FILE:-}" ]]; then
  NUWAX_SLAMTEC_ARGS+=("-DCMAKE_TOOLCHAIN_FILE=$CMAKE_TOOLCHAIN_FILE")
fi
cmake -S "$NUWAX_SLAMTEC_REPO/drivers/sensors/slamtec_native" -B "$NUWAX_SLAMTEC_BUILD" \
  -DCMAKE_BUILD_TYPE=Release "${NUWAX_SLAMTEC_ARGS[@]}" "$@"
cmake --build "$NUWAX_SLAMTEC_BUILD" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
