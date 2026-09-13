#!/usr/bin/env bash
# 构建独立二维算法；可使用系统依赖或显式指定的本机依赖前缀。
set -euo pipefail
NUWAX_SLAM_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
NUWAX_SLAM_DEPS="$NUWAX_SLAM_REPO/runtime_data/build_dependencies/slam_toolbox/sysroot/usr"
NUWAX_SLAM_CARGO="$(command -v cargo || true)"
if [[ -z "$NUWAX_SLAM_CARGO" && -x "$HOME/.cargo/bin/cargo" ]]; then
  NUWAX_SLAM_CARGO="$HOME/.cargo/bin/cargo"
fi
if [[ -z "$NUWAX_SLAM_CARGO" ]]; then
  echo "缺少Rust构建工具cargo" >&2
  exit 1
fi
if [[ -d "$NUWAX_SLAM_DEPS" && -z "${CMAKE_TOOLCHAIN_FILE:-}" && -z "${CARGO_BUILD_TARGET:-}" ]]; then
  export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}$NUWAX_SLAM_DEPS"
  NUWAX_SLAM_MULTIARCH="$(gcc -dumpmachine)"
  export LD_LIBRARY_PATH="$NUWAX_SLAM_DEPS/lib/$NUWAX_SLAM_MULTIARCH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec "$NUWAX_SLAM_CARGO" build --manifest-path "$NUWAX_SLAM_REPO/services/nuwax-workspace/Cargo.toml" \
  -p nuwax-slam-toolbox -p nuwax-slam-2d-runtime --features nuwax-slam-2d-runtime/worker --release "$@"
