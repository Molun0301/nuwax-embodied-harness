#!/usr/bin/env bash
# 构建当前生产算法并启动本机导航模拟器；不连接或控制实机。
set -euo pipefail
simulator_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$simulator_root"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  exec python3 scripts/diagnostics/navigation_simulator.py --help
fi
build_only=false
if [[ "${1:-}" == "--build-only" ]]; then
  build_only=true
  shift
  if [[ "$#" != 0 ]]; then
    printf '仅构建模式不接受运行参数\n' >&2
    exit 2
  fi
fi
if [[ -f .nuwax-test-env/env.sh ]]; then
  source .nuwax-test-env/env.sh
else
  export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$simulator_root/runtime_data/navigation_simulator_build}"
fi
export CARGO_BUILD_JOBS="$(python3 "$simulator_root/scripts/build_jobs.py")"
printf '构建并行预算：%s\n' "$CARGO_BUILD_JOBS"
cargo build --release --locked --manifest-path scripts/diagnostics/navigation_simulator_native/Cargo.toml --bin nuwax-navigation-simulator
if [[ "$build_only" == true ]]; then
  printf '导航程序：%s/release/nuwax-navigation-simulator\n' "$CARGO_TARGET_DIR"
  exit 0
fi
exec python3 scripts/diagnostics/navigation_simulator.py \
  --binary "$CARGO_TARGET_DIR/release/nuwax-navigation-simulator" "$@"
