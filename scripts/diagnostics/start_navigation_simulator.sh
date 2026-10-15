#!/usr/bin/env bash
# 构建当前生产算法并启动本机导航模拟器；不连接或控制实机。
set -euo pipefail
simulator_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$simulator_root"
if [[ -f .nuwax-test-env/env.sh ]]; then
  source .nuwax-test-env/env.sh
else
  export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$simulator_root/runtime_data/navigation_simulator_build}"
fi
export CARGO_BUILD_JOBS="$(python3 "$simulator_root/scripts/build_jobs.py")"
printf '构建并行预算：%s\n' "$CARGO_BUILD_JOBS"
cargo build --release --locked --manifest-path scripts/diagnostics/navigation_simulator_native/Cargo.toml
exec python3 scripts/diagnostics/navigation_simulator.py \
  --binary "$CARGO_TARGET_DIR/release/nuwax-navigation-simulator" "$@"
