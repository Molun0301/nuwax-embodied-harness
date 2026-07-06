#!/usr/bin/env bash
# scripts/ci/smoke_real_lcm_failfast.sh
#
# Phase 1 fail-fast smoke 守门:
#   验证 nuwax-geometry-runtime 在 NUWAX_LCM_BACKEND=real-lcm 启动时
#   拒绝启动、不 fallback fake-lcm,并把 LCM_BACKEND_UNAVAILABLE 关键字
#   输出到 stderr。
#
# 退出码:
#   0  - 验收通过(real-lcm 启动期被拒,stderr 含 LCM_BACKEND_UNAVAILABLE)
#   1  - 验收失败(binary 不在 / 启动未拒绝 / 输出缺关键字)
#
# 失败模式:
#   - binary 缺 → 提前 exit 1,提示先 cargo build
#   - binary 启动但未拒 → exit 1
#   - binary 拒但 stderr 缺 LCM_BACKEND_UNAVAILABLE → exit 1

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# --- 1. locate binary (按 CARGO_TARGET_DIR / .nuwax-test-env 常见路径查找) ---
# Phase 0 已确认 binary 存在;Phase 1 额外需要在 .nuwax-test-env 路径下查找
# (因为测试 env 强制使用 CARGO_TARGET_DIR=$NUWAX_TEST_ENV_ROOT/target)。
find_binary() {
  local candidates=(
    "${CARGO_TARGET_DIR:-}/release/nuwax-geometry-runtime"
    "${CARGO_TARGET_DIR:-}/debug/nuwax-geometry-runtime"
    "${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/release/nuwax-geometry-runtime"
    "${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/target/debug/nuwax-geometry-runtime"
    "/home/ml/Deeprob/dimos/.nuwax-test-env/target/release/nuwax-geometry-runtime"
    "/home/ml/Deeprob/dimos/.nuwax-test-env/target/debug/nuwax-geometry-runtime"
  )
  for cand in "${candidates[@]}"; do
    if [[ -x "${cand}" ]]; then
      echo "${cand}"
      return 0
    fi
  done
  return 1
}

BINARY="$(find_binary || true)"
if [[ -z "${BINARY}" ]]; then
  echo "ERROR: nuwax-geometry-runtime binary not built; please run:" >&2
  echo "  source /home/ml/Deeprob/dimos/.nuwax-test-env/env.sh && \\" >&2
  echo "  cargo +1.88.0 build --manifest-path ${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/Cargo.toml --locked" >&2
  exit 1
fi
echo "==> using binary: ${BINARY}"

# --- 2. 启动 real-lcm,捕获 stderr ---
TMP_OUT="$(mktemp)"
TMP_ERR="$(mktemp)"
trap 'rm -f "${TMP_OUT}" "${TMP_ERR}"' EXIT

# env-only 启动契约:只 export env,不传 CLI args。
# 用未占用 port + 临时 ready_file,避免污染生产路径。
NUWAX_LCM_BACKEND=real-lcm \
NUWAX_GEOMETRY_RUNTIME_PORT=59999 \
NUWAX_GEOMETRY_RUNTIME_READY_FILE="$(mktemp -u)" \
  "${BINARY}" >"${TMP_OUT}" 2>"${TMP_ERR}" &
BIN_PID=$!

# --- 3. 等进程退出 ---
EXIT_CODE=0
wait "${BIN_PID}" || EXIT_CODE=$?

# --- 4. 校验:exit 非 0 + stderr 含 LCM_BACKEND_UNAVAILABLE ---
if [[ "${EXIT_CODE}" -eq 0 ]]; then
  echo "FAIL: binary 退出码 0,期望 fail-fast 非零退出" >&2
  echo "--- stdout ---" >&2
  cat "${TMP_OUT}" >&2 || true
  echo "--- stderr ---" >&2
  cat "${TMP_ERR}" >&2 || true
  exit 1
fi

if ! grep -q "LCM_BACKEND_UNAVAILABLE" "${TMP_ERR}"; then
  echo "FAIL: stderr 未输出 LCM_BACKEND_UNAVAILABLE 关键字" >&2
  echo "--- stderr ---" >&2
  cat "${TMP_ERR}" >&2 || true
  exit 1
fi

echo "PASS: real-lcm 启动期 fail-fast(exit=${EXIT_CODE}),stderr 含 LCM_BACKEND_UNAVAILABLE"
exit 0