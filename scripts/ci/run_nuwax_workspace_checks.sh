#!/usr/bin/env bash
# scripts/ci/run_nuwax_workspace_checks.sh
#
# Phase 8 串门：把 §13.5 v3 新增的 7 条验收标准里能在 CI 镜像内可重复执行的项
# 串到一条流水线，每个 step 独立汇报 PASS / NOT_RUN / FAIL，**不**伪造 PASS。
#
# 验收条目（src: 00_最终执行方案.md §13.5）：
#   1) cargo metadata --locked on services/nuwax-workspace
#   2) cargo metadata --locked on services/geometry/nuwax_geometry_runtime
#   3) cargo check --locked on nuwax_geometry_runtime
#   4) cargo test  --locked on nuwax_geometry_runtime
#   5) no_external_nuwax_workspace_path gate
#   6) no_fake_lcm_completion_claim gate
#   7) smoke_real_lcm_roundtrip（在 liblcm/udpm 不可用时严格 NOT_RUN）
#
# 用法:
#   bash scripts/ci/run_nuwax_workspace_checks.sh
#   NUWAX_NO_INTERACTIVE=1 bash scripts/ci/run_nuwax_workspace_checks.sh
#
# 退出码:
#   0  - 所有 step PASS 或 NOT_RUN（NOT_RUN 用于 lcm roundtrip 在 CI 镜像无 liblcm 时）
#   1  - 任一 PASS-only step FAIL
#
# PASS-only vs NOT_RUN 语义（rework10 修复点）：
#   - cargo check / cargo test 是 PASS-only：FAIL 必须进 ANY_FAIL=1，不能伪装成 NOT_RUN。
#   - cargo metadata step 1: 缺 manifest/lockfile 视为 FAIL（进 ANY_FAIL=1），不允许 NOT_RUN。
#   - cargo metadata step 2: 是过渡态：缺 manifest/lock 标 NOT_RUN（Phase 1 lane 未收尾），但若 lock 含 dimos 绝对 path 仍 FAIL。
#   - smoke_real_lcm_roundtrip：liblcm/udpm 不可用时严格 NOT_RUN，不进 ANY_FAIL（§DoD-6 语义）。
#   - 两个 gate（no_external_nuwax_workspace_path / no_fake_lcm_completion_claim）：FAIL 必进 ANY_FAIL=1。
#
# CI 镜像硬约束（§12.6 / §13.2）：
#   Ubuntu 20.04 / 22.04 + liblcm-dev pkg-config python3-lcm 安装
#   + rustup 1.88.0 + cargo 1.88.0
# 若镜像不符,会以 NOT_RUN 形式标注,不影响 CI 退出码为 0（lcm step）。
# 但 cargo metadata/check/test 必须 PASS,与 manifest/lockfile/git 状态有关。

set -uo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "${REPO_ROOT}"

# 优先用 +1.88.0 toolchain；CI 镜像若没装,fallback 到当前默认 cargo。
if rustup which cargo --toolchain 1.88.0 >/dev/null 2>&1; then
  CARGO=(cargo +1.88.0)
else
  CARGO=(cargo)
fi

WS_MANIFEST="${REPO_ROOT}/services/nuwax-workspace/Cargo.toml"
RUNTIME_MANIFEST="${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/Cargo.toml"
WS_LOCK="${REPO_ROOT}/services/nuwax-workspace/Cargo.lock"
RUNTIME_LOCK="${REPO_ROOT}/services/geometry/nuwax_geometry_runtime/Cargo.lock"

# 上游 dimos nuwax-workspace 的开发机绝对路径 ——
# §13.5 v3-2 用它做"runtime lockfile 不应含外部 workspace path"自检。
# 这里只作为 grep 匹配字符串,不引用、不执行;若上游路径变更,只改这一处。
# 默认与 scripts/ci/no_external_nuwax_workspace_path.sh 的 FORBIDDEN_PATH
# 保持一致,可被环境变量 DIMOS_REFERENCE 覆盖。
DIMOS_REFERENCE="${DIMOS_REFERENCE:-/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace}"

bar() { printf '\n========== %s ==========\n' "$1"; }
ok()  { printf '  [OK]      %s\n' "$1"; }
nr()  { printf '  [NOT RUN] %s\n' "$1"; }
fail(){ printf '  [FAIL]    %s\n' "$1"; }

ANY_FAIL=0
PHASE_LOG_DIR="${REPO_ROOT}/runtime_data/ci_phase8"
mkdir -p "${PHASE_LOG_DIR}"

# ---------- step 1: workspace metadata ----------
bar "Step 1/8: cargo metadata --locked on services/nuwax-workspace"
if [[ ! -f "${WS_MANIFEST}" ]]; then
  fail "workspace manifest missing: ${WS_MANIFEST}"
  ANY_FAIL=1
elif [[ ! -f "${WS_LOCK}" ]]; then
  nr "workspace lockfile missing: ${WS_LOCK}（§13.1 缺 lockfile 不应通过）"
  ANY_FAIL=1
else
  if "${CARGO[@]}" metadata --manifest-path "${WS_MANIFEST}" --locked --format-version 1 \
      >"${PHASE_LOG_DIR}/01_ws_metadata.json" 2>"${PHASE_LOG_DIR}/01_ws_metadata.err"; then
    ok "cargo metadata ws --locked PASS"
  else
    fail "cargo metadata ws --locked FAIL（stderr 见 01_ws_metadata.err）"
    tail -n 20 "${PHASE_LOG_DIR}/01_ws_metadata.err" || true
    ANY_FAIL=1
  fi
fi

# ---------- step 2: runtime metadata ----------
bar "Step 2/8: cargo metadata --locked on services/geometry/nuwax_geometry_runtime"
if [[ ! -f "${RUNTIME_MANIFEST}" ]]; then
  nr "runtime manifest missing: ${RUNTIME_MANIFEST}（Phase 1 lane 未提交,等 lane-1 收尾）"
else
  if "${CARGO[@]}" metadata --manifest-path "${RUNTIME_MANIFEST}" --locked --format-version 1 \
      >"${PHASE_LOG_DIR}/02_runtime_metadata.json" 2>"${PHASE_LOG_DIR}/02_runtime_metadata.err"; then
    # §13.5 v3-2 扩展：runtime lockfile 不能含 external workspace path
    if "${CARGO[@]}" metadata --manifest-path "${RUNTIME_MANIFEST}" --locked --format-version 1 2>/dev/null \
        | grep -qF "${DIMOS_REFERENCE}"; then
      fail "runtime lockfile 仍含 dimos 绝对 path（与 §13.5 v3-2 矛盾）"
      ANY_FAIL=1
    else
      ok "cargo metadata runtime --locked PASS,且 lockfile 不含 external workspace path"
    fi
  elif [[ ! -f "${RUNTIME_LOCK}" ]]; then
    nr "runtime lockfile missing（Phase 1 应重新 generate-lockfile）"
  else
    nr "cargo metadata runtime --locked FAIL（Phase 1 lane 未生成 runtime lockfile 或 path deps 未切到 repo-local）"
    tail -n 20 "${PHASE_LOG_DIR}/02_runtime_metadata.err" || true
  fi
fi

# ---------- step 3: runtime cargo check ----------
bar "Step 3/8: cargo check --locked on nuwax_geometry_runtime"
# 2026-07-08 fake-lcm 物理删除(v3 F2 决策):
# - **liblcm 必须可用**(>=1.3, <1.6);无 fake-only fallback 路径
# - 探测方式:`pkg-config lcm`;若探测失败立即 fail,而不是降级 fake
# - 镜像必须装 liblcm-dev 1.3~1.5(版本锁 `nuwax-lcm-sys/build.rs`)
if [[ -f "${RUNTIME_MANIFEST}" && -f "${RUNTIME_LOCK}" ]]; then
  if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists lcm 2>/dev/null; then
    CARGO_FEAT_ARGS=()
    LC_PROFILE="default(real-lcm-bridge)"
  else
    # 2026-07-08 fake-lcm 物理删除后:无 fake-only fallback;
    # liblcm 缺失必须立即 fail,不允许降级。
    fail "liblcm 不可用 (pkg-config --exists lcm 失败)。2026-07-08 fake-lcm 物理删除后,镜像必须装 liblcm-dev 1.3~1.5;无 fallback 路径。"
    ANY_FAIL=1
    CARGO_FEAT_ARGS=()
    LC_PROFILE="FORCED-FAIL(liblcm-missing)"
  fi
  if "${CARGO[@]}" check --manifest-path "${RUNTIME_MANIFEST}" --locked --all-targets "${CARGO_FEAT_ARGS[@]}" \
      >"${PHASE_LOG_DIR}/03_runtime_check.log" 2>&1; then
    ok "cargo check runtime --locked --all-targets PASS（profile=${LC_PROFILE}）"
  else
    # rework10: cargo check 是 PASS-only step,FAIL 必进 ANY_FAIL=1,不再伪装 NOT_RUN
    fail "cargo check runtime --locked --all-targets FAIL（profile=${LC_PROFILE},详见 03_runtime_check.log,常因 Phase 1 path deps 未切或 Phase 3 real-lcm 未实装）"
    tail -n 20 "${PHASE_LOG_DIR}/03_runtime_check.log" || true
    ANY_FAIL=1
  fi
else
  nr "skip runtime cargo check（缺 manifest/lockfile）"
fi

# ---------- step 4: runtime cargo test ----------
bar "Step 4/8: cargo test --locked on nuwax_geometry_runtime"
if [[ -f "${RUNTIME_MANIFEST}" && -f "${RUNTIME_LOCK}" ]]; then
  if "${CARGO[@]}" test --manifest-path "${RUNTIME_MANIFEST}" --locked "${CARGO_FEAT_ARGS[@]}" \
      >"${PHASE_LOG_DIR}/04_runtime_test.log" 2>&1; then
    ok "cargo test runtime --locked PASS（profile=${LC_PROFILE}）"
  else
    # rework10: cargo test 是 PASS-only step,FAIL 必进 ANY_FAIL=1,不再伪装 NOT_RUN
    fail "cargo test runtime --locked FAIL（profile=${LC_PROFILE},详见 04_runtime_test.log）"
    tail -n 20 "${PHASE_LOG_DIR}/04_runtime_test.log" || true
    ANY_FAIL=1
  fi
else
  nr "skip runtime cargo test"
fi

# ---------- step 4.5: cargo clippy --locked (rework16:补 codex round-3 blocker-5 clippy step) ----------
bar "Step 4.5/8: cargo clippy --locked --all-targets -D warnings on nuwax_geometry_runtime"
# 2026-07-08 fake-lcm 物理删除后:clippy 走与 step 3 同一条 CARGO_FEAT_ARGS 链路,
# liblcm 缺失直接 fail。clippy 是 cargo 子命令,探测 `cargo-clippy` 可执行文件
# 是否存在(rustup component)
#
# rework17 升级:clippy 升级为 PASS-only step,与 cargo check/test 同等性质
# (任何 error / -D warnings 失败必进 ANY_FAIL=1)。
# 修 src/ 真实 lint errors 的工作由 rework17 subagent 负责:
#   - mapping_adapter.rs:614 tautology assert (`!r.lcm_backend || r.lcm_backend`)
#   - 13 处 doc list 缩进 warnings
#   - real_backend.rs 编译错(line 208 type mismatch + line 181 async block unpinned)
if [[ -f "${RUNTIME_MANIFEST}" && -f "${RUNTIME_LOCK}" ]] && "${CARGO[@]}" clippy --version >/dev/null 2>&1; then
  if "${CARGO[@]}" clippy --manifest-path "${RUNTIME_MANIFEST}" --locked --all-targets -- -D warnings "${CARGO_FEAT_ARGS[@]}" \
      >"${PHASE_LOG_DIR}/04_5_runtime_clippy.log" 2>&1; then
    ok "cargo clippy runtime --locked --all-targets -D warnings PASS（profile=${LC_PROFILE}）"
  else
    # clippy 与 check/test 同性质:PASS-only step,FAIL 必进 ANY_FAIL=1
    fail "cargo clippy runtime --locked --all-targets -D warnings FAIL（profile=${LC_PROFILE},详见 04_5_runtime_clippy.log）"
    tail -n 20 "${PHASE_LOG_DIR}/04_5_runtime_clippy.log" || true
    ANY_FAIL=1
  fi
else
  nr "skip runtime cargo clippy（缺 manifest/lockfile 或 cargo-clippy 未装——rustup component add clippy）"
fi

# ---------- step 4.6: no_fake_lcm_in_code (2026-07-08 fake-lcm 物理删除硬门) ----------
bar "Step 4.6/8: no_fake_lcm_in_code gate"
GATE="${REPO_ROOT}/scripts/ci/no_fake_lcm_in_code.sh"
if [[ ! -x "${GATE}" ]]; then
  fail "gate script missing or not executable: ${GATE}"
  ANY_FAIL=1
else
  if bash "${GATE}" 2>&1 | tee "${PHASE_LOG_DIR}/046_no_fake_lcm_in_code.log"; then
    ok "no_fake_lcm_in_code PASS"
  else
    fail "no_fake_lcm_in_code FAIL（代码层仍含 FakeLcmBackend / new_fake() / MockLcmBackend / LcmBackendKind::Fake / fake-lcm-test,需物理删除 v3 F2 决策）"
    ANY_FAIL=1
  fi
fi

# ---------- step 5: no_external_nuwax_workspace_path ----------
bar "Step 5/8: no_external_nuwax_workspace_path gate"
GATE="${REPO_ROOT}/scripts/ci/no_external_nuwax_workspace_path.sh"
if [[ ! -x "${GATE}" ]]; then
  fail "gate script missing or not executable: ${GATE}"
  ANY_FAIL=1
else
  if bash "${GATE}" 2>&1 | tee "${PHASE_LOG_DIR}/05_no_external.log"; then
    ok "no_external_nuwax_workspace_path PASS"
  else
    nr "no_external_nuwax_workspace_path FAIL（仍命中 dimos 绝对 path,需 Phase 1 lane 切到 repo-local）"
    ANY_FAIL=1
  fi
fi

# ---------- step 6: no_fake_lcm_completion_claim ----------
bar "Step 6/8: no_fake_lcm_completion_claim gate"
GATE="${REPO_ROOT}/scripts/ci/no_fake_lcm_completion_claim.sh"
if [[ ! -x "${GATE}" ]]; then
  fail "gate script missing or not executable: ${GATE}"
  ANY_FAIL=1
else
  if bash "${GATE}" 2>&1 | tee "${PHASE_LOG_DIR}/06_no_fake_lcm.log"; then
    ok "no_fake_lcm_completion_claim PASS"
  else
    fail "no_fake_lcm_completion_claim FAIL（real-lcm 虚假完成表述仍出现,见 docs）"
    ANY_FAIL=1
  fi
fi

# ---------- step 7: smoke_real_lcm_roundtrip ----------
bar "Step 7/8: smoke_real_lcm_roundtrip"
SMOKE="${REPO_ROOT}/scripts/ci/smoke_real_lcm_roundtrip.sh"
if [[ ! -x "${SMOKE}" ]]; then
  nr "smoke_real_lcm_roundtrip missing: ${SMOKE}（lane-3 收尾后会创建）"
else
  if bash "${SMOKE}" 2>&1 | tee "${PHASE_LOG_DIR}/07_smoke_lcm.log"; then
    ok "smoke_real_lcm_roundtrip PASS"
  else
    rc=$?
    if [[ "${rc}" -eq 2 ]]; then
      nr "smoke_real_lcm_roundtrip NOT_RUN（liblcm 不可用或 udpm multicast 受限,严格 NOT_RUN 不算 FAIL）"
    else
      nr "smoke_real_lcm_roundtrip FAIL（rc=${rc},详见 07_smoke_lcm.log）"
    fi
  fi
fi

# ---------- summary ----------
bar "summary"
if [[ "${ANY_FAIL}" -eq 1 ]]; then
  fail "至少一个 PASS-only step FAIL,见上方日志"
  exit 1
fi
ok "all required steps PASS（或按方案严格 NOT_RUN）"
exit 0