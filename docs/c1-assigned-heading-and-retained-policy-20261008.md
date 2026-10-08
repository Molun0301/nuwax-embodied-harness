# C1：已保存地图重定位朝向与历史策略读取

日期：2026-10-08。调查基线为 `0276e5f73431106b36412578f339a8d4e49fda31`，检出分支为 `fix/v3-legacy-cloud-seal-20261006`。本文记录本机离线证据和验证边界，不代表机器人部署结果。

## 问题与修复

基线的局部 ICP 只要 MSE 有限就直接提交估计，跳过后续全局配准。合成反例包含 128 个源点、136 个目标点：仅 8 个重复结构点吻合，ICP 仍报告 `converged=true`、MSE `4.434e-32`，精匹配支持率为 `0.0625`，提交朝向为 −17°，真实朝向为 37°，误差 54°。因此仅要求 ICP 收敛不能解决这一问题。

修复复用已经计算的局部评分，沿用原全局管线的数值条件：内点数至少 3、支持率至少 0.45、RMSE 有限、重力倾角不超过 10°。不满足时继续原有全局求解；没有新增数值门限、求解器收敛门、重试、锁、哈希或全图复制。相同反例在修复后得到正确的 37°，支持率 1.0、128 个内点、57 个特征匹配，MSE `2.634e-29`。

局部评分继续使用原始点的权重；全局评分继续使用 0.1 m 体素降采样后的权重。两者使用相同数值条件，但评分域并不相同。代码注释和算法策略描述明确保留这一区别。

另一个修复是使用源帧已经捕获的有效注册变换作为初值，避免重复重定位时遗漏此前提交的校正：

- `A`：会话固定的 `map <- odom` 锚点。
- `S`：源帧捕获的当前有效注册变换，包含已提交校正。
- 局部输入为 `S * odom_points`，最终配准为 `T = delta * S`。
- 输出仍为绝对校正 `C = A⁻¹ * T`，应用仍为 `A * C = T`。

初值 `S` 与固定锚点 `A` 使用独立参数，避免重复累计旧校正。测试覆盖非零锚点、非单位旧校正连续两次重定位，以及带非零平移和偏航初值扰动的 4 点稀疏输入、32 点局部可见输入。保存/加载坐标系、四元数约定和原有重复、过期结果处理链没有改变；高度专题不在本次范围内。

## 算法二历史记录的只读兼容

当前算法策略由 2 升为 3。原持久化读取复用了仅允许当前策略的执行准入，升级后旧记录可能使整个终态仓库无法打开。因此将历史读取与新执行准入分开。

历史读取只接受当前完整描述，或已发布的精确旧描述：schema 2、source policy 3、deadline policy 1、algorithm policy 2，以及完全对应的参数字节。未知版本或修改过的参数仍被拒绝。支持的路径为持久化请求/终态读取、查询及响应解析、已提交位姿取样，以及高层相同请求的恢复；新求解执行仍只接受当前策略。

旧终态、候选和身份字节不改写；不清库、不迁移为新请求、不运行旧算法。取样和采用结果仍核对当前安装快照、资产、完整激活身份、提供方来源、目标可用性和当前规范修正/位姿。历史 `Applied` 本身不能证明当前机器人仍使用该修正；进程重启或拥有方更换后可能需要重新定位。

固定夹具 `tests/fixtures/relocalization_policy_algorithm2_0276.json` 来自基线提交的 Python 编码器，并与独立编译的同提交 Rust 编码器输出逐字节核对。原生测试使用真实 V3 候选和 Applied 回执构造持久化记录，在旧记录与当前记录混合的仓库中验证重新打开、查询及字节不变。这是合成协议夹具，不包含现场地图或机器人记录。

Python 高层测试验证：旧请求在已发送、结果未知及已完成阶段，使用相同超时可查询和取样恢复，且不重新发送求解；改变超时在 RPC 前拒绝。执行入口拒绝旧策略，原有地图/会话/提供方变化拒绝测试也通过。

## 已完成验证及复现命令

Rust 使用 1.88.0、现有本地缓存和单个编译作业。两次生产测试二进制增量编译分别为 34.65 s、22.27 s；无原生 SDK 重建。最终定向回归 38 项通过，另运行 2 项历史兼容测试，其中 1 项已包含在 38 项中，去重共 **39 项**。独立摘录程序和前轮重复测试不计入这个数量。

以下命令在本次工作机从仓库根目录执行。环境脚本和工具路径是该工作机已安装的依赖，不是通用安装步骤。复测前应协调本机负载窗口；命令不启动项目服务。

```bash
source /home/ml/Documents/Codex/2026-10-07/task/neh-performance-investigation/native-project-env.sh
export CARGO_BUILD_JOBS=1
export CARGO_PROFILE_TEST_DEBUG=0
export CARGO_TARGET_DIR=/home/ml/Documents/Codex/2026-10-07/task/neh-done-work/services/nuwax-workspace/target
export LCM_GEN_EXECUTABLE=/home/ml/.local/bin/lcm-gen
export PROTOC=/home/ml/.local/bin/protoc

selectors=(
  relocalization_local_consensus
  'relocalization_identity::tests::'
  'relocalization_terminal_v2::tests::'
  relocalization_policy_parameters_preserve_current_exact_bytes_without_hashes
  relocalization_source_seal_uses_each_frame_odom_and_retains_boundary_guard
  canonical_transaction_uses_nonzero_anchor_and_absolute_c_without_accumulation
  relocalization_target_rollover_is_typed_preclaim_terminal
  relocalization_session_rollover_is_typed_preclaim_terminal
  relocalization_runtime_duplicate_is_ignored_and_same_key_conflict_fail_closes
  relocalization_sparse_window_matches_and_rejects_degenerate_geometry
  relocalization_runtime_algorithm_reject_releases_cutoff_guard
  retained_policy_v2_applied_reopens_with_current_records_and_queries_exact_bytes
)
for selector in "${selectors[@]}"; do
  cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
    --locked --offline \
    --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
    --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
    --bin nuwax-geometry-runtime "$selector" -- --test-threads=1 || exit 1
done
```

实际验证在增量构建后直接使用同一测试二进制运行各过滤器；上面的循环可复制执行相同的去重范围，并复用缓存。现有库目标不包含这些测试，必须选择 `--bin nuwax-geometry-runtime`。

Python 实际运行版本为 **3.12.3**，合同 29 项、提供方 32 项、工作区恢复 26 项，合计 **87 项**，18.158 s 全部通过。此前通过轮或失败轮不另行累加。

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests \
  /home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python -B -m unittest \
  test_assigned_relocalization_contract \
  test_assigned_relocalization_provider \
  test_assigned_workspace_relocalization
```

当时全部 9 个改动 Python 文件（包括另一地图激活阶段的两个文件）通过 `ast.parse(feature_version=(3, 8))`，并通过现有 Ruff 的 `check --no-cache --select E9,F63,F7,F82`。AST 检查发生在 Python 3.12.3 上，**仅证明语法可解析，不是 Python 3.8.10 实际运行验证**。当前 Rust/Python 的三组策略参数字节一致，旧夹具与基线原生编码器的三组字节也一致。

## 持久证据与未验证范围

本机证据目录：

```text
/home/ml/Documents/Codex/2026-10-07/task/neh-performance-investigation/map-activation-heading-20261008/heading-investigation/
```

- `baseline_result.log`、`after_result.log`：相同输入的 −17° 与 37° 对照。
- `heading_probe.rs`、`run_probes.py`：合成反例和摘录验证；不替代生产编译。
- `native_new_tests.log`、`native_compat.log`、`native_final_regression.log`：生产二进制构建及定向结果。
- `python_compat.log`：新增历史恢复测试曾在 Python 取样策略比较处失败的原始日志；`python_compat_final.log` 为修复后的 87 项成功结果。
- `policy_parity.log`、`historical_policy_provenance.log`：当前跨语言一致性和独立旧夹具来源校验。
- `python38_syntax.log`、`python_critical_lint.log`：语法及关键静态检查。
- `heading-complete.patch`、`final-report.txt`：本阶段补丁和详细交接；补丁不含另一地图激活阶段的修改。

尚无真实失败帧，不能断言此次修复覆盖全部现场偶发朝向问题。真实保存地图、稠密非均匀采样、机器人运动及对称场景未复测；对称或重复结构存在多个高支持率姿态时，原算法仍可能无法唯一判别。未运行目标 Python 3.8.10，未连接机器人、SSH、原项目服务或执行部署。本文不预设最终提交编号。
