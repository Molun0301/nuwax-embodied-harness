# 共享地面与近期障碍重构：技术说明与验收

记录日期：2026-10-13。本文描述本次实现及已完成的离线验证；不代表实机部署、整仓测试或性能验收全部通过。

## 数据与责任边界

- **先关联物理层，再统计真实点。** `ground_observations.rs` 在有界局部 patch 中拟合、分离地面层，保留原始已接纳返回点的 sequence、timestamp、point index。每层最多保留 32 个真实内点；统计量描述这批有界样本，不是整列所有点的平均值、所有历史返回的总体统计或传感器标定误差。混合地面/障碍的体素不被当作纯地面。
- **代表平面与统计均值分开。** `LayerObservation.plane_origin` 是固定 XY 单元中心的拟合平面位置，normal 是关联层的法向；sample mean/covariance 等仅作观测统计。规划几何成员、平面或法向变化才推进几何版本，观测散布、样本数及时间更新不单独使全局几何失效。输出保持区间相对于固定平面判断，累积位移不会永远被吞掉。
- **PCT 与局部控制共用地面解释。** 两者消费同一物理层及 legacy band 的 `GroundLayer` 语义，不在消费者中各自重拟合、拼层或根据试探机体高度改变地面。旧量化 band 只能给接触区间近似；其区间中点不是实际点均值。实体占据体采用暴露顶面，同时保留完整占据区间。
- **几何不是逐格时效许可。** 生产判定已移除依赖地面逐格 hits、TTL 或重复“静态可行性许可”的通行门槛；旧地面不因为未重复扫到便失去几何有效性。真实机体接触、已知落差、障碍、边界及旋转/平移扫掠检查继续生效。
- **未知只能有界估计。** 缺测处仅在有界、连通、同平面邻域内估计，并保留 measured / legacy / estimated / unknown 区分。未知不等于占据，也不等于自由；允许估计不能证明现实没有坑。传感器看不到的落差、反射异常与标定错误仍需实机安全验证。

主要入口：`services/geometry/nuwax_geometry_runtime/src/ground_observations.rs`、`services/nuwax-workspace/crates/nuwax-map-chunks/src/terrain_observations.rs`、`nuwax-types/src/ground_layer.rs`、`nuwax-nav-mls-planner/src/surface/body.rs`、`nuwax-nav-pct/src/tomogram.rs`（后四者位于 workspace 的 `crates/`）。

## 静态地图与 live 近期障碍分离

静态真实障碍继续保留在持久地图。独立的 `nuwax_msgs.RecentCloud3D` v1 经 `/nav/observations3d` 传输近期返回，携带映射 session/generation、frame generation、producer generation、commit boot nonce 以及原始 LIO cloud identity。它不冒充地面更新，也不依赖 terrain revision：接纳了新扫描、地图没有新 commit 时，仍发布 live 数据。

近期视图使用源时间窗口，最多保留八帧不可变 frame；retention 为零时只保留当前帧。ground indices 表示物理层候选，并非无条件删除障碍的白名单；混合体素与实际表面仍进入机体几何判定。PGO 对归档已接纳扫描进行现有的一次校正投影，保留原始点序与身份，不把重放生成的新时间当作 live 健康证据。

source 健康独立于静态地面：消费端检查 authority/domain、源序列和时间单调性及配置的源期限。静态地图更新不会续期 live source；域变化、源停更和显式 revoke 有各自处理。当前没有新增周期性空 revoke 发布器，原入口也不会把空扫描接纳成健康数据。启用源健康检查时，失效不能靠持久地面绕过。

## 局部重规划与命令复核

- recent guide overlay 仅属于本次局部绕行请求。PCT 的可选 segment filter 覆盖起终点连接、邻边和路径缩减，不修改静态 Tomogram，也不把近期点写入全局图或使全局图整体失效
- 外接机体半径投影只是导引近似，不授予运动许可；导引无路时保留原有精确姿态 Scan 尝试。最终路径仍逐段执行真实机体扫掠，搜索动作、精确 pose key、代价及 256-expansion 预算未放宽
- 同一 pose 下 recent 高频微抖不会仅凭版本变动取消已选结果：计算中或已发布的单条选中轨迹，使用最新 recent 几何重新复核，不重新枚举并排名候选，也不延长原有效期限。新障碍确实相交时拒绝非零命令
- pose 改变会退役旧姿态结果。计算中追 recent、复核后发布、已发布结果 replay 均核对 pose version，不能把旧姿态轨迹重新标记成新姿态输出

固定起点 A/B 的完整因果与边界见 [navigation-fine-search-recovery.md](navigation-fine-search-recovery.md#recent-obstacle-guide-starvation-regression-2026-10-13)。该案例证明原静态导引造成有效转向候选饥饿；不是实体机器人运动闭环演示。

## 存储与 Python 兼容性

携带层观测的新增格式为 BL2、manifest v3、archive AR3（纯几何 archive 仍可为 AR2）；新读端保留 BL1、旧 manifest v2、AR1/AR2 读取。旧资料不会被捏造出真实点总体。AR3 的 assigned artifact identity schema 仍为 2；传输、保存、重开、重上传需保持既有身份绑定。**旧二进制不保证能读新格式**，回滚前应保留旧程序可读的地图备份；本次未访问、迁移或覆写用户地图。

Python 读端只做 wire shape、有界数组、字段名、整数范围、有限数值及既有 archive/checkpoint 身份约束；物理层算法语义由 Rust owner 负责，Python 不重复拟合或质量判定。

CPython **3.8.10 实际运行**通过 21 项 archive、8 项 scalar/vector、43 项 checkpoint，共 **72 项**，并通过 Rust Writer 生成 AR3 的跨语言读取。环境使用 grpcio 1.70.0、protobuf 5.29.6、numpy 1.24.4、pydantic 2.10.6。测试仅隔离 `services` 包初始化中无关的 cv2 副作用；实际 archive、provider、store、protobuf 模块均真实加载。这不是只做 3.8 语法解析，也不是完整 Web 服务启动验证。

## 已完成与未完成验收

以下来自保留日志；ignored 不计作通过。

| 项目 | 已观察结果 | 范围/限制 |
| --- | --- | --- |
| 底层来源与协议 | map-chunks 57/57；codec 14/14；source 12/12 + live 发布 1/1 | 不包含仍失败的原 PGO 10 秒预算验收 |
| MLS 最终全 lib | 311 passed / 0 failed / 15 ignored | 最终共享地面实现 |
| PCT 全 lib | 35 / 0 / 3 ignored | 最终接口另有 focused 1/1 |
| Scan profile 全 lib | 50 / 0 / 2 ignored | 在删除死 witness、增加诊断测试前运行；生产搜索未改 |
| Scan 最终 focused | body witness 5/5；两组 A/B 2/2 | 不能代替再次全组测试 |
| Runtime 最终 pose 版 | 225 / 0 / 45 ignored / 5 filtered | 五项真实 UDS 测试受运行环境限制 |
| Runtime latest recent | 5/5 | 包括同 pose 微抖、两时机 pose 退役、障碍拒绝及期限不续期 |
| Runtime 三项显式 ignored probe | 3/3 | 最终 pose 二进制；源停更负例及两种命令 replay |
| Rust assigned artifact | 8/8 | 包括 AR3 保留 identity 2；旧 envelope 拒绝 |
| Python 3.8.10 | 72/72 | 真实 archive/store 回归，初始化隔离见上 |
| Stack focused 集成 | selective refresh 27/27；recent consumer 1/1；surface chunks 8/8（2 ignored） | 含真实长门洞 1/1（317.68 秒）；不是 Stack 全组 |
| Stack qualification feature | 最终 `--no-run` 编译成功（exit 0） | 首次编译暴露五处测试接口未迁移，修复后通过；完整 opt3 / 60 秒资格场景本轮未跑 |
| release A/B 性能 | 五对独立进程，10/10 断言通过 | A 按预期耗尽预算；B 返回有效路径，详见下节 |

五项 UDS 过滤不能表述为忽略后“全通过”：独立重试仍在 bind 时出现 `PermissionDenied: Operation not permitted`。需要允许 Unix socket 的环境重跑。未执行全 workspace、实机传感器回放/运动闭环、实机 CPU/延迟长时采样及整 Web 服务启动验收。**不能据此宣称实机 98% CPU 问题已解决。**

## 性能诊断与保留失败

原 PGO Raytracing `background_catchup_preserves_replay_cadence_maps_and_checkpoint` 的 **10 秒预算仍失败**，没有调大预算掩盖失败。独占 debug 对照中关闭 layer capture 也失败：31 帧 × 2993 点的 workload 仅处理 14 帧便超时；capture 关/开分别约 10.201/10.307 秒。诊断测试自身通过只说明成功记录 A/B，不代表原 PGO 验收通过，也不足以断言全部开销来源。

同组 Voxel capture 开为 8.94/8.46 秒，关为 7.65/7.67 秒。这是特定 fixture 的单次 debug 诊断样本，不能宣称普遍加速，也不能外推实机 CPU。release 局部导引 A/B 已完成五对新进程测量，按 AB / BA 交替顺序。两组使用同一 release/profile 二进制、同一合成固定起点场景；A 用静态 PCT 导引，B 从五个真实近期返回构建 request-local 导引。十次测试进程均 exit 0，其中 A 每次按预期返回 `BudgetExhausted`（256 展开、642 nodes），B 每次返回 186 poses，并通过真实机体逐段二次扫掠。

| release 测量口径 | A 中位数（范围），ms | B 中位数（范围），ms |
| --- | --- | --- |
| guide + Scan（`total_ms`） | 84.62（71.32–125.40） | 6.70（6.54–7.14） |
| 全测试进程 wall | 148.30（121.26–213.22） | 62.38（59.56–66.25） |

B 的 guide 中位数为 1.67 ms、Scan 为 5.00 ms。公共 Scan 沿导引直接成功，fallback 搜索调用与展开均为零；它与手工 witness 导引直接调用局部搜索得到“六次展开”的早期诊断不是同一测量路径。

`total_ms` 排除 fixture / 静态图构建及输出后二次扫掠，进程 wall 包含这些工作。该 A/B 比较的是同版本算法导引选择，**不是旧 commit 对新 commit 的基准**；A 失败、B 成功的输出语义不同，不能表述成等效任务普遍加速。五对合成样本不能外推实机 CPU、运动闭环、长尾延迟或全平台性能。

## 复现入口

从仓库根目录执行，使用 `rust-toolchain.toml` 的 Rust 1.88.0、仓库锁文件及所需 native 依赖（包括 protoc）。测试串行执行，避免并行 workload 污染计时。以下命令不等于声称本节所有入口均已完成；结果以验收表为准。

```bash
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-map-chunks --lib -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-codec --lib recent_cloud::tests -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-codec --lib lcm::lio::tests -- --test-threads=1
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime fine_cloud:: -- --test-threads=1
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime in_process_registered_scan_retains_unclassified_fine_evidence -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-mls-planner --lib -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-pct --lib -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-scan --features profile --lib -- --test-threads=1
# 在允许 Unix socket 的环境运行完整 Runtime，不能将下面结果与过滤版混算
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-runtime --lib -- --test-threads=1
# 上述受限环境的已记录命令，只排除下面五项；其余原 ignored 保留
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-runtime --lib -- --test-threads=1 \
  --skip tests::cancel_preempts_delayed_renew_over_real_uds_with_qualified_latency \
  --skip tests::cancel_preempts_delayed_submit_over_real_uds_with_qualified_latency \
  --skip tests::cancel_replays_identical_fence_after_first_response_timeout \
  --skip tests::real_uds_adapter_requires_correlated_applied_and_fence_proofs \
  --skip tests::server_deadline_renewal_keeps_legal_slow_commands_alive
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-runtime --lib latest_recent_ -- --test-threads=1
for selector in \
  route_moving_worker_tests::moving_5cm_source_positive_and_expired_negative_probe \
  planner_worker::tests::retained_command_replay_requires_current_full_braking_permission \
  planner_worker::tests::retained_steady_command_replay_requires_current_full_braking_permission; do
  cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-runtime --lib "$selector" -- --ignored --exact --test-threads=1 || exit "$?"
done
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-stack --lib --features mls,guard-runtime surface_dataflow::tests::selective_refresh_tests -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-stack --lib --features mls,guard-runtime recent_consumer -- --test-threads=1
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-stack --lib --features mls,guard-runtime surface_chunks::tests -- --test-threads=1 --skip complete_fine_manifest_resolves_a_real_door_across_storage_blocks
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-stack --lib --features mls,guard-runtime complete_fine_manifest_resolves_a_real_door_across_storage_blocks -- --test-threads=1
# 仅编译，不执行完整 qualification
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-stack --lib --features mls,guard-runtime,surface-recovery-qualification --no-run
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime planar_artifacts::assigned_identity_tests -- --test-threads=1
# 原预算失败复现，不应为获得通过而修改预算
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime pgo_map::tests::background_catchup_preserves_replay_cadence_maps_and_checkpoint -- --exact --test-threads=1
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime pgo_map::tests::pgo_capture_cost_same_fixture_ab -- --ignored --exact --nocapture --test-threads=1
NEH_ON_DIAGNOSTIC=1 cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-scan --features profile --lib recent_wide_obstacle_fixed_start_reference_ab -- --ignored --nocapture --test-threads=1
```

release A/B 的精确入口如下。先构建，再由 Cargo JSON 获取本次真实测试二进制；每次直接启动新进程，避免把 Cargo 启动开销混入进程 wall。需要统计 wall 时应在每个二进制进程外独立计时。

```bash
build_log=$(mktemp)
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-scan --lib --release --features profile --no-run --message-format=json > "$build_log" || exit "$?"
scan_binary=$(python3 - "$build_log" <<'PYBIN'
import json, sys
with open(sys.argv[1]) as stream:
    rows = [json.loads(line) for line in stream if line.startswith('{')]
matches = [row['executable'] for row in rows
           if row.get('reason') == 'compiler-artifact'
           and row.get('target', {}).get('name') == 'nuwax_nav_scan'
           and row.get('profile', {}).get('test') and row.get('executable')]
assert len(matches) == 1, matches
print(matches[0])
PYBIN
) || exit "$?"
for order in 'A B' 'B A' 'A B' 'B A' 'A B'; do
  for case in $order; do
    NEH_RECENT_AB_CASE="$case" "$scan_binary" recent_detour_diagnostic_tests::recent_wide_obstacle_exported_guide_ab --exact --ignored --nocapture --test-threads=1 || exit "$?"
  done
done
```

Python 3.8.10 复现 72 项回归，可在装有上述依赖的隔离虚拟环境中从仓库根运行以下等价入口；只隔离包初始化，不替换被测模块：

```bash
python3.8 - <<'PY'
import runpy, sys, types
from pathlib import Path
root = Path.cwd()
sys.path[:0] = [str(root), str(root / 'tests')]
for name in ('services', 'services.geometry'):
    package = types.ModuleType(name)
    package.__path__ = [str(root / name.replace('.', '/'))]
    sys.modules[name] = package
for filename in ('scripts/check_fine_map_archive_identity.py',
                 'tests/test_fine_archive_vector.py',
                 'tests/test_checkpoint_identity_v2.py'):
    sys.argv = [filename]
    try:
        runpy.run_path(str(root / filename), run_name='__main__')
    except SystemExit as result:
        if result.code:
            raise
PY
```

此入口不重新生成跨语言 AR3 fixture；跨语言验证另需 Rust Writer 产物，不能以纯 Python fixture 替代。

## 部署前检查：PointLIO blind

现有 native PointLIO 未显式覆盖时默认 `blind=0.5`；检索到的 2026-04 旧 ROS 配置也为 0.5。近期实机旧值未知，本次没有访问或改动实机配置。这两份源码线索不能证明当前设备正在使用什么值。

请在设备上自行检查实际 `NUWAX_GEOMETRY_POINTLIO_OPTIONS` / PointLIO options 是否有 `blind=0` 覆盖；若有，依据该设备的近距标定、机身遮挡和雷达安装恢复合适的非零盲区。不要把 0.5 米统一写死给所有机器人。修改后需重新启动 geometry runtime 与 LIO 进程，并核对生效参数、近距点云和源健康；只改配置文件不会改变已启动进程。先在受控场景验证，再恢复真实运动。
