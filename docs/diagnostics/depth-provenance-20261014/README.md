# 深度地面来源保留：定向验收（2026-10-14）

基于本地集成检查点 `9d713ae1f35410315e207040f5c352197d8bf08f` 上的来源修复。本文记录实际执行结果，不独立证明提交、推送、部署或全项目通过。

- [来源归档升级、配对部署与回滚边界](../../depth-ground-provenance-archive-20261014.md)
- [此前合并检查点的独立验收](../../integration-09db973-20261014.md)

## 结果

Rust 1.88.0；Python 3.8.10；云端 CPU；已有本地 liblcm 1.5.2。使用原有 release profile、`--locked --offline -j2` 和已有 target，没有 clean、降低优化、放宽预算或并行启动第二条 Cargo 编译通道。

几何主程序测试构建必须使用 `--bin nuwax-geometry-runtime`；相关模块不在 `--lib`。最终几何构建通过，耗时 8 分 53 秒（完整 LTO）；五个工作区 crate 的测试构建通过，耗时 3 分 36 秒。编译通过与下表实际运行分开统计。

| 范围 | 通过 | 最终仍未执行的显式 ignored 项 | 说明 |
| --- | ---: | ---: | --- |
| geometry binary | 106 | 10 | 来源、准入、PGO、归档及已有点云回归 |
| nuwax-map-chunks | 68 | 1 | 完整 lib 测试；包含 9 项来源回归 |
| nuwax-nav-mls-planner | 24 | 3 | observations、shared ground、recent query |
| nuwax-nav-pct | 14 | 3 | tomogram |
| nuwax-codec | 16 | 1 | navigation map 15 项，加单独执行的 warmed fixture 1 项 |
| nuwax-nav-stack | 44 | 2 | 生产 Reader、层传输和 selective-refresh owner |
| 合计 | 272 | 20 | 去重统计，不重复计算重跑与交叉语言 fixture 生成 |

另有 1 项真实跨进程 multicast RPC 测试因环境失败，未通过、未验证：`surface_chunks::rpc_process_tests::real_process_rpc_recovers_late_missing_and_corrupt_blocks_without_partial_install`。liblcm 在初始化 `udpm` 时报告网络不可达、缺少 multicast route，尚未进入 RPC 行为断言。未修改网络配置，未换成模拟传输。该文件中的 Manifest API 迁移已编译，跨进程运行仍须在支持 multicast 的环境补验。

Python 127 项通过：archive identity 26、vector reader 8、checkpoint identity 44、legacy cloud 36、assigned AR1 13。另将真实 Rust Writer 生成的 1762 字节 AR4/BL3 合成归档，经 Python provider 导出、checkpoint 保存、重开、重新上传和下载后逐字节核对成功：3 个来源保留，辅助来源 sequence 0 保留，相同数字样本 ID 不合并成一个来源；应用哈希禁止哨兵保持启用。此交叉语言诊断不计入 127。

## 已执行的关键证明

- fine-cloud 18 项：包含新增辅助来源 6 项、原始索引、独立来源命名空间、来源内水位线、同时间戳序列推进、更低时钟数值、旧归档与有界保留
- mapping-observation 6 项：实际准入、过滤后原始索引、有效观测相等比较、捕获位姿和来源绑定
- PGO 11 项：包含辅助点云在串行和 worker 路径中按修正位姿重投影，保留原始来源/时间/索引；原有 10 秒 catch-up 预算未放宽
- planar-artifacts 12 项：9 项 assigned identity，加 3 项 planar pair；覆盖新 AR4 及旧 AR1/2/3 读取边界
- 其余 geometry 59 项：pointcloud 40、self-filter 6、LiDAR owner 4、真实进程内 ground ingress 1、assigned export/transfer/activate/relocalize RPC 4、checkpoint assembly 4
- 新增 `source_watermark_only_transfer_preserves_blocks_and_geometry`：固定占据/样本/根时间，仅推进来源水位线和加入另一时钟来源；Writer→Transfer→生产 Reader 后块对象、几何修订及原始样本不变，脏列为空
- 新增 `selective_refresh_source_watermarks_add_zero_geometry_or_planner_work`：沿用已有真实进程内 liblcm owner/scheduler 测试装置，只在初始化观察一次占据；水位线变化不增加 normal/quick planner 或 reference-check 调用，路径修订不变，命令持续执行
- 层传输测试实际经过 BL3/v4 Reader，验证来源、时间、索引和水位线；纯诊断变化不脏化几何，平面变化与删除仍产生精确脏列

## 保留的失败与修复记录

1. 首次几何编译失败，未执行测试：生产 `mapping_observation` 调用了不存在的 `GeometryRuntimeHandle::runtime_generation()`，测试也有同样调用；另一个旧归档测试尝试直接修改 `Arc<Manifest>`。来源修复使用实际已有的 `handle.nav.producer_runtime_generation()` 权威值，测试克隆 Manifest 值后修改。修复后按完全相同 profile 重建并通过。原失败属于代码/API 问题，未归为环境失败。
2. 首次 codec navigation-map 运行：14 通过、1 失败、2 ignored；失败是测试启动的 C 编译器找不到已安装 SDK 的 `lcm/lcm_coretypes.h`。仅通过 `CPATH` 暴露已有 liblcm include 路径，未改源代码、测试 oracle 或二进制；原测试组重跑为 15 通过、2 ignored。随后单独运行 warmed fixture，通过；仅确认 fixture/编解码断言，不宣称性能提升。
3. 上述真实跨进程 multicast RPC 环境失败保留为未完成验收，没有计入通过。

原始构建日志、逐个非零测试选择列表、测试日志与合成 fixture 保存在 checkout 外。每个执行 selector 均先使用 `--list` 验证命中非零测试。ignored 项主要是显式性能诊断、外部输入诊断或专用 interop；未自动扩充为性能或实机结论。

## 复现

以下路径均为占位符。先准备已有 Rust 1.88.0、Python 3.8.10、LCM 工具和项目 Python 依赖；从仓库根目录执行。SDK include 修复只作用于命令环境。

```sh
source /path/to/test-env.sh
LCM_PREFIX=/path/to/existing/liblcm
export CPATH="$LCM_PREFIX/include${CPATH:+:$CPATH}"
EVIDENCE=/path/outside-checkout/qualification
mkdir -p "$EVIDENCE"

cargo test --release --locked --offline -j2 \
  --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
  --bin nuwax-geometry-runtime --no-run
cargo test --release --locked --offline -j2 \
  --manifest-path services/nuwax-workspace/Cargo.toml \
  -p nuwax-map-chunks -p nuwax-nav-mls-planner -p nuwax-nav-pct \
  -p nuwax-nav-stack -p nuwax-codec --features nuwax-nav-stack/mls --lib --no-run
```

使用构建输出中实际列出的测试可执行文件，不能复用旧二进制路径。每个 selector 先运行 `BINARY SELECTOR --list`，确认非零；再运行 `BINARY SELECTOR --nocapture --test-threads=1`。

| 二进制 | selector |
| --- | --- |
| geometry | `pointcloud::`；`lcm_runtime::mapping_observation::tests::`；`pgo_map::tests::`；`planar_artifacts::` |
| geometry | `self_cloud::tests::`；`lio_ingress::registered_scan_ownership_tests::`；`in_process_registered_scan_retains_unclassified_fine_evidence` |
| geometry | `server::lifecycle_tests::assigned_v2_`；`server::assigned_checkpoint_assembly_tests::` |
| map-chunks | 空 selector，运行整个 lib |
| MLS | `surface::observations::`；`shared_ground_tests::`；`recent_query_tests::` |
| PCT | `tomogram::` |
| codec | `lcm::schema::navigation_map_3d_tests::` |
| nav-stack | `surface_chunks::tests::`；`selective_refresh_tests::` |

codec 额外 fixture 使用完整 selector `lcm::schema::navigation_map_3d_tests::warmed_fine_map_codec_baseline --exact --ignored --nocapture --test-threads=1`。跨进程 RPC 使用上文完整 selector，在有真实 multicast 支持的环境单独执行；此测试创建随机测试命名空间、TTL 0 的 LCM endpoint，不连接机器人。

Python 127 项的命令保留测试所需的最小包初始化隔离，仍导入真实 runtime/provider/store/protobuf 模块：

```sh
python3.8 - <<'PY'
import runpy, sys, types
from pathlib import Path
root = Path.cwd()
sys.path[:0] = [str(root), str(root / 'tests')]
for name in ('services', 'services.geometry'):
    package = types.ModuleType(name)
    package.__path__ = [str(root / name.replace('.', '/'))]
    sys.modules[name] = package
from services.geometry.runtime import PlatformGeometryRuntime
sys.modules['services.geometry'].PlatformGeometryRuntime = PlatformGeometryRuntime
for filename in ('scripts/check_fine_map_archive_identity.py',
                 'tests/test_fine_archive_vector.py', 'tests/test_checkpoint_identity_v2.py',
                 'tests/test_legacy_cloud_import.py', 'tests/test_assigned_ar1_import.py'):
    sys.argv = [filename]
    try:
        runpy.run_path(str(root / filename), run_name='__main__')
    except SystemExit as result:
        if result.code:
            raise
PY

NEH_PROVENANCE_ARCHIVE_OUT="$EVIDENCE/ground.ar4" \
  cargo test --release --locked --offline -j2 \
  --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-map-chunks --lib \
  source_provenance_tests::source_watermarks_survive_ar4_restore_without_cross_clock_cutoffs \
  -- --exact --nocapture
python3.8 scripts/diagnostics/check_ground_provenance_roundtrip.py \
  "$EVIDENCE/ground.ar4" > "$EVIDENCE/crosslanguage.json"
```

## 结论边界

这是合成云端 CPU 的定向源码、生产 Reader/Writer、进程内 owner/scheduler、归档及 checkpoint 验证。没有物理机器人、真实深度相机、现场数据、未同步设备时钟校准、部署、浏览器/WebGL 或全项目验收。来源命名空间与原始数值保留不等于跨时钟时间转换证明；现有时间准入、捕获位姿与根边界契约仍然适用。旧归档缺少真实来源时保持 Legacy，不补造传感器身份或水位线。
