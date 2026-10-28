# 虚拟禁区遮罩去除冗余拷贝资格记录

2026-10-06 UTC。候选的 12 项独立 native 功能检查全部通过，七个新进程的同进程配对基准全部完成。512²、2048²、4096² FailClosed 的候选批次均值各赢 7/7，按配对比中位数计算分别降低 50.54%、52.73%、52.37%。内部矩形的收益较小且有反例：512² 赢 5/7，2048² 赢 6/7；512² 候选单次最大值更差，2048² 候选的观测 P95 更差。

**处置状态：独立结果审查通过，保留这项边界明确的去拷贝优化。** 审查复核了 12 项 focused 检查、7 份原始日志、35 个场景、70 个计时臂、784 个整数纳秒样本及分位数，并核对源码、冻结参考、二进制和 profile 身份未变。保留依据是精确行为一致及本 profile 下的局部配对证据；不把纯遮罩构建结果扩大成发布、控制周期、探索或实机导航收益，也不宣称矩形尾部一致改善。

## 修改边界与源码推导

生产修改只涉及 Geometry 的两处函数区域：

- `mapping_adapter.rs::virtual_obstacles_snapshot` 保留原有 applied-zones 门控及缺少 raw-map 的错误。改为借用 raw map，逐项复制 width、height、resolution、origin_x、origin_y、frame_id、revision、timestamp_ns 八个头字段，并用 `vec![0; raw.data.len()]` 直接创建虚拟层的私有缓冲区；不以 width×height 作为新分配长度
- `forbidden_zones.rs` 提取原有布局验证；借用入口仍在 clone 前验证一次，新增 owned 入口验证已有缓冲区一次，两者进入同一 owned rasterizer。原有乘法、条件顺序、错误文本、FailClosed、enabled/outside/inside 遍历和最终写入顺序保持不变；光栅化函数只把几何读取从 raw 改为 effective
- 没有加入尺寸或 origin 有限性门槛，没有改动禁区文件读取、缓存、session/frame/receipt 验证、元数据相等比较、锁范围或 full-cloud owner 路径

令 N=`raw.data.len()`；元素为 i8，因此 N 也是 occupancy payload 的字节数。原实现先 clone N 字节 raw payload、清零，再在 compose 中 clone N 字节；候选直接建立零缓冲并原地 compose。由源码可推导：每次实际生成遮罩移除 2N 字节的 payload 拷贝工作，N 大小的 payload 分配表达式由两个减少到一个，即每次 capture 少一个 N 大小缓冲区。清零和最终掩码写入仍存在。

`lcm_runtime.rs::navigation_map_3d_publication_metadata` 的三次合法 capture 均保留：初次取得元数据、发送前重验、发送后重验（本次冻结源中对应调用位于 10592、10675、10699 行）。在具有实际遮罩且成功完成这三个 capture 的发布路径上，源码推导的移除量为 6N 字节 payload 拷贝，以及三个 N 大小的临时缓冲区分配表达式。重试、无掩码或 clear 路径不能直接套用这个次数。

这是源码工作量推导，未测分配器调用、物理内存带宽、RSS 差值，也不等同于峰值常驻内存下降；其他消息构造/编码的克隆未删除。没有合并或省略任何 capture。

## 正确性证据与冻结参考

生产修改前已保存旧 virtual 方法及独立旧 compose 主体，并实际运行三项新增功能测试和忽略基准。冻结 compose 不调用候选入口；它共享未修改的底层光栅几何原语。候选构建后，两份冻结测试/参考文件与基线逐字节一致。

新增测试使用 `MappingAdapter::new(sample_config())`，直接赋值 raw_map 和 applied_forbidden_zones，不调用 start_session、checkpoint 安装或 backend/socket 启动。测试明确覆盖：

- 完整 payload 与八个头字段逐项相等；resolution、origin_x、origin_y 用 `to_bits()` 比较，包括 -0.0；FailClosed 全部为 100，普通虚拟层只含 0/100，不投影物理占据/未知单元
- applied 缺失、empty、全部 disabled，在 raw 缺失或布局无效时仍先返回 None；启用/FailClosed 且 raw 缺失时保持原错误；FailClosed 无 enabled zones 仍生成遮罩
- width/height 为 0、payload 长度不足/过长、resolution 为 ±0、负数、NaN、±∞ 的既有错误；这些布局错误仍先于 FailClosed 填充。origin 为 NaN、±∞ 时继续保持原有接受行为，没有新增验证门槛
- 原 raw bytes/headers 不变；修改返回遮罩的 data/frame/origin/revision 不影响 raw 或后续 snapshot；借用 compose 仍保留未被禁区覆盖的 -1/100/0/42 单元，返回输出不与输入共享可变存储
- 既有 focused 检查继续覆盖线段保守 supercover、矩形内含、outside 区域交集、圆形 outside、凹多边形、坏文件 FailClosed、已应用快照与未来文件隔离、两层禁区/commit identity、伪造 receipt/root/frame 拒绝

12 项 exact selectors 均各自执行、退出 0；完整列表见文末。不是全 Geometry/workspace 测试通过声明。

## Profile、预检与计时边界

- Rust/cargo 1.88.0，CMake 3.31.6；使用原有缓存和安装 SDK，Cargo jobs=2、CMake jobs=1。三个 native CMake consumer 的 `FETCHCONTENT_FULLY_DISCONNECTED=ON` 配置通过；两个 GTSAM consumer 均指向同一组安装 archive，没有 `_deps`，没有执行 SDK rebuild
- 普通 Geometry `--bin nuwax-geometry-runtime` test profile：主 crate opt-level=0、debuginfo=0、debug assertions 与 overflow checks 开启；MLS 与 runtime package opt-level=3。features 为 default、real-lcm-bridge。不是 release
- 基线构建 19:47:11–19:49:02 UTC，候选构建 19:58:04–19:59:20 UTC，均退出 0。各七个受保护源码/锁文件的构建前后 SHA-256 校验均通过
- 正式数据来自七个新候选进程；每个进程在同一个 adapter 上交替运行冻结旧版 R 与候选 C。每个场景固定 2 对预热，FailClosed 固定 16 对计时、内部矩形固定 4 对计时，轮换先运行的版本且次数平衡。未运行 4096² 普通矩形或更大的矩形基准
- timed call 内对 Result 和 Some 做成功检查，对实际 MapState `black_box` 后析构；这两个步骤对两臂一致并计入耗时。source 构建、raw 备份、精确结果/输入断言均在计时外；计时前后都检查输出等价与原输入不变
- 原始样本和总量均保存为精确整数 ns。七次全部退出 0、无超时；保留全部样本与异常尾部，不删除慢进程。每臂每个 FailClosed 场景有 112 个正式操作样本，每个矩形场景有 28 个，共 784 个正式操作样本
- 这是直接调用真实 virtual-mask 方法的纯构建基准，不启动 live transport 或 session/backend/socket；已有功能测试的覆盖也不构成真实传输、端到端发布 deadline 或硬件验证

## 修改前控制与首次候选（不并入正式统计）

单位 ms/次，均由原始 total_ns/实际次数计算。前两列是真实生产修改前、两臂逻辑相同的控制进程；后两列为首次候选资格进程。这些运行均成功，但它们不是跨进程性能配对，不能据此归因。控制中的 2048² FailClosed 等明显波动也完整保留。

| 场景 | 控制 R | 控制当前 | 首次候选 R | 首次候选 C |
| --- | --- | --- | --- | --- |
| 512² FailClosed | 3.055429 | 3.023435 | 2.139916 | 0.923860 |
| 512² 内部矩形 | 29.251875 | 30.208150 | 17.400885 | 17.912107 |
| 2048² FailClosed | 54.994800 | 38.194718 | 35.988056 | 15.500629 |
| 2048² 内部矩形 | 388.808380 | 436.343464 | 348.731914 | 286.865051 |
| 4096² FailClosed | 176.057418 | 176.400710 | 148.151712 | 69.657083 |

## 七次正式配对：批次均值

R 为冻结旧方法与旧 compose，C 为当前候选。每个进程每臂的“批次均值”是该场景 16 或 4 次操作的算术均值。下面的中位数取七个批次均值，不是单次操作 P50。组中位数比=`median(C)/median(R)`；配对比中位数=`median(Cᵢ/Rᵢ)`，两者分别列出。胜次按同进程批次均值判断。

| 场景 | R 批次均值中位数 ms | C 批次均值中位数 ms | 组中位数比 | 配对比中位数 | C 更快 |
| --- | --- | --- | --- | --- | --- |
| 512² FailClosed | 2.373331 | 1.158616 | 0.488181 | 0.494632 | 7/7 |
| 512² 内部矩形 | 21.625703 | 20.138458 | 0.931228 | 0.961276 | 5/7 |
| 2048² FailClosed | 39.496851 | 19.138163 | 0.484549 | 0.472681 | 7/7 |
| 2048² 内部矩形 | 318.456300 | 298.736715 | 0.938078 | 0.901801 | 6/7 |
| 4096² FailClosed | 165.625525 | 80.754476 | 0.487573 | 0.476334 | 7/7 |

按配对比中位数计算，三个 FailClosed 场景减少 50.54%、52.73%、52.37%；512²/2048² 矩形分别减少 3.87%/9.82%。矩形结果有输掉的进程与更差的尾部，不支持“所有场景、所有尾部都改善”的说法。

| 场景 | R 批次均值最小–最大 ms | C 批次均值最小–最大 ms | 配对比最小–最大 |
| --- | --- | --- | --- |
| 512² FailClosed | 1.962430–13.032662 | 0.881838–6.490785 | 0.449360–0.552513 |
| 512² 内部矩形 | 17.619666–77.130163 | 17.725726–87.313828 | 0.865058–1.132032 |
| 2048² FailClosed | 36.179706–88.196240 | 17.101445–43.437846 | 0.419761–0.496797 |
| 2048² 内部矩形 | 302.465026–663.115507 | 272.763197–442.168523 | 0.666805–1.020566 |
| 4096² FailClosed | 158.807701–242.192555 | 73.795396–110.847189 | 0.364143–0.522009 |

全部逐进程配对比 C/R（没有删去不利结果）：

| 进程 | 512² FailClosed | 512² 内部矩形 | 2048² FailClosed | 2048² 内部矩形 | 4096² FailClosed |
| --- | --- | --- | --- | --- | --- |
| 1 | 0.449360 | 1.006019 | 0.472681 | 0.901801 | 0.502900 |
| 2 | 0.498040 | 1.132032 | 0.492514 | 0.939020 | 0.476334 |
| 3 | 0.471278 | 0.961276 | 0.496797 | 0.893881 | 0.517799 |
| 4 | 0.517012 | 0.945653 | 0.457273 | 1.020566 | 0.468572 |
| 5 | 0.463826 | 0.865058 | 0.486479 | 0.937073 | 0.464684 |
| 6 | 0.494632 | 0.985015 | 0.454339 | 0.899511 | 0.364143 |
| 7 | 0.552513 | 0.922681 | 0.419761 | 0.666805 | 0.522009 |

## 正式操作样本：观测分布

本表单独汇总七个进程的单次操作样本，未把“批次均值”当作操作样本。中位数为通常样本 median；P95 使用 nearest-rank，即排序后第 ceil(0.95×n) 项，不插值。它们是有限样本的观测值，不是生产延迟界限或置信区间。单位 ms。

| 场景 | 臂 | n | 最小值 | 中位数 | 观测 P95 | 最大值 |
| --- | --- | --- | --- | --- | --- | --- |
| 512² FailClosed | R | 112 | 1.540202 | 2.258862 | 16.020067 | 29.655885 |
| 512² FailClosed | C | 112 | 0.650518 | 1.010776 | 6.937769 | 20.727241 |
| 512² 内部矩形 | R | 28 | 16.092286 | 21.061539 | 82.333616 | 96.342505 |
| 512² 内部矩形 | C | 28 | 15.918411 | 19.860413 | 81.383376 | 117.212252 |
| 2048² FailClosed | R | 112 | 31.275179 | 38.125815 | 110.975559 | 154.725590 |
| 2048² FailClosed | C | 112 | 13.126514 | 18.146093 | 50.363762 | 74.242452 |
| 2048² 内部矩形 | R | 28 | 290.529011 | 323.645053 | 415.550211 | 1514.289339 |
| 2048² 内部矩形 | C | 28 | 265.648299 | 294.916170 | 439.981282 | 601.784076 |
| 4096² FailClosed | R | 112 | 139.201237 | 165.006150 | 262.240052 | 821.294932 |
| 4096² FailClosed | C | 112 | 62.665233 | 76.938806 | 141.619551 | 260.521760 |

必须保留的尾部反例与波动：

- 512² 矩形候选最大 117.212252 ms，旧版最大 96.342505 ms；候选在第 1、2 个进程的批次均值更慢，第 2 个进程候选均值 87.3138285 ms、旧版 77.130163 ms
- 2048² 矩形候选观测 P95 为 439.981282 ms，高于旧版 415.550211 ms；第 4 个进程候选批次更慢。第 7 个进程同时包含候选 601.784076 ms 和旧版 1514.289339 ms 的单次样本，均没有移除
- 4096² FailClosed 单次最大值仍达候选 260.521760 ms、旧版 821.294932 ms；第 6 个进程的旧版批次均值 242.192555 ms 包含该旧版慢样本。其余所有范围见表与原始 ns 数组

全部三个候选批次更慢的矩形记录（ms/次）：

| 进程 | 场景 | R 批次均值 | C 批次均值 | C/R |
| --- | --- | --- | --- | --- |
| 1 | 512² 内部矩形 | 17.619666000 | 17.725725750 | 1.006019 |
| 2 | 512² 内部矩形 | 77.130163000 | 87.313828500 | 1.132032 |
| 4 | 2048² 内部矩形 | 318.456299750 | 325.005789250 | 1.020566 |

因此保留稳健的 FailClosed 配对改善与矩形混合证据，不用一次慢样本反推系统原因，也不承诺尾部期限改善。

## 全进程资源，仅作环境记录

每个进程同时运行两种实现，还含 fixture raw 备份、预热、精确断言和进程开销。以下 CPU、墙钟、峰值 RSS 无法归因到单臂，不能用来报告候选 CPU/RSS 节省。所有正式进程的 launch_peak_rss_kib 为 134656 KiB。

| 进程 | 墙钟 s | user CPU s | system CPU s | 峰值 RSS KiB | voluntary/involuntary context switches |
| --- | --- | --- | --- | --- | --- |
| 1 | 11.922103 | 11.490428 | 0.406980 | 134656 | 2/114 |
| 2 | 15.926717 | 15.479264 | 0.431482 | 134656 | 3/48 |
| 3 | 12.526773 | 12.103903 | 0.400646 | 134656 | 3/86 |
| 4 | 12.258486 | 11.750383 | 0.491730 | 134656 | 3/88 |
| 5 | 12.372534 | 12.046543 | 0.273830 | 134656 | 2/101 |
| 6 | 13.284776 | 13.037044 | 0.195537 | 134656 | 2/42 |
| 7 | 19.434021 | 18.732934 | 0.634940 | 134656 | 3/155 |

## 证据身份

本机证据根目录为 `Q=/workspace/shared/neh-virtual-mask-copy-20261006`。`baseline/` 与 `candidate/` 保存源码快照、构建脚本/日志/退出值、前后 source 校验、二进制身份及首次运行；`matched/01.log` 至 `07.log` 为正式重复。`matched/results.json` 保存每次精确样本、总 ns、批次均值、比值、全进程资源与各日志 SHA-256；`repeat.py` 保存汇总算法。它们是本机可追溯证据路径，不是已发布网页。

| 对象 | SHA-256 |
| --- | --- |
| 基线 mapping_adapter.rs | `10bacd314c079c7cbb81b2ae6435866639a22b6ba01046841558598df8158aa9` |
| 候选 mapping_adapter.rs | `a47da168db294ff75b1c43eef0198c51ba129b16a32f70865a6f16aaf9831032` |
| 基线 forbidden_zones.rs | `1071bc3c5d6b4effffcb14476b1059060f8c44714b7a629f06f533de3df6590a` |
| 候选 forbidden_zones.rs | `f71fe19a45e5394f66db2b651aea5f30ff973a764cfe6804bd6fdebfee5afc96` |
| 共同冻结 virtual 方法/测试/基准 | `426bf0c68aaf7fe4701582dfb70ec05d06832796a0ee8c87897bd9b4cdcb6911` |
| 共同冻结旧 compose 参考 | `1a6393b4c01d8e380d6c06756554e4c727d786a9656d6415ba91b35256376da1` |
| 测试前置 delta | `5c33f45bc9209c84f2d80fef5af3872fb7f083a33ae05eae989055d0d84868c4` |
| 生产 delta | `0c65ce4ad23036d126e340be278467676a20a04f7adfd137e6fd367e78580bda` |
| 基线测试二进制 | `b3243a2295097c93fa97cd4fe75d674caa1029a5acbba4547f7a1c3fe2036f6b` |
| 候选测试二进制 | `31c2fbfdc364500b4bb52896c964bd4ba46353c6dce4eab5ef591ca713af27f4` |
| 正式 results.json | `704c9e6858fec2cd712a05fdfeb2a689afc97e0940dd10dd46fdde420bbe5ce6` |

两版本共同未改的其他保护项（相对 Geometry crate）：

```text
c71c6ea19ef5fa2ba75e29f10425ab62bbaaf94cab7bdca3b09431f472bdab90  src/lcm_runtime.rs
9d0be2a30be10ef941bd8fd82045c9a48be8a4c9f1a48007abc1b0a3912c2401  Cargo.lock
a0ecb96b0a4b77e4a7c03d953a90483b702c6bdcdaef781bded063ec5711acdf  ../../nuwax-workspace/crates/nuwax-nav-pct/src/tomogram.rs
```

## 实际预检、构建与执行命令

环境脚本固定 SDK/target、debug=0，清除外来 RUSTFLAGS 和 wrapper；overlay 将 Cargo jobs 固定为 2、CMake jobs 固定为 1。预检记录 `preflight/environment.txt`、三个 CMake log 和 `preflight/result.txt` 均保留。下面是实际使用的预检流程（没有执行 cmake --build）：

```bash
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
Q=/workspace/shared/neh-virtual-mask-copy-20261006
for entry in gtsam:nuwax-gtsam/native pgo:nuwax-nav-pgo-bridge/native/pgo slam:nuwax-slam-toolbox-sys/native; do
  name=${entry%%:*}; directory=${entry#*:}
  cmake -S "$REPO/services/nuwax-workspace/crates/$directory" \
    -B "$Q/preflight/$name" -DCMAKE_PREFIX_PATH="$SDK;$META" \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    -DLCM_GEN_EXECUTABLE="$LCM_GEN_EXECUTABLE" \
    > "$Q/preflight/$name.log" 2>&1
done
```

两个 GTSAM consumer 的记录均与以下已安装文件一致，文件存在性已检查，预检目录无 `_deps`：

```text
/workspace/shared/neh-native-20261006/sdk/lib/libgtsam.a
/workspace/shared/neh-native-20261006/sdk/lib/libmetis-gtsam.a
/workspace/shared/neh-native-20261006/sdk/lib/libcephes-gtsam.a
```

两版实际 Cargo 构建命令相同，分别保存独立日志、manifest 和冻结二进制：

```bash
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
cd "$REPO/services/geometry/nuwax_geometry_runtime"
cargo test --locked --offline --bin nuwax-geometry-runtime \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  --no-run --message-format=json
```

修改前功能检查为 3 passed、0 failed、1 ignored；candidate 的 12 个 selectors 各按下列 exact 命令执行。基准只在明确指定 ignored selector 时执行；正式七个进程均使用同一候选二进制 SHA-256。

```bash
Q=/workspace/shared/neh-virtual-mask-copy-20261006
"$Q/baseline/nuwax-geometry-runtime-test" \
  mapping_adapter::tests::virtual_mask_copy_trial:: --nocapture --test-threads=1
"$Q/candidate/nuwax-geometry-runtime-test" "$selector" \
  --exact --nocapture --test-threads=1
# stage=baseline: 修改前控制；stage=candidate: 首次资格及正式七次新进程
"$Q/$stage/nuwax-geometry-runtime-test" \
  mapping_adapter::tests::virtual_mask_copy_trial::copy_cost_baseline \
  --exact --ignored --nocapture --test-threads=1
```

12 项通过的 exact selectors：

```text
mapping_adapter::tests::virtual_mask_copy_trial::parity_headers_payload_and_isolation
mapping_adapter::tests::virtual_mask_copy_trial::gates_and_layout_errors_match_frozen
mapping_adapter::tests::virtual_mask_copy_trial::borrowed_compose_preserves_physical_cells
forbidden_zones::tests::diagonal_line_uses_conservative_supercover_cells
forbidden_zones::tests::rectangle_inside_marks_interior_and_boundary
forbidden_zones::tests::outside_zones_intersect_their_allowed_interiors
forbidden_zones::tests::circle_outside_keeps_only_cells_fully_inside
forbidden_zones::tests::concave_polygon_inside_is_rasterized_without_filling_the_notch
forbidden_zones::tests::malformed_document_fails_closed
mapping_adapter::tests::virtual_obstacles_use_applied_snapshot_without_physical_projection_or_future_file
lcm_runtime::mapping_feed_tests::navigation_map_3d_virtual_obstacles_keep_commit_identity_and_block_both_floors
lcm_runtime::mapping_feed_tests::retained_cloud_metadata_rejects_forged_receipt_root_and_frame
```

功能日志和逐项 SHA-256 位于 `candidate/focused-results.json`、`focused-01.log` 至 `focused-12.log`。此记录不声称全量测试、live transport、session 生命周期、SDK 重建或实机部署已经验证，也不代表已提交、推送或部署。
