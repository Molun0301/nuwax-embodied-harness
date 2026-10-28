# C1 Inside Rectangle 范围扫描资格记录

2026-10-06 UTC；唯一预定义隔离控制实验于 21:20:42 UTC 完成。经独立审查，最终保留狭义范围候选；本记录明确限定该决定的资格边界。

**最终处置：保留 42 行 Inside Rectangle 范围候选，停止性能归因实验。精确矩形行为与局部收益已确认；通用 FailClosed 非退化仍未证明。** 独立审查已复核原始构建身份、功能结果、14 份正式日志及统计计算，并建议这一有界保留。唯一预定义隔离实验完成 28/28 次成功运行，没有复现一致变慢的模式；原始不利控制仍是证据，不能称为全路径性能通过或因果证明。

## 修改边界与数值等价性

原始矩形/混合基准与 16 项 focused 结果指向 `Q/candidate` 的冻结候选。后加入的独立 FailClosed 测试文件及 `mapping_adapter.rs` 测试 include 已另行编译，隔离结果只属于 `Q/isolation` 下的新二进制；两组构建身份不能混用。当前 source（包括隔离文件/include）与已编译 `isolation/candidate` 的十项 source manifest 完全匹配，生产候选为 291e…；隔离共 28 次运行通过。此前 16 项 focused 测试没有在新 harness 二进制上重跑，也没有整个 workspace 测试通过的声明。

- 生产差异仅为 `forbidden_zones.rs` 中 Inside Rectangle 的 42 行范围路径；候选 SHA-256 为 `291eabe6a22cd6335c9dca2971c0b4e6aac1d1e319d4609fd80164c68cc85361`。测试模块包含在两版构建中，不能把它们误算为生产差异
- 原有布局校验、FailClosed 提前填充、enabled/Outside 门控、共享几何原语、其他形状、metadata/session/receipt 验证、锁范围和 Cargo.lock 均未改动；没有添加 origin 有限性校验，也没有改变直接调用者的既有 panic/error 次序
- 先用原有 `rectangle_from_corners`、`boundary_half_width`、`expand_rect` 得到扩张矩形；在每轴对实际 `cell_rect` 做两次 first-true 二分，再对命中行的连续布尔切片 `fill(true)`。不使用 floor/ceil 或反算世界坐标到格索引
- 下界为第一个 `cell.max >= expanded.min - GEOMETRY_EPSILON`；排他上界为第一个 `!(cell.min <= expanded.max + GEOMETRY_EPSILON)`。正有限 resolution 下，原始浮点单元边界单调不减，巨型 origin 的舍入平台仍可二分；两个连续轴区间的笛卡尔积正是原来逐格 inclusive overlap 的命中集合
- 必须保留 `(origin + i*r) + r`；不能换成 `origin + (i+1)*r`。NaN 下界使 start=len，NaN 上界使 end=0，因此保留原始比较的空命中；±Inf、扩张溢出和巨大 origin 也沿用实际比较结果，没有“修复”原有输入语义
- 两轴 `2^53` origin、resolution=0.25 的回归明确固定 borrowed payload 为 `[-1,0,42,100,-1,100,100,100]`，virtual payload 为 `[0,0,0,0,0,100,100,100]`，防止改变 cell.max 舍入造成边界扩大

## 功能证据与范围

- 修改生产代码前，基线 `rectangle_scan` 筛选实际运行 4 项新增 corpus 测试并全部通过；捕获的畸形 vertices panic 是预期的对等结果，不是测试失败
- 候选实际运行 16 个独立 exact selectors，全部退出 0：12 项既有 copy/parity/形状/应用快照/两楼层 commit identity/伪造 receipt-root-frame 检查，以及相同的 4 项 rectangle corpus 检查
- 新 corpus 同时比较借用 compose 与真实 `virtual_obstacles_snapshot` 调用者：完整 payload、全部头字段及浮点位模式、原 raw 不变、返回变体、错误文本、预期 panic；包括缺 map、无 applied、disabled/empty/FailClosed 门控和无效布局优先级
- 数值 corpus 覆盖反序/部分/全覆盖/不相交/退化矩形，1×N/N×1、重叠、stroke 大小与异常值，epsilon 边界及邻接可表示值，±0、NaN、±Inf、f64 极值、f32 次正规/极值 resolution、扩张溢出、巨型 origin 平台、混合其他形状与 Outside 交集
- 可核对的完整 selectors、命令、退出值和日志 SHA 在 `candidate/focused-results.json`；基线证据在 `baseline/new-tests-attempt-1.{json,log}`。这是 focused 资格检查，不是全 Geometry/workspace 测试通过声明

## 固定构建与计时方法

- Rust 1.88.0、CMake 3.31.6，使用既有 SDK/cache；Cargo jobs=2、CMake jobs=1。普通 Geometry test profile：主 crate opt-level=0、debuginfo=0、debug assertions/overflow checks 开启；MLS/runtime package opt-level=3；features=default,real-lcm-bridge。**不是 release**
- 基线构建 20:29:42–20:30:43 UTC；候选构建 20:39:10–20:40:01 UTC；均退出 0，九项受保护 source/lock 的构建前后校验通过。两版 profile、features、测试/参考相同
- **主比较：copy-only 基线可执行文件的 production 臂， versus range 候选可执行文件的 production 臂。** 旧 frozen virtual 臂含上一轮去拷贝工作，只作为输出 oracle/测量控制，绝非本轮增量 raster 基线
- 正式执行 7 对新进程，共 14 次。奇数对 baseline→candidate，偶数对 candidate→baseline；每个进程运行完全相同的旧基准，内部 production/frozen 两臂交替。每场景 2 对预热，FailClosed 16 对计时、矩形 4 对计时；无超时、全部退出 0
- 所有 timed call 包括成功解包检查、MapState black_box 和结果析构；fixture/raw 备份与结果/输入断言在计时外。没有启动 session/backend/socket/live transport；未测 4096² 普通矩形
- 每个版本每个 FailClosed 场景 112 个 production 样本，矩形 28 个。保留全部样本、慢进程和尾部。以下时间为 ms；批次均值=每进程该场景的算术平均，组中位数取 7 个批次均值；配对比取 candidateᵢ/baselineᵢ

## 主比较：正式 production 结果

| 场景 | 基线批次中位数 | 候选批次中位数 | 组中位数比 | 配对比中位数 | 候选更快 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 512² FailClosed | 0.770331 | 0.902608 | 1.171714 | 1.102554 | 0/7 |
| 512² Inside Rectangle | 17.159464 | 2.670582 | 0.155633 | 0.146945 | 7/7 |
| 2048² FailClosed | 14.558968 | 15.246656 | 1.047235 | 1.039646 | 3/7 |
| 2048² Inside Rectangle | 297.297911 | 47.333295 | 0.159212 | 0.166959 | 7/7 |
| 4096² FailClosed | 64.017946 | 76.589018 | 1.196368 | 1.199683 | 0/7 |

矩形两种尺寸均赢 7/7；512² 组中位数 17.159→2.671 ms，2048² 297.298→47.333 ms。FailClosed 512²/4096² 均 0/7，不能用矩形收益掩盖控制路径的退化信号。

| 场景 | 基线批次均值 min–max | 候选批次均值 min–max |
| --- | ---: | ---: |
| 512² FailClosed | 0.728225–0.875977 | 0.822297–0.990528 |
| 512² Inside Rectangle | 15.802449–19.663677 | 2.390060–3.104967 |
| 2048² FailClosed | 13.830229–18.712276 | 14.051284–20.322614 |
| 2048² Inside Rectangle | 270.883836–330.593290 | 45.509296–141.391101 |
| 4096² FailClosed | 61.804086–67.037203 | 67.240506–110.031011 |

全部正式逐对 production 比值，按第 1–7 对排列；精确整数 ns 与未四舍五入汇总仍保存在原始 JSON：

- 512² FailClosed：1.0329432282979563, 1.133540657647545, 1.0614097043606385, 1.10038503576777, 1.3601948230285967, 1.2144411228995835, 1.1025542741674146
- 512² Inside Rectangle：0.1469219790269199, 0.1964864495524222, 0.16555318580710487, 0.15563316837868596, 0.14694501322134287, 0.13992676522393593, 0.14200397113981061
- 2048² FailClosed：1.0845883709711504, 1.099225296202965, 0.9252474287643995, 1.4694343081464876, 1.0396458204223122, 0.9843234843296905, 0.7792731657027111
- 2048² Inside Rectangle：0.16651955262007886, 0.21478612257482912, 0.15189420066378456, 0.47136229420959364, 0.1733664565845177, 0.16695878118628554, 0.15575159894076754
- 4096² FailClosed：1.2359269783897786, 1.6735826467363597, 1.1996828970230828, 1.6332946446184977, 1.1399440601294626, 1.0879621377139461, 1.0173104971330051

### 单次操作观测分布

下面汇总 production 单次操作，不与批次中位数混用；P95 为 nearest-rank 第 ceil(0.95n) 项，无插值。仅为有限样本观测，不是 realtime 界限或置信区间。

| 场景 | 版本 | n | min | median | P95 | max |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 512² FailClosed | baseline | 112 | 0.642246 | 0.703920 | 1.212733 | 2.103288 |
| 512² FailClosed | candidate | 112 | 0.729806 | 0.808779 | 1.420355 | 2.226044 |
| 512² Inside Rectangle | baseline | 28 | 15.272101 | 16.622727 | 18.892947 | 29.584537 |
| 512² Inside Rectangle | candidate | 28 | 2.151403 | 2.489739 | 3.377337 | 4.882345 |
| 2048² FailClosed | baseline | 112 | 11.684051 | 14.364161 | 21.725162 | 28.892154 |
| 2048² FailClosed | candidate | 112 | 12.614212 | 14.603426 | 24.230472 | 38.882003 |
| 2048² Inside Rectangle | baseline | 28 | 264.654759 | 283.974573 | 349.977511 | 457.330872 |
| 2048² Inside Rectangle | candidate | 28 | 37.674513 | 50.274783 | 117.583548 | 299.431096 |
| 4096² FailClosed | baseline | 112 | 54.760087 | 62.754053 | 74.263157 | 96.401931 |
| 4096² FailClosed | candidate | 112 | 62.275836 | 71.642502 | 130.295730 | 367.785688 |

2048² 矩形候选 pooled median=50.274784 ms、P95=117.583548 ms、max=299.431096 ms；第 4 对候选批次均值达 141.391102 ms。FailClosed 候选三个尺寸的 P95/max 均更差；4096² max=367.785688 ms。全部保留，不做离群剔除。

## FailClosed：诊断与仍未证明的范围

- 跨二进制 frozen 臂 FailClosed 配对比中位数：512²=0.9461988404436602，2048²=0.9307797059789663，4096²=1.2131629661455032。只描述控制波动，**不能据此归一化消除 production 的已观测退化**；两臂全部原始 ns 在 14 份日志与 results.json 中保留
- 只读汇编核对：rasterizer 栈帧从 984 增至 1384 bytes；FailClosed 早退 fill、header copy 操作序列保持，归一化的 fill/SpecFill/IterMut 代码完全相同。这是诊断线索，尚未证明栈帧或任何其他因素导致退化
- 512² FailClosed 是旧基准的首个场景，不能用“同一进程前面的矩形负载”解释它的退化。大尺寸场景的缓存/分配器历史也只是假设，不能当作已证原因
- 唯一预先定义的独立控制实验已执行：512²、4096² 各以一个 exact ignored selector 单独启动新进程；使用相同 fixture，2 次 production 预热、16 次 production 计时，保留析构；只在所有计时后运行 frozen 结果核对。此前没有矩形、其他尺寸或 frozen 工作负载。两版本采用相同新增 harness 与既定交替进程顺序；不调整生产参数、不改主 benchmark、不删旧证据、不挑最优重跑
- **隔离实验已完成，但不构成 FailClosed 非退化证明。** 它改变了运行负载历史及二进制布局，不能单独证明原始退化的原因。独立审查后的最终决定为保留狭义矩形候选并停止归因；通用 FailClosed 非退化仍未证明，原始负面证据继续保留

## 唯一预定义隔离实验：独立保留

512²/4096² 各 7 对全新进程，奇数对 baseline→candidate，偶数对 candidate→baseline，共 28 次 exact ignored selector，全部退出 0、无超时。每进程只有 2 次 production 预热、16 次 production 计时；冻结参考、完整 header/payload/input parity 断言在全部计时之后执行。结果析构仍计时，共保留 448 个整数 ns 样本。没有做第二轮性能采样、调整参数或挑选重跑。

构建编排异常与恢复必须保留：原始隔离 baseline 构建 21:12:31–21:13:28 UTC，candidate 构建 21:13:28–21:14:13 UTC，均成功；两者共用 Cargo 输出路径，baseline 可执行文件在独立捕获前被 candidate 输出覆盖。**在任何隔离测量前发现，没有测量使用被覆盖的 baseline。** 在 baseline 重建前于 21:19:06 UTC 冻结 candidate，再于 21:19:06–21:19:51 UTC 仅重建一次完全相同的 baseline 并立即冻结；随后恢复 candidate source。新基线来自 `isolation/baseline-rebuild`，不是 `isolation/baseline`。完整恢复记录为两份 source-restoration.json。

两版仍用原固定 Geometry opt0、MLS/runtime opt3、相同 features/profile；相同隔离 harness SHA 为 `6a9b8f487b78c2da5f3f4d6ac21593b0a90d70c2a7f92d1dbfd4194665fe112b`，mapping_adapter.rs 为 `cfcd64c95f722d75b7d6a2b228b4d283b0a79988a1e39379c6f54918cdbbcaee`。forbidden_zones.rs 分别保持原基线 2e2a… / 候选 291e…，无生产调整。

| 场景 | 基线批次中位数 ms | 候选批次中位数 ms | 组中位数比 | 配对比中位数 | 候选更快 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 512² FailClosed | 0.967076 | 0.837638 | 0.866155 | 0.870484 | 5/7 |
| 4096² FailClosed | 80.494666 | 77.757183 | 0.965992 | 0.853019 | 4/7 |

全部隔离逐对 candidate/baseline 比值（第 1–7 对，不与原始混合基准合并）：

- 512²：1.141692730078931, 0.6457392387971651, 0.7534252627885091, 0.8869578932945381, 0.8582460626652977, 1.1595936955370088, 0.870483824229615
- 4096²：0.7855821640250763, 0.6966060815947827, 1.264329489485727, 1.0441710889382187, 0.8523880814801905, 1.3082545014303166, 0.8530185042461981

| 场景 | 版本 | n | min ms | median ms | P95 ms | max ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 512² | baseline | 112 | 0.642169 | 0.890967 | 1.957774 | 3.201458 |
| 512² | candidate | 112 | 0.729599 | 0.792420 | 1.315578 | 1.684642 |
| 4096² | baseline | 112 | 61.341827 | 76.191176 | 129.936472 | 338.587492 |
| 4096² | candidate | 112 | 59.522569 | 73.036626 | 132.123263 | 231.414030 |

隔离 512² candidate 输第 1、6 对；4096² 输第 3、4、6 对，且 candidate P95=132.123263 ms 仍高于 baseline 129.936472 ms。批次均值范围：512² baseline 0.722143–1.297177 ms / candidate 0.808072–0.997235 ms；4096² baseline 74.467856–110.463725 ms / candidate 66.164606–101.640855 ms。全部慢值保留。

隔离只说明该单独 harness 下没有复现“512²/4096² 每对均慢”的模式；它不能覆盖原始 0/7 结果、证明通用非退化，或把退化归因为前序矩形、编译布局、栈帧、缓存、分配器或系统噪声。最终决定是保留狭义范围候选并停止进一步性能归因实验；不能将这个决定改写为通用 FailClosed 非退化通过。

全进程 CPU 中位数（baseline/candidate）：512² 0.040502/0.035854 s，4096² 2.373459/2.351674 s；28 次 peak RSS 均 134784 KiB。仍包含 fixture、预热和计时后 frozen/输入核对，不能归因为单独 production CPU/RSS。

| 隔离证据 | SHA-256 |
| --- | --- |
| baseline-rebuild 二进制（124566120 bytes） | `7cf459e3cfbf73c5825c4ce13bf658d7e278364caa6fe399fa3766835c05284c` |
| candidate 二进制（124572288 bytes） | `6ba5bd6475ad9c51d520f78acc96fca28d11cc4a436f8b8f436c5032ac1887c1` |
| isolation/matched/results.json | `fbf54ee04b0f26f957353f65d429da8050d2ed0072e2916fe00866a838a8e2d3` |
| isolation/run-isolated.py | `a05621b0ef7731399d726a93467cc4859dc07220d2889db1dfd7b2dba3ba61e2` |

每次命令为冻结版 `nuwax-geometry-runtime-test mapping_adapter::tests::virtual_mask_fail_closed_isolation::isolated_fail_closed_512 --exact --ignored --nocapture --test-threads=1`（另一个 selector 后缀为 `_4096`）。采样脚本 `python "$Q/isolation/run-isolated.py"` 固定顺序并拒绝覆盖已有 matched 目录；此处只记录实际命令，不要求再运行。完整 28 份日志 hash、样本、资源、身份与恢复记录随 `c1-rectangle-range-20261006-results.json` 的 `isolation_experiment` 保存。

## 首次运行：独立保留，不并入正式统计

两版 `benchmark-attempt-1` 均成功、无超时；以下为各臂批次均值。它们不是正式交替进程配对。

| 场景 | 基线 production | 基线 frozen | 首次候选 production | 首次候选 frozen |
| --- | ---: | ---: | ---: | ---: |
| 512² FailClosed | 0.871425 | 2.192731 | 0.902759 | 1.999188 |
| 512² Inside Rectangle | 23.567766 | 27.616718 | 2.579304 | 17.090461 |
| 2048² FailClosed | 13.381224 | 36.915175 | 15.651909 | 34.742309 |
| 2048² Inside Rectangle | 288.180355 | 330.488057 | 49.091067 | 299.629582 |
| 4096² FailClosed | 66.783068 | 157.156301 | 89.837583 | 184.235993 |

## 资源与不可外推范围

正式 baseline 墙钟 10.756957–11.907867 s、user CPU 10.479516–11.636068 s、system CPU 0.181095–0.252543 s；candidate 分别 8.436602–13.657236 s、8.230655–13.207877 s、0.160394–0.540777 s。14 个进程 peak RSS 均为 134816 KiB；逐次数值/上下文切换在 results.json。首次运行资源单列在各版 benchmark-attempt-1.json。

全进程资源包含 production、frozen、fixture 拷贝、预热、断言与启动开销，不能归因为 production 单臂 CPU/RSS 节省。本记录没有 realtime/deadline、guard 闭环、硬件、控制周期、端到端发布或锁竞争资格结论。

## 证据身份与复现

证据根目录 `Q=/workspace/shared/neh-rectangle-range-20261006`。baseline/candidate 各保留源码快照、build.sh/jsonl/log/exit、构建前后 hash 校验、二进制与 identity；matched/01-baseline.log 至 07-candidate.log 为 14 份正式日志。`matched/results.json` 包含两臂所有整数 ns、批次均值、比值、资源及日志 SHA；`compare.py` 保存采样顺序与算法。这些是本地证据路径，不是已发布网页。

| 对象 | SHA-256 |
| --- | --- |
| 基线 forbidden_zones.rs | `2e2a09d1babd442f01d3796fd6a32299d148a0044f3846de9339d6c99a8b3931` |
| 候选 forbidden_zones.rs | `291eabe6a22cd6335c9dca2971c0b4e6aac1d1e319d4609fd80164c68cc85361` |
| 共同 mapping_adapter.rs | `de3bc5fc239f96e82aa868a513c4ae56793e1516efd50c6ca1608660ae0b0c48` |
| 共同冻结 virtual 测试/基准 | `426bf0c68aaf7fe4701582dfb70ec05d06832796a0ee8c87897bd9b4cdcb6911` |
| 共同冻结旧 compose | `1a6393b4c01d8e380d6c06756554e4c727d786a9656d6415ba91b35256376da1` |
| 基线二进制（124551160 bytes） | `661de8d694f3b387e55414ce47a360dea431a676f59287e462153fd91bdb9a05` |
| 候选二进制（124557344 bytes） | `97165adb5c8bfef1dd9c056adac67dd94fb111165da890374c25fb85bc3988cd` |
| 正式 matched/results.json | `01c5fd22d7efe82c6c19f217b0298f0b5c42a96bda7dd7f841601356fa04e454` |
| compare.py | `1a42461023db9d80fff5b7e108689fe9444a0c8a42c999310aa16594084c0606` |
| 基线首次 benchmark log | `feec3ecb90bdcd710482067ccf39963a32b451bbd47a0ca6becbe2413d22f77c` |
| 候选首次 benchmark log | `bdbba78bf54aaadd7508f57e1b5a74a9f9d8065a4cd088aa13b6ab00fbd67560` |

两版 identity 的 source_hashes 还固定了两份新 corpus、lcm_runtime.rs、Cargo.lock、tomogram.rs。重建必须匹配所选版源码快照及这些 hashes；不能拿后续加入隔离 harness 的二进制冒充上述身份。实际共同构建命令：

```bash
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
Q=/workspace/shared/neh-rectangle-range-20261006
cd "$REPO/services/geometry/nuwax_geometry_runtime"
cargo test --locked --offline --bin nuwax-geometry-runtime \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  --no-run --message-format=json
"$Q/baseline/nuwax-geometry-runtime-test" rectangle_scan --nocapture --test-threads=1
# Each candidate selector is recorded verbatim in candidate/focused-results.json:
"$Q/candidate/nuwax-geometry-runtime-test" "$selector" --exact --nocapture --test-threads=1
# The same benchmark command is used for both preserved executables:
"$Q/$stage/nuwax-geometry-runtime-test" \
  mapping_adapter::tests::virtual_mask_copy_trial::copy_cost_baseline \
  --exact --ignored --nocapture --test-threads=1
```

正式采样实际执行 `python "$Q/compare.py"`；其 `xb` 写日志模式会拒绝覆盖已有证据。复现实验应使用新的证据目录及相应脚本路径，保持 7 对顺序、相同 executable identities、样本数和所有计时参数不变。源码范围可直接核对 `candidate/production-only.patch`。本资格记录不授权提前提交或推送。
