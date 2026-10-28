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

以下记录 2026-10-13 的 BL2/AR3 阶段。2026-10-14 的来源保留升级统一新写 Manifest v4/AR4，并增加 BL3；配对部署、旧数据读取及回滚边界见 [地面观测来源归档升级](depth-ground-provenance-archive-20261014.md)。

携带层观测的新增格式为 BL2、manifest v3、archive AR3（纯几何 archive 仍可为 AR2）；新读端保留 BL1、旧 manifest v2、AR1/AR2 读取。旧资料不会被捏造出真实点总体。AR3 的 assigned artifact identity schema 仍为 2；传输、保存、重开、重上传需保持既有身份绑定。**旧二进制不保证能读新格式**，回滚前应保留旧程序可读的地图备份；本次未访问、迁移或覆写用户地图。

Python 读端只做 wire shape、有界数组、字段名、整数范围、有限数值及既有 archive/checkpoint 身份约束；物理层算法语义由 Rust owner 负责，Python 不重复拟合或质量判定。

CPython **3.8.10 实际运行**通过 21 项 archive、8 项 scalar/vector、43 项 checkpoint，共 **72 项**，并通过 Rust Writer 生成 AR3 的跨语言读取。环境使用 grpcio 1.70.0、protobuf 5.29.6、numpy 1.24.4、pydantic 2.10.6。测试仅隔离 `services` 包初始化中无关的 cv2 副作用；实际 archive、provider、store、protobuf 模块均真实加载。这不是只做 3.8 语法解析，也不是完整 Web 服务启动验证。

## 已完成与未完成验收

以下来自保留日志；ignored 不计作通过。

| 项目 | 已观察结果 | 范围/限制 |
| --- | --- | --- |
| 底层来源与协议 | map-chunks 57/57；codec 14/14；source 12/12 + live 发布 1/1 | 原 PGO 10 秒预算在 debug 仍失败；本节下方另列真实 release 对照 |
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
| Stack qualification feature | release opt3 编译成功；普通回归 15/15、几何预检 1/1、源健康启动正负例 1/1；最终六项 live 场景 6/6 | 先修正 fixture 的独立 recent 源接线，再修复首次交付旧位姿缺口；原 wall OFF 碰撞失败保留，详见下节 |
| release A/B 性能 | 五对独立进程，10/10 断言通过 | A 按预期耗尽预算；B 返回有效路径，详见下节 |

五项 UDS 过滤不能表述为忽略后“全通过”：独立重试仍在 bind 时出现 `PermissionDenied: Operation not permitted`。需要允许 Unix socket 的环境重跑。未执行全 workspace、实机传感器回放/运动闭环、实机 CPU/延迟长时采样及整 Web 服务启动验收。**不能据此宣称实机 98% CPU 问题已解决。**

### 完整表面恢复资格：独立 live 源接线与保留失败（2026-10-13）

使用原 workspace release 配置（opt3、thin LTO、codegen-units=1），只构建 `nuwax-nav-stack --lib --features mls,guard-runtime,surface-recovery-qualification` 的必要依赖图，复用现有 target；未 clean 或全 workspace 重建。每个场景独立进程、串行运行，保留原 60 秒窗口和全部 oracle。

第一次六项都在初始运动前超时：调度器为 Idle、无路径、无运动指令，清理成功。原因是测试保留了一秒源健康要求，但 feeder 只发布静态 fine map 和 pose，未发布新增的 `RecentCloud3D`。生产调度器按要求把缺失的 live 源标为不可用，静态地图不能续期它。这六项原失败保留，不计作通过。

修正仅在 qualification fixture：每次新合成扫描用当前采样 voxel 的中心返回同时发布独立 recent cloud；保留所有障碍点、采样顺序、真实采样时间、对应 pose 序号/时间和地图 authority。数据来自该次扫描，不读取 Writer 累积历史，也不把地图重放变成健康扫描。原一秒 TTL、碰撞与停车判据、目标容差不变。一个 1.25 秒的启动回归证实：只有地图/pose 时同一目标不准入；接入新 live 扫描后同一目标开始运动，取消有相关联的归零回执。

仅修正 fixture、尚未修复首次交付缺口时，普通 qualification 回归 15 passed / 8 ignored；显式几何预检 1/1；六项原 live 场景结果：

| 场景 | 结果 | 测试进程 wall |
| --- | --- | --- |
| 已知落差恢复，局部规划 OFF | passed；恢复原目标到达 | 6.71 s |
| 已知落差恢复，局部规划 ON | passed；恢复原目标到达 | 7.14 s |
| 持续 box 障碍，局部规划 OFF | passed；绕行原目标到达，连续 oracle 通过 | 11.60 s |
| 持续 box 障碍，局部规划 ON | passed；绕行原目标到达，连续 oracle 通过 | 12.57 s |
| 持续 wall 障碍，局部规划 OFF | **failed**；连续碰撞 oracle 检出实际轨迹穿入 | 8.95 s |
| 持续 wall 障碍，局部规划 ON | passed；保持阻挡等待，连续 oracle 通过 | 10.34 s |

wall OFF 已保持零速等待五秒，但此前实际 +0.05 m/s 指令从 x=1.8476569601 保持 49.590924 ms，归零时 x=1.8501365063。独立连续 oracle 在实际 segment 558 检出约 **0.136476 mm** 穿入（SAT gap=-0.000136476498 m）。原 epsilon=1e-8 未变，不能因穿入较小而忽略。源期限尚未过期；清理成功不能把轨迹失败算成通过。确切 guard 计算位姿与首次交付的时序还需定向诊断，不能据此宣布全部运动安全资格通过。

后续两个确定性测试用真实 RouteFollower/机体守卫、仅屏障住发布时机，复现了首次交付缺口：从 x=.533 计算得到 +.1 m/s，已有速度在 80 ms 内移动到 x=.541；新位姿在发布前或发布后到达时，worker 仍交付旧候选，原 250 ms 窗口尚余约 167 ms，但在最新位姿执行同一完整停车扫掠会失败。两项原 desired-safety 断言均失败，日志保留。它证明真实旧位姿首次交付缺口，但不能伪称已捕获上述 wall 进程的确切内部 guard 计算位姿。

生产修正只复核已选命令的一条完整停车轨迹：同一工作线程接纳最新已收位姿与实际 applied velocity，用原机体查询重新复核；不重新枚举候选。新位姿到达时主动唤醒这条复核，在交付处仍未跟上最新状态时等待；原选择开始时间与最早来源期限不续期。原 recent-only 复核与新 current-state 复核共享同一停车查询。时间戳单独变化不使同一几何姿态失效。两个旧 synthetic 单测原先明确允许旧位姿首次交付，已迁移为同姿态 provenance 保持和新姿态必须重算，同时保留一次消费、过期和终态断言。

修正后首次交付两项缺口回归通过；latest-recent 五项通过，覆盖微抖、已选复核、障碍拒绝、当前位姿复核与原期限不续期。40 Hz 位姿 / 20 Hz 控制的 1.2 s 理想移动回归记录 48 个位姿、24 拍控制、24 次非零输出、0.544 m 行程；47 次单轨迹复核多数 0.2–0.9 ms、最大 2.687 ms，完整候选选择 25 次。这是该 fixture 的实际无饥饿证据，不是硬 20 Hz 长尾保证。

最后一组独立进程完整 live 资格 **6 passed / 0 failed**：support OFF/ON 为 6.76 / 8.37 s，box OFF/ON 为 10.71 / 14.04 s，wall OFF/ON 为 7.41 / 7.74 s。原 60 秒窗口、1 秒源 TTL、.1 m 位置/.15 rad 朝向容差与连续 oracle 的 epsilon 均未改。wall OFF 此次通过原连续碰撞审计并保持障碍等待五秒；没有把旧失败改写成通过。

最终 Runtime release 全 lib（同五项真实 UDS 过滤）为 **226 passed / 0 failed / 47 ignored / 5 filtered**；另显式运行首次交付两项与已消费指令 replay 两项，均通过。最终二进制上的 live 源启动正负例重跑通过（1.20 s）。这些 focused/ignored 结果另记，不与全 lib 计数重复相加。

该复核以“最新已收到的位姿”为依据，不代表已证明传感器位姿延迟或真实执行器激活后的物理位置误差全部解决。

这些是同一真实 liblcm memq context、生产 planner/worker/scheduler 与理想速度保持器的云端软件测试，不是物理制动、实机避障、实机 CPU、UDS 或硬实时证明。

## 性能诊断与保留失败

原 PGO Raytracing `background_catchup_preserves_replay_cadence_maps_and_checkpoint` 的 **debug 配置下 10 秒预算仍失败**，没有调大预算掩盖失败。后续真实 release 基线与候选都通过，不能把该 debug 失败直接表述为生产配置也超时。独占 debug 对照中关闭 layer capture 也失败：31 帧 × 2993 点的 workload 仅处理 14 帧便超时；capture 关/开分别约 10.201/10.307 秒。诊断测试自身通过只说明成功记录 A/B，不代表原 PGO 验收通过，也不足以断言全部开销来源。

同组 Voxel capture 开为 8.94/8.46 秒，关为 7.65/7.67 秒。这是特定 fixture 的单次 debug 诊断样本，不能宣称普遍加速，也不能外推实机 CPU。release 局部导引 A/B 已完成五对新进程测量，按 AB / BA 交替顺序。两组使用同一 release/profile 二进制、同一合成固定起点场景；A 用静态 PCT 导引，B 从五个真实近期返回构建 request-local 导引。十次测试进程均 exit 0，其中 A 每次按预期返回 `BudgetExhausted`（256 展开、642 nodes），B 每次返回 186 poses，并通过真实机体逐段二次扫掠。

| release 测量口径 | A 中位数（范围），ms | B 中位数（范围），ms |
| --- | --- | --- |
| guide + Scan（`total_ms`） | 84.62（71.32–125.40） | 6.70（6.54–7.14） |
| 全测试进程 wall | 148.30（121.26–213.22） | 62.38（59.56–66.25） |

B 的 guide 中位数为 1.67 ms、Scan 为 5.00 ms。公共 Scan 沿导引直接成功，fallback 搜索调用与展开均为零；它与手工 witness 导引直接调用局部搜索得到“六次展开”的早期诊断不是同一测量路径。

`total_ms` 排除 fixture / 静态图构建及输出后二次扫掠，进程 wall 包含这些工作。该 A/B 比较的是同版本算法导引选择，**不是旧 commit 对新 commit 的基准**；A 失败、B 成功的输出语义不同，不能表述成等效任务普遍加速。五对合成样本不能外推实机 CPU、运动闭环、长尾延迟或全平台性能。

### PGO 法线重算热点与真实 release 对照（2026-10-13）

后续诊断保留原失败日志、10 秒预算和完整占据图、点云、checkpoint 等价断言。31 帧 × 2993 点的**独立串行射线近似场景**中，debug 法线 refresh 占射线更新 13.962 / 15.297 秒（91.3%）；这不是完整 runtime，也不能用这组绝对时间代表 PGO 验收。

唯一生产算法调整是复用 IRLS 最后一个已完成迭代的特征分解，避免对同一 covariance 再做一次分解。权重不足提前退出时保留上一次已完成结果；低点数、低支持率、planarity / linearity / scattering 阈值、特征值排序、射线节拍及地图提交均不变，没有新增生产验证、锁、哈希或 clone。独立原实现参考逐 bit 对比法线与 recency，覆盖低点数、零/线/并列/倾斜协方差、低支持率、权重下溢提前退出和 128 组确定性邻域。射线 crate debug 与同样 full-LTO release 的单元测试各 35 passed / 1 diagnostic ignored，包含上述逐 bit 等价测试。该 crate 原有 standalone lock 需要更新；本轮在临时源码副本中用 geometry lock 作为离线解析起点运行相同源码的测试，没有修改仓库 manifests / lockfiles。

使用 geometry 自身 release 配置（opt-level=3、full LTO、codegen-units=1；没有覆盖 profile），先保存未改算法的基线二进制，再增量构建候选。首次该 profile 构建复用共享 target，但需要补齐 native PGO/GTSAM release 依赖；后续候选只重编射线库与 geometry 测试二进制，没有清理 target 或重建整个 workspace。原测试在**基线即已通过**，一次完整测试进程为 3.12 秒。

五对 AB / BA 交替的新进程运行同一个原始 PGO 测试，十次全部通过原断言与预算。每次包含预期完整重放与分批追赶，两者合计 62 个 Raytracing 帧：

| 口径 | 基线 A 中位数（范围），ms | 候选 B 中位数（范围），ms |
| --- | --- | --- |
| Raytracing update 阶段累计 | 649.278（635.140–736.060） | 602.719（582.319–678.097） |
| 完整测试进程 wall | 3234.801（3185.699–3701.892） | 3167.534（3146.382–3647.818） |

update 中位数降低 7.17%，进程 wall 中位数降低 2.08%，但范围有重叠。prepare / dynamic scene 两个控制阶段中位数为 30.243→30.308 / 227.749→230.063 ms。候选 release PGO 组为 9 passed / 5 ignored；surface-owner 组 5/5。capture 开/关诊断各次均完成 31 帧，候选 Raytracing 开为 935–952 ms、关为 908–917 ms。

这只支持删除一次重复特征分解带来的局部收益，不证明普遍加速，也不证明实机 98% CPU、长尾延迟或运动闭环问题已经解决。候选 debug 原测试复跑仍在 Raytracing 处理第 16 帧时返回 `BudgetExceeded`（测试进程共 27.75 秒）；没有把失败记作通过。debug 的预算失败与 release 的通过必须分别保留。

最终格式化后正常生产编译检查（非测试目标）已通过 nav-stack 的 `simple-2d,replanning-astar` 组合，以及 geometry `--bin nuwax-geometry-runtime`。首次 geometry release 测试编译另外暴露一处跨 crate 调用 test-only setter 的旧 fixture：只删除该无效调用，保留各点 observed_ns、完整源标记和所有断言；上述 surface-owner 5/5 覆盖此修复。

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
# 完整 qualification 使用 release opt3；下面仅构建，执行必须另选 exact 场景
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 -p nuwax-nav-stack --lib --release --features mls,guard-runtime,surface-recovery-qualification --no-run
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime planar_artifacts::assigned_identity_tests -- --test-threads=1
# 原预算失败复现，不应为获得通过而修改预算
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --bin nuwax-geometry-runtime pgo_map::tests::background_catchup_preserves_replay_cadence_maps_and_checkpoint -- --exact --test-threads=1
# 真实生产优化配置下运行同一原测试；与上面的 debug 结果分开记录
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked -j2 --release --bin nuwax-geometry-runtime pgo_map::tests::background_catchup_preserves_replay_cadence_maps_and_checkpoint -- --exact --test-threads=1
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
