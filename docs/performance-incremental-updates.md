# 增量更新性能记录

## SurfaceMap 脏列无变化路径（2026-10-13）

基线为 `29384e3d8ae420e4b47b49d68b992d03bd66280e` 的 `SurfaceMapBuilder::build_column_updates`。原实现即使发现列没有变化，也先分配高度索引向量、删除并重新插入持久化占据/高度集合；最终复用快照不能收回前面已经付出的分配和集合页复制成本。

候选在原有输入验证过程中逐项比较借用的旧列和高度范围，仅对真正变化的列分配和替换。保留重复列、排序、有限数值和范围校验，保留整列删除、精度变化、reset 失败恢复和旧快照不可变语义；没有改变几何算法、阈值、地图版本或观测有效性，也没有新增哈希身份或额外整图比较。

### 可复现诊断

新增示例：`services/nuwax-workspace/crates/nuwax-nav-mls-planner/examples/surface_dirty_columns.rs`。在相同源码中分别使用基线和候选的生产函数，保留完全相同的示例，再各构建并保存独立可执行文件：

```sh
cd services/nuwax-workspace
cargo build --release --offline -p nuwax-nav-mls-planner --example surface_dirty_columns
# 在基线/候选构建后分别复制 target/release/examples/surface_dirty_columns
# 到 surface-dirty-baseline / surface-dirty-candidate，再交替运行：
for i in 1 2 3 4 5; do
  if ((i % 2)); then order='baseline candidate'; else order='candidate baseline'; fi
  for kind in $order; do
    ./surface-dirty-$kind > surface-dirty-pair-$i-$kind.jsonl
  done
done
cargo test --release --offline -p nuwax-nav-mls-planner --lib
```

实际使用原 workspace release 配置（opt3、thin LTO、codegen-units=1）及已有 target；没有 clean 或整仓重建。五对独立进程按上述顺序交替运行，测量期间无其他 Cargo/CPU 基准并发。其他工作线程的 codec 接口/测试增量不在此示例执行路径内。

每个进程建立 4,096 / 65,536 个规划列，提交 1,024 列脏块批次，分别测量全部不变和仅一列增加/删除支撑面下方占据体素。每种尺寸/模式运行 20 次，共 800 次测量调用。示例断言真实变化列数、构建路径、支撑单元数不变和最初快照保留。它特意隔离“多数脏列未变”的开销，不代表所有变化密度。

全局计数分配器记录每次调用的分配次数和请求字节；它们不是峰值/保留内存，计数原子操作也会影响绝对用时。下表为五个进程各自中位数的中位数：

| 原地图列数 | 批次 | 基线 µs | 候选 µs | 基线分配次数 | 候选分配次数 | 基线分配字节 | 候选分配字节 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 4,096 | 全部不变 | 1,839.322 | 54.529 | 5,843 | 10 | 2,326,180 | 36,988 |
| 4,096 | 一列变化 | 1,640.841 | 59.035 | 5,854 | 37 | 2,327,611 | 48,671 |
| 65,536 | 全部不变 | 4,863.935 | 79.829 | 13,316 | 10 | 4,675,688 | 36,988 |
| 65,536 | 一列变化 | 4,550.344 | 95.981 | 13,330 | 43 | 4,679,215 | 52,311 |

### 正确性与范围

最终 MLS release 全 lib：**312 passed / 0 failed / 15 ignored / 0 filtered**，测试执行 5.29 秒。ignored 诊断未运行，不计入通过。新增回归覆盖不变快照复用、多列不变加一列完整删除与全量参考一致、后续重复列报错不提交先前候选删除、旧快照保持，以及源/规划精度相同情况下仍校验非法高度范围。既有测试覆盖精细高度变化、reset 失败恢复、精度变化、删除/重连、机体限制和前沿变化。

这是云端合成增量批次的定向优化，不是完整地图流水线、全平台或实机 CPU 验收；不能据此声称已解决实机 98% CPU、实时期限或传感器回放问题。

## 嵌套点云解码去除完整临时副本（2026-10-13）

`LioCloudEnvelope`（包括独立 `RecentCloud3D` 内嵌的 envelope）原先把整个 `PointCloud2` body 复制到临时向量并添加八字节 fingerprint，再调用顶层解码器。`NavigationMap3D` 原先先 `to_vec()`，再复制到添加 fingerprint 的第二个向量。这些 fingerprint 是解码器自行补入的常量，不是收到的嵌套协议字段。

现在顶层和嵌套解码复用同一 body parser，直接借用已有 body。顶层 `PointCloud2`、LIO 和地图消息仍验证各自收到的 fingerprint；字段解析、边界、trailing bytes 及 envelope/domain 验证均保留。最终拥有型 `PointCloud2.data` 仍分配一次，未改成借用型 API。按代码路径计算，LIO/recent 每次减少一个完整 body 临时分配/复制，NavigationMap 每次减少两个；这不是运行时全局分配器采样，也不代表总内存峰值。

没有改动原始扫描时间、序号、独立源健康、地图 authority、近期帧 retention、无新 map commit 时发布的行为，也没有改动源锁或消费者/机体几何算法。已有 publisher 已直接序列化借用的 envelope，本次没有重复重写它。

### release 基线与测量

先加入相同的测试/诊断和共享 parser，再保留两个旧嵌套复制调用点构建并保存基线二进制；随后只替换这两个调用点、删除旧 LIO helper，保存候选。采用原 workspace release（opt3、thin LTO、codegen-units=1）及同一共享 target，未 clean 或整仓重建。构建和计时与其他工作串行。

两个 ignored 诊断分别使用 100,000 个 XYZ 返回（1,200,000 字节点数据），每次进程解码 2,000 次；开始前断言完整往返 DTO 相同。时间包含解码、原有验证和返回值销毁，不包含 wire 构造。每个诊断五对新进程按 AB/BA 交替，未使用分配计数器。

第一组五对结果（每个进程解码循环毫秒）：

| 入口 | 基线中位数（范围），ms | 候选中位数（范围），ms |
| --- | ---: | ---: |
| RecentCloud3D | 367.817（332.531–382.673） | 143.995（139.083–156.034） |
| NavigationMap3D | 2,107.109（2,061.481–2,461.599） | 2,557.651（1,605.059–5,242.127） |

RecentCloud3D 该组中位数降低 60.85%。NavigationMap 第一组波动大且候选中位数更慢，结果保留，不能宣称该组通过性能改善验收。

随后另做五对新进程，同时用父进程 `resource.getrusage(RUSAGE_CHILDREN)` 记录子进程 CPU 时间；不删除第一组。第二组解码循环中位数 Recent 为 366.810→149.230 ms、NavigationMap 为 1,854.343→1,573.380 ms。整测试子进程 CPU 时间（含 fixture 与起始往返断言）为：

| 入口 | 基线中位数（范围），ms | 候选中位数（范围），ms |
| --- | ---: | ---: |
| RecentCloud3D | 381.591（353.827–440.055） | 162.599（148.349–238.613） |
| NavigationMap3D | 1,861.646（1,833.069–2,185.178） | 1,581.176（1,503.905–1,721.954） |

第二组 CPU 中位数分别降低 57.39% / 15.07%，但 NavigationMap 两组表现不一致，不能省略第一组或外推普遍加速。确证收益是删除上述完整临时副本；近期点云的两组本地解码结果支持该定向优化。这里没有测量传输、源接纳、地图累积、规划流水线、实时长尾或实机 CPU。

复现入口（先由 Cargo JSON 获取、分别保存基线和候选真实测试二进制，再串行交替执行）：

```sh
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 \
  --release -p nuwax-codec --lib --no-run --message-format=json
./codec-binary lcm::recent_cloud::tests::recent_nested_cloud_decode_cost \
  --ignored --exact --nocapture --test-threads=1
./codec-binary lcm::schema::navigation_map_3d_tests::navigation_nested_cloud_decode_cost \
  --ignored --exact --nocapture --test-threads=1
./codec-binary --test-threads=1
```

### 协议与兼容回归

新增回归覆盖空点云与空 body 拒绝、混合字段布局/行 padding/大小端标记保留、所有截断长度、负值/超大数组长度、trailing bytes、顶层和 LIO 外层错误 fingerprint；既有 recent 原时间/成员索引与 Python 往返继续运行。跨语言测试使用实际 CPython 3.8.10。

基线与候选首次 full lib 均为 **108 passed / 1 failed / 3 ignored**。失败在既有 fresh-generated bindings 字节比对：仓库 `NavigationState.py` 末尾比 lcm-gen 1.5.2 输出少一个空行。首次未配置 native C include/lib 路径时另外两项编译失败，补齐 `CPATH` / `LIBRARY_PATH` 后通过；这些环境失败没有算作成功。仅恢复该 Python 文件末尾空行，不改 schema、字段或 Python 行为，后续复跑结果另记。

恢复 exact-generated 末尾空行后，最终同一 release codec 二进制全 lib 为 **109 passed / 0 failed / 3 ignored / 0 filtered**（2.47 秒）。包含原失败的 fresh C/Python 互操作测试，未跳过任何普通测试。该正确性复跑经协调在另一 crate 编译阶段运行，不用于性能比较。生成文件的必要 EOF 空行会触发普通 `git diff --check` 的 `new blank line at EOF` 提示；移除它会再次破坏原有 exact-generated 比对，不能当作手写代码尾空白删除。

## PlannerWorker 未变化输入不再重复复制（2026-10-13）

`PlannerWorker` 的机体查询/局部控制计算已在独立 `nav-local-compute` 线程、输入交换锁外执行；Scan/PCT 请求另由 `PlanningOwner` 线程承担。本轮没有增加线程或锁，也没有把候选搜索放回控制接收路径。

发现的确定性浪费在输入快照：每次控制请求及位姿/已执行速度追赶都会复制 `ExecutionMap`，随后仅在 map version 变化时才使用它。Surface 变体主要复制 Arc，但 Planar 的 `CostMap2D` 拥有完整 `Vec<u8>`，因此未变化的平面地图也在共享输入锁内被完整复制。边界向量同样在每次唤醒时复制，即使没有重新安装。

候选只在对应 map version 变化时复制地图，在 boundary version 或 goal version 变化时复制边界；goal 重设后的边界重新安装仍保留。位姿、applied velocity、地图版本和无地图状态继续按原路径交付。不改首次交付当前状态复核、recent-only 复核、候选生成/排名、原选择起点、最早来源期限及非零命令一次性消费。最后一条已选轨迹的完整停车查询仍执行，不能把它当作可删除的重复候选搜索。

### 平面快照合成测量

新增 `worker_snapshot_tests.rs`：使用真实 worker 和极简合成 planner，安装一次 2,048×2,048 的平面地图（4 MiB cells），先预热一次，再串行请求/消费 256 个结果。每次进程断言地图只安装一次、总共 257 次合成选择；此 planner 不做机体几何，不能给真实运动授予许可。另有普通回归核对连续请求不重复安装地图、goal 重设恢复原边界、位姿及地图更新后安装新版本。

先在仅加入同一测试的生产基线构建并保存二进制，再只应用上述输入复制调整构建候选；两次都包含本轮相同的 MLS/codec 依赖增量。使用原 workspace release 配置（opt3、thin LTO、codegen-units=1）、同一 target 和 `-j2`，没有 clean 或整仓重建。五对独立进程按 AB/BA 交替，计时无其他基准或 Cargo 并发，十次全部通过。

| 口径 | 基线中位数（范围），ms | 候选中位数（范围），ms |
| --- | ---: | ---: |
| 256 次 worker 请求/消费 | 64.415（58.120–65.625） | 4.165（3.091–5.422） |
| 整个测试进程 wall | 74.145（64.851–75.897） | 12.346（9.796–14.368） |

按生产代码路径推导，该定时段基线额外复制 256 次×4 MiB＝1 GiB 未变化的 cells，候选为零次未变化地图复制；首次安装所需复制仍存在。这是代码路径计数，不是分配器或峰值内存测量。边界只有三个顶点，不是该探针的主要负载。

测试线程等待结果时也记录获取同一输入锁的耗时。五个基线进程的各自最大采样值为 1.490–2.074 ms，候选为 0.000697–0.201 ms。采样次数由线程调度和进程进度决定，两组不同；这些值包括抢占/调度，既不是纯锁持有时间，也不能作为固定采样控制延迟分位数或硬实时上限。诊断日志保留原始样本数量和分位数，但不能用它们宣称生产 p95 改善。

该结果只证实删除不变平面地图复制在这个刻意隔离复制成本的 workload 中有效，不代表完整平面导航，更不代表近期表面障碍路径、实机 CPU 或 20 Hz 长尾资格。显示路径复制、旧 Arc 最后拥有者的销毁等其他候选点没有本轮测量，因此未扩展改写。

复现入口：

```sh
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked -j2 \
  --release -p nuwax-nav-runtime --lib --no-run --message-format=json
# 分别保存基线/候选 Cargo JSON 中的真实测试二进制，再 AB/BA 交替：
./runtime-binary planner_worker::tests::snapshot_tests::worker_unchanged_planar_snapshot_cost \
  --ignored --exact --nocapture --test-threads=1
./runtime-binary planner_worker::tests::snapshot_tests:: \
  --nocapture --test-threads=1
```

### 保留当前状态安全回归

最终 Runtime release lib 为 **227 passed / 0 failed / 48 ignored / 5 filtered**，52.50 秒。五个过滤项仍是此前环境不能 bind 的真实 UDS 测试；本轮未重试 UDS，也不能称完整 Runtime 全通过。比此前 226/47 新增一个普通输入安装回归和一个 ignored 性能探针；ignored 未混入通过数。

同一最终二进制另显式运行首次交付旧位姿两项（2/2）、已消费命令 replay 两项（2/2），全部通过。latest-recent 五项重跑 5/5；40 Hz 位姿 / 20 Hz 控制理想移动回归重跑 1/1，记录 48 个位姿、24 拍控制、24 个非零结果、0.54365 m 行程，25 次完整选择、47 次单轨迹复核。复核耗时 219–1,009 µs 是本次 fixture 样本，不能承诺实机期限。原期限不续期、一次消费、最新 recent 阻挡和新位姿复核断言均保留；这两组普通回归已包含在上述 lib 通过数中，重跑结果不重复累加。

```sh
./runtime-binary first_command_pose_change_ --ignored --nocapture --test-threads=1
./runtime-binary retained_command_replay_requires_current_full_braking_permission \
  --ignored --nocapture --test-threads=1
./runtime-binary retained_steady_command_replay_requires_current_full_braking_permission \
  --ignored --nocapture --test-threads=1
./runtime-binary latest_recent_ --nocapture --test-threads=1
./runtime-binary latest_received_pose_recheck_keeps_twenty_hz_moving_progress \
  --nocapture --test-threads=1
```

本轮未重跑六项完整 live Stack 资格，也未运行整 workspace、实机传感器/执行器闭环或实机 CPU 采样；此前 wall OFF 的原碰撞失败及其修复验收记录继续保留在 `terrain-recent-obstacles-refactor.md`。

## Map Writer 脏层字节账本增量维护（2026-10-13）

基线 `bb863f7a86e79408ef3ab5490bea4db0de19c724` 的 `Writer::load_dirty` 每次首次加载一个脏块，都会调用 `dirty_layer_bytes()` 遍历所有已加载脏块及其层记录，再判断是否需要 pressure flush。即使各块没有层，遍历脏块的成本仍在；一次批次连续加载 N 个新块会反复扫描此前的块，形成二次累计工作。

候选维护一个 `usize` 脏层字节总量：加载已有块时加该块层大小，成功替换层时减旧加新，删除占据成员时在原有 retain 遍历中扣除被删层；只有成功 flush/reset 才清零。失败 flush 的已冻结对象及其 dirty 账本都保留，非法或相同层替换不改账本。计费公式仍为每层 `128 + members.len()*2 + samples.len()*64`，不是实际堆容量或内存峰值；原来的除以 64、严格大于预算才 flush、点数计费、reader cache 和 pending bytes 均不变。未新增哈希身份、锁、整图验证或生产路径核对扫描。

### release 对照

先仅增加相同测试/诊断并保存生产基线 release 测试二进制，再实现计数器并保存候选。使用原 workspace release（opt3、thin LTO、codegen-units=1）、已有 target、`--locked --offline -j2`；未 clean 或整仓构建。五对新进程按 AB/BA 交替，计时无其他基准或 Cargo 并发。

诊断使用 64 MiB Writer 预算，先建立 1,024 或 8,192 个脏块，分别每块零层或一层（一个真实成员和一个来源样本）。计时一次 `observe`，加入同样数量的新块，每块一个占据命中。初始构造、层填充、最终断言及销毁不在计时内。最终断言所有新旧块/点数及字节账本准确，且没有编码/flush；因此定时段无对象存储写入，刻意隔离脏块 bookkeeping，不能代表所有层密度或完整地图导入。

| 初始块数 / 新块数 | 初始每块层数 | 基线中位数（范围），ms | 候选中位数（范围），ms |
| --- | ---: | ---: | ---: |
| 1,024 / 1,024 | 0 | 3.849（3.710–5.585） | 1.019（0.720–1.312） |
| 1,024 / 1,024 | 1 | 7.447（6.178–10.902） | 0.791（0.723–1.007） |
| 8,192 / 8,192 | 0 | 406.341（353.074–430.836） | 7.737（7.257–9.710） |
| 8,192 / 8,192 | 1 | 1,618.839（1,499.774–1,693.492） | 7.781（6.894–8.246） |

基线首次单次探索记录为 1,024 块零层 7.355 ms、有层 6.230 ms；8,192 块零层 409.999 ms、有层 1,548.336 ms。它不计入上述五对结果；不同进程有调度/缓存噪声，不能把单次极值作为实时承诺。

复现入口：

```sh
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked --offline -j2 \
  --release -p nuwax-map-chunks --lib --no-run --message-format=json
# 分别保存基线/候选 Cargo JSON 中 executable 的真实路径，然后五对 AB/BA：
./map-binary writer::accounting_tests::dirty_layer_accounting_cost \
  --ignored --exact --nocapture --test-threads=1
./map-binary --test-threads=1
```

### 正确性和边界

相同两项新增普通回归先在基线通过，最终候选 release 全 lib 为 **59 passed / 0 failed / 1 ignored / 0 filtered**（1.58 秒）；ignored 诊断另显式运行，不混入普通通过数。测试中的独立全扫描 oracle 覆盖加载、增大/缩小替换、非法/相同替换、同源时间无变化、删除最后层及最后占据、失败 flush 与原冻结 ID 重试保留、成功 flush 后重新加载、reset，以及预算恰好相等不 flush、再加入一个点后按原边界 flush。既有层 geometry/source、restore、frame rehome、归档和保留对象回归一并运行。

这里确证的是脏块记账消除重复扫描。未测量 allocator、实际峰值/保留内存、完整地图流水线、实机 CPU 或 20 Hz 控制长尾，不扩展为这些指标的验收结论。

## 四项增量优化后的集成复验（2026-10-13）

在本篇四项优化全部合入后的同一代码状态，复用既有 target、Rust 1.88.0 和锁文件，执行最终普通生产二进制检查；未 clean、未构建整 workspace，也未修改任何生产配置或资格阈值：

- `nuwax-nav-stack --bin nuwax-nav-stack --features simple-2d,replanning-astar`：`cargo check --locked --offline -j2` 通过，4.90 秒
- 独立 geometry manifest 的 `--bin nuwax-geometry-runtime`：同样检查通过，24.59 秒
- 两项都是非 `cfg(test)` 的生产编译检查，不是将测试二进制编译当作生产检查；既有 warning 保留，未发现新的编译错误

随后对 `nuwax-nav-stack --lib --features mls,guard-runtime,surface-recovery-qualification` 进行 release 增量构建（2 分 58 秒），使用原 opt3、thin LTO、codegen-units=1，没有覆盖 profile。由此次 Cargo JSON 提取真实测试二进制，确认 test profile 的 opt_level 为 3；编译结束后，所有场景独立进程串行执行，无其他 Cargo/性能探针并发。

普通 qualification 回归 **15 passed / 0 failed / 8 ignored**；显式 source-health 启动正负例 **1/1**（1.21 秒），原 codec/geometry/body witness preflight **1/1**（0.90 秒）。六项完整 live 场景全部显式 `--ignored --exact` 执行，结果 **6 passed / 0 failed**：

| 场景 | 局部规划 OFF | 局部规划 ON |
| --- | ---: | ---: |
| 已知落差支撑恢复并到达原目标 | 7.60 秒，通过 | 10.13 秒，通过 |
| 持续 box 障碍 | 12.85 秒，通过 | 16.91 秒，通过 |
| 持续 wall 障碍 | 9.19 秒，通过 | 10.43 秒，通过 |

表中时间为测试框架记录的场景耗时，不是算法耗时或优化前后配对基准。原 60 秒窗口、1 秒源 TTL、0.1 m 位置 / 0.15 rad 朝向容差、五秒障碍等待、连续碰撞/停车 oracle 及 epsilon 均未改变。此次复验补足前文 PlannerWorker 单项阶段尚未重跑六项资格的范围；没有将普通回归的八个 ignored 自动计入通过，也没有把先前 wall OFF 的碰撞失败改写成成功。

复现使用上述生产检查命令对应的 manifest，并运行：

```sh
cargo +1.88.0 test --manifest-path services/nuwax-workspace/Cargo.toml \
  --locked --offline -j2 -p nuwax-nav-stack --lib --release \
  --features mls,guard-runtime,surface-recovery-qualification \
  --no-run --message-format=json
# 从本次 Cargo JSON 提取 opt_level=3 的 nuwax_nav_stack 测试 executable 后：
./stack-binary qualification --nocapture --test-threads=1
prefix=lcm_dataflow::surface_dataflow::tests
./stack-binary "$prefix::support_recovery_qualification::live_source_is_required_before_goal_admission" --ignored --exact --nocapture --test-threads=1
./stack-binary "$prefix::dynamic_occupancy_qualification::original_codec_geometry_and_body_witness_preflight" --ignored --exact --nocapture --test-threads=1
for flag in off on; do
  ./stack-binary "$prefix::support_recovery_qualification::original_goal_known_drop_recovers_and_arrives_local_planning_$flag" --ignored --exact --nocapture --test-threads=1 || exit "$?"
done
for scene in box wall; do
  for flag in off on; do
    ./stack-binary "$prefix::dynamic_occupancy_qualification::dynamic_${scene}_local_planning_$flag" --ignored --exact --nocapture --test-threads=1 || exit "$?"
  done
done
```

原始 production check 输出、此次 build JSON、binary provenance、精确选择器、每场景独立日志及进程退出码均已保留。本轮没有再次运行完整 Runtime lib；此前 **5 项真实 UDS 仍受环境限制**，不能据上述结果宣称 UDS 已通过。以上是云端合成闭环集成回归，不是实机传感器/执行器闭环、实机 CPU/峰值内存、硬实时或长时 20 Hz 延迟验收，也不是整 workspace 全测试。

## 最新实际位姿复核保留已构造的当前视图（2026-10-13）

`SurfaceFollower::recheck_selected_body_command_from` 对最新实际位姿执行完整停车轨迹复核时，会强制从当前 `source_map` 构造机体支撑视图。此前复核结束总会还原进入函数前的 map 和 dirty 状态；当新位姿已置 dirty 时，刚构造并实际查询过的视图随即丢弃，下一次普通视图刷新又构造同一实际站姿的视图。

现在只有查询旧选择位姿的分支保存并还原临时视图；使用最新实际位姿的分支保留刚构造的视图。**每次复核仍强制刷新并执行原完整停车轨迹检查**，不跳过 geometry/recent 检查，不复用旧运动许可，不改变首次交付、原期限、一次消费、停止策略或字段合同。保留强制刷新也覆盖 signed-zero 位姿变化，不能因为普通数值相等就跳过。源、近期障碍、观察和位姿变化仍由原 setter 失效；新视图始终从环境源派生，不累计旧足迹清除。没有新增锁、缓存表、摘要或版本身份。

### 实际 Runtime release 对照

云执行环境被替换后，本轮重新恢复仓库、Rust 1.88.0、锁文件依赖和 native LCM 1.3.1；未假称先前丢失的 target 仍存在。只构建 Runtime lib 及其依赖，没有整 workspace 构建或 clean。先加入同一诊断、保存生产未改的基线二进制，再做上述窄改动，保存候选；使用原 release opt3 / thin LTO / codegen-units=1 和 `--locked --offline -j2`。候选另增加四项普通所有权/拒绝回归，诊断的场景、采样和调用口径未变。

诊断 `current_pose_recheck_then_view_refresh_cost` 使用真实 `SurfaceFollower` 与完整停车检查，先通过真实选择得到非零命令，再在两个相距 1 mm 的实际位姿间交替复核。地图分别有 800 / 12,800 个地面列；每种尺寸都测完整观测地面，以及增加一个静态自体回波、需要派生支撑视图的场景。它不发布执行命令、不代表传感器回放，也没有把近期外部障碍当作可清除自体回波。

每进程每场景 30 次，计时包括复核及随后单独调用的普通视图刷新；初始构图、选中命令、打印和最终断言在计时外。五对新进程按 AB/BA 交替，编译及其他性能探针暂停，10/10 进程成功。下表为五个进程各自中位数的中位数，单位 ms；“合计”直接对每次两段之和取中位数，不是两列中位数相加。

| 地面列数 / 场景 | 复核：基线 → 候选 | 后续视图刷新：基线 → 候选 | 两段合计：基线 → 候选 |
| --- | ---: | ---: | ---: |
| 800 / 完整地面 | 0.011072 → 0.010981 | 0.000110 → 0.000020 | 0.011182 → 0.011006 |
| 800 / 自体回波支撑派生 | 17.669178 → 17.583954 | 5.689258 → 0.000260 | 23.853009 → 17.584206 |
| 12,800 / 完整地面 | 0.011207 → 0.011257 | 0.000110 → 0.000020 | 0.011332 → 0.011287 |
| 12,800 / 自体回波支撑派生 | 25.019875 → 24.670152 | 8.251041 → 0.000240 | 33.309234 → 24.670447 |

两个支撑派生场景的合计进程中位数范围分别为基线 22.824126–24.665455 / 候选 16.661772–18.046578 ms，以及基线 33.081164–33.790601 / 候选 22.471713–27.759561 ms。完整地面是负对照，本来就共享环境快照，绝对成本很低；不宣称普遍加速。复核本身仍包含约 5.8 / 8.1 ms 的强制视图重建，候选没有删除这道步骤。

### 回归及限制

新增四项普通回归全部通过，覆盖最新实际视图保留、旧选择位姿复核还原独立当前视图、旧足迹不累积、新近期障碍拒绝及选择清除、来源恢复必须重新准入、无地图不得恢复旧视图、signed-zero 仍强制刷新。既有源/观察变化、停止、失败、路径和边界测试包含在后续普通 lib 运行中。

候选完整、不加过滤的 Runtime release lib 结果为 **231 passed / 5 failed / 49 ignored / 0 filtered**，51.50 秒。五个失败都是此前同类真实 UDS fixture 在 bind 时返回 `PermissionDenied: Operation not permitted`；本轮确实重试了它们，不能记作通过或宣称完整 lib 成功。其余普通测试通过，新增 ignored 诊断另行显式运行，未混入普通通过数。

首次交付新位姿两项、已消费命令 replay 两项均显式通过。原 latest-recent 五项通过；40 Hz 位姿 / 20 Hz 控制移动回归记录 48 个位姿、24 拍控制、24 次非零结果、0.543624 m 行程、25 次完整选择和 47 次单轨迹复核，复核 215–970 µs。它们保留原断言与期限，重跑结果不重复累加到普通测试数。

复现入口：

```sh
cargo test --manifest-path services/nuwax-workspace/Cargo.toml --locked --offline -j2 \
  --release -p nuwax-nav-runtime --lib --no-run --message-format=json
# 从本次 Cargo JSON 保存基线/候选真实 executable，各运行五对 AB/BA：
./runtime-binary current_pose_recheck_then_view_refresh_cost --ignored --nocapture --test-threads=1
./runtime-binary current_view_recheck_diagnostic --nocapture --test-threads=1
./runtime-binary --test-threads=1
./runtime-binary first_command_pose_change_ --ignored --nocapture --test-threads=1
./runtime-binary retained_command_replay_requires_current_full_braking_permission --ignored --nocapture --test-threads=1
./runtime-binary retained_steady_command_replay_requires_current_full_braking_permission --ignored --nocapture --test-threads=1
./runtime-binary planner_worker::tests::latest_recent_ --nocapture --test-threads=1
./runtime-binary latest_received_pose_recheck_keeps_twenty_hz_moving_progress --nocapture --test-threads=1
```

这里测量的是支撑派生场景中“复核 + 下一次视图刷新”的冗余成本，后半段没有包含下一拍完整候选选择，不能表述为整个控制循环加速比例。没有完成实机 CPU/峰值内存、硬实时或长时 20 Hz 资格，也没有在本节重跑六项完整 live Stack 资格；这些边界和 UDS 失败必须保留。

另对候选执行非 `cfg(test)` 的 `cargo check --release --locked --offline -j2 -p nuwax-nav-runtime --lib`，通过，49.68 秒；这只是 Runtime 生产库编译检查，保留六条既有缺失文档 warning，不代替 Nav Stack 集成或上述失败的 UDS 验收。

### 当前状态视图保留后的完整 Stack 资格复验（2026-10-13）

本节是在上述 current-view 候选工作区执行的集成结果：基于已包含两项 Web/dense 优化的本地父版本，加上尚未提交的 `surface_body_control.rs` 视图保留改动及 `current_view_recheck_diagnostic.rs` 回归。测试时未加入后续 frontier/perception 优化。精确父版本与此次工作区差异另随原始构建证据保留，不能把本节结果转用于之后尚未验证的修改。

恢复后的云端检查确认，本组 Stack feature 依赖图不包含 PCL、GTSAM 或 native PGO，因此无需为这些场景恢复其缓存。使用 Rust 1.88.0、锁文件、离线模式、原 workspace release opt3 / thin LTO / codegen-units=1，仅构建 `nuwax-nav-stack --lib --features mls,guard-runtime,surface-recovery-qualification` 的必要依赖，1 分 41 秒成功；未 clean、未整仓构建、未覆盖 profile。由本次 Cargo JSON 提取实际测试 executable 并核对 opt_level=3。

编译完成后，普通 qualification **15 passed / 0 failed / 8 ignored**；显式源健康启动正负例 **1/1**，原 codec/geometry/body witness preflight **1/1**。六项原完整 live 场景在独立进程中串行执行，无其他 Cargo/计时 workload 并发，全部 **6 passed / 0 failed**：

| 场景 | 局部规划 OFF | 局部规划 ON |
| --- | ---: | ---: |
| 已知落差支撑恢复并到达原目标 | 6.60 秒，通过 | 7.62 秒，通过 |
| 持续 box 障碍 | 12.00 秒，通过 | 16.47 秒，通过 |
| 持续 wall 障碍 | 7.64 秒，通过 | 7.60 秒，通过 |

表中为测试框架记录的耗时；完整进程 wall 另有记录。每场使用原 `--ignored --exact --nocapture --test-threads=1` 选择器，沿用前文“四项增量优化后的集成复验”复现入口。原 60 秒窗口、1 秒源 TTL、位置/朝向容差、五秒等待及连续碰撞/停车 oracle 均未改动。普通回归的八个 ignored 不计入通过；此轮显式场景独立记录，不与 Runtime 单测重复累计。场景耗时不是优化前后配对性能结果。

这补足前一节 current-view 候选尚未完成的六项 Stack 资格，不抹去历史 wall OFF 碰撞失败，也不解除 **5 项真实 UDS bind 环境失败**。本次未重新执行完整 Runtime lib、geometry 生产二进制或整 workspace；没有实机传感器/执行器闭环、实机 CPU/内存、硬实时或长时 20 Hz 验收。云端合成闭环通过仅覆盖上述精确候选与场景。

## 前沿局部失效按既有列索引枚举（2026-10-13）

`surface/frontiers.rs` 已只对脏包围盒内单元计算前沿收益，但寻找这些单元仍遍历所有表面槽位并逐个读取坐标。小变化的候选只枚举包围盒内的既有 `surface_lookup` 列及该列全部高度；当矩形列数超过原有槽位数时，仍使用原来的槽位扫描，避免远隔变化在稀疏地图上枚举巨大空矩形。面积使用宽整数计算，坐标夹到已有 i32 定义域，范围双端包含。没有新增空间索引、哈希身份、线程或调度等待。

前沿收益、失效半径、合并延迟更新、禁区/边界兼容性和机器人可达性保持原逻辑。几何前沿集合仍会复制未受影响的旧候选，当前起点的可达成本仍需单独计算；本项不能宣称整个查询与地图大小无关。

新增 `frontier_dirty_bounds` 示例对两层 64×64 / 256×256 地图，各交替删除/重插中央列下层单元 20 次，只计时 `prepare_frontier_geometry`。构图、列更新、冷参考与可达性比较在计时之外；每种输入的所有候选/收益/代价在两层起点分别与冷构图完全相等，并验证原快照未改变。两种地图均重新评估 721/722 个单元；局部路径访问 361 列，原路径扫描 8,192 / 131,072 槽位。

基线先用原生产循环构建保存，随后仅更改枚举循环构建候选。两个二进制使用相同示例、同一 workspace release 配置和 Rust 1.88，未 clean；五对独立进程按 AB/BA 交替执行，测量期间无其他编译或 CPU 基准。以下为各进程 20 次调用中位数再取五进程中位数，单位 µs：

| 地图支撑单元 | 基线中位数（范围）µs | 候选中位数（范围）µs | 中位数降幅 |
| --- | ---: | ---: | ---: |
| 8,192 | 966.890（878.947–1,032.354） | 833.719（765.997–847.334） | 13.8% |
| 131,072 | 4,700.015（4,489.727–5,318.269） | 1,498.867（1,249.452–1,702.931） | 68.1% |

复现：

```sh
cd services/nuwax-workspace
cargo build --release --offline -j2 -p nuwax-nav-mls-planner --example frontier_dirty_bounds
# 分别保存原枚举与候选枚举版本的 target/release/examples/frontier_dirty_bounds 后交替运行
cargo test --release --offline -j2 -p nuwax-nav-mls-planner --lib
```

最终 MLS release 全 lib：**315 passed / 0 failed / 15 ignored / 0 filtered**，执行 4.16 秒。新增回归覆盖范围双端包含、负坐标、全 i32 定义域面积不会溢出、三层列完整删除/重插与槽位复用、旧快照不变，以及远隔脏列延迟累积后的稀疏回退；完整前沿索引均与冷重建一致。既有边界/虚拟障碍失效、占据证据变化和可达性测试亦通过。ignored 诊断未运行，不计入通过。

这是云端合成地图局部几何刷新测量，不含生产探索线程排队、导航可达搜索、传感器流水线或实机 CPU；未宣称完整平台性能验收。

### 近期障碍高度行查询优化后的最终导航集成复验（2026-10-13）

本轮验证已集成 current-view 保留、近期障碍高度行 query-local 优化及禁止区域身份修复的最终导航代码；在随后持续运行 soak fixture 加入前完成构建与执行。没有改动生产代码、资格 fixture 或阈值。

- 普通非 `cfg(test)` 的 `nuwax-nav-stack` 二进制：`cargo check --locked --offline -j2 -p nuwax-nav-stack --bin nuwax-nav-stack --features simple-2d,replanning-astar` 通过，1 分 08 秒
- Stack qualification release 增量构建通过，1 分 55 秒；由本次 Cargo JSON 提取实际 executable，核对 opt_level=3，使用原 thin LTO / codegen-units=1。复用既有 target，未 clean 或构建整 workspace
- 普通 qualification：**15 passed / 0 failed / 8 ignored**
- 显式源健康启动正负例 **1/1**；原 codec/geometry/body witness preflight **1/1**
- 六项原完整 live 场景：**6 passed / 0 failed**，每项独立进程、串行执行，无其他 Cargo 或计时 workload 并发

| 场景 | 局部规划 OFF | 局部规划 ON |
| --- | ---: | ---: |
| 已知落差支撑恢复并到达原目标 | 9.84 秒，通过 | 6.93 秒，通过 |
| 持续 box 障碍 | 12.54 秒，通过 | 12.23 秒，通过 |
| 持续 wall 障碍 | 7.38 秒，通过 | 8.22 秒，通过 |

上述为测试框架场景耗时，不是优化前后配对性能结果。命令沿用前文同一 release 构建入口及六项 `--ignored --exact --nocapture --test-threads=1` 选择器。原 60 秒窗口、1 秒源 TTL、位置/朝向容差、五秒等待和连续碰撞/停车 oracle 均未改变；原失败历史继续保留。

同一生产状态另重新构建并执行 Runtime 普通 release lib，**231 passed / 0 failed / 49 ignored / 5 filtered**，54.95 秒。仅显式排除以下五个此前因环境不能绑定 Unix socket 而失败的真实 UDS 项：

```text
tests::cancel_preempts_delayed_renew_over_real_uds_with_qualified_latency
tests::cancel_preempts_delayed_submit_over_real_uds_with_qualified_latency
tests::cancel_replays_identical_fence_after_first_response_timeout
tests::real_uds_adapter_requires_correlated_applied_and_fence_proofs
tests::server_deadline_renewal_keeps_legal_slow_commands_alive
```

这五项本轮没有重试，不能称 Runtime 无过滤全通过；49 ignored 未计入通过数。各次构建 JSON、二进制 provenance、完整命令、单场景日志及退出码均已保留。本轮未重跑 geometry 生产二进制、完整原生验收或整 workspace。以上也不替代长时 soak、实机传感器/执行器闭环、实机 CPU/内存、硬实时与控制延迟验收。
