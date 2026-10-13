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
