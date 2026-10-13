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
