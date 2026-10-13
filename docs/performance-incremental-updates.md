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
