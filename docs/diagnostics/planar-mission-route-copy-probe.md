# 二维返航路线复制成本诊断（2026-10-13）

## 结论与范围

保留诊断，暂不修改生产所有权结构。本次只覆盖二维 Wavefront 探索的返航预算查询，不是三维机器人控制性能结果。500×500 图的每次占据数组复制为 500,000 字节，但热进程中的 Costmap 复制中位数仅 10.38–12.89 微秒；少量复制总字节较大，不代表同等规模的调度延迟。不能将阶段中位数简单相加，或将其解释成已实现的优化收益。

源码审计确认，三维 SurfaceSnapshot 使用 Arc 共享几何，SurfaceObservations 使用持久化 im 索引；本次没有发现值得以完整点云复制为目标的小型三维修改。二维移除第二次复制需要调整消费型规划器接口或引入占位状态，复杂度与本次测得的绝对成本不匹配。

## 执行与可复现条件

- 云执行器，2026-10-13 20:07 UTC；另一个持续测量任务释放独占 CPU 时段后运行，无并行构建或测试
- rustc 1.88.0 (6b00bc388 2025-06-23)，cargo 1.88.0 (873a06493 2025-05-10)
- 工作区 release 配置，thin LTO、codegen-units=1；没有新依赖或生产修改
- 测试：[mission_route_copy_probe.rs](../../services/nuwax-workspace/crates/nuwax-nav-simple-planner/tests/mission_route_copy_probe.rs)，默认 ignored，显式执行后 1 passed，运行 0.13 秒；编译 26.89 秒

实际命令（云工作区路径需按复现环境调整）：

```bash
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
cd /workspace/scratch/71cdb9d17732/neh-cloud-3d1901d/services/nuwax-workspace
cargo test -p nuwax-nav-simple-planner --release --test mission_route_copy_probe -- --ignored --nocapture --test-threads=1 > /workspace/scratch/71cdb9d17732/mission-route-copy-probe.log 2>&1
```

## 输入、计数和计时边界

每阶段每场景 32 次。同一进程顺序执行、重复使用同一源图，后续样本具有暖缓存效果；没有单独预热或剔除首样本，不是冷启动或大工作集随机访问测量。100×100 是较小对照，500×500 对齐现有探索归因诊断的尺寸；两者分辨率均为 0.2 米。每尺寸分别测试全自由图和 5 格未知边界加远端稀疏障碍图，后者高度表分别有 78 / 2,450 项。路线位于已知自由区域，起点 (4.1, 4.1)、候选 (6.1, 4.1)、home (4.1, 6.1)。机体为 0.5×0.3 米矩形。

字节数只统计占据数组的 len × size_of::<OccupancyCell>()；本次元素为 2 字节。分配次数是依据明确 Vec 复制调用推导的占据数组分配次数，并验证源/副本缓冲指针不同；没有分配器插桩，没有统计总分配器字节。Costmap 的高度表、阻塞集合、路线、配置及临时对象不计入这些字节数，但其实际操作包含在对应阶段计时内。

源码路径：explore_compute_backend.rs:574 复制一次准备图；:635–653 按 homeward、outward、return 顺序查询；explore_mission.rs:377 每次复制 Costmap；costmap.rs:188 的 planning_view 再复制一次。具备机体几何且三段成功时，查询阶段共有六次占据数组复制，加准备阶段一次。500×500 为 3,000,000 + 500,000 字节，100×100 为 120,000 + 20,000 字节。未知/失败分支提前退出时不可套用七次模型。

计时不包含验证断言、输出及外层返回结果析构；完整查询内部的规划器/地图析构包含在调用时间内。准备阶段只测 grid.clone，不包含 MissionRoutes::planar 的边界遍历或 Costmap 构建。三段阶段运行真实 SimplePlanner 查询，未调用完整探索调度器，也未模拟取消。

## 结果

时间单位为微秒。p50 使用排序后第 17 个样本；p95 使用第 31 个样本。32 个样本的尾部估计不稳定，几个完整查询的 p95 达到 3–5 毫秒；不据此承诺任何调度尾延迟收益。

| 尺寸 | 场景 | 阶段 | p50 µs | p95 µs | 每样本占据数组复制字节 | 源高度项 |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| 100×100 | 全自由 | route_preparation_grid_clone_only | 0.391 | 0.401 | 20000 | 0 |
| 100×100 | 全自由 | query_owned_costmap_clone_only | 0.401 | 0.411 | 20000 | 0 |
| 100×100 | 全自由 | body_planning_view_clone_and_clear | 0.951 | 1.182 | 20000 | 0 |
| 100×100 | 全自由 | three_route_queries_full_source_counted_copies | 179.241 | 3327.743 | 120000 | 0 |
| 100×100 | 稀疏障碍 | route_preparation_grid_clone_only | 0.401 | 0.481 | 20000 | 78 |
| 100×100 | 稀疏障碍 | query_owned_costmap_clone_only | 0.511 | 0.571 | 20000 | 78 |
| 100×100 | 稀疏障碍 | body_planning_view_clone_and_clear | 1.232 | 1.903 | 20000 | 78 |
| 100×100 | 稀疏障碍 | three_route_queries_full_source_counted_copies | 170.638 | 191.659 | 120000 | 78 |
| 500×500 | 全自由 | route_preparation_grid_clone_only | 10.396 | 143.036 | 500000 | 0 |
| 500×500 | 全自由 | query_owned_costmap_clone_only | 10.376 | 11.747 | 500000 | 0 |
| 500×500 | 全自由 | body_planning_view_clone_and_clear | 11.187 | 12.278 | 500000 | 0 |
| 500×500 | 全自由 | three_route_queries_full_source_counted_copies | 251.569 | 4353.392 | 3000000 | 0 |
| 500×500 | 稀疏障碍 | route_preparation_grid_clone_only | 10.255 | 11.737 | 500000 | 2450 |
| 500×500 | 稀疏障碍 | query_owned_costmap_clone_only | 12.889 | 29.224 | 500000 | 2450 |
| 500×500 | 稀疏障碍 | body_planning_view_clone_and_clear | 17.666 | 19.580 | 500000 | 2450 |
| 500×500 | 稀疏障碍 | three_route_queries_full_source_counted_copies | 1031.807 | 5147.771 | 3000000 | 2450 |

## 行为检查与后续边界

- 成功三段逐次比较精确路线点和末端 yaw；第三段从第二段实际末端点、末端 yaw 出发，而不是直接采用候选请求朝向
- 返航选项与生产 home_goal_options 一致：禁止倒车、位置容差 0.1 米、朝向容差 0.2 弧度；平面候选使用默认选项
- 不可达目标检查精确重复错误；homeward 失败只执行一段，outward 失败只执行两段，并检查重复结果一致
- 原始占据图保持不变，复制缓冲独立；这些是同实现重复性与隔离检查，不是优化实现对参考实现的 A/B 等价证明

单次请求的前两段共享同一准确起点，理论上可复用一次机体清除视图；第三段必须使用外出路径的实际末端姿态。现有消费型 SimplePlanner 查询仍需要拥有地图，简单克隆已准备视图并不会消除复制。没有为此增加持久缓存、锁、哈希、生产分支或占位状态。
