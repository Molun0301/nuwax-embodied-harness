# 本机导航规划质量与性能专项 · 2026-10-08

本阶段未产生可发布的导航算法修复。已确认 OFF 控制评分与全局参考朝向的语义冲突；最小候选解决合成绕障停滞，但加重实录失败，已保留证据并撤回生产修改。没有降低安全断言、改变净空阈值或触碰机器人。

## 基线与执行条件

- 独立检出：`/home/ml/Documents/Codex/2026-10-07/task/neh-done-work`，分支 `fix/v3-legacy-cloud-seal-20261006`。本阶段源码基线为 `a0d31df23f93992cf75b458cba756e6d6543eb6d`。
- 旧二进制来自 `a1a656b1` 加模拟器锁文件同步；其后至 a0d31df 的提交不含导航改动。候选来自 a0d31df 加本次评分修改。没有采用旧 537ed937 的测试作为当前基线。
- Rust 1.88.0、现有 LCM/PCL/GTSAM、本检出 release 增量缓存；locked/offline 构建通过。模拟器 Cargo.lock 原来缺少当前 nuwax-map-chunks 的依赖记录，唯一同步为增加 getrandom 0.3.4/libc、删除 sha2，没有改包版本或下载 SDK。
- 使用真实 PCT body_route、RouteFollower、机体检查及独立速度积分的纯模拟器。五个冻结场景各 ON/OFF，共十请求；速度 0.5m/s、原目标容差 0.35m，原始请求和完整失败均保留。

## 首个候选与拒绝理由

`nuwax-nav-pct/src/body_route.rs` 将全局节点 yaw 设为前后邻点的平均方向。OFF 直接执行稀疏全局参考；`surface_body_control.rs::tracking_error` 却在变化的 yaw 间插值，当作可执行曲线切向。实际绕障首段 [0,0]→[2.375,1.775] 的方向约 0.642rad，中点旧评分偏好约 0.152rad，导致评分和追踪目标互相牵制。

候选仅在 OFF 平移评分引用中采用该 XY 段切向，ON 精细姿态和同位置转向保持不变；不修改实际路径、不替换直线规划，不变更碰撞/制动/观测/目标成功条件。冻结评分回归在旧行为下失败，修改后 runtime 库测试 194 通过、45 忽略、0 失败；忽略项未记为通过。

完整模拟暴露候选实录 OFF 更早受净空检查阻断，因此单元测试与合成改善不足以准入。候选 diff、二进制和失败保留在任务目录；生产源文件恢复到 HEAD。

| 场景 / 模式 | 旧版结果 / 模拟秒 | 候选结果 / 模拟秒 | 旧/候选全局规划次数 |
|---|---:|---:|---:|
| straight-seven / OFF | 到达 / 14.30 | 到达 / 14.25 | 1 / 1 |
| straight-seven / ON | 到达 / 14.00 | 到达 / 14.00 | 1 / 1 |
| obstacle-seven / OFF | 超时 / 120.25 | 到达 / 20.45 | 2 / 1 |
| obstacle-seven / ON | 到达 / 18.35 | 到达 / 18.35 | 1 / 1 |
| recorded-0-5565 / OFF | 超时 / 119.50 | 超时 / 119.55 | 122 / 2247 |
| recorded-0-5565 / ON | 到达 / 30.75 | 到达 / 30.75 | 2 / 2 |
| disconnected-up / OFF | 预期不可达 / 0.00 | 预期不可达 / 0.00 | 1 / 1 |
| disconnected-up / ON | 预期不可达 / 0.00 | 预期不可达 / 0.00 | 1 / 1 |
| upper-straight / OFF | 到达 / 4.05 | 到达 / 4.05 | 1 / 1 |
| upper-straight / ON | 到达 / 4.00 | 到达 / 4.00 | 1 / 1 |

绕障 OFF：旧版 120.58 秒墙钟超时，候选 20.77 秒墙钟／20.45 秒模拟时间到达，8.070 米路程、2.245rad 累计转向、1 次规划。两版 ON 均 18.35 秒模拟时间到达。

实录 OFF：旧版 119.50 秒模拟时间／120.36 秒墙钟内 122 次全局规划，走 4.957 米、剩 2.819 米；候选 119.55 秒模拟时间／120.34 秒墙钟内 2247 次，走 0.914 米、剩 6.419 米。候选固定在 (-0.69455,-0.31863,-0.04431)，重复 reference 相同且扫掠返回 required clearance。旧版也有重复路线与净空失败，不能据此证明通道在当前实际站姿可通行。全十请求的独立运动检查失败计数均为零，但这不等价于实机安全认证。

这些重规划次数来自纯模拟器 tick：ReplanRequired 后制动停稳便重新全局规划。生产调度器还会撤销规划器、进入 AwaitingPath 并保留原等待期限，不能将模拟次数直接断言为实机调度频率。

## 有限 A/A 与 A/B

预先固定七米绕障 OFF、最多 30 秒模拟时间；A/B 各一轮预热后两轮测量，无浏览器渲染或并行构建。直接 native step 不加墙钟节拍，单独观察算法工作量。样本很少、任务结果不同，不做显著性或 CPU 改善结论。

| 版本/轮 | 模拟结果 | native CPU 秒 | 控制总 ms / p95 ms | 全局总 ms |
|---|---|---:|---:|---:|
| A1 | running 30.00s | 0.810 | 741.67 / 3.366 | 7.817 |
| A2 | running 30.00s | 0.780 | 733.81 / 3.350 | 7.590 |
| B1 | succeeded 20.45s | 0.760 | 737.14 / 4.379 | 4.798 |
| B2 | succeeded 20.45s | 0.760 | 722.85 / 4.217 | 5.892 |

A 两轮轨迹一致，30 秒模拟时间后剩 4.286 米；B 两轮轨迹一致，20.45 秒模拟时间到达。B 单拍 p95 并未下降，不能称为普遍性能优化。控制 ms 是 follower.step 算法时间；进程 CPU 和客户端 native 交换墙钟另列。没有测真实控制发布、守卫或命令消费等待，也未证明 Go2 CPU 问题已解决。

## 后续分析边界

优先使用保留的实录停点、前后站姿和 reference，定位全局位置参考到实际全机体接续为何不一致；需要在相同净空策略下给出可通过运动见证后再修改。重复路线的重试需分别研究纯模拟与生产调度，不应凭模拟循环直接增加生产缓存或改重试预算。宽泛的高度/地面/支撑/净空/漂移阈值专项仍暂缓。

本次仅发布模拟器 Cargo.lock 的依赖记录同步及本诊断文档，不包含已撤回的导航评分候选。

证据根目录：`/home/ml/Documents/Codex/2026-10-08/task/neh-navigation-quality`。关键文件：

- `baseline/summary.json`、`baseline/runs/*`：旧版十请求完整结果。
- `candidate1/summary.json`、`candidate1/runs/*`、`candidate1/recorded-stop-fixture.json`：候选十请求及最小停点片段。
- `candidate1/scoring-candidate.diff`、`candidate1/nuwax-navigation-simulator`：撤回候选的可复现实现。
- `baseline/scoring-regression-red.log`、`baseline/runtime-candidate1.log`：先失败后通过证据。
- `finite-comparison/summary.json`、`finite-comparison/{A,B}/*.jsonl`、`docs/开发文档/有限对照方案.md`：有限对照及事前方案。
- `baseline/lock-sync.diff`、`baseline/build-locked-synced.log`、`baseline/build-candidate1.log`：离线构建及锁文件证据。
