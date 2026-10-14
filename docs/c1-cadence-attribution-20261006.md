# 移动用例节拍归因：保留失败，不改放行条件

2026-10-06 UTC。本阶段只增加测试诊断和独立的校准坡道用例，没有新的生产算法改动。
冷接触域优化保持原已验证版本；之前的三个节拍失败没有被覆盖或改成通过。

## 最小诊断改动

`route_moving_worker_tests.rs` 与174行测试辅助模块 `route_moving_cadence_trace.rs`：

- 有界预分配的标量tick记录保留原始唤醒时刻、deadline/reset、原始gap和各消费阶段。
  大于75ms的gap严格配对前一个tick的body、相邻间隔、当前sleep调用前后窗口
- 现有Probe锁等待、profile begin/end及嵌套map调用分别报告；所有诊断成本保留在原始
  耗时中，不减去成本获得“通过”。格式化和独立安全oracle仍在定时循环及worker回收之后
- `/proc/thread-self/schedstat` 在实际线程内打开，复用固定读缓冲；读取成本、时间窗口、
  线程ID、缺失/全零/回退/时钟无效均明确报告。它是内核已记账的runtime/runqueue数据，
  不是精确线程CPU时钟，短时间零增量不代表没有工作，记账延迟也不由读窗口给出上界
- 消费tick只连接本tick的真实非零receipt及其准确result ID，报告计算结束、发布、领取、
  原有oracle时间戳之间的检查后差值。不通过最近step编号猜测关联
- 原始75ms节拍、50ms控制/map限制、观察TTL、真实帧、运动oracle和命令准入全部未改。
  50ms超限计数在可能较早失败的节拍断言前打印。新记录溢出可见

没有新生产锁、依赖、不安全代码或新的运动许可。传感器线程本次没有新增scheduler计数，
仅保留已有SOURCE_ASSEMBLY wall阶段；这项归因限制仍然存在。

## 实际编译和执行

固定混合test profile：MLS planner/nav-runtime opt-level=3，其余crate普通test，debug=0。
开启body-query-profile的增量构建通过（Cargo 1m10s），关闭该feature的编译也通过
（1m49s）。两次构建前后四项源文件摘要一致。后者只编译，没有重复执行用例。

带feature的测试二进制SHA256：
`c4aa36e4969c2da1f496a3b2e5a1bb057340a6d47bc065e41ea5f06f562e819f`。

实际执行7项：三个新纯诊断单测、一个新校准坡道用例、三个既有独立移动用例。
**6通过、1失败**。其中三个诊断单测和坡道通过；OFF失败，ON和held本次通过。
这不是release、实机或稳定实时性证明。

| 本次移动用例 | 功能结果 | control中位/p95/max ms | gap中位/p95/max ms | >75ms |
|---|---|---|---|---:|
| OFF | 到达；位置误差0.063230m，23.492s | 1.462 / 5.003 / 36.352 | 50.001 / 52.022 / 214.091 | 9 |
| ON | 到达；位置误差0.149006m，20.450s | 1.432 / 3.096 / 8.208 | 49.999 / 50.322 / 62.579 | 0 |
| held | 连续运动1.075429m、6个held非零命令、后续新计划准入；不是7m全目标到达 | 1.446 / 3.506 / 21.877 | 50.002 / 50.138 / 52.721 | 0 |

本次三组control/map的>50ms报告计数均为零。OFF依然先在原75ms断言失败，后面的
control/map断言没有执行；不能将未执行断言写成通过。ON和held完成了原断言。
全部独立区间及receipt制动oracle都在节拍断言之前执行并通过这些特定场景。

完整进程资源（含构造、诊断、oracle，非内层planner资源）：

| 用例 | wall秒 | user+system CPU秒 | peak RSS KiB |
|---|---:|---:|---:|
| OFF | 24.547 | 2.819227 | 71,644 |
| ON | 21.299 | 1.566582 | 66,612 |
| held | 15.175 | 1.483210 | 76,676 |

## OFF的9次失败定位

独立结果审查确认9个gap全部精确重构，打印的18个前/当前tick阶段也全部与body窗口
相符，没有未分配的墙钟gap余项。9次超限均位于sleep调用前到原始唤醒时刻的窗口：

- 请求睡眠36.40–49.84ms，额外等待35.61–164.32ms
- 前一tick body仅0.011–0.470ms；相邻bookkeeping最高2.440ms
- 本轮最高control 36.352ms、map更新0.114ms、消费worker.step 1.967ms
- 8个超限tick有准确非零receipt；其计算结束→发布约0.001–0.005ms，而发布→领取
  83.834–207.497ms。另一个没有非零receipt，保持缺失而不借用邻近结果

这支持“这些失败主要发生在消费者等待/唤醒区间”，**不支持全部归因为纯CPU调度等待**：
tick214的164.317ms额外等待中，runqueue记账增加110.478ms；但多数其他tick只增加
0.003–2.216ms。tick271在155.631ms额外等待中反而记录139.170ms runtime和14.092ms
runqueue增量。内核记账滞后及sleep窗口内部原因不能从这些样本完全确定。

现有诊断锁不是本轮大gap的证据：OFF compute Probe等待最大0.018915ms，map Probe
最大0.000552ms。新增consumer/compute读取中位约0.00126/0.00814ms，最大
2.756/2.490ms；这些成本已经包含在观察结果里，未被扣除。

准确receipt关联只表示结果身份准确，不表示guard计算与领取时状态相同。OFF的221/338、
ON的204/305、held的142/213仍报告状态不同；本次均没有缺失proof或无效clock pair。
receipt链的末端是原有oracle时间戳，不是物理执行器完成时刻；例如tick261的
receipt/applied-feedback阶段2.320ms不能由0.005ms的领取→oracle差值替代。

## 使用结论与证据

本次ON/held通过与之前失败之间没有新的生产算法变化；不能归功于诊断代码、宣称稳定
20Hz或删去旧失败。没有证据支持改动安全阈值、忙等替换sleep或增加全局锁来获得测试通过。
继续保留冷接触域小型优化，下一步功能覆盖优先于没有依据的调度改写。

原始目录：`/workspace/shared/neh-cadence-ramp-20261006/`。
`build.sh`、`normal-feature-build/build.sh`保存准确编译命令；`run-focused.py`和
`results.json`保存精确selectors/退出码/资源值；各`.log`保留全部原始输出。
诊断三个selector位于 `route_moving_worker_tests::cadence_trace::`，移动用例分别为
`moving_measured_5cm_primed_off_full_goal`、`moving_measured_5cm_primed_on_full_goal`、
`moving_5cm_primed_held_planning_continuity_only`，均以`--exact --ignored --nocapture
--test-threads=1`执行。校准坡道资格见通行性补充记录。

冻结源SHA256：

- `route_moving_worker_tests.rs`: `924423230141fab36d4e2d4713d8b41bb62e1c0e9f078a13815440f8f6d92f4a`
- `route_moving_cadence_trace.rs`: `687f02f528e11dd8542591388e8d644d78d97fa9e8d591ea39146d3a416c8048`
- `surface_follower.rs`: `894a21d5e12723aa9413d454c14dcee6524deb6f2e7af2c1875484a600aa5862`
- `body.rs`: `5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305`

没有Git提交或推送。真实assigned-map→Motion Guard传输组合的AF_UNIX阻断仍然存在，
本组测试不能替代该组合或硬件资格。
