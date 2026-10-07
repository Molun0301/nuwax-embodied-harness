# 地图变化后的恢复拥有方：源码核对与未验证边界

2026-10-06 UTC。本项为当前源码的只读核对，没有新增运行结果或生产改动。
结论是生产代码存在跨拥有方的自动重新规划路径，不能仅从局部planner被stop推断目标丢失；
但完整“新增障碍→停车等待→持续保留障碍的绕行→原目标到达”本次尚未取得组合运行证据。

## 实际交接路径

1. `nuwax-nav-runtime/src/lib.rs` 的 ReplanRequired 分支先通过原运动接口执行零速，
   再停止局部planner，转为AwaitingPath并发布当前状态。该路径不是直接把旧路线继续执行
2. `nuwax-nav-stack/src/surface_dataflow.rs` 的状态消费者只接受匹配当前目标的状态。
   waiting/controlling且sink_ready时保留原目标；等待状态的path_revision必须匹配当前
   版本（或尚无路径）才设置needs_plan，迟到旧版等待不能撤销刚发布的新路径
3. 普通同域几何更新保留目标与已交付路径。显式clear、域替换、取消、终态和构建失败有
   不同退休规则，不能把这些分支混为“任意地图刷新都会保留目标”
4. 后台规划使用最新表面、实际当前pose及保留的原目标。结果通过原epoch/取消/目标/
   地图权限检查后，递增path_revision并发布路径，交还导航调度器准入

相关源码位置（此记录版本）：runtime `lib.rs:1649–1655`；stack
`surface_dataflow.rs:727–775, 817–841, 866–931`。这是源码接线事实，不是完整并发恢复
或物理制动已经运行通过的证明，也没有据此修改现有生命周期。

## 现有用例分别证明什么

- `surface_dataflow::tests::distant_route_changes_wait_for_local_replan_from_actual_pose`
  记录真实body_path的起点，并约束远端变化不能触发每帧全路线重复搜索；它不等于完整
  持续障碍绕行后的原目标到达
- `actual_surface_backend_stops_for_changed_map_and_resumes_only_new_path` 使用真实
  SurfaceFollower/PlannerWorker与FakeMotion检查调度停走顺序，随后由测试提交新路径；
  它不证明全球路径拥有方自动给出绕行，也不证明真实Motion Guard或执行器归零
- `surface_oriented::tests::inserted_obstacle_replans_around_it_and_continues_to_original_position`
  插入障碍时立即由测试重规划并替换路径，所以不能证明旧路径先拒绝。其零速等待是
  固定位姿的运动学模型，不应描述为积分校准减速或实际ZeroAck

没有为了增加通过数而重跑这些已有用例，也没有增加测试自有的重规划流程来冒充生产恢复。

## 完整拥有方用例为何仍受阻

已有surface_dataflow Harness绑定UDP并创建真实LCM。即使选择memq provider，
`nuwax-lcm/src/lib.rs:780–783` 也先创建CommandWakeup；其 `new` 在约68行调用
`UnixStream::pair()`。因此memq不是绕开当前AF_UNIX限制的无socket替代路线。

当前环境已经在普通执行及一次获准提升权限执行中拒绝AF_UNIX创建，详见
`c1-assigned-navigation-composition-20261006.md`。本项未再次尝试、替换传输或操作用户电脑。

后续在获准支持所需socket的环境，可扩展已有Harness/SurfacePathComputer：保留真实
body_path计算，先确认非零运动，再插入有限障碍，短暂阻塞第二次真实规划，验证同一目标/
会话在AwaitingPath下保持零目标且计算起点是实际停止位姿；释放后验证更高路径版本、
真实绕行、恢复运动和原目标到达。必须保持障碍，且证明从确切停止位姿仍存在完整机体
绕行空间；不能回退/移动该位姿、清除障碍或放宽前进约束制造可达性。

该Harness仍使用RecordingMotion和瞬时运动学反馈，只能补充拥有方/调度恢复资格。
真实执行器制动另需真实归零证据；更强的模拟制动也必须显式积分声明的减速并验证每段
制动扫掠，而不是等待一段时间后假定物理位置已经停止。
