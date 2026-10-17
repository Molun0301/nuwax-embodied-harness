# 5 cm 导航来源与测试世界资格记录

2026-10-17。此批交付显式诊断和可重跑测试，不改变默认导航策略，不代表完整闭环已通过。运行使用云端 x86_64 release、同一增量 target；真实 codec + 同一 liblcm context 的 memq，不覆盖 UDP、硬件执行器或 Go2 实机。

## 已运行与未通过部分

- 默认七包类型检查、旧 `surface-recovery-qualification` 构建及新 `local-guidance-qualification` 构建均通过。默认不启用新采样模块或 Scan/profile
- 14 项新旧独立连续 oracle 控制、显式 mask 完整 footprint preflight、fixed-recent wire preflight通过，共16项
- 显式 mask Wall live：几何连续审计591段通过，但 StaleInput，未完成原5秒 active 零速窗；exit101
- 无mask平地延伸 Wall live：StaleInput 后终止，真实执行段133与实体墙相交，SAT gap约−0.00686958m；exit101。不能再归为观测域 oracle 问题
- fixed-recent Box live：已发布并接纳初始 Path，后续局部无可执行运动等待30秒后 PlannerFailed；cleanup、原连续几何审计通过，但测量健康资格不足；exit101
- fixed-recent Wall live 未运行，不能算通过
- 新诊断中 planning fork 与原 source 的 owner 相等断言不成立，已修正并新增两个真实fork→正常Scan→capture/replay接缝回归，均通过；合成 admission 计数仅验证存储/复放，不作为真实 global h 资格

原失败日志和筛选条件保留，没有调整速度、预算、60秒总窗口、30秒局部等待、源期限或碰撞容差来改判通过。

## 旧 Wall 测试的世界前提

原 PUBLIC_NAV 未下发 boundary 或 virtual mask。真实占据只有有限墙，地面观测矩形也有限。旧测试却要求完整 footprint 永留观测矩形并且 Wall 必须持续保持 active。

两次旧运行实际绕过有限墙下端并到达原目标。补充离线分析使用全部2711/2750条实际段，连续墙体分离证书下界约6.864/6.817cm；原支持域失败仍保留，后来 footprint 越观测域约40cm，不能用浮点误差解释。若真实世界域外是悬空，禁止信息不能凭空由“没有观测”推出。

新增明确命名的两种世界，不覆盖旧入口：

1. 无mask、真实平地继续延伸：允许未知域绕有限墙，独立连续实体碰撞与严格原目标/零速 fence 仍检查
2. 显式100×60、5cm许可 mask：通过真实 NavigationMap3D.virtual_obstacles 编码、cache、hub、scheduler传递；域外禁止是明确设置。完整footprint域审计与原Wall负例全部保留

新运行暴露的时序/碰撞失败与上述旧测试前提是不同问题，不能混为一项。

## fixed recent 测量不等于 raw hints 为空

新 fixed-recent 用例保留原6000 ground returns、真实递增时间/sequence、原期限和通道；只有 persistent 新增障碍。原严格 raw-empty preflight 仍保留：Box插入后导出192条提示时，它真实exit101。

新 preflight 78次离线hint查询通过测量/来源资格，明确 raw_hints_empty=false；Box为192/288、Wall为960/480。空与非空独立报告，不能把它们当作实际规划分支已经受影响。实际 `recent_guide_interference` 仍未知；没有用跨运行高度推导冒充该次真实 PlanKernel 输入。

## 5 cm fixed Box 的进展问题

初始全局Path约180ms接纳。日志随后显示两条到x≈1.85的短局部轨迹，完整绕行约350.9ms晚于其源位姿被显示，机器人已前移约69.85mm。最终机身中心x≈1.81143，Box左面约2.0；0.30m方体的旋转半径约0.21213。在该fixture的前向、无侧移/倒退、固定Z/yaw模型下，可解析证明此站姿已不能绕出Box。完整轨迹晚到前约177ms即越过这一充分条件边界。

这不意味着所有小运动不可能，也不证明改变某一排序必能避免。日志没有直接记录两个短结果的 partial/branch 标记，下一步补独立 FirstPartialBranch 事实窗口；原controller/碰撞规则不改。两分支BudgetExhausted在等待开始后约601ms已完成，30秒不能解释成一个搜索一直运行30秒。

## 时间与守卫的解释边界

新Wall运行的源生成、静态map接纳和控制更新有明显延迟。输入桥存在串行scheduler/publish ACK等待；recent构造也在该actor同步执行，但纯consumer独立基线100k点中位约10.6–33.4ms，不能据此直接归因500ms停顿。

实际框架的 `CommandDispatchBound` 是最晚准入，不能当作持续速度租约。模拟Plant没有完整Python guard/watchdog/物理制动模型；模拟真实相交保留，但不能直接宣布硬件guard失效。续租时内部真实输入状态尚须补trace，已发布STATE可能滞后，不能据它推断当时内部时效。

## 重跑入口

在 `services/nuwax-workspace` 使用已配置工具链与原增量target，例如：

```sh
cargo test --locked --release -p nuwax-nav-stack --lib --features surface-recovery-qualification --no-run
```

显式运行的前缀为 `lcm_dataflow::surface_dataflow::tests::dynamic_occupancy_qualification::`。使用 `--exact --ignored --nocapture --test-threads=1`，每例串行：

- `wall_world::explicit_mask_wire_cache_full_footprint_preflight`
- `wall_world::explicit_mask_wall_preserves_active_goal`
- `wall_world::finite_wall_unknown_flat_continuation_arrives`
- `static_source_only::fixed_recent_geometry_wire_preflight`
- `static_source_only::fixed_recent_box_local_planning_on`
- `static_source_only::fixed_recent_wall_local_planning_on`（本轮未运行）

`local-guidance-qualification` 是单独显式观察特性，会启用 Scan/profile。旧Box h wrapper仅得到 global_curved=0、NOT_QUALIFIED；旧Wall wrapper在原闭环断言失败，未获得h对照。所有这些 ignored 资格必须显式运行，普通CI通过不代表它们通过。

## 首个真实 partial 分支事实（2026-10-17 UTC 21:50）

独立 `FirstPartialBranch` 窗口只保留第一个真实 `Ok(partial=true)` 分支；原h资格不变，不离线重放，不改变选择策略。显式 runtime/stack check 与 no-run 通过，两个 fork 接缝回归通过；默认无Scan/profile的七包tests类型检查另通过；一次原参数 fixed Box live 运行 38.16 秒、exit101，原 PlannerFailed/30秒局部路径等待失败保留。

本次来源与时序测量资格为 `QUALIFIED_OBSERVED_FAILURE`：cleanup、原连续运动/几何oracle和fixed-recent测量审计均通过，不能用上一次晚期pose过期解释本次失败。首partial绑定唯一 Inserted 原source（revision5），事实窗口为 `QUALIFIED_CAPTURED_FACT`：

- request_detour=false、branch_detour=false、request_straight=true、follow_reference=false、prefer_straight=true
- 真实 start=(1.1372021953,1.5,0.5)，目标=(3.5,1.5,0.5)、容差0.1；原reference与实际Scan reference一致
- 输出 partial=true、smoothed=false，末点≈(1.8492436168,1.4521450198,0.5)；该分支365.062242ms，预算256未变
- 原全局路线真实来自owner；此为按“首partial结果”条件取样，不能冒充随机性能样本或旧curved-global h资格
- 后续91行display的XYZ与捕获结果一致，但display没有request identity；controller admission/execution的严格同请求归因仍为unknown

当前生产代码遇到该nominal partial即返回，确实不会在同次compute继续现有detour分支。因此“nominal partial暂存、尝试完整detour”的候选触发条件已由实际记录确认；它是否得到完整路线、额外等待是否值得、旧路径是否继续推进，仍未验证。不得把分支事实当作候选收益或闭环修复。

新精确入口使用 `local-guidance-qualification`：
`lcm_dataflow::surface_dataflow::tests::dynamic_occupancy_qualification::static_source_only::fixed_recent_box_first_partial_branch`。
原始CAPTURE、ATTRIBUTION、STATUS及失败终态分别保留在工作目录阶段日志，不将重型日志写入Git。

## 实际 actor 时间观察（单次，未复现旧失败）

显式 recovery 诊断会话记录定容标量：fixture 点云生成/审计，输入桥decode/build与scheduler ACK，状态输出ACK，以及scheduler实际renew内部快照、控制和fence。默认无会话不记录；不增加工作线程、await、source gate或新鲜度计算。try_lock记录不等待；容量或竞争丢失明确报告。期限、数据、原时序与碰撞oracle不变。

旧 `surface-recovery-qualification` 的 runtime/stack 增量check/no-run及6项记录器控制通过；实际features无local-guidance/Scanprofile。唯一 `wall_world::trace_finite_wall_flat_input_timing` 运行26.86秒，原result/cleanup/连续几何oracle全Ok，Succeeded并完成原零速fence。此前无插桩失败仍保留，本次没有复现该失败，不能作为修复或加速证据。

记录42701条，竞争丢91条，overflow/poison/open-spans均0，报告明确incomplete。22个end缺start、14个start缺end；只配对完整区间才能报告elapsed，缺失不能补猜。65536×224=14680064字节只是固定记录区逻辑容量，不是RSS。时间包括抢占/等待/插桩，不能当线程CPU。

普通默认七包tests check另通过。原始 ACTOR_TIMING、完整来源和终态保留在工作目录；后续使用这些已有数据归因，不为得到理想结果重复取样。此资格不包含UDP网络、Python真实guard或机器人。
