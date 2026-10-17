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
