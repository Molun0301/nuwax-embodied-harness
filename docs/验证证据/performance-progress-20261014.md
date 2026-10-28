# NEH 云端性能优化阶段总结（2026-10-14）

更新时间：2026-10-14 10:51 北京时间（2026-10-14 02:51 UTC）。本轮约定工作窗口至北京时间11:00；本文汇总已完成改动与资格，最终推送回执以Git记录为准，不是整机验收声明。

## 当前结论与交付状态

已在导航、地图接纳与保存、任务查询、日志、感知、记忆和网页显示中落实多项有证据的局部优化，同时补齐所有权、兼容性和持续运动回归。收益各有测量范围，不能相加成整机加速比例；没有证据证明机器人 98% CPU 问题已解决，也没有声称所有模块都已优化。

- 仓库：`Molun0301/nuwax-embodied-harness`
- 工作分支：`fix/v3-legacy-cloud-seal-20261006`
- 本轮起始远端基线：`65d15c5262af71ee0ab532133bd0afdcb5e75911`，已确认是当前提交的祖先
- **交付核对截点：86个新增提交已普通Git CLI推送并独立核对**。2026-10-14 **02:49:47 UTC** 的`ls-remote`为 `a09890e2e16dc10933870c9a56ed19f403654880`，与当时本地一致；工作树干净。此后本总结的文档整理另计，以Git记录为准
- 同一 Library 检查点 **v43，5,512,676 bytes** 已顺序应用并核对最终tree，覆盖上述86提交；后续文档更新的检查点回执另计。高密度recent的完整纯文本原始日志也已单独保存
- 新设备登录已认证账号Molun0301。全局Git配置因只读写入失败，随后使用单次命令credential helper完成推送；此前认证阻塞已解除，没有网络封锁证据。推送成功不等于部署、CI通过或实机验收

**最新导航结果：** 完整 `NavScheduler` 局部恢复修复已提交为 `0eed9d2`，固定持续障碍的原目标在 **13.32秒**完成，仅2次规划/2次准入，232条完整停车尾迹和715条实际运动边全部通过。早期60秒失败与两次短程停车尾迹失败仍保留；独立调查发现采样位姿到实际应用间的转动差异，当前只是合同诊断，没有生产修复或真实50 ms执行器hold证明。不能据此称现场机器人卡住或安全时序问题全部解决。

**最新集成与归因：** Python3.8.10最新59目标去重选集 **1227项通过**，涵盖地图状态10项与完整installer57项；唯一排除仍为明确执行legacy内容哈希的pred1 fixture。后续改动仅涉及Rust/JS及诊断，未改这份已验证的Python生产状态；不能把它说成最终HEAD所有语言/模块全通过。Runtime适用范围236项通过但仍排除5项真实UDS。新RSS诊断支持allocator/free-page retention是复现晚窗增长的主要原因，剩余约43/114 KiB未归因；没有修改生产allocator或宣称无泄漏。

下面的“旧 → 新”均采用各专项报告的匹配基线，不意味着统一对比上述远端版本。局部连续优化的收益也不相加。

## 一、导航、碰撞与探索

### 已落实的性能改动

- 最新实际位姿的完整停车轨迹复核结束后，保留刚构造的当前支撑视图，避免随后的普通刷新重做相同工作。每次复核仍强制刷新并完整验证；旧选择位姿分支仍还原临时视图。12,800 地面列、自体回波支撑派生夹具中，“复核＋随后视图刷新”中位数 **33.309234 → 24.670447 ms**；800 列为 **23.853009 → 17.584206 ms**。完整地面负对照基本不变。这不是完整下一拍选择或整个控制循环耗时。[证据](performance-incremental-updates.md#最新实际位姿复核保留已构造的当前视图2026-10-13)
- 前沿局部失效使用已有列索引枚举脏范围，稀疏大范围仍回退原槽位扫描。131,072 支撑单元夹具仅访问 361 列，局部几何刷新 **4700.015 → 1498.867 µs**；8,192 单元 **966.890 → 833.719 µs**。候选、收益、代价与冷重建比较一致；不含导航可达搜索和调度排队。[证据](performance-incremental-updates.md#前沿局部失效按既有列索引枚举2026-10-13)
- 近期障碍碰撞查询只构造需要的高度行，并在一次完整位姿查询内复用相邻细列所属的同一粗列。10 cm 粗网格 / 5 cm 来源、8 帧清空场景 **294.08 → 159.64 µs**，门洞 **297.48 → 156.82 µs**；1 帧障碍拒绝 **17.18 → 9.46 µs**。基线为 `bb1a14f` 完整查询，9 对交替测量，暖来源缓存、刻意绕过位姿结果缓存。等分辨率、无 recent、coarse-only 负对照没有实质收益；没有分配器/RSS或整机 CPU 测量。[证据与完整结果](body-recent-query-reuse-20261013.md)

- Scan非arrival路线窗口复用第一次完全相同的polyline投影拒绝结果（`8949d9d`），保留原passage尝试、取消检查、失败优先级和witness。真实recent-guide fine-escape非arrival夹具的 `[body-view/endpoint/sweep]` 调用 **124/124/67 → 122/122/67**，结果仍68 poses、1 expansion；boundary-drained **81/81/19 → 80/80/18**。arrival对照完全不变（仍119 poses、3 expansions）。这是少做少量查询，不是延迟或新路径能力提升。[证据](../diagnostics/scan-polyline-reuse-and-guide-eligibility-20261013.md)

### 新增近期障碍列构造优化（`e5854d7`）

只更改occupancy-column构造：有效整数voxel keys集中排序/去重后按XY分组，批量建立原BTreeMap；ground点保持原输入顺序和精确值，source identity/time、retention、错误/消费顺序和obstacle-overrides-ground不变，没有加缓存、锁、队列或分布阈值。

三对独立进程固定AB/BA/AB，12 cohorts、各8次warmup/7次sample，完整真实 `RecentSourceState::consume → NavScheduler ACK`：

- 100k分散点，1帧 **34.752052→19.448489 ms**；8帧 **35.248652→19.988084 ms**（三个进程中位数的中位数）
- 重复列负对照保留：10k/8帧 **533.776→592.184 µs**，约增加58 µs；100k/1帧 **6.344087→6.710390 ms**、8帧 **6.048280→6.366792 ms**。100k各配对方向混合，不能仅凭总体中位数或单个配对挑选结论，也不把退化断言为噪声
- 100k分散点的最终height Vec容量：1帧 **1,600,000→400,000 bytes**，8帧 **12,800,000→3,200,000 bytes**。这是capacity×元素大小，**不是RSS/allocator调用数**；每XY仍有一个height Vec
- 初始等价candidate记录的瞬时已知重叠payload **6,167,168 bytes**（flat keys＋group entries＋height Vecs），**不含**原points、retained frames、ground/obstacle sets、BTreeMap节点、sort内部和allocator开销；flat keys在bulk tree前drop，最终容量变小不意味着所有峰值变小

测量含真实codec、状态/watermark更新，但使用无active goal的SurfaceFollower＋RecordingMotion，没有PlannerWorker、LCM/transport或真实plant。独立stage测量不能从full consume相减推算析构/分配成本。900次初始test-only等价比较与最终生产/original oracle资格分别保留；第一次copied oracle的Self返回类型编译失败只改测试类型，没有放宽生产语义。详细结果及冻结binary边界见[报告](../diagnostics/recent-column-build-qualification-20261014.md)和[原始结构化结果](../diagnostics/recent-column-build-qualification-20261014-results.json)。报告/JSON中的“root commit pending”是提交前记录；现已本地提交e5854d7，不代表远端发布。

### 高密度近期点云：新增一次完整导航负载验证

在原box障碍、局部规划ON的基础上新增独立测试，原六项live场景及通过标准保持不变。通过真实recent-cloud编码、同上下文liblcm memq、consumer、调度器ACK、PlannerWorker与controller，输入 **96,000个清空场景返回点 / 97,824个插入障碍后的返回点**。这些点落在原6,000个XY列中，不是100k独立列，也不是硬件采样保真声明。

第一次也是唯一一次live运行通过：原封闭目标 **16.768秒到达**，位置误差0.07233 m、yaw误差0.10752 rad，随后匹配的zero fence通过。原连续SE(2) oracle检查 **1,162个轨迹区间/节点**全部通过，原命令期限审计通过，没有无期限非零命令。

全程发布79帧；其中77帧位于首末实际非零命令之间的16.350秒窗口，该窗口可能包含停顿。实测source间隔200.013–225.661 ms、均值213.142 ms，**有效4.692 Hz，调度目标仍为5 Hz**。不能写成稳定精确5 Hz，也不能把publication数当作精确消费/ACK计数。没有基线配对、RSS或实机刹车结论；这只补足此前无active-goal ingestion测量与小点云live测试之间的负载覆盖。[完整证据](../diagnostics/dense-recent-qualification-20261014.md)

### 持续运行与搜索结论

- 局部规划 OFF/ON 分别运行 **305.845 / 305.671 秒 live 控制时间**，完成 **49 / 47 个目标**。合计 **96 个目标、11,438 个非零回执、42,724 条实际理想 plant 运动边**均经原碰撞及停车尾迹 oracle 检查；实际接受非零命令的积分时间分别为 **291.985 / 279.719 秒**，不能把全部 live 时间称为持续非零运动
- 真实 `RouteFollower`、`PlannerWorker` 和 5 Hz 合成来源跨目标保留，覆盖近期远端非地面点变化、测量抖动、目标/路径替换、取消重启。OFF/ON 最大复核 **20.682 / 33.732 ms**，最大消费者间隔 **53.454 / 58.266 ms**；后者沿用 ≤75 ms 的调度容许，不是严格 50 ms 实时保证
- 这是理想命令积分 plant；不是物理机器人、完整 `NavScheduler`/memq 边界、执行器延迟或真实刹车验证。RSS 首尾有增长，不能宣称零泄漏；首次地面分类夹具与修正后的非地面障碍夹具分别保留。见[持续运行报告](navigation-sustained-worker-20261013.md)
- 当前真实 recent-guide 的新机体敏感回归：7 个实际 recent hints 改变引导，必须先完成精确 **1 cm 直行**才能脱离初始约束；现有 fallback 已能在 **3 次展开、15 个节点、frontier peak 12** 内返回 **119 个全部复扫的位姿**。这是现有能力的正向回归，生产启发式、预算、原语和容差均未改变。[证据](../diagnostics/recent-guide-fine-escape-regression-20261013.md)
- 历史五个**强制static-guide**夹具中，原排序、放松下界及四倍下界均耗尽256次展开，只有测试oracle成功；这不代表实际运行链失败。新检查在相同地图/body/障碍/start/motion/arrival输入上调用生产 `RecentGuideOverlay → exported Scan`，**五项全部成功且均无需local search**，186/183/183/190/160个返回位姿独立复扫通过。没有oracle或启发式修改。只有hints不可用或overlay失败才可能回退static，而这五例并未触发该回退。[现行guide资格](../diagnostics/scan-polyline-reuse-and-guide-eligibility-20261013.md)、[保留的历史ablation](../diagnostics/local-search-priority-audit-20261013.md)

### 完整调度器恢复：已提交局部修复，另有独立时序边界

与前述96目标的持续worker测试不同，新夹具经过真实 `NavScheduler → PlannerWorker → RouteFollower → PlanningOwner`。原固定姿态因果试验证明：局部请求已启动，但上层处理 `ReplanRequired` 后确认ZERO并停止worker，使owner epoch **6→7**、route generation **5→6**，把刚启动的绕行取消。独立绕行见证已全身复扫通过，该固定姿态试验本身不是移动资格。

第一候选只保留请求、改走Pending，仍让旧admission产生直行命令，最终 **60秒未到达、215次规划提交**；该次终点断言先于最终安全审计，故尾审计未执行。后续同障碍短程归因证明原计划转弯在捕获起点合法，在继续移动后的实际位置碰撞；这只证明失去该转弯空间，不证明所有路线都不可能。两次短程实验另保留了完整停车尾迹失败，不能由后来通过覆盖。

已提交修复 `0eed9d2` 只在当前controller拒绝且局部可恢复时撤回旧执行admission，保留global route/goal、source tickets、planning owner和route generation；不调用会取消计算的广义revoke。未准入分支走原Pending，只有现有完整候选准入才恢复执行。OFF、严格错误、边界拒绝、取消/fence保持原语义，没有放宽期限、碰撞、budget或source-age阈值。

- 原固定持续障碍full-goal夹具 **13.32秒完成**，2次规划、2次实际准入；原路径revision1和目标不变
- 650 ms held恢复期间，非零回执和旧路径Command结果均为0（含Command(ZERO)）；owner epoch6、route generation5保留，3次真实新source acquisition继续含同一障碍
- 全程66次真实来源采集；**232条实际回执的完整停车尾迹、715条实际执行边**独立验证全部通过
- 首轮普通范围234通过/1失败的cleared-obstacle夹具实际需要转弯，却固定不积分yaw；修正为按真实elapsed积分转动和回馈位姿，保留原正向速度断言、5秒deadline及owner/submission约束，并增加实际边/停车尾迹检查，**2.27秒通过**
- 该恢复修复阶段的Runtime范围 **235 passed / 0 failed / 56 ignored / 5 UDS filtered，51.15秒**。full-goal运行于v1二进制，v2差别仅为上述夹具修正和显式ignored诊断；之后RSS诊断不在这轮覆盖内

这是云端理想plant的模块集成资格，不是实机或一般时延保证。默认 `path_wait_timeout=0` 禁用等待超时，Pending不能被称为天然有界恢复。[完整失败、归因与修复时间线](../diagnostics/navigation-replan-trigger-audit-20261013.md)、[结构化证据](../diagnostics/navigation-replan-trigger-audit-20261013.json)

### 完整调度器新增恢复稳健性

生产恢复不变，两个新的cfg(test) moving arms覆盖原持续障碍目标之外的真实调度边界，包含已提交Scan复用：

- 持有局部规划时真实新source清除障碍：原目标 **13.24秒完成**，2次提交/2次准入，230条回执完整停车尾迹和712条实际边全部通过；held期间无非零回执
- 持有旧detour时按实际支持的“cancel→确认fence→启动新session”转向不同目标 `[3.0,2.3,0.5]`：从goal1起总计 **10.85秒完成goal2**，3次提交（含被取消请求）/2次准入，185条停车尾迹和571条实际边通过。所有后续非零回执核对goal2及更新epoch/goal/path version；没有发明运行中直接replace API

两项共用原60秒总窗口、不为第二目标重置，650 ms真实held及5 Hz acquisition、40 Hz pose、原deadline/source bound和阈值均不变，安全审计在goal/timeout断言之前执行。既有persistent-obstacle arm未重复计数。最终普通Runtime **236 passed / 0 failed / 58 ignored / 5 UDS excluded，56.45秒**；两个moving arms为额外显式执行的ignored项，不混入普通计数。50 ms仍只是ideal-plant假设，未解决dispatch合同问题。[完整报告](../diagnostics/navigation-recovery-robustness-20261013.md)

### 持续worker RSS：有主因证据，非生产内存优化

保留原五阶段motion/oracle标准的诊断重跑，OFF/ON各49目标，live **303.844/304.146秒**，accepted nonzero **291.892/291.747秒**，分别5839/5836回执、21860/21780边全部通过。这是另一次诊断，不替代或累加原96目标试验，也不通过完整NavScheduler。

晚窗定义严格为第40–49次post-drain均值减第6–15次均值：RSS **+2742.0/+2666.8 KiB**，同期GNU allocator in-use+mmap仅 **+44,252.8/+117,067.2 bytes**，free arena **+3,198,550.4/+1,962,472.0 bytes**。所有持续诊断Vec容量在49次drain中固定、clear后len为0。fixture owners释放、worker/sensor join后，单次实验性 `malloc_trim(0)`使RSS **−24,776/−21,072 KiB**，allocator in-use不变。

这直接支持**allocator/free-page retention是本次复现增长的主要原因**，不证明精确Rust活载荷、生产无泄漏或早期每字节同因；约43/114 KiB晚窗残差尚未定位。采样非原子且sensor并发，不能把所有阶段差额独占归于oracle。没有生产trim、allocator或缓存策略修改。首次unsafe lint失败保留，GNU FFI例外仅限测试诊断。[完整RSS报告](navigation-rss-ownership-20261013.md)

### 采样到实际应用的位姿漂移：仅诊断，未改生产

保留的短程失败中，采样guard yaw≈0.2371653139，实际receipt yaw≈0.2585811667；按0.8 rad/s相当于约26.77 ms持续旋转，这是与记录一致的解释，不是物理SDK时序测量。精确几何重建中，真实生产callback与独立oracle均接受guard位姿、拒绝receipt位姿，并命中同一cell `[17,29,9]`。诊断“通过”表示成功复现反例，不表示延迟命令安全。

现有输入只有采样位姿与最新twist，没有相关的完整应用转移历史；guard APPLIED时间未完整传入规划。dispatch admission bound也不等于真实actuator activation期限。生产Go2调用尚无已证明的每命令50 ms expiry，fixture的50 ms cap只约束理想plant；不能改写reaction-time含义或用常量twist外推掩盖空缺。本轮在这个合同边界结束诊断，**没有生产改动、新增安全框架或性能收益宣称**。[独立合同诊断](../diagnostics/navigation-dispatch-drift-contract-20261013.md)

后续只读核对还从保留的第二次失败记录算出：采样位姿到实际receipt仅 **1.132700 ms**，其中结果发布到取走 **0.405146 ms**，完整receipt停车尾迹仍失败。因此“完成就立即唤醒”单独不能保证receipt状态安全；它还会触及原20 Hz每步可达速度的语义。这里只分析原记录，没有新benchmark或生产改动，也未另做第二例几何重建。[新增时序核对](../diagnostics/navigation-dispatch-drift-contract-20261013.md#2026-10-14-read-only-review-dispatch-ownership-and-latency)

后续只读复核进一步明确：理想plant在到期或fence时瞬间归零，合法已执行边不能替代包含有限减速/反应延迟的**必需receipt完整停车尾迹**；该尾迹失败也不等于已记录物理碰撞。最后一次replay的Unix计算时间会刷新，而原worker monotonic lease起点保留，不能把两者当作同一时钟对或用最终计算间隔推断原lease年龄。原失败判决不变，尚无新生产修复或新实验；下一步建议是固定几何的成对时间线诊断，并显式记录失败分支与实际生效deadline。[第85阶段复核与最小后续oracle](../diagnostics/navigation-dispatch-next-oracle-20261014.md)

### 探索所有权负结果与旧测试合同修正

移走本次决策私有的observation-cell sets，确实可少复制64个局部BTreeSet；但三对固定进程没有证明完整selection稳定提速，大夹具配对中位数反而退化约5–6%。因此**该性能candidate已撤回，生产仍保留原deep-copy路径**；只保留测试计数和诊断，不把复制字节减少当作实际加速。[负面审计](../diagnostics/exploration-view-ownership-audit-20261014.md)

调查中两边原suite均为56通过/3失败/1 ignored，失败后来确定是过时测试假设，另行只改fixture：

- blind-start把缺失支撑误当成占据障碍。新断言同时要求raw pose collision-clear、raw support为MissingSupport、私有current-body view有支撑且没有伪造原观测，返回goal在原图上有支撑并全身clear
- 两个长机体heading fixture把原 **324个frontier全部标为reached并排除**，所以NoFrontier是夹具结果。修正加入真实未分类recent occupied ring；原static frontier向量不变，独立side-pose先证明radial碰撞、tangential clear。两个sensor模式各80个visited、244个radial碰撞，返回goal全身clear且其radial pose必须精确报InsufficientClearance

最终原workspace release范围 **59 passed / 0 failed / 2 ignored**；原324/324/324/324 diagnostic显式通过，旧失败日志保留。ignored不计入59通过，性能profile没有重跑。生产代码、碰撞/支撑责任、live-source健康和命令时复核均未改变。**这是测试合同修正，不是实机导航拒绝或机器人移动问题修复**。[准确资格](../diagnostics/exploration-fixture-contract-20261014.md)

### 私有等待路径省去未使用的frontier扫描（第81阶段）

仅当本次poll使用私有planning view、同一原始source/notification已消费且没有retry时，跳过随后不会使用的frontier几何准备。当前body/support验证及原Pending/SourceUnavailable仍照常执行；新source、retry、约束重配和实际selection保留原路径。此前被否决的set-move候选仍未采用。

三个独立配对release进程中，20/80/160格宽合成changed-pose等待夹具的**完整private fork＋prepared poll＋discard**中位数分别 **475.858→42.635 µs、12,041.177→73.356 µs、84,518.204→133.852 µs**，扫描单元400/6400/25600→0。实际selection控制没有稳定收益，same-pose本来已短路；blind-footprint仍受body重建主导，历史慢样本保留。没有测得线上触发频率，不能据此说每秒5次完整扫描被消除或真实目标选择更快。

基线和候选surface范围各 **60通过、0失败、4 ignored**；新10场景生命周期/精确错误测试已包含在60项内，其他adapter/controller选集也重叠，不累加。独立MLS planning-fork合同 **4通过**。后续当前生产非cfg(test)的surface library release `cargo check` **exit0、1.35秒**，检查当前本地MLS/routing/types/base依赖；默认feature为空，此次未启用wavefront/runtime，没有link。未重跑Geometry全量LTO或worker-runtime集成，也不构成实机导航修复。[资格与原始范围](../diagnostics/surface-private-wait-frontier-20261014.md)

## 二、地图、接纳、保存与身份

- 完整点云 XYZ 边界归约改为各坐标轴的步进视图，保留全部有限值、精确边界和身份检查。4M 点合成资产完整验证 **259.257 → 15.523 ms**，暖文件系统真实检查点加载 **298.398 → 39.697 ms**；不含 RPC、原生恢复、定位、冷盘或浏览器。[证据](performance-dense-admission.md)
- canonical occupancy 校验在原 1 MiB 块内使用 NumPy。16,777,216 个有效 cell：bytes 完整成员验证 **617.388 → 14.082 ms**，FrozenPayload **569.598 → 12.758 ms**。早期无效输入存在代价：1 MiB 首字节非法的 bytes 拒绝 **0.0047 → 0.1185 ms**；仍在首个非法块停止，没有减少验证。[证据](performance-canonical-occupancy.md)
- assigned 探索检查点列表在单次请求内复用一个已验证的当前资产，但每条仍执行新鲜元数据和来源检查。相同地图、50 检查点、10 KB progress、262,144 occupancy cells：**4254.5 → 175.0 ms**；资产模型解析 **50 → 1**、资产读取 **26,897,850 → 537,957 bytes**。不同地图和 A/B/A 交错不保证受益；检查点正文、progress、目录检查仍保留。canonical 来源字段错误另作正确性修复，性能基线已包含该修复。[证据](performance-checkpoint-listing.md)
- 原生 DenseCloud append 按页更新可见性并复用已序列化整数 key。100k 全静态点、live-key 开启、新 cloud：**12.249 → 10.081 ms**；保留快照场景 **12.936 → 10.253 ms**。24 组合均改善，但配对分配调用次数和累计请求字节完全相同，这是时间优化，不能宣传为降低分配。独立真实模块夹具含文件写入，不代表整个 Geometry 服务。[证据](performance-native-dense-append.md)
- 禁行区无签名来源采用真实观察 ID，显式签名仍保留，去掉该解析路径上的内容哈希；缓存稳定性、变更失效和 fail-closed 行为有回归。legacy 恢复发布只复用同一不可变已接纳 target 中已验证的 digest，去掉一次重复云字节哈希；两项 helper 回归通过，未测耗时，也不是完整恢复流程资格。**legacy 导出、激活及校验仍有可执行哈希**，不能说全仓无哈希或默默删除兼容校验。[审计与修复边界](content-identity-hash-audit-20261013.md)

- schema-1 外层检查点中的 **AR2/AR3 fine archive** 导入assigned路径（`96241b2`）：保留精确字节、原始occupancy/cloud receipts和frame/producer generations，为新不可变owner分配真实ID。该阶段Python有guard的**87项通过**；新导入5次有界root解析、4次全归档遍历，较初版少一次全遍历但未测延迟。22:18原生资格3项通过已提交为 `46db738`：2项实际Python AR2/3请求→transfer sealing/admission/replay，1项AR1现有底层install/next-commit/evidence/fences。该阶段报告的“AR1仍拒绝”是历史范围，已由下述成对扩展推进。[AR2/3阶段](fine-archive-legacy-import-20261013.md)
- **AR1成对兼容扩展已提交 `a4333a3`**，涉及11个生产文件：新增RuntimeStatus field6支持版本声明；缺失/空/未知能力不授予AR1权限，旧服务器仍拒绝。现有GetStatus连接epoch内捕获能力，无新轮询；import/provider/store/Rust配对接受原AR1字节与原receipt，不伪装AR2、不制造revision。既有origin guard仅在catalog原 `os.replace` 线性化点覆盖rename，文件/目录fsync和回调均在runtime/provider锁外；rename前reconnect拒绝，rename后视为有效adoption，后续upload仍重查来源。AR2/3保持原save路径
- AR1资格：**143项Python通过**（包括13项聚焦能力、reconnect/接纳race回归），**4项native通过**和**8项assigned-store通过**分别记录、不累计为全仓结果。实际Python on-disk importer生成请求/字节，经真实native seal、typed admission、cloud-writer install、下一次真实commit及retained replay；Python bridge/status/activation RPC仍为test double。no-hash sentinel在fixture前启用。早期缺最终rename fence的红测保留。此项是兼容与该支持迁移路径的应用哈希移除，**不是全RPC激活、所有legacy哈希移除或延迟提升**，也无部署AR1比例结论。[实现与准确资格](ar1-assigned-import-proposal-20261013.md)

### schema-1地图状态只读取所需的库名（`afcd7a6`）

活动旧版地图的状态同步原先构造完整记忆摘要，只取其中一个active-library name；现改用已有标量getter，仍在定位协调之后、runtime readiness之前读取。**schema-2仍走原assigned状态分支，没有这项提速**；非活动地图不读记忆。

实际 `handle_list_maps` 会先列地图、再读活动workspace，因此完整摘要从 **3次降至1次**；响应确实需要的 `latest_memory_summary` 保留。1k记忆行、八个schema-1地图、真实file-backed SQLite与catalog的离线配对中，完整handler中位数 **412.007→141.150 ms**；单独list_maps为 **127.836→2.350 ms**，不能把后者当完整接口收益。没有网络、实机或所有地图通用加速声明。

状态读取不再顺带执行摘要专属修复或遇到摘要专属错误，这是明确的范围变化，不声称副作用完全相同。直接summary/query及完整workspace继续保留验证、repair和错误；标量异常不吞掉，定位/readiness顺序不变。相关 **129项测试通过，零失败/跳过/排除，application hash attempts=0，framework ETag=0**；包括真实summary后续仍会修复的控制。早期1160项属于该改动前；后续最新1227项合并选择已覆盖这次地图状态改动。[报告](../diagnostics/map-status-scalar-20261014.md)

### 重定位准备：实测基线与目标所有权交接优化

第67阶段直接测量真实私有source-sealing helper，确认大target在送入既有 `spawn_blocking` 之前有同步准备成本。4M点合成target在三个source规模下的完整准备中位数约 **95.93–100.13 ms**；这只是声明的容量夹具，不是用户地图大小或实机典型延迟。原调查没有改并发、lease或actor状态；独立decode/Arc阶段计时不能相加或从总耗时相减。[基线](../diagnostics/relocalization-preparation-audit-20261014.md)

第69阶段 `6eec75e`只改两行生产代码：私有sealed target从 `Arc<[Point3<f64>]>` 改为 `Arc<Vec<Point3<f64>>>`，并用 `Arc::new`接住原decoded Vec，避免再复制所有坐标。source仍用原Arc slice，compute仍读同一slice；没有新缓存、额外寿命、并发/worker、lease释放、scoring、heading或取消规则变化。

- 实际完整helper配对：**1M target / 100k source，6.079899→2.737593 ms**；**4M / 100k，99.732179→38.283897 ms**，三个进程对均改善
- 小target聚合负对照：100k target / 250k source **1.976584→1.996333 ms**，约增加19.749 µs；三对中一对退化15.6%、另两对略改善，不宣称普遍提速或抹去退化
- 18个固定AB/BA/AB诊断进程、**270个记录样本（每臂135）**全部保留；这是重复诊断采样，不是270项独立测试
- 100k/1M/4M target原Vec容量为2.4/24/96 MB；新owner保留该坐标allocation，去掉全尺寸交接副本及其瞬时重叠。最终坐标payload不变。诊断两臂仍独立跑旧Arc-slice stage，**不能用其RSS/HWM证明候选进程省了多少内存**
- **19项精确功能测试通过**，含pointer/capacity/signed-zero、最后读者释放、source/checkpoint owner、lease、边界错误、actor取消/重复/回滚与已有heading规则；非cfg生产binary-target `cargo check`通过（30.952秒），不是链接好的部署程序或全native suite

基线报告中的“仅proposal”是先前状态，现已完成这项窄改动及资格。actor仍同步封口并保留原生命周期lease；没有测实际排队延迟，也没有证明提前解锁map activation。通过已有wrong-heading回归只说明保留所覆盖行为，**不是现场朝向反转问题的诊断或修复**。[配对结果及所有权资格](../diagnostics/relocalization-target-handoff-qualification-20261014.md)

## 三、任务、日志与兼容性

- TaskManager 在原 RLock 下维护活动记录索引，count/bool 不复制状态、不扫描历史，列表只访问活动项。50,000 个 terminal＋50 个 active：list **22.4505 → 0.6645 ms**，锁内 **0.004046 ms**；count **0.000811 ms**。任务列表＋20 条历史摘要 **25.56 → 3.11 ms**。保留终态、隔离、回执和历史所有权；标量读取早期阶段仍有约 20 ms 扫描成本，后续索引才解决它。[最终索引](task-active-index-20261013.md)、[前一阶段](task-active-scalar-reads-20261013.md)
- 日志来源枚举利用原 covering index 按来源跳转，并对稀疏来源回退 DISTINCT。1M 行 / 3 来源、limit-20 页面相关 SQL **58.861 → 0.044 ms**，SQLite 操作约 **4,000,100 → 300**。不含文件刷新、解码、原 ID 核验或 HTTP。全唯一来源基本持平；原 seek-only 方案约十倍退化，已否决。[证据](performance-log-source-queries.md)
- Python 3.8 StrEnum 回退修复是真实正确性修复：warning/error 先前可能被当普通遥测丢弃；现正确进入保护容量并产生反馈及对应日志。未提前按 INFO 过滤，保留排队期间动态日志级别、计数、缺口、独立副本与回放语义。实际小 payload 捕获约 **203 µs/事件**，不据此改写日志所有权或声称整机加速。[证据](python38-strenum-telemetry-20261013.md)

- artifact metadata以规范ID直接探测规范sidecar路径（`ad1f820`），先保留原payload-cache优先级，再回退原递归查找。10k artifacts、Python lookup冷缓存/OS暖缓存、七个不同ID配对：metadata **13.2967→0.1197 ms**，真实调用顺序metadata→raw响应构造 **33.9221→11.0870 ms**；目录项分别 **20,001→0**、**60,004→20,002**。原raw payload发现、hold/取消/range所有权未改，不含网络传输。已索引对照无收益宣称；非法重复ID且无缓存时会优先规范位置，库存仍拒绝重复，不能声称非法数据选择完全等价。最初预热payload-cache方案因冷成本与选择差异被否决。生产改动后**177项聚焦测试通过**。[完整证据](performance-artifact-metadata-access.md)

### Native构建复用：只去掉jobs变化导致的失效

`c5e2f43`仅移除slam-toolbox-sys与gtsam两个build.rs对 `CMAKE_BUILD_PARALLEL_LEVEL` 的rerun watcher；该变量只决定CMake `--parallel`，合法source/prefix/toolchain变化仍触发构建且使用当次jobs。实际Cargo/rustc1.88＋真实copied build scripts、fake CMake的**56次调用**均符合预期：jobs2→3时原版2次CMake、候选0次；源码、dependency prefix、toolchain变化仍各2次，configure/build故障及不改输入重试保留正确失败/恢复。注入故障的预期exit101不误报为Cargo全部exit0。

这是避免重复build-script/CMake调度，不是实测SDK/native object编译或部署耗时收益；未链接SDK。generation-specific prefix仍是合法失效来源，**尚未采用稳定prefix生产修改**。后续便携脚本已在Python3.8.10对最终managed-binding脚本执行64-case；这是扩展后的独立记录，不与最初56-case相加。[证据](../diagnostics/native-build-jobs-reuse-20261014.md)

### SDK源码构建缓存与受管理依赖绑定已成对提交（`35ac900`）

修复的是两个相关问题：换源码pin后继续使用同一个CMake目录会报source-directory mismatch；即使新依赖B编译成功，consumer仍可能沿旧 `Ceres_DIR/TBB_DIR` 编译于A，造成同版本号下的ABI错配。小型真实CMake夹具重现了旧版A→B不重编译consumer、运行时undefined-symbol失败。

新逻辑先读已有CMake cache的source绑定：兼容则保留原目录，不兼容才使用已接纳源码basename对应的构建目录。没有迁移、删除或重命名旧cache；换pin首次需正常编译，重复及回到旧pin可复用各自对象。安装prefix仍按generation隔离，**不是曾因旧时间戳漏重编译而否决的稳定安装prefix方案**。

配套改动只给installer发起的slam2d-worker Cargo调用传入受管理generation prefix；CMake在此模式下从指定prefix选TBB/Ceres，缺包必须失败，离开模式显式清空。开发者非受管理模式的显式包目录仍保留；不修改父进程环境、不增加runtime设置、身份registry或内容哈希。

资格按范围分开：

- **10个source-cache阶段**：真实原编排器＋小型CMake/C++，换pin只重建必要对象，重复/返回A复用，configure/compile失败和重试保持，旧generation库仍可运行
- **13个managed-binding阶段**：真实依赖发现片段，小型同版本不同ABI的A/B编译、链接、执行；B选对包，重复0/2对象重编译，缺包失败，retry、回滚A、切到开发者自定义目录、再切回B通过
- **57项installer身份/环境隔离测试**：包括5项source选择及1项新环境合同，应用hash/network/真实外部进程guards仍启用
- **64项真实Cargo＋fake CMake编排检查**：jobs-only不触发多余构建，source/prefix/toolchain/managed-mode变化正确失效；退出managed模式清空参数，失败/重试和no-op保持

这些不同范围的结果不相加为一个系统测试总数。小型真实构建证明所列pin/重复/回滚行为，**没有完整SDK闭包、工具链/架构迁移、同时改Boost recipe、NEH整套安装回滚或部署耗时资格**。首次fake Boost fixture缺少FOUND标志的失败保留；修复夹具没有弱化产品检查。[source目录复用](../diagnostics/slam2d-source-build-reuse-20261014.md)、[成对依赖绑定](../diagnostics/slam2d-managed-binding-20261014.md)、[被否决的稳定prefix](../diagnostics/slam2d-prefix-audit-20261014.md)

真实已安装SDK包发现也完成两个额外configure/generate检查：直接使用未改的production native CMake文件，在新隔离目录中初次配置、再配置均成功。实际选择的TBB/Ceres及生成include/link路径均指向已有toolchain managed prefix。**没有构建native target、生成其object/library或安装**；这是实际包发现与CMake生成资格，不是SDK编译/链接/运行。该Debian Ceres选项与installer source recipe不同，既有CMP0167/METIS诊断保留，不混入13/64-stage计数。[真实包发现证据](../diagnostics/slam2d-real-package-discovery-20261014-results.json)

## 四、感知与语义记忆

- YOLO 输出归一化直接使用新建分割契约，去掉深复制：20 检测×128 点 **14.587 → 6.011 ms**；64×512 点 **281.206 → 153.505 ms**。后续在此基础上复用不可变坐标和有符号面积，三独立进程的 64×512 点为 **156.353/156.363/151.340 → 134.908/145.363/138.761 ms**。这是两个相继且不同基线的比较，收益不相加；不含模型推理或相机帧率。[证据](../diagnostics/yolo-mask-normalization-copy.md)
- 深度投影保留 bbox 局部支持点索引，省去扩回完整布尔数组再扫描。8 个检测共享 100k 点完整 augment 调用，稀疏场景三进程 **22.961/24.161/23.567 → 21.155/21.218/21.760 ms**。完整调用峰值仍约 9.82 MB；密集局部选择的峰值反而 **2,297,928 → 2,797,082 bytes**，不是普遍省内存。[证据](performance-perception-support-indices.md)
- 有限场景列表按原排序与残余过滤有界读取，够 limit 即停。10k 场景、宽过滤、limit 5：**433.618 → 10.845 ms**，模型及 token 各 **10,000 → 5**；每100项匹配、limit 20：**415.093 → 44.749 ms**，hydrate 2,000 项、token 20 项。无匹配仍须完整检查，不把 Python hydration 数当 SQLite 内部扫描数。[证据](c1-limited-scene-listing-20261013.md)
- summary 使用已有关闭逐结果 readiness 的选项，每条少建三个不用的字典，保留向量读取、来源验证与修复。10k enabled-image 场景峰值 **3,322,676 → 3,136,627 bytes**，耗时 **1430.94 → 1431.65 ms**，没有证明普遍延迟收益。[证据](memory-summary-readiness-20261013.md)
- relation/summary 自然身份写入只浅复制已私有 JSON 解码的模型包装，保留输入序列化隔离、最终返回深复制及事务。2,048 IDs 的完整 service upsert：summary **10.023 → 7.429 ms**，relation **4.038 → 3.197 ms**；小 summary 接近噪声，不含磁盘 fsync 或整场景聚合。[证据](memory-natural-identity-copy-20261013.md)
- 语义地图先按原 Pillow NEAREST 取最终显示像素再着色。4096² 地图、显示宽1280、三进程中位数：完整 PNG 渲染 **1105.0 → 156.6 ms**，进程峰值 RSS **503.5 → 246.2 MiB**；128² 小图 **48.1 → 49.5 ms**，不宣称小图改善。完整 PNG 字节与元数据有对照。[证据](performance-semantic-map-rendering.md)

## 五、网页资源与请求

- assigned 完整点云下载完成后停止空尾块轮询；实时、legacy、未完成下载和重试保持原语义。实际 JS 模块＋虚拟时钟、两点资产、60 秒：请求 **121 → 1**，残留定时器 **1 → 0**。这是请求计数，不是浏览器/GPU/网络吞吐量提升。[证据](performance-dense-polling.md)
- 三维语义区域按实际变化复用几何、材质、纹理。三个区域夹具：单区域 revision 变化的释放资源/新画布 **18/3 → 0/0**，名称变化 **18/3 → 2/1**，轮廓变化 **18/3 → 2/0**；地图绑定变化仍完整重建。另修复误释放 Three Sprite 共享几何体的所有权问题。实际 Three/模块等价检查通过，但模拟 Canvas2D 不等于浏览器像素或 GPU 验证。[证据](performance-semantic-region-reuse.md)

- 新增实时v2增量点云确定性回放（`e3ce1d6`，仅测试）：**230次接受事务、12次重连、24次预期故障/resync**；最大36,878点、37来源页、3render slabs。真实连接/parser/assembler/Three资源对照独立数组oracle，97个geometry全部恰好释放一次；最终无保留slab、pending、accepted base或timer。复制、扫描及scheduled upload计数有精确断言，但不是实测GPU流量、GC、FPS或网络性能；生产代码未改。[证据](../diagnostics/incremental-pointcloud-replay-20261013.md)
- scene graph相机更新把本帧不变量移出节点循环（`6cc8a3a`）：500节点×20帧，clientHeight读取 **10,020→20**，FOV读取和tan各 **10,000→20**；每个scale值严格相等、资源身份不变。500为API上限，不使用超界1000节点探索样本宣传；这是运算计数减少，没有计时或浏览器layout/FPS测量。[证据](../diagnostics/scene-graph-camera-invariants-20261013.md)

### MJPEG浏览器解析：只复制最后选中的一帧

第68阶段仅改两处：parser返回同步使用的临时subarray，consumer选出批次最后完整frame后、进入异步render前才做独立slice。临时view仍可变，依据是私有同步所有权窗口，不把它包装成“不可变快照”；不完整tail复制继续保留，避免小tail牵住整块输入。server共享multipart、SHM独立snapshot、decode admission及生命周期未改。

三独立Node进程、固定synthetic JPEG批次中，256 KiB×8帧的slice-copy字节 **9,443,360→7,608,352**，parser/append/selection中位数 **0.960510→0.758195 ms**；64 KiB×4帧 **0.076956→0.058889 ms**。少复制的量恰为 `(frames−1)×JPEG bytes`。单帧复制数量不变，微小时延方向混合，**不声称单帧加速**。

第一阶段实际生产源通过 **29项Node合同测试**，包含分片/metadata/error/tail等价、async前独立复制、取消和generation变化、真实render-admission函数的fake display/URL清理。初始26项不与最终29相加。计时不含fetch、Blob、真实解码、DOM/GPU、网络或消费者async调度；**仍无实际浏览器验证、FPS/机器人CPU或持续内存结论**。[完整报告](../diagnostics/mjpeg-parser-owned-selection-20261014.md)

### MJPEG第二阶段：每批只分离一次后续缓冲

在第一阶段已只复制selected frame的基础上，新的两行生产改动让完整frame后的remaining暂作同步subarray，解析完一批后才独立复制最终tail，再跨异步边界。selected frame的独立slice不变；no-complete、裁剪、size/error、取消与generation语义不变。临时变量仍可能维持原buffer可达，不宣称立即GC或RSS下降。

第二阶段自己的基线是第一阶段版本，**不能与前一阶段收益相加**。256 KiB×8帧固定批次，slice-copy **7,608,352→262,146 bytes**，同步parser/append/两处copy中位数 **0.700559→0.058919 ms**。多帧固定cohort三个进程均改善；单帧256 KiB **0.023646→0.024097 ms**，4 KiB也有三进程小退化，原样保留，不说非退化。4 MiB incomplete tail仍有大复制，一进程退化，不宣称此类输入获益。

最终 **48项Node合同通过**。历史第一阶段29项和当前源29项兼容检查亦通过，但与48项重叠，不累计；第一阶段计时未重跑。新验证覆盖input在render/next-read期间变更、分片续读、tail、overflow和generation；fetch/display仍mock。未验证真实浏览器、GPU解码、网络分布、FPS、端到端延迟或机器人CPU。[第二阶段报告](../diagnostics/mjpeg-parser-owned-tail-20261014.md)

### 实际浏览器资格的最新边界

官方云浏览器的CDP连接可用。通过它对本次执行器自有HTTP标记服务 `127.0.0.1:18979` 做的一次访问返回 **`net::ERR_CONNECTION_REFUSED`**，没有加载NEH源码或MJPEG页面；标记服务随后已停止。因此实际浏览器MJPEG解码、DOM/GPU及生命周期仍未验证，已有Node合同结果不受影响。这只证明本次loopback访问失败，不代表所有云浏览器或网络不可用。

此结果与早期在执行器启动原生Chromium154时的process-singleton socket **EPERM/exit134** 是两条独立路径，不应合并为“没有浏览器能力”。

### 点云快照预处理：复用同一次边界测量（`a6fe715`）

`_preparePointCloud`中，visibleSource与原src是同一对象时，复用本次已算出的bounds，不再扫描第二遍；不同过滤buffer即使长度相同也仍独立测量。只改一个表达式并更新正常/worker入口版本号，**没有跨调用缓存、不可变source声明或验证削减**。

该调用链是artifact、legacy saved snapshot和旧snapshot fallback；**当前live/assigned增量page以及完整dense模式绕过此processor，不属于收益范围**。三独立Node进程的完整同步processor测量：1M点mono **6.556628→3.729920 ms**，height **65.921241→63.454588 ms**，bounds坐标访问 **6M→3M**。过滤对照扫描数不变，singleFloor等有小退化并保留，不宣称普遍不退化。

最终实际源 **114项合同通过**，含完整position/color/bounds/floor输出、same-length独立过滤、非有限值既有错误与下次调用重新计算所有权；相邻27/21/22/4项suite分开记录，不累加总数。计时不含worker序列化/transfer、viewer admission、rendering、browser/GPU或network，因此不是FPS或机器人CPU收益。[准确范围与结果](../diagnostics/pointcloud-measurement-reuse-20261014.md)

### 点云颜色由单一消费者接管（第82阶段）

私有prepared-result的颜色已是独立Uint8Array，viewer改用普通`THREE.BufferAttribute`接管，省去专用`Uint8BufferAttribute`构造器的再次复制。这**明确收窄了旧别名隔离合同**：调用后再修改result.colors会修改geometry，将同一result交给多个viewer会共享颜色；这些直接复用方式不受支持，不能宣称完全等价。已审计当前同步及worker调用链均单次消费，worker transfer令producer buffer脱离，过期/重复响应被拒绝，没有活动调用方保留颜色再修改；属性子类型也改变，已检查实际renderer依赖数组类型。

三进程固定配对测量完整`_applyPreparedPointCloud`，1M点initial **0.481236→0.142065 ms**，replacement **0.441967→0.104348 ms**，约省 **0.34 ms**及每次3,000,000逻辑复制字节。计时不含processor准备/transfer，不是allocator、RSS、browser、GPU或增量live/page收益；空输入控制的混合微小退化保留，不与前阶段收益累加。

最终 **30项合同通过**，覆盖真实worker handler的Node transfer语义、admission、alias变化、颜色值及资源释放；另外pairing22项和相机/overlay合同分别通过。缺少fixture的native-wire viewer脚本未运行，真实浏览器/WebGL仍未验证。[所有权边界、配对和资格](../diagnostics/pointcloud-owned-colors-20261014.md)

### 前端交付引用核对（第83阶段）

六条HTML/viewer/worker/processor/scene-graph引用均通过本地精确URL唯一性与文件存在检查。发现console.js仍沿用旧查询版本，现仅更新HTML引用为`20261014-mjpeg-owned-tail`；颜色viewer及processor/worker引用保留各自新版本。该改动补上资源URL版本差异，不代表观察到用户旧页面或测得浏览器/proxy缓存行为；已打开的页面仍需正常reload。静态服务的mtime-size ETag机制只作源码检查，没有计算ETag、发网络请求或新跑browser/GPU；这是交付引用资格，不是完整前端重新验收。[六条引用与缓存范围](../diagnostics/frontend-delivery-checklist-20261014.md)

## 六、没有贸然改动的路径

- 传感器进程快照已有 unchanged 省略与 pending 合并；250k 点合成 native-LIO 快照实际投递中位数 **13.318 ms**，短样本维持约20 Hz。暂停接收100 ms会阻塞共用通道中的错误发送，属于已记录限制。没有砍 metadata、限点或增加新通道。[审计](performance-sensor-snapshot-audit.md)
- 二维返航查询每次500×500占据复制500,000 bytes，但暖 Costmap clone 中位数仅 **10.38–12.89 µs**；没有为了字节总量引入复杂所有权接口。三维快照本已共享，不把二维探针当三维全点云复制问题。[审计](../diagnostics/planar-mission-route-copy-probe.md)
- 更新状态/检查已在线程池，状态 GET 不查询远端；重叠 check 仍各自查询，故障重试及等待时分支变化也依赖重新读取。三项 fake-Git 并发探针各 **12 次 Git 调用 / 2 次 ls-remote**，这是调用证据，不是性能收益；未新增合并、锁或缓存。[审计](update-status-source-audit-20261013.md)
- 语义索引已按受影响记录更新。StateStore 浅复制保护旧快照；0/100背景 key时占 helper耗时约 **1.3–2.8%**，1,000 key时约 **4.5–7.7%**，还是包含锁和 StateEntry 构造的上界。没有改变事务后回调、取消隔离或读者所有权。[审计](../diagnostics/semantic-index-update-audit-20261013.md)

- 导航cache/allocation调查（`6ed7e95`）没有发现所审路径无限跨目标保留，但entry上限不是byte预算，该阶段未解释晚窗RSS，后续专门诊断已支持主要allocator retention归因但保留小残差。216次非零guard均通过；稀疏pending-band测试候选虽有41,472次精确等价，多层暖场景仍退化约 **7.6–13.1%**，单层无稳定收益，候选已移除，保留诊断而未改生产算法。[证据](navigation-cache-allocation-audit-20261013.md)
- observation/robot-state/HTTP结果调查（`81a3712`）没有生产修改。16适配器合成ingest **298.94→220.93 µs** 的测试方案需扩大协议，绝对收益不足；JSON size-reuse收益混合且增加精确字节预算复杂性。保留独立结果树、redaction与原完整size enforcement。[证据](observation-state-serialization-audit-20261013.md)
- stream/event调查（`bafa7b7`）没有生产修改。默认100pose、单客户端轨迹编码 **0.219 ms**，8客户端批次 **1.717 ms**；task progress单客户端public projection **0.357 ms**，含threadpool **0.653 ms**。这些不是实际网络或event-loop延迟；对象及嵌套pose可变，不支持泛化identity memoization，因此保留回放、缺口、回调顺序和脱敏。[证据](stream-delivery-audit-20261013.md)

- artifact剩余raw payload直达方案未采用：ID只有kind、没有权威filename，生产允许任意extension（含无点后缀）。小夹具还证明inventory接受“canonical pair＋root同ID孤立payload”，冷递归会选root而直接canonical会改变返回字节。它不同于先前被inventory拒绝的重复sidecar，不能作为纯性能改动；两项tiny诊断通过，未做新计时或生产改动。[边界审计](performance-artifact-metadata-access.md#remaining-payload-lookup-rejected-direct-path-candidate)
- provider pointcloud deep-copy只在full-perception有标定/检测但缓存无可用cloud时回退；video fast path、native LIO及recording走订阅。未找到固定高频polling，PointCloudFrame和ndarray仍可变，浅借用会削弱原隔离。保留getter，只有调用审计，没有额外timing或test结果。[后续传感器审计](performance-sensor-snapshot-audit.md#follow-on-provider-polling-and-ownership-audit)
- 语义记忆完整查询的重复publication predicate审计：5k合法file-backed SQLite fixture，普通两次约 **1.107/1.153秒**；中间profile **1.599秒**，重复半段归因仅 **19.370 ms（约1.2%）**。三次完整输出一致、request-local状态清理、六表count及5000个completed_fresh保持；未实现candidate。重水合、source/provenance验证有各自权威边界，不能跨阶段复用模型/score冒充冗余。旧fuzzy-length候选继续撤回。这是负面归因，不是A/B收益，JSON累计调用时间互相包含不可相加。[新记忆审计](../diagnostics/memory-query-owned-work-20261013.md)

- assigned activation确有同一immutable canonical target在preflight与install各解码构造一次HashSet。1M点单decode诊断约65–76 ms、validate＋decode约137–148 ms，但**未采用保留set方案**：每个大请求多保留约26 MiB（实测27,267,072 mapped bytes），准备发生在transition锁前，缺少server-wide pending-task数量上限；50个set约1.27 GiB仅为并发风险示例，不是实际负载。18个进程是重复诊断、非18独立测试；它未实现完整activation、writer-lock时延或真实peak比较。没有移动准入次序、增加信号量/缓存或声称激活加速。[负面审计](../diagnostics/activation-decode-audit-20261013.md)
- recent ingestion初始基线的100k点XYZ materialization **0.820–0.846 ms**、finite scan **0.143–0.155 ms**，未支持扩大iterator/error-drain重构；两遍parser还会重复decode。保留原接口/错误/格式范围，三个ordinary contracts通过。之后较大的column构造热点才推进上述独立优化，不能把基线归因冒充同一A/B结果。[基线](../diagnostics/recent-ingestion-baseline-20261013.md)

- 重定位candidate-pair索引复用的小型离线夹具没有测出实质、可重复的完整计算收益；未修改生产索引或定位阈值/heading规则。后续同步准备基线与两行target交接优化已独立完成，未采纳该pair-index方案；不把小夹具外推为实机地图代表负载。[负面审计](../diagnostics/relocalization-pair-reuse-audit-20261014.md)

- MCP最终JSONResponse搬进已有projection worker的test-only方案已否决，没有生产改动。真实router/投影/字节预算＋同loop heartbeat与initialize请求显示大**合成容量**响应确会拖延loop，但小offload未实质改善：8 MiB fixture handler **408.391→420.091 ms**、每轮最大heartbeat gap的中位数 **319.133→325.256 ms**。2,340-byte小对照gap **1.490→1.509 ms**。没有把容量夹具当日常流量，GIL是可能机制而非每项延迟的唯一实证归因；不能说“放线程就不阻塞”或伪称整个系统已改善。九项原encoding合同各臂通过、计数/取消行为一致，18次执行不是18独立用例，也未新增广泛集成run。[审计](../diagnostics/mcp-response-heartbeat-20261014.md)

- observation过滤顺序候选未采纳：提早range/FOV判断在全out-of-range合成控制有效，但真实完整view操作的in-range/map-like/planar夹具无一致收益。16 m planar批次 **57.376→59.783 ms**，原ray预算、callback顺序、NaN/重复语义对比保留；没有生产过滤修改。[报告](../diagnostics/observation-filter-order-audit-20261014.md)
- ray traversal用stack `[usize;3]`替代临时axis Vec也未采纳：完整directional view批次 **17.688→19.163 ms**；10k条长三轴tie ray批次 **26.566→35.766 ms**，三对均退化。8,600组顺序callback/结果等价通过，最终routing **15 passed / 0 failed / 3 ignored**，两个axis诊断另显式通过。源码里少一个Vec不证明实际allocator次数减少；没有采纳生产射线优化，也不是完整探索/实机导航资格。[报告](../diagnostics/observation-stack-axes-audit-20261014.md)

- 第78阶段incremental pointcloud有限值检查候选未采纳：把数组 `.every(Number.isFinite)`替为同顺序scalar检查，113项合同通过，但完整apply结果混合。1M height reset **53.773272→48.711075 ms**，ground8192点reset **24.889073→26.246019 ms**，三进程有两次退化且一次显著，不能用height收益抵消。没有生产改动、额外分支或allocator/RSS结论，数组语法不证明V8真分配heap对象。[负面审计](../diagnostics/pointcloud-finite-scalar-audit-20261014.md)

- 第84阶段wavefront lazy-inflation候选未采纳：只在首个实际评分候选出现时构造膨胀图，96次精确对照通过，但三固定进程的完整selector结果不支持普遍收益。free401 **35.210→34.370 ms**、filtered401 **33.780→33.494 ms**方向混合，normal401 **80.698→82.234 ms**；明显收益仅见all-occupied立即停止合成控制 **2.524→0.135 ms**。该crate **63通过、0失败、1 ignored**，profile另显式执行；只保留cfg(test)诊断，生产selector未改，不与surface60项累加。[负面审计](../diagnostics/wavefront-lazy-inflation-audit-20261014.md)
- 后续源码审计未发现大规模selected-goal拷贝；有序mission各leg输入和取消/错误短路不同，不能直接并行。body-support重建确有更深成本，但仅部分connector/rounding路径存在局部复用机会，需要真实branch频率与严格witness合同；没有新改动或计时。[剩余热点与生产check范围](../diagnostics/exploration-remaining-hotspots-20261014.md)

## 七、验证状态与未完成边界

以下按实际版本和范围分别报告，重叠测试数不相加，也不把历史通过等同最终 HEAD 全通过：

- 最新独立范围：探索test-only fixture修正后59通过/2 ignored；MJPEG第二阶段48项Node合同；真实SDK包初次及复用configure两次通过且未构建/安装native target。MCP worker方案与探索set-move均未采纳生产修改，不算新增优化收益


- 最新target交接为19项精确功能通过、18个诊断进程/270样本及非cfg Geometry binary-target check；第一阶段MJPEG为29项Node合同通过，第二阶段48项另列。两者是Rust/JS各自窄资格，不与既有Python1227或Runtime236混成“全HEAD通过”


- **最新recent-column冻结窗口**：MLS **322 passed / 0 failed / 17 ignored，20.22秒**；Stack ingestion contracts **3通过**、原qualification **15通过/8 ignored**、source-health/preflight两个exact通过、六项原live分别通过；Runtime **236通过 / 58 ignored / 5 UDS filtered，56.50秒**。新增focused MLS含在322中，不另加；各轮重复qualification不累计
- 这一轮六live的OFF/ON耗时分别为known-drop **7.08/6.93秒**、box **10.95/12.71秒**、wall **10.51/7.69秒**，只是原fixture资格，不是速度对照。原60秒窗口、1秒source TTL、容差及collision/braking oracle不变
- Stack/Runtime binary均冻结于`c5e2f43` watcher变更前；这一轮结果不是“由后来的e5854d7 HEAD全量构建”的声明。watcher两文件另经56-case测试，不改navigation算法。后续所有报告保持各自source window；没有全仓、实机或未过滤UDS通过声明


- schema-1地图状态改动前的Python集成 **3.8.10，1160 passed / 0 failed / 0 skipped / 1 deselected，62.13秒**，既有httpx warning1条，exit0。57-target去重并集实际收集1161项；唯一排除的是会执行legacy digest的literal pred1 manifest fixture。以前排除的abrupt-exit测试本次已包含并通过，不再把它列为当前环境阻塞。执行HEAD为`3073f85`前后不变，当时后续native/文档提交未改Python生产代码，但之后`afcd7a6`改变地图状态路径，该1160结果只作历史记录；本轮新1227结果见下项；便携runner只重做collection确认1160节点，不把它称作再次执行。[集成报告](../diagnostics/python-guarded-integration-20261013.md)
- 最新合并Python资格 **1227 passed / 0 failed / 1 exact pred1 deselected，65.42秒**，保留1条既有httpx warning。59个唯一目标包含原57目标、地图状态10项和完整installer身份套件57项；不是仅运行新增6项installer测试。application hash attempts **0**，精确框架mtime-size ETag例外 **4次**。执行前HEAD为`35ac900`、后为`041b6f1`，期间前进的是负面诊断/文档提交，生产代码未变；不把两个HEAD说成相同。129/1160/57与该选择重叠，不累计。本次只有一次实际集成执行，outer runner66.061秒，测试耗时不是性能对照；installer在此运行中仍使用fake外部工作，不重复tiny真实CMake资格。[新合并报告](../diagnostics/python-guarded-integration-20261014.md)。高密度recent负载仍仅是一次candidate-only live资格，不称paired性能验证
- 该Python执行在application/test imports前安装guard，**application hash attempts=0**；Starlette原FileResponse的mtime-size metadata ETag有**4次精确限定框架例外**，不读文件payload。不能称零框架hash，也不与早期572/652/177/143相加
- 较早moving robustness阶段Runtime普通范围：**236 passed / 0 failed / 58 ignored / 5 UDS excluded，56.45秒**，以及另外显式执行的两个moving恢复arm。233/235通过属于之前源状态，不重复累计。[范围](../diagnostics/navigation-recovery-robustness-20261013.md)
- 历史近期碰撞优化MLS普通库 **320 passed / 16 ignored**；最终Scan default-feature **53 passed / 1 ignored**，profile **56 passed / 5 ignored**，non-test lib release check通过；两个Scan套件重叠不合并。五例current-guide和匹配复用diagnostic另外显式通过，不暗示其余ignored已运行
- 新native集成基于冻结的`8949d9d`加scheduler test fixture；未在build时采样Git revision，23:34物化报告时的`79bb33f`不冒充build-time provenance。**两个非cfg(test) release cargo check通过**：Stack binary目标7.21秒、Geometry binary目标1分21秒；它们不是完整链接的部署binary或硬件SDK资格
- 同轮Stack release测试构建1分34秒；普通资格 **15 passed / 0 failed / 8 ignored / 282 filtered**。源健康和codec/geometry/body preflight两个exact项、known-drop/box/wall各OFF/ON六个原live exact项逐进程显式运行，全部各1通过。原窗口/TTL/容差/oracle未改；场景耗时不是A/B加速。[最新native报告](../diagnostics/final-native-integration-20261013.md)
- Native 在 `9eb9617` 的扩展选择 **208 passed / 7 ignored**；此前不同范围147通过单独保留。后续 forbidden-zone21项、retained verifier2项分别验证，不构成最终原生全套通过。早期误执行五项实际 checksum 测试已在原生报告记录并排除扩展计数，未在限制传达后重复。[原生范围与例外](native-offline-regression-20261013.md)
- 早期 checkpoint/dense 报告的旧 fixture 失败保留历史。后续按现有 canonical 合同修复预期并重跑609/652范围，并非修改生产接纳逻辑让旧测试通过；不可继续把这些旧失败描述为当前未解决缺陷

- 新增artifact **177项**、早期AR2/3 Python **87项**、后续AR1 Python **143项**各自为聚焦选择，互有重叠，不累计为新的全仓总数。原生早期3项已提交46db738，后续4项和8项覆盖a4333a3对应变化，不构成整个原生程序全通过。图相机与live点云仍是独立JS合同/计数验证；负面实验不是已投产优化
- cache调查另披露一次流程偏差：约21:18/21:21 UTC曾各执行一次人工binary SHA-256 provenance计算，违反本轮禁止手工哈希要求。旧记录保留、未重算；此后不再执行此类计算。该事实不能被早期checksum例外的说明覆盖，也不能称整轮严格无哈希

仍需明确保留：

1. **真实 UDS 未通过**：五项曾在 Unix socket 初始化返回 EPERM。最小 AF_UNIX 探针在正常及获准执行中均失败；没有禁用安全设置或绕过限制，允许该原语的环境还需原断言复验
2. **真实浏览器/WebGL 未验证**：官方云浏览器CDP可用，但一次访问执行器自有loopback标记服务返回ERR_CONNECTION_REFUSED，未加载任何NEH源码；服务已停止。早期原生Chromium的socket EPERM是独立失败。Node/Three结果有效，但不能替代真实MJPEG解码、像素、GPU或浏览器生命周期验收
3. **实机未验证**：未运行真实传感器/执行器闭环、机器人CPU/RSS、真实网络/刹车或硬实时验收；没有将合成动力学、模型外归一化、暖缓存测量外推到硬件
4. **legacy哈希与兼容仍在**：已去掉受审重复工作，并成对扩展受支持保存源的assigned迁移；其他仍承担接纳验证的legacy导出/激活接口没有整体移除。不得外推为全仓或所有外部客户端无哈希
5. **局部修复通过不等于全部边界完成**：完整NavScheduler固定障碍恢复已通过并提交；原60秒失败、两次短程停车尾迹失败保留。独立dispatch漂移诊断未作生产修复，真实执行器50 ms hold/expiry未获证明。AR1成对扩展已资格验证，不等于完整激活RPC或实机部署
6. **交付与验收分开**：已确认的代码、测试与报告均已普通推送；最终文档更新继续按相同链路提交、读回和保存。当前分支不匹配仓库push CI的main/feature触发条件，不能称远端CI通过或已部署

### 后续验证优先顺序

1. **真实负载与CPU**：先在物理机器人、真实source和持续任务下分项测量CPU/RSS与调度负载，确认本轮局部收益是否覆盖现场热点
2. **dispatch制动证明**：补齐实际receipt状态、已应用转移历史与hold/expiry合同，证明延迟后的完整停车尾迹，而非仅依赖采样时guard；保留原receipt失败，先区分理想执行和必需停车包络。[诊断范围](../diagnostics/navigation-dispatch-next-oracle-20261014.md)
3. **环境资格补齐**：在允许AF_UNIX的环境复验5项UDS；真实浏览器验证MJPEG解码、Worker/页面生命周期及WebGL资源
4. **legacy迁移**：逐接口保留接纳/兼容证据后再移除剩余内容哈希，不把当前成对导入能力外推为全部legacy路径完成

这是剩余工作优先级，不表示已安排实机执行或新增交付承诺。

## 八、最终复查与交付范围

- 私有wait-path与颜色接管已按上文资格落地，lazy-inflation已否决；dispatch只读复查已成文，没有新增生产修复。五处近期核心改动另经独立只读审查，未发现具体可触发的回归：target Arc<Vec>、私有等待、snapshot bounds复用、颜色接管、MJPEG tail分离。该审查没有新增测试、构建、计时或实机证据
- 交付截点与恢复检查点如前。原始`refactor/map-explore-nav`检出仍干净且保持`3be3568bbc2d9ddcbec8f32d9a592d36f8633688`，所有本轮修改在唯一工作分支连续提交
- 工作窗口截至北京时间11:00（03:00 UTC）；实机、UDS、实际浏览器和dispatch时序边界仍保留，不因Git推送或只读审查通过而改变

## 附：证据阅读顺序

本文所有链接为仓库内专项报告；其中的 JSON/JSONL 保存原始测量或结构化汇总。各报告自带具体基线、方法、fixture、命令及历史失败。工作区原始日志不是远端已发布文件，若后续交付需要独立复现，应随最终备份核对。

- 导航数值与控制范围：[增量记录](performance-incremental-updates.md)、[完整位姿查询](body-recent-query-reuse-20261013.md)、[持续运行JSON](navigation-sustained-worker-20261013-results.json)
- 搜索的正负结果：[priority arms](../diagnostics/local-search-priority-results-20261013.json)、[真实guide fallback](../diagnostics/recent-guide-fine-escape-results-20261013.json)
- 地图：[dense接纳](performance-dense-admission.md)、[occupancy结果](performance-canonical-occupancy-results.json)、[checkpoint配对结果](performance-checkpoint-listing-results.jsonl)、[原生append](performance-native-dense-append.md)
- 任务/日志：[active-index结果](task-active-index-results-20261013.json)、[日志SQL结果](performance-log-source-queries-results.jsonl)
- 感知/记忆：[分割三进程](../diagnostics/yolo-mask-normalization-points-results.json)、[投影三进程](performance-perception-support-indices-results.jsonl)、[有限列表](c1-limited-scene-listing-20261013-results.json)、[summary](memory-summary-readiness-20261013-results.json)、[自然身份](memory-natural-identity-copy-20261013-results.json)
- 网页：[区域资源计数](../performance-evidence/semantic-region-reuse-counts.json)、[点云轮询合同](performance-dense-polling.md)
- 版本相关回归：[Python时间线](overnight-python-integration-20261013.md)、[原生离线范围](native-offline-regression-20261013.md)、[身份与剩余legacy路径](content-identity-hash-audit-20261013.md)

- 新增证据：[artifact canonical配对JSON](../diagnostics/artifact-canonical-lookup-20261013-results.json)、[实时点云回放JSON](../diagnostics/incremental-pointcloud-replay-20261013-results.json)、[相机运算JSON](../diagnostics/scene-graph-camera-invariants-20261013-results.json)、[cache负面结果](../diagnostics/navigation-cache-allocation-audit-20261013-results.json)、[stream结果](../diagnostics/stream-delivery-audit-20261013-results.json)

- 最新验证与负面调查：[RSS逐drain结果](../diagnostics/navigation-rss-ownership-20261013-results.json)、[Scan复用结果](../diagnostics/scan-polyline-reuse-results-20261013.json)、[五例真实guide](../diagnostics/recent-guide-five-case-eligibility-results-20261013.json)、[Python逐node与guard](../diagnostics/python-guarded-integration-20261013-results.json)、[moving恢复](../diagnostics/navigation-recovery-robustness-20261013.json)、[native精确选择及provenance](../diagnostics/final-native-integration-20261013-results.json)、[记忆query归因](../diagnostics/memory-query-owned-work-20261013-results.json)

- 最新阶段：[activation decode机会与保留风险](../diagnostics/activation-decode-audit-20261013.md)、[recent ingestion原始基线](../diagnostics/recent-ingestion-baseline-20261013.md)、[列构造资格及12-cohort样本](../diagnostics/recent-column-build-qualification-20261014-results.json)、[jobs-only构建56-case](../diagnostics/native-build-jobs-reuse-20261014-results.json)

- 后续阶段：[地图状态完整handler配对结果](../diagnostics/map-status-scalar-20261014-results.json)、[129项相关回归](../diagnostics/map-status-scalar-20261014-regression.json)、[高密度recent完整负载结果](../diagnostics/dense-recent-qualification-20261014-results.json)

- 第64阶段：[source-cache真实编排结果](../diagnostics/slam2d-source-build-reuse-20261014-results.json)、[managed-binding与64-case结果](../diagnostics/slam2d-managed-binding-20261014-results.json)

- 最新Python：[2026-10-14合并资格](../diagnostics/python-guarded-integration-20261014.md)、[1227个唯一node结果与原始guard记录](../diagnostics/python-guarded-integration-20261014-results.json)

- 第67–69阶段：[真实重定位准备基线](../diagnostics/relocalization-preparation-audit-20261014.md)、[基线45样本](../diagnostics/relocalization-preparation-audit-20261014-results.json)、[MJPEG选帧29项及三进程配对](../diagnostics/mjpeg-parser-owned-selection-20261014-results.json)、[target交接270样本与19项资格](../diagnostics/relocalization-target-handoff-qualification-20261014-results.json)

- 最新独立调查与第二阶段：[MCP loop与编码结果](../diagnostics/mcp-response-heartbeat-20261014-results.json)、[探索set所有权负面结果](../diagnostics/exploration-view-ownership-audit-20261014-results.json)、[探索fixture合同59项结果](../diagnostics/exploration-fixture-contract-20261014-results.json)、[MJPEG第二阶段48项及配对](../diagnostics/mjpeg-parser-owned-tail-20261014-results.json)

- 第76–77阶段：[snapshot测量复用配对及114合同](../diagnostics/pointcloud-measurement-reuse-20261014-results.json)、[过滤顺序负面结果](../diagnostics/observation-filter-order-audit-20261014-results.json)、[stack-axis负面结果与routing资格](../diagnostics/observation-stack-axes-audit-20261014-results.json)

- 第78阶段：[scalar finite-check完整apply负面样本](../diagnostics/pointcloud-finite-scalar-audit-20261014-results.json)

- 第81–82阶段：[私有等待84个计时与42个精确跨binary对照](../diagnostics/surface-private-wait-frontier-20261014-results.json)、[颜色接管30项合同及三进程配对](../diagnostics/pointcloud-owned-colors-20261014-results.json)

- 第84阶段：[wavefront 96对照及三进程负面样本](../diagnostics/wavefront-lazy-inflation-audit-20261014-results.json)
