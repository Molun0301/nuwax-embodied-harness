# NEH 云端夜间开发进度（持续更新）

更新时间：2026-10-14 05:17 北京时间（2026-10-13 21:17 UTC）。本轮仍在进行，计划继续至北京时间 2026-10-14 11:00；本文不是收尾或整机验收声明。

## 当前结论与交付状态

已在导航、地图接纳与保存、任务查询、日志、感知、记忆和网页显示中落实多项有证据的局部优化，同时补齐所有权、兼容性和持续运动回归。收益各有测量范围，不能相加成整机加速比例；没有证据证明机器人 98% CPU 问题已解决，也没有声称所有模块都已优化。

- 仓库：`Molun0301/nuwax-embodied-harness`
- 工作分支：`fix/v3-legacy-cloud-seal-20261006`
- 已确认的远端基线：`65d15c5262af71ee0ab532133bd0afdcb5e75911`
- 本文核对的本地代码状态：`efe424c78458beef248e3b247258dd10cc820544`，相对上述基线新增 37 个本地提交；本文自身及后续工作不包含在此计数中
- 这 37 个提交已保存到同一 Library 检查点 v19。云环境重置后 CLI 登录缺失，Git 推送仍受阻；不能把本地提交或备份当作 GitHub 已更新。远端、检查点和后续提交状态需在截止前再次核对

下面的“旧 → 新”均采用各专项报告的匹配基线，不意味着统一对比上述远端版本。局部连续优化的收益也不相加。

## 一、导航、碰撞与探索

### 已落实的性能改动

- 最新实际位姿的完整停车轨迹复核结束后，保留刚构造的当前支撑视图，避免随后的普通刷新重做相同工作。每次复核仍强制刷新并完整验证；旧选择位姿分支仍还原临时视图。12,800 地面列、自体回波支撑派生夹具中，“复核＋随后视图刷新”中位数 **33.309234 → 24.670447 ms**；800 列为 **23.853009 → 17.584206 ms**。完整地面负对照基本不变。这不是完整下一拍选择或整个控制循环耗时。[证据](performance-incremental-updates.md#最新实际位姿复核保留已构造的当前视图2026-10-13)
- 前沿局部失效使用已有列索引枚举脏范围，稀疏大范围仍回退原槽位扫描。131,072 支撑单元夹具仅访问 361 列，局部几何刷新 **4700.015 → 1498.867 µs**；8,192 单元 **966.890 → 833.719 µs**。候选、收益、代价与冷重建比较一致；不含导航可达搜索和调度排队。[证据](performance-incremental-updates.md#前沿局部失效按既有列索引枚举2026-10-13)
- 近期障碍碰撞查询只构造需要的高度行，并在一次完整位姿查询内复用相邻细列所属的同一粗列。10 cm 粗网格 / 5 cm 来源、8 帧清空场景 **294.08 → 159.64 µs**，门洞 **297.48 → 156.82 µs**；1 帧障碍拒绝 **17.18 → 9.46 µs**。基线为 `bb1a14f` 完整查询，9 对交替测量，暖来源缓存、刻意绕过位姿结果缓存。等分辨率、无 recent、coarse-only 负对照没有实质收益；没有分配器/RSS或整机 CPU 测量。[证据与完整结果](body-recent-query-reuse-20261013.md)

### 持续运行与搜索结论

- 局部规划 OFF/ON 分别运行 **305.845 / 305.671 秒 live 控制时间**，完成 **49 / 47 个目标**。合计 **96 个目标、11,438 个非零回执、42,724 条实际理想 plant 运动边**均经原碰撞及停车尾迹 oracle 检查；实际接受非零命令的积分时间分别为 **291.985 / 279.719 秒**，不能把全部 live 时间称为持续非零运动
- 真实 `RouteFollower`、`PlannerWorker` 和 5 Hz 合成来源跨目标保留，覆盖近期远端非地面点变化、测量抖动、目标/路径替换、取消重启。OFF/ON 最大复核 **20.682 / 33.732 ms**，最大消费者间隔 **53.454 / 58.266 ms**；后者沿用 ≤75 ms 的调度容许，不是严格 50 ms 实时保证
- 这是理想命令积分 plant；不是物理机器人、完整 `NavScheduler`/memq 边界、执行器延迟或真实刹车验证。RSS 首尾有增长，不能宣称零泄漏；首次地面分类夹具与修正后的非地面障碍夹具分别保留。见[持续运行报告](navigation-sustained-worker-20261013.md)
- 当前真实 recent-guide 的新机体敏感回归：7 个实际 recent hints 改变引导，必须先完成精确 **1 cm 直行**才能脱离初始约束；现有 fallback 已能在 **3 次展开、15 个节点、frontier peak 12** 内返回 **119 个全部复扫的位姿**。这是现有能力的正向回归，生产启发式、预算、原语和容差均未改变。[证据](diagnostics/recent-guide-fine-escape-regression-20261013.md)
- 静态误导引导的五个独立可行夹具中，原排序、放松下界和四倍放松下界均耗尽 256 次展开；仅测试用 oracle 引导在 3–6 次展开成功。没有把 oracle 当作已实现算法，没有量化位姿或增大预算。现有 recent-guide 的另一开放绕障夹具本来就无需进入 local search。[负面结果](diagnostics/local-search-priority-audit-20261013.md)、[归因](diagnostics/local-search-frontier-audit-20261013.md)

## 二、地图、接纳、保存与身份

- 完整点云 XYZ 边界归约改为各坐标轴的步进视图，保留全部有限值、精确边界和身份检查。4M 点合成资产完整验证 **259.257 → 15.523 ms**，暖文件系统真实检查点加载 **298.398 → 39.697 ms**；不含 RPC、原生恢复、定位、冷盘或浏览器。[证据](performance-dense-admission.md)
- canonical occupancy 校验在原 1 MiB 块内使用 NumPy。16,777,216 个有效 cell：bytes 完整成员验证 **617.388 → 14.082 ms**，FrozenPayload **569.598 → 12.758 ms**。早期无效输入存在代价：1 MiB 首字节非法的 bytes 拒绝 **0.0047 → 0.1185 ms**；仍在首个非法块停止，没有减少验证。[证据](performance-canonical-occupancy.md)
- assigned 探索检查点列表在单次请求内复用一个已验证的当前资产，但每条仍执行新鲜元数据和来源检查。相同地图、50 检查点、10 KB progress、262,144 occupancy cells：**4254.5 → 175.0 ms**；资产模型解析 **50 → 1**、资产读取 **26,897,850 → 537,957 bytes**。不同地图和 A/B/A 交错不保证受益；检查点正文、progress、目录检查仍保留。canonical 来源字段错误另作正确性修复，性能基线已包含该修复。[证据](performance-checkpoint-listing.md)
- 原生 DenseCloud append 按页更新可见性并复用已序列化整数 key。100k 全静态点、live-key 开启、新 cloud：**12.249 → 10.081 ms**；保留快照场景 **12.936 → 10.253 ms**。24 组合均改善，但配对分配调用次数和累计请求字节完全相同，这是时间优化，不能宣传为降低分配。独立真实模块夹具含文件写入，不代表整个 Geometry 服务。[证据](performance-native-dense-append.md)
- 禁行区无签名来源采用真实观察 ID，显式签名仍保留，去掉该解析路径上的内容哈希；缓存稳定性、变更失效和 fail-closed 行为有回归。legacy 恢复发布只复用同一不可变已接纳 target 中已验证的 digest，去掉一次重复云字节哈希；两项 helper 回归通过，未测耗时，也不是完整恢复流程资格。**legacy 导出、激活及校验仍有可执行哈希**，不能说全仓无哈希或默默删除兼容校验。[审计与修复边界](content-identity-hash-audit-20261013.md)

## 三、任务、日志与兼容性

- TaskManager 在原 RLock 下维护活动记录索引，count/bool 不复制状态、不扫描历史，列表只访问活动项。50,000 个 terminal＋50 个 active：list **22.4505 → 0.6645 ms**，锁内 **0.004046 ms**；count **0.000811 ms**。任务列表＋20 条历史摘要 **25.56 → 3.11 ms**。保留终态、隔离、回执和历史所有权；标量读取早期阶段仍有约 20 ms 扫描成本，后续索引才解决它。[最终索引](task-active-index-20261013.md)、[前一阶段](task-active-scalar-reads-20261013.md)
- 日志来源枚举利用原 covering index 按来源跳转，并对稀疏来源回退 DISTINCT。1M 行 / 3 来源、limit-20 页面相关 SQL **58.861 → 0.044 ms**，SQLite 操作约 **4,000,100 → 300**。不含文件刷新、解码、原 ID 核验或 HTTP。全唯一来源基本持平；原 seek-only 方案约十倍退化，已否决。[证据](performance-log-source-queries.md)
- Python 3.8 StrEnum 回退修复是真实正确性修复：warning/error 先前可能被当普通遥测丢弃；现正确进入保护容量并产生反馈及对应日志。未提前按 INFO 过滤，保留排队期间动态日志级别、计数、缺口、独立副本与回放语义。实际小 payload 捕获约 **203 µs/事件**，不据此改写日志所有权或声称整机加速。[证据](python38-strenum-telemetry-20261013.md)

## 四、感知与语义记忆

- YOLO 输出归一化直接使用新建分割契约，去掉深复制：20 检测×128 点 **14.587 → 6.011 ms**；64×512 点 **281.206 → 153.505 ms**。后续在此基础上复用不可变坐标和有符号面积，三独立进程的 64×512 点为 **156.353/156.363/151.340 → 134.908/145.363/138.761 ms**。这是两个相继且不同基线的比较，收益不相加；不含模型推理或相机帧率。[证据](diagnostics/yolo-mask-normalization-copy.md)
- 深度投影保留 bbox 局部支持点索引，省去扩回完整布尔数组再扫描。8 个检测共享 100k 点完整 augment 调用，稀疏场景三进程 **22.961/24.161/23.567 → 21.155/21.218/21.760 ms**。完整调用峰值仍约 9.82 MB；密集局部选择的峰值反而 **2,297,928 → 2,797,082 bytes**，不是普遍省内存。[证据](performance-perception-support-indices.md)
- 有限场景列表按原排序与残余过滤有界读取，够 limit 即停。10k 场景、宽过滤、limit 5：**433.618 → 10.845 ms**，模型及 token 各 **10,000 → 5**；每100项匹配、limit 20：**415.093 → 44.749 ms**，hydrate 2,000 项、token 20 项。无匹配仍须完整检查，不把 Python hydration 数当 SQLite 内部扫描数。[证据](c1-limited-scene-listing-20261013.md)
- summary 使用已有关闭逐结果 readiness 的选项，每条少建三个不用的字典，保留向量读取、来源验证与修复。10k enabled-image 场景峰值 **3,322,676 → 3,136,627 bytes**，耗时 **1430.94 → 1431.65 ms**，没有证明普遍延迟收益。[证据](memory-summary-readiness-20261013.md)
- relation/summary 自然身份写入只浅复制已私有 JSON 解码的模型包装，保留输入序列化隔离、最终返回深复制及事务。2,048 IDs 的完整 service upsert：summary **10.023 → 7.429 ms**，relation **4.038 → 3.197 ms**；小 summary 接近噪声，不含磁盘 fsync 或整场景聚合。[证据](memory-natural-identity-copy-20261013.md)
- 语义地图先按原 Pillow NEAREST 取最终显示像素再着色。4096² 地图、显示宽1280、三进程中位数：完整 PNG 渲染 **1105.0 → 156.6 ms**，进程峰值 RSS **503.5 → 246.2 MiB**；128² 小图 **48.1 → 49.5 ms**，不宣称小图改善。完整 PNG 字节与元数据有对照。[证据](performance-semantic-map-rendering.md)

## 五、网页资源与请求

- assigned 完整点云下载完成后停止空尾块轮询；实时、legacy、未完成下载和重试保持原语义。实际 JS 模块＋虚拟时钟、两点资产、60 秒：请求 **121 → 1**，残留定时器 **1 → 0**。这是请求计数，不是浏览器/GPU/网络吞吐量提升。[证据](performance-dense-polling.md)
- 三维语义区域按实际变化复用几何、材质、纹理。三个区域夹具：单区域 revision 变化的释放资源/新画布 **18/3 → 0/0**，名称变化 **18/3 → 2/1**，轮廓变化 **18/3 → 2/0**；地图绑定变化仍完整重建。另修复误释放 Three Sprite 共享几何体的所有权问题。实际 Three/模块等价检查通过，但模拟 Canvas2D 不等于浏览器像素或 GPU 验证。[证据](performance-semantic-region-reuse.md)

## 六、没有贸然改动的路径

- 传感器进程快照已有 unchanged 省略与 pending 合并；250k 点合成 native-LIO 快照实际投递中位数 **13.318 ms**，短样本维持约20 Hz。暂停接收100 ms会阻塞共用通道中的错误发送，属于已记录限制。没有砍 metadata、限点或增加新通道。[审计](performance-sensor-snapshot-audit.md)
- 二维返航查询每次500×500占据复制500,000 bytes，但暖 Costmap clone 中位数仅 **10.38–12.89 µs**；没有为了字节总量引入复杂所有权接口。三维快照本已共享，不把二维探针当三维全点云复制问题。[审计](diagnostics/planar-mission-route-copy-probe.md)
- 更新状态/检查已在线程池，状态 GET 不查询远端；重叠 check 仍各自查询，故障重试及等待时分支变化也依赖重新读取。三项 fake-Git 并发探针各 **12 次 Git 调用 / 2 次 ls-remote**，这是调用证据，不是性能收益；未新增合并、锁或缓存。[审计](update-status-source-audit-20261013.md)
- 语义索引已按受影响记录更新。StateStore 浅复制保护旧快照；0/100背景 key时占 helper耗时约 **1.3–2.8%**，1,000 key时约 **4.5–7.7%**，还是包含锁和 StateEntry 构造的上界。没有改变事务后回调、取消隔离或读者所有权。[审计](diagnostics/semantic-index-update-audit-20261013.md)

## 七、验证状态与未完成边界

以下按实际版本和范围分别报告，重叠测试数不相加，也不把历史通过等同最终 HEAD 全通过：

- 最新 occupancy/perception/natural-identity 组合：Python **3.8.10，30个不重复目标模块，572 passed / 0 failed / 1 deselected，42.26秒**。此轮早于随后所有提交，不替代最终快照验证。更早 `bb1a14f` 的652通过为另一跨栈选择；不与572累计。[完整时间线](overnight-python-integration-20261013.md)
- 持续运动 fixture 后 Runtime 普通范围：**233 passed / 0 failed / 53 ignored / 5 filtered，49.41秒**；五个真实 UDS 项明确排除，ignored 不算通过
- 最近碰撞优化 MLS 普通库：**320 passed / 0 failed / 16 ignored**。随后 Scan default-feature **50 passed / 1 ignored**，profile **52 passed / 4 ignored**；两者重叠且版本范围不同，不合并
- 导航 Stack 普通资格 **15 passed / 8 ignored**，源健康正负例和 preflight 分别通过，六项原 live OFF/ON场景显式通过；场景耗时不是配对加速证据。此前失败与夹具修订历史保留
- Native 在 `9eb9617` 的扩展选择 **208 passed / 7 ignored**；此前不同范围147通过单独保留。后续 forbidden-zone21项、retained verifier2项分别验证，不构成最终原生全套通过。早期误执行五项实际 checksum 测试已在原生报告记录并排除扩展计数，未在限制传达后重复。[原生范围与例外](native-offline-regression-20261013.md)
- 早期 checkpoint/dense 报告的旧 fixture 失败保留历史。后续按现有 canonical 合同修复预期并重跑609/652范围，并非修改生产接纳逻辑让旧测试通过；不可继续把这些旧失败描述为当前未解决缺陷

仍需明确保留：

1. **真实 UDS 未通过**：五项曾在 Unix socket 初始化返回 EPERM。最小 AF_UNIX 探针在正常及获准执行中均失败；没有禁用安全设置或绕过限制，允许该原语的环境还需原断言复验
2. **真实浏览器/WebGL 未验证**：Chromium154保留正常沙箱，在加载本地页面前因 process-singleton socket EPERM 退出134；Node/Three检查不能替代像素、GPU和浏览器生命周期验收
3. **实机未验证**：未运行真实传感器/执行器闭环、机器人CPU/RSS、真实网络/刹车或硬实时验收；没有将合成动力学、模型外归一化、暖缓存测量外推到硬件
4. **legacy哈希与兼容仍在**：只移除经证据支持的重复工作；未移除还承担接纳验证的legacy接口。完整迁移需要成对生产者/消费者设计与验证
5. **最终交付仍在进行**：剩余工作、后续回归、最终提交/备份和远端状态应在截止前更新本文。当前不能称全部完成或已推送

## 附：证据阅读顺序

本文所有链接为仓库内专项报告；其中的 JSON/JSONL 保存原始测量或结构化汇总。各报告自带具体基线、方法、fixture、命令及历史失败。工作区原始日志不是远端已发布文件，若后续交付需要独立复现，应随最终备份核对。

- 导航数值与控制范围：[增量记录](performance-incremental-updates.md)、[完整位姿查询](body-recent-query-reuse-20261013.md)、[持续运行JSON](navigation-sustained-worker-20261013-results.json)
- 搜索的正负结果：[priority arms](diagnostics/local-search-priority-results-20261013.json)、[真实guide fallback](diagnostics/recent-guide-fine-escape-results-20261013.json)
- 地图：[dense接纳](performance-dense-admission.md)、[occupancy结果](performance-canonical-occupancy-results.json)、[checkpoint配对结果](performance-checkpoint-listing-results.jsonl)、[原生append](performance-native-dense-append.md)
- 任务/日志：[active-index结果](task-active-index-results-20261013.json)、[日志SQL结果](performance-log-source-queries-results.jsonl)
- 感知/记忆：[分割三进程](diagnostics/yolo-mask-normalization-points-results.json)、[投影三进程](performance-perception-support-indices-results.jsonl)、[有限列表](c1-limited-scene-listing-20261013-results.json)、[summary](memory-summary-readiness-20261013-results.json)、[自然身份](memory-natural-identity-copy-20261013-results.json)
- 网页：[区域资源计数](performance-evidence/semantic-region-reuse-counts.json)、[点云轮询合同](performance-dense-polling.md)
- 版本相关回归：[Python时间线](overnight-python-integration-20261013.md)、[原生离线范围](native-offline-regression-20261013.md)、[身份与剩余legacy路径](content-identity-hash-audit-20261013.md)
