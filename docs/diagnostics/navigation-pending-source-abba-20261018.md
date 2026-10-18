# R11 完整 ABBA：拒绝本轮广泛保留 active 的候选

2026-10-18。结论：触发事先冻结的负控暂停规则，父已拒绝本轮保留 active 全案。语义60/60和几何组合预检成功均保留，但不能抵消实际恢复延迟退化。没有修改冷却、点、预算、输入相位或追加样本。

## 1. 全部四进程与八 case

依据固定四进程原始日志、结果与直接源码字节对照。实际固定顺序A1→B1→B2→A2，每进程exact命中1、exit0、无watchdog；每个case各一次。正式两臂均7文件，5个共同资格文件逐字相同，只有owner/follower两源不同，未把16文件语义包混作B。原日志与失败候选保留。

四次 OnlyActive 都在D到来时实际只有一个已进入compute且未返回的A；四次 InitialSlotObservation 都由原first-dequeue严格区间证明D已安装后才首次取槽，自然覆盖全成立。不是未覆盖或缺失端点导致暂停。

全部case观测无丢失、无issues、无缺端点、原cleanup返回；新增replace-lock计数均0、builder均2。初始pending的A臂只实际执行1个kernel，B臂执行2个；OnlyActive两臂均起2个，但A旧源很快取消、B旧源完整算完。

## 2. 原D安装完成到端点（毫秒）

下表全部保留，不只摘取不利或有利子集。非零是原L层controller的 `linear=(0,0), angular_z=0.1`，全部 `bound_present=false`，不等同dispatch许可、桥交付或实车运动。

| run | case | 最新源实际entry | 首L层合法非零 | control次数 | max control执行 | max due→enter |
|---|---|---:|---:|---:|---:|---:|
| A1 | OnlyActive | 0.385 | 300.477 | 7 | 0.494 | 0.133 |
| B1 | OnlyActive | 250.337 | 600.532 | 13 | 0.546 | 33.310 |
| B2 | OnlyActive | 250.093 | 550.461 | 12 | 0.471 | 0.143 |
| A2 | OnlyActive | 3.013 | 350.727 | 8 | 0.556 | 0.268 |
| A1 | InitialSlot | 50.438 | 450.873 | 9 | 0.772 | 0.260 |
| B1 | InitialSlot | 300.137 | 650.624 | 13 | 0.536 | 4.368 |
| B2 | InitialSlot | 300.133 | 600.636 | 12 | 0.545 | 0.193 |
| A2 | InitialSlot | 50.604 | 400.983 | 8 | 0.885 | 0.109 |

固定pair B−A：

| case | pair | entry增量ms | 首非零增量ms |
|---|---|---:|---:|
| OnlyActive | B1−A1 | +249.952 | +300.055 |
| OnlyActive | B2−A2 | +247.080 | +199.735 |
| InitialSlot | B1−A1 | +249.700 | +199.751 |
| InitialSlot | B2−A2 | +249.529 | +199.654 |

原D采集区间也不重叠：OnlyActive entry的B−A可见区间为[215.388,256.932]ms、[232.360,269.324]ms；首非零为[265.492,307.035]ms、[185.015,221.979]ms。InitialSlot的entry为[217.485,270.315]ms、[222.428,258.372]ms；首非零为[167.536,220.366]ms、[172.553,208.496]ms。均是B.lower−A.upper到B.upper−A.lower，包含各次真实Writer acquisition宽度，没有拿后续较新Empty源年龄替代D总恢复延迟。

四进程最小观察到的正Instant差为24、25、22、24ns；这只是钟诊断，不是硬件精度/统计置信界。上述百毫秒级差在可见采集区间之外，满足既定不利筛查。两样本仍不能推断普遍分布、CPU或尾百分位。

control执行最大值不是两次都退化，不能把它也说成双样本性能失败。B1 OnlyActive有33.31ms调度迟到且跳过1个pose due；B2没有同样尖峰。端点退化在两次均成立，且不依赖把该单次尖峰归因于生产机制。不同窗口长度的control max不是等长稳态吞吐比较。

## 3. 实际事件拆分：不能把损失全归250ms冷却

以下时刻相对于各自初次提交receipt的t0，单位ms。kernel wrapper的return在owner正式发布前；没有把它冒充publication线性化点。

| OnlyActive run | D完成 | 旧A return | 新源entry | 新源return | 首非零 |
|---|---:|---:|---:|---:|---:|
| A1 | 50.109 | 50.485 Failure/cancelled | 50.494 | 337.088 full | 350.586 |
| B1 | 50.098 | 295.718 full | 300.434 | 600.279 full | 650.630 |
| B2 | 50.100 | 277.153 full | 300.193 | 592.108 full | 600.561 |
| A2 | 50.097 | 53.102 Failure/cancelled | 53.110 | 393.135 full | 400.823 |

原策略在D相关Columns到达后cancel：epoch变2、submitted清空、retry恢复到now，50ms的正常tick已接受最新源。worker协作取消旧A后马上进入最新工作，实际D完成→entry仅0.385/3.013ms。

候选在同样D下保留epoch1和submitted1。单票据使新merge入口轻筛退出，未新建P；50–200ms内原冷却亦在，但250ms tick时该截止已过，旧A仍在算，原 `schedule` 的同pose/mode submitted去重仍阻止新源提交。旧A实际在295.718/277.153ms才完整返回；300ms正常step消费并按原proof拒收旧A、清submitted，再正常提交最新source。因此限制并非“只是250ms冷却”：保留受损旧A、同pose去重、单槽无pending可换，以及原50ms take节奏共同形成实际等待。

B1从旧A wrapper return到最新entry约4.716ms，B2约23.040ms，主要等待下一正常300ms tick；不能将这些重叠阶段再次加到总端点上。新源B1在600.279ms返回，略晚于600ms tick结束600.100ms，因此非零要等650ms；B2在592.108ms返回，在600ms tick接纳。这个相位差解释部分首非零差异，不能用hold时间或修改tick频率“修正”观测。

InitialSlot进一步排除了“必须让旧A算完”的假设：A1/A2在D后取消尚未取出的id1，首次原pop是最新id2，分别50.440/50.606ms；B1/B2则在D完成9.248/6.135µs之后，41.971/11.554µs首次取到旧id1。它们先完整算旧源到287.474/272.279ms，仍到300ms正常tick才起最新源。原策略避免了这整个旧源compute，而候选没有。观测本身会影响微小自然窗口，这一限制已在两臂相同探针中披露，未靠人为hold或改相位制造覆盖。

## 4. 保留的成功事实与本轮范围

M1/M2真实可用首次candidate不再白白取消、Q1旧P不再起算、M3真碰撞仍拒收、Writer中间变化不可洗白、owner原子/生命周期和原31边界都仍是实际成功证据。失败在整体接受门：当只有一个受损旧请求时，广泛保留active显著延后最新源恢复。

本轮真实组合full/物理损伤/最新恢复预检运行的是B候选；没有额外跑A独立full组合预检。A在正式replay中的原whole-epoch取消是另一条实际证据，不冒称独立full预检。没有bridge/E层、物理行走、CPU/分配/锁持有时长、churn/高频map数据；未补跑。

八case完整整数摘要随本报告保存于 `navigation-pending-source-abba-20261018.json`；由四份已完成原日志离线整理。摘录不启动项目、不产生新样本、不自动批准生产。

## 5. 处置及独立后续边界

父已拒绝广泛保留active候选，并交唯一执行者恢复A7纯诊断/原生产。已亲读最终final.json / final-source-proof.json / original-contracts-restored.json：B7→A7的完整源匹配成立，原M/Q合同恢复；4个既有改动文件剥离限定cfg诊断新增行后逐字等于原HEAD。最终default7 exit0，7.76s；candidate-only文件已从repo移除并留仓外档案。保留本轮所有外部候选、语义PASS和失败ABBA，不提交被拒绝的生产变更。

后续可独立设计 **Q1-only pending换源**：地图更新的原 `affected` 判定与整epoch取消逐字恢复，只在原策略本就保留epoch、两真实票据且submitted自身有完整非空changes时，step-before-take原子替换实际P。预计仍可解决原global分离Q1；不能再声称解决M1/M2，因为原相关Columns取消恢复后它们仍会丢掉可用首次candidate。

这是新命名策略，需新方案/新协议和实际资格；本轮60PASS与失败ABBA均不覆写、不复用为新策略已通过。当前只写设计，不生成新生产补丁，不调整250ms冷却或几何/预算。
