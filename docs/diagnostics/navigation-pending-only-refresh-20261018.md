# 阶段42：保留原相关取消，仅刷新真正 pending 的旧源

## 接受结论及范围

基线9066616198e5724f41db810f148930895fdd0840。本次最小 pending-only 生产改动已通过预先约定的接受门：新语义/组合预检66个 distinct selector跨批均有PASS；独立N负控8case全覆盖且未触发既定不利门；N过门后另行批准的独立Q四次均自然覆盖，两固定配对均观察到旧P不再起算、最新源entry及首L非零更早。本轮于2026-10-18接受这一最小范围。最终合并qualification tests check通过（10s）、stack no-run通过（211s）、默认七包tests check通过（9s）；3个新诊断选择器实际各1、旧Q1为0。合并4驱动与正式A/B源逐字一致，生产与已测语义不变，没有新增N/Q样本或重复运行语义。 本报告随实现提交，交付状态以对应Git记录为准。

结论严格限于本次固定输入和两组目标配对，不是普遍提速、统计等价、CPU节省或全部实时期限通过。阶段41被否决的广泛保留active方案仍不接受，其旧60项语义和失败ABBA完整保留，未计入本阶段通过。原M1/M2相关变化仍取消，因此那两项可用候选机会尚未解决。

## 生产范围

整个update_execution_map逐字不动，包括global、retained local_path、Unknown/链断裂以及原whole-epoch取消。只有原策略本来就保留epoch、已有两真实source tickets、submitted对应票据有完整非空changes时，原step经过当前body检查与advance后、consume/take前尝试一次replace-only。

原exchange锁内检查expected P id、pending epoch和current epoch；真正匹配才调用窄factory，已起算/无pending/错epoch直接miss，无普通submit回退、无新建槽。沿原epoch使用实际当前map Arc、最新合法pose及已推进progress；P原detour mode与原250ms retry deadline完整保留。只退休旧P票据，新P从实际请求源建票，A累积的真实变化链不洗白；ticket.basis已observe到最新不等于request.source已换源。旧payload在锁外析构。无新生产字段、锁、队列、hash或几何校验；原consume、rejection、cancel及Stop行为保留。

## 新语义证据及首失败

新owner10、follower16、stack4加独立B组合几何预检1，另跑原boundary31、原related-cancel1、原M1/M2/M3三项，共66 distinct。首批65PASS/1前置FAIL保存；只修正tests-only local控制合同并单例补验1PASS，总67次执行、66PASS/1保留FAIL。不是首批66无失败，也不借用旧60。

local首失败发生在S2之前：初始真实full448路径已admit；固定pose12.55→13.55、yaw0触发原0.9m偏离guard，当前admission已None。修订仍保留原pose、固定点、阈值、取消断言，新增精确原wait_reason“当前轨迹受阻，保持零速等待局部接续：机体偏离连续三维路线”和448个pose逐bits/frame/stamp同一retained path断言。实际补验通过：S2 global分离但命中retained local_path，原epoch提升、票据清空、retry-now，并取得原ordinal2 explicit=true/cancelled=true回执。此例证明撤权后保留path仍参与原map取消，未证明持续已admitted运动。

新Q1/真实Writer删除与删除后新增、mode保留、原take竞态/两槽/Unknown/Stop、原M1/M2/M3均按各自精确selector验证。旧反向Q1测试函数显式移除，共用helper保留；旧名在两计时臂actual-list均为0，历史原源码/commit/日志仍在。新Q1语义与无holdQ分别取证。

## 新N负控：四进程、八case

固定N-A1→N-B1→N-B2→N-A2，每进程OnlyActive与InitialSlot各一次。原真实Writer M1+M3、原固定几何/预算、pose25ms/control50ms、首次Empty200ms。两臂都保留原related取消。OnlyActive四次满足实际A entry < D begin < cancelled A return；InitialSlot四次满足D.finished < 原首次dequeue-before，首次实际取出epoch2的id2，确认旧id1确为初始pending。8case均epoch1→2、清旧票据/submitted，新helper lock尝试0，正常请求builder2。

单位ms。D为真实damage安装返回，L为原控制层首合法非零。

| 样本 | Case | D→最新entry | D→首L非零 | max control | max due→enter |
|---|---|---:|---:|---:|---:|
| N-A1 | OnlyActive | 4.624549 | 350.205419 | 0.456122 | 0.364618 |
| N-A1 | InitialSlotObservation | 52.332064 | 600.619605 | 0.672982 | 4.211029 |
| N-B1 | OnlyActive | 0.310146 | 350.490431 | 0.494468 | 0.124813 |
| N-B1 | InitialSlotObservation | 50.581536 | 350.663335 | 0.521028 | 0.152429 |
| N-B2 | OnlyActive | 3.711229 | 350.561625 | 0.593825 | 0.119851 |
| N-B2 | InitialSlotObservation | 50.462143 | 400.835852 | 0.737419 | 0.122904 |
| N-A2 | OnlyActive | 0.263619 | 350.664376 | 0.683075 | 0.119032 |
| N-A2 | InitialSlotObservation | 50.592912 | 400.571137 | 0.491251 | 0.158954 |

固定配对B−A，单位ms：

| 配对 | Case | D→最新entry | D→首L非零 | max control | max due→enter |
|---|---|---:|---:|---:|---:|
| N-B1−N-A1 | OnlyActive | -4.314403 | 0.285012 | 0.038346 | -0.239805 |
| N-B1−N-A1 | InitialSlotObservation | -1.750528 | -249.956270 | -0.151954 | -4.058600 |
| N-B2−N-A2 | OnlyActive | 3.447610 | -0.102751 | -0.089250 | 0.000819 |
| N-B2−N-A2 | InitialSlotObservation | -0.130769 | 0.264715 | 0.246168 | -0.036050 |

没有一个冻结核心指标在两固定配对中均B更慢且超过原观察分辨率（配对24ns），因此未触发N停止接受门。所有逆向值保留；不能把混向结果称等价。InitialSlot首配对首L差约250ms也不能归给本优化，这些负控本身均未执行pending替换。N没有丢观察、漏pose/control、窗外entry/return或缺端点。

## 新Q目标：四次自然真实pending

固定Q-A1→Q-B1→Q-B2→Q-A2，各1case；全部完成后统一判读，无补样、无改相位。真实S0→S1 Empty于250ms安装，再合法pose13.55和原step正常提交P；固定S2 Columns于275ms安装；目标step仍300ms。为防预建D经200ms续观测提前发布，Q的首个S2后Empty固定400ms，后600/800ms；N仍保留200ms。这一差异在运行前单独冻结。所有raw source stamp、acquisition/build/install区间原样保留。

S2真实update返回、原D.finished端点记录后，两臂同样只读一次原exchange锁，检查250ms唯一receipt的id2/epoch1。四次都实际queued=(active=true,completion=false)，锁外探针区间分别72/159/235/313ns；这不是锁持有时长。300ms之前无中间take或第二consumer；原worker完成槽背压保留P。四个300ms step全部在原P约501–502ms deadline内完成，retry/mode不变。

A在300ms accepted增量0，之后实际id2从S1起算，再由id3从已安装的S2同几何Empty后继起算；B该step增量恰1、旧P票据退休、id3真实输入S2，后续kernel仅id1/id3。每个实际entry均与receipt的source Arc、source_ns、pose stamp、mode、generation及epoch唯一对应。四次target_uncovered=[]，无歧义多接受或资格补偿。只读成功receipt状态不是直接记录retired返回/线性化；精确原子退休由独立新语义测试证明。

单位ms；漏拍列为pose/control计数。

| 样本 | kernel request ids | D→最新entry | D→首L非零 | max control | max due→enter | 漏拍 |
|---|---|---:|---:|---:|---:|---:|
| Q-A1 | [1, 2, 3] | 325.080644 | 625.514319 | 0.516316 | 1.492579 | 0/0 |
| Q-B1 | [1, 3] | 75.044142 | 425.943501 | 0.794536 | 1.754616 | 0/0 |
| Q-B2 | [1, 3] | 75.058206 | 476.218850 | 1.144435 | 1.725280 | 0/0 |
| Q-A2 | [1, 2, 3] | 475.035420 | 1102.532799 | 171.990100 | 55.662638 | 2/4 |

固定配对B−A，单位ms：

| 配对 | D→最新entry | D→首L非零 | max control | max due→enter |
|---|---:|---:|---:|---:|
| Q-B1−Q-A1 | -250.036502 | -199.570818 | 0.278220 | 0.262037 |
| Q-B2−Q-A2 | -399.977214 | -626.313949 | -170.845665 | -53.937358 |

每臂均构造3份请求；A实际执行3份、B执行2份，所以只能直接说省去一份旧P真实kernel entry，不能称少构造请求或已测CPU下降。B最新entry使用S2本身，A用后来真实Empty后继；另分别保留最初D采集至恢复的等待，不能把A较新source的年龄替代旧D总等待。按真实D acquisition区间，两个entry差区间分别[-303.385090,-244.400502]ms、[-418.348346,-382.770586]ms，首L差区间分别[-252.919406,-193.934818]ms、[-644.685081,-609.107321]ms。区间不是置信区间，不解决运行间因果混杂。

### 不利观测必须同时保留

Q-A2最后一个control耗时171.990100ms（超过原50ms节拍），due→enter55.662638ms；漏2个pose due、4个control due。原因没有测得，不能命名为已证实的调度噪声，也不能归因于生产策略。该样本原样保留，既不删除也不复跑；其早期300ms pending资格完整成立，但其首L的巨大差不能全算候选收益。Q-B1相对A1的max control高0.278220ms、max due→enter高0.262037ms，也未被有利entry/L结果抵消或隐去。

本次可接受的直接结果是：原相关取消仍在、自然目标的旧P确实省去，两固定配对实测entry/L更早。两配对不足以证明完整时差均来自优化、总体分布改善或实时期限全面合格。

## 层级、未观测端点与收尾

所有N/Q首L命令均linear=(0,0)、angular_z=0.1，dispatch bound缺省。实际max_observation_age=None，scheduler pose/costmap/path-wait TTL均NOT_EXERCISED。因此“没有观测到expired-nonzero标志”不等于E层期限验证；不称桥交付、机器人移动或物理闭环。

source channel enqueue/recv、owner publish/consume/reject、Q的P dequeue、replace线性化/retired返回均UNOBSERVED；kernel RETURN在owner publish之前，不能互相替代。CPU、分配、锁持有时间未测。原Stop/Drop返回并排空观察，不代表单独暴露了内部owner join Result。

采用同一原release opt3/LTO target、locked/offline -j4、相同资格features和共同观察，每次切臂后沿原路径立即运行，无A/B二进制复制/并存缓存、第二target或hash。两臂计时九源只有owner/follower两生产文件差异，其余七源相同。最终Q后已精确恢复语义九源、原lib并撤下四计时文件，默认七包tests cargo check exit0；这不是七项测试运行。诊断永久合并后的最终检查及提交状态另记。

完整12case整数、原始事件、固定配对、source age、样本身份/manifest、保留首失败索引及逐阶段结果见同名JSON。仓外各阶段完整raw日志、源集、patch、actual-list、build/check结果独立归档；阶段41失败证据没有覆盖。旧阶段final.json中的“后续批未运行”是当时快照，后续批准和执行另存，未改写历史。
