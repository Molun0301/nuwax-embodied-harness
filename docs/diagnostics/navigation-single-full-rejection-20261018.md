# Single-full 候选：N 独立复核结论

结论：按本轮预声明门拒绝生产候选。71 项语义通过仍为有效证据；不能抵消本次 N 不利结果。父已要求归档后恢复17c1，T/R不跑，不补样、不变门槛。

## 证据与复算

- 原始顺序 A1、B1、B2、A2，4 个独立进程各 exact 1 / exit0，8 个 case 自然覆盖与端点齐全
- 我独立读取四份原始日志，逐条核 receipt 与 entry 的 id/epoch/source 指针/source_ns/pose stamp/mode/generation；重新计算真实 D、map/control bracket、初始 dequeue 与全部六端点
- 初始 P 四格均在整个 D 结束后首次 dequeue 新 id2；旧 id1 从未 compute。OnlyActive 四格均为旧 compute entry < D.began < return。8格都 epoch1→2、清原 ticket/submitted，issues=[]、drops0、cleanup返回
- 独立结果与 validator 的12项冻结 gate JSON 逐项全等：9项触发，其中 OnlyActive 6/6、InitialSlot 3/6

## 固定配对，原整数纳秒

| case / 指标 | B1 − A1 (ns) | B2 − A2 (ns) | 冻结门 |
|---|---:|---:|---|
| OnlyActive / installed_to_entry_ns | 3640421 | 13838 | ADVERSE_STOP |
| OnlyActive / installed_to_nonzero_ns | 249807063 | 100089542 | ADVERSE_STOP |
| OnlyActive / max_control_ns | 983033 | 6705 | ADVERSE_STOP |
| OnlyActive / max_due_delay_ns | 74452934 | 30629915 | ADVERSE_STOP |
| OnlyActive / damage_map_update_ns | 56531 | 33256 | ADVERSE_STOP |
| OnlyActive / max_map_update_ns | 56531 | 33537 | ADVERSE_STOP |
| InitialSlotObservation / installed_to_entry_ns | 278919 | -3333 | STOP_RULE_NOT_TRIGGERED |
| InitialSlotObservation / installed_to_nonzero_ns | 99757795 | 1120445 | ADVERSE_STOP |
| InitialSlotObservation / max_control_ns | -233607 | -27257 | STOP_RULE_NOT_TRIGGERED |
| InitialSlotObservation / max_due_delay_ns | 998663 | 1135997 | ADVERSE_STOP |
| InitialSlotObservation / damage_map_update_ns | -1055 | 1367 | STOP_RULE_NOT_TRIGGERED |
| InitialSlotObservation / max_map_update_ns | 47526 | 1367 | ADVERSE_STOP |

两对 clock reference 为25/26ns，仅作为预声明诊断停收线，不是噪声界、置信区间或零回归证明。B1 OnlyActive 的一次 control due-delay 为74,701,593ns，超过50ms监测线，跳过1个control网格；B2 OnlyActive跳过1个pose网格。没有把它们删作离群点。

## 可观测分解，不作因果归因

OnlyActive 的首 L 不利可用原有 wall 时间分成三个相加的区间，单位ns：

| 固定对 | D结束→新entry差 | 新entry→kernel return差 | kernel return→首L差 | 总首L差 |
|---|---:|---:|---:|---:|
| SFN-B1−SFN-A1 | 3640421 | 216655251 | 29511391 | 249807063 |
| SFN-B2−SFN-A2 | 13838 | 84221803 | 15853901 | 100089542 |

这只是同一组现成时间戳的分解。kernel return 仍早于 owner publish，因此末段包含未知的发布/控制等待/原接纳路径，不等于纯调度等待或consume成本。entry→return也是wall，不是CPU。没有独立隔离锁等待、环境调度、cache或编译布局的因果证据。不能说D-map额外几十微秒必然造成首L额外100–250ms。

两个B的OnlyActive恢复compute wall也较长，是后续解释应考虑的原始事实，不能因此把本次生产候选重判为通过。所有首L都是linear=0、angular_z=0.1且bound=None：本次没有前进制动期限覆盖、真实scheduler TTL、bridge/hardware或物理闭环结论。

## 历史与接线说明

首个A check的E0425和exec transport中断单独保留；未形成计时样本。父批准的共同helper别名修订后，仅既定四样本，无追加或补覆盖。两生产差异与stage42基线未变化。原未修订草案、实际修订两臂源和恢复材料均留档。

更合适的后续逻辑方向：保留当前原取消边界，先用既有entry/return/control时序定位恢复wall时间的分布和未知区间，围绕已有规划/guard路径的真实成本找独立证据。此建议不提出新的保槽实现，不扩大active/P/partial，不授权新测试，也不改变本次拒绝结论。

原整数与逐case证据见 [本轮记录](navigation-single-full-rejection-20261018.json)。唯一执行者已完成语义B恢复/default7（4s），随后恢复17c1基线：4既有源与原11核心方法逐字一致，2新测试已归档撤出，工作树干净。未增加永久诊断接线、未运行T/R。
