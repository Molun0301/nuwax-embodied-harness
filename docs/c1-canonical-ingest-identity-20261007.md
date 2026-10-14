# Canonical LIO 准入身份消除重复点内容 fingerprint

2026-10-07 UTC。保留这个范围限定的生产改动：canonical LIO 把已有backend/runtime-generation/source-sequence一直传到MappingSession，替代该路径每帧扫描XYZ位模式的内容fingerprint。**普通raw/通用旧API保留其内容去重合同，不能称为全项目“hash清零”。** 47项不同聚焦检查通过；100,000点固定输入的7对prepare测量均更快。实时移动、完整传输、执行器和release资格未由此解决。

基点为真实远端`baa93057e005132b6326fdb6338a3ada68289bb0`，tree`c9d7bea863ff94258901dbdaba699412c4963218`，同树本地`914014667eea78f3bfdc1169b47d1e282ede7209`。新提交必须接真实远端父提交，不携带本地合成祖先。

## 已证明的工作与最小修改

原生产路径由`AcceptedRegisteredCloud`的不可变登记拥有方发起。CommonLioIngress已经验证完整wire字节、backend/generation、严格递增sequence/time和registration binding，消费一次性许可；进入mapping时却只留下时间/会话标量，再在持有mapping mutex与生命周期守卫期间计算所有点XYZ的DefaultHasher fingerprint。

- `AcceptedRegisteredCloud::source_identity()`只读取原私有envelope标量，不调用`scan()`、变换body点或复制点云
- live与startup/deferred replay两个真正的canonical handoff都调用同一个`canonical_mapping_source`，带原backend/runtime-generation/source-sequence；没有生成接收计数器、新epoch、内容摘要或另一个地图身份
- `MappingCloudSource::{Legacy,SourceFencedCompatibility,Canonical(key)}`替代原布尔mode。裸slice兼容helper明确选compatibility；接受`AcceptedRegisteredCloud`的`_with_correction_for_test`仍走canonical
- 所有模式沿原owned task→commit→prepare链转发。原session/frame/correction/relocalization检查、backend校验及coalescing都在keyed feed之前；锁范围、点云拥有方、提交和发布路径未重构
- MappingSession保留原HashSet+FIFO和容量，用tag区分legacy内容键与admitted producer键；共用同一有界历史，没有第二套无限历史。普通容器仍会hash固定大小键；消除的是点内容扫描
- generic `feed_pointcloud`通过栈上、静态分发的lazy key helper沿原顺序处理freshness→内容fingerprint→dedup→容量→append/remember。新的`feed_admitted_pointcloud`不调用points_fingerprint
- `AdmittedFrameKey`三个字段私有，构造器只校验正数。它是已准入的可信元数据，**不是认证或不可伪造能力**；真正字节绑定仍由现有canonical上游负责

## 明确重放与兼容合同

1. 仍在历史中且未被freshness拒绝的相同key不append、不淘汰点、不增加session generation、backend token、source-frame sequence或revision。重复receipt时间不会创造新采集
2. 不同producer key表示不同采集，即使几何完全相同；session级测试允许相同时间，不代表放宽上游canonical严格时间递增规则
3. 同key换正文是可信调用者违约，session不重新扫描来识别它；上游真实wire/binding变更拒绝测试仍通过。不能声称HashSet验证了正文
4. freshness在dedup之前。source-fenced路径仍用冻结adapter receipt时钟，输出保持真实source timestamp，包括零；旧重放可能先返回StaleRevision
5. coalesced帧尚未进入feed，不能提前记key，之后可重试。已经feed接受的帧在backend失败后仍被记住；“未发布”不是重复append授权
6. FIFO淘汰、history关闭或reset后允许重新处理，既有上游one-shot边界继续负责更强的世代/序列拒绝。这不是永久或跨进程幂等保证
7. raw/通用API没有可信producer epoch，仍允许同source-time而正文/位姿/point-step不同的帧，并抑制精确重放。其原fingerprint保留到未来完成真实producer身份配对；不伪造epoch或改成接收计数器

## 功能资格

最终冻结候选`candidate-r3`实际执行13组筛选/精确命令：

- mapping_session：33通过、1个手动fingerprint基准默认忽略，其中16个新增admitted合同用例
- 新adapter合同：3通过，覆盖零source时间、接受后backend失败重放不变、coalesced key可重试、不同序列同几何、无效frame不记key和stale优先级
- 4个既有adapter检查：source freshness、source-fenced零时间、同时间新正文继续发布、backend coalescing/重复完成
- 6个真实LIO准入检查：backend/generation/seq/time、重启旧envelope、改wire/body/metadata不消费许可、零时间一次性许可、配置frame、被拒登记不改adapter/permit
- 1个已有lazy capture检查加入source_identity精确标量断言，确认仅取key不触发body capture

合计**47个不同方法通过、0失败、1忽略**，不是整个Geometry或workspace套件。新mapping单元测试的cfg(test)线程局部计数器实际检查canonical入口零次point fingerprint，legacy新鲜尝试一次，stale尝试零次；计数器不进入作为Geometry依赖的生产mapping代码。

这些是直接模块/线程和真实canonical准入逻辑检查，不启动真实LCM→MotionGuard组合。原生live/replay两个handoff的实际source转发和未变化的外围fence由源码审查与编译核对；不能扩大为真实transport、完整启动恢复或实机资格。

## 基线、构建及中断 provenance

首先只加入两个cfg(test)基准，再编译/运行原生产路径，随后才编辑生产。baseline Geometry构建1m00s通过；mapping的offline首轮因缓存缺少`anes0.1.6`退出101。用同一locked命令允许官方registry补齐所需依赖后51.90s通过，Cargo.lock不变，registry增加7,405,832 bytes。没有重建SDK或更新锁文件。

首次候选编译发现两个旧owned helper测试仍传bool而非新enum，退出101；仅机械改为Legacy。01:16开始的replacement build在任务中断后没有exit/end/test artifact，session已不可续接，标为**结果未知，不能算通过**；未声称中断期间持续工作或知道中断原因。01:45恢复后核对source未变，使用新目录candidate-r3重建：Geometry2m14s、mapping5.38s，均exit0，九项source/lock核对通过。原失败和中断文件完整保留。

Rust/Cargo1.88，Cargo jobs2/CMake jobs1。固定混合test profile：Geometry/mapping为opt0，MLS/runtime opt3，debug0、assertions与overflow checks开启；Geometry features=default,real-lcm-bridge。**不是release**。独立审查核对baseline→candidate全部352个Geometry compiler-artifact的profile/features一致，不只是Cargo摘要一致。

| 冻结二进制 | SHA256 |
| --- | --- |
| baseline Geometry | `83027c0740ba7fac21c8455e961bad642441dd5dfb5fd8d862a134fa72edef28` |
| candidate Geometry | `e9a1f72a5049c841f7ba63c14582d688a34e76e614309d64d1b1d54f1b86228e` |
| baseline mapping tests | `1dfe9504d7a6339f336b4daa8c79c3d17e6fb211240b29b59bedef00c7ab4bf5` |
| candidate mapping tests | `d1e7e2c16d3f2fda0ecd5e52f9a8da77650b4e86402bff038b4dbcb6edaa8e20` |

## 相同100,000点的匹配prepare测量

固定合成map-frame点，原sample配置、相同pose/frame/source/receipt时钟、rolling容量100,000。每进程4次warmup后16个新帧；7对新进程按AB/BA交替。每次都取得真正prepared token，检查完整payload、source时间、序列、窗口/generation/revision/coalesced计数，再在计时外用原fail_backend_compute退役准确token，避免首帧后只测coalescing。

测量分离uncontended mutex等待与取得锁后prepare墙钟，后者结束于unlock之前。setup、断言、返回值析构和token退休都在计时外；没有backend submit/compute/commit/publication。candidate参数求值还在锁内构造小型key，实际production handoff在锁前构造；这是保留的少量保守开销。fixture惟一预期参数变化是source-fenced兼容mode→原producer标量构造的canonical key。

首次独立baseline：prepare16样本算术均值15.680348ms；单独进程的legacy fingerprint均值10.199548ms。fingerprint不是从prepare中减去的校正值，也不是按比例推导最终结果。最终对照不计入这两次初始运行。

| 正式prepare统计 | baseline | candidate |
| --- | ---: | ---: |
| 7个批次均值的中位数 ms | 15.606905 | 4.935277 |
| 112个单次样本median ms | 15.043243 | 4.784174 |
| 单次P95（nearest-rank）ms | 24.620829 | 6.962341 |
| 单次max ms | 43.604922 | 12.913210 |
| 单次min ms | 13.136037 | 4.151979 |

candidate赢7/7；配对比中位数0.3212551932，对应**67.8745%**下降。完整七个比值：0.3120421527、0.3330171699、0.2793477793、0.3540123868、0.3212551932、0.3251308811、0.2466349092。组中位数比与配对比中位数不是同一统计量。

uncontended lock-wait单次median baseline/candidate为0.000121/0.000114ms，P95为0.000259/0.000180ms，max为0.001322/0.001966ms；candidate最大等待反而更大，不能宣称改善锁竞争。整子进程CPU中位数0.788627→0.567943s，14次launch-envelope peak RSS均134784KiB；包含fixture/checks/worker生命周期，不作为阶段CPU或内存节省测量。

## 结论和下一边界

源码确认消除canonical临界段中的整点内容hash；实际相同输入prepare成本显著降低，行为检查通过，独立审查建议保留。其余必要点校验、复制、wire精确比较和backend计算仍存在，没有无锁/零复制/全链路性能结论。

组合源码移动基线见`c1-combined-moving-20261007.md`：OFF通过、ON/held节拍失败、held有65.927ms控制慢样本。该fixture不走本次canonical ingress，所以没有为了此改动重跑，也不把本结果称为解决其失败。原75ms/50ms、TTL、实时障碍和所有权规则保持。

证据目录`/workspace/shared/neh-canonical-ingest-20261007`保存baseline、candidate失败、candidate-r2中断、candidate-r3成功、47项日志、14份正式对照日志和精确runner；配套results JSON保存全部ns样本、命令、资源和身份。后续generic/raw producer配对、实际生产LCM→MotionGuard、代表性release/hardware及其余平台优化仍未完成。
