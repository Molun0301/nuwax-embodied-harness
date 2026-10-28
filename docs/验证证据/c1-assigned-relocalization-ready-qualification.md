# 分配式重定位、真实位姿与坐标目标准入

日期：2026-10-06 UTC。基线为唯一工作分支
`fix/v3-legacy-cloud-seal-20261006` 的远端提交
`8ef3c92a6a60bb4685897ee0c35e0a2a08ad88f6`，源树
`5047fe546a151a8344c368fca38e7ccaaeef83e4`；对应本地同树检查点
`ffda055c71d92108ce5c930257a08ce640701a11`，发布时不携带本地合成恢复历史。

本阶段在支持的 voxel/raytracing 默认 correction owner 上接通真实 assigned
Relocalize 链。它推进了上一保存阶段的 LOCALIZING 边界，不把所有配置或硬件描述为已就绪。

## 可执行链与证据

- 复用已有 assigned target、source accumulation、ICP/全局匹配和 correction finalizer
- live schema-2 RPC 先做现有强类型请求准入；目标 availability 来自真实 actor admission
  与完整封存目标，PGO/无对应 actor 配置不会只因存在资产就得到 true
- 保留 legacy target getter 对 assigned 返回 None，不制造旧摘要或复制另一份全图目标
- 新 `GetAssignedLocalizationSample` 接收原完整重定位请求，查询其实际已应用终态
- 在既有 lifecycle/shared-state 顺序下，核对当前封存激活、canonical correction、
  candidate sequence、session/frame/registration，以及真实 pose 与 map→odom→body TF
- pose/两条 TF 必须同一时间且晚于或等于终态冻结 odometry；之后真实移动产生的新位姿可读
- Python 固定 provider/连接/交付代次/运行态安装，拒绝旧同 frame pose、其它 correction
  或当前拥有方；不通过通用 GetCurrentPose 补齐证明
- 原请求先持久化再 dispatch；真实终态持久化后才读取 sample，并沿原 runtime/session CAS
  采用。原 session token 在 dispatch/sample 之前保留，回调不能偷换成较新的 token
- 只有真实 sample、已应用终态和一致 session/workspace 才发布 READY；get_relocalized_pose
  使用同一原生 sample 路径，坐标导航无需掉回旧摘要重定位

原 canonical applier 清除 `checkpoint_localization_required` 后，原有坐标目标准入
继续使用其导航、motion-admission 和实时障碍规则。本阶段没有放宽这些检查，也没有
替换原生匹配接受策略、发明算法分数阈值或增加全局串行门。

## 恢复与失败

未知 dispatch 只查询原完整请求；pending/not_found 不授权重新执行。已完成的相同请求
保持原 ID/timeout/policy，改变 timeout 或为 cloud 请求提供 planar seed 在 RPC 前拒绝。
重启可以只读采用当前原生封存激活，再核对它与原请求完全一致，读取终态及真实位姿。
已退役的原请求只在精确终态查询和不同的当前持久封存 replacement 证明下收敛，不安装旧 pose。

相同终态字节在前次 rename/fsync 失败后重新完成持久化。清除日志时，rename 本身及
后续 fsync 取消均受保护；仅恢复精确原 tombstone，no-replace hardlink 不覆盖后继拥有方。
发布、RPC、磁盘/资产工作都不在运行态/provider/session 元数据锁内进行。

## 实际验证

**Python 74/74 通过**：新 public workspace 25、新 protobuf/sample/session 18，以及直接
受影响的目录/工作区回归 31。最终两份运行分别为 43/43（6.600 秒）、31/31（2.074 秒）。
这些 Python 测试使用真实服务/存储与假原生 RPC，不冒充真实网络测试。

**原生 5/5 精确 selector 通过**，其中两个新测试分别对 voxel/raytracing 贯通：

真实生成 tonic client/server loopback → 原生 Relocalize actor → CommonLioIngress 扫描准入 →
真实 GeometryCorrectionSink/canonical applier → 原生 sample RPC → 既有 SetNavigationGoal。

新测试覆盖 apply 前关闭准入、回复丢失后的精确查询/重放不二次应用、后续移动位姿、
pose/TF 破坏、第二次 correction 淘汰前一次终态、地图继续推进、typed motion proof、
无直接 Twist 输出和目标退役。另三个回归是 voxel/ray 导出传输安装及 postclaim/回复丢失。

SDK 不重建。断网 CMake 预检的两个消费者都使用已安装 GTSAM/Metis/Cephes，无 `_deps`
回退。Cargo locked/offline、jobs=2，CMake jobs=1。首次 core 编译 6m15s；补齐最终 fixture
后发现一处测试 write-guard 解引用类型错误，窄修后最终编译 2m08s 通过。失败日志保留。

新增文件 Ruff、修改文件 critical Ruff/编译、PEP604、分层硬规则、确定性 protobuf
生成检查和 diff 检查均通过。源码经独立审查。精确 case、日志、源码与二进制摘要见
`c1-assigned-relocalization-ready-results.json`。

## 未宣称完成的部分

- 原生 fixture 使用注入的 LCM transport/readiness，并不验证实机、完整导航栈执行或避障效果
- 没有运行完整 Python/Rust 套件；实际坐标目标证明到准入，不等于机器人已完成行驶
- PGO 配置没有默认显式重定位 owner 时保持 unavailable；planar assigned 激活仍不在此阶段
- mapping-cloud/place/georeference authority、探索暂停/楼层合同以及保存 dense-cloud 显示
  仍有旧摘要消费者，后续分别配对，不用摘要别名兼容
- 通用 assigned-session 组件旧 API 兼容保留；新公开重定位/导航路径只使用上述原生 sample
