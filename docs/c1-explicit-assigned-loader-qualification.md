# 显式 schema-2 后端加载：有界验证记录

日期：2026-10-06 UTC。

本阶段增加可执行的 Python provider/runtime 显式加载入口，继续使用现有原生
ActivateMap 的 activation_contract_version=2 分支。Python 回归已通过；同一提交
包含经过源码审查的原生集成测试，**原生测试尚未编译、尚未执行**，等待 SDK 恢复。
这不是完整用户地图切换、默认保存/加载配对完成、实机验证或发布就绪声明。

## 本阶段能力与边界

- Runtime 入口：PlatformGeometryRuntime.activate_assigned_backend_checkpoint
- Provider 入口：GeometryRuntimeProvider.activate_assigned_platform_map
- 调用方必须保留原来的六字段请求：activation_request_id、session_id、map_name、
  asset_identity、checkpoint_id、expected_runtime_generation
- Runtime 只按指定 backend_kind 和 checkpoint_id 重开 schema-2 checkpoint，加载
  原有不可变成员与制品身份。支持 voxel/raytracing，不转译为 legacy SHA 身份
- 普通加载只接受 IDLE/STOPPED 后端或同会话原请求重放。另一活跃拥有方必须先经
  调用方显式 StopMapping；加载器不会暗中停止它。完整 A→B 工作区生命周期仍待配对
- 不切换默认保存/加载入口，不安装 Python 工作区快照，不设置前驱 pose，不开放
  assigned 重定位可用性。返回值保留原生实际可用性，false 不会被提升为 true

## 所有权、失败与重试

一次操作固定 stub 和连接代次，状态读、制品上传、ActivateMap 与释放均绑定该来源。
上传只重放保留的 ID、offset 和原字节，不生成摘要或替代 ID。完整 ACK 必须匹配
原六字段、原 cloud 成员、安装回执、revision 和实际 policy，才能采用本地映射身份。
有效 ACK 后本地采用失败会保留 applied 事实；释放失败不覆盖成功或原始错误。

ActivateMap 已 dispatch 而没有可信 ACK 时返回 UNKNOWN；不会查询状态来猜测成功，
也不会自动重执行。仅保留一个未决尝试的不可变 checkpoint 引用、六字段不可变键及
原连接证据，不复制整图、不保留第二份 protobuf body。

原生可能在 publication 失败后保留生命周期锁，使 GetMappingState 无法完成。因此，
调用方以后显式再次提交同一请求时，只有同一 provider/连接内保留的未决 dispatch
证据，且所有 checkpoint 字节与几何描述符精确相同，才允许略过该状态读，由原生
账本裁定原请求。浮点描述符按实际 f32/f64 位比较，包含正负零。改变任一字段或 body
会在 RPC 前拒绝。新 provider、断连或重启不能获得这项豁免；本阶段不声明跨重启恢复。
断连撤销本地重试资格，原 UNKNOWN 仍然是 UNKNOWN。

映射 transition 预约持续到制品释放结束，避免后继上传与前次 RELEASE 竞争；无需
在 RPC/字节比较期间持有状态锁。丢弃最后一个保留载荷引用也发生在两个状态锁之外。

## 已完成的 Python 验证

使用现有 Python 3.12 测试环境，unittest discover -s tests。

- 初轮全量：298/298 通过
- 补充 signed-zero 位级身份回归后的最终全量：299/299 通过
- 新 provider loader：28 个测试；新 runtime entry：14 个测试
- 新测试文件 Ruff、Python 编译、PEP 604 兼容检查、分层硬规则、ROS2 residual
  静态 gate 及 git diff --check 均通过
- 回归套件的 publication callback failure 日志为既有故障注入用例预期输出

覆盖：原六字段及实际 protobuf/body，保留制品上传/释放，封存重放只安装一次，另一
活跃拥有方拒绝且无 StopMapping/上传，显式停止后的加载，未决状态读锁下的原请求
恢复，新 provider/缺失 native record，所有六字段及 body/描述符篡改，异常公开字典
不能伪造内部证据，正负零，响应丢失 UNKNOWN，ACK 后本地失败，断连代次，释放失败
不覆盖结果，中断清理，上传/释放重叠 barrier，锁外释放载荷，默认入口保持关闭。

这些是 Python 组件回归，包含真实 protobuf 和 checkpoint 存储、假 RPC；不等于
真实 gRPC 服务、原生运行时、硬件或整个平台端到端通过。

## 待执行的原生测试

同一源码检查点新增以下精确选择器：

- server::lifecycle_tests::assigned_v2_voxel_export_transfer_activate_replay_and_conflicts
- server::lifecycle_tests::assigned_v2_ray_export_transfer_activate_replay_and_conflicts

测试通过现有 injected transport 与真实 backend worker，调用实际 schema-2 导出、
AR2/DC1 分块传输/封存、显式前驱停止和 version-2 ActivateMap；检查原始成员回执、
释放临时制品后的重放、owner/body 冲突拒绝、不重复安装/发布、前驱 pose/TF 清空及
新会话 pose 保留。assigned 重定位可用性仍应为 false。

**当前仅源码审查通过。尚无原生编译或执行结果。** SDK 就绪后需在指定独占构建 lane
编译，核对二进制测试清单，再逐个 exact 运行并记录结果；可能仍需修正测试或源码。
继续保持默认与工作区门控，直到完整调用方配对与原生证据到位。
