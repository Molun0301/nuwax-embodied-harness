# 分配式地图工作区与定位会话采用

日期：2026-10-06 UTC。基于已合并的源树
`0046774e1220deec04dbc767dc3a8372a7dd26e3`。

本阶段接通公开 `MapWorkspaceService.activate_map` 对已 finalized 的 schema-2
资产的加载与本地采用。最终新增行为验证 **77/77 通过**：公开流程 32、目录日志 17、
运行态采用 28。没有为分支整理重复功能测试，也没有在本阶段重编译原生 SDK/运行时。

## 实际行为

- 公开入口按真实资产 schema 分派；schema-1 使用原有实现，默认保存仍未改成 schema-2
- 先保存原六字段请求、目标资产/checkpoint 和稳定前驱会话身份，然后才允许副作用
- A→B 由工作区显式停止记录的 A，并核对 session、map、session generation、runtime
  generation、backend kind 的真实 STOPPED 结果；provider 不会自行停止另一拥有方
- 保留原生既有探索/导航安全拒绝，不引入自动取消策略
- 使用已实现的显式 loader，原请求在重试中不换 ID、不补造代次、不转换成 SHA 身份
- 持久化完整原生 ACK 的小型身份/策略证据后，采用不可变 bundle、实际安装快照、
  MappingService 快照、目录选择与真正的 assigned LocalizationSession
- 新定位会话为 LOCALIZING、无 pose、无重定位终态；激活成功不等于定位或导航 READY

## 持久化和恢复

新增完整 assigned journal 保留原 `activation_request`、前驱描述和小型 backend
证据。状态区分 prepared、predecessor_stop_dispatching、predecessor_stopped、
backend_dispatching、backend_applied、reconciling。旧的不完整 schema-3 日志仍可读，
但不会猜测缺失的原 session/runtime 字段以执行。

请求必须在 StopMapping/ActivateMap 前完成持久化。原子 rename 后目录 fsync 失败时，
即使日志已经可见，同阶段显式重试也要先重新完成持久化；故障持续时不发任何变更 RPC。
完整请求不旋转 ID；延迟失败不能把别的调用已推进的阶段退回去。

丢失回复保留未知阶段和交付 fence。磁盘旧回执是历史比较证据，不能单独证明当前
原生拥有方。恢复必须获得固定连接上的真实当前封存状态，或由调用方显式提交原请求
给已限定的 provider 重放机制。启动恢复和 getter 不重新 dispatch Stop/Activate。

真实 GetMappingState 若完整证明同一未决请求已经封存，可以按连接、transition、
原 pending 对象的 CAS 撤销该 provider 的 UNKNOWN 重试记录；缺失、部分或错配状态
不能撤销。这样完成只读对账后，下一次 B 激活不会被残留的 A 未决记录阻挡。

最终目录回执先写入但日志保留，直到本地安装与会话一致；清理失败或取消仍保留、重建
同一请求的 fence。恢复可完成本地采用，不把原生已完成安装执行第二次。

## 所有权和并发

准备、真实 RPC、磁盘/bundle 工作、地图导入、记忆/禁行区操作和回调不在新增的
runtime/session 元数据临界区中执行。沿用已有 executor/catalog 机制做短事务，
已有 runtime/provider 状态锁只保护固定来源与 session/队列提交。

实际 provider 交换、移除、断连的元数据写入与采用 guard 使用同一已有锁关系；
原来的输入/RPC/交付 drain 仍在锁外。普通 Start/Stop 不能绕过正在处理的 assigned
fence；工作区拥有的 exact predecessor helper 执行该事务本身需要的停止步骤。

相同 receipt、policy、availability 的重复采用复用同一安装 token/generation；
真正变化的可用性元数据按原快照 CAS 更新，不能用旧证据覆盖新安装。A→B 同时更新
运行态已有 mapping context，后续操作不会继续选择缓存中的 A。

保存内容的 source epoch 和 source occupancy/cloud receipt 保持原义；安装后的
receipt 是另一份明确标注的证据。MappingService 的内容去重另外比较 assigned
安装身份，避免同一地图的新安装被旧内容去重吞掉，不生成摘要或复制整图作为身份。

发布结束重新读取实际 runtime、mapping、catalog、session。同步回调若切换到 B，
A 调用不会返回历史成功。StateStore 增加使用已有 namespace 锁的条件撤销，只能
删除自己刚写入的旧 workspace 对象，不覆盖后继缓存，也不递归调用扩展 write。
异常与 BaseException 取消同样执行这项保护，并保留原错误。

## 验证范围

采用现有 Python 3.12 环境，分别运行三个新测试文件：

- `tests/test_assigned_workspace_activation.py`：32/32
- `tests/test_assigned_workspace_journal.py`：17/17
- `tests/test_assigned_runtime_adoption.py`：28/28

公开流程测试使用真实工作区、目录、资产/checkpoint 存储、运行态/provider、
MappingService、LocalizationService，配合假原生 protobuf RPC、记忆和导航端点。
覆盖首次采用、显式 A→B、同一拥有方重放、同图新安装、冷启动真实封存状态采用、
丢失回复、持久化失败、取消、CAS/回调竞争、schema-aware getter 与旧 pose 隔离。

新增模块/测试 Ruff、修改文件 PEP 604 检查、分层硬规则、编译和 diff 检查均通过。
逐例记录和对应源文件摘要见 `c1-assigned-workspace-adoption-results.json`。
源码经过独立审查；开发过程中发现并修正了锁内资产读取、陈旧可用性/context、
连接替换竞争、旧映射去重、未知尝试未退役、回调陈旧发布、目录 fsync 与取消清理问题。

## 仍然没有宣称完成的能力

- 不是完整 Python/Rust 全套测试通过，也不是实机或真实 gRPC 网络资格验证
- 默认保存未切换，普通默认 save→restart→load 的完整配对还没有宣布完成
- assigned 重定位公共分派/READY 推进尚未在此提交接通；同 frame 的缓存 pose 或
  ACK 后简单读取 pose 不具备原生拥有方与新鲜度证明，不能让会话伪装 READY
- 冷启动只能根据实际当前原生封存证据采用；原生结果仍未知、代次改变或查询不成功时
  保留恢复要求，不声称无证据的跨重启重执行安全
- 原生探索/运动守卫和实时障碍检查保持原有职责，没有通过本提交放宽
