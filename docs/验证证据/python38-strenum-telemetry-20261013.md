# Python 3.8 字符串枚举与遥测日志验证（2026-10-13）

## 结论

修复 Python 3.8 的 `StrEnum` 回退实现：`__str__ = str.__str__`。这恢复了调用方依赖的字符串值语义，与 Python 3.11+ 原生 `StrEnum` 一致。没有修改遥测过滤、队列、计数器、事件回放或日志输出拥有者。

INFO 级别下普通定位遥测仍会先捕获独立的有界日志副本，随后在输出线程检查 DEBUG 是否启用。这确有开销，但本次真实小载荷基准不支持为优化而改变计数、缺口及动态日志级别语义。

## 发现的实际错误

Python 3.8.10 上旧实现 `str(RuntimeEventSeverity.WARNING)` 返回 `RuntimeEventSeverity.WARNING`，ERROR 同理，类别也返回枚举名称而非字符串值。日志与反馈代码使用 `str(severity)` 比较 `warning` / `error`，导致：

- 定位 warning/error 不被识别为反馈事件
- 定位 warning/error 被当作普通低价值遥测，在 INFO 日志级别下没有对应告警输出
- 队列只有预留容量时，它们被错误计入 `telemetry_dropped`，而不是使用受保护容量

独立故障复现使用实际 `EventBus → GatewayRuntime → RuntimeEventLog`：`max_items=1, reserved_items=1`。旧版 WARNING 和 ERROR 均为反馈 0、日志 0、`telemetry_dropped=1`、`protected_dropped=0`，原事件仍在回放中。这不是以模拟 logger 判断替代实际通路。

修复后的回归先发布普通定位遥测（应因仅剩预留容量而拒绝），再分别发布 WARNING/ERROR。两者均进入预留容量，产生正确级别的日志与反馈；计数为 accepted=delivered=1、telemetry_dropped=1、protected_dropped=0、writer_failed=0，缺口仅包含前一条普通遥测，回放保留两条事件。

## 变更范围

- `contracts/compat.py`：仅回退类增加字符串转换；Python 3.11+ 原生路径不变
- `scripts/check_str_enum_compat.py`：实际运行时及强制回退路径，验证 str、repr、格式化、JSON、相等性、hash、成员迭代/查找；Pydantic 往返及实际 warning/error 反馈与保护容量验证
- `scripts/check_runtime_event_log_isolation.py`：只在状态读取测试中补充其缺少的 `mapping_service.notification_status` 桩；生产代码没有添加兼容性 guard

## 性能测量

2026-10-13 18:52–18:57 UTC，等待其他 Rust release 构建结束后运行；Python 3.8.10。使用实际总线、运行时桥接和日志输出线程；运行时以最小 fixture 构造，不连接设备、模型或网络。定位事件与生产者一致：消息“定位快照已更新。”，payload 仅含 `source_name`，没有点云或位姿数组。

每组预先构造 10,000 个事件，测量同步 publish 调用；INFO/DEBUG 交替顺序各三轮。日志 sink 仅收集记录，不包含磁盘/网络 handler 延迟。独立捕获测量与发布测量分开，避免逐次捕获计时污染发布延迟。以下为修复后数据：

| 项目 | INFO | DEBUG |
| --- | --- | --- |
| 10k 发布总时间 | 2.82–3.01 s | 3.20–3.39 s |
| 单次 publish p50 | 222–229 µs | 223–224 µs |
| 单次 publish p95 | 591–669 µs | 700–1201 µs |
| 单次 publish p99 | 844–1121 µs | 1781–1975 µs |
| 输出线程 CPU / 10k | 0.220–0.254 s | 0.638–0.686 s |
| accepted / delivered | 10000 / 10000 | 10000 / 10000 |
| dropped / writer_failed / gap_reports | 0 / 0 / 0 | 0 / 0 / 0 |
| 回放事件数 | 10000 | 10000 |
| 实际普通遥测日志记录 | 0 | 1 |

DEBUG 仅输出一条是现有 10 秒日志限频策略，仍会为每条事件做预览和限频判断。关闭 drain 完成后 outstanding items/bytes 均为 0。没有桥接订阅的总线基线 publish p50 约 10.6 µs；它用于展示整体桥接成本，不是可替代实现。

独立捕获 10k 次耗时 2.026 s，约 203 µs/事件，序列化日志副本平均 451 bytes（总计 4,508,894 bytes）。按假设的 50 Hz 换算，捕获约消耗 10 ms CPU/s，即单核约 1%；这是从突发基准推算，不是机器人实测事件速率。

旧版纯遥测 capture 约 219 µs/事件、493 bytes/事件；修复后类别/严重级别值更短。该变化是正确性修复的结果，不宣称受控性能优化收益。

## 契约检查与边界

- INFO→DEBUG：两条已入队记录在释放输出门闩后按新的 DEBUG 级别输出
- DEBUG→INFO：同样两条已入队记录在输出时被过滤，accepted/delivered 仍为 2
- 发布者在提交后修改原事件 payload，不改变已捕获日志副本
- 混合 10k 事件含 10 个定位 WARNING、10 个 task.succeeded；INFO 和 DEBUG 均产生 20 条反馈，10 条 WARNING 运行时记录，任务流最后游标为 10000，日志无丢失且完整回放
- 既有日志隔离套件继续验证容量、在途字节、保护容量、缺口、异常、脱敏、关闭与任务状态隔离

不前移日志级别判断：这会改变排队期间动态调级的语义，也可能改变 accepted/delivered/drop/gap 的含义。`_capture` 中多个有界投影各自有身份/正文预算、脱敏与截断契约，简单合并一次投影并不等价；本次不添加缓存、锁或其他防御层。

## 测试结果

Python 3.8.10 共 196 个测试通过：

| 脚本 | 测试数 |
| --- | ---: |
| check_str_enum_compat.py | 3 |
| check_runtime_event_log_isolation.py | 24 |
| check_event_bus_contracts.py | 32 |
| check_state_store_contracts.py | 11 |
| check_agent_terminal_receipts.py | 21 |
| check_agent_wait_isolation.py | 27 |
| check_task_active_index.py | 9 |
| check_task_call_identity.py | 12 |
| check_task_receipt_evidence.py | 35 |
| check_task_summary_reads.py | 22 |

Python 3.12.14 另运行 `python scripts/check_str_enum_compat.py EnumCompatibility`：原生与强制回退 2 个测试通过。3.12 只用于枚举语义对照，没有宣称运行完整网关测试。`git diff --check` 通过。本次为相关契约套件验证，不是全仓测试通过声明。

工作区原始证据位于相邻 `neh-evidence/disabled-telemetry-20261013/`，包括 `measure.py`、`results.jsonl`（旧版）、`results-fixed.jsonl`、`warning-contract.json`、各测试输出及独立捕获 profile。旧版混合基准按预期因缺失 warning 反馈断言失败，保留失败输出；修复后全套测量退出码为 0。
