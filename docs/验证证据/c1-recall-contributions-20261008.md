# 召回初筛不再构造无人读取的贡献列表

日期：2026-10-08。基线：`bacd8c6b547257995a71d1166ace0d723f6c297d`。

## 单行生产改动

删除 `MemoryService._query_records` 第一遍候选中的
`point.payload["query_vector_contributions"] = sorted(scores)`，保留第二遍幸存候选的同一赋值。

第一遍的 `_RecallRank` 仅保存键、分数、去重键、关联 ID 和源版本，没有持有 point/payload；过滤已在该赋值前完成，评分、去重和排名不读取此键。最终 `retrieval_mode` 和贡献 metadata 的两个读取点只消费重新取得的第二遍快照。`query_snapshot` 中的点和投影 payload 是本次调用私有对象，不回写 live source 或外部向量缓存。

因此每个通过第一遍 kind/filter 的点少一次 0–2 个模态名的列表构造／排序和 payload 写入。不会减少 JSON 解码、模型校验或 SQL，不宣称大幅提速或给出未经测量的延迟数字。

## 定向验证

实际解释器 CPython 3.8.20；部署目标为 Python 3.8.10，未将二者混称。

- `scripts/check_memory_provider_modes.py`：10 项通过，新增一项包含词法、文本、图像、双模态四种子场景
- `scripts/check_memory_service_regressions.py`：36 项通过
- `scripts/check_memory_snapshot_payload_ownership.py`：4 项通过
- `scripts/check_memory_recall_equivalence.py`：场景脚本退出 0

新测试用现有离线 fixture 和实际 `query_snapshot`，观察第一遍写入为零、第二遍仍写入，结果保留相应模态贡献及检索模式。每场景三个源点，输出仍为确定的同一记录。

冻结基线与候选通过隔离模块加载对比：最终排名输出的记录 ID、精确分数、贡献列表、retrieval_mode 投影序列化逐字一致。该比较不是所有响应字段的完整字节等价声明。原基线在四种模式都按预期未通过新增的“第一遍不写入”断言，其最终输出仍等价；原失败日志保留。

新测试开发时曾发现假图像读取器错误索引空制品列表、检索模式预期名称不正确；修正测试夹具后才运行上述最终四套检查，未为此改动生产行为。最终源码另存备份，未将开发期失败称为通过。

没有新增哈希、锁、业务校验或缓存。没有运行模型、原生 SDK、设备或新的时间基准；历史 A/A 未通过结论保持。
