# 任务快照复用已拥有的metadata（2026-10-08）

基线 `7e8093ff144d956cbb20fc21c58f5b246e6c6397`。

## 一行生产改动

`GatewayRuntime.get_task_snapshot` 在 `capture.snapshot()` 完整复制后，把 `metadata = dict(spec.metadata)` 改为 `metadata = spec.metadata`。此局部映射只读取两次 `.get()`；原有 checkpoint/finalize 选中字典的浅复制以及 result 更新复制保留。

`TaskReceiptCapture.snapshot()` 已通过 `_copy_task_value` 创建调用方独占内容；经过正常任务准入与拥有方复制的metadata为独立普通dict。映射不在其他地方暴露，不会新增任务管理器可变别名。反馈回调仅收到复制事件，其结果亦独立；原封套身份、源版本、错误与公开投影边界不变。

减少每个完整快照一次与顶层metadata条目数相关的浅遍历／分配，必要完整快照复制仍在。没有锁范围变化，没有新增校验、缓存、哈希或框架。未测延迟、CPU或RSS。

## 验证与未覆盖项

实际 CPython 3.8.20：

- 任务回执35项通过（33项已有、2项新增）；新增20组完整历史／当前快照与MCP投影差分，包含空、512个无关键、checkpoint/finalize及无效选中值、不同result形状
- 修改返回TaskSpec、提取的checkpoint/finalize、嵌套结果后，另次快照和原源不变；选中字典原有独立外层保持，新的metadata版本不改变旧capture
- 完整历史方法与Git基线逐字核对（只处理缩进及末尾空白），同一回执测试35项亦通过
- task summary18、agent terminal receipts21、agent wait status reads16项通过；当前候选合计90项，不把历史35项重复相加
- Python编译与diff检查通过。没有实际HTTP请求测试，不把MCP投影差分说成HTTP路由验收

终态恢复 `check_agent_terminal_recovery.py` 尚未执行测试：最初缺少jsonschema；在隔离3.8环境确认项目固定jsonschema4.19.2后，继续导入时缺少websockets。项目固定websockets10.4的安装遭环境网络策略阻断，未用桩替代依赖或标记通过。原导入失败及安装日志保留。这是未覆盖的恢复集成回归，不是已运行的代码断言失败。

未运行精确Python3.8.10、全部网关测试或实机。候选资格依据限于上述所有权证明、当前90项及历史对照，后续具备依赖环境再补终态恢复回归。

复现已覆盖集合：`check_task_receipt_evidence.py`、`check_task_summary_reads.py`、`check_agent_terminal_receipts.py`、`check_agent_wait_status_reads.py`（均在scripts目录），按项目依赖使用Python3.8环境执行。
