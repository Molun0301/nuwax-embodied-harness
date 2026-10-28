# 默认调用账本与源码更新兼容

2026-10-08，资格基线 74b9256c。

本机真实网关安装发现默认 `artifact_dir.parent` 位于源码根，两份 Agent 调用 SQLite 账本被 Git 当作未跟踪源码，生产 `require_clean` 因而拒绝更新。

本次只为根目录 `agent_calls.sqlite3`、`agent_calls.terminals.sqlite3` 及各自的 `-journal`、`-wal`、`-shm` 添加精确忽略规则。没有迁移、删除或改写账本；存储位置、作用域和历史查询保持原样。没有忽略所有 SQLite 文件，没有放宽生产源码检查。已经跟踪的数据库仍会报告修改。

`check_agent_ledger_git_hygiene.py` 使用真实 Git 仓库及 SQLite WAL/回滚事务，直接调用生产 `require_clean`。实际 CPython 3.8.20 和 3.12.14 各 5 项通过；3.8.10 未实测。旧规则在同一 3.8.20 用例中出现预期的两项数据库错误，另外三项通过。无关数据库、备份文件、嵌套同名数据库和源码改动仍被拒绝；活跃数据库中的历史行保持可读。

最初测试把嵌套反例放在项目原本忽略的 fixtures 目录，导致反例断言失败。纠正为普通 sample_package 目录后重跑通过，初次日志保留。这不是扩大忽略规则的理由。

此项不能代替完整安装和健康失败回退验证。独立本机回退试验发现的 supervisor TERM 返回状态问题由启动链路阶段继续处理。
