# 自动更新调度的磁盘读取解耦（2026-10-08）

基线：`02c81f20da52b97efe4c0c2a68e9e4d7a44c9992`。

## 问题和最小改动

网关 lifespan 启动 `UpdateService.run()`，该任务与HTTP共享 asyncio 事件循环。原实现每轮直接读取 preferences.json 和 check.json，实际经过 `Path.read_text` 与 JSON 解析；虽然 check/install 已在线程池，前置两次磁盘读仍可阻塞其他请求。

新增一个同步 `_read_schedule()` 返回原两份字典，使用既有 `run_in_threadpool` 一次调用完成两次读取。保持原顺序、默认值、30秒轮询、next_attempt退避和自动安装条件，不增加线程池、锁、缓存、复制、哈希或校验。

写方仍以原临时文件替换发布；返回值为独立字典。两个文件原本就不是事务性快照，此改动不声称增强一致性。读取不触及机器人或运行时线程亲和资源。

## 取消边界

`close()` 取消并等待调度任务，取消不会被异常重试分支吞掉。取消等待不等于取消物理文件读取：已开始的线程可能继续完成这两个局部只读操作，之后不能继续 check/install。沿用现有线程池取消语义，没有新增等待、重试或清理机制。

## 验证

新增 `scripts/check_update_schedule_nonblocking.py`，实际 CPython 3.8.20：

- 13项定向测试通过，包括两次读取同一工作线程和原顺序、分别门控两个读取时同一事件循环HTTP请求先完成、取消后无更新动作且显式释放观察实际工作线程、默认缺文件、开关／到期／退避／自动安装条件、每轮重读设置及两类读取错误
- 原 `scripts/check_self_update.py` 55项通过
- 原 `scripts/check_update_processes.py` 13项通过
- 合计81项通过；另 CPython 3.12.14 的13项定向测试通过
- 对可信Git导出的原02c81f20源文件运行两项门控HTTP测试，均按预期失败：HTTP直到看门狗释放阻塞读取后才执行；没有挂死或放宽断言
- py_compile 与 diff --check 通过；没有重建或更改原有测试

复现：在项目依赖环境执行 `python scripts/check_update_schedule_nonblocking.py`。历史负对照可通过该脚本 `--source` 参数加载可信的旧 `gateways/updates.py` 文件并选择两项 `test_*_read_does_not_block_peer_http` 测试。此参数会执行指定Python源码，只能使用可信项目源码。

门控测试证明并发顺序，不测量实际磁盘延迟或吞吐收益。实际Python3.8.10、生产网关整链和机器人均未运行；未检查网络更新、构建部署或安装真实服务。
