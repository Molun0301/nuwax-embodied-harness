# 更新检查与状态轮询审计（2026-10-13）

本轮结论：保留现有更新行为，没有修改生产代码。状态轮询、检查和调度已经在线程池；并发检查确有重复远端查询，但直接合并会改变分支变化及失败重试语义，没有足够收益证据支持增加协调状态。

## 已有解耦与增量缓存

- `gateways/updates.py:280–322`：状态、readiness、检查、设置和安装入口均通过 `run_in_threadpool` 执行同步工作；`218–245` 的后台设置读取、检查和安装同样已卸载
- `UpdateService.check` 只持有独立 `check_mutex`；`scripts/self_update.py:96–107` 使用独立的非阻塞 check 文件锁。网络检查不持有写操作准入锁
- GET 状态没有 Git、网络、构建或服务控制调用，也不获取服务互斥锁。它读取更新记录、安装清单、有效设置和进程归属；过期 busy 记录额外检查 `/proc` 工作进程身份
- 浏览器可见时每 5 秒读取状态，并限制本页单个在途轮询/操作；状态轮询不会触发远端检查。后台默认每 60 分钟检查，配置最短 15 分钟
- `scripts/self_update.py:338–348` 只清理临时 downloads，保留 build、sources、Cargo/Rustup、wheel 和依赖缓存。`scripts/check_self_update.py` 的 `CleanupTests` 已包含 fake-Cargo 目标/产物复用及符号链接保留回归，本轮只读未重跑
- 真正更新仅调用一次安装器 `apply`；所选 Rust binary recipe 使用已有 target 做增量构建，没有在 status/check 中构建，也没有清空 target

## 并发检查的隔离计数

实际 CPython **3.8.10**，使用真实 `UpdateService.check`、`updater.check` 和现有线程/文件锁；只替换 Git subprocess 返回值和不适用的部署检查。测试目录位于工作区证据目录；没有真正启动 Git、访问远端、读取凭据、部署或控制服务。

第一项 fake `ls-remote` 门控等待第二调用抵达原 mutex，各等待均限 3 秒。构造服务时的一次本地版本查询不计入以下检查调用：

- 相同上下文的两个重叠检查：**12 次 Git 调用、2 次 ls-remote**
- 等待时切换分支、upstream 和本地 HEAD：同为 **12/2**；首调用返回 main/origin，次调用重新读取 feature/other 和新 HEAD
- 首次远端检查失败：同为 **12/2**；首调用报错，次调用成功并清除错误记录

三项全部通过。这是调用数量和顺序断言，不是 CPU/延迟基准。

`check()` 不接受源参数，也没有锁前源快照；获得锁后才执行五次本地 Git 查询和一次远端查询。简单等待后返回已有状态会丢掉上述分支变化或错误重试行为。安全合并仍需重新读取源身份、区分本次成功结果和旧记录，故本轮没有新增 single-flight、缓存、锁、时间戳新鲜度规则或哈希。

另发现每次已安装状态读取会经 `install_home → read_current` 和 `inspect_installation → read_current` 重读一次安装清单。这是线程池中的局部 I/O；未测出实际影响，未据此改写。

## 证据与边界

独立证据目录：`neh-evidence/update-status-audit-20261013/`，相对于本任务工作区；其中 `check_concurrent_fake_git.py` 为可重跑探针，`concurrent-fake-git-py3810.json` 为完整结果，`source-audit.md` 为逐路径审计。可将脚本的 `ROOT` 指向可信检出，在已安装项目依赖的 Python 3.8.10 环境运行。

已有调度门控回归见 `scripts/check_update_schedule_nonblocking.py` 和 `docs/c1-update-schedule-offload-20261008.md`。已有准入锁不阻塞事件循环回归是 `scripts/check_update_processes.py` 的 `AdmissionTests.test_contended_admission_does_not_block_event_loop`；它不是状态 GET 的门控性能测试。`check_self_update.py` 的维护状态 GET 测试证明可读取，不证明读取耗时。

本轮没有运行真实网关/网络更新/构建部署，也没有新增 HTTP 服务器或生产路径基准。显式安装仍在准入锁内排队和派发，这是事务边界，不能用常规状态轮询的结论替代安全审查。
