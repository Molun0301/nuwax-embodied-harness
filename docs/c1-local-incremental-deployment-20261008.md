# C1 本机真实增量构建与更新恢复验收

基线：`a1a656b1bec2f32f364ddc9234731f1654ffa6da`，分支 `fix/v3-legacy-cloud-seal-20261006`。修改范围为安装/构建回归、本文档和 self_update.py 的恢复失败诊断；不修改产品构建实现。测量环境为用户工作电脑，Python **3.12.3**；没有精确 Python 3.8.10，3.8 语法兼容检查不代表该解释器执行验收。

## 权限、范围与准备

共享检出 `/home/ml/Documents/Codex/2026-10-07/task/neh-done-work`，持久可写任务目录 `/home/ml/Documents/Codex/2026-10-08/task-2`。本任务未操作共享检出的 Git 索引、HEAD 或远端。收到父任务确认基线和原生独占窗口后执行，最多两个编译任务。

已只读查看 README、公开安装部署/升级恢复文档、安装器和更新器；当前树及祖先未发现 AGENTS.md 或相关 .agents/skills。本机 memory_summary.md 可读，未匹配安装历史。已有 `services/nuwax-workspace/target`、Cargo registry、Rust 1.88.0 与 SDK 均保留。系统已有 CMake、C++、Eigen 3.4.0、PCL 1.14.0、LCM 1.5.0。本阶段不连接机器人、不改 systemd/鉴权/网络、不下载依赖。

## 调用链及真实范围

`install.py → module_installer.cli.main → manager.apply → builds.build_modules`。
完整 slam2d 模块选择会补齐 core/geometry，并准备二维依赖、构建 slamtec、Rust worker/geometry。此次只选择其中真实的 `builds._cmake` → 仓内 `drivers/sensors/slamtec_native` 子组件作为编译切片；其 SDK 已在仓内，不重建整套 SDK，也没有把编排 mock 算作编译收益。没有执行完整 Python 环境安装、二维依赖准备或 Rust 模块构建。

持久证据目录：`/home/ml/Documents/Codex/2026-10-08/task-2/c1-evidence`。包含源码夹具 `slamtec-source/`、构建缓存 `build/`、安装夹具 `generation-*`、每阶段原始日志以及 `measurements.json`（完整命令、退出码、时间和重编对象路径）。不将当前/回退产物当作临时缓存清理。

实际每阶段调用以下命令，配置变化阶段另加 `-DCMAKE_CXX_FLAGS=-DNEH_INCREMENTAL_TEST=1`：

```sh
cmake -S "$E/slamtec-source" -B "$E/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$E/generation-N" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build "$E/build" --parallel 2
cmake --install "$E/build"
```

编译计数取真实 `.o` 文件的 mtime_ns 变化，不新增哈希扫描。各成功产物通过独立 Python 进程加载共享库并调用无设备访问的 `nw_slamtec_abi_version`；仅在源码夹具内将返回值从 1 改为 2，以确认修改实际进入编译产物。

## 真实结果

时间为单次配置、构建、安装总 wall time，属于本机样本，不是跨平台性能承诺。

| 场景 | 重编对象/总数 | 秒 | 结果 |
| --- | --- | --- | --- |
| 首次准备和构建 | 16/16 | 2.776 | 通过，ABI=1 |
| 同前缀完全无变化 | 0/16 | 0.156 | 通过，ABI=1 |
| 源码/选项无变化，仅新安装代际 | 0/16 | 0.128 | 通过，ABI=1 |
| 局部修改 bridge.cpp，新代际 | 1/16 | 0.469 | 通过，ABI=2 |
| 编译选项改变，新代际 | 16/16 | 2.413 | 通过，ABI=2 |
| 上述选项无变化，新代际 | 0/16 | 0.114 | 通过，ABI=2 |
| 注入真实 C++ #error | 0 个成功更新，编译退出码 2 | 0.280 | 预期失败，已有 generation-5 库仍可加载且 ABI=2 |
| 修复夹具源码后复用缓存重试 | 1/16 | 0.452 | 通过，其余 15 个对象复用，ABI=2 |

根因判断：此子组件每次仍执行 CMake configure，但新代际路径没有导致对象重编；无变化总耗时约 0.1–0.2 秒。源码局部修改及全局编译选项改变分别触发合理的局部和全部重编。没有实证支持修改该实现，故不添加配置签名、哈希、锁或通用缓存框架。原先对每代 prefix 导致无谓重编的猜测在此子组件上不成立；不能外推到 Livox/LCM/Rust 的依赖路径行为。

可复用回归：`scripts/check_local_incremental_build.py`。选择一个新的持久证据目录运行完整矩阵：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 scripts/check_local_incremental_build.py \
  --evidence-dir /path/to/new-persistent-evidence-directory
```

`--retry-only` 用于在已有前六阶段夹具上补跑失败/恢复两个阶段；保留旧记录，不下载或清理缓存。已有此次证据请保留，不对同目录重新执行首次构建计时。

## 更新事务调用链

已追踪 `self_update.execute`：固定目标 → 停机 → 快进 → `manager.apply(repair=True, keep_previous=True)` → 启动/核对 → 清理临时下载。失败时尝试恢复旧源码和 current，再启动核对旧版本；构建缓存不由失败路径清空。`manager.apply` 在新代际未生效时删除失败代际，保留 current 和缓存。

## 恢复验证（轻量 Python 阶段）

本机已有解释器 `/home/ml/.local/share/nuwax/scene-decision/venv/bin/python` 可复用。实际 Python 3.12.3，FastAPI 0.115.14、Pydantic 2.10.6、HTTPX 0.28.1、Starlette 0.46.2；后二者与 core.in 固定的 0.27.2/0.44.0 不同，不宣称锁定依赖环境验收。本任务未改包或安装依赖。

初次沙箱运行中，socket 创建返回 EPERM，异步相关用例停滞；保留初次日志后中止测试。通过正常平台审核允许本机回环 socket 后，既有完整回归全部通过，未改代理、认证或系统权限：

| 回归 | 结果 | 原始日志（相对于持久 recovery-evidence） |
| --- | --- | --- |
| check_self_update.py | 55 项通过 | check-self-update-approved.log |
| check_update_processes.py | 13 项通过 | check-update-processes-approved.log |
| 修复后 TransactionTests + DescriptorTransactionTests | 13 项通过 | transactions-after-fix.log |

此前还运行了 39 项针对性 Git/事务/缓存测试及 8 项真实进程测试，均通过；它们是上述集合的子集，不累计为额外覆盖。

### 无替身的隔离更新事务

新增 `scripts/check_update_recovery_live.py`，使用标准库和生产更新器。持久证据根：`/home/ml/Documents/Codex/2026-10-08/task-2/recovery-evidence`，`live-baseline/` 保存修复前观测，`live-fixed/` 保存修复后观测。各自包含真实本地 origin/checkout Git 夹具、安装记录、启动器、测试 HTTP 服务、状态、Git/进程/更新日志与 `results.json`。这些夹具不是项目额外工作分支；没有操作共享检出 Git。

实际执行：

```sh
PYTHONDONTWRITEBYTECODE=1 python3 scripts/check_update_recovery_live.py \
  --root /home/ml/Documents/Codex/2026-10-07/task/neh-done-work \
  --evidence-dir /path/to/new-persistent-recovery-evidence
```

测试不替换 `execute`、Git、queue、command、stop、start 或 verify。目标提交中的安装器文件在导入末尾故意抛错或等待；真实安装子进程失败/中断。旧安装是小型标准库 HTTP 进程，持有真实启动锁、PID 记录与安装环境身份，报告启动时读取的真实 Git revision。它不是实际产品网关，不访问设备。

| 场景 | 结果 | 实际观察 |
| --- | --- | --- |
| HTTP 返回错误版本 | 通过 | 真实 verify 拒绝，明确超时原因；独立于事务测试 |
| 目标安装入口失败 | 通过 | 实际 fetch/快进后子进程退出 1；旧 HEAD/current 恢复；旧 HTTP 进程重启并返回旧版本，状态 rolled_back |
| 同一失败目标再次重试 | 通过 | 能重新排队、重走实际事务并回退；缓存夹具内容与 mtime 不变 |
| SIGTERM 中断安装阶段 | 通过 | 真实更新器捕获信号，安装子进程退出且不残留，旧 HEAD/current/HTTP 恢复，状态 rolled_back |
| 原失败后旧启动器也退出 23 | 通过 | 保留旧 HEAD/current，状态 recovery_required，禁止再次排队，不谎报恢复成功 |

这里缓存文件仅证明事务没有清空缓存；真实编译复用量只取前一阶段 slamtec 的对象统计。安装失败发生在目标安装器导入阶段，没有执行模型/SDK/原生编译，也没有绕过后续构建后宣称完整安装成功。

### 已复现缺陷与最小修复

修复前，旧启动器真实退出导致恢复验证抛出“新网关启动后退出”，但 `self_update.execute` 的恢复异常捕获丢弃该原因。持久状态只有最初安装失败 `failure` 和泛化的人工恢复提示，构建日志中也没有这条恢复原因。

修复仅在原有异常处理分支保留 `recovery_failure=str(recovery_error)`，并把原因加入现有 message；保留原始 `failure`。没有改变回退、信号处理、缓存清理或生命周期行为，没有新增哈希、防御校验、锁或缓存框架。真实回归现在同时断言原失败退出码、恢复失败原因及用户消息，全部通过。

## 验收边界与保留事项

**通过**：slamtec 真实增量矩阵和失败后重试；既有 55+13 项回归；真实隔离更新入口失败、重试、SIGTERM 回退、恢复失败诊断；Python 3.8 语法检查。

**未运行**：精确 Python 3.8.10；完整模块首次/二次安装；实际产品网关与 SDK 全链；新安装构建成功后因新网关健康失败触发的无替身完整事务（既有事务回归覆盖该编排，真实 HTTP verify 单独覆盖错误版本）；SIGKILL/掉电后的完整人工恢复。不能将本次结果外推为全部模块速度或硬件能力验收。

已有目标、SDK、构建缓存、当前/回退产物均保留。只结束本测试启动的进程。原生窗口已释放；本恢复阶段没有原生编译或正式性能测量。所有源码改动尚未暂存/提交，交由唯一 Git 操作者审查；本文档受 `.gitignore` 的 `*.md` 规则忽略，纳入版本时需显式处理。
