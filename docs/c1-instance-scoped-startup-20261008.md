# C1 启动清理按实例归属修复

本专项仅修改 `scripts/start_gateway.sh`、直接回归 `scripts/check_gateway_cleanup_ownership.py` 和本文档。未修改共享 `scripts/systemd/process_identity.sh`、安装协议或此前已发布的四文件。没有 Git 暂存、提交、推送操作。

## 问题与调用链

模块生成的 `generations/<id>/start.sh` → `scripts/start_gateway.sh` → 启动前清理 / stop_existing_gateway / stop_gateway → 各组件清理函数 → terminate_* / send_signal_to_process_scope。

修复前，网关残留通过命令名称和端口子串匹配；几何、跟随、视频和 external cleanup 通过全局名称匹配。PID 文件没有安装归属核对；PGID 不等于自身便发送整个进程组信号。几何兜底还能按端口调用 fuser/lsof 杀进程并无条件删除公共 `/tmp/nuwax_video.sock`、`/tmp/nuwax_video.shm`。这些入口不全受 core 模块关闭配置约束。独立安装目录和端口不足以保护用户其他实例。

## 最小修复

- 在启动脚本内复用已有环境身份。受管安装优先比较规范化的绝对 `NUWAX_INSTALL_HOME`，同一安装旧代际仍可回收；早期代际缺少 home 时，接受该安装 generations 内的 settings/launcher 路径。明确属于其他 home 的进程不通过回退规则认领。
- 直接启动比较绝对 bootstrap 配置路径；旧式默认配置子进程可使用精确 launcher 路径。新启动在已有标题 exec 前将 bootstrap 归一化并导出，使 supervisor 本身及后代携带一致的现有身份字段，不新增身份数据库。
- 旧式无环境标记 supervisor 只在本实例 PID 文件、精确脚本调用和工作目录共同匹配时回收；无法证明归属或只有不明确相对标记时，不按名称猜测。stop 输出提示，要求由原拥有方停止一次再重试。新启动带绝对标记，正常后续重启不因此被永久阻止。
- 扫描仅把匹配组件且属于本实例的进程交给现有退出逻辑。网关不再依靠端口筛选，因此同一实例换端口后的残留仍能回收。
- 信号目标必须属于本实例。原有进程组后代清理改为逐 PID 检查实例身份后发信号，混合组中的其他拥有方保持运行。保留原先关闭顺序、跟随 SIGINT 及有界 TERM/KILL 收尾。
- `/proc` 显示为 zombie 的进程视为已退出，避免正常退出后继续等待并尝试对已失去环境的进程升级信号。
- external cleanup 尊重各组件已有 systemd 托管标志。移除无归属的端口杀进程和公共视频文件删除；保留实例内 ready 文件的原有清理。不再认领无法确认归属的共享监听者/文件，冲突须由实际拥有方处理。

没有新增哈希、锁、无限重试、通用校验框架或新共享 helper。角色名称仍用于选择需要清理的组件，但不再作为拥有权证据。

## 隔离与测试方法

只在临时 bwrap namespace 内运行进程测试，禁止在宿主运行完整网关启动链。实际探针结果：UID 1000；可见 PID 仅 1/2；网络仅 lo，回环收发通过；宿主根只读，任务目录可写；私有 /tmp、/run 为空；最小虚拟 /dev；CapEff 为 0。未改变宿主网络、sysctl、权限、代理或安全配置。

回归脚本自动使用相同收紧方式启动 bwrap，检查 PID namespace 已变化，才运行测试。普通 sleep/HTTP listener 等是真实子进程；名称与环境用作夹具，所有 ps、/proc、信号和存活检查都实际执行。多数用例抽取生产 shell 函数，不执行整套网关；supervisor 用例使用完整生产启动脚本与一个轻量测试 Python worker。首轮 12 项版本通过抽取的 stop_existing_gateway 停止；本轮已改为实际主入口 `stop`，并新增主入口 `restart`，检查旧 supervisor/worker 退出、新 PID 发布及新 worker 就绪。这仍不等于完整产品网关验收。

执行方式：

```sh
python3 -B scripts/check_gateway_cleanup_ownership.py \
  --evidence-dir /home/ml/Documents/Codex/2026-10-08/task-2/startup-review-final
```

证据目录必须是新目录；回归保留夹具和每次 shell/子进程日志。原始结果保存在同目录 `results.json`，主日志为相邻 `startup-review-final.log`。测试结束只处理自身测试子进程，退出 namespace；未触碰宿主服务或机器人。

## 首轮结果（历史）

| 场景 | 最终结果 |
| --- | --- |
| 同名同端口，不同安装 | 本实例残留退出；其他安装保留 |
| 同一安装旧端口 worker | 正常回收 |
| 几何、跟随、视频、导航及 LCM 残留 | 本实例退出；不同安装同名进程保留 |
| 错误 PID 文件 | 拒绝停止；无关进程与 PID 记录保留，status 不误认运行 |
| 跨代际，含旧代际仅有 bootstrap | 正常回收 |
| 同一进程组混有其他拥有方 | 本实例成员退出，其他拥有方保留 |
| 公共视频文件和无关监听者 | 保留 |
| 已标记 systemd 托管组件 | 各清理入口及 external cleanup 均保留 |
| 直接启动的不同 bootstrap | 本实例退出；其他配置与不明确相对标记保留 |
| 旧 launcher 标记 / 无归属进程 | 已归属残留退出；无归属目标保留并明确提示 |
| 无标记旧 supervisor | PID + 脚本 + cwd 匹配时停止；随后重试成功 |
| 完整 shell supervisor 两次启动 | 默认路径及相对 bootstrap 输入均归一化，可停止并再次启动 |

首轮最终 12/12 通过（8.935 秒，测试墙钟，仅用于记录，不是构建性能数据）。首次实现也为 12/12；增加相对 bootstrap 边界后才执行最终一轮。

对保存的修复前脚本运行同一最终回归：2 通过、7 断言失败、3 错误。错误分别是批量旧清理超时、旧端口 worker 未退出、公共视频文件被删除；其余失败包括真实跨实例误停、混合组误停、托管组件误停、错误 PID 目标被停等。原始对照日志：`/home/ml/Documents/Codex/2026-10-08/task-2/startup-owner-before.log`；各用例详情保存在 `startup-owner-before/`。对照也仅在私有 namespace 内运行。

`bash -n scripts/start_gateway.sh`、`git diff --check` 通过。回归脚本的 Python 3.8 语法解析通过；实际执行解释器为 Python 3.12.3，没有精确 3.8.10 运行证明。该轮没有运行原生编译或完整 core 安装。

## core 后续离线依赖准备

按允许的轻量范围，只读取已有 pip 缓存并在任务目录创建 wheel 符号链接，没有复制 wheel 字节、下载或安装包。真实 pip 使用 `--dry-run --ignore-installed --no-cache-dir --no-index --find-links` 解析原样 `requirements/modules/core.in` 加 pip/setuptools/wheel，36 个包全部解析到本地 file 来源。AnyIO 最新缓存版本与固定 typing_extensions 冲突，pip 正常回溯至缓存 AnyIO 4.14.2；项目的 typing_extensions==4.13.2 等版本声明均未修改。

解析证据：`/home/ml/Documents/Codex/2026-10-08/task-2/core-offline-resolve.json` 和 `.log`；缓存引用目录为 `wheel-links-readonly-cache/`。这是 Python 3.12 当前平台的离线解析通过，不是安装、pip check 或运行验收。之后已按父任务排期完成离线 core + customrobot + foreground 安装：固定已发布 a0d31df23f93992cf75b458cba756e6d6543eb6d，首次 6.891 秒、原样第二次 0.243 秒，pip check、目录声明 imports、16 项直接固定依赖、manifest/隔离路径/启动器语法通过。证据在 task-2/core-full/evidence/；服务未启动，启动候选未应用。本轮保留这些记录，没有重复安装计时。真实网关健康失败回退仍未放行。


## 本轮审查修复与主入口验收

父任务解除上述三文件冻结后，本轮补齐以下生命周期边界：

1. **同安装混合组中的托管组件。** 原逐成员逻辑只检查安装身份，仍会间接终止 systemd 托管组件。现在对已选中的单 PID，通过命令角色复用现有四类托管标志；`send_signal_to_process_scope` 在主目标和同组成员发送任何信号前都排除托管组件。没有增加全系统扫描、身份字段或通用管理框架。组件名称仅决定是否保留，不能建立拥有权。
2. **实际 stop/restart 锁顺序。** 运行中的 supervisor 持有生命周期锁，原主入口在 case 分派前抢锁，导致 stop/restart 直接报锁占用退出。现有锁仅由 start 在启动前获取、restart 在停止旧实例后获取；stop/status 不获取启动锁。仍为非阻塞 flock；没有引入第二把锁或调整锁文件布局。
3. **已跟踪子进程停止。** `stop_gateway`（INT/TERM trap）、worker 退出收尾和 activation owner 清理会调用 `stop_*`。geometry/nav/person-follow/video 的定向停止原来已先看组件托管标志，再经 terminate_* 核对身份，但随后的 wait 可能仍等待被拒绝停止的目标。本轮将身份和托管排除放到该 kill/wait 分支之前。三类 monitor 停止入口（person-follow、runtime activation、nav/geometry monitors）原来直接 `kill -TERM PID`；现在逐个已跟踪 PID 检查身份和托管排除，再定向 kill/wait，不遍历其他进程。内部 shell monitor 由 `$!` 跟踪，正常不匹配原生组件角色，即使对应原生组件由 systemd 托管也可以收掉自有 monitor。

调用链具体为：CLI stop/restart → stop_existing_gateway → terminate_process_scope → send_signal_to_process_scope；supervisor TERM/INT → stop_gateway → worker terminate + 各 stop_*；worker 退出/重启循环 → stop_runtime_activation_monitor / stop_runtime_monitors / 各组件 stop_*；activation owner 退出 → stop_person_follow_monitor + 本 owner 新建的组件 stop_*。跟随组件仍保持 SIGINT 优先。无法证明归属的目标继续保留并明确提示原拥有方处理。

新增交叉用例把同安装 gateway、普通自有成员、geometry/LCM/nav/video/person-follow/Go2-video 六类托管进程放在**同一个 PGID**：普通自有成员退出，六类托管进程全部存活。另测三类 monitor 定向停止与四类组件定向停止，以及真实 CLI stop → 再次 start、真实 CLI restart 替换旧 worker、CLI stop/restart 拒绝无标记 PID。

最终仓库版本：**17/17 通过，20.728 秒**（轻量测试墙钟，不是编译性能计时）。实际使用 Python 3.12.3，Python 3.8 语法兼容检查另行通过，未运行精确 3.8.10。最终日志与逐测试夹具：

- `/home/ml/Documents/Codex/2026-10-08/task-2/startup-review-final.log`
- `/home/ml/Documents/Codex/2026-10-08/task-2/startup-review-final/results.json`

回归夹具本轮补齐 `core/config_catalog.json`，避免旧夹具 config_values 的缺文件诊断干扰；没有改该生产文件。

最终同一套回归对本轮修复前冻结候选的对照保存在 `startup-review-final-baseline.log` 和 `startup-review-final-baseline/results.json`。原候选实际暴露四处失败：CLI stop 锁占用、CLI restart 锁占用、同安装混合组误停托管组件、monitor 定向停止误停无关目标。其余场景保留通过。

未运行：真实产品网关启动、安装成功后新网关健康失败的完整回退、原生编译。未改 systemd、宿主网络/权限、机器人连接或其他任务文件。完整 core 安装记录保留，不重复安装计时。本轮三文件交回审阅，后续继续冻结。


## 第二轮审查：精确 argv 角色分类与 PID 保留

父任务指出上一版 `process_managed_by_systemd` 对整条 ps args 做子串匹配，普通 supervisor 的安装路径或无关参数包含组件名时也会被误判。现只读取所选 PID 的 `/proc/<pid>/cmdline`：原生组件仅匹配 argv[0] 的 basename/既有 exec 标题；Python 视频组件仅匹配生产启动形式 `python[版本] -m drivers.go2.video_streamer`。目录名称、普通参数、Python `-c` 后的文本均不参与角色判定。没有增加身份字段或通用参数解析框架。

`stop_existing_gateway` 在 terminate 返回后重新检查目标存活；仍存活便报错返回 1 并保留 PID 记录，restart 不继续抢锁或启动。只有目标实际退出才删除其 PID 文件。

回归夹具相应改为真实 argv[0] 原生角色标题和实际 Python `-m` 测试模块，不再以尾随普通参数模拟组件身份。同一安装、同组中的六类托管组件保留用例继续通过。新增普通自有进程路径/参数含组件名与存活目标保留 PID 的用例；真实 CLI restart 也改在 `nuwax-nav-stack/source` 路径下运行并启用 nav 的已有 systemd 托管标志，旧 supervisor/worker 退出、新实例就绪与最终 stop 均通过。

最终 **19/19 通过，13.367 秒**。同套回归对上一版冻结候选为 **16 通过、2 断言失败、1 超时错误**：组件名路径中的普通进程不退出、真实 CLI 重启因残留锁失败、仍存活的托管目标 PID 文件被删除。墙钟只是轻量测试记录，不是性能比较。全部在私有 bwrap namespace 内完成。

证据：`task-2/startup-argv-final.log`、`task-2/startup-argv-final/results.json`、`task-2/startup-argv-baseline.log`、`task-2/startup-argv-baseline/results.json`。本轮生产增量为 `task-2/c1-startup-argv-production-delta.patch`。bash -n、git diff --check 和 Python 3.8 语法检查通过；实际执行 Python 3.12.3，未运行精确 3.8.10。

仅三文件变更，交回父任务审查后冻结；无 Git 写操作、宿主网关、原生构建或重复安装计时。
