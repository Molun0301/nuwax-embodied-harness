# 公开导航目标省略高度策略资格记录

2026-10-06 UTC。**最终策略：仅 assigned schema-2 三维地图自动采用拥有方机体高度；legacy schema-1 三维和 planar 省略 Z 仍为原兼容值 0，不是拥有方高度证据。** 显式 Z 不变。

初轮相关回归112项通过（其中高度文件21项）的历史证据保留在下文。最终源码审查发现初轮对legacy范围的承诺过宽，随后完成兼容性修正并通过28项高度聚焦检查。**112项没有在最终修正上重跑，不能称为最终源码的112项通过。** 最终新增原始日志、四文件摘要和独立审查见文末“最终schema范围修正”。本记录不包含原生组合、实机运动或执行器安全资格证明。

## 策略与范围

- MCP 与 HTTP 保留未传入 `z` 的语义；HTTP 的平铺坐标及嵌套 `position` 均覆盖。
- assigned schema-2 的 `lio_3d` XY 请求在工作区及定位准备完成后，通过
  `map_workspace_service.get_relocalized_pose(map_name, require_assigned=True)` 读取当前有效机体高度。
  已分配地图路径由现有服务核对精确原生终态、激活、定位会话及提供器拥有方；
  网关只补充导航高度的坐标系、有限位置及墙钟新鲜度准入。
- 位姿与目标必须使用同一规范坐标系。保持精确相等比较，不将 `map` 与
  `other/map` 视为同一坐标系，也不增加帧别名宽容范围。
- 复用浏览器现有时间窗口：源时间年龄及 RPC 接收时间年龄都在闭区间
  `[-1 秒, 2 秒]`，即最多允许源时间或接收时间超前当前时间 1 秒、滞后 2 秒。
  新 RPC 的接收时间不能刷新旧的源时间。没有新增等待或失鲜预算。
- 异步受理的未解析目标带内部 pending 标记，受理载荷省略 `z` 并标明
  `target_height_pending`。准备回调返回已解析目标，派发边界拒绝任何仍带
  pending 标记的目标；通用 `Pose` 的数值默认值不能以未解析目标派发。
- 高度只解析一次。首次规划输入拒绝与已停车的失鲜输入重试都保留完整的已解析
  目标，不重新读取变化中的机体高度。
- 显式 `z`（含零值、正值与负值）保留原值。legacy schema-1 三维与 `slam_2d` 的 XY 默认高度仍为零；legacy不读取位姿，也不把零称为当前机体或地面高度。
  assigned样本缺失、过旧、过度超前、坐标系不符或拥有方无效时明确失败，不猜测地面高度；准备后资产无法分类同样失败。
- 未改变探索共用的位姿辅助函数、原生源代码或原生组合的显式目标高度；没有
  新增哈希核验、全局锁或跨 RPC 的状态锁。未使用通用定位缓存或 GO2 调试位姿
  作为省略高度的后备来源。公开工具与 HTTP schema 已说明新策略。

## 初轮冻结文件（历史身份）

生产差异：`gateways/runtime.py` 增加 61 行、删除 7 行；
`gateways/http/api.py` 增加 4 行、删除 3 行。
高度测试文件由原 8 项现状复现更新和扩展为 21 项，共 515 行。
该测试文件当前由 Git 忽略，发布测试时需要显式纳入；本次没有暂存或提交。

| 文件 | SHA-256 |
| --- | --- |
| `gateways/runtime.py` | `cc0b44dc0ef781a4cf18574c14ba40bd141d33e89966fcc7e3a0f6c550e4861e` |
| `gateways/http/api.py` | `34d96f4a06bc1818f19456116560b0d4f96361f950e59c2d86ffe9cf120c8ee4` |
| `tests/test_navigation_goal_height_public_input.py` | `90e3be1ee99f96d1da48ac8693981c897ed995d865384cae29ba7cebf75bf750` |

初轮记录编写前校验过以上摘要。它们对应初轮112项检查，不是后述最终修正源码。

## 初轮21项高度专用测试

公开输入与导航任务闭包 14 项：

1. 真实 MCP schema 说明各建图链的省略高度策略，不再声明统一的零值默认。
2. 探索共用的通用位姿辅助函数保持原行为。
3. MCP 三维 XY 请求保持 pending，准备完成后派发实际机体高度。
4. MCP 在两条建图链保留显式正值、零值及负值。
5. HTTP 位姿兼容辅助函数保留缺失与显式零值的字段区别。
6. HTTP 两种坐标形态保留省略语义并派发拥有方高度。
7. HTTP 两种形态及两条建图链保留显式高度。
8. MCP 与 HTTP 的二维 XY 请求继续派发零高度。
9. 缺失、过旧、过度超前、不同坐标系、非有限坐标及无时区时间均不猜测或派发。
10. 源时间与接收时间精确覆盖 `[-1 秒, 2 秒]` 的包含边界及越界值。
11. 异步准备可以替换旧定位，解析发生在准备之后。
12. 已准入的同步调用在提交任务前解析高度。
13. 首次 `PLANNER_INPUT_STALE` 拒绝和后续已停车 `StaleInput` 恢复共三次提交，
    完整目标序列化结果严格相同，仅读取一次高度。
14. 即使准备未清除 pending 标记，派发边界也阻止提交。

真实已分配工作区、运行时、提供器、定位会话及 protobuf 准入集成 7 项：

15. 当前拥有方的真实准入样本到达导航派发，历史缓存高度不会覆盖当前样本。
16. 成功 RPC 的新接收时间不能使过旧或过度超前的源时间有效。
17. 地图位姿不能为另一目标坐标系提供高度。
18. 准备后原生 correction owner 不匹配时不派发。
19. 准备后原生样本缺少位姿几何时不派发。
20. 样本 RPC 期间提供器拥有方变化时不派发。
21. 样本 RPC 期间定位会话变化时不派发。

集成测试模拟原生 RPC 与导航后端；不启动原生服务或机器人。时间边界单测固定
验证时钟，避免测试执行耗时改变未来时间负例的语义。

## 初轮执行命令与结果

工作目录：
`/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006`

```bash
PYTHONPATH=.:tests /workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/bin/python -m unittest -q test_navigation_goal_height_public_input test_assigned_localization_sample test_assigned_workspace_relocalization test_runtime_assigned_relocalization test_assigned_localization_session
```

初轮结果：`Ran 112 tests in 14.443s`，`OK`，退出码 0。最后一次输出还包含定位会话
发布回调故障注入用例的预期 `RuntimeError: callback failure` 日志；测试总结果通过。

**证据保留限制：**上述初轮结果来自终端工具输出块 `b002ba`，它是工具执行记录
标识，不是本地文件路径，也不是已保存的原始日志。初轮没有保存测试 raw log；
本 Markdown 是对已观察执行结果的资格记录，不能声称可由一个不存在的日志文件
重新核对。编写此记录时没有重跑测试。

## 初轮静态检查与审查

```bash
/workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/bin/ruff check tests/test_navigation_goal_height_public_input.py
bash scripts/check_layering.sh
git diff --check -- gateways/runtime.py gateways/http/api.py
```

- 高度测试文件 Ruff 通过；初轮输出也在工具记录 `b002ba`。
- 分层硬规则全部通过，R6/R7 保留既有人工复核提示；工具记录 `07eb81`。
- 差异空白检查通过。
- 对两个生产文件分别使用 Ruff `check --output-format json --stdin-filename <path> -`，
  比较 `git show HEAD:<path>` 与当前内容的诊断 code/message 多重集合。
  `gateways/runtime.py` 既有 98 项、当前 98 项；`gateways/http/api.py` 既有 5 项、
  当前 5 项；**新增诊断为零**。工具记录 `d1c151`。这不表示生产文件全量 lint 清洁。
- 初轮独立只读审查确认了assigned路径的异步顺序、拥有方准入、精确帧匹配及完整目标重试；随后累积源码审查发现legacy范围缺口，以下修正取代初轮的全三维承诺。

## 最终schema范围修正（21:34 UTC记录）

### 发现与最小修正

初轮resolver适用于所有lio_3d，但工作区的legacy `get_relocalized_pose` 只有旧回执相等检查，再调用没有原生拥有方证据的 `GetCurrentPose`。它不能满足assigned路径的强拥有方承诺。保留旧helper读能力并不意味着应该将其用于新的自动高度策略。

最终只改四个文件：

- 在定位准备之后把已有workspace传给高度resolver，不增加catalog加载、RPC、全局锁或点云复制
- 明确schema1时只移除pending标记，保留输入的原默认0，完全不调用高度getter；schema2调用既有强准入路径；缺失/未知prepared asset不当作legacy
- 没有pending标记的显式Z和planar请求立即返回，不增加schema/pose查询或拒绝条件
- `MapWorkspaceService.get_relocalized_pose(..., require_assigned=False)` 新增可选关键字。True时，当前catalog缺失或降级在读取任何legacy回执/位姿前返回None，防止prepared schema2掉入旧路径。默认False保持所有原调用者的legacy兼容读取能力；未声称增强其拥有方证据
- MCP能力/参数、skill和HTTP描述明确区分两类地图，不再承诺所有3D地图都能自动获得owner-bound Z

### 实际运行与红绿证据

证据目录：`/workspace/shared/neh-height-legacy-scope-20261006`。所有运行保存独立command/stdout/stderr/exitcode；`before/`、`after/`保存四文件快照，source SHA校验把日志绑定到相应源码。没有重跑112项或原生测试。

| 运行 | 方法数/结果 | 说明 |
| --- | --- | --- |
| `01-baseline` | 21通过，2.773s，exit0 | 原高度文件在修正前的保留基线 |
| `02-red` | 27方法，13个失败subtest分布于4方法，3.754s，exit1 | 新legacy/异步/不可分类负例；实际观察到legacy采用91.5或调用不应读取的getter |
| `03-red-guard` | 2方法均失败，0.417s，exit1 | 首个弱downgrade断言曾偶然由旧回执不一致拒绝；明确禁止legacy回执读取后，分别复现缺失和降级的真实回落缺口 |
| `04-focused` | **28方法通过，4.219s，exit0** | 最终修正，完整高度文件；含原assigned有效/陈旧/frame/provider/session集成 |

新增7个方法覆盖legacy MCP、legacy HTTP两形态、异步准备后schema选择、无法分类owner、旧helper默认/显式False兼容、真实assigned服务的catalog降级、真实assigned服务的catalog缺失。显式正/零/负高度的原用例扩展到两schema×两pipeline。最终catalog负例禁止legacy回执和pose读取，不以偶然的旧回执拒绝冒充新guard通过。

最终命令（与baseline、首个red同一工作目录和选择范围）：

```bash
PYTHONPATH=.:tests /workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/bin/python -m unittest test_navigation_goal_height_public_input -v
```

| 最终文件 | SHA-256 |
| --- | --- |
| `gateways/runtime.py` | `887241d36c0f1b51c08538045449e73d5495a445b718bd54ced025ba7da24671` |
| `gateways/http/api.py` | `3eebb2427318d1666c7aefd082ce63af0cce8bbdd3c8cf6bc1e9cb5b891e48a2` |
| `skills/map_workspace.py` | `1f6b4915c70bb07d8a48f1aad690af9adf08b93a3131f46d7d4b847fc6c76032` |
| `tests/test_navigation_goal_height_public_input.py` | `fd156e3b2a299018ab042eed35b2e368dc6874644a184968fc72fb900e4d4ae6` |

`04-focused.stderr.txt` SHA-256：`23246de063ba12c7481554216a1abd4129f21f0d79250fe57d2a408ec339ea75`。最终after快照、focused source记录和当前四文件摘要一致。独立审查已核对实际分支、真实assigned服务接线、红绿日志及摘要，清除此项范围阻塞；没有新增native owner接口或将legacy采样包装成强证据。

## 解释边界

本记录只证明所列 Python 输入、编排、拥有方验证衔接及派发载荷行为。
原生样本几何计算、实时规划节拍、实际传感器数据、执行器制动与实机安全均不在
此次证明范围。没有运行原生构建、没有实机运动，没有 Git 暂存、提交或推送。
