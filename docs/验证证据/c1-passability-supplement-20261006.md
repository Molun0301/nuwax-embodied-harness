# 通行性补充：现有坡道与窄廊转向用例

2026-10-06 UTC。此补充只运行先前基线/候选批次没有执行的两个精确用例，未重跑整个
测试集。复用已冻结候选二进制，MLS planner/nav-runtime opt-level=3，其余 Rust crate
沿用普通 test profile，debug=0、body-query-profile 开启；不是 release 或实机测试。

候选基于本地 `2bf54cc108a63dfa728a486fefd4de8ce8331620`、树
`1fabe2eac417c4fecde21552148f22b2b93dbe72`，对应真实远端
`410290dbc45c20db10bf2782556b555a0175bcc4`，包含已记录的冷接触域未提交优化。
本次没有重新编译或修改测试输入。

## 结果：两个精确用例通过

| 用例 | 进程 wall | user+system CPU | peak RSS |
|---|---:|---:|---:|
| `surface_follower::tests::shared_surface_planning_and_planar_commands_close_a_real_ramp_route` | 0.1223s | 0.1075s | 12,796 KiB |
| `surface_oriented::tests::small_required_turn_precedes_translation_into_a_narrow_corridor` | 5.4109s | 5.4100s | 35,104 KiB |

CPU/RSS 是整个测试进程的 `wait4` 测量，包含构造地图和测试开销，不是单次控制计算成本。
每个进程恰好执行一个用例，均退出0；Rust测试本身分别报告0.07s和5.37s。

### 坡道覆盖边界

实际规划4m、坡度0.15（约8.53°）的支撑路线，基础 SurfaceFollower 在不超过1200个
合成50ms步骤内到达原3D目标，误差小于0.12m、升高超过0.4m。模拟高度逐步重新附着到
真实 `locate_support` 结果，但姿态只有yaw。此用例**没有完整机体包络、校准MotionLimits、
实时观察、实际应用命令反馈或完整制动检查**，不能据此宣称校准机体坡道导航通过。

### 窄廊转向覆盖边界

体素墙形成0.25m净宽的窄廊，机体为0.60×0.24m。起始偏角0.026rad（约1.49°），
手工给定的路径先转向再平移。基础与校准全向运动两个分支均执行，逐步调用
`observe_applied_command` 并验证实际完整机体区间。首次必须原地转向的断言只覆盖
未校准分支。使用simulation=true和合成50ms时间，没有实时观察、RouteFollower/
PlannerWorker异步准入或Motion Guard传输。此结果不是90°墙体转弯资格。

## 原始证据与二进制身份

原始命令、日志和进程资源记录保存在
`/workspace/shared/neh-passability-gaps-20261006/`：
`run-existing.py`、`results.json`、`ramp-basic.log`、`narrow-corridor-required-turn.log`。
两条命令均使用 `--exact --nocapture --test-threads=1`。

- runtime测试二进制 SHA256：`97028fbdb3d4bccc19977ffbbe433b0be5b6ca06c488387366533b6579c1dbd7`
- stack测试二进制 SHA256：`5a16965161c221b3a84cef6e6f827202ab0725bd6f40fd4010598b2d6c98bf5a`

原有三个真实墙钟移动用例仍因75ms节拍断言失败；这两个新通过结果没有改写那些失败。
下一项最小缺口是保留真实坡道规划并加入校准完整机体、实际命令反馈及独立3D区间验证。
真实assigned-map→生产Motion Guard的运动组合仍受AF_UNIX权限限制，另见专门资格记录。

## 后续资格记录 校准完整机体坡道闭环

2026-10-06 UTC。上述两个现有用例的记录及其当时覆盖边界原样保留。本节记录新增
`surface_follower::tests::calibrated_body_closes_real_ramp_route` 的独立编译和一次精确执行，
补上完整机体、显式运动标定、实际应用反馈及独立3D区间验证的静态坡道缺口。
没有修改原有平地/坡道用例、生产算法、依赖、锁文件或验收阈值，也没有重跑整个测试集。

### 冻结输入与独立模拟

- 复用 `test_snapshot(0.15, false)` 与实际 `map.plan`，请求支撑端点为
  x=0.55→4.55m、y=1.55m，保持原4m坡道路由
- 明确完整矩形机体0.60×0.32m，局部Z范围[-0.05, 0.20]m，机体到支撑偏移0.4m；
  上/下阶上限均0.2m，横坡上限18°，额外支撑不确定度0m
- Differential `MotionLimits`：前进上限0.2m/s，倒退/横移上限0；角速度上限0.55rad/s；
  线加速/保证制动均1m/s²，角加速/保证角制动均2rad/s²，反应时间0.05s，
  最小转弯半径0m，允许原地旋转
- 模拟器使用点云的原始生成平面 `h(x)=0.05+0.15*x`，法向为
  `n=(-0.15, 0, 1)/sqrt(1+0.15²)`；机体位置按支撑点加`0.4*n`构造，
  包含水平偏移。等价机体高度为 `0.05+0.15*body_x+0.4*sqrt(1+0.15²)`，
  完整姿态保持同一法向及实际水平朝向。体素顶面只作为量化几何依据，不把实际高度吸附到格顶
- 每拍50ms，先检查原始请求的前进/转向速度和相对上一实际应用速度的加减速界，
  不裁剪非法命令。随后独立按前后速度均值积分SE(2)圆弧，反馈端点速度到
  `observe_applied_command`；每拍核对反馈已被消费
- 逐拍检查解析平面的高度连续性、原图的完整机体支撑及源单元高度带，使用原图
  `validate_body_motion` 验证实际完整姿态区间。模拟器不调用
  `body_pose_after_motion` 或 `body_orientation_at`，不以路径点替代实际运动
- 原始3D目标独立于返回路径构造，规划首末点必须与请求一致；任何失败/重规划立即判失败。
  全程断言完整包络及原始标定仍保留，不能回退为仅支撑点跟随

本用例使用 `simulation=true` 和静态地图，没有观察TTL、实时外部里程计、
PlannerWorker异步准入、生产Motion Guard传输或实机执行。`Succeeded` 只验收进入原3D目标
容差，不声称实体已经停稳。

### 精确命令与到达结果

```sh
/workspace/shared/neh-cadence-ramp-20261006/nuwax-nav-runtime-test surface_follower::tests::calibrated_body_closes_real_ramp_route --exact --nocapture --test-threads=1
```

恰好执行1个用例，退出0；`1 passed; 0 failed; 0 ignored; 235 filtered out`，
Rust测试报告1.08s，`arrived=true`、`failure=None`。

| 指标 | 结果 |
|---|---:|
| 合成实际运动步骤 | 440（上限1200） |
| 原始3D目标误差 | 0.047395m（阈值<0.12m） |
| 实际积分路径长度 | 4.024015m |
| 实际高度增益 | >0.4m的断言通过 |
| 控制调用峰值墙钟耗时 | 85.062ms |
| 整进程wall | 1.135383491004177s |
| user CPU | 1.045151s |
| system CPU | 0.054008s |
| user+system CPU | 1.099159s |
| `launch_peak_rss_kib` | 24,652 KiB |

日志打印的最终合成实际XYZ为
`[4.447123466986551, 1.5675457845596836, 1.1215434883711164]`，原始目标XYZ为
`[4.4906640095627655, 1.5500000715255737, 1.1280745697575485]`。
原始日志没有打印初始高度或精确高度增益，因此只记录通过的`>0.4m`断言。

进程wall由 `run-focused.py` 的单调时钟测量，包含启动、地图构造、测试及退出等待采样开销；
CPU和RSS来自其对子进程的 `wait4` 记录。RSS为该次启动记录的进程`ru_maxrss`，
不是单次控制内存或净增量。
控制峰值85.062ms仅覆盖 `follower.step(0.32)`，包含当时启用的诊断开销，且已超过
50ms合成控制周期及75ms节拍界限。本结果**不是实时节拍通过**，不能替代真实墙钟移动资格。

### 编译身份与证据

此次新增用例重新编译了冻结候选；MLS planner/nav-runtime使用opt-level=3，其余Rust crate
沿用普通test profile，debug=0，debug assertions和overflow checks开启，不是release构建。

- 开启 `nuwax-nav-runtime/body-query-profile` 的编译通过，并执行了上述精确用例
- 仅默认features、未启用 `body-query-profile` 的普通feature编译也通过；该二进制
  **只编译，未执行**，不能据此声称普通feature下的坡道运行结果
- 两次均为 `cargo test --locked --offline`、`--lib --no-run --message-format=json`，
  两个指定crate的opt-level覆盖保持一致；各自`build.exit`为0，
  `build.jsonl`末条均为`build-finished: success=true`，构建后源哈希检查均通过
- 本次实际执行的feature二进制SHA256：
  `c4aa36e4969c2da1f496a3b2e5a1bb057340a6d47bc065e41ea5f06f562e819f`
- 冻结的 `crates/nuwax-nav-runtime/src/surface_follower.rs` SHA256：
  `894a21d5e12723aa9413d454c14dcee6524deb6f2e7af2c1875484a600aa5862`
- 未改动的候选 `crates/nuwax-nav-mls-planner/src/surface/body.rs` SHA256：
  `5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305`

原始证据在 `/workspace/shared/neh-cadence-ramp-20261006/`：
`calibrated-ramp.log`、`results.json`中的`calibrated-ramp`记录、`binary-identity.json`、
`source-before-build.sha256`、`source-after-build.check`、`build.sh`、`build.exit`和`build.jsonl`。
普通feature的对应编译脚本、日志、退出码及源哈希检查位于其`normal-feature-build/`子目录。
本次追加仅整理已完成执行的证据，没有新增运行或修改冻结fixture。
