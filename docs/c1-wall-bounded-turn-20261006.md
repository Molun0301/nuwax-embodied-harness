# 校准完整机体有墙直角弯静态闭环验收

2026-10-06 UTC。新增精确用例
`surface_oriented::tests::calibrated_body_plans_and_closes_wall_bounded_ninety_degree_turn`
在冻结输入下首次执行通过：真实朝向规划器返回路线，SurfaceFollower在293个合成执行步骤后
返回`Succeeded`，原始3D目标误差0.117092m、方向误差0.061806rad。
这补充了有真实封闭墙体的90°转弯覆盖，不替代实时、异步工作线程或实机运动资格。

本次只在`services/nuwax-workspace/crates/nuwax-nav-stack/src/surface_oriented.rs`测试模块
增加一个场景辅助函数和一个用例。没有为该结果修改生产算法、依赖、地图几何、机体尺寸、
运动限制或验收阈值。用例经过执行前独立源审查，编译后恰好运行一次；本记录整理现有证据，
没有新增运行。

## 固定地图与机体标定

净空为三个矩形的固定并集，单位均为米：

- 横臂：x∈[-2.10,0]、y∈[-0.25,0.25]
- 竖臂：x∈[-0.25,0.25]、y∈[0,2.10]
- 居中转向口袋：x、y均∈[-0.40,0.40]

使用0.05m源体素，整数索引采用Rust半开区间：横臂`(-42..0,-5..5)`、
竖臂`(-5..5,0..42)`、口袋`(-8..8,-8..8)`。自由格并集恰好936格；格中心为
`((ix+0.5)*0.05,(iy+0.5)*0.05)`，地面点中心Z=0.025m、占据顶面支撑Z=0.05m。

每个与自由格八邻接的非自由格均放置真实占据墙柱，Z中心从0.025m到0.775m，共16层，
墙顶0.80m。两臂端盖和对角封口均存在并有断言；没有墙外开阔地面或仅多边形边界替代。
墙顶高于实际机体最高点0.65m。

通过现有`SurfaceNavigationConfig::new`及`apply_calibration`构造完整机体：
矩形0.60×0.32m，局部Z范围[-0.05,0.20]m，机体到支撑偏移0.40m。
实际机体参考Z固定为0.45m。半长0.30m、半宽0.16m、角点半径0.34m；居中旋转的口袋
最小墙间余量0.06m，两臂对齐直行的单侧余量0.09m。上/下阶参数为配置的0.20m、
横坡上限20°、额外支撑不确定度0m；本场景只执行水平地面运动。

校准路径保留有限墙体软偏好：`wall_clearance_m=0`、`wall_buffer_m≈0.44`、
`wall_buffer_weight=2`，没有额外圆盘硬膨胀或关闭墙偏好来获得通过。

差速`MotionLimits`与控制配置固定如下：

- 前进上限0.20m/s，倒退、横移上限均0；角速度上限0.55rad/s
- 线加速和保证制动均1m/s²；角加速和保证角制动均2rad/s²
- 反应时间0.05s，最小转弯半径0m，允许原地旋转
- 控制周期50ms、`simulation=true`；目标位置容差0.12m、方向容差0.15rad
- `max_observation_age_sec=None`，没有实时观察TTL覆盖

## 独立通行见证和实际执行链

原始起点为(-1.60,0,0.45)、yaw=0；原始目标为(0,1.60,0.45)、yaw=π/2。
规划前在不可变原地图上验证起终点、真实支撑高度和独立见证：沿横臂直行至中心，
在中心原地旋转90°，再沿竖臂直行至目标。见证长度严格为3.20m。
该见证只证明场景可通行，**没有作为路线交给规划器或跟随器**。

同一原地图的两个负例均通过拒绝断言：

1. 起点直接到目标的对角完整机体扫掠被拒绝
2. 仍在横臂起点时原地旋转90°的完整机体扫掠被拒绝

负例检查的是`validate_body_motion(...).is_err()`，不声称验证了某一种碰撞错误类型。

实际路线由`surface_oriented::plan`生成，固定搜索预算8000。返回路线须保留原起点，
通过`assert_forward_path`的前进与无横移断言，每个完整机体区间都在原地图重新验证，
终点须满足原始3D目标及yaw容差。路线不修改、不替换，直接转换为带时间戳的`Path3D`
传入`SurfaceFollower::replace_execution_path`。

每个50ms合成执行区间先检查原始控制请求的有限性、前进/转向速度上限、禁止倒退/横移及
相对上一实际端点速度的线角加减速限制。模拟器不裁剪命令；再检查前后端点速度均值的界，
独立使用该均值积分SE(2)圆弧，Z恒为0.45m。没有调用生产`integrate`或地形投影，
没有吸附到规划路线或用地图查询改写实际位姿。实际端点速度反馈至`observe_applied_command`。

每个实际积分区间都使用不可变原地图的`validate_body_motion`核验完整机体；原地图包络
在循环中和最终保持一致。完整MotionLimits及其公开运动学副本在配置不变地移入跟随器前
作一致性断言，此后按原限制检查全部输出。该跨crate测试不能直接读取跟随器私有的
config、map/source_map或last_command，因此不把公开入参和输出检查写成内部状态已被读取。

预先固定的通过条件为：规划长度≤4.80m、实际积分行程≤5.0m、累计绝对转角≤π，
1200次控制循环以内实际返回`Succeeded`，最终原始3D目标误差<0.12m、yaw误差<0.15rad。
`Pending`、`ReplanRequired`及其他失败均立即导致用例失败，没有重规划重试或见证回退。

## 准确编译命令和二进制身份

原始脚本为`/workspace/shared/neh-wall-turn-20261006/build.sh`。
在`services/nuwax-workspace`下执行，沿用
`/workspace/shared/neh-native-20261006/project-env.sh`提供的既有原生构建环境：

```sh
cargo test --locked --offline \
 --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
 --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
 -p nuwax-nav-stack --features nuwax-nav-stack/mls,nuwax-nav-runtime/body-query-profile \
 --lib --no-run --message-format=json
```

本次增量构建的Cargo日志报告20.88s；UTC开始16:50:06、结束16:50:27。
`build.exit=0`，`build.jsonl`末条为`build-finished: success=true`。
四个冻结源文件构建前后SHA256一致，写本文时再次只读核对也一致。

这是固定混合test profile：MLS planner与nav-runtime opt-level=3，stack及其余crate普通
test profile，debug=0；stack二进制身份记录opt-level=0，debug assertions、overflow checks
开启。`body-query-profile`诊断开启，成本没有从结果扣除。不是全release构建。

实际执行文件为`/workspace/shared/neh-wall-turn-20261006/nuwax-nav-stack-test`，
来自Cargo产物`/workspace/shared/neh-native-20261006/cargo-target/debug/deps/nuwax_nav_stack-9de63e8f646d88ed`，
大小28,062,608字节，SHA256：
`a039e8dbad25f3c1dcdd0e5217af6ff5fc9c9a3df1715c63616b0f9a3a652be9`。

冻结源摘要如下，路径相对`services/nuwax-workspace`：

- `crates/nuwax-nav-stack/src/surface_oriented.rs`：`0452362cfc31d109525397bf22f1b632b52381ce7fedd5161bbf8020b98a63e0`
- `crates/nuwax-nav-stack/src/surface_config.rs`：`83f3fa2fc8f83f33295f539cb33fa6e0a74db7c0d02daa8c6e1ddde15c61eeea`
- `crates/nuwax-nav-runtime/src/surface_follower.rs`：`894a21d5e12723aa9413d454c14dcee6524deb6f2e7af2c1875484a600aa5862`
- `crates/nuwax-nav-mls-planner/src/surface/body.rs`：`5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305`

## 首次精确执行结果

```sh
/workspace/shared/neh-wall-turn-20261006/nuwax-nav-stack-test surface_oriented::tests::calibrated_body_plans_and_closes_wall_bounded_ninety_degree_turn --exact --nocapture --test-threads=1
```

恰好执行一个用例，退出0、未超时；`1 passed; 0 failed; 0 ignored; 0 measured;
230 filtered out`，Rust测试报告0.79s。最终`arrived=true`、`failure=None`。

| 指标 | 首次执行结果 |
|---|---:|
| 规划墙钟耗时 | 269.967ms |
| 朝向搜索展开状态 / 生成节点 | 24 / 70 |
| 实际返回规划路线长度 | 2.899036m |
| 独立通行见证长度 | 3.20m |
| 实际积分行程 | 2.781520m |
| 合成实际执行步骤 | 293 |
| 原始3D目标误差 | 0.117092m |
| 原始目标yaw误差 | 0.061806rad |
| 累计绝对实际转角 | 2.209826rad |
| 控制调用累计墙钟耗时 | 415.951ms |
| 控制调用峰值墙钟耗时 | 9.175ms |

控制计时只包围`follower.step(0.32)`，包含最终`Succeeded`调用；没有包含外部独立模拟
和原图区间oracle。实际行程累计每个区间均值前进速度乘以50ms；转角累计均值角速度绝对值
乘以50ms。通过允许的目标容差终止，不要求执行至规划最后一个点，所以不把实际行程短于
规划长度视为路径已完整逐点走完。规划路线比3.20m见证短也不证明全局最优；见证只是
独立可行上界，不是最短路基准，且规划允许终点容差。

整进程资源直接来自`wall-turn-attempt-1.json`：

| 资源 | 记录值 |
|---|---:|
| 进程wall | 0.8028416489978554s |
| user CPU | 0.7532369999999999s |
| system CPU | 0.037583s |
| user+system CPU | 0.790820s |
| `launch_peak_rss_kib` | 39,628 KiB |
| 自愿上下文切换 | 41 |
| 非自愿上下文切换 | 179 |

进程wall由执行脚本单调时钟测量，包含进程启动、地图构造、断言、退出及等待采样开销；
CPU、RSS及上下文切换来自`wait4`。RSS为该进程启动记录的`ru_maxrss`，不是单次控制内存
或净增量。单次用例仅报告已有累计值和峰值，不推导延迟分位数、稳定20Hz或deadline保证。

## 证据范围

本结果仅证明这一固定静态封闭场景、固定混合profile及合成50ms反馈下的真实规划到跟随器
闭环。没有覆盖实时观察TTL、Scan路线优化、PlannerWorker异步准入、生产MotionGuard、
传感器或硬件运动；`Succeeded`不等于实体物理停稳。此前真实墙钟移动用例的节拍失败和
其他资格限制不因本次通过而改变，参见[节拍归因记录](c1-cadence-attribution-20261006.md)
及[通行性补充](c1-passability-supplement-20261006.md)。

原始证据目录：`/workspace/shared/neh-wall-turn-20261006/`。关键文件为
`build.sh`、`build.log`、`build.jsonl`、`build.exit`、`build-start.txt`、`build-end.txt`、
`source-before-build.sha256`、`source-after-build.check`、`binary-identity.json`、
`run-focused.py`、`wall-turn-attempt-1.log`及`wall-turn-attempt-1.json`。
这些工作区路径用于定位本次原始证据，不声称已随仓库发布。
