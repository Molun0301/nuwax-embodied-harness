# 有墙直角弯朝向规划耗时归因（未实施生产优化）

2026-10-06 UTC。对既有静态有墙90°弯用例增加局部、`cfg(test)`诊断，两份冻结二进制各执行一次，均通过。确认passage与后备引导的两次支撑图查询输入及输出逐位相同，但第二次查询仅占本次总规划墙钟的约0.36%—0.47%；当前证据不足以支持把它作为主要优化。**没有复用候选、生产算法改动或已实现的性能提升。**

本轮工作仅保留测试归因及本文；未执行提交或推送，按当前要求在2026-10-07 00:00 UTC前不进行Git写入。历史[原场景验收](c1-wall-bounded-turn-20261006.md)的269.967ms来自另一份源码/二进制及执行，只作历史记录，不能与本轮400.941ms或322.840ms构成配对前后优化结果。

## 固定场景与实际结果

沿用`surface_oriented::tests::calibrated_body_plans_and_closes_wall_bounded_ninety_degree_turn`；地图、0.60×0.32m标定机体、0.20m/s前进上限、50ms合成执行、搜索预算8000和原验收阈值均未变化。独立3.20m通行见证未输入规划器或跟随器，完整路线和实际运动区间仍由原地图验证。具体几何及断言见原场景验收。

| 规划墙钟，ms | guide-baseline | centered-baseline |
|---|---:|---:|
| 整次plan | 400.941 | 322.840 |
| raw完整机体规划 | 221.460235 | 189.611929 |
| raw内朝向搜索 | 126.621557 | 117.638685 |
| 初次圆角 | 0.011666 | 0.011073 |
| 居中阶段合计 | 179.449786 | 133.201299 |
| passage引导查询 | 2.125180 | 1.956786 |
| 后备引导查询 | 1.898473 | 1.147449 |
| 目标区域准备，截至该检查点 | 19.712467 | 9.952910 |
| 搜索入口，截至该检查点 | 94.831777 | 71.966205 |

第二份诊断进一步拆分居中阶段：入口条件1.830521ms、`centered_forward_candidate`整体114.074749ms、第二次圆角16.911677ms、最终比较0.368293ms。实际进入该阶段，候选为`Some`、圆角前158个位姿；这不是最终路线点数。最终比较含终点检查、路径代价及接纳时替换/释放旧路径，不能单称代价计算。helper计时包含其支撑准备、引导生成、候选连接等全部工作，尚未把这些子项单独计时。

两次实际`map.plan_in_boundary`调用均为每处一次，输入使用同一调用内的map引用、同一起终支撑坐标及空boundary；完整结果均为同样的4个支撑点，`f64::to_bits`逐项相等。诊断同时能保留原始错误，但本次结果均为`Ok`。跨进程map地址不同，不应据此否定单次调用内的同源性。两份日志中的完整最终路线均为152个位姿，全部7个位置/四元数分量的位模式逐项相等，不只比较末点或近似长度。

两次均即时朝向搜索1次、展开24状态、生成70节点；没有预算耗尽或延后搜索。实际返回`Succeeded`，293个合成执行步骤；规划长度2.899036m、实际行程2.781520m、累计绝对转角2.209826rad、原始3D目标误差0.117092m、yaw误差0.061806rad、`failure=None`。各自测试摘要均为`1 passed; 0 failed; 231 filtered out`，退出0、未超时。

## 构建、冻结源码及证据位置

证据根目录为`/workspace/shared/neh-oriented-guide-20261006/`：

- `baseline/`：`guide-baseline-attempt-1.log`、`guide-baseline-attempt-1.json`
- `centered-baseline/`：`centered-baseline-attempt-1.log`、`centered-baseline-attempt-1.json`
- 两目录均保留`surface_oriented.rs`、`trace.diff`、`build.sh`、`build.jsonl`、`build.log`、`build.exit`、`source-before-build.sha256`、`source-after-build.check`、`binary-identity.json`、复制的`nuwax-nav-stack-test`及`run-focused.py`

第一份构建UTC为18:53:41—18:54:30，第二份18:58:48—18:59:04；两份`build.exit=0`，七个记录源码均通过构建后摘要核对。两次使用同一固定混合test profile：MLS planner及nav-runtime opt-level=3，stack及其它crate为普通test profile；stack元数据opt-level=0、debug=0、debug assertions及overflow checks开启。`body-query-profile`开启；这不是全release性能。

在指定checkout的`services/nuwax-workspace`执行，原脚本先加载`/workspace/shared/neh-native-20261006/project-env.sh`，保持既有SDK/target，Cargo jobs=2、CMake jobs=1：

```sh
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
cd "$REPO/services/nuwax-workspace"
cargo test --locked --offline \
 --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
 --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
 -p nuwax-nav-stack --features nuwax-nav-stack/mls,nuwax-nav-runtime/body-query-profile \
 --lib --no-run --message-format=json
```

实际精确执行命令分别为：

```sh
/workspace/shared/neh-oriented-guide-20261006/baseline/nuwax-nav-stack-test surface_oriented::tests::calibrated_body_plans_and_closes_wall_bounded_ninety_degree_turn --exact --nocapture --test-threads=1
/workspace/shared/neh-oriented-guide-20261006/centered-baseline/nuwax-nav-stack-test surface_oriented::tests::calibrated_body_plans_and_closes_wall_bounded_ninety_degree_turn --exact --nocapture --test-threads=1
```

| 身份（SHA256） | guide-baseline | centered-baseline |
|---|---|---|
| `surface_oriented.rs` | `675dfe1dca70b7e79e0efd2eeeeea40e7fb9bcc6f3322887cb3d1cdde53939ee` | `44fd89fe867aba0e234c9c1a3f6d25aed2c0711c837f985943a2a0936c85d47c` |
| 实际执行二进制 | `d407d05938b1a01d500a824675b86154afe4ec7052fc4d72aadeca655293b88d` | `eccb3c475aa64ea88e5f5c4d48e9207ad341139e726a7dda62d1559735a90c82` |
| 二进制字节数 | 28,214,208 | 28,221,360 |

其余六个记录源码两次相同，路径相对`services/nuwax-workspace`：

- `crates/nuwax-nav-stack/src/surface_config.rs`：`83f3fa2fc8f83f33295f539cb33fa6e0a74db7c0d02daa8c6e1ddde15c61eeea`
- `crates/nuwax-nav-runtime/src/surface_follower.rs`：`894a21d5e12723aa9413d454c14dcee6524deb6f2e7af2c1875484a600aa5862`
- `crates/nuwax-nav-mls-planner/src/surface/body.rs`：`5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305`
- `crates/nuwax-nav-stack/src/surface_chunks.rs`：`2104254ba2c5a9c51e5297a7481937bc650b8b50a42107808e88328501cd5547`
- `crates/nuwax-map-chunks/src/lib.rs`：`074ccc3910fed71e0aeea3c1da56e6db1f2eb613b7b6c24c1dbd8aa2f66b5b64`
- `crates/nuwax-codec/src/lcm/schema.rs`：`e70a4dc7cf467816b75a1f78caab0371951034749c75bc0c8a000ca675465adb`

这些摘要锁定本次记录文件，不声称是整个WIP源树的提交身份。

## 整进程资源与解释边界

下表直接来自各自attempt JSON的`wait4`记录，包含场景创建、规划、控制、独立oracle、诊断输出和teardown，不是规划阶段CPU或内存。

| 资源 | guide-baseline | centered-baseline |
|---|---:|---:|
| runner进程wall，s | 1.6155145109951263 | 1.002718414994888 |
| user CPU，s | 1.444859 | 0.882845 |
| system CPU，s | 0.107614 | 0.041877 |
| user+system CPU，s | 1.552473 | 0.924722 |
| launch peak RSS，KiB | 39,916 | 39,720 |
| 主动 / 非主动上下文切换 | 21 / 160 | 37 / 200 |

runner以100ms间隔轮询子进程完成，所报wall含启动/收尾及发现退出的轮询延迟。RSS为整个子进程启动包络峰值，不是规划器增量分配量。控制调用累计/峰值墙钟分别834.686/28.263ms和477.381/10.541ms，只记录实际值，不作稳定性或加速结论。

- 所有阶段都是inclusive墙钟，不是CPU。search、引导查询及准备检查点包含在raw中，居中子项包含在居中合计中，禁止把嵌套阶段相加。
- 诊断仅在该fixture的原`plan(...)`调用周围启用，固定两个引导记录，无锁、无哈希、无地图副本；逐位引导记录及有限路线位姿签名用于复核。各guide计时不含随后结果复制，raw包含诊断复制/记录开销；第二轮增加标量计时的成本未扣除。最终日志在规划和控制计时结束后输出。
- 两份不同诊断版本各只有一个样本，未作交替A/B；不能从其差值推导改进、均值、p95、稳定尾延迟或调度因果。旧269.967ms也不能作为本轮改进分母。
- 独立源码审查确认局部引导复用在语义上可行，但可省的单次查询仅约1—2ms；目前没有找到足够大且安全的重复工作。居中helper及第二次圆角涉及候选连接和完整机体校验，不能因耗时高或最终候选未被选中就删除；尚无证据证明这些工作可整体省略。
- 本结果限于静态地图、合成反馈及原配置。没有覆盖观察TTL、Scan、异步规划worker、MotionGuard、真实墙钟50ms消费节拍、硬件制动或真实机器人安全；`Succeeded`不是物理停止证据。

结论：保留原算法及既有安全/到达断言；本轮归因已完成，小规模重复引导不是主瓶颈。进一步优化需要先证明更大工作量能在相同输入下语义等价消除，再用冻结二进制与不变场景验证，不能用放宽目标、地图或控制阈值制造加速。
