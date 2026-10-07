# 同一虚拟遮罩跨占据修订复用

2026-10-07 UTC，基线 `537ed937b02541abdb7a61ca703963d872e147bc`。
本轮只消除物理源和完整虚拟遮罩都未变化时的重复派生，不实现全局 PCT 增量算法。
95 个不同的非网络测试通过；真实 LCM 全局拥有方联验在传输初始化处受阻，不能宣称通过。
本报告对应未发布候选，提交/推送由上层统一执行。

## 改动及权限边界

原 `SurfaceMapCache` 把占据修订作为虚拟层的保守失效条件。即使物理 Arc 和遮罩完全相同，
较新占据提交仍会复制派生图、重建区域并生成新的 `SurfaceData`；这进一步使全局/局部 PCT
失效，并取消局部候选。生产端支持保留旧云、推进占据回执且虚拟区不变的组合发布，因此该
路径不只是测试构造。真实部署中它出现的频率尚未测量。

生产改动限于 `nuwax-nav-mls-planner/src/surface.rs` 和
`nuwax-nav-stack/src/surface_map.rs`：

- 保留同修订快路；额外比较只在相同物理 Arc、同域和启动 nonce、严格较新占据修订时执行
- 保存固定大小布局标量，比较分辨率、尺寸和完整原点位置/四元数位模式
- MLS 层再次直接核对保留遮罩的 XY、分辨率、尺寸；以已有积分图的单格精确和逐格核对
  输入 0/100 值，无内容哈希、第二份遮罩副本或新增全图索引
- 单次比较最坏 O(遮罩格数)，额外工作内存 O(1)，不匹配立即走原构造/失败路径
- Schema、精细块冲突及观察装配仍在复用判定前执行；不能跳过构造器的尺寸/有限范围约束
- 无遮罩与有限范围的全空闲遮罩保持不同；新增/删除/变化、清空/reset/换域不能继承该基线
- 复用几何仍挂接当前观察。当前 authority、源时间、接收时间和 `PlannerWorker.map_version`
  继续独立更新；缓存不是命令许可，不延长观察寿命或旧命令期限

精细占据在同一粗格内变化时，`observations.same_geometry` 仍使 PCT 失效。当前机体、边界、
新鲜度、制动和指令交付规则均未改变。新增拥有方/命令用例只增加测试代码。

## 先核对工作量，再计时

1200 格确定性用例中，初始 MLS 派生和 PCT 各构造一次：

- 8 次同修订安装：额外单格比较 0 次
- 8 次较新且相同遮罩：单格比较 9600 次，新增 MLS 派生 0 次，新增 PCT 构造 0 次
- 随后首格改变：1 次比较发现差异，按原流程新增派生及 PCT

计数是 `cfg(test)` 的实际迭代/构造分支统计；完整 `build` 原有校验仍会遍历输入，不能将
“同修订额外比较为零”解读成整个入口恒定时间。

## 验证

95 个不同的非网络测试通过，包括：

- 6 个真实 `SurfaceMapCache` 独立冷构造对照，比较当前消息的表面/高度带、PCT 字段和完整路线
- 积分图逐格比较、所有位置翻转、相同阻断数的移动、布局/符号零、错误迭代长度及构造器限制
- 真实 fine Writer/Receiver：占据回执更新不刷新旧观察；新观察替换机体查询；粗格不变的精细孔洞使 PCT 重建
- 全局与局部 PCT 全字段、支撑/墙距、邻接顺序、已知列及双层绕行/跨层拒绝的精确冷对照
- 真实局部 PlanningOwner 的受控屏障：相同遮罩延续，变化遮罩/精细源/新拥有方取消旧候选
- 同一 SurfaceSnapshot Arc 再安装仍退休 `PlannerWorker` 的在途/已存命令；完整 worker 模块 32 通过、2 个既有手动用例忽略
- 完整 PCT 模块 25 通过、3 个既有手动用例忽略；其余观察过期、缓存隔离、回执顺序和失败恢复检查通过

真实 LCM 全局拥有方测试保留失败结果：`LcmTransport::new_real` 返回
`connect: Network is unreachable`，无法建立组播路由，尚未进入被测逻辑。没有更改网络设置、
切换传输冒充通过或绕过既有 MotionGuard AF_UNIX 限制。局部组合验证足以限定本次几何复用，
不能扩展为端到端传输/运动验收。

首次编译出现新测试重复包装 `Arc` 的三个类型错误，修正的是同一处测试夹具，失败日志保留。
后续四 crate 编译和最终 stack 编译成功。独立设计/代码审查均通过；测试审查发现的夹具 PCT
拥有方意外共享已恢复为原有两份拥有方，再重新构建最终 stack 二进制。

## 配对测量

先冻结门槛和夹具，再执行 3 组交替进程对。基线恢复两个原生产文件，只编译 stack 测试目标；
省略依赖新测试接口的 `mask_work_tests`，保留与候选逐字节相同的基准及冷对照文件。
只运行指定手动基准，不在基线上执行候选行为断言。两份二进制分别冻结，构建后已精确恢复候选源码。
278 项依赖/feature/profile 记录完全一致。

固定 80×60、0.1m 地面和 0.30m 方形机体；保留云修订，每臂 3 次预热、16 次计量更新。
相同遮罩臂与真实变化的路线外遮罩臂均保留完整路线；消息准备/克隆及独立冷对照在计时外。
以下是各臂 3 个进程共 48 次更新的合并中位数，样本并非 48 个独立进程：

| 每次更新，ms | 相同遮罩基线 | 相同遮罩候选 | 变化遮罩基线 | 变化遮罩候选 |
| --- | ---: | ---: | ---: | ---: |
| SurfaceMapCache::build | 8.109 | 2.043 | 7.103 | 7.096 |
| PctPlanner::map | 51.210 | 0.0039 | 43.660 | 42.176 |
| 同次两阶段相加 | 58.719 | 2.047 | 50.613 | 49.927 |

不能用两个阶段各自的中位数相加代替最后一行。同一进程的相同遮罩合计中位数，基线为
67.325/56.147/56.162ms，候选为 1.709/1.704/5.881ms，存在明显进程间波动。
相同遮罩每进程 MLS/PCT 重建次数从 16 降至 0；变化遮罩两边仍为 16，未把必要工作隐藏成复用。
变化臂的微小计时差不归因为优化。六个进程均通过，两臂全部完整 body/PCT 路线位模式、点数、
长度和终点误差一致。
真实变化最坏会先付出一次完整遮罩比较再构造；本变化对照的首个差异在路线外局部块，
不是“直到最后一格才发现变化”的最坏时延测量，不能据此宣称所有变化输入均无开销回退。

这仍是 MLS/runtime opt-level=3、PCT/stack opt-level=0 的混合 test profile，非 release 或实机。
阶段值是墙钟；整进程 CPU/RSS 包含两臂、夹具、冷构造、断言、独立对照和销毁，不能当作规划
阶段 CPU/内存。RSS 有启动包络下限，本轮不作内存节省结论。不宣称普遍导航提速、20Hz 或硬件资格。

## 证据及重现

完整数值/命令/身份见同目录 `c1-unchanged-mask-reuse-20261007-results.json`。
原始日志、冻结二进制、前后源码、构建及运行脚本保存在
`/workspace/shared/neh-mask-reuse-20261007/`。主要子目录：`baseline/`、`candidate/`、`matched/`。
`GATES.txt` 记录计时前门槛；`candidate/attempt-1/` 保留初次编译错误及网络阻塞；
`candidate/final-stack-focused/13.log` 保留最终二进制的网络联验阻塞。

使用既有 SDK 环境，Cargo jobs=2、CMake jobs=1，测试目标：

```sh
cargo test --locked --offline -p nuwax-nav-stack -p nuwax-nav-runtime \
  -p nuwax-nav-mls-planner -p nuwax-nav-pct --features nuwax-nav-stack/mls \
  --lib --no-run --message-format=json \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3'
```

手动选择器是
`lcm_dataflow::surface_dataflow::tests::mask_reuse_tests::calibrated_repeated_occupancy_mask_reuse_cost_baseline`，
加 `--exact --ignored --nocapture --test-threads=1`；`run-matched.py` 保存交替顺序和全部样本。
真实几何变化时完整/局部窗口 PCT 重建、全局 mid-build AtomicBool 取消及 quick 零预算仍有原限制，
本补丁没有以较窄路径改变这些语义。
