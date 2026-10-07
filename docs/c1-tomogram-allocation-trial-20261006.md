# 层析分配削减试验资格记录

2026-10-06 UTC。结论：两处候选改动通过本轮语义检查，但没有证明可重复的性能收益，独立结果审查同意不保留为生产优化。七对候选/基线时间比的中位数分别为构建 1.024759、代价重算 1.054068，候选仅在 2/7、3/7 对中更快。单独计算两组中位数虽然较低，也不能替代配对结果。

两处生产改动已精确撤回，冻结基准保留。恢复后源码与已通过资格检查的基线逐字节一致；其他导航优化未改变。本轮没有证明统计可靠的整体变慢，也不能证明任一单独改动有害。

## 改动范围与输入

仅试验 `crates/nuwax-nav-pct/src/tomogram.rs` 的两处分配削减：

1. 新建 `Cell.neighbors` 的 `Vec::with_capacity(8)` 改为 `Vec::new()`；`connect` 自己的容量预留保留，因它随后替换初始邻接向量
2. `compute_costs` 两轮 BFS 从每次出队克隆邻接向量，改为按原下标顺序读取并复制 `usize`，再取得目标可变借用

邻接顺序、所有 checkpoint、浮点运算和冻结基准均保持原样。没有改地图语义、参数或阈值。两处作为一个组合试验，本轮不能分别量化其收益。

冻结的 `mixed_80x64` 输入包含：0.1 m 分辨率、带门洞的已知高墙、另有开口的缺测带、轻微地面高差，以及局部上下层和需要合并的上层测量带。高墙产生与地面不连通的高顶单元，并非空列阻挡掩码。输入 5,012 列、5,524 高度带；输出 5,268 单元、40,120 有向邻接边。地面路线展开 1,901 单元、输出 4 点；上层路线展开 136 单元、输出 2 点。

## 测量边界和执行资格

- 同一固定混合 test profile：Rust 1.88.0，PCT `opt-level=0`，MLS `opt-level=3`；命令也固定 runtime package `opt-level=3`。PCT `debuginfo=0`，debug assertions、overflow checks 和 `cfg(test)` cancellation 检查开启，features 为 `default, mls-adapter`。不是 release 或生产运行时测量
- 每个新进程两项各预热 4 次；构建计 32 次、已有精确地图上的 `compute_costs` 计 128 次。报告值是每次进程内的批次算术平均值，再跨七次取统计量，不是单次操作延迟的 P50/P95/P99
- 构建时间包括必需的输入深拷贝及构建期间消费输入的释放，不包括返回地图的析构；输入生成在计时外。重算复用同一地图，不重建邻接或克隆输入，最终地图析构也在计时外
- 精确输出序列化、路线查询、输出比较和可选 `create_new` 文件写入均不在上述计时内；路线的 `elapsed_ms` 不进入精确输出
- 基线构建 19:03:02–19:04:27 UTC、候选构建 19:06:05–19:06:11 UTC，均退出 0；构建前后的四份源文件 SHA-256 检查全部通过
- 候选现有功能与取消检查为 15 passed、0 failed、1 ignored；忽略项为本基准。语义日志中 `exercise RAII restoration` 是被捕获的预期 panic，该项通过
- 七组 AB/BA 交替顺序使用冻结二进制启动新进程，共 14/14 退出 0，无超时。未删除任何一组结果，没有追加试验挑选正结果

## 性能结果

B 为基线、C 为候选；单位 ms/次，C/B 小于 1 表示该对候选更快。

| 对号 | 启动顺序 | 构建 B | 构建 C | 构建 C/B | 重算 B | 重算 C | 重算 C/B |
|---|---|---:|---:|---:|---:|---:|---:|
| 1 | B→C | 54.344499 | 61.310703 | 1.128186 | 19.931121 | 21.104394 | 1.058866 |
| 2 | C→B | 63.857572 | 65.404656 | 1.024227 | 21.268012 | 22.992650 | 1.081091 |
| 3 | B→C | 64.530052 | 60.359490 | 0.935370 | 30.853672 | 30.156170 | 0.977393 |
| 4 | C→B | 71.543820 | 75.326120 | 1.052867 | 23.732220 | 26.771313 | 1.128058 |
| 5 | B→C | 54.722097 | 56.076957 | 1.024759 | 18.281642 | 19.270097 | 1.054068 |
| 6 | C→B | 70.329256 | 54.823563 | 0.779527 | 22.902251 | 19.939876 | 0.870651 |
| 7 | B→C | 72.605946 | 144.377752 | 1.988511 | 24.475079 | 21.918884 | 0.895559 |

| 汇总方式 | 构建 | 代价重算 |
|---|---:|---:|
| 基线七次批次均值的中位数 | 64.530052 ms | 22.902251 ms |
| 候选七次批次均值的中位数 | 61.310703 ms | 21.918884 ms |
| 两组中位数之比 C/B | 0.950111 | 0.957062 |
| 七个配对比值 C/B 的中位数 | 1.024759 | 1.054068 |
| 候选更快的对数 | 2/7 | 3/7 |

配对第 7 组构建从 72.605946 ms 到 144.377752 ms（+98.85%），完整保留。配对前的单次首次运行也完整保留：构建 55.118466→65.342024 ms（+18.55%）；重算 19.362772→22.836549 ms（+17.94%）。这些首次运行不并入七对统计。没有证据将差异归因于边界检查、调度或其他机制；本轮结论是收益未获证明，不是已确证某种性能回退原因。

14 个配对进程记录的 `launch_peak_rss_kib` 均为 19,584 KiB，未显示进程 RSS 节省。首次独立运行的 15,476/15,440 KiB 也不足以证明内存收益。进程 CPU/墙钟包含输入夹具、预热、测试框架、序列化与路线查询等，不能替代目标阶段计时或作为分配收益证据；本轮没有分配器计数。

## 精确一致性与来源

基线、候选的精确输出先逐字节比较一致，再运行七组配对；文件各 1,417,479 字节，SHA-256 均为：

`774a0715e6f7d94ac5a04b3b790e542ee489551464e1e6eb246afe29b90b18b5`

输出保留单元顺序、column/layer、ground/interval/ceiling/两种 clearance/cost 的浮点位、邻接顺序、同列索引、排序后的 known-column 集合及两条路线点位和工作计数。每次基准另验证重复重算前后该完整输出一致；配对运行不再写文件。

证据根目录 `Q=/workspace/shared/neh-tomogram-allocation-20261006`。SHA-256：

| 冻结对象 | SHA-256 |
|---|---|
| 原生产及原测试 `tomogram.before.rs` | `9622d3843170b04a8b4e4c16be974abfbabc14a6073a721ae76a0ef11a50a8dd` |
| 加冻结基准的基线 `tomogram.test-only.rs` | `448aa791f764a812dac4c8c88804cbc44c92a5ed97acfd7a2183c063c64bc899` |
| 组合候选 `tomogram.candidate.rs` | `9d277a9d32e18950ab8b214e41184930fe0943d41e4086d132f170589e7cb4f8` |
| 基线 `baseline/nuwax-nav-pct-test` | `834e1a6398d1fbd49651b2dc7200c85b61130c075ee0adb0b69f15eb0ea284bd` |
| 候选 `candidate/nuwax-nav-pct-test` | `768a5999579632620a11ee50fb8b0771d4fa421ee405a6ed0465f4d0339140ce` |
| 配对汇总 `matched/results.json` | `726531f9af1f91935064a9655490282297cdf3ca8525f8f68a2b9ef406770354` |

共同未变源文件相对 `services/nuwax-workspace` 的路径及 SHA-256：

```text
b8d4f869e4b2b7ec8506bed6f6bc462dd2d99572708fea3594ab7c78a7f50f08  crates/nuwax-nav-pct/src/search.rs
1dcda1cb776f6d8d99f0d24d01b554bc5435afcf74bff1ba4792953c6ef6b340  crates/nuwax-nav-pct/src/cancellation.rs
5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305  crates/nuwax-nav-mls-planner/src/surface/body.rs
```

基线、候选各自的 `build.sh` 记录同一构建命令；环境脚本固定 SDK/target 缓存、debug=0、Cargo jobs=2，并清除外来 RUSTFLAGS/wrapper：

```bash
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
cd "$REPO/services/nuwax-workspace"
cargo test --locked --offline \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  -p nuwax-nav-pct --lib --no-run --message-format=json
```

实际基准命令，其中 `stage` 为 `baseline` 或 `candidate`，首次运行另设 `NEH_TOMOGRAM_ALLOC_DUMP="$Q/$stage/exact-output.txt"`，配对时移除此变量：

```bash
"$Q/$stage/nuwax-nav-pct-test" \
  tomogram::allocation_benchmark_tests::tomogram_allocation_frozen_baseline \
  --exact --ignored --nocapture --test-threads=1
```

`Q` 保留 `candidate.diff`、基准模块与 test-only patch、两套 source/build/binary identity、首次运行日志/JSON/精确输出、`candidate/focused-regressions.log`、`compare.py`、14 份配对原始日志及 `matched/results.json`。这些是执行环境中的资格证据；本文不声称已上传 GitHub 或在用户设备上验证。

## 已验证的撤回

独立结果审查同意“配对证据未证明收益，拒绝组合候选”。恢复前先核对完整候选源文件哈希，再仅撤回两处生产差异；恢复后的 `tomogram.rs` 与冻结、已资格检查的 `tomogram.test-only.rs` 逐字节一致，SHA-256 为 `448aa791f764a812dac4c8c88804cbc44c92a5ed97acfd7a2183c063c64bc899`。已有测试与冻结基准保留，没有回滚其他文件或此前优化。

恢复后没有冗余构建或重跑；依据是精确恢复至本轮已验证的基线源码及其冻结二进制。保留完整正负结果，不为这次未证明的收益扩大参数矩阵或改用 release 重测。本阶段没有 Git 提交或推送。
