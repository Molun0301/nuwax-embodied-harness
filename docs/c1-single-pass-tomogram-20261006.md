# 禁区层析构建单次代价计算资格记录

2026-10-06 UTC。四个固定场景中，候选在七次新进程内的冻结旧版/当前版配对均更快（各 7/7）；17 项功能测试通过，0 失败，2 项手动基准默认忽略。所有计时进程的同快照精确断言通过。独立结果审查核对了七份原始日志、28 个场景记录、源码/参考/二进制/profile 身份及汇总算术，同意保留这项适配器优化。

## 修改与覆盖

生产差异仅在 `services/nuwax-workspace/crates/nuwax-nav-pct/src/tomogram.rs::from_surface_window`：保留源窗口、`take_while` 取消检查及逐列禁区判定；拒绝列用 `bands.clear()` 清空高度带，仍把原列交给构造器登记为已知。移除临时 blocked 集合、构造后的已知列补登记和第二轮 `compute_costs`。构造参数与最终 checkpoint 未改。

由源码流程可知，含拒绝列且成功构建的地图从两轮代价计算变成一轮，临时禁区集合也被移除；本轮未采集分配器计数，不能给出分配次数或内存节省量。此前被否决的邻接初始容量/BFS 分配试验没有重新引入。

测试在生产修改前逐字节冻结旧包装函数，放入独立 trait 实现；新旧共用未改变的底层构造器，不共用新的空高度带转换。一个 40×30 精细网格房间同时包含上下重叠楼层、范围较小的 36×26 掩码、可从末端绕行的虚拟线及真正缺测条带。只取粗源回退与完整精细源各一个代表输入，分别调用真实公开全局和裁剪局部构建路径：

| 场景 | 分辨率 | 输入及已知列 | 输出单元 |
|---|---:|---:|---:|
| 粗源全局 | f64::from(0.1_f32) | 298 | 448 |
| 粗源局部 | f64::from(0.1_f32) | 238 | 396 |
| 精细源全局 | f64::from(0.05_f32) | 1,192 | 1,792 |
| 精细源局部 | f64::from(0.05_f32) | 952 | 1,584 |

断言明确检查：禁区列和掩码范围外列均已知但无单元；真正缺测列仍未知；局部确实减少源列和单元；上下楼层都可绕行、不可跨层连接；全阻断返回同一错误。精确比较所有地图标量浮点位、按原顺序存储的单元/邻接/同列索引、known-columns 集合及路线点位、expanded/cells 工作计数，排除路线 elapsed_ms。没有排序单元、邻接或路线。

公开入口的预取消确实执行过；中途 AtomicU64 取消由与候选相同的列转换模式和真实构造器确定性触发，第 8 个拒绝列后停止。后者是构造器模式验证，不宣称覆盖公开入口中途取消的时序竞争。原有 15 项回归继续覆盖证据/cache、缺测、台阶、层间、边界及搜索/连接/代价阶段取消。

## 执行与计时边界

- Rust 1.88.0；固定混合 test profile：PCT opt-level=0，MLS opt-level=3，命令也固定 runtime package opt-level=3。PCT debuginfo=0，debug assertions、overflow checks、cfg(test) cancellation 检查开启；features 为 default、mls-adapter。不是 release 或生产机器人测量
- 修改前先真实构建、运行两项新增功能测试及四场景忽略基准，随后才编辑生产函数。基线构建 19:23:40–19:23:47 UTC，候选构建 19:26:37–19:26:43 UTC，均退出 0；五个源文件的构建前后哈希检查全部通过
- 正式重复为七个新候选进程。每个进程内，各场景固定 4 对预热、32 对计时，轮换先运行的版本；新旧始终针对同一个不可变 SurfaceSnapshot。哈希源容器的迭代顺序可能跨进程变化，因此不做跨进程存储顺序等价判断
- 源准备、路线/精确检查、序列化均在计时外；返回地图析构也在计时外，构建时消费输入的清理仍包含。精确断言在计时前后执行
- 每个记录是 32 次操作的算术均值；以下中位数是七个批次均值的中位数，不是单次操作 P50/P95/P99。七次全部退出 0、无超时，保留所有结果
- 测量范围是未缓存的含禁区地图构建，不能换算为每个控制 tick 的收益，也没有证明探索吞吐、控制期限或实机导航改善

## 修改前控制与首次候选

单位 ms/次。前两列来自实际生产修改前的同进程运行，当时旧参考与当前实现相同，用于检查测试基线；它们不是跨进程性能配对。首次候选资格运行完整保留，未并入后面的七次统计。

| 场景 | 修改前参考 | 修改前当前 | 首次候选内参考 | 首次候选当前 |
|---|---:|---:|---:|---:|
| 粗源全局 | 7.393390 | 7.423834 | 6.949856 | 5.278395 |
| 粗源局部 | 6.412286 | 6.642125 | 5.451779 | 4.605131 |
| 精细源全局 | 27.490081 | 27.559511 | 26.301852 | 19.901340 |
| 精细源局部 | 24.748348 | 23.796054 | 25.807990 | 19.039653 |

## 七次重复结果

R 为冻结旧包装函数，C 为当前候选。组中位数比为 median(C)/median(R)；配对比中位数为 median(Cᵢ/Rᵢ)，两者分别列出。

| 场景 | R 中位数 ms | C 中位数 ms | 组中位数比 C/R | 配对比中位数 | C 更快 |
|---|---:|---:|---:|---:|---:|
| 粗源全局 | 6.936785 | 5.507482 | 0.793953 | 0.796441 | 7/7 |
| 粗源局部 | 6.219577 | 4.818001 | 0.774651 | 0.747145 | 7/7 |
| 精细源全局 | 29.609607 | 22.084931 | 0.745870 | 0.762773 | 7/7 |
| 精细源局部 | 25.739872 | 19.415334 | 0.754290 | 0.774468 | 7/7 |

按配对比中位数计算，粗源全局/局部分别减少 20.36%/25.29%，精细源全局/局部分别减少 23.72%/22.55%；这些百分比不取两组中位数之比。

全部批次均值与配对比范围：

| 场景 | R 最小–最大 ms | C 最小–最大 ms | 配对比最小–最大 |
|---|---:|---:|---:|
| 粗源全局 | 6.332819–8.407306 | 4.882629–6.695923 | 0.718897–0.815128 |
| 粗源局部 | 6.024782–8.655203 | 4.553206–5.751146 | 0.664473–0.844548 |
| 精细源全局 | 27.408088–36.165406 | 20.906136–29.008691 | 0.696353–0.802112 |
| 精细源局部 | 23.031930–31.920115 | 18.295876–25.944555 | 0.729542–0.832634 |

逐进程配对比 C/R（不是逐操作比值）：

| 新进程 | 粗源全局 | 粗源局部 | 精细源全局 | 精细源局部 |
|---|---:|---:|---:|---:|
| 1 | 0.722832 | 0.780323 | 0.762773 | 0.774468 |
| 2 | 0.718897 | 0.747145 | 0.735939 | 0.832634 |
| 3 | 0.809627 | 0.744188 | 0.801998 | 0.754290 |
| 4 | 0.802815 | 0.800932 | 0.767849 | 0.752903 |
| 5 | 0.796441 | 0.664473 | 0.802112 | 0.729542 |
| 6 | 0.795954 | 0.844548 | 0.696353 | 0.812796 |
| 7 | 0.815128 | 0.715174 | 0.745870 | 0.791435 |

七个进程的 launch_peak_rss_kib 均为 17,280 KiB；每个进程同时运行两种实现并包含夹具、预热和检查，CPU/墙钟/RSS 均无法归因到某一种实现。不能据此声称 RSS 或 CPU 资源节省；时间结论仅使用各自构建计时区域。

## 可追溯来源与复现命令

证据根目录 `Q=/workspace/shared/neh-single-pass-tomogram-20261006`。`baseline/`、`candidate/` 分别保存源码、构建命令/日志/退出值/哈希检查、冻结二进制及首次运行结果；`matched/01.log` 至 `07.log` 为正式重复原始日志，`matched/results.json` 保存总纳秒、均值、比值、全进程资源和日志哈希，`repeat.py` 保存汇总算法。

| 冻结对象 | SHA-256 |
|---|---|
| 新增测试前 `tomogram.rs` | `448aa791f764a812dac4c8c88804cbc44c92a5ed97acfd7a2183c063c64bc899` |
| 冻结旧包装函数，2,199 字节 | `a76a78f4c559a9872c109b75fed57f6e8505454eb2d977b08afc57df84160815` |
| 修改前基线 `tomogram.rs` | `cbdfff78235015b8fa3f1ca6b16be972c3222ba4443c175238e58adb6bfaf40c` |
| 候选 `tomogram.rs` | `a0ecb96b0a4b77e4a7c03d953a90483b702c6bdcdaef781bded063ec5711acdf` |
| 共同冻结测试模块 | `1ce5f0874db78fc5c936018e1bfa0b840543343220656d53e068a2574603409b` |
| 生产差异 `candidate.diff` | `e6bed9af12d78245f6265f106c7c06c04aae3e28fcebb7233d025f199e3ed82d` |
| 基线测试二进制 | `6b1bda993443c4bfd76e4191c66ac1a7ccda14dbf41373ce5d0914231ce9cfb0` |
| 候选测试二进制 | `e6f16bbe54bf9db1089e9adc7c91a801c200fe268e1b1fc73f7238f6b6c4f212` |
| 七次完整结果 `matched/results.json` | `503cab6c49a196e20fa26228efcfe10c6ae63390057340efd72a8ad5c89399dc` |

两版本共同未改源文件（相对 `services/nuwax-workspace`）：

```text
b8d4f869e4b2b7ec8506bed6f6bc462dd2d99572708fea3594ab7c78a7f50f08  crates/nuwax-nav-pct/src/search.rs
1dcda1cb776f6d8d99f0d24d01b554bc5435afcf74bff1ba4792953c6ef6b340  crates/nuwax-nav-pct/src/cancellation.rs
5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305  crates/nuwax-nav-mls-planner/src/surface/body.rs
```

两次实际构建命令相同。环境脚本固定 SDK/target、debug=0、Cargo jobs=2，并清除外来 RUSTFLAGS/wrapper：

```bash
source /workspace/shared/neh-native-20261006/project-env.sh
export REPO=/workspace/scratch/71cdb9d17732/neh-assigned-workspace-20261006
cd "$REPO/services/nuwax-workspace"
cargo test --locked --offline \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  -p nuwax-nav-pct --lib --no-run --message-format=json
```

实际执行命令：

```bash
Q=/workspace/shared/neh-single-pass-tomogram-20261006
# 修改前的两项新增功能测试：2 passed
"$Q/baseline/nuwax-nav-pct-test" tomogram::masked_surface_tests:: \
  --skip masked_surface_alternating_baseline --nocapture --test-threads=1
# 候选完整 PCT 单元测试：17 passed、0 failed、2 ignored
"$Q/candidate/nuwax-nav-pct-test" --nocapture --test-threads=1
# stage=baseline 为修改前控制；stage=candidate 为首次资格及七次新进程重复
"$Q/$stage/nuwax-nav-pct-test" \
  tomogram::masked_surface_tests::masked_surface_alternating_baseline \
  --exact --ignored --nocapture --test-threads=1
```

两项默认忽略测试分别是此前的分配试验基准和本轮单次代价基准；本轮只对后者单独显式运行。功能日志中的 exercise RAII restoration 是被捕获的预期 panic，该测试通过。此记录未把 focused PCT 单元测试扩大成全 workspace、探索或硬件验证，也不声称已提交、推送或部署。
