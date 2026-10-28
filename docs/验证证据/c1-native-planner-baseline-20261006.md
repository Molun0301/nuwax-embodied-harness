# 当前导航/通行性原生基线（存在实时性失败）

2026-10-06 UTC。真实远端基线 `410290dbc45c20db10bf2782556b555a0175bcc4`，
源树 `1fabe2eac417c4fecde21552148f22b2b93dbe72`，本地同树 `2bf54cc108a63dfa728a486fefd4de8ce8331620`。
本文件记录改动前的保留二进制，不把后续候选或历史文档结果混入基线。

## 环境与范围

Rust/Cargo 1.88，x86_64，SDK 已存在。Cargo jobs=2、CMake jobs=1；两项离线 CMake
预检均使用已安装 GTSAM/Metis/Cephes，无依赖回退或 SDK 重建。缺失的 locked proptest
缓存经官方 registry 补齐，registry 增加 17,616 KiB，锁文件未改。初次编译 5m48s。

统一 **混合 test profile**：MLS 与 runtime opt-level=3，其它 Rust crate 使用普通 test
profile，debug=0，启用 body-query-profile。这不是 release 性能测试。四个 emitted test
binary 已逐一复制、记录摘要与实际 --list；构建及运行均串行使用同一 native lane。

## 实际结果

十二组终态完整：**32 通过、3 失败、2 个明确未运行的观察基准**，无超时或缺失摘要。

- 6 个 body passage、9 个观察、2 个 ON/OFF 障碍/支持/新鲜度、1 个阻塞地图替换和
  10 个 dispatch 拥有方/过期/交接用例通过
- 实际 chunk 仓储→surface→定向路线→SurfaceFollower 窄门组合通过：32cm 机体，
  33cm 正例/32cm 接触负例，含对齐/5°/偏移；进程 wall 36.095s、CPU 36.089s
- Scan 33cm 正例/30cm 负例通过：wall 26.319s、CPU 26.269s
- 实际双帧新鲜源与真实等待后过期负例、捕获入口重复命令诊断均通过

三个 moving 用例均在 ≤75ms 消费节拍断言失败：

| 用例 | 行为结果 | consumer gap 中位/p95/max ms | >75ms | control step 中位/p95/max ms |
|---|---|---|---:|---|
| local OFF | 到达；位置误差0.063733m、yaw误差0.131177rad | 50.000/51.921/172.463 | 8 | 1.646/8.040/102.798 |
| local ON | 到达；位置误差0.143084m、yaw误差0.131417rad | 50.001/53.283/238.452 | 9 | 1.761/7.203/38.601 |
| held planning | 连续性完成，6个有效held命令、2次真实新规划采用；未测全程到达 | 49.999/52.299/217.990 | 6 | 2.134/13.517/179.050 |

OFF/ON priming 后分别23.204/22.683s到达；实际位移0.936267/0.856916m。held用例
16.157s连续性完成、位移1.067603m、剩余目标5.932397m。各自进程 CPU 为
3.008/2.599/3.102s，launch-envelope peak RSS 为70,292/70,244/81,688 KiB。

断言次序已独立核对：执行段检查、receipt时刻完整制动 oracle、到达/连续性断言在节拍失败
之前均通过；后续 map/control ≤50ms 断言未执行。日志仍证明 OFF 与 held 的 control 最大值
超50ms；map安装最大值仅0.115/0.481/0.406ms。不能称整体实时资格通过。

891个非零receipt均有准确 trace，缺失/worker→guard基础不匹配/非法时钟均为0；
guard→receipt精确状态不匹配为221/213/143，不能声称receipt状态与原guard完全一致。
独立receipt oracle在这些场景通过，不意味着这些不匹配已证明碰撞，也不证明任意实机安全。

## 冷接触域与调度归因

初次捕获入口 whole step145.451ms，body motion126.003ms，contact-domain91.613ms，
其中BFS45.630ms。**whole warm step是1.021/1.765ms**；0.160/0.154ms仅为嵌套body计时。

同一保留二进制再做三次新进程/新拥有方：cold whole step50.610/58.358/48.339ms，
work counters完全一致（43个domain、165,560个点、179,076个访问列）。因此不能把初次
145ms当作稳定基线，候选必须与原二进制交替比较。锁获取仅约0.002ms，不支持锁竞争热点结论。

不改源码的ON调度采样重复仍失败（max gap138.185ms、7次>75ms），control max46.039ms。
100ms外部采样看到了consumer/test线程累计221.227ms runqueue wait，以及单采样区间
79.018/85.024ms等待增长；compute线程累计41.833ms。采样自身间隔也可达231.943ms，
缺少每tick关联ID，cpu.stat/cpu.max/pressure不可读，所以只能证明存在调度延迟贡献，不能
把全部失败归因于环境或声称已证明限流。完整raw证据单独保留。

## 解释边界

- 窄门组合使用simulation=true与合成50ms时间，无观察TTL；moving使用真实墙钟与1s TTL，
  但为0.30m方形通用机体、平面1m目标、理想命令积分，不是实机执行/执行器制动
- 查询stage是inclusive wall，不可相加或叫CPU；进程CPU/RSS包括fixture、保留快照、日志、
  teardown与独立oracle。小用例38,476KiB RSS启动下限也限制解释
- dense payload byte counter是累计申请量，非峰值内存/allocator总量/全图复制量
- 没有路径最优性、曲率、平滑度或全生产入口动态障碍 churn 的完整量化证明
- 历史文档的更好时延缺少本环境原日志/二进制，不能当作本次前后对照

下一步仅针对接触域做小型语义等价优化、固定二进制交替测量，并保持原节拍/新鲜度/障碍断言。
候选结果必须单独记录，不能改写或掩盖这份失败基线。
