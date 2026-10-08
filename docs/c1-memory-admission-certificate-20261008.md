# 语义记忆首轮词法准入优化（2026-10-08）

## 改动与适用范围

基线为 `246dd66857f52377af4b9e0bda73d8a00a78fbde`。仅对 memory 源、已有有效非空向量分数的首轮候选计算作优化。对同一组已采样非词法分量，组合评分关于词法分量单调；词法取0和1的最终浮点分数完全相等时，该区间内最终分数相同，因此只需判断原词法准入条件，不必找出所有文本的最大词法分数。合格最终分数不超过1，使准入阈值不超过0.5，后续精确文本的0.99返回不会撤销此前已成立的准入。端点不等则执行原完整词法评分。

原始文本列表仍按原顺序即时构造。其他源类型、词法-only路径、堆与去重、candidate_count、源校验和原2×limit批次保持；幸存者二阶段重新完整计算分数和理由。没有新增缓存、容量策略、哈希、锁、重试或校验规则。

### 时间和异常顺序边界

本优化把相关 memory 行的图像余弦、空间和新近度采样移到耗时词法扫描之前。等价结论针对相同已采样分量；跨300／3600／86400／604800秒新近度分桶边界或异常图像提供者时，时间／异常先后顺序可能不同。固定时钟差分不能证明任意实时交错都等价。未为消除此边界增加采样、锁或补偿逻辑。

## 正确性资格

实际 CPython 3.8.20；部署目标3.8.10未实跑。新增 `scripts/check_memory_admission_certificate.py` 对两个完整源文件使用相同真实 SQLite 夹具，比较293组成对完整结果。仅移除结果／匹配／导航候选自动生成时间戳，保留源与Pose时间、身份、来源、就绪信息、分数、理由和候选计数。覆盖文本／图像／混合／停用／繁忙、强弱零负向量、阈值邻接浮点、晚到精确匹配、混合源、并列去重、过滤、live-only、陈旧源、无效向量实际修复，以及两阶段之间真实持久源变化。异常修复用例在两臂间恢复同一夹具状态。

现有7个脚本通过：service 37、provider modes 10、provider selection 19、payload 4、durable 21、hash-free 16，共107项unittest，另 recall 脚本全部断言通过。新测试初期夹具错误及其原始日志保留；未放宽旧断言。

重现差分时，在已安装项目依赖的可信环境导出上述基线的 `services/memory/service.py`，再执行：

```sh
git show 246dd66857f52377af4b9e0bda73d8a00a78fbde:services/memory/service.py > /tmp/neh-memory-baseline.py
python scripts/check_memory_admission_certificate.py --baseline /tmp/neh-memory-baseline.py --candidate services/memory/service.py
```

两参数会加载执行指定Python源码，仅使用可信的项目源文件。这里的临时基线可从Git随时重建，不承担备份用途。

## 一次冻结性能实验

以下为完整测量报告。仅固定5000条强语义合成负载；不能外推到生产、全部模式、Go2导航CPU或弱语义路径。此前长度上界剪枝试验仍为撤回状态。

# One-shot semantic-memory admission measurement

## Outcome

All predeclared A/A, A/B, real-call reduction, full-output, cache-lifecycle, durable-aggregate and byte-provenance gates passed. This is a fixture-level benefit; normal exit status alone was not used as evidence of passing.

Exactly one run, one 5,000-row SQLite FULL setup, and 22 full public semantic queries. Public defaults: similarity threshold 0.25, limit 5. Actual CPython 3.8.20; target 3.8.10 remains unrun. Default GC, process-local numerical threads 1, no profiling or timing instrumentation during actual-count queries. No retries or sample expansion.

## Gate results

| Stage | Metric | A median s | B median s | Paired median gain | Ratio-of-medians gain | A relative MAD | B relative MAD | B faster pairs |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| AA | cpu_s | 2.611520400000 | 2.663613703000 | -0.660800% | -1.994750% | 0.348945% | 3.095186% | 2/4 |
| AA | wall_s | 2.611716873500 | 2.664422774003 | -0.687739% | -2.018056% | 0.348388% | 3.114728% | 2/4 |
| AB | cpu_s | 2.619109890500 | 2.070279719000 | 21.099845% | 20.954836% | 1.030528% | 1.775582% | 4/4 |
| AB | wall_s | 2.620224868500 | 2.070466476001 | 21.096084% | 20.981344% | 0.991772% | 1.776880% | 4/4 |

A/A gates: absolute paired-median bias <=5%, each label MAD <=10%. A/B thresholds were both 5% (`max(5%, 2 × A/A absolute bias)`); both metrics beat them with 4/4 winning pairs and MAD <=10%. The unfavorable second A/A pair is retained below; no outlier was removed.

Actual SequenceMatcher constructor/ratio calls: baseline 9,998/9,998; candidate 5,512/5,512, a 44.868974% reduction. Counts came from two separate untimed full public queries, with their outputs still checked.

## Every measured raw pair

Seconds; labels are semantic baseline/candidate in A/B, identical baseline in A/A. Actual counterbalanced execution order is recorded. No samples omitted.

| Stage/pair | Order | A wall | A CPU | B wall | B CPU |
|---|---|---:|---:|---:|---:|
| AA/0 | AB | 2.6741327679992537 | 2.674032539999999 | 2.6691938290023245 | 2.668039227999998 |
| AA/1 | BA | 2.6005962089984678 | 2.600484438999999 | 2.8455529070051853 | 2.8453931499999996 |
| AA/2 | AB | 2.6046397099999012 | 2.6043308240000016 | 2.5032147689998965 | 2.5031516450000026 |
| AA/3 | BA | 2.6187940370000433 | 2.618709975999998 | 2.6596517190046143 | 2.6591881779999937 |
| AB/0 | AB | 2.600541826999688 | 2.5983714749999933 | 2.176253211997391 | 2.1760201400000057 |
| AB/1 | BA | 2.6399079100010567 | 2.6398483060000046 | 2.077136113999586 | 2.0769666750000084 |
| AB/2 | AB | 2.6525151469977573 | 2.6523527720000004 | 2.063796838003327 | 2.0635927630000026 |
| AB/3 | BA | 2.5321191260009073 | 2.532073453999999 | 2.0035566950027714 | 2.003447652999995 |

## Exactness and durable evidence

- All 22 complete canonical results and final-score hex values equal; candidate_count is 4,912 and returned_candidate_count is 5 throughout
- Canonicalization exactly reuses qualification’s reviewed function: removes only generated result, match and navigation-candidate timestamps; preserves source/Pose timestamps, record identities, all nested provenance/readiness/filter metadata and scores/reasons
- Real query-local cache cleanup assertions passed before/after all 22 queries
- SQLite synchronous=2 (FULL); 5,000 semantic source rows, index sources, publications, points and named vectors; 5,000 completed_fresh states; no pending work or worker token/claim; all durable counters/aggregates unchanged afterward
- Before/after literal comparisons passed for baseline versus named commit, current candidate versus frozen source, both qualification copies, every byte outside the changed method spans, and exact canonicalizer
- measure.py, PLAN.md and canonicalizer.py were frozen before execution and remain byte-identical afterward
- Maximum process time limit was 600 seconds; normal completed exit code was 0; setup elapsed 34.958526580 seconds

## Scope limits

This measures only one fixed synthetic strong-semantic, flat-final-score scenario. It establishes neither production/all-mode benefit nor timings for weak, nonflat, no-semantic, mixed-kind or race scenarios. Independent qualification covers those correctness cases subject to its stated sampled-component/clock-boundary/provider-exception limitations. It does not establish unconditional temporal equivalence. Python 3.8.10 was not exercised. The earlier withdrawn length-bound experiment remains failed and untouched.

## Evidence

- results.jsonl: every full canonical output, raw timing, gate, actual count, and before/after durable/source record
- summary.json: machine-readable complete raw pair arrays and gate summaries
- run.log and exit-code.txt: one-shot process result
- frozen/{measure.py,PLAN.md,canonicalizer.py}, freeze-verification.log and preparation-checks.log: declared protocol, executable source and literal comparison evidence
