# 未保留的密集表扫描试验

2026-10-06 UTC。结论：试验通过语义检查，但没有表现出可重复的目标阶段收益，因此仅撤回
本次17行差异。此前已验证的单带缓存/BFS优化保持原样，不将本次结果称为已证实的回退。

## 试验与先验依据

原二进制在dense selection和ordered emission两段中仍有动态整数除法/余数，用于从
flat index恢复X/Y。捕获场景两段各进行179,076次坐标恢复。试验仅将两段改为相同
X-major/Y-minor的chunks_exact行遍历：11行增加、6行删除，没有新缓存、分配、锁、哈希、
BFS或浮点表达式变化。visited显式释放仍位于原emission计时范围。

独立审查确认索引次序、miss/eviction次序、seed tie、i32边界、点/平面位与原算法等价。
实验前body.rs为 `5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305`，
实验版本为 `9f7870a134e890f879883819f1e48c912a958e4fd98a821689661ca8bd703cb9`。

## 实际资格

- 同一混合test profile增量编译通过，1m34s；MLS/runtime opt3、其他crate普通test、
  debug0、debug assertions和overflow checks开启、body-query-profile开启
- 11项相关语义测试通过；两个有意忽略的微基准没有在该语义批次运行
- 七组AB/BA交替、新进程/新拥有方的捕获场景比较全部退出0。独立重新解析14份日志，
  冷/暖命令、progress、索引、状态标记、body及route stage调用数/工作counter一致

| 原始测量 | 原已验证版本中位数 | 试验中位数 |
|---|---:|---:|
| whole cold step | 39.323388ms | 39.099583ms |
| selection+emission（先在每条记录内相加） | 7.403419ms | 7.420242ms |
| selection | 5.832522ms | 6.090200ms |
| emission | 1.404050ms | 1.561262ms |

受影响两段只有3/7对变快；配对candidate−baseline差值的中位数为+0.207121ms。
whole-step变化很小，运行存在明显离群值，不能据此宣布性能收益。进程CPU/RSS包含
构造、其他步骤和oracle，也不能拿来替代缺失的目标阶段收益。

独立结果审查同意“不保留，收益未证实”。没有为了得到正结果继续扩大样本、选择性删除
离群值或改用不同优化级别。既然本试验被撤回，不再额外跑低可达率性能矩阵。

## 撤回和证据

只从已冻结的精确前置文件恢复body.rs，恢复后SHA256重新验证为
`5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305`。
其他导航、测试和文档工作未回滚。此恢复不改变先前24.6%冷中位改善的独立对照记录。

原始目录：`/workspace/shared/neh-dense-scan-trial-20261006/`。
保留 `body-baseline.rs`、`body-candidate.rs`、`trial.diff`、构建日志/摘要、
`run-semantic.py`、`compare-cold.py`、语义结果、14份原始对照日志、
`matched-cold/results.json`、`decision.txt`。对照原二进制为cadence-ramp目录保留的
runtime测试二进制 `c4aa36e4969c2da1f496a3b2e5a1bb057340a6d47bc065e41ea5f06f562e819f`。

本阶段没有Git提交或推送，也没有改变安全、观察新鲜度或实时节拍阈值。
