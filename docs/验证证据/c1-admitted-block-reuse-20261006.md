# C1 已准入精细块复用验证记录

2026-10-06 UTC。将 FineMapReader 的准入与消费接到同一个有界解码缓存，消除已实际证明的第二次块读取和物化。23 项聚焦原生测试通过；同一输入的 7 组成对测量中，两个观察阶段合计中位数由 25.348 ms 降至 14.064 ms，下降 44.5163%。该结论限于此处的全内联组件工作负载和混合优化测试构建，不代表实时截止期、全传输链路或实机资格。

## 生产改动与所有权

- `Receiver::new(repository)` 仍采用原有 4 MiB 默认缓存；新增 `Receiver::with_cache_budget(repository, budget_bytes)` 配置同一个既有 RegionReader
- 新增实际消费操作 `Receiver::block(&BlockRef) -> Result<Arc<Block>, String>`，直接调用其 RegionReader；没有为了测试暴露缓存 getter
- FineMapReader 删除独立的 RegionReader，使用 32 MiB Receiver，并让 observations 与 columns 从它读取。原消费工作集的 32 MiB 预算保留，未缩为 4 MiB；不再额外持有独立的 4 MiB 准入缓存
- 缓存仍由原表面构建任务中的 Receiver 独占管理；未增加全局缓存、锁、内容哈希、点云复制或 DTO 适配层
- RegionReader、Receiver 的 receive/progression/fetch/collection 逻辑未改动。公开的 block 操作不要求引用属于当前根，因此已接受且仍由调用方持有 Arc<Manifest> 的旧根仍可读取
- 弱根登记、live-root 保留集合、磁盘垃圾回收和失败接收行为保持原样。消费会更新同一个既有缓存的 LRU 年龄；缓存压力下允许重读被驱逐对象
- 共享 Arc<Block> 只共享不可变原始证据，不授予当前运动权限。根绑定、时间新鲜度、观察拥有方替换与权限退休仍由现有消费路径负责

生产差异为两文件 +14/-7 行；聚焦测试另改 `nuwax-map-chunks/src/tests.rs`。`repository.rs` 与此前合格的 `surface/body.rs` 均未修改。schema 验证器中临时反序列化后丢弃的 decode_block 也未优化；本测量没有把该独立成本算作已消除。

## 消除重复工作的直接证据

保留的 `inline_admission_and_separate_reader_materialize_distinct_blocks` 用新目标仓库接收 4 个内联块。准入阶段 read_object 调用为 0；随后独立消费 RegionReader 产生 4 次额外 read_object，得到值相等但 Arc::ptr_eq 为假的块。重复消费仅复用其自己的 Arc。

新增的 `inline_admission_and_shared_reader_reuse_blocks_without_object_reads` 从 Receiver 私有准入缓存捕获 Arc，再经公开 block 操作读取，逐块证明同一个 Arc，额外 read_object 为 0。点数不匹配仍拒绝且不触发磁盘读；下一证据修订接收后，保留的旧根仍获得原 Arc。每个证据修订的四块缓存逻辑记账为 262656 字节，两个证据修订合计为 525312 字节；这不是分配器或 RSS 测量。此项减少的是每个证据修订的这四块消费阶段的 4 次重读和 4 个独立解码块实例，不包括重复内联正文的必要重新验证。

实际 FineMapReader 的单独 Arc 测试检查 observations、columns 与重复 receive 后的共享身份。Arc 探针与计时测试分开。计时测试删掉已不存在的独立消费缓存字节诊断，没有预读或预解码新探针，也未增加公开测试接口。

预算与回收测试还覆盖：默认 4 MiB 下最多保留该夹具一个 2097280 字节块，32 MiB 配置保留三个块共 6291840 字节且消费零重读；192 字节预算下，旧根仍被持有时，被驱逐的一点块恰好重读一次且值不变；最后一个旧根引用释放后，后续 GC 移除其磁盘对象和缓存，同时保留其他仍持有的根。

## 构建与源码身份

本次候选为未提交工作树。记录的本地基点为 `2bf54cc108a63dfa728a486fefd4de8ce8331620`，基点 tree 为 `1fabe2eac417c4fecde21552148f22b2b93dbe72`；记录的远端提交为 `410290dbc45c20db10bf2782556b555a0175bcc4`。这些提交不代表候选已提交或推送。

候选构建于 17:40:02–17:41:09 UTC 完成，退出码 0。使用 locked/offline 的 test profile，MLS 与 runtime 两包 opt-level=3，其余 Rust 包为普通测试优化级别 0；debug=0，启用 body-query-profile。这不是统一 release 构建。构建前后五个相关源码校验一致；编写本文时再次核对一致。

```sh
cargo test --locked --offline \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' \
  -p nuwax-map-chunks -p nuwax-nav-stack -p nuwax-nav-mls-planner \
  --features nuwax-nav-stack/mls,nuwax-nav-runtime/body-query-profile \
  --lib --no-run --message-format=json
```

完整环境加载和实际命令见下述证据目录的 `build.sh`、`build.log` 与 `build.jsonl`；本文编写阶段未重新运行构建或测试。

源码路径以下均相对 `services/nuwax-workspace`：

| 文件 | SHA-256 |
| --- | --- |
| `crates/nuwax-nav-stack/src/surface_chunks.rs` | `2104254ba2c5a9c51e5297a7481937bc650b8b50a42107808e88328501cd5547` |
| `crates/nuwax-map-chunks/src/tests.rs` | `b2705f0cf16675e2829f28685174478c000657355314c74786ba4d629c369c54` |
| `crates/nuwax-map-chunks/src/lib.rs` | `2e08aaf4cbd10dba820eddabce7d86769e41d8527cf457d5d6dac91708d8cb17` |
| `crates/nuwax-map-chunks/src/repository.rs` | `ff8183ba4ae0032c0743b34901ad5e9cff1dd31d09ecef8a8cf929ef0bbd1670` |
| `crates/nuwax-nav-mls-planner/src/surface/body.rs` | `5c1622546dc922566197b72797c8271029d6c31144836b629e17363bd6be2305` |

| 测试二进制 | 字节数 | SHA-256 |
| --- | ---: | --- |
| 候选 `nuwax_map_chunks` | 5436464 | `be1a6fdbd1c2cbbc84ca60ef1695bede4eafb345bb7d6f0b2b63ec6fc24e986d` |
| 候选 `nuwax_nav_stack` | 28139472 | `d59e7315db32c883b59fd2a078ef7b50bf87607cb9c540bed38a2f36baf5d800` |
| 候选 `nuwax_nav_mls_planner` | 8709120 | `e8e0546632d3c2f5e2ffe7f7b3fce35e504cd506e617ea2807302230fc6447b8` |
| 基线 `nuwax_nav_stack` | 28111808 | `71ac2e634d0b4ca5748c394eaa808adfeec1968d7d055ca52dfadfb6b3a97cf9` |

## 已通过的聚焦回归

17 次测试进程调用共执行 23 项测试，全部通过，失败、忽略和超时均为 0。下面记录的是本次实际运行范围，不是整个 workspace 全套通过。

| 日志名 | 通过数 | 用例选择器 |
| --- | ---: | --- |
| `receiver-regressions.log` | 6 | `tests::receiver_` |
| `inline-both-cache-layouts.log` | 2 | `tests::inline_admission_` |
| `pinned-eviction-gc.log` | 1 | `tests::shared_reader_reloads_evicted_pinned_root_and_releases_it_after_gc` |
| `failed-fetch-old-reader.log` | 1 | `tests::failed_fetch_preserves_current_root_and_retained_old_reader` |
| `duplicate-conflict.log` | 1 | `tests::duplicate_root_zero_work_and_included_conflict_is_admitted` |
| `cleanup-failure.log` | 1 | `tests::cleanup_failure_does_not_report_a_failed_committed_receive` |
| `cache-hit.log` | 1 | `tests::cached_block_hit_does_not_reopen_or_rescan_members` |
| `immutable-conflict.log` | 1 | `tests::immutable_install_conflict_never_overwrites` |
| `legacy-spatial-alias.log` | 1 | `tests::legacy_alias_paths_and_shared_spatial_keys_are_retained` |
| `fine-reader-reuse.log` | 1 | `surface_chunks::tests::observations_and_columns_reuse_admitted_block_arcs` |
| `fine-evidence-refresh.log` | 1 | `surface_chunks::tests::evidence_revision_refreshes_observations_without_dirtying_geometry` |
| `fine-duplicate-conflict.log` | 1 | `surface_chunks::tests::duplicate_root_still_rejects_conflicting_included_body_without_installing_it` |
| `fine-changed-columns.log` | 1 | `surface_chunks::tests::changed_blocks_merge_vertical_and_non_aligned_neighbors_and_keep_deletions` |
| `fine-local-read-budget.log` | 1 | `surface_chunks::tests::fixed_dirty_region_reads_same_points_when_far_map_grows` |
| `fine-reply-binding.log` | 1 | `surface_chunks::tests::reliable_block_reply_must_echo_exact_scope_and_bounded_bytes` |
| `fine-door-integration.log` | 1 | `surface_chunks::tests::complete_fine_manifest_resolves_a_real_door_across_storage_blocks` |
| `query-owner-retirement.log` | 1 | `surface::observations::snapshot_cache_tests::identical_observation_owner_preserves_queries_but_replacement_retires_them` |

覆盖不可变 ID 冲突、重复根携带冲突正文、虚假几何/块修订、未来证据、域错配、根外 inclusion、失败补取保留当前及旧根、清理失败不反转已提交 receive、精确 RPC 根/块响应绑定、观察证据刷新，以及观察拥有方替换退休查询。已有跨存储块窄门用例也通过，实际日志记录测试主体约 41.39 s；它是本地组件/积分用例，不等于真实传输与执行器链路。

## 同输入成对操作测量

使用已保存且校验过的基线与候选原生测试二进制。7 对均采用全新进程；奇数对先基线，偶数对先候选。14 次均通过同一个 `evidence_revision_refreshes_observations_without_dirtying_geometry` 用例。没有重采样或丢弃较慢结果。

每次输入均为 4096 个体素、4 个不同 assigned-ID 块，每个证据修订的 included 正文共 49216 字节，两个修订只更新证据，几何修订不变。八段顺序均为 receive1、observations1、columns1、receive2、observations2、columns2、repeat_receive2、repeat_observations2。首次列展开读取 4096 点，证据更新后的列展开读取 0 点。每个体素的时间/命中从 1/1 更新至 2/2，旧观察保留原值；重复根与重复观察复用原 Arc，RPC client 始终未初始化。

下表是各段独立中位数；“两个观察合计”和“八段合计”先在每次运行内相加，再取中位数，不是把各段中位数相加。百分比为 `(候选中位数/基线中位数 - 1) × 100%`。

| 操作 | 基线中位数 ms | 候选中位数 ms | 变化 |
| --- | ---: | ---: | ---: |
| receive1 | 3.689 | 3.535 | -4.17% |
| observations1 | 15.227 | 10.075 | -33.83% |
| columns1 | 11.552 | 10.467 | -9.39% |
| receive2 | 4.058 | 4.486 | +10.55% |
| observations2 | 9.522 | 4.606 | -51.63% |
| columns2 | 0.035 | 0.027 | -22.86% |
| repeat_receive2 | 3.651 | 3.578 | -2.00% |
| repeat_observations2 | 0.003 | 0.003 | +0.00% |
| 两个观察合计 | 25.348 | 14.064 | -44.5163% |
| 八段合计 | 47.770 | 37.506 | -21.4863% |

两个观察合计的范围为基线 14.602–32.382 ms、候选 12.237–18.070 ms；八段合计范围为基线 32.322–76.690 ms、候选 31.449–53.728 ms。第二次 receive 的中位数上升 10.55%，不能把所有阶段都描述为变快。观察转换与替换仍是必需工作，没有声称此前约 14 ms 的全部成本消失。

| 配对 | 基线观察合计 ms | 候选观察合计 ms | 观察合计下降 | 基线八段 ms | 候选八段 ms |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 23.450 | 12.237 | +47.8166% | 45.300 | 37.506 |
| 2 | 14.602 | 18.051 | -23.6201% | 32.322 | 40.895 |
| 3 | 27.529 | 13.193 | +52.0760% | 54.709 | 31.449 |
| 4 | 25.348 | 18.070 | +28.7123% | 45.218 | 53.728 |
| 5 | 27.559 | 13.023 | +52.7450% | 55.830 | 34.660 |
| 6 | 32.382 | 15.105 | +53.3537% | 76.690 | 38.886 |
| 7 | 20.324 | 14.064 | +30.8010% | 47.770 | 34.706 |

逐对观察合计改善比例的中位数为 47.8166%，7 对中 6 对更快；这与两个分组中位数之比的 44.5163% 是不同统计量。八段合计逐对改善中位数为 27.3477%，7 对中 5 对更快。样本规模小且波动明显，只支持此单一工作负载的局部收益。

## 进程资源与未验证范围

资源来自同一 14 次调用的 wait4 与进程计时，包含夹具准备、断言和测试框架；不能直接归因于两段观察操作。

| 指标 | 基线中位数 | 候选中位数 | 变化 |
| --- | ---: | ---: | ---: |
| 进程 user+system CPU | 108.201 ms | 99.056 ms | -8.4519% |
| 进程 wall | 114.822 ms | 110.040 ms | -4.1644% |
| launch peak RSS | 40576 KiB | 40576 KiB | +0.0000% |

CPU 范围为基线 76.525–174.960 ms、候选 77.787–127.138 ms；wall 范围为基线 78.182–177.045 ms、候选 83.557–133.915 ms。非自愿上下文切换中位数/范围分别为基线 9/1–87、候选 5/1–50。所有 launch peak RSS 均为 40576 KiB；该读数未显示差异，也不能作为 Rust 堆分配或内存节省证据。

- 本次没有运行完整原生套件、全地图输入分布或严格实时基准
- 本次没有运行真实进程缺块 RPC。环境已知 AF_UNIX 创建受阻，未重试、替换传输或绕过限制；背景见 [组合资格记录](c1-assigned-navigation-composition-20261006.md) 与 [恢复拥有方核对](c1-map-change-recovery-audit-20261006.md)
- 精确 RPC 响应绑定用例通过只证明响应检查逻辑，不证明真实网络恢复过程通过
- 没有实机传感器、Motion Guard 全链路、执行器或发布资格；不据此修改现有新鲜度、碰撞、制动或权限边界
- 本记录对应本地未提交候选；本次未创建分支、提交或推送

## 原始证据位置

候选证据目录 `Q=/workspace/shared/neh-admitted-block-reuse-20261006`；基线目录 `/workspace/shared/neh-inline-block-attribution-20261006`。它们是当前验证环境中的本地证据位置，不是仓库内可随克隆取得的附件。

- `Q/candidate-results.json`：23 项聚焦测试的选择器、退出码、资源和逐项日志路径
- `Q/matched-operations/results.json`：14 次实际测量的输入签名、八段原始微秒值、资源、二进制身份与汇总
- `Q/matched-operations/01-baseline.log` 至 `07-candidate.log`：全部成对原始日志
- `Q/source-binaries.json`、`source-before-build.sha256`、`source-after-build.check`：源码与二进制身份
- `Q/build.sh`、`build.jsonl`、`build.log`、`build.exit`、`build-start.txt`、`build-end.txt`：实际构建证据
- `Q/reuse.patch` 和 `Q/before/`：本次窄改动与修改前内容

| 证据文件 | SHA-256 |
| --- | --- |
| `candidate-results.json` | `ab2a2c5e5be946a46fbbcf50ddba16f0c18feb512221b56b107d72f97824a36e` |
| `matched-operations/results.json` | `760dae670f614c3503671f17215eacee36615efb8ea01c484963700e85aee44c` |
| `source-binaries.json` | `60e47c55aeaf20e9ed60aa59fe02481aadcf2c967c20c4a2d98f0e076d60a214` |
| `reuse.patch` | `a8a94f1a6d1a325dece81c9ef04ceec479ff92c6246542ae56e9220eaa1fd37d` |
| `build.sh` | `133bfda4d7d6b999bfc7d6734001d3ed31defc6cf28c7029179b45cd55120393` |
