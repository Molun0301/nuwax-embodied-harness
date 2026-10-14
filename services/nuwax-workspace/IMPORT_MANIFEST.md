# IMPORT_MANIFEST

快照导入时间(UTC):2026-07-07T05:55:46Z
source_git_commit:cc730ac82aec1f267b754e51f182ce16203d2041
crate 总数:50
excludes: [target/, .omc/, .omx/, generated/c/, *.lock.tmp, .cargo/config.toml(如含机器私有路径)]
更新命令:`bash scripts/import_nuwax_workspace_snapshot.sh --source <path>`
deferred:[]

---

## W1 ledger 追加对齐(2026-07-11,OCC lane worker Phase 0A #4)

来源:`.omx/plans/inventory_workspace_disposition.json`(50 条 disposition)

### 关键数字

- verified_on_disk(物理在磁盘上):41
- verified_removed(物理已删除):9
- 41 + 9 = 50(对齐原 snapshot `crate_count=50`)

### 9 个物理已删除(原列在 `on_disk_but_not_in_members`,实际磁盘已删)

- nuwax-slam-fastlio2-bridge(FastLIO2 → RTAB-Map,active SLAM 替换)
- nuwax-vision-clip
- nuwax-vision-detect-2d
- nuwax-vision-detect-3d
- nuwax-vision-embedding
- nuwax-vision-fiducial
- nuwax-vision-reid
- nuwax-vision-spatial
- nuwax-vision-tracker

### disposition 摘要

| disposition        | 数量 | 含义                                                                                  |
| ------------------ | ---- | ------------------------------------------------------------------------------------- |
| required-now       | 22   | 首发链路必需                                                                          |
| qualified          | 13   | 已实装并测试,等待 P0C/profile gate                                                    |
| unsupported-gated  | 4    | native-far / native-local / native-path-follower / native-pgo,首发改走 wavefront      |
| intentional-change | 2    | nuwax-mapping-raytracing(stage-0 shim 待)/ nuwax-nav-visual-servoing(TF trait only)    |
| removed            | 9    | 物理已删,原 `on_disk_but_not_in_members` 列表保留以反映 2026-07-07 snapshot 状态      |

### 首发优先级(first-release, Phase P0B §12)

- 探索器(explorer):**wavefront**(非 far-planner)
- 规划器(planner):**simple-2d**(非 replanning-astar)
- 路径跟随:**nuwax-nav-basic-path-follower**(纯 Rust fallback,非 native-pure-pursuit)

### 4 个 unsupported-gated crate 与 wavefront 优先级

- nuwax-nav-far-planner-bridge:`PreferWavefront` 策略 runtime 禁用,首发不启
- nuwax-nav-local-planner-bridge:wavefront 路径无独立 local planner,首发不启
- nuwax-nav-path-follower-bridge:首发走 nuwax-nav-basic-path-follower 纯 Rust fallback
- nuwax-nav-pgo-bridge:wavefront 路径不要求 PGO,首发不启

### 与 snapshot 原数据的对齐

| 原字段                          | 原值                                                                                  |
| ------------------------------- | ------------------------------------------------------------------------------------- |
| snapshotted_at_utc              | 2026-07-07T05:55:46Z                                                                  |
| source_git_commit               | cc730ac82aec1f267b754e51f182ce16203d2041                                              |
| crate_count                     | 50(含 9 个物理已删)                                                                   |
| crate_count_in_workspace_members | 40                                                                                    |
| on_disk_but_not_in_members      | 10(含 9 个已删 + 1 个 raytracing standalone)                                          |
