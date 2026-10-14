# nuwax-explore-wavefront

Pure-Rust 重写的 Wavefront Frontier 探索算法 crate。

## 公共 API 边界

- 公共输出仅为 [`FrontierGoal`](../../nuwax-nav-types-messages/src/visual.rs)。
- 内部 cluster / mask / open set 只用于 `WavefrontDiagnostics` 摘要,**不进入导航 API**。
- `FrontierCluster` / `FrontierCandidate` 为 `pub(crate)`。
- `DetectionOutcome::{Candidates(Vec<FrontierCandidate>), NoReachableFreeSpace}` 为 crate-private。

## Features

| feature | 默认 | 说明 |
|---|---|---|
| `runtime` | ❌ | tokio 异步 runtime 与 stream/RPC begin/end/stop/goal loop |
| `mapping-inflate` | ❌ | 复用 `nuwax-mapping` 的 obstacle inflation(需先在 mapping 与 nav grid 之间提供 adapter) |
| `legacy-parity` | ❌ | Python 坐标语义兼容测试,只用于 `legacy_parity.rs` 验收 |
| `bench` | ❌ | `criterion` benchmark,要求 `ndarray` |

默认 core **不依赖** tokio / stream / RPC / runtime,符合方案 §4.1 依赖原则。

## 测试

```bash
cargo test -p nuwax-explore-wavefront                      # 默认 unit + integration
cargo test -p nuwax-explore-wavefront --features legacy-parity
cargo bench -p nuwax-explore-wavefront --features bench --bench wavefront_500x500
```

## 完成定义

参见 `docs/子任务/探索模块重写/探索模块重写方案.md` §4.3 / AC-3 / AC-4。
