# Ray Tracing Migration Assets（2026-07-02）

本目录存放从原 `dimos/mapping/ray_tracing/rust/src/` 1:1 迁移、但**当前阶段未启用**的源文件。

## 背景

原 crate 依赖外部上游：
- `dimos_module` / `dimos_module_derive`（过程宏框架）；
- `lcm_msgs`（sensor_msgs / nav_msgs / geometry_msgs / std_msgs 协议栈）；
- `pyo3` + `numpy`（Python 绑定）；
- `dimos_voxel_ray_tracing`（自身 crate 名）。

本 nuwax workspace 当前不提供上述 crate 的同位替代。阶段 0（依赖适配层）完成前，
这些文件无法在 `cargo check` 下编译，因此被移出 `src/`，作为"已迁移资产"存放。

## 文件清单

| 文件 | 行数 | 来源 | 启用条件 |
|---|---|---|---|
| `main.rs` | 511 | 原 `dimos-module` Module derive 入口 + LcmTransport + LCM 协议适配 | 阶段 0 实装 `dimos-module` 兼容 shim + `nuwax-protocol` 覆盖 lcm-msgs 类型 |
| `python.rs` | 231 | 原 pyo3 `#[pymodule] dimos_voxel_ray_tracing` 绑定 | 启用 `pyo3` + `numpy` features（需要 MSRV/Cross 矩阵单独评估） |

## 启用方式

1. 完成上游适配 shim；
2. 把本目录文件回拷到 `src/`；
3. 在 `Cargo.toml` 取消注释 `python` / `lcm-bridge` features；
4. 重新跑 `cargo check --features python,lcm-bridge` 验证。

## 当前阶段策略

默认 `features = []`，仅暴露 `voxel_ray_tracer`（1318 行纯 Rust 核心算法）。
此 crate 已可 `cargo check / clippy / build / test` 通过。

参见：`docs/交叉审查/2026-07-02_修复响应_v3_复核.md` 与
`docs/交叉审查/2026-07-02_修复响应_v4.md`（v4 修复响应）的 CRITICAL 2 处理。
