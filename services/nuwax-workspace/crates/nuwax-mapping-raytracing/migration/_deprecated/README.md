# `_deprecated/` — 2026-07-08 整体弃用

> **状态:整体弃用**(2026-07-08 fake-lcm 物理删除 v3 F2 决策)

## 文件清单(已 git mv 迁入)

| 文件 | 原始 commit 锚点 | 说明 |
|---|---|---|
| `main.rs` | (本仓 2026-07-08 之前 dimos 1:1 迁移) | 511 行原始 `dimos-module` + `lcm-msgs` 适配入口,**调用 `dimos_module::LcmTransport::new()`(fake backend 默认)**,已不可能编译通过 |
| `python.rs` | (本仓 2026-07-08 之前 dimos 1:1 迁移) | 231 行 pyo3 binding,与 main.rs 同命运 |
| `README.md`(本文件) | (原 `migration/README.md` 内容) | dimos 旧 fake API 入口说明 |

## 为什么整体弃用

- `dimos_module::LcmTransport::new()` 默认返回**内存假 backend**(`FakeLcmBackend` 等价)
- 2026-07-08 fake-lcm 物理删除后,本仓 `nuwax-transport` 不再有 `FakeLcmBackend`,
  也无 `new_fake()` 工厂
- 这些文件**不再可能编译通过**(引用了已删的 fake API)
- 即便补一个 dimos-shim,也要重写 main.rs 的 `LcmTransport::new()` → `LcmTransport::new_real(udpm://...)`,等同于重写,不算"恢复"

## 恢复条件(若后续要重新启用)

按 v3 决策,**不预设**恢复路径。如有真实需求,需先:

1. 在 `crates/nuwax-core` 实现 `dimos-module` 过程宏的最小兼容 shim(当前未计划)
2. 在 `lcmtypes` 覆盖 `lcm-msgs` 的所有引用类型
3. 重写 `main.rs` 把 `LcmTransport::new()` 改为 `LcmTransport::new_real("udpm://...")`,
   并把 dimos 的 `run::<RayTracingVoxelMap, _>(transport)` 适配为 nuwax-core 的等价 lifecycle
4. 重写 `python.rs` 把 pyo3 绑定接进当前 Python gateway 入口
5. 重新开 nuwax-mapping-raytracing 的 workspace member 声明(当前 standalone)

## 历史 commit 锚点

- 2026-07-08 fake-lcm 根除 commit(本任务):`git log --all --oneline -- migration/_deprecated/` 找到具体 hash
- 2026-07-08 之前 dimos 1:1 迁移 commit:`git log --all --oneline -- migration/main.rs` 反查
- 上游 dimos 锚点:不跟踪(本仓断尾,见 `feedback_root_cause_before_fix.md`)

## 占位原则

本文件夹**不再被任何 cargo feature / build target 引用**。如需清理,可后续用 `git rm -r migration/_deprecated/` 物理删除(本任务不动)。
