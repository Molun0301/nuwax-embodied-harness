# nuwax-explore-terrain-map-bridge

**Thin adapter / profile** — 极薄适配层,只 re-export `nuwax-nav-terrain-map-ext` 的 typed 接口
并提供 `ExplorationTerrainMapExtProfile` 探索 profile 选择。

## 严格约束

- **不启动 C++ 进程**(无 native runner 依赖)
- **不实现** `voxel_grid` / `crop` / `ground` / `merge` 等算法模块
- 不复制 `terrain_map_ext.py` 逻辑
- 真实算法唯一 owner 是 `nuwax-nav-terrain-map-ext`

## 用法

```rust
use nuwax_explore_terrain_map_bridge::{
    ExplorationTerrainMapExtProfile,
    TerrainMapExtConfig,
    TerrainMapExtProcessor,
    PointCloud,
    TimedPointCloud,
};

let profile = ExplorationTerrainMapExtProfile::default_exploration();
// processor 由 nuwax-nav-terrain-map-ext 提供
```

## 测试

```bash
cargo test -p nuwax-explore-terrain-map-bridge
```

## 详细方案

参见 `docs/子任务/探索模块重写/探索模块重写方案.md` §7。
