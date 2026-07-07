//! re-export 验收(对应方案 §11.2 `reexport_processor_compiles`)

use nuwax_explore_terrain_map_bridge::{TerrainMapExtConfig, TerrainMapExtProcessor};

#[test]
fn reexport_pointers_are_usable() {
    let _ = TerrainMapExtConfig::default();
    // 编译期使用 Processor 类型,确保导入路径稳定
    fn _processor_typed(_p: TerrainMapExtProcessor) {}
    let _ = _processor_typed;
}
