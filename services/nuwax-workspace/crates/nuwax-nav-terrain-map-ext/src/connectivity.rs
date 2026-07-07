//! Connectivity 占位：详细 BFS 在 `ground::estimate_ground` 内完成；
//! 本模块提供复用接口（按 planar voxel 中心点列表查连通性）和单元测试 helper。

/// 仅做连通性检查 + center elevation 填充（已合并到 `ground::estimate_ground`）。
/// 此模块保留为公开 helper，方便其他 crate 复用。
#[must_use]
pub fn is_connected(conn: &[i8], idx: usize) -> bool {
    conn.get(idx).copied().unwrap_or(0) == 2
}
