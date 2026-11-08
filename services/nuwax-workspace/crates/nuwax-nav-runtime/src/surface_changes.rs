//! Shared bounded geometry-delta proofs for global and local planning owners.
use crate::SurfaceSnapshot;
use std::sync::Arc;
/// Raw changes relative to one complete source, including deleted columns.
#[derive(Debug)]
pub enum SurfaceGeometryChanges {
    /// Proven unchanged geometry (timestamps and other metadata may change).
    Empty,
    /// Complete fine and coarse column differences; heights are deliberately unbounded.
    Columns {
        /// Actual source resolution in metres.
        fine_size: f32,
        /// Changed source XY columns, including deletions.
        fine: Vec<[i32; 2]>,
        /// Existing planner resolution in metres.
        planner_size: f32,
        /// Changed planner XY columns, including deletions.
        coarse: Vec<[i32; 2]>,
    },
    /// Missing differences, semantic changes or exceeded work bounds.
    Unknown,
}

/// A map publication carries the exact baseline of its geometry differences.
#[derive(Clone)]
pub struct SurfaceMapUpdate {
    /// Latest immutable source for the execution guard.
    pub snapshot: Arc<SurfaceSnapshot>,
    /// Successful source immediately preceding these differences.
    pub change_basis: Option<Arc<SurfaceSnapshot>>,
    /// Complete geometry differences, never inferred from a revision number alone.
    pub changes: Arc<SurfaceGeometryChanges>,
}
impl SurfaceMapUpdate {
    /// 传递不可变感知与变更来源；原生地图由现有计算线程准备。
    /// 持有完整差异的调用者可替换默认未知变更描述。
    pub fn prepare(snapshot:Arc<SurfaceSnapshot>)->Result<Self,String> {
        Ok(Self {snapshot,change_basis:None,changes:Arc::new(SurfaceGeometryChanges::Unknown)})
    }

}
