//! PatrolRouter implementations。

use nuwax_nav_types_messages::{
    FrontierGoalInput as FoundationFrontierGoalInput, GridSnapshot as FoundationGridSnapshot,
};

/// PatrolRouter trait：纯算法核心，由 router 输入/输出 `PoseStamped`（自包含）。
pub trait PatrolRouter: Send {
    /// 更新 grid（自包含 grid 数据）
    fn update_grid(&mut self, grid: GridSnapshot);
    /// 更新 pose
    fn update_pose(&mut self, pose: (f64, f64, f64));
    /// 下一个巡逻目标
    fn next_goal(&mut self) -> Option<(f64, f64, f64)>;
    /// 当前 saturation
    fn saturation(&self) -> f32;
    /// reset
    fn reset(&mut self);
}

/// Grid 快照（自包含，不依赖外部 OccupancyGrid 类型）
///
/// 字段语义与 Foundation `GridSnapshot` 完全对齐（`width/height/resolution/
/// origin_x/origin_y/data`），但内部 router 索引大量使用 `usize`；
/// Foundation 是 `u32`，故保留本地类型，但通过 `foundation_re_export`
/// 暴露 Foundation `GridSnapshot` 供调用方对接（参见 `tests_for_foundation_alias`）。
pub type GridSnapshot = FoundationGridSnapshot;

/// Frontier 输入（探索板块发出，导航不持有 wavefront 算法）
///
/// 与 Foundation `FrontierGoalInput` 字段对齐后改为别名：
/// `pose / score / information_gain / distance_cost / frontier_area_m2 / producer`
/// 六字段一致。Foundation 类型即为 alias 目标。
pub use nuwax_nav_types_messages::FrontierGoalInput;

pub mod coverage;
pub mod frontier;
pub mod random;

pub use coverage::CoveragePatrolRouter;
pub use frontier::FrontierPatrolRouter;
pub use random::RandomPatrolRouter;

/// GridSnapshot 扩展：本地依赖 `free_mask()`，
/// Foundation `GridSnapshot` 未提供，本 trait 补齐。
pub trait GridSnapshotExt {
    /// free mask (row-major, size = w*h)
    #[must_use]
    fn free_mask(&self) -> Vec<bool>;
}

impl GridSnapshotExt for GridSnapshot {
    fn free_mask(&self) -> Vec<bool> {
        self.data.iter().map(|&v| v == 0).collect()
    }
}

// 抑制 unused 警告 —— 类型别名仅用于跨 crate 对齐
#[allow(unused_imports)]
const _: fn() = || {
    let _: FoundationFrontierGoalInput = FoundationFrontierGoalInput::new((0.0, 0.0, 0.0), 0.0, "");
};
