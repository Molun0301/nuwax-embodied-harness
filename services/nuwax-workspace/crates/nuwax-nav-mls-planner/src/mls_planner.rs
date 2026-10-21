// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! 共享多层表面地图的几何与前沿配置。

use validator::{Validate, ValidationError};

// `#[validate(...)]` 属性宏由 validator crate 的 `Validate` derive 生成;
// 保留 nuwax 旧 `#[native_config]` 行为(序列化 + 字段校验),此处只使用 Validate。
/// MLS（多层表面）地图配置：体素、机器人几何、墙距、节点、目标容差等。
#[derive(Clone, Validate)]
#[validate(schema(function = "validate_wall_buffer"))]
pub struct Config {
    /// 全局参考坐标系名，例如 `map`。
    pub world_frame: String,
    /// 体素边长(米)。
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    /// 机器人身高(米),用于判断体素是否可站立。
    #[validate(range(exclusive_min = 0.0))]
    pub robot_height: f32,
    /// 形态学闭运算半径(米),填充提取表面中的小孔(可达 2 倍半径宽)。
    #[validate(range(min = 0.0))]
    pub surface_closing_radius: f32,
    /// 节点之间的最小间距(米),NMS 使用。
    #[validate(range(exclusive_min = 0.0))]
    pub node_spacing_m: f32,
    /// 硬净空(米):距墙/边缘小于此值的体素不可通行。
    #[validate(range(min = 0.0))]
    pub wall_clearance_m: f32,
    /// 软缓冲宽度(米),在 clearance 之外的代价软增长区。
    #[validate(range(min = 0.0))]
    pub wall_buffer_m: f32,
    /// 软墙代价峰值:在 clearance 边缘处乘子为 `1 + weight`,外缘衰减到 1。
    #[validate(range(min = 0.0))]
    pub wall_buffer_weight: f32,
    /// 最大可跨越的垂直高度(米),超过则不可通行。
    #[validate(range(min = 0.0))]
    pub step_threshold_m: f32,
    /// 共享表面允许的连续坡道最大坡度（度）；台阶另需具备真实踏面。
    #[validate(range(min = 0.0, exclusive_max = 90.0))]
    pub max_slope_deg: f32,
    /// 台阶两侧沿行进方向必须存在的最小踏面深度（米）。
    #[validate(range(exclusive_min = 0.0))]
    pub min_step_tread_depth_m: f32,
    /// 每米爬升增加的软代价权重。
    #[validate(range(min = 0.0))]
    pub step_penalty_weight: f32,
    /// 距目标的三维欧氏距离(米),低于此值停止重新规划。
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f32,
    /// 可视化产物发布频率上限(Hz),`0` 关闭。路径本身不限流。
    #[validate(range(min = 0.0))]
    pub viz_publish_hz: f32,
}

/// 软墙代价只在已配置的非零范围内生效。
fn validate_wall_buffer(config: &Config) -> Result<(), ValidationError> {
    if config.wall_buffer_weight > 0.0 && config.wall_buffer_m == 0.0 {
        return Err(ValidationError::new(
            "wall_buffer_weight requires wall_buffer_m > 0",
        ));
    }
    Ok(())
}

impl Config {
    /// 由闭运算半径计算膨胀及腐蚀次数。
    pub fn closing_passes(&self) -> u32 {
        (self.surface_closing_radius / self.voxel_size).ceil() as u32
    }
}
