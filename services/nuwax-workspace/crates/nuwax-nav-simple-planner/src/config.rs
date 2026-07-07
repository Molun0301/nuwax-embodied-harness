//! `SimplePlannerConfig`：默认参数与字段定义。
//!
//! 所有数值与 Python 原 `SimplePlannerConfig` 对齐；字段顺序未保持，仅语义一致。

/// 默认坐标系名称常量。Foundation lane 完成 types-messages 后替换为统一常量。
pub const FRAME_MAP: &str = "map";
/// 默认 body 坐标系。
pub const FRAME_BODY: &str = "body";
/// 默认 sensor 坐标系。
pub const FRAME_SENSOR: &str = "sensor";

/// 简易路径规划器配置。
#[derive(Debug, Clone)]
pub struct SimplePlannerConfig {
    /// 世界坐标系
    pub world_frame: String,
    /// 机器人本体坐标系
    pub body_frame: String,
    /// 传感器坐标系（Unity sim bridge 使用）
    pub sensor_frame: String,

    /// 每 cell 边长（米）
    pub cell_size: f64,
    /// 障碍物高度阈值（米，相对于地面）
    pub obstacle_height_threshold: f64,
    /// 默认膨胀半径（米）
    pub inflation_radius: f64,

    /// lookahead 距离：沿 A* 路径往前多少米发布 waypoint
    pub lookahead_distance: f64,
    /// A* 重规划频率（Hz）
    pub replan_rate: f64,
    /// waypoint 发布频率（Hz）
    pub waypoint_rate: f64,
    /// 重规划冷却时间（秒），冷却内不重跑 A*
    pub replan_cooldown: f64,
    /// A* 最大扩展节点数
    pub max_expansions: usize,
    /// ground_offset_below_robot（米）：rz - 此值 = ground_z
    pub ground_offset_below_robot: f64,

    /// Stuck 检测：超过该秒数未进展且仍在最小阈值之上则缩小 inflation
    pub stuck_seconds: f64,
    /// Stuck 检测：goal_dist 至少缩短该值才算进展
    pub progress_epsilon: f64,
    /// 每次升级的缩小系数
    pub stuck_shrink_factor: f64,
    /// inflation 最小值（硬下限）
    pub stuck_min_inflation: f64,

    /// 到达该距离视为目标到达，进入 idle
    pub goal_reached_threshold: f64,
}

impl Default for SimplePlannerConfig {
    fn default() -> Self {
        Self {
            world_frame: FRAME_MAP.into(),
            body_frame: FRAME_BODY.into(),
            sensor_frame: FRAME_SENSOR.into(),
            cell_size: 0.3,
            obstacle_height_threshold: 0.15,
            inflation_radius: 0.2,
            lookahead_distance: 1.0,
            replan_rate: 5.0,
            waypoint_rate: 30.0,
            replan_cooldown: 2.0,
            max_expansions: 200_000,
            ground_offset_below_robot: 1.3,
            stuck_seconds: 5.0,
            progress_epsilon: 0.25,
            stuck_shrink_factor: 0.5,
            stuck_min_inflation: 0.05,
            goal_reached_threshold: 0.39,
        }
    }
}
