//! `TerrainMapExtConfig`：方案 §5.3 表格全部 20 字段。

/// 默认坐标系名常量。Foundation lane 完成 types-messages 后替换为统一常量。
pub const FRAME_MAP: &str = "map";

/// 扩展地形图配置（对应 Python `TerrainMapExtConfig`）。
#[derive(Debug, Clone)]
pub struct TerrainMapExtConfig {
    /// 世界坐标系
    pub world_frame: String,

    /// scan 下采样 leaf size（米）
    pub scan_voxel_size: f64,
    /// 累积点衰减时间（秒）
    pub decay_time: f64,
    /// 距机器人该距离内的点不衰减（米）
    pub no_decay_distance: f64,
    /// 手动 clearing 距离（米）
    pub clearing_distance: f64,

    /// 是否用排序取 quantile 作为地面高度
    pub use_sorting: bool,
    /// ground 估计分位数（0..1）
    pub quantile_z: f64,

    /// 车辆高度（米）
    pub vehicle_height: f64,

    /// z 相对车辆的高度裁剪下界（米）
    pub lower_bound_z: f64,
    /// z 相对车辆的高度裁剪上界（米）
    pub upper_bound_z: f64,
    /// 距离对裁剪的缩放（米/米）
    pub distance_ratio_z: f64,

    /// voxel 点数超阈值触发下采样
    pub voxel_point_update_threshold: usize,
    /// voxel 时间超阈值触发下采样（秒）
    pub voxel_time_update_threshold: f64,

    /// 是否做 terrain BFS 连通性过滤
    pub check_terrain_connectivity: bool,
    /// 车下地面相对车辆 z 的偏移（米）
    pub terrain_under_vehicle: f64,
    /// 连通性 elevation diff 阈值（米）
    pub terrain_connectivity_threshold: f64,
    /// elevation diff 大于该值标记 ceiling（米）
    pub ceiling_filtering_threshold: f64,

    /// local terrain map 半径（米）
    pub local_terrain_map_radius: f64,
    /// 是否合并 radius 内的 local_terrain
    pub merge_local_terrain: bool,

    /// rolling grid 单元边长（米）
    pub terrain_voxel_size: f64,
    /// ground estimation grid 单元边长（米）
    pub planar_voxel_size: f64,
}

impl Default for TerrainMapExtConfig {
    fn default() -> Self {
        Self {
            world_frame: FRAME_MAP.into(),
            scan_voxel_size: 0.1,
            decay_time: 4.0,
            no_decay_distance: 0.0,
            clearing_distance: 30.0,
            use_sorting: false,
            quantile_z: 0.25,
            vehicle_height: 1.5,
            lower_bound_z: -1.5,
            upper_bound_z: 1.0,
            distance_ratio_z: 0.1,
            voxel_point_update_threshold: 100,
            voxel_time_update_threshold: 2.0,
            check_terrain_connectivity: true,
            terrain_under_vehicle: -0.75,
            terrain_connectivity_threshold: 0.5,
            ceiling_filtering_threshold: 2.0,
            local_terrain_map_radius: 0.5,
            merge_local_terrain: true,
            terrain_voxel_size: 2.0,
            planar_voxel_size: 0.4,
        }
    }
}
