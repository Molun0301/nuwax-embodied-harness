//! `ExplorationTerrainMapExtProfile`:探索 profile 选择(不持有算法)。

use nuwax_nav_terrain_map_ext::TerrainMapExtConfig;

/// 探索 profile:基于 `TerrainMapExtConfig` 只覆盖探索参数。
///
/// 仅持有 typed `TerrainMapExtConfig`,不实现任何 voxel/crop/ground/merge 算法。
#[derive(Debug, Clone)]
pub struct ExplorationTerrainMapExtProfile {
    /// profile 名(诊断用)
    pub profile_name: String,
    /// 完整 typed config(由 `nuwax-nav-terrain-map-ext` 提供)
    pub config: TerrainMapExtConfig,
}

impl ExplorationTerrainMapExtProfile {
    /// 探索默认 profile(覆盖 `scan_voxel_size=0.1` / `decay_time=4.0` /
    /// `use_sorting=true` / `quantile_z=0.1` / `lower_bound_z=-2.5`)。
    ///
    /// 其它字段继承 `TerrainMapExtConfig::default()`。若后续导航 stack 复用同一组
    /// 常量,应从本 adapter 导入而不是复制。
    #[must_use]
    pub fn default_exploration() -> Self {
        // 探索 profile 选定参数(方案 §7.3)
        let config = TerrainMapExtConfig {
            scan_voxel_size: 0.1,
            decay_time: 4.0,
            use_sorting: true,
            quantile_z: 0.1,
            lower_bound_z: -2.5,
            ..TerrainMapExtConfig::default()
        };
        Self {
            profile_name: "exploration/default".to_string(),
            config,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_terrain_map_ext::PointCloud;

    #[test]
    fn default_profile_name_matches_adoc() {
        let p = ExplorationTerrainMapExtProfile::default_exploration();
        assert_eq!(p.profile_name, "exploration/default");
    }

    #[test]
    fn default_profile_overrides_selected_params() {
        let p = ExplorationTerrainMapExtProfile::default_exploration();
        assert!((p.config.scan_voxel_size - 0.1).abs() < 1e-9);
        assert!((p.config.decay_time - 4.0).abs() < 1e-9);
        assert!(p.config.use_sorting);
        assert!((p.config.quantile_z - 0.1).abs() < 1e-9);
        assert!((p.config.lower_bound_z - -2.5).abs() < 1e-9);
    }

    #[test]
    fn reexports_are_accessible() {
        // 编译期断言:`PointCloud` 是从 `nuwax-nav-types-messages` 透传的 re-export
        // (type_name 给出原始 crate 路径,而非 re-export 路径,这是正常的)。
        assert_eq!(
            std::any::type_name::<PointCloud>(),
            "nuwax_nav_types_messages::sensors::PointCloud"
        );
    }
}
