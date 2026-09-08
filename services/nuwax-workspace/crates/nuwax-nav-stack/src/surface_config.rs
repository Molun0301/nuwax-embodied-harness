//! 楼梯、斜坡规划与跟随共用的物理参数；机体到支撑面的标定不得猜测。

use nuwax_nav_mls_planner::mls_planner::Config;
use nuwax_nav_runtime::SurfaceFollowerConfig;

use crate::{NavStackError, NavStackResult};

/// 一次启动捕获并同时传给规划器和控制器的三维导航参数。
#[derive(Clone)]
pub struct SurfaceNavigationConfig {
    /// 支撑面、净空、台阶和坡道约束。
    pub geometry: Config,
    /// 机体参考位姿与受控速度参数。
    pub follower: SurfaceFollowerConfig,
    /// 实际机器人宽度，单位米。
    pub robot_width_m: f64,
}

impl SurfaceNavigationConfig {
    /// 从现有设置加载器提供的环境读取，缺少支撑面标定时明确拒绝。
    pub fn from_env() -> NavStackResult<Self> {
        let offset = std::env::var("NUWAX_NAV_BODY_TO_SUPPORT_Z_M")
            .ok()
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| {
                NavStackError::InvalidConfig(
                    "mls-3d requires calibrated NUWAX_NAV_BODY_TO_SUPPORT_Z_M".into(),
                )
            })?;
        let offset = parse_number("NUWAX_NAV_BODY_TO_SUPPORT_Z_M", &offset)?;
        let width = configured_number("NUWAX_NAV_ROBOT_WIDTH_M", 0.3)?;
        let height = configured_number("NUWAX_NAV_ROBOT_HEIGHT_M", 0.6)?;
        let voxel = configured_number("NUWAX_NAV_SURFACE_VOXEL_SIZE_M", 0.1)?;
        let max_step = configured_number("NUWAX_GEOMETRY_TRAVERSABILITY_MAX_STEP_HEIGHT_M", 0.15)?;
        let max_slope = configured_number("NUWAX_GEOMETRY_TRAVERSABILITY_MAX_SLOPE_DEG", 18.0)?;
        let tread = configured_number("NUWAX_NAV_MIN_STAIR_TREAD_M", 0.2)?;
        let frame = std::env::var("NUWAX_GEOMETRY_MAP_FRAME").unwrap_or_else(|_| "map".into());
        Self::new(
            frame, offset, width, height, voxel, max_step, max_slope, tread,
        )
    }

    /// 显式构造便于部署校验和无硬件回归，不读取或修改进程环境。
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        frame: String,
        body_to_support_z_m: f64,
        robot_width_m: f64,
        robot_height_m: f64,
        voxel_size_m: f64,
        max_step_height_m: f64,
        max_slope_deg: f64,
        min_stair_tread_m: f64,
    ) -> NavStackResult<Self> {
        if frame != "map"
            || [
                body_to_support_z_m,
                robot_width_m,
                robot_height_m,
                voxel_size_m,
                min_stair_tread_m,
            ]
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
            || !max_step_height_m.is_finite()
            || max_step_height_m < 0.0
            || !max_slope_deg.is_finite()
            || !(0.0..90.0).contains(&max_slope_deg)
        {
            return Err(NavStackError::InvalidConfig(
                "invalid calibrated surface navigation geometry".into(),
            ));
        }
        let geometry = Config {
            world_frame: frame,
            voxel_size: voxel_size_m as f32,
            robot_height: robot_height_m as f32,
            // 生产支撑面不能靠形态学补出原本没有观测证据的地面。
            surface_closing_radius: 0.0,
            node_spacing_m: (voxel_size_m * 3.0) as f32,
            wall_clearance_m: (robot_width_m * 0.5) as f32,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            step_threshold_m: max_step_height_m as f32,
            step_penalty_weight: 2.0,
            max_slope_deg: max_slope_deg as f32,
            min_step_tread_depth_m: min_stair_tread_m as f32,
            goal_tolerance: 0.2,
            viz_publish_hz: 0.0,
        };
        if !(body_to_support_z_m as f32).is_normal()
            || !geometry.voxel_size.is_normal()
            || !geometry.robot_height.is_normal()
            || !geometry.wall_clearance_m.is_normal()
            || !geometry.min_step_tread_depth_m.is_normal()
            || !geometry.step_threshold_m.is_finite()
            || !geometry.node_spacing_m.is_normal()
            || geometry.min_step_tread_depth_m < geometry.voxel_size * 2.0
        {
            return Err(NavStackError::InvalidConfig(
                "surface geometry exceeds float32 domain or tread is below two voxels".into(),
            ));
        }
        Ok(Self {
            geometry,
            follower: SurfaceFollowerConfig::new(body_to_support_z_m),
            robot_width_m,
        })
    }
}

fn configured_number(key: &str, default: f64) -> NavStackResult<f64> {
    match std::env::var(key) {
        Ok(value) if !value.trim().is_empty() => parse_number(key, &value),
        _ => Ok(default),
    }
}

fn parse_number(key: &str, value: &str) -> NavStackResult<f64> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| NavStackError::InvalidConfig(format!("{key} must be a finite number")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_and_calibration_are_validated_before_starting_either_owner() {
        for (frame, offset, voxel, tread, step) in [
            ("map", 0.0, 0.1, 0.2, 0.2),
            ("odom", 0.4, 0.1, 0.2, 0.2),
            ("map", 0.4, 0.1, 0.1, 0.2),
            ("map", 1e300, 0.1, 0.2, 0.2),
            ("map", 0.4, 0.1, 0.2, 1e300),
        ] {
            assert!(SurfaceNavigationConfig::new(
                frame.into(),
                offset,
                0.3,
                0.6,
                voxel,
                step,
                35.0,
                tread
            )
            .is_err());
        }
        let valid =
            SurfaceNavigationConfig::new("map".into(), 0.4, 0.3, 0.6, 0.1, 0.2, 35.0, 0.2).unwrap();
        assert_eq!(valid.geometry.wall_clearance_m, 0.15);
        assert_eq!(valid.follower.body_to_support_z_m, 0.4);
        assert_eq!(valid.geometry.surface_closing_radius, 0.0);
    }
}
