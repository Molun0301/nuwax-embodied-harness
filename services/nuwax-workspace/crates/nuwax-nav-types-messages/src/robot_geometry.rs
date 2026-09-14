//! 机器人机体坐标外形，以及规划时随当前位置重新生成的栅格轮廓。

use serde::{Deserialize, Serialize};

use crate::geometry::Point2;
use crate::grid::{OccupancyCell, OccupancyGrid2D};

/// 由统一模型/用户设置解析得到的机器人外包络。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RobotGeometry {
    /// 机体坐标下的逆时针凸多边形，单位米。
    pub footprint_xy_m: Vec<[f64; 2]>,
    /// 机体坐标下最低点。
    pub z_min_m: f64,
    /// 机体坐标下最高点。
    pub z_max_m: f64,
    /// 模型或手填参数的来源说明。
    #[serde(default)]
    pub source: String,
}

/// 同一朝向下的离散机体轮廓，可复用于当前占据区和待进入位置。
#[derive(Debug, Clone)]
pub struct GridFootprint {
    /// 相对于机器人所在格的偏移。
    pub offsets: Vec<(i64, i64)>,
    geometry: RobotGeometry,
    resolution: f64,
    yaw: f64,
    map_width: u32,
    map_height: u32,
}

impl RobotGeometry {
    /// 解析启动入口物化的统一外形，拒绝不成立的几何描述。
    pub fn from_runtime_json(value: &str) -> Result<Self, String> {
        let geometry: Self =
            serde_json::from_str(value).map_err(|error| format!("机器人外形格式错误：{error}"))?;
        geometry.validate()?;
        Ok(geometry)
    }

    /// 检查有限性、正高度和凸包方向；不读取设备或模型文件。
    pub fn validate(&self) -> Result<(), String> {
        let vertices = &self.footprint_xy_m;
        if vertices.len() < 3
            || !self.z_min_m.is_finite()
            || !self.z_max_m.is_finite()
            || self.z_min_m >= self.z_max_m
            || vertices.iter().flatten().any(|value| !value.is_finite())
        {
            return Err("机器人外形需要有限的凸包顶点和正高度".into());
        }
        let mut area = 0.0;
        for index in 0..vertices.len() {
            let a = vertices[index];
            let b = vertices[(index + 1) % vertices.len()];
            let c = vertices[(index + 2) % vertices.len()];
            let turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
            if !turn.is_finite() || turn < -1e-10 {
                return Err("机器人平面外形必须是逆时针凸包".into());
            }
            area += a[0] * b[1] - a[1] * b[0];
        }
        if !area.is_finite() || area <= 1e-12 {
            return Err("机器人平面外形面积必须大于零".into());
        }
        Ok(())
    }

    /// 按真实朝向将机体外形栅格化；栅格分辨率决定离散误差。
    pub fn grid_footprint(
        &self,
        resolution: f64,
        yaw: f64,
        map_width: u32,
        map_height: u32,
    ) -> Result<GridFootprint, String> {
        self.grid_footprint_with_phase(resolution, yaw, map_width, map_height, [0.0, 0.0])
    }

    fn grid_footprint_with_phase(
        &self,
        resolution: f64,
        yaw: f64,
        map_width: u32,
        map_height: u32,
        phase: [f64; 2],
    ) -> Result<GridFootprint, String> {
        if !resolution.is_finite() || resolution <= 0.0 || !yaw.is_finite() {
            return Err("机器人轮廓需要有效分辨率和朝向".into());
        }
        let (sin, cos) = yaw.sin_cos();
        let vertices: Vec<[f64; 2]> = self
            .footprint_xy_m
            .iter()
            .map(|point| {
                [
                    cos * point[0] - sin * point[1] + phase[0],
                    sin * point[0] + cos * point[1] + phase[1],
                ]
            })
            .collect();
        let bounds = vertices.iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |mut bounds, point| {
                bounds[0] = bounds[0].min(point[0]);
                bounds[1] = bounds[1].min(point[1]);
                bounds[2] = bounds[2].max(point[0]);
                bounds[3] = bounds[3].max(point[1]);
                bounds
            },
        );
        let min_x = (bounds[0] / resolution - 0.5).ceil();
        let min_y = (bounds[1] / resolution - 0.5).ceil();
        let max_x = (bounds[2] / resolution + 0.5).floor();
        let max_y = (bounds[3] / resolution + 0.5).floor();
        if [min_x, min_y, max_x, max_y]
            .iter()
            .any(|value| !value.is_finite())
            || max_x - min_x > f64::from(map_width) + 1.0
            || max_y - min_y > f64::from(map_height) + 1.0
        {
            return Err("当前栅格范围不能容纳机器人外形".into());
        }
        let axes = polygon_axes(&vertices);
        let mut offsets = Vec::new();
        for y in min_y as i64..=max_y as i64 {
            for x in min_x as i64..=max_x as i64 {
                if axes_overlap_cell(
                    &axes,
                    x as f64 * resolution,
                    y as f64 * resolution,
                    resolution * 0.5,
                ) {
                    offsets.push((x, y));
                }
            }
        }
        Ok(GridFootprint {
            offsets,
            geometry: self.clone(),
            resolution,
            yaw,
            map_width,
            map_height,
        })
    }
}

impl GridFootprint {
    /// 生成此轮廓时的机体偏航，单位弧度。
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    /// 保留真实位姿在格内的偏移，不能把机器人吸附到格中心后扩大自体清除。
    pub fn at_position(&self, origin: Point2, position: Point2) -> Result<Self, String> {
        if !position.x.is_finite() || !position.y.is_finite() {
            return Err("机器人位置无效".into());
        }
        let x = ((position.x - origin.x) / self.resolution).floor();
        let y = ((position.y - origin.y) / self.resolution).floor();
        let phase = [
            position.x - origin.x - (x + 0.5) * self.resolution,
            position.y - origin.y - (y + 0.5) * self.resolution,
        ];
        self.geometry.grid_footprint_with_phase(
            self.resolution,
            self.yaw,
            self.map_width,
            self.map_height,
            phase,
        )
    }

    /// 检查任意真实位置的轮廓，而不是仅检查所在格中心。
    pub fn is_clear_at(&self, grid: &OccupancyGrid2D, position: Point2) -> bool {
        let Some((x, y)) = grid.world_to_cell(position) else {
            return false;
        };
        self.at_position(grid.origin, position)
            .is_ok_and(|footprint| footprint.is_clear(grid, x, y))
    }

    /// 按实际起始/目标朝向遍历转向和平移新覆盖的格，供规划与执行共同使用。
    pub fn visit_motion(
        &self,
        origin: Point2,
        from: (Point2, f64),
        to: (Point2, f64),
        mut visit: impl FnMut(u32, u32) -> bool,
    ) -> bool {
        let length = (to.0.x - from.0.x).hypot(to.0.y - from.0.y);
        if !length.is_finite() || !from.1.is_finite() || !to.1.is_finite() {
            return false;
        }
        let mut stamp = |point: Point2, yaw: f64| {
            let x = ((point.x - origin.x) / self.resolution).floor() as i64;
            let y = ((point.y - origin.y) / self.resolution).floor() as i64;
            // 直接生成真实格内相位轮廓，省去随后被丢弃的格中心轮廓。
            let phase = [
                point.x - origin.x - (x as f64 + 0.5) * self.resolution,
                point.y - origin.y - (y as f64 + 0.5) * self.resolution,
            ];
            let Ok(footprint) = self.geometry.grid_footprint_with_phase(
                self.resolution,
                yaw,
                self.map_width,
                self.map_height,
                phase,
            ) else {
                return false;
            };
            footprint.offsets.iter().all(|&(dx, dy)| {
                x.checked_add(dx)
                    .zip(y.checked_add(dy))
                    .and_then(|(x, y)| Some((u32::try_from(x).ok()?, u32::try_from(y).ok()?)))
                    .is_some_and(|(x, y)| x < self.map_width && y < self.map_height && visit(x, y))
            })
        };
        let turn = (to.1 - from.1).sin().atan2((to.1 - from.1).cos());
        let radius = self
            .geometry
            .footprint_xy_m
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f64::max);
        let turns = ((turn.abs() * radius / (self.resolution * 0.5)).ceil() as usize).max(1);
        for index in 0..=turns {
            if !stamp(from.0, from.1 + turn * index as f64 / turns as f64) {
                return false;
            }
        }
        let steps = ((length / (self.resolution * 0.5)).ceil() as usize).max(1);
        for index in 0..=steps {
            let factor = index as f64 / steps as f64;
            if !stamp(
                Point2::new(
                    from.0.x + (to.0.x - from.0.x) * factor,
                    from.0.y + (to.0.y - from.0.y) * factor,
                ),
                to.1,
            ) {
                return false;
            }
        }
        true
    }

    /// 只在调用方的规划副本中消除当前机体占据区域，不改写规范地图。
    pub fn clear_current_occupancy(&self, grid: &mut OccupancyGrid2D, position: Point2) {
        let Some((x, y)) = grid.world_to_cell(position) else {
            return;
        };
        let Ok(current) = self.at_position(grid.origin, position) else {
            return;
        };
        for &(dx, dy) in &current.offsets {
            let Some(x) = i64::from(x).checked_add(dx) else {
                continue;
            };
            let Some(y) = i64::from(y).checked_add(dy) else {
                continue;
            };
            if x >= 0 && y >= 0 {
                if let (Ok(x), Ok(y)) = (u32::try_from(x), u32::try_from(y)) {
                    grid.set(x, y, OccupancyCell::Free);
                }
            }
        }
    }

    /// 候选位置将覆盖的每个环境格是否可通行。
    pub fn is_clear(&self, grid: &OccupancyGrid2D, x: u32, y: u32) -> bool {
        self.offsets.iter().all(|&(dx, dy)| {
            i64::from(x)
                .checked_add(dx)
                .zip(i64::from(y).checked_add(dy))
                .and_then(|(x, y)| Some((u32::try_from(x).ok()?, u32::try_from(y).ok()?)))
                .and_then(|(x, y)| grid.get(x, y))
                .is_some_and(OccupancyCell::is_known_traversable)
        })
    }
}

fn polygon_axes(vertices: &[[f64; 2]]) -> Vec<([f64; 2], f64, f64)> {
    let axes = [[1.0, 0.0], [0.0, 1.0]]
        .into_iter()
        .chain((0..vertices.len()).map(|index| {
            let a = vertices[index];
            let b = vertices[(index + 1) % vertices.len()];
            [a[1] - b[1], b[0] - a[0]]
        }));
    axes.map(|axis| {
        let (low, high) =
            vertices
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), p| {
                    let projection = p[0] * axis[0] + p[1] * axis[1];
                    (low.min(projection), high.max(projection))
                });
        (axis, low, high)
    })
    .collect()
}

fn axes_overlap_cell(axes: &[([f64; 2], f64, f64)], x: f64, y: f64, half: f64) -> bool {
    axes.iter().all(|&(axis, low, high)| {
        let center = x * axis[0] + y * axis[1];
        let extent = half * (axis[0].abs() + axis[1].abs());
        high >= center - extent - 1e-10 && low <= center + extent + 1e-10
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body() -> RobotGeometry {
        RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.1],[0.3,-0.1],[0.3,0.1],[-0.3,0.1]],"z_min_m":-0.2,"z_max_m":0.2,"source":"测试外形"}"#).unwrap()
    }

    #[test]
    fn body_mask_rotates_and_clears_only_current_occupied_view() {
        let body = body();
        let footprint = body.grid_footprint(0.05, 0.0, 80, 80).unwrap();
        let turned = body
            .grid_footprint(0.05, std::f64::consts::FRAC_PI_2, 80, 80)
            .unwrap();
        assert!(footprint.offsets.contains(&(5, 0)));
        assert!(!footprint.offsets.contains(&(0, 5)));
        assert!(turned.offsets.contains(&(0, 5)));
        let source = OccupancyGrid2D::filled(
            Point2::new(-2.0, -2.0),
            0.05,
            80,
            80,
            OccupancyCell::Unknown,
        );
        let mut view = source.clone();
        let pose = Point2::new(0.025, 0.025);
        footprint.clear_current_occupancy(&mut view, pose);
        assert!(footprint.is_clear(&view, 40, 40));
        assert!(!footprint.is_clear(&view, 48, 40));
        assert_eq!(source.get(40, 40), Some(OccupancyCell::Unknown));
        assert_eq!(view.get(50, 40), Some(OccupancyCell::Unknown));
    }

    #[test]
    fn current_footprint_keeps_subcell_position_and_external_obstacle() {
        let geometry = RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.1],[0.34,-0.1],[0.34,0.1],[-0.3,0.1]],"z_min_m":-0.2,"z_max_m":0.2}"#).unwrap();
        let footprint = geometry.grid_footprint(0.05, 0.0, 80, 80).unwrap();
        let mut view =
            OccupancyGrid2D::filled(Point2::new(-1.0, -1.0), 0.05, 80, 80, OccupancyCell::Free);
        view.set(27, 20, OccupancyCell::Occupied);
        let position = Point2::new(0.001, 0.025);
        footprint.clear_current_occupancy(&mut view, position);
        assert_eq!(
            view.get(27, 20),
            Some(OccupancyCell::Occupied),
            "真实前缘0.341米不能清掉从0.35米开始的外部格"
        );
        assert!(footprint.is_clear_at(&view, position));
        assert!(!footprint.is_clear_at(&view, Point2::new(0.051, 0.025)));
    }

    #[test]
    fn malformed_body_geometry_is_rejected() {
        let mut geometry = body();
        geometry.footprint_xy_m.reverse();
        assert!(geometry.validate().is_err());
        let geometry = body();
        assert!(geometry.grid_footprint(0.0, 0.0, 20, 20).is_err());
        assert!(geometry.grid_footprint(0.01, 0.0, 2, 2).is_err());
    }
}
