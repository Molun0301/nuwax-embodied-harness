//! 使用统一机体轮廓检查路径前方的占据。

use crate::helpers::{Path2D, Point2};
use crate::navigation_map::{CostMap2D, OCCUPIED};
use nuwax_nav_types_messages::RobotGeometry;
use std::collections::HashSet;
// 注：本文件不直接调用 make_* helpers; Path2D 由调用方提供(make_path2d 构造)

/// 路径前向障碍检测
#[derive(Debug, Clone)]
pub struct PathClearance {
    path: Path2D,
    path_lookup_distance: f64,
    costmap: Option<CostMap2D>,
    pose_index: usize,
    path_start: Option<Point2>,
    robot_geometry: RobotGeometry,
    robot_pose: Option<(Point2, f64)>,
    rotation_target: Option<f64>,
}

impl PathClearance {
    /// 构造
    ///
    /// # 参数
    /// - `path`：路径
    /// - `robot_geometry`：统一机器人外形
    /// - `path_lookup_distance`：从当前 pose_index 向前的查找距离（米，默认 3.0）
    #[must_use]
    pub fn new(path: Path2D, robot_geometry: RobotGeometry, path_lookup_distance: f64) -> Self {
        Self {
            path,
            robot_geometry,
            path_lookup_distance,
            costmap: None,
            pose_index: 0,
            path_start: None,
            robot_pose: None,
            rotation_target: None,
        }
    }

    /// 当前已占据区域随真实位置和朝向重新确定，不累积清除历史。
    pub fn update_robot_pose(&mut self, position: Point2, yaw: f64) {
        self.robot_pose = Some((position, yaw));
    }

    /// 初始/最终原地旋转使用控制器本轮的实际目标朝向。
    pub fn update_rotation_target(&mut self, target: Option<f64>) {
        self.rotation_target = target;
    }

    /// 更新 costmap
    pub fn update_costmap(&mut self, costmap: CostMap2D) {
        self.costmap = Some(costmap);
    }

    /// 更新 pose_index（当前路径上的进度）
    pub fn update_pose_index(&mut self, index: usize) {
        self.pose_index = index;
        self.path_start = None;
    }

    /// 从机器人在线段上的实际投影更新前瞻起点，支持稀疏路径。
    pub fn update_path_progress(&mut self, index: usize, projected_position: Point2) {
        self.pose_index = index;
        self.path_start = Some(projected_position);
    }

    /// 是否前方出现已观测障碍；未知不是障碍类别，路径可达性由全局规划负责。
    #[must_use]
    pub fn is_obstacle_ahead(&mut self) -> bool {
        let Some(costmap) = self.costmap.as_ref() else {
            return true;
        };
        let Some(segments) = self.lookahead_segments() else {
            return true;
        };
        if segments.iter().any(|(start, end)| {
            [start, end].into_iter().any(|point| {
                let (x, y) = costmap.world_to_cell(*point);
                !costmap.in_bounds(x, y)
            })
        }) {
            // 图外不能因为栅格绘制被裁剪而变成“没有障碍”。
            return true;
        }
        let Some((mask, occupied_now)) = self.compute_geometry_mask() else { return true; };
        let Some(costmap) = self.costmap.as_ref() else {
            return true;
        };
        let width = costmap.width();
        let height = costmap.height();
        for y in 0..height {
            for x in 0..width {
                if mask[y * width + x]
                    && !occupied_now.contains(&(x, y))
                    && costmap.get(x, y) == OCCUPIED
                {
                    return true;
                }
            }
        }
        false
    }

    /// 对实际将覆盖的路径区域检查环境，自体当前区域仅作为已占据位置。
    fn compute_geometry_mask(&self) -> Option<(Vec<bool>, HashSet<(usize, usize)>)> {
        let geometry = &self.robot_geometry;
        let costmap = self.costmap.as_ref()?;
        let (position, yaw) = self.robot_pose?;
        let width = u32::try_from(costmap.width()).ok()?;
        let height = u32::try_from(costmap.height()).ok()?;
        let resolution = costmap.resolution();
        let origin = costmap.cell_center_world(0, 0);
        let origin = Point2::new(origin.x - resolution * 0.5, origin.y - resolution * 0.5);
        let current = geometry
            .grid_footprint(resolution, yaw, width, height)
            .ok()?
            .at_position(origin, position)
            .ok()?;
        let anchor = costmap.world_to_cell(position);
        let occupied_now: HashSet<(usize, usize)> = current
            .offsets
            .iter()
            .filter_map(|&(dx, dy)| {
                let x = i64::from(anchor.0).checked_add(dx)?;
                let y = i64::from(anchor.1).checked_add(dy)?;
                (x >= 0 && y >= 0 && x < i64::from(width) && y < i64::from(height))
                    .then_some((x as usize, y as usize))
            })
            .collect();
        let mut segments = self.lookahead_segments()?;
        if let Some((first, _)) = segments.first().copied() {
            if first != position {
                segments.insert(0, (position, first));
            }
        }
        let mut mask = vec![false; costmap.width() * costmap.height()];
        let mut visit = |x: u32, y: u32| {
            mask[y as usize * costmap.width() + x as usize] = true;
            true
        };
        if let Some(target) = self.rotation_target {
            return current
                .visit_motion(origin, (position, yaw), (position, target), &mut visit)
                .then_some((mask, occupied_now));
        }
        let mut previous_yaw = yaw;
        for (start, end) in segments {
            let length = (end.x - start.x).hypot(end.y - start.y);
            let heading = if length > 1e-9 {
                (end.y - start.y).atan2(end.x - start.x)
            } else {
                previous_yaw
            };
            if !current.visit_motion(origin, (start, previous_yaw), (end, heading), &mut visit) {
                return None;
            }
            previous_yaw = heading;
        }
        Some((mask, occupied_now))
    }

    fn lookahead_segments(&self) -> Option<Vec<(Point2, Point2)>> {
        let points = &self.path.points;
        if points.is_empty() || !self.path_lookup_distance.is_finite() {
            return None;
        }
        let start_index = self.pose_index.min(points.len() - 1);
        let mut start = self.path_start.unwrap_or(points[start_index]);
        if !start.x.is_finite() || !start.y.is_finite() {
            return None;
        }
        let mut remaining = self.path_lookup_distance.max(0.0);
        let mut segments = vec![(start, start)];
        for end in &points[start_index + 1..] {
            if remaining <= 0.0 {
                break;
            }
            let length = (end.x - start.x).hypot(end.y - start.y);
            if !length.is_finite() {
                return None;
            }
            if length > f64::EPSILON {
                let drawn = remaining.min(length);
                let ratio = drawn / length;
                let clipped = Point2::new(
                    start.x + (end.x - start.x) * ratio,
                    start.y + (end.y - start.y) * ratio,
                );
                if !clipped.x.is_finite() || !clipped.y.is_finite() {
                    return None;
                }
                segments.push((start, clipped));
                remaining -= drawn;
            }
            start = *end;
        }
        Some(segments)
    }

    /// 当前 path 引用（只读）
    #[must_use]
    pub fn path(&self) -> &Path2D {
        &self.path
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::make_path2d;
    use crate::navigation_map::CostMap2D;

    fn test_geometry() -> RobotGeometry { nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() } }

    fn path_through_straight_line() -> Path2D {
        make_path2d((0..=10).map(|i| Point2::new(i as f64 * 0.5, 0.0)).collect())
    }

    #[test]
    fn local_unknown_value_is_not_a_positive_obstacle_observation() {
        use crate::navigation_map::UNKNOWN;
        let path = make_path2d(vec![Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)]);
        let mut checker = PathClearance::new(path, test_geometry(), 1.0);
        let mut map = CostMap2D::new_all_free("map", 40, 40, 0.1);
        map.set(15, 10, UNKNOWN);
        checker.update_robot_pose(Point2::new(1.0, 1.0), 0.0);
        checker.update_costmap(map.clone());
        assert!(
            !checker.is_obstacle_ahead(),
            "未知255不能因大于100被误作已观测障碍"
        );
        map.set(15, 10, OCCUPIED);
        checker.update_costmap(map);
        assert!(checker.is_obstacle_ahead());
    }

    #[test]
    fn model_footprint_ignores_current_body_and_checks_new_translation_and_rotation() {
        let geometry = RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.1],[0.3,-0.1],[0.3,0.1],[-0.3,0.1]],"z_min_m":-0.3,"z_max_m":0.1}"#).unwrap();
        let position = Point2::new(1.025, 1.025);
        let mut map = CostMap2D::new_all_free("map", 80, 80, 0.05);
        let footprint = geometry.grid_footprint(0.05, 0.0, 80, 80).unwrap();
        for &(dx, dy) in &footprint.offsets {
            map.set((20 + dx) as usize, (20 + dy) as usize, OCCUPIED);
        }
        let mut checker = PathClearance::new(
            make_path2d(vec![position, Point2::new(2.025, 1.025)]),
            geometry.clone(),
            1.0,
        );
        checker.update_robot_pose(position, 0.0);
        checker.update_costmap(map.clone());
        assert!(
            !checker.is_obstacle_ahead(),
            "当前自体回波不应造成刚起步就停车"
        );
        map.set(32, 20, OCCUPIED);
        checker.update_costmap(map);
        assert!(
            checker.is_obstacle_ahead(),
            "前方新增占用仍须由外形扫过区域检测"
        );
        let mut map = CostMap2D::new_all_free("map", 80, 80, 0.05);
        map.set(24, 24, OCCUPIED);
        let mut turn = PathClearance::new(
            make_path2d(vec![position, Point2::new(1.025, 2.025)]),
            geometry.clone(),
            1.0,
        );
        turn.update_robot_pose(position, 0.0);
        turn.update_costmap(map);
        assert!(turn.is_obstacle_ahead(), "转向中新增覆盖的角部空间仍需检查");
    }

    #[test]
    fn is_obstacle_ahead_returns_true_without_costmap() {
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, test_geometry(), 2.0);
        assert!(pc.is_obstacle_ahead());
    }

    #[test]
    fn changed_origin_or_resolution_rebuilds_the_mask_for_new_obstacles() {
        for (resolution, origin_x) in [(1.0, -10.0), (0.5, 0.0)] {
            let path = make_path2d(vec![Point2::new(1.5, 1.5), Point2::new(4.5, 1.5)]);
            let mut pc = PathClearance::new(path, test_geometry(), 3.0);
            pc.update_robot_pose(pc.path.points[0], 0.0);
        pc.update_costmap(CostMap2D::new_all_free("map", 30, 10, 1.0));
            assert!(!pc.is_obstacle_ahead());

            let mut changed = CostMap2D::new_all_free("map", 30, 10, resolution);
            changed.set_origin(Point2::new(origin_x, 0.0));
            let (x, y) = changed.world_to_cell(Point2::new(2.5, 1.5));
            changed.set(x as usize, y as usize, OCCUPIED);
            pc.update_robot_pose(pc.path.points[0], 0.0);
        pc.update_costmap(changed);
            assert!(pc.is_obstacle_ahead(), "变化后的世界坐标障碍必须落入新掩膜");
        }
    }

    #[test]
    fn sparse_path_obstacle_lookahead_advances_with_projected_position() {
        let path = make_path2d(vec![Point2::new(0.5, 0.5), Point2::new(6.5, 0.5)]);
        let mut pc = PathClearance::new(path, test_geometry(), 3.0);
        let mut map = CostMap2D::new_all_free("map", 10, 3, 1.0);
        map.set(4, 0, OCCUPIED);
        pc.update_robot_pose(pc.path.points[0], 0.0);
        pc.update_costmap(map);
        pc.update_path_progress(0, Point2::new(0.5, 0.5));
        assert!(!pc.is_obstacle_ahead());
        pc.update_robot_pose(Point2::new(1.5, 0.5), 0.0);
        pc.update_path_progress(0, Point2::new(1.5, 0.5));
        assert!(pc.is_obstacle_ahead());
    }

    #[test]
    fn outside_map_lookahead_is_blocked_but_distant_outside_endpoint_is_not() {
        for (start, end, blocked) in [
            (Point2::new(-2.0, 0.5), Point2::new(-1.0, 0.5), true),
            (Point2::new(3.5, 0.5), Point2::new(8.5, 0.5), true),
            (Point2::new(0.5, 0.5), Point2::new(8.5, 0.5), false),
        ] {
            let mut pc = PathClearance::new(make_path2d(vec![start, end]), test_geometry(), 3.0);
            pc.update_robot_pose(pc.path.points[0], 0.0);
        pc.update_costmap(CostMap2D::new_all_free("map", 5, 3, 1.0));
            assert_eq!(pc.is_obstacle_ahead(), blocked);
        }
    }

    #[test]
    fn mask_respects_path_lookup_distance() {
        // obstacle 远离 path_lookup_distance：不应被检测
        let mut cm = CostMap2D::new_all_free("map", 60, 20, 0.1);
        cm.set_origin(Point2::new(-0.5, -0.5));
        cm.set(50, 5, OCCUPIED);
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, test_geometry(), 2.0);
        pc.update_robot_pose(pc.path.points[0], 0.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(!pc.is_obstacle_ahead());
    }

}
