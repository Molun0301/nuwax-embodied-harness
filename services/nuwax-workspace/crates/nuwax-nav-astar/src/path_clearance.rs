//! 路径前向障碍检测
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/path_clearance.py`。
//!
//! Phase 1 实现：
//! - `PathClearance` 缓存最近一次 costmap 与 pose_index。
//! - `mask`：根据 `robot_width`（米）计算圆盘膨胀（cell 单位），并结合 `path_lookup_distance`
//!   限制只覆盖路径前向 `path_lookup_distance` 米。
//! - `is_obstacle_ahead()`：检查 mask 覆盖的 cell 是否有 OCCUPIED。
//!
//! 与 Python `make_path_mask` 的真实实现相比，本阶段使用等效几何：圆盘 mask + 路径段 clip。
//! 复杂 `make_path_mask`（涉及 rasterize / rotation）放在后续 PR 与 mapping 协作完成。

use crate::helpers::{Path2D, Point2};
use crate::navigation_map::{CostMap2D, OCCUPIED};
use nuwax_nav_types_messages::RobotGeometry;
use std::collections::HashSet;
// 注：本文件不直接调用 make_* helpers; Path2D 由调用方提供(make_path2d 构造)

/// 路径前向障碍检测
#[derive(Debug, Clone)]
pub struct PathClearance {
    path: Path2D,
    robot_width: f64,
    path_lookup_distance: f64,
    costmap: Option<CostMap2D>,
    pose_index: usize,
    path_start: Option<Point2>,
    last_mask: Option<Vec<bool>>,
    robot_geometry: Option<RobotGeometry>,
    robot_pose: Option<(Point2, f64)>,
    rotation_target: Option<f64>,
}

impl PathClearance {
    /// 构造
    ///
    /// # 参数
    /// - `path`：路径
    /// - `robot_width`：机器人宽度（米），用于圆盘膨胀半径
    /// - `path_lookup_distance`：从当前 pose_index 向前的查找距离（米，默认 3.0）
    #[must_use]
    pub fn new(path: Path2D, robot_width: f64, path_lookup_distance: f64) -> Self {
        Self {
            path,
            robot_width,
            path_lookup_distance,
            costmap: None,
            pose_index: 0,
            path_start: None,
            last_mask: None,
            robot_geometry: None,
            robot_pose: None,
            rotation_target: None,
        }
    }

    /// 设置来自统一模型/设置的外形，后续前瞻使用真实机体包络。
    pub fn set_robot_geometry(&mut self, geometry: RobotGeometry) {
        self.robot_geometry = Some(geometry);
        self.last_mask = None;
    }

    /// 当前已占据区域随真实位置和朝向重新确定，不累积清除历史。
    pub fn update_robot_pose(&mut self, position: Point2, yaw: f64) {
        if self.robot_pose != Some((position, yaw)) {
            self.last_mask = None;
        }
        self.robot_pose = Some((position, yaw));
    }

    /// 初始/最终原地旋转使用控制器本轮的实际目标朝向。
    pub fn update_rotation_target(&mut self, target: Option<f64>) {
        self.rotation_target = target;
    }

    /// 更新 costmap
    pub fn update_costmap(&mut self, costmap: CostMap2D) {
        if self.costmap.as_ref().is_some_and(|previous| {
            previous.width() != costmap.width()
                || previous.height() != costmap.height()
                || previous.resolution() != costmap.resolution()
                || previous.frame_id() != costmap.frame_id()
                || previous.cell_center_world(0, 0) != costmap.cell_center_world(0, 0)
        }) {
            // 掩膜是栅格索引，原点或分辨率变化后即使尺寸相同也必须重算。
            self.last_mask = None;
        }
        self.costmap = Some(costmap);
    }

    /// 更新 pose_index（当前路径上的进度）
    pub fn update_pose_index(&mut self, index: usize) {
        if self.pose_index != index || self.path_start.is_some() {
            self.last_mask = None;
        }
        self.pose_index = index;
        self.path_start = None;
    }

    /// 从机器人在线段上的实际投影更新前瞻起点，支持稀疏路径。
    pub fn update_path_progress(&mut self, index: usize, projected_position: Point2) {
        if self.pose_index != index || self.path_start != Some(projected_position) {
            self.last_mask = None;
        }
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
        let (mask, occupied_now) = if self.robot_geometry.is_some() {
            let Some(value) = self.compute_geometry_mask() else {
                return true;
            };
            value
        } else {
            (self.compute_mask(), HashSet::new())
        };
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
        let geometry = self.robot_geometry.as_ref()?;
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

    /// 计算 mask（bool grid）
    ///
    /// 算法：
    /// - 取从 `pose_index` 开始、累积距离 `≤ path_lookup_distance` 的路径段。
    /// - 在路径段每个 cell 周围画半径 `robot_width / 2 / resolution` 的圆盘（cell 单位）。
    /// - 合并成 mask。
    #[must_use]
    pub fn compute_mask(&mut self) -> Vec<bool> {
        let Some(costmap) = self.costmap.as_ref() else {
            return Vec::new();
        };
        if let Some(mask) = self.last_mask.as_ref() {
            return mask.clone();
        }

        let width = costmap.width();
        let height = costmap.height();
        let mut mask = vec![false; width * height];
        // Bound the radius to a representable integer. `stamp_disk` clips its
        // iteration to the map, so a very small map resolution cannot turn a
        // malformed but finite input into an unbounded loop.
        let radius_value = (self.robot_width * 0.5) / costmap.resolution();
        let radius_cells = if !radius_value.is_finite() {
            if radius_value.is_sign_positive() {
                i32::MAX
            } else {
                0
            }
        } else {
            radius_value.ceil().clamp(0.0, f64::from(i32::MAX)) as i32
        };
        let r2 = i128::from(radius_cells) * i128::from(radius_cells);

        if let Some(segments) = self.lookahead_segments() {
            for (start, end) in segments {
                self.stamp_line(&mut mask, costmap, &start, &end, radius_cells, r2);
            }
        }

        self.last_mask = Some(mask.clone());
        mask
    }

    #[allow(clippy::too_many_arguments)]
    fn stamp_line(
        &self,
        mask: &mut [bool],
        costmap: &CostMap2D,
        start: &Point2,
        end: &Point2,
        radius: i32,
        r2: i128,
    ) {
        let Some((start_x, start_y, end_x, end_y)) =
            Self::clip_cell_segment(costmap, start, end, radius)
        else {
            return;
        };
        // Degenerate map metadata can produce a footprint wider than the
        // entire map. Once the centerline touches the expanded bounds, mark
        // the map conservatively instead of rasterizing billions of cells.
        let radius_cells = usize::try_from(radius.max(0)).unwrap_or(usize::MAX);
        if radius_cells > costmap.width().max(costmap.height()) {
            mask.fill(true);
            return;
        }
        let (mut x, mut y) = (i64::from(start_x), i64::from(start_y));
        let (end_x, end_y) = (i64::from(end_x), i64::from(end_y));
        let dx = (end_x - x).abs();
        let step_x = if x < end_x { 1 } else { -1 };
        let dy = -(end_y - y).abs();
        let step_y = if y < end_y { 1 } else { -1 };
        let mut error = dx + dy;

        loop {
            if let (Ok(cx), Ok(cy)) = (i32::try_from(x), i32::try_from(y)) {
                self.stamp_disk(mask, costmap.width(), costmap.height(), cx, cy, radius, r2);
            }
            if x == end_x && y == end_y {
                break;
            }
            let twice_error = 2 * error;
            if twice_error >= dy {
                error += dy;
                x += step_x;
            }
            if twice_error <= dx {
                error += dx;
                y += step_y;
            }
        }
    }

    /// Convert a world-space segment to cells and clip it before Bresenham.
    /// Without this bound, a finite path far outside the map could require
    /// billions of iterations before reaching the first in-bounds cell.
    fn clip_cell_segment(
        costmap: &CostMap2D,
        start: &Point2,
        end: &Point2,
        radius: i32,
    ) -> Option<(i32, i32, i32, i32)> {
        if !start.x.is_finite()
            || !start.y.is_finite()
            || !end.x.is_finite()
            || !end.y.is_finite()
            || costmap.width() == 0
            || costmap.height() == 0
        {
            return None;
        }

        let (start_x, start_y) = costmap.world_to_cell(*start);
        let (end_x, end_y) = costmap.world_to_cell(*end);
        let max_x = i32::try_from(costmap.width() - 1).ok()?;
        let max_y = i32::try_from(costmap.height() - 1).ok()?;

        // Liang-Barsky clipping in cell coordinates keeps the subsequent
        // integer rasterization bounded by the map plus robot footprint.
        let x0 = f64::from(start_x);
        let y0 = f64::from(start_y);
        let dx = f64::from(end_x) - x0;
        let dy = f64::from(end_y) - y0;
        let radius = f64::from(radius.max(0));
        let min_x = -radius;
        let min_y = -radius;
        let max_x_bound = f64::from(max_x) + radius;
        let max_y_bound = f64::from(max_y) + radius;
        let mut t_min: f64 = 0.0;
        let mut t_max: f64 = 1.0;

        for (p, q) in [
            (-dx, x0 - min_x),
            (dx, max_x_bound - x0),
            (-dy, y0 - min_y),
            (dy, max_y_bound - y0),
        ] {
            if p.abs() <= f64::EPSILON {
                if q < 0.0 {
                    return None;
                }
                continue;
            }
            let ratio = q / p;
            if p < 0.0 {
                if ratio > t_max {
                    return None;
                }
                t_min = t_min.max(ratio);
            } else {
                if ratio < t_min {
                    return None;
                }
                t_max = t_max.min(ratio);
            }
        }

        if t_min > t_max {
            return None;
        }
        let clamp_cell = |value: f64| {
            value
                .round()
                .clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32
        };
        Some((
            clamp_cell(x0 + dx * t_min),
            clamp_cell(y0 + dy * t_min),
            clamp_cell(x0 + dx * t_max),
            clamp_cell(y0 + dy * t_max),
        ))
    }

    #[allow(clippy::too_many_arguments)] // 函数为内联热路径,分组会增加栈分配
    fn stamp_disk(
        &self,
        mask: &mut [bool],
        width: usize,
        height: usize,
        cx: i32,
        cy: i32,
        radius: i32,
        r2: i128,
    ) {
        if width == 0 || height == 0 {
            return;
        }
        let radius = i64::from(radius.max(0));
        let cx = i64::from(cx);
        let cy = i64::from(cy);
        let max_x = i64::try_from(width - 1).unwrap_or(i64::MAX);
        let max_y = i64::try_from(height - 1).unwrap_or(i64::MAX);
        let min_x = (cx - radius).max(0).min(max_x);
        let max_x = (cx + radius).max(0).min(max_x);
        let min_y = (cy - radius).max(0).min(max_y);
        let max_y = (cy + radius).max(0).min(max_y);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let dx = x - cx;
                let dy = y - cy;
                if i128::from(dx) * i128::from(dx) + i128::from(dy) * i128::from(dy) > r2 {
                    continue;
                }
                mask[y as usize * width + x as usize] = true;
            }
        }
    }

    /// 返回裁剪到前瞻长度的连续线段；非法坐标不能参与安全判断。
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

    fn path_through_straight_line() -> Path2D {
        make_path2d((0..=10).map(|i| Point2::new(i as f64 * 0.5, 0.0)).collect())
    }

    #[test]
    fn local_unknown_value_is_not_a_positive_obstacle_observation() {
        use crate::navigation_map::UNKNOWN;
        let path = make_path2d(vec![Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)]);
        let mut checker = PathClearance::new(path, 0.3, 1.0);
        let mut map = CostMap2D::new_all_free("map", 40, 40, 0.1);
        map.set(15, 10, UNKNOWN);
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
            0.3,
            1.0,
        );
        checker.set_robot_geometry(geometry.clone());
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
            0.3,
            1.0,
        );
        turn.set_robot_geometry(geometry);
        turn.update_robot_pose(position, 0.0);
        turn.update_costmap(map);
        assert!(turn.is_obstacle_ahead(), "转向中新增覆盖的角部空间仍需检查");
    }

    #[test]
    fn is_obstacle_ahead_returns_false_when_clear() {
        let cm = CostMap2D::new_all_free("map", 30, 10, 0.1);
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, 0.3, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(!pc.is_obstacle_ahead());
    }

    #[test]
    fn is_obstacle_ahead_returns_true_when_occupied() {
        let mut cm = CostMap2D::new_all_free("map", 30, 10, 0.1);
        // 在路径前向 1.5m 处放 obstacle（cell 15,5 = world (1.55, 0.55)）
        cm.set(15, 5, OCCUPIED);
        let path = path_through_straight_line();
        // robot_width=1.2 让 disk 半径覆盖 y=5
        let mut pc = PathClearance::new(path, 1.2, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(pc.is_obstacle_ahead());
    }

    #[test]
    fn is_obstacle_ahead_returns_true_without_costmap() {
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, 0.3, 2.0);
        assert!(pc.is_obstacle_ahead());
    }

    #[test]
    fn changed_origin_or_resolution_rebuilds_the_mask_for_new_obstacles() {
        for (resolution, origin_x) in [(1.0, -10.0), (0.5, 0.0)] {
            let path = make_path2d(vec![Point2::new(1.5, 1.5), Point2::new(4.5, 1.5)]);
            let mut pc = PathClearance::new(path, 0.0, 3.0);
            pc.update_costmap(CostMap2D::new_all_free("map", 30, 10, 1.0));
            assert!(!pc.is_obstacle_ahead());

            let mut changed = CostMap2D::new_all_free("map", 30, 10, resolution);
            changed.set_origin(Point2::new(origin_x, 0.0));
            let (x, y) = changed.world_to_cell(Point2::new(2.5, 1.5));
            changed.set(x as usize, y as usize, OCCUPIED);
            pc.update_costmap(changed);
            assert!(pc.is_obstacle_ahead(), "变化后的世界坐标障碍必须落入新掩膜");
        }
    }

    #[test]
    fn sparse_path_obstacle_lookahead_advances_with_projected_position() {
        let path = make_path2d(vec![Point2::new(0.5, 0.5), Point2::new(6.5, 0.5)]);
        let mut pc = PathClearance::new(path, 0.0, 3.0);
        let mut map = CostMap2D::new_all_free("map", 10, 3, 1.0);
        map.set(4, 0, OCCUPIED);
        pc.update_costmap(map);
        pc.update_path_progress(0, Point2::new(0.5, 0.5));
        assert!(!pc.is_obstacle_ahead());
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
            let mut pc = PathClearance::new(make_path2d(vec![start, end]), 0.0, 3.0);
            pc.update_costmap(CostMap2D::new_all_free("map", 5, 3, 1.0));
            assert_eq!(pc.is_obstacle_ahead(), blocked);
        }
    }

    #[test]
    fn mask_respects_path_lookup_distance() {
        // obstacle 远离 path_lookup_distance：不应被检测
        let mut cm = CostMap2D::new_all_free("map", 60, 10, 0.1);
        cm.set(50, 5, OCCUPIED);
        let path = path_through_straight_line();
        let mut pc = PathClearance::new(path, 0.3, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);
        assert!(!pc.is_obstacle_ahead());
    }

    #[test]
    fn sparse_segment_mask_stops_at_the_lookup_cutoff() {
        let mut cm = CostMap2D::new_all_free("map", 60, 10, 0.1);
        cm.set(50, 0, OCCUPIED);
        let path = make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(5.0, 0.0)]);
        let mut pc = PathClearance::new(path, 0.0, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(0);

        let mask = pc.compute_mask();
        assert!(mask[10], "a 1m obstacle must be inside the lookahead mask");
        assert!(mask[20], "the exact 2m cutoff must remain inside the mask");
        assert!(!mask[50], "the 5m endpoint must remain outside the mask");
        assert!(!pc.is_obstacle_ahead());
    }

    #[test]
    fn rasterization_clips_a_far_segment_to_the_costmap() {
        let cm = CostMap2D::new_all_free("map", 10, 3, 1.0);
        let path = make_path2d(vec![
            Point2::new(-1_000_000_000.0, 1.5),
            Point2::new(1_000_000_000.0, 1.5),
        ]);
        let mut pc = PathClearance::new(path, 0.0, 2_000_000_000.0);
        pc.update_costmap(cm);

        let mask = pc.compute_mask();
        assert!((0..10).all(|x| mask[10 + x]));
        assert_eq!(mask.iter().filter(|marked| **marked).count(), 10);
    }

    #[test]
    fn footprint_detects_a_path_centerline_just_outside_the_map() {
        let cm = CostMap2D::new_all_free("map", 5, 5, 1.0);
        let path = make_path2d(vec![Point2::new(-1.0, 0.0), Point2::new(-1.0, 4.0)]);
        let mut pc = PathClearance::new(path, 2.0, 4.0);
        pc.update_costmap(cm);

        let mask = pc.compute_mask();
        assert!(
            (0..5).all(|y| mask[y * 5]),
            "footprint must reach map column 0"
        );

        let far_cm = CostMap2D::new_all_free("map", 5, 5, 1.0);
        let far_path = make_path2d(vec![Point2::new(-2.0, 0.0), Point2::new(-2.0, 4.0)]);
        let mut far_pc = PathClearance::new(far_path, 2.0, 4.0);
        far_pc.update_costmap(far_cm);
        assert!(!far_pc.compute_mask().iter().any(|marked| *marked));
    }

    #[test]
    fn rasterization_handles_negative_origin_and_repeated_points() {
        let mut cm = CostMap2D::new_all_free("map", 5, 3, 1.0);
        cm.set_origin(Point2::new(-5.0, -5.0));
        let path = make_path2d(vec![
            Point2::new(-10.0, -4.5),
            Point2::new(-10.0, -4.5),
            Point2::new(-2.0, -4.5),
        ]);
        let mut pc = PathClearance::new(path, 0.0, 8.0);
        pc.update_costmap(cm);

        let mask = pc.compute_mask();
        assert!((0..=3).all(|x| mask[x]));
        assert_eq!(mask.iter().filter(|marked| **marked).count(), 4);
    }

    #[test]
    fn mask_keeps_the_current_cell_at_the_last_path_index() {
        let cm = CostMap2D::new_all_free("map", 5, 1, 1.0);
        let path = make_path2d(vec![Point2::new(0.0, 0.0), Point2::new(4.0, 0.0)]);
        let mut pc = PathClearance::new(path, 0.0, 2.0);
        pc.update_costmap(cm);
        pc.update_pose_index(1);

        let mask = pc.compute_mask();
        assert!(mask[4]);
        assert_eq!(mask.iter().filter(|marked| **marked).count(), 1);
    }

    #[test]
    fn invalid_length_and_tiny_resolution_remain_bounded() {
        let cm = CostMap2D::new_all_free("map", 3, 3, f64::MIN_POSITIVE);
        let path = make_path2d(vec![
            Point2::new(-f64::MAX, 0.0),
            Point2::new(f64::MAX, 0.0),
        ]);
        let mut pc = PathClearance::new(path, 0.1, 2.0);
        pc.update_costmap(cm);

        let mask = pc.compute_mask();
        assert!(!mask.iter().any(|marked| *marked));

        let tiny_cm = CostMap2D::new_all_free("map", 3, 3, f64::MIN_POSITIVE);
        let tiny_path = make_path2d(vec![Point2::new(0.0, 0.0)]);
        let mut tiny_pc = PathClearance::new(tiny_path, 0.1, 2.0);
        tiny_pc.update_costmap(tiny_cm);
        assert!(tiny_pc.compute_mask().iter().all(|marked| *marked));
    }
}
