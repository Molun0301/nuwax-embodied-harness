//! 在线与离线 Geometry 共用的持久 canonical occupancy 累加器。

use nuwax_mapping::OccupancyGrid;
use nuwax_nav_types_messages::grid::OCCUPANCY_OBSTACLE_THRESHOLD;

fn normalize_frame_id(frame_id: &str) -> String {
    frame_id.trim().trim_start_matches('/').to_string()
}

/// 占据类与导航共用阈值，避免导航判障碍的旧代价与新自由观测被误认为同类。
pub(crate) const OCCUPANCY_CHANGE_CONFIRMATIONS: u8 = 3;

/// Persistent 2D occupancy state independent from the bounded point-cloud
/// window.  The point-cloud window is useful for accumulating new geometry,
/// but it must not be the source of truth for already published cells because
/// it intentionally evicts old points.
#[derive(Debug, Clone)]
pub(crate) struct PersistentOccupancyGrid {
    pub(crate) grid: OccupancyGrid,
    pending_values: Vec<i8>,
    pending_counts: Vec<u8>,
    last_observation_timestamp: Option<f64>,
}

impl PersistentOccupancyGrid {
    /// 水平扫描只提供本帧真实射线，持久图在原拥有方内累积。
    pub(crate) fn stage_observation(current: Option<&Self>, latest: &OccupancyGrid) -> Result<Self, String> {
        let mut staged = match current {
            Some(current) => current.clone(),
            None => {
                let mut unknown = latest.clone();
                unknown.data.fill(-1);
                Self::from_grid(unknown)?
            }
        };
        staged.merge_observation(latest)?;
        Ok(staged)
    }

    /// 缺测不投票；同一源帧的再次物化也不能重复确认障碍或清除。
    pub(crate) fn merge_observation(&mut self, latest: &OccupancyGrid) -> Result<bool, String> {
        self.validate_compatible(latest)?;
        if !latest.ts.is_finite() {
            return Err("observation timestamp must be finite".into());
        }
        if self.last_observation_timestamp.is_some_and(|timestamp| latest.ts <= timestamp) {
            return Ok(false);
        }
        if latest.data.iter().all(|value| *value < 0) {
            self.grid.ts = latest.ts;
            self.last_observation_timestamp = Some(latest.ts);
            return Ok(false);
        }
        let mut changed = self.expand_to_include(latest)?;
        let projection = GridProjection::new(&self.grid, latest);
        for row in 0..latest.height {
            for col in 0..latest.width {
                let incoming = latest.data[latest.index(row, col)];
                if incoming < 0 { continue; }
                let target = projection.project(row, col)?;
                let existing = self.grid.data[target];
                let occupied = occupancy_class(incoming) != 0;
                if (existing < 0 && !occupied)
                    || (existing >= 0 && occupancy_class(existing) == occupancy_class(incoming)) {
                    changed |= existing != incoming;
                    self.grid.data[target] = incoming;
                    self.pending_values[target] = -1;
                    self.pending_counts[target] = 0;
                    continue;
                }
                // 障碍需两次真实命中，清除需三次真实穿越；中间缺测不充当反证。
                let count = if self.pending_counts[target] > 0
                    && occupancy_class(self.pending_values[target]) == occupancy_class(incoming) {
                    self.pending_counts[target].saturating_add(1)
                } else { 1 };
                self.pending_values[target] = incoming;
                self.pending_counts[target] = count;
                let required = if occupied { 2 } else { OCCUPANCY_CHANGE_CONFIRMATIONS };
                if count >= required {
                    self.grid.data[target] = incoming;
                    self.pending_values[target] = -1;
                    self.pending_counts[target] = 0;
                    changed = true;
                }
            }
        }
        self.grid.ts = latest.ts;
        self.last_observation_timestamp = Some(latest.ts);
        Ok(changed)
    }

    pub(crate) fn from_grid(grid: OccupancyGrid) -> Result<Self, String> {
        validate_checkpoint_grid(&grid, grid.frame_id.trim().trim_start_matches('/'))?;
        let cell_count = grid.data.len();
        Ok(Self {
            grid,
            pending_values: vec![-1; cell_count],
            pending_counts: vec![0; cell_count],
            // 保存资产的源时间属于旧会话，不能作为新来源的去重水位。
            last_observation_timestamp: None,
        })
    }

    fn validate_compatible(&self, incoming: &OccupancyGrid) -> Result<(), String> {
        validate_checkpoint_grid(incoming, self.grid.frame_id.trim().trim_start_matches('/'))?;
        if normalize_frame_id(&incoming.frame_id) != normalize_frame_id(&self.grid.frame_id) {
            return Err("persistent/live frames differ".to_string());
        }
        let tolerance = self.grid.resolution.abs().max(1.0) * 1e-6;
        if (incoming.resolution - self.grid.resolution).abs() > tolerance {
            return Err("persistent/live resolutions differ".to_string());
        }
        if incoming.origin_orientation != self.grid.origin_orientation {
            return Err("persistent/live origin orientations differ".to_string());
        }
        Ok(())
    }

    fn expand_to_include(&mut self, incoming: &OccupancyGrid) -> Result<bool, String> {
        let resolution = self.grid.resolution;
        let relative_min_col =
            snap_grid_index((incoming.origin[0] - self.grid.origin[0]) / resolution);
        let relative_min_row =
            snap_grid_index((incoming.origin[1] - self.grid.origin[1]) / resolution);
        let relative_max_col = snap_grid_index(
            (incoming.origin[0] + incoming.width as f64 * incoming.resolution
                - self.grid.origin[0])
                / resolution,
        );
        let relative_max_row = snap_grid_index(
            (incoming.origin[1] + incoming.height as f64 * incoming.resolution
                - self.grid.origin[1])
                / resolution,
        );
        let min_col = checked_grid_index(relative_min_col.min(0.0).floor())?;
        let min_row = checked_grid_index(relative_min_row.min(0.0).floor())?;
        let max_col = checked_grid_index(relative_max_col.max(self.grid.width as f64).ceil())?;
        let max_row = checked_grid_index(relative_max_row.max(self.grid.height as f64).ceil())?;
        let width = max_col
            .checked_sub(min_col)
            .ok_or_else(|| "persistent grid width underflow".to_string())?;
        let height = max_row
            .checked_sub(min_row)
            .ok_or_else(|| "persistent grid height underflow".to_string())?;
        if width <= 0 || height <= 0 || width > i64::from(u32::MAX) || height > i64::from(u32::MAX)
        {
            return Err("persistent grid dimensions are invalid".to_string());
        }
        let width = width as u32;
        let height = height as u32;
        if u64::from(width) * u64::from(height) > u64::from(u32::MAX) {
            return Err("persistent grid exceeds occupancy storage capacity".to_string());
        }
        let cell_count = (width as usize)
            .checked_mul(height as usize)
            .ok_or_else(|| "persistent grid dimensions overflow".to_string())?;

        if min_col == 0 && min_row == 0 && width == self.grid.width && height == self.grid.height {
            return Ok(false);
        }

        let mut expanded =
            OccupancyGrid::with_frame(width, height, self.grid.resolution, &self.grid.frame_id);
        expanded.origin = [
            self.grid.origin[0] + min_col as f64 * resolution,
            self.grid.origin[1] + min_row as f64 * resolution,
            self.grid.origin[2],
        ];
        expanded.origin_orientation = self.grid.origin_orientation;
        expanded.ts = self.grid.ts.max(incoming.ts);
        let mut pending_values = vec![-1; cell_count];
        let mut pending_counts = vec![0; cell_count];

        for row in 0..self.grid.height {
            for col in 0..self.grid.width {
                let target_row = i64::from(row) - min_row;
                let target_col = i64::from(col) - min_col;
                let target_index = (target_row as u32 * width + target_col as u32) as usize;
                let source_index = self.grid.index(row, col);
                expanded.data[target_index] = self.grid.data[source_index];
                pending_values[target_index] = self.pending_values[source_index];
                pending_counts[target_index] = self.pending_counts[source_index];
            }
        }
        self.grid = expanded;
        self.pending_values = pending_values;
        self.pending_counts = pending_counts;
        Ok(true)
    }

    #[cfg(test)]
    fn project_cell(&self, source: &OccupancyGrid, row: u32, col: u32) -> Result<usize, String> {
        let world_x = source.origin[0] + (col as f64 + 0.5) * source.resolution;
        let world_y = source.origin[1] + (row as f64 + 0.5) * source.resolution;
        let target_col = checked_grid_index(
            snap_grid_index((world_x - self.grid.origin[0]) / self.grid.resolution).floor(),
        )?;
        let target_row = checked_grid_index(
            snap_grid_index((world_y - self.grid.origin[1]) / self.grid.resolution).floor(),
        )?;
        if target_col < 0
            || target_row < 0
            || target_col >= i64::from(self.grid.width)
            || target_row >= i64::from(self.grid.height)
        {
            return Err("persistent/live cell projects outside persistent grid".to_string());
        }
        Ok(self.grid.index(target_row as u32, target_col as u32))
    }
}

/// 行列缓存沿原公式计算，未知单元仍不触发投影错误。
struct GridProjection {
    source_origin: [f64; 3],
    source_resolution: f64,
    target_origin: [f64; 3],
    target_resolution: f64,
    rows: Vec<Option<i64>>,
    columns: Vec<Option<i64>>,
    width: u32,
    height: u32,
}

impl GridProjection {
    fn new(target: &OccupancyGrid, source: &OccupancyGrid) -> Self {
        // 长条大图回退逐格计算，避免行列缓存比地图本身更大。
        let cache_axes = source.width <= 16_384 && source.height <= 16_384;
        let axis = |axis: usize, count: u32| {
            (0..if cache_axes { count } else { 0 })
                .map(|index| {
                    let world = source.origin[axis] + (index as f64 + 0.5) * source.resolution;
                    checked_grid_index(
                        snap_grid_index((world - target.origin[axis]) / target.resolution).floor(),
                    )
                    .ok()
                })
                .collect()
        };
        Self {
            source_origin: source.origin,
            source_resolution: source.resolution,
            target_origin: target.origin,
            target_resolution: target.resolution,
            rows: axis(1, source.height),
            columns: axis(0, source.width),
            width: target.width,
            height: target.height,
        }
    }

    fn project(&self, row: u32, col: u32) -> Result<usize, String> {
        let (target_col, target_row) = if self.rows.is_empty() {
            let x = self.source_origin[0] + (col as f64 + 0.5) * self.source_resolution;
            let y = self.source_origin[1] + (row as f64 + 0.5) * self.source_resolution;
            (
                checked_grid_index(
                    snap_grid_index((x - self.target_origin[0]) / self.target_resolution).floor(),
                )?,
                checked_grid_index(
                    snap_grid_index((y - self.target_origin[1]) / self.target_resolution).floor(),
                )?,
            )
        } else {
            (
                self.columns[col as usize]
                    .ok_or_else(|| "grid coordinate is not representable".to_string())?,
                self.rows[row as usize]
                    .ok_or_else(|| "grid coordinate is not representable".to_string())?,
            )
        };
        if target_col < 0
            || target_row < 0
            || target_col >= i64::from(self.width)
            || target_row >= i64::from(self.height)
        {
            return Err("persistent/live cell projects outside persistent grid".to_string());
        }
        Ok((target_row as u32 * self.width + target_col as u32) as usize)
    }
}

fn occupancy_class(value: i8) -> u8 {
    u8::from(value >= OCCUPANCY_OBSTACLE_THRESHOLD as i8)
}

pub(crate) fn snap_grid_index(value: f64) -> f64 {
    let rounded = value.round();
    if (value - rounded).abs() <= 1.0e-6 {
        rounded
    } else {
        value
    }
}

fn checked_grid_index(value: f64) -> Result<i64, String> {
    if !value.is_finite() || value < i64::MIN as f64 || value > i64::MAX as f64 {
        return Err("grid coordinate is not representable".to_string());
    }
    Ok(value as i64)
}

pub(crate) fn validate_checkpoint_grid(
    grid: &OccupancyGrid,
    expected_frame_id: &str,
) -> Result<(), String> {
    if grid.width == 0 || grid.height == 0 {
        return Err("checkpoint width/height must be positive".to_string());
    }
    let expected_len = (grid.width as usize)
        .checked_mul(grid.height as usize)
        .ok_or_else(|| "checkpoint dimensions overflow".to_string())?;
    if grid.data.len() != expected_len {
        return Err(format!(
            "checkpoint data size mismatch: expected={expected_len} actual={}",
            grid.data.len()
        ));
    }
    if !grid.resolution.is_finite() || grid.resolution <= 0.0 {
        return Err("checkpoint resolution must be finite and positive".to_string());
    }
    if grid.frame_id.trim().trim_start_matches('/') != expected_frame_id {
        return Err(format!(
            "checkpoint frame must be {expected_frame_id}, received {}",
            grid.frame_id
        ));
    }
    if grid
        .origin
        .iter()
        .chain(grid.origin_orientation.iter())
        .any(|value| !value.is_finite())
    {
        return Err("checkpoint origin contains non-finite values".to_string());
    }
    if grid.data.iter().any(|value| !(-1..=100).contains(value)) {
        return Err("checkpoint occupancy values must be within [-1, 100]".to_string());
    }
    Ok(())
}

#[cfg(test)]
mod projection_tests {
    use super::*;

    #[test]
    fn slanted_free_observations_remove_the_same_residue_as_three_dimensional_rays() {
        use nalgebra::point;
        use nuwax_mapping::{cost_mapper::SimplePointCloud2, pointclouds::occupancy::build_occupancy_auto};
        let observe = |timestamp, points| build_occupancy_auto(
            &SimplePointCloud2::with_metadata(points, point![0.025, 0.025, 0.0], "map", timestamp),
            0.05,
        ).unwrap();
        let first = observe(1.0, vec![point![1.025, 0.025, 0.0]]);
        let mut persistent = PersistentOccupancyGrid::stage_observation(None, &first).unwrap();
        persistent.merge_observation(&observe(2.0, vec![point![1.025, 0.025, 0.0]])).unwrap();
        let at = |persistent: &PersistentOccupancyGrid| {
            let grid = &persistent.grid;
            let x = ((1.025 - grid.origin[0]) / grid.resolution).floor() as u32;
            let y = ((0.025 - grid.origin[1]) / grid.resolution).floor() as u32;
            grid.data[grid.index(y, x)]
        };
        assert_eq!(at(&persistent), 100);
        // 终点不在水平切片内，但射线真实穿过旧障碍格；同帧重投不能重复投票。
        for timestamp in [3.0, 4.0] {
            let free = observe(timestamp, vec![point![4.025, 0.025, 0.2]]);
            persistent.merge_observation(&free).unwrap();
            persistent.merge_observation(&free).unwrap();
            assert_eq!(at(&persistent), 100);
        }
        persistent.merge_observation(&observe(5.0, vec![point![4.025, 0.025, 0.2]])).unwrap();
        assert_eq!(at(&persistent), 0);
    }

    #[test]
    fn planar_observations_confirm_hits_and_clears_without_reusing_timestamps() {
        let mut scan = OccupancyGrid::with_frame(3, 1, 0.05, "map");
        scan.data = vec![0, 100, -1];
        scan.ts = 1.0;
        let mut map = PersistentOccupancyGrid::stage_observation(None, &scan).unwrap();
        assert_eq!(map.grid.data, [0, -1, -1]);
        map.merge_observation(&scan).unwrap();
        assert_eq!(map.grid.data[1], -1);
        scan.ts = 2.0;
        map.merge_observation(&scan).unwrap();
        assert_eq!(map.grid.data, [0, 100, -1]);
        scan.data[1] = 0;
        for ts in [3.0, 4.0] {
            scan.ts = ts;
            map.merge_observation(&scan).unwrap();
            assert_eq!(map.grid.data[1], 100);
        }
        scan.ts = 3.5;
        map.merge_observation(&scan).unwrap();
        assert_eq!(map.grid.data[1], 100);
        scan.ts = 5.0;
        scan.data.fill(-1);
        map.merge_observation(&scan).unwrap();
        assert_eq!(map.grid.data[1], 100);
        scan.ts = 6.0;
        scan.data[1] = 0;
        map.merge_observation(&scan).unwrap();
        assert_eq!(map.grid.data, [0, 0, -1]);
        let restored = PersistentOccupancyGrid::from_grid(map.grid.clone()).unwrap();
        assert_eq!(restored.grid.data, map.grid.data);
    }

    #[test]
    fn planar_restored_map_accepts_new_source_clock_without_reusing_pending_votes() {
        let mut saved = OccupancyGrid::with_frame(2,1,0.05,"map");
        saved.data = vec![100,-1];
        saved.ts = 10000.0;
        let mut restored = PersistentOccupancyGrid::from_grid(saved.clone()).unwrap();
        let mut observation = saved;
        observation.data = vec![0,100];
        for timestamp in [1.0,2.0] {
            observation.ts = timestamp;
            restored.merge_observation(&observation).unwrap();
        }
        assert_eq!(restored.grid.data,[100,100]);
        observation.ts = 3.0;
        restored.merge_observation(&observation).unwrap();
        assert_eq!(restored.grid.data,[0,100]);
    }

    #[test]
    fn planar_missing_returns_do_not_expand_or_erase_history() {
        let mut old = OccupancyGrid::with_frame(2, 1, 0.05, "map");
        old.origin[0] = 100.0;
        old.data = vec![0, 100];
        old.ts = 1.0;
        let mut map = PersistentOccupancyGrid::from_grid(old.clone()).unwrap();
        let mut absent = OccupancyGrid::with_frame(1, 1, 0.05, "map");
        absent.ts = 2.0;
        map.merge_observation(&absent).unwrap();
        assert_eq!(map.grid.origin,old.origin);
        assert_eq!(map.grid.data,old.data);
        assert_eq!(map.grid.ts,2.0);
    }

    #[test]
    fn long_thin_grid_keeps_projection_memory_bounded() {
        let target = OccupancyGrid::with_frame(40_000, 1, 0.05, "map");
        let source = OccupancyGrid::with_frame(20_000, 1, 0.05, "map");
        let persistent = PersistentOccupancyGrid::from_grid(target.clone()).unwrap();
        let projection = GridProjection::new(&target, &source);
        assert!(projection.rows.is_empty() && projection.columns.is_empty());
        for col in [0, 1, 16_383, 16_384, 19_999] {
            assert_eq!(
                projection.project(0, col),
                persistent.project_cell(&source, 0, col)
            );
        }
    }

    #[test]
    fn cached_axes_match_original_cell_projection_including_boundaries() {
        for origin in [[-20.05, 3.15, 0.0], [0.0, 0.0, 0.0], [1e6, -1e6, 0.0]] {
            let mut target = OccupancyGrid::with_frame(127, 139, 0.05, "map");
            target.origin = origin;
            let persistent = PersistentOccupancyGrid::from_grid(target.clone()).unwrap();
            for shift in [-2.0, -1e-7, 0.0, 0.5, 1.0, 8.0] {
                let mut source = OccupancyGrid::with_frame(31, 37, 0.05, "map");
                source.origin = [origin[0] + shift * 0.05, origin[1] - shift * 0.05, 0.0];
                let projection = GridProjection::new(&target, &source);
                for row in 0..source.height {
                    for col in 0..source.width {
                        assert_eq!(
                            projection.project(row, col),
                            persistent.project_cell(&source, row, col)
                        );
                    }
                }
            }
        }
    }
}
