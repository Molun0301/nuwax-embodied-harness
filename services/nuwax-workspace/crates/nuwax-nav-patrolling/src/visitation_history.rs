//! 访问历史：维护 visited grid mask 和 saturation。

/// 访问历史（纯算法核心，依赖由调用方注入 grid / position）
#[derive(Debug, Clone)]
pub struct VisitationHistory {
    /// 清除半径（米）
    clearance_radius_m: f64,
    /// cell 数 = ceil(clearance_radius_m / resolution)
    clearance_radius_cells: usize,
    /// 圆盘 mask（(2r+1) x (2r+1)）
    clearance_disk: Vec<bool>,
    /// 访问点缓存（世界坐标）
    points: Vec<(f64, f64)>,
    /// visited mask
    visited: Option<Vec<bool>>,
    /// grid 宽度
    width: usize,
    /// grid 高度
    height: usize,
    /// grid 分辨率（m/cell）
    resolution: f64,
    /// origin（米）
    origin_x: f64,
    origin_y: f64,
    /// saturation threshold（>0 时启用 drop oldest half）
    saturation_threshold: f32,
    /// min distance between stored points
    min_distance_m: f64,
}

impl VisitationHistory {
    /// 构造
    #[must_use]
    pub fn new(clearance_radius_m: f64) -> Self {
        Self {
            clearance_radius_m,
            clearance_radius_cells: 0,
            clearance_disk: vec![true],
            points: Vec::new(),
            visited: None,
            width: 0,
            height: 0,
            resolution: 0.0,
            origin_x: 0.0,
            origin_y: 0.0,
            saturation_threshold: 0.5,
            min_distance_m: 0.05,
        }
    }

    /// 设置 saturation threshold（>0 启用 drop oldest half）
    pub fn set_saturation_threshold(&mut self, t: f32) {
        self.saturation_threshold = t;
    }

    /// 设置最小记录间隔（m）
    pub fn set_min_distance_m(&mut self, d: f64) {
        self.min_distance_m = d;
    }

    /// 圆盘 mask（用于外部测试 / 路径估算）
    #[must_use]
    pub fn clearance_disk(&self) -> &[bool] {
        &self.clearance_disk
    }

    /// 圆盘半径（cell）
    #[must_use]
    pub fn clearance_radius_cells(&self) -> usize {
        self.clearance_radius_cells
    }

    /// 更新 grid（尺寸/分辨率变化时 rebuild）
    pub fn update_grid(
        &mut self,
        width: usize,
        height: usize,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
    ) {
        let new_radius_cells = (self.clearance_radius_m / resolution).ceil() as usize;
        let rebuild = match &self.visited {
            None => true,
            Some(v) => {
                v.len() != width * height
                    || self.width != width
                    || self.height != height
                    || (resolution - self.resolution).abs() > 1e-9
                    || new_radius_cells != self.clearance_radius_cells
            }
        };
        self.width = width;
        self.height = height;
        self.resolution = resolution;
        self.origin_x = origin_x;
        self.origin_y = origin_y;
        self.clearance_radius_cells = new_radius_cells;
        self.clearance_disk = circular_disk(self.clearance_radius_cells);
        if rebuild {
            self.visited = Some(vec![false; width * height]);
            self.points.clear();
        }
    }

    /// 处理 odom：去重 + 写入 visited
    pub fn handle_odom(&mut self, x: f64, y: f64) {
        if let Some((lx, ly)) = self.points.last().copied() {
            let dx = x - lx;
            let dy = y - ly;
            if dx * dx + dy * dy < self.min_distance_m * self.min_distance_m {
                return;
            }
        }
        self.points.push((x, y));
        self.stamp(x, y);
        if self.saturation() >= self.saturation_threshold && !self.points.is_empty() {
            let n = self.points.len();
            self.points.drain(..n / 2);
            // rebuild from remaining points（先 clone 出来避免借用冲突）
            let remaining: Vec<(f64, f64)> = self.points.clone();
            self.visited = Some(vec![false; self.width * self.height]);
            for (x, y) in remaining {
                self.stamp(x, y);
            }
        }
    }

    /// 标记 visited
    pub fn stamp(&mut self, x: f64, y: f64) {
        let Some(visited) = self.visited.as_mut() else {
            return;
        };
        if self.width == 0 || self.height == 0 || self.resolution <= 0.0 {
            return;
        }
        let r = self.clearance_radius_cells;
        let col = ((x - self.origin_x) / self.resolution).floor() as isize;
        let row = ((y - self.origin_y) / self.resolution).floor() as isize;
        let h = self.height as isize;
        let w = self.width as isize;
        if row + (r as isize) < 0
            || row - (r as isize) >= h
            || col + (r as isize) < 0
            || col - (r as isize) >= w
        {
            return;
        }
        let r_min = (row - r as isize).max(0) as usize;
        let r_max = (row + r as isize + 1).min(h) as usize;
        let c_min = (col - r as isize).max(0) as usize;
        let c_max = (col + r as isize + 1).min(w) as usize;
        let d_r_min = r_min as isize - (row - r as isize);
        let d_c_min = c_min as isize - (col - r as isize);
        let disk = &self.clearance_disk;
        let disk_dim = (2 * r + 1) as isize;
        for ri in 0..(r_max - r_min) {
            for ci in 0..(c_max - c_min) {
                let didx = ((d_r_min + ri as isize) * disk_dim + (d_c_min + ci as isize)) as usize;
                if disk[didx] {
                    visited[(r_min + ri) * self.width + (c_min + ci)] = true;
                }
            }
        }
    }

    /// 当前 saturation
    pub fn saturation(&self) -> f32 {
        let Some(visited) = &self.visited else {
            return 0.0;
        };
        if visited.is_empty() {
            return 0.0;
        }
        // 实际 free 计数需要调用方传入 free mask；本 crate 退化为 visited/total
        let n = visited.iter().filter(|v| **v).count();
        n as f32 / visited.len() as f32
    }

    /// 当前 saturation（free mask 已知版本）
    pub fn saturation_with_free(&self, free_mask: &[bool]) -> f32 {
        let Some(visited) = &self.visited else {
            return 0.0;
        };
        let total = free_mask.iter().filter(|v| **v).count();
        if total == 0 {
            return 0.0;
        }
        let mut vf = 0;
        for i in 0..visited.len().min(free_mask.len()) {
            if visited[i] && free_mask[i] {
                vf += 1;
            }
        }
        vf as f32 / total as f32
    }

    /// reset
    pub fn reset(&mut self) {
        self.points.clear();
        self.visited = None;
    }

    /// 是否已 visit
    #[must_use]
    pub fn is_visited(&self, x: f64, y: f64) -> bool {
        let Some(visited) = &self.visited else {
            return false;
        };
        if self.width == 0 || self.height == 0 || self.resolution <= 0.0 {
            return false;
        }
        let col = ((x - self.origin_x) / self.resolution).floor() as isize;
        let row = ((y - self.origin_y) / self.resolution).floor() as isize;
        if row < 0 || col < 0 {
            return false;
        }
        let (row, col) = (row as usize, col as usize);
        if row >= self.height || col >= self.width {
            return false;
        }
        visited[row * self.width + col]
    }

    /// visited buffer
    #[must_use]
    pub fn visited(&self) -> Option<&[bool]> {
        self.visited.as_deref()
    }

    /// width
    #[must_use]
    pub fn width(&self) -> usize {
        self.width
    }

    /// height
    #[must_use]
    pub fn height(&self) -> usize {
        self.height
    }

    /// resolution (m/cell)
    #[must_use]
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// origin x（米）
    #[must_use]
    pub fn origin_x(&self) -> f64 {
        self.origin_x
    }

    /// origin y（米）
    #[must_use]
    pub fn origin_y(&self) -> f64 {
        self.origin_y
    }
}

/// 构造圆盘 mask（(2r+1) x (2r+1)）
#[must_use]
pub fn circular_disk(radius_cells: usize) -> Vec<bool> {
    let dim = 2 * radius_cells + 1;
    let mut v = vec![false; dim * dim];
    for dy in 0..dim {
        for dx in 0..dim {
            let cy = dy as isize - radius_cells as isize;
            let cx = dx as isize - radius_cells as isize;
            if cx * cx + cy * cy <= (radius_cells as isize) * (radius_cells as isize) {
                v[dy * dim + dx] = true;
            }
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disk_shape() {
        let d = circular_disk(2);
        // 5x5, 中心是 true
        assert_eq!(d.len(), 25);
        assert!(d[2 * 5 + 2]); // 中心
                               // 角落是 false
        assert!(!d[0]);
        assert!(!d[4 * 5 + 4]);
    }

    #[test]
    fn grid_rebuild_on_size_change() {
        let mut v = VisitationHistory::new(0.5);
        v.update_grid(10, 10, 0.1, 0.0, 0.0);
        v.handle_odom(0.5, 0.5);
        assert!(v.is_visited(0.5, 0.5));
        // 尺寸变化 -> rebuild
        v.update_grid(20, 20, 0.1, 0.0, 0.0);
        // 之前记录被清空
        assert_eq!(v.visited().unwrap().iter().filter(|x| **x).count(), 0);
    }

    #[test]
    fn saturation_with_free_mask() {
        let mut v = VisitationHistory::new(0.5);
        v.update_grid(10, 10, 0.1, 0.0, 0.0);
        let mut free = vec![true; 100];
        // 全部 occupied except center
        free.iter_mut().for_each(|f| *f = false);
        free[55] = true;
        free[45] = true;
        v.handle_odom(0.55, 0.55);
        let sat = v.saturation_with_free(&free);
        // 至少有一个 visited free cell
        assert!(sat > 0.0);
    }
}
