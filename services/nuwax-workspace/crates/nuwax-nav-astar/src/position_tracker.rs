//! 位置追踪与 stuck 检测
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/position_tracker.py`。
//!
//! 行为契约：
//! - 维护时间窗内（默认 8s）最近的位置采样（环形缓冲，1000 samples/s 容量）。
//! - `is_stuck()`：当所有最近点到质心的距离 < threshold 时返回 true。
//! - 用于 `GlobalPlanner` 在 `stuck_time_window` 之后触发 replan。

use std::time::Instant;

use crate::helpers::{pose_to_point2, Point2, PoseStamped};

const MAX_POINTS_PER_SECOND: usize = 1000;

/// 位置追踪器
#[derive(Debug, Clone)]
pub struct PositionTracker {
    time_window_sec: f64,
    max_points: usize,
    threshold: f64,
    timestamps: Vec<Instant>,
    positions: Vec<Point2>,
    index: usize,
    size: usize,
}

impl PositionTracker {
    /// 构造
    ///
    /// # 参数
    /// - `time_window_sec`：时间窗（秒），默认 8.0
    /// - `threshold`：stuck 阈值（米），所有点距质心 < 该值时判定 stuck
    #[must_use]
    pub fn new(time_window_sec: f64, threshold: f64) -> Self {
        let max_points = (MAX_POINTS_PER_SECOND as f64 * time_window_sec).ceil() as usize;
        let max_points = max_points.max(1);
        Self {
            time_window_sec,
            max_points,
            threshold,
            timestamps: vec![Instant::now(); max_points],
            positions: vec![Point2::new(0.0, 0.0); max_points],
            index: 0,
            size: 0,
        }
    }

    /// 清空所有采样
    pub fn reset_data(&mut self) {
        self.index = 0;
        self.size = 0;
    }

    /// 添加一个 odom 采样
    pub fn add_position(&mut self, pose: PoseStamped) {
        let now = Instant::now();
        self.timestamps[self.index] = now;
        self.positions[self.index] = pose_to_point2(&pose);
        self.index = (self.index + 1) % self.max_points;
        if self.size < self.max_points {
            self.size += 1;
        }
    }

    /// 当前是否 stuck
    #[must_use]
    pub fn is_stuck(&self) -> bool {
        if self.size == 0 {
            return false;
        }
        let cutoff = Instant::now() - std::time::Duration::from_secs_f64(self.time_window_sec);

        // 收集时间窗内的点
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut count = 0usize;
        let mut points: Vec<Point2> = Vec::new();
        if self.size < self.max_points {
            for i in 0..self.size {
                if self.timestamps[i] >= cutoff {
                    sum_x += self.positions[i].x;
                    sum_y += self.positions[i].y;
                    count += 1;
                    points.push(self.positions[i]);
                }
            }
        } else {
            // 环形 buffer，按时间顺序
            let order: Vec<usize> = (0..self.max_points)
                .map(|i| (self.index + i) % self.max_points)
                .collect();
            for i in order {
                if self.timestamps[i] >= cutoff {
                    sum_x += self.positions[i].x;
                    sum_y += self.positions[i].y;
                    count += 1;
                    points.push(self.positions[i]);
                }
            }
        }

        if count == 0 {
            return false;
        }

        let centroid = Point2::new(sum_x / count as f64, sum_y / count as f64);
        for p in &points {
            let dx = p.x - centroid.x;
            let dy = p.y - centroid.y;
            if (dx * dx + dy * dy).sqrt() >= self.threshold {
                return false;
            }
        }
        true
    }

    /// 当前时间窗内采样数
    #[must_use]
    pub fn sample_count(&self) -> usize {
        self.size
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::helpers::make_pose_stamped;
    use std::thread::sleep;
    use std::time::Duration;

    fn pose(x: f64, y: f64) -> PoseStamped {
        make_pose_stamped("map", x, y, 0.0)
    }

    #[test]
    fn empty_is_not_stuck() {
        let pt = PositionTracker::new(8.0, 0.4);
        assert!(!pt.is_stuck());
    }

    #[test]
    fn clustered_positions_are_stuck() {
        let mut pt = PositionTracker::new(8.0, 0.4);
        for _ in 0..5 {
            pt.add_position(pose(1.0, 1.0));
        }
        assert!(pt.is_stuck());
    }

    #[test]
    fn moving_positions_not_stuck() {
        let mut pt = PositionTracker::new(8.0, 0.4);
        for i in 0..5 {
            pt.add_position(pose(i as f64, 0.0));
        }
        assert!(!pt.is_stuck());
    }

    #[test]
    fn reset_data_clears_buffer() {
        let mut pt = PositionTracker::new(8.0, 0.4);
        for _ in 0..5 {
            pt.add_position(pose(1.0, 1.0));
        }
        pt.reset_data();
        assert_eq!(pt.sample_count(), 0);
        assert!(!pt.is_stuck());
    }

    #[test]
    fn max_points_capped() {
        let mut pt = PositionTracker::new(0.1, 0.4); // 100 points max
        for i in 0..200 {
            pt.add_position(pose(i as f64, 0.0));
        }
        assert_eq!(pt.sample_count(), 100);
    }

    #[test]
    fn threshold_excludes_outer_points() {
        // threshold 0.1m, 点分布在 1m 范围内，质心距每个点 ~0.5m, 不应 stuck
        let mut pt = PositionTracker::new(8.0, 0.1);
        for i in 0..5 {
            pt.add_position(pose(i as f64, 0.0));
        }
        assert!(!pt.is_stuck());
    }

    #[test]
    fn time_window_filters_old_samples() {
        // 大 time_window 让两个点都在窗内，stuck 算法对 2 个远点不判定 stuck
        let mut pt = PositionTracker::new(8.0, 0.4);
        pt.add_position(pose(0.0, 0.0));
        sleep(Duration::from_millis(2));
        pt.add_position(pose(5.0, 5.0));
        // 两点都在窗内，距离 ~7m > threshold 0.4，不应 stuck
        assert!(!pt.is_stuck());

        // 第二个 position_tracker：time_window 极小，让第一个点被过滤
        let mut pt2 = PositionTracker::new(0.001, 0.4); // 1ms window
        pt2.add_position(pose(0.0, 0.0));
        sleep(Duration::from_millis(50));
        pt2.add_position(pose(5.0, 5.0));
        // 第一个点已过期；窗内只剩一个点 (5,5)，size 报告仍为 2 但 is_stuck 用时间窗过滤
        // 单点距质心 0 < threshold 会 stuck；所以此处应验证时间窗过滤逻辑本身
        // 用第二个 point_tracker 验证过期过滤：size=2 但 is_stuck() 中 count=1
        let cutoff = std::time::Instant::now() - std::time::Duration::from_secs_f64(0.001);
        let mut fresh_count = 0;
        for i in 0..pt2.size {
            if pt2.timestamps[i] >= cutoff {
                fresh_count += 1;
            }
        }
        assert_eq!(fresh_count, 1, "老点应被时间窗过滤");
    }
}
