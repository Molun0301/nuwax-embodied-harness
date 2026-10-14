//! Test-only helpers for integration tests(方案 §11.1)。
//!
//! 集成测试不直接接触 crate-private `DetectionOutcome` / `FrontierCandidate` /
//! `CandidateGeometry`(满足 AC-2 公共 API 边界)。
//! 本模块通过 `DetectionOutcomeKind` 摘要 + `DetectorError` 摘要让 integration tests
//! 验证行为而不接触 internal types。
//!
//! `test-internals`由测试开发依赖启用，生产默认关闭；生产消费者不引用本模块。

#![doc(hidden)]

pub use crate::detector::WavefrontDetector;
pub use crate::scoring::{CandidateGeometry, FrontierScorer};

use crate::session::ExplorationSession;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;

/// detection outcome 摘要(集成 test 可断言 variant 计数;不暴露 crate-private 类型)
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DetectionOutcomeKind {
    /// 候选 cluster 列表(包含数量)
    Candidates(usize),
    /// 找不到任何可达 free cell
    NoReachableFreeSpace,
}

/// 检测错误摘要，供集成测试隔离生产错误类型的具体实现。
#[derive(Debug, Clone, PartialEq)]
pub enum DetectorError {
    /// 观察标定或计算不成立。
    InvalidObservation(String),
    /// 机器人位姿在栅格外(包含 (x, y))
    OutOfBounds { x_m: f64, y_m: f64 },
    /// 起点轮廓观测尚未完整。
    FootprintUnobserved { x_m: f64, y_m: f64 },
}

impl DetectorError {
    /// Display 形式
    #[must_use]
    pub fn to_display(&self) -> String {
        match self {
            Self::InvalidObservation(reason) => format!("invalid observation model: {reason}"),
            Self::OutOfBounds { x_m, y_m } => {
                format!("robot pose out of grid bounds: ({x_m}, {y_m})")
            }
            Self::FootprintUnobserved { x_m, y_m } => {
                format!("robot footprint not fully observed at: ({x_m}, {y_m})")
            }
        }
    }
}

impl std::fmt::Display for DetectorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.to_display())
    }
}

impl std::error::Error for DetectorError {}

/// 单次调用 detector 并返回**摘要** outcome / error(integration tests 用)
#[allow(dead_code)]
pub fn detect_kind(
    detector: &mut WavefrontDetector,
    robot_pose: Point2,
    costmap: &OccupancyGrid2D,
    session: &ExplorationSession,
) -> Result<DetectionOutcomeKind, DetectorError> {
    let out = detector
        .detect_candidates(robot_pose, costmap, session)
        .map_err(|e| match e {
            crate::detector::WavefrontError::InvalidObservation(reason) => DetectorError::InvalidObservation(reason),
            crate::detector::WavefrontError::RobotPoseOutOfBounds(x, y) => {
                DetectorError::OutOfBounds { x_m: x, y_m: y }
            }
            crate::detector::WavefrontError::RobotFootprintUnobserved(x, y) => {
                DetectorError::FootprintUnobserved { x_m: x, y_m: y }
            }
        })?;
    Ok(match out {
        crate::detector::DetectionOutcome::Candidates(cs) => {
            DetectionOutcomeKind::Candidates(cs.len())
        }
        crate::detector::DetectionOutcome::NoReachableFreeSpace => {
            DetectionOutcomeKind::NoReachableFreeSpace
        }
    })
}
