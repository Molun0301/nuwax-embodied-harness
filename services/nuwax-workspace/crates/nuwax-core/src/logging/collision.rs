//! 导航失败附带原生碰撞判断实际命中的格。

use std::fmt;
use std::time::Instant;

use super::DiagnosticLimiter;

#[derive(Clone, Copy, Default)]
#[repr(C)]
struct Cell {
    x: f64,
    y: f64,
    bottom: f64,
    top: f64,
    layer: u32,
}

/// 与两个原生规划器共用的当拍碰撞记录。
#[derive(Default)]
#[repr(C)]
pub struct CollisionEvidence {
    x: f64,
    y: f64,
    ground: f64,
    count: usize,
    cells: [Cell; 10],
}

/// 碰撞日志中的地图位姿。仅有位置的旧调用保留未知朝向。
#[derive(Clone, Copy)]
pub struct CollisionLogPose {
    /// 机体参考点的地图坐标。
    pub position: [f64; 3],
    orientation_xyzw: Option<[f64; 4]>,
}

impl From<[f64; 3]> for CollisionLogPose {
    fn from(position: [f64; 3]) -> Self {
        Self { position, orientation_xyzw: None }
    }
}

impl From<([f64; 3], [f64; 4])> for CollisionLogPose {
    fn from((position, orientation): ([f64; 3], [f64; 4])) -> Self {
        Self { position, orientation_xyzw: Some(orientation) }
    }
}

impl CollisionLogPose {
    /// 原生局部控制使用偏航角。
    pub fn with_yaw(position: [f64; 3], yaw: Option<f64>) -> Self {
        Self { position, orientation_xyzw: yaw.map(|yaw| [0., 0., (yaw / 2.).sin(), (yaw / 2.).cos()]) }
    }
}

impl CollisionEvidence {
    /// 以机体当前位置排序，垂向区间相对当拍估计地面表达。
    pub fn new(position: [f64; 3], body_to_ground: f64) -> Self {
        Self { x: position[0], y: position[1], ground: position[2] - body_to_ground, ..Default::default() }
    }

    /// 只在本次失败可能写入日志时收集碰撞证据。
    pub fn capture(limiter: &DiagnosticLimiter, now: Instant) -> bool {
        tracing::enabled!(target: "navigation.collision", tracing::Level::INFO) && limiter.ready_at(now)
    }

    /// 保持规划返回值，仅用现有的一秒限频写一行。
    pub fn log_failure(
        &self, limiter: &mut DiagnosticLimiter, now: Instant, stage: &str, reason: &str,
        robot: CollisionLogPose, goal: Option<CollisionLogPose>,
    ) {
        if let Some(suppressed) = limiter.record_at(now) {
            tracing::info!(target: "navigation.collision", stage, reason, suppressed,
                robot_xyz = ?robot.position, robot_orientation_xyzw = ?robot.orientation_xyzw,
                goal_xyz = ?goal.map(|pose| pose.position),
                goal_orientation_xyzw = ?goal.and_then(|pose| pose.orientation_xyzw),
                ground_z_m = self.ground, blocking_cells = %self, "导航失败碰撞格");
        }
    }
}

impl fmt::Display for CollisionEvidence {
    fn fmt(&self, out: &mut fmt::Formatter<'_>) -> fmt::Result {
        out.write_str("[")?;
        for (index, cell) in self.cells[..self.count].iter().enumerate() {
            if index > 0 {
                out.write_str(",")?;
            }
            let source = match cell.layer {
                0 => "占据体素",
                1 => "表面接触",
                2 => "业务禁行",
                _ => unreachable!(),
            };
            write!(out, r#"{{"x_m":{:.4},"y_m":{:.4},"source":"{}","z_bottom_m":"#,
                cell.x, cell.y, source)?;
            let height = |out: &mut fmt::Formatter<'_>, z: f64| {
                if z.is_finite() { write!(out, "{z:.4}") } else { out.write_str("null") }
            };
            height(out, cell.bottom)?;
            out.write_str(r#","z_top_m":"#)?;
            height(out, cell.top)?;
            out.write_str(r#","above_ground_bottom_m":"#)?;
            height(out, cell.bottom - self.ground)?;
            out.write_str(r#","above_ground_top_m":"#)?;
            height(out, cell.top - self.ground)?;
            out.write_str("}")?;
        }
        out.write_str("]")
    }
}
