//! Stream / RPC adapter (rework22 P0-11)
//!
//! `Module` trait + `TerrainMapExtModule` 实现:
//! - 3 输入 (registered_scan / odometry / merge_request)
//! - 1 输出 (terrain_map_ext_out)
//! - 10Hz lifecycle;不接 LCM transport,用 in-memory mpsc。

use crate::{NavError, Point3, Pose3};

/// Tick 周期(rework22 P0-11:10Hz)
pub const TICK_PERIOD_MS: u64 = 100;

/// 模块单次输出
#[derive(Debug, Clone)]
pub enum ModuleOutput {
    /// 输出扩展地形图(点云 + robot pose)
    TerrainMap {
        /// 点云
        points: Vec<Point3>,
        /// robot pose
        pose: Pose3,
    },
    /// 无输出(idle)
    None,
}

/// 模块输入种类
#[derive(Debug, Clone)]
pub enum ModuleInput {
    /// 已注册的 scan(点云 + intensity)
    RegisteredScan {
        /// points
        points: Vec<Point3>,
        /// intensities
        intensities: Vec<f64>,
    },
    /// 里程计
    Odometry(Pose3),
    /// 合并请求(空闲时可处理其他积压)
    MergeRequest,
}

/// rework22 P0-11 `Module` trait:统一接口。
pub trait Module: Send + Sync + 'static {
    /// 启动
    fn start(&mut self) -> Result<(), NavError>;
    /// 停止
    fn stop(&mut self) -> Result<(), NavError>;
    /// 处理 registered_scan
    fn on_registered_scan(&mut self, points: Vec<Point3>, intensities: Vec<f64>) -> Result<(), NavError>;
    /// 处理 odometry
    fn on_odometry(&mut self, pose: Pose3) -> Result<(), NavError>;
    /// 当前输出(供 lifecycle tick 调用)
    fn tick(&mut self) -> ModuleOutput;
}

/// `TerrainMapExtModule`:扩展地形图模块包装。
///
/// Phase 1 仅缓存最近一次输入并产出最近一次合并结果(占位)。
pub struct TerrainMapExtModule {
    running: bool,
    last_scan: Option<(Vec<Point3>, Vec<f64>)>,
    last_pose: Option<Pose3>,
    /// 输出缓冲(由 tick 消费)
    output: ModuleOutput,
}

impl TerrainMapExtModule {
    /// 构造
    #[must_use]
    pub fn new() -> Self {
        Self {
            running: false,
            last_scan: None,
            last_pose: None,
            output: ModuleOutput::None,
        }
    }

    /// 是否运行
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }
}

impl Default for TerrainMapExtModule {
    fn default() -> Self {
        Self::new()
    }
}

impl Module for TerrainMapExtModule {
    fn start(&mut self) -> Result<(), NavError> {
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        self.running = false;
        Ok(())
    }

    fn on_registered_scan(
        &mut self,
        points: Vec<Point3>,
        intensities: Vec<f64>,
    ) -> Result<(), NavError> {
        if !self.running {
            return Err(NavError::Internal("module not running".into()));
        }
        self.last_scan = Some((points, intensities));
        Ok(())
    }

    fn on_odometry(&mut self, pose: Pose3) -> Result<(), NavError> {
        if !self.running {
            return Err(NavError::Internal("module not running".into()));
        }
        self.last_pose = Some(pose);
        Ok(())
    }

    fn tick(&mut self) -> ModuleOutput {
        if !self.running {
            return ModuleOutput::None;
        }
        match (&self.last_scan, &self.last_pose) {
            (Some((pts, _)), Some(pose)) => {
                let out = ModuleOutput::TerrainMap {
                    points: pts.clone(),
                    pose: *pose,
                };
                // Phase 1:消费掉,避免重复输出同一帧
                self.output = out.clone();
                self.last_scan = None;
                out
            }
            _ => ModuleOutput::None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pose() -> Pose3 {
        Pose3 {
            position: Point3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: crate::Pose3::default().orientation,
        }
    }

    #[test]
    fn module_lifecycle_idempotent() {
        let mut m = TerrainMapExtModule::new();
        assert!(m.start().is_ok());
        assert!(m.start().is_ok());
        assert!(m.is_running());
        assert!(m.stop().is_ok());
        assert!(m.stop().is_ok());
        assert!(!m.is_running());
    }

    #[test]
    fn on_inputs_require_running() {
        let mut m = TerrainMapExtModule::new();
        assert!(m
            .on_registered_scan(vec![], vec![])
            .is_err());
        assert!(m.on_odometry(make_pose()).is_err());
        m.start().unwrap();
        assert!(m
            .on_registered_scan(vec![Point3 { x: 1.0, y: 2.0, z: 3.0 }], vec![1.0])
            .is_ok());
        assert!(m.on_odometry(make_pose()).is_ok());
    }

    #[test]
    fn tick_emits_terrain_map_when_both_inputs_present() {
        let mut m = TerrainMapExtModule::new();
        m.start().unwrap();
        m.on_odometry(make_pose()).unwrap();
        m.on_registered_scan(vec![Point3 { x: 1.0, y: 2.0, z: 3.0 }], vec![0.5])
            .unwrap();
        match m.tick() {
            ModuleOutput::TerrainMap { points, .. } => {
                assert_eq!(points.len(), 1);
            }
            other => panic!("expected TerrainMap, got {other:?}"),
        }
        // 已被消费 → tick 返回 None
        assert!(matches!(m.tick(), ModuleOutput::None));
    }
}

/// 兼容历史命名(外部代码可能仍调 `subscribe_registered_scan`)
/// 现在直接 forward 到 module trait 入口的 adapter。
pub fn subscribe_registered_scan<F>(callback: F) -> Result<(), NavError>
where
    F: Fn(Vec<Point3>, Vec<f64>) + Send + Sync + 'static,
{
    // Phase 1 占位:Foundation 接入后切换为 nuwax-stream
    let _ = callback;
    Err(NavError::Internal(
        "subscribe_registered_scan:待 Foundation 接入 transport,改用 TerrainMapExtModule.on_registered_scan".into(),
    ))
}

/// 兼容历史命名
pub fn subscribe_odometry<F>(callback: F) -> Result<(), NavError>
where
    F: Fn(Pose3) + Send + Sync + 'static,
{
    let _ = callback;
    Err(NavError::Internal(
        "subscribe_odometry:待 Foundation 接入 transport,改用 TerrainMapExtModule.on_odometry".into(),
    ))
}