//! 二维算法安全接口。每个进程至多一个实例；实例与其创建线程绑定。
use nuwax_slam_toolbox_sys as raw;
pub use raw::Pose;
use serde::{Deserialize, Serialize};
use std::{
    ffi::{c_char, c_void, CStr, CString},
    fmt,
    marker::PhantomData,
    path::Path,
    ptr::NonNull,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub scan_count: u32,
    pub angle_min: f64,
    pub angle_increment: f64,
    pub range_min: f64,
    pub range_max: f64,
    pub range_threshold: f64,
    /// 雷达到机体的平面外参，依次为米、米、弧度。
    pub base_from_laser: [f64; 3],
    pub resolution: f64,
    pub minimum_travel: f64,
    pub minimum_heading: f64,
    pub minimum_response: f64,
    pub local_window: u32,
    pub max_nodes: u32,
    pub loop_closing: bool,
    pub solver_threads: u32,
    pub use_scan_barycenter: bool,
    pub loop_minimum_chain: u32,
    pub link_maximum_distance: f64,
    pub loop_search_distance: f64,
    pub search_dimension: f64,
    pub search_resolution: f64,
    pub search_smear: f64,
    pub max_scan_gap_s: f64,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            scan_count: 360,
            angle_min: -std::f64::consts::PI,
            angle_increment: std::f64::consts::TAU / 360.0,
            range_min: 0.05,
            range_max: 30.0,
            range_threshold: 12.0,
            base_from_laser: [0.0; 3],
            resolution: 0.05,
            minimum_travel: 0.1,
            minimum_heading: 0.087,
            minimum_response: 0.3,
            local_window: 20,
            max_nodes: 20000,
            loop_closing: true,
            solver_threads: 2,
            use_scan_barycenter: true,
            loop_minimum_chain: 10,
            link_maximum_distance: 6.0,
            loop_search_distance: 4.0,
            search_dimension: 0.5,
            search_resolution: 0.01,
            search_smear: 0.03,
            max_scan_gap_s: 0.5,
        }
    }
}
impl Config {
    /// 校验真实原生参数，不创建实例或更改已有图。
    pub fn validate(&self) -> Result<(), Error> {
        if unsafe { raw::nw_slam_abi_version() } != 1 {
            return Err(Error {
                code: 1,
                message: "二维原生接口版本不匹配".into(),
            });
        }
        let mut error = [0; 1024];
        let code =
            unsafe { raw::nw_slam_validate_config(&self.raw(), error.as_mut_ptr(), error.len()) };
        check(code, &error)
    }
    fn raw(&self) -> raw::Config {
        raw::Config {
            abi_version: 1,
            scan_count: self.scan_count,
            local_window: self.local_window,
            max_nodes: self.max_nodes,
            angle_min: self.angle_min,
            angle_increment: self.angle_increment,
            range_min: self.range_min,
            range_max: self.range_max,
            range_threshold: self.range_threshold,
            base_from_laser: Pose {
                x: self.base_from_laser[0],
                y: self.base_from_laser[1],
                yaw: self.base_from_laser[2],
            },
            resolution: self.resolution,
            minimum_travel: self.minimum_travel,
            minimum_heading: self.minimum_heading,
            minimum_response: self.minimum_response,
            search_dimension: self.search_dimension,
            search_resolution: self.search_resolution,
            search_smear: self.search_smear,
            max_scan_gap_s: self.max_scan_gap_s,
            loop_closing: u32::from(self.loop_closing),
            solver_threads: self.solver_threads,
            use_scan_barycenter: u32::from(self.use_scan_barycenter),
            loop_minimum_chain: self.loop_minimum_chain,
            link_maximum_distance: self.link_maximum_distance,
            loop_search_distance: self.loop_search_distance,
        }
    }
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Error {
    pub code: i32,
    pub message: String,
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}
impl std::error::Error for Error {}
fn check(code: i32, buffer: &[c_char]) -> Result<(), Error> {
    if code == 0 {
        return Ok(());
    }
    // 原生接口总在预清零缓冲区中写入以零结尾的错误。
    let message = unsafe { CStr::from_ptr(buffer.as_ptr()) }
        .to_string_lossy()
        .into_owned();
    Err(Error { code, message })
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Disposition {
    Initialized,
    Accepted,
    MotionSkipped,
    MatchRejected,
}
#[derive(Debug, Clone)]
pub struct Estimate {
    pub disposition: Disposition,
    pub output: raw::Output,
}
impl Estimate {
    pub fn accepted(&self) -> bool {
        matches!(
            self.disposition,
            Disposition::Initialized | Disposition::Accepted
        )
    }
}
#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    pub metadata: raw::Grid,
    pub cells: Vec<i8>,
}

/// 不实现Send或Sync：调用者把整个实例放在独立算法线程内创建和销毁。
pub struct Slam {
    handle: NonNull<c_void>,
    config: Config,
    _thread: PhantomData<*mut ()>,
}
impl Slam {
    pub fn new(config: Config) -> Result<Self, Error> {
        if unsafe { raw::nw_slam_abi_version() } != 1 {
            return Err(Error {
                code: 1,
                message: "二维原生接口版本不匹配".into(),
            });
        }
        let mut handle = std::ptr::null_mut();
        let mut error = [0; 1024];
        // 参数只在同步调用期间借用，原生实现复制配置。
        let code = unsafe {
            raw::nw_slam_create(&config.raw(), &mut handle, error.as_mut_ptr(), error.len())
        };
        check(code, &error)?;
        let handle = NonNull::new(handle).ok_or_else(|| Error {
            code: 2,
            message: "原生创建成功但未返回上下文".into(),
        })?;
        Ok(Self {
            handle,
            config,
            _thread: PhantomData,
        })
    }
    pub fn resume_after_pause(&mut self) -> Result<(), Error> {
        let mut error = [0; 1024];
        let code = unsafe { raw::nw_slam_resume(self.handle.as_ptr(), error.as_mut_ptr(), error.len()) };
        check(code, &error)
    }
    pub fn config(&self) -> &Config {
        &self.config
    }
    /// None选择本进程的局部激光里程计；实例中途不能更换来源。
    pub fn process(
        &mut self,
        timestamp_ns: u64,
        sequence: u64,
        ranges: &[f64],
        odometry: Option<Pose>,
    ) -> Result<Estimate, Error> {
        if ranges.len() != self.config.scan_count as usize {
            return Err(Error {
                code: 1,
                message: "扫描束数与配置不一致".into(),
            });
        }
        let input = raw::Scan {
            timestamp_ns,
            sequence,
            ranges: ranges.as_ptr(),
            count: ranges.len(),
            odom_from_base: odometry.unwrap_or_default(),
            use_laser_odometry: u32::from(odometry.is_none()),
        };
        let mut output = raw::Output::default();
        let mut error = [0; 1024];
        let code = unsafe {
            raw::nw_slam_process(
                self.handle.as_ptr(),
                &input,
                &mut output,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        check(code, &error)?;
        let disposition = match output.disposition {
            0 => Disposition::Initialized,
            1 => Disposition::Accepted,
            2 => Disposition::MotionSkipped,
            3 => Disposition::MatchRejected,
            _ => {
                return Err(Error {
                    code: 2,
                    message: "未知扫描处置结果".into(),
                })
            }
        };
        Ok(Estimate {
            disposition,
            output,
        })
    }
    pub fn occupancy_grid(&mut self) -> Result<OccupancyGrid, Error> {
        let mut metadata = raw::Grid::default();
        let mut error = [0; 1024];
        let code = unsafe {
            raw::nw_slam_grid(
                self.handle.as_ptr(),
                &mut metadata,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        check(code, &error)?;
        let len = (metadata.width as usize)
            .checked_mul(metadata.height as usize)
            .filter(|n| *n <= 16 * 1024 * 1024)
            .ok_or_else(|| Error {
                code: 2,
                message: "原生栅格尺寸超出容量".into(),
            })?;
        let mut cells = vec![0; len];
        let code = unsafe {
            raw::nw_slam_copy_grid(
                self.handle.as_ptr(),
                cells.as_mut_ptr(),
                len,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        check(code, &error)?;
        Ok(OccupancyGrid { metadata, cells })
    }
    /// 写入全新的图文件前缀；资产目录的提交由上层拥有方负责。
    pub fn save(&mut self, prefix: &Path) -> Result<(), Error> {
        let path = path_string(prefix)?;
        let mut error = [0; 1024];
        let code = unsafe {
            raw::nw_slam_save(
                self.handle.as_ptr(),
                path.as_ptr(),
                error.as_mut_ptr(),
                error.len(),
            )
        };
        check(code, &error)?;
        for suffix in [".posegraph", ".data"] {
            let filename = format!("{}{suffix}", prefix.display());
            std::fs::File::open(&filename)
                .and_then(|file| file.sync_all())
                .map_err(|e| Error {
                    code: 2,
                    message: format!("同步图文件失败: {e}"),
                })?;
        }
        Ok(())
    }
    pub fn load(&mut self, prefix: &Path, localization: bool, seed: Pose) -> Result<(), Error> {
        let path = path_string(prefix)?;
        let mut error = [0; 1024];
        let code = unsafe {
            raw::nw_slam_load(
                self.handle.as_ptr(),
                path.as_ptr(),
                u32::from(localization),
                seed,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        check(code, &error)
    }
}
fn path_string(path: &Path) -> Result<CString, Error> {
    let text = path.to_str().ok_or_else(|| Error {
        code: 1,
        message: "图路径必须为UTF-8".into(),
    })?;
    CString::new(text).map_err(|_| Error {
        code: 1,
        message: "图路径含零字节".into(),
    })
}
impl Drop for Slam {
    fn drop(&mut self) {
        unsafe { raw::nw_slam_destroy(self.handle.as_ptr()) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;
    static OWNER: Mutex<()> = Mutex::new(());
    fn room(config: &Config, base: Pose) -> Vec<f64> {
        let o = config.base_from_laser;
        let c = base.yaw.cos();
        let s = base.yaw.sin();
        let x = base.x + c * o[0] - s * o[1];
        let y = base.y + s * o[0] + c * o[1];
        (0..config.scan_count)
            .map(|i| {
                let angle =
                    base.yaw + o[2] + config.angle_min + f64::from(i) * config.angle_increment;
                let dx = angle.cos();
                let dy = angle.sin();
                let mut best = f64::INFINITY;
                for wall in [-4.0, 6.0] {
                    let t = (wall - x) / dx;
                    if t > 0.0 {
                        let py = y + t * dy;
                        if (-3.0..=5.0).contains(&py) {
                            best = best.min(t);
                        }
                    }
                }
                for wall in [-3.0, 5.0] {
                    let t = (wall - y) / dy;
                    if t > 0.0 {
                        let px = x + t * dx;
                        if (-4.0..=6.0).contains(&px) {
                            best = best.min(t);
                        }
                    }
                }
                best
            })
            .collect()
    }
    #[test]
    fn real_mapping_external_and_laser_odometry_with_offset() {
        let _guard = OWNER.lock().unwrap_or_else(|error| error.into_inner());
        for external in [true, false] {
            let config = Config {
                base_from_laser: [0.22, -0.08, 0.15],
                ..Config::default()
            };
            let mut slam = Slam::new(config.clone()).unwrap();
            let mut last = Pose::default();
            let mut accepted = 0;
            for i in 0..40u64 {
                let actual = Pose {
                    x: i as f64 * 0.02,
                    y: 0.05 * (i as f64 * 0.1).sin(),
                    yaw: i as f64 * 0.002,
                };
                let output = slam
                    .process(
                        1_000_000_000 + i * 100_000_000,
                        i,
                        &room(&config, actual),
                        external.then_some(actual),
                    )
                    .unwrap();
                assert_ne!(output.disposition, Disposition::MatchRejected, "{output:?}");
                if output.accepted() {
                    accepted += 1;
                }
                last = output.output.map_from_base;
            }
            assert!(accepted > 5, "必须真实建立多个图节点");
            assert!((last.x - 0.78).abs() < 0.10, "{external}: {last:?}");
            assert!((last.yaw - 0.078).abs() < 0.06, "{last:?}");
            let grid = slam.occupancy_grid().unwrap();
            assert!(grid.cells.iter().filter(|v| **v == 100).count() > 100);
            assert!(grid.cells.iter().filter(|v| **v == 0).count() > 1000);
        }
    }
    #[test]
    fn closed_loop_runs_real_pose_graph_optimization() {
        let _guard = OWNER.lock().unwrap_or_else(|error| error.into_inner());
        let config = Config {
            minimum_travel: 0.06,
            minimum_heading: 0.04,
            use_scan_barycenter: false,
            link_maximum_distance: 1.0,
            loop_search_distance: 2.0,
            loop_minimum_chain: 5,
            ..Config::default()
        };
        let mut slam = Slam::new(config.clone()).unwrap();
        let mut optimizations = 0;
        let mut last = Pose::default();
        for i in 0..=200u64 {
            let angle = std::f64::consts::TAU * i as f64 / 200.0;
            let actual = Pose {
                x: 2.0 * angle.cos(),
                y: 2.0 * angle.sin(),
                yaw: angle,
            };
            let drifted = Pose {
                x: actual.x + 0.001 * i as f64,
                y: actual.y + 0.0005 * i as f64,
                yaw: actual.yaw,
            };
            let result = slam
                .process(
                    1_000_000_000 + i * 100_000_000,
                    i,
                    &room(&config, actual),
                    Some(drifted),
                )
                .unwrap();
            if result.accepted() {
                last = result.output.map_from_base;
            }
            optimizations = result.output.optimizations;
        }
        assert!(optimizations > 0, "必须由真实回环触发图优化");
        assert!(
            (last.x - 2.0).abs() < 0.15 && last.y.abs() < 0.15,
            "{last:?}"
        );
    }
    #[test]
    fn unobservable_corridor_reports_degradation_without_advancing_map() {
        let _guard=OWNER.lock().unwrap_or_else(|e|e.into_inner());
        let config=Config::default();let mut slam=Slam::new(config.clone()).unwrap();
        let ranges:Vec<f64>=(0..config.scan_count).map(|i| {
            let sine=(config.angle_min+f64::from(i)*config.angle_increment).sin().abs();
            let range=2.0/sine;if range<config.range_max {range}else{f64::NAN}
        }).collect();
        assert!(slam.process(1_000_000_000,0,&ranges,None).unwrap().accepted());
        let before=slam.occupancy_grid().unwrap();
        let error=slam.process(1_100_000_000,1,&ranges,None).expect_err("不可观测方向不能当成零运动成功");
        assert!(error.message.contains("退化"),"{error}");
        let after=slam.occupancy_grid().unwrap();
        assert_eq!(before.metadata.graph_revision,after.metadata.graph_revision);
        assert_eq!(before.cells,after.cells);
    }
    #[test]
    fn invalid_inputs_do_not_create_map_and_source_cannot_change() {
        let _guard = OWNER.lock().unwrap_or_else(|error| error.into_inner());
        let config = Config::default();
        let mut slam = Slam::new(config.clone()).unwrap();
        assert!(Slam::new(config.clone()).is_err());
        assert!(slam.process(1, 0, &vec![f64::NAN; 360], None).is_err());
        assert!(slam.occupancy_grid().is_err());
        let ranges = room(&config, Pose::default());
        assert!(slam
            .process(1_000_000_000, 1, &ranges, None)
            .unwrap()
            .accepted());
        assert!(slam.process(1_000_000_000, 2, &ranges, None).is_err());
        assert!(slam
            .process(1_100_000_000, 2, &ranges, Some(Pose::default()))
            .is_err());
        assert!(slam.process(2_000_000_000, 2, &ranges, None).is_err());
    }
    #[test]
    fn graph_save_restore_localization_and_resume_are_real() {
        let _guard = OWNER.lock().unwrap_or_else(|error| error.into_inner());
        let path = std::env::temp_dir().join(format!("nuwax-slam-core-{}", std::process::id()));
        std::fs::create_dir(&path).unwrap();
        let prefix = path.join("graph");
        let config = Config::default();
        let mut slam = Slam::new(config.clone()).unwrap();
        for i in 0..30u64 {
            let p = Pose {
                x: i as f64 * 0.03,
                y: 0.0,
                yaw: 0.0,
            };
            slam.process(
                1_000_000_000 + i * 100_000_000,
                i,
                &room(&config, p),
                Some(p),
            )
            .unwrap();
        }
        let before = slam.occupancy_grid().unwrap();
        slam.save(&prefix).unwrap();
        assert!(slam.save(&prefix).is_err());
        drop(slam);
        for localization in [true, false] {
            let mut restored = Slam::new(config.clone()).unwrap();
            restored
                .load(
                    &prefix,
                    localization,
                    Pose {
                        x: 0.8,
                        y: 0.0,
                        yaw: 0.0,
                    },
                )
                .unwrap();
            let grid = restored.occupancy_grid().unwrap();
            assert_eq!(before.metadata.width, grid.metadata.width);
            assert_eq!(before.cells, grid.cells);
            let mut accepted_tracking = 0;
            let mut base_nodes = None;
            let mut final_nodes = 0;
            for i in 0..180u64 {
                let actual = Pose {
                    x: 0.8 + i as f64 * 0.005,
                    y: 0.0,
                    yaw: 0.0,
                };
                let odom = Pose {
                    x: i as f64 * 0.005,
                    y: 0.0,
                    yaw: 0.0,
                };
                let result = restored
                    .process(
                        10_000_000_000 + i * 100_000_000,
                        i,
                        &room(&config, actual),
                        Some(odom),
                    )
                    .unwrap();
                assert_ne!(result.disposition, Disposition::MatchRejected, "{result:?}");
                assert!(
                    (result.output.map_from_base.x - actual.x).abs() < 0.10,
                    "{result:?}"
                );
                if result.accepted() {
                    accepted_tracking += 1;
                }
                let original_nodes = *base_nodes
                    .get_or_insert(result.output.node_count - u32::from(result.accepted()));
                final_nodes = result.output.node_count;
                if localization {
                    assert!(result.output.node_count <= config.local_window + original_nodes);
                }
            }
            if localization {
                assert!(
                    accepted_tracking > config.local_window,
                    "必须实际触发滑窗淘汰"
                );
                assert_eq!(final_nodes, base_nodes.unwrap() + config.local_window);
            }
            if !localization {
                restored.save(&path.join("continued")).unwrap();
            }
        }
        std::fs::remove_dir_all(path).unwrap();
    }
}
