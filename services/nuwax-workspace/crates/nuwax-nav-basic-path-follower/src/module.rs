//! P0-13 Module 副作用层。
//!
//! 把 `BasicPathFollowerCore`(纯算法)包成有 lifecycle + 端口的 Module 真副作用对象:
//! - `start()` / `stop()`:idempotent,后台 spawn 10 Hz lifecycle 任务
//! - `on_path()` / `on_odom()`:数据输入口(喂入最新 path 与当前 pose)
//! - `cmd_vel()`:数据输出口(读最新计算出的 twist)
//! - `goal_reached()`:事件信号口
//!
//! 端口 mock 通过 `tokio::sync::mpsc` / `watch` 提供;测试中可直接订阅。
//!
//! 与 C++ 路径跟随器(`nuwax-nav-path-follower-bridge`)职责不重叠:
//! 本 Module 是 Rust fallback / 公式核心,只算 twist,不接 native transport。

#![allow(clippy::module_name_repetitions)]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::{Deserialize, Serialize};
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;

use nuwax_nav_base::NavError;
use nuwax_nav_types_messages::{Path2D, Point2 as MsgPoint2, PoseStamped, Twist2};
use nuwax_numeric::Point2 as NumPoint2;

use crate::BasicPathFollowerConfig;

/// 默认控制频率(Hz),与 Python `BasicPathFollower` 默认对齐
pub const DEFAULT_CONTROL_RATE_HZ: u32 = 10;

/// 默认 tick 周期
pub const DEFAULT_TICK_PERIOD: Duration = Duration::from_millis(100);

/// Module 配置:端口 topic 名 + 控制频率 + robot_id 标识
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BasicPathFollowerModuleConfig {
    /// 机器人 id(用于日志/多实例区分,本期不直接绑定 transport)
    pub robot_id: String,
    /// 控制循环频率(Hz)
    pub control_rate_hz: u32,
    /// cmd_vel 输出 topic(占位字段,本期无 native 绑定)
    pub cmd_vel_topic: String,
    /// 路径输入 topic(占位字段)
    pub path_topic: String,
    /// odom 输入 topic(占位字段)
    pub odom_topic: String,
    /// 显式 stop 输入 topic(占位字段)
    pub stop_topic: String,
    /// 控制器公式核心配置
    pub core_config: BasicPathFollowerConfig,
}

impl Default for BasicPathFollowerModuleConfig {
    fn default() -> Self {
        Self {
            robot_id: "default_robot".to_string(),
            control_rate_hz: DEFAULT_CONTROL_RATE_HZ,
            cmd_vel_topic: "/cmd_vel".to_string(),
            path_topic: "/plan_path".to_string(),
            odom_topic: "/odom".to_string(),
            stop_topic: "/stop_movement".to_string(),
            core_config: BasicPathFollowerConfig::default(),
        }
    }
}

/// Module trait:对外暴露的 lifecycle + 端口能力
///
/// 设计要点:
/// - `Send + 'static`:必须能在 `tokio::spawn` / 多线程 task 间移动;
/// - 不直接绑定 ROS transport,而是 in-memory 端口 + `start()` 起 lifecycle;
/// - 端口订阅者由外部构造(本期用 `tokio::sync::mpsc::Receiver` / `watch::Receiver`)。
pub trait Module: Send + 'static {
    /// 启动 lifecycle(幂等)
    fn start(&mut self) -> Result<(), NavError>;
    /// 停止 lifecycle(幂等)
    fn stop(&mut self) -> Result<(), NavError>;
    /// 注入最新路径
    fn on_path(&mut self, path: Path2D);
    /// 注入最新里程计位姿
    fn on_odom(&mut self, odom: PoseStamped);
    /// 读取最新 twist(若无则 None)
    fn cmd_vel(&self) -> Option<Twist2>;
    /// 是否刚到达目标
    fn goal_reached(&self) -> bool;
    /// 订阅 cmd_vel 输出(返回 mpsc::Receiver,逐事件消费)
    fn subscribe_cmd_vel(&self) -> mpsc::Receiver<Twist2>;
}

/// 把 Foundation msg `Point2` 转 nalgebra `Point2`(给 core 用)
fn msg_to_num_point(p: MsgPoint2) -> NumPoint2 {
    NumPoint2::new(p.x, p.y)
}

/// 把 Foundation `Twist2` 的 linear.x 折算成 scalar 速度(m/s),给 core 公式用。
///
/// core 仅使用 `linear.x`(y 分量由 forward kinematics 决定);这层只做 2D 平面映射。
fn twist_linear_x(tw: &Twist2) -> f64 {
    tw.linear.x.hypot(tw.linear.y)
}

/// 端口共享状态(跨线程,给 lifecycle task 用)
#[derive(Default)]
struct SharedInputs {
    /// 最新路径(已转为 nalgebra Point2)
    path: Option<Vec<NumPoint2>>,
    /// 最新里程计位姿
    odom: Option<PoseStamped>,
}

/// 端口共享输出(跨线程,给 lifecycle task 写、调用方读)
#[derive(Default)]
struct SharedOutputs {
    /// 最新 twist
    cmd_vel: Option<Twist2>,
    /// 最近是否到达目标
    goal_reached: bool,
}

/// `BasicPathFollowerModule` 真副作用对象。
pub struct BasicPathFollowerModule {
    core_config: BasicPathFollowerConfig,
    config: BasicPathFollowerModuleConfig,
    running: bool,
    inputs: Arc<Mutex<SharedInputs>>,
    outputs: Arc<Mutex<SharedOutputs>>,
    cmd_vel_tx: watch::Sender<Twist2>,
    lifecycle_handle: Option<JoinHandle<()>>,
}

impl BasicPathFollowerModule {
    /// 构造(立即校验 core config)
    pub fn new(config: BasicPathFollowerModuleConfig) -> Result<Self, NavError> {
        if config.control_rate_hz == 0 {
            return Err(NavError::InvalidInput {
                field: "control_rate_hz",
                reason: "must be > 0".into(),
            });
        }
        config
            .core_config
            .validate()
            .map_err(|e| NavError::InvalidInput {
                field: "core_config",
                reason: e.to_string(),
            })?;
        let (cmd_vel_tx, _initial_rx) = watch::channel(Twist2::ZERO);
        Ok(Self {
            core_config: config.core_config.clone(),
            config,
            running: false,
            inputs: Arc::new(Mutex::new(SharedInputs::default())),
            outputs: Arc::new(Mutex::new(SharedOutputs::default())),
            cmd_vel_tx,
            lifecycle_handle: None,
        })
    }

    /// 当前 cmd_vel 输出 topic(调试用)
    #[must_use]
    pub fn cmd_vel_topic(&self) -> &str {
        &self.config.cmd_vel_topic
    }

    /// 当前 robot_id
    #[must_use]
    pub fn robot_id(&self) -> &str {
        &self.config.robot_id
    }

    /// 同步单步 tick:在 caller 线程里跑一次 core step(便于测试与外部驱动)。
    ///
    /// 不依赖 lifecycle,可独立调用。
    pub fn tick_and_publish(&mut self) -> Option<Twist2> {
        let (path, odom) = {
            let Ok(g) = self.inputs.lock() else {
                return None;
            };
            (g.path.clone(), g.odom.clone())
        };
        let out = self.compute_step(&path, odom.as_ref());
        self.publish_step(out)
    }

    /// 把 `(path, odom)` 喂入 core 算一次,返回 StepOutput
    fn compute_step(
        &self,
        path: &Option<Vec<NumPoint2>>,
        odom: Option<&PoseStamped>,
    ) -> crate::StepOutput {
        let mut core = match crate::BasicPathFollowerCore::new(self.core_config.clone()) {
            Ok(c) => c,
            Err(_) => return crate::StepOutput::no_path(),
        };
        if let Some(wps) = path.as_ref() {
            if !wps.is_empty() {
                core.set_path(wps.clone());
            }
        }
        match odom {
            Some(od) => {
                let position = NumPoint2::new(od.pose.position.x, od.pose.position.y);
                // 由 quaternion (x,y,z,w) 提取 yaw:绕 z 轴旋转, yaw = 2*atan2(z,w)
                let yaw = od.pose.orientation.z.atan2(od.pose.orientation.w) * 2.0;
                core.step(position, yaw)
            }
            None => crate::StepOutput::no_path(),
        }
    }

    fn publish_step(&mut self, out: crate::StepOutput) -> Option<Twist2> {
        // core 的 ControllerCommand { linear: f64, angular: f64 } 转 Foundation Twist2
        // 线速度放 linear.x;angular 直接放 angular_z(yaw 平面控制)
        let tw = Twist2 {
            linear: MsgPoint2::new(out.command.linear, 0.0),
            angular_z: out.command.angular,
        };
        if let Ok(mut g) = self.outputs.lock() {
            g.cmd_vel = Some(tw);
            g.goal_reached = out.goal_reached;
        }
        if out.goal_reached {
            if let Ok(mut g) = self.inputs.lock() {
                g.path = None;
            }
        }
        let _ = self.cmd_vel_tx.send(tw);
        Some(tw)
    }
}

impl Module for BasicPathFollowerModule {
    fn start(&mut self) -> Result<(), NavError> {
        if self.running {
            return Ok(()); // 幂等
        }
        let period =
            Duration::from_millis(1_000_u64 / u64::from(self.config.control_rate_hz));
        let inputs = Arc::clone(&self.inputs);
        let outputs = Arc::clone(&self.outputs);
        let core_cfg = self.core_config.clone();
        let cmd_vel_tx = self.cmd_vel_tx.clone();
        // lifecycle task:周期性 step 并发布到 watch 端口。
        // core 是 &mut self -- 在 task 内独立构造,避免与 caller 抢锁。
        let handle = tokio::spawn(async move {
            let mut ticker = tokio::time::interval(period);
            // 跳过首次立即 tick,避免启动 burst
            ticker.tick().await;
            loop {
                ticker.tick().await;
                let (path, odom) = {
                    let Ok(g) = inputs.lock() else { continue };
                    (g.path.clone(), g.odom.clone())
                };
                let out = {
                    let mut core = match crate::BasicPathFollowerCore::new(core_cfg.clone()) {
                        Ok(c) => c,
                        Err(_) => continue,
                    };
                    if let Some(wps) = path.as_ref() {
                        if !wps.is_empty() {
                            core.set_path(wps.clone());
                        }
                    }
                    match odom.as_ref() {
                        Some(od) => {
                            let position =
                                NumPoint2::new(od.pose.position.x, od.pose.position.y);
                            let yaw =
                                od.pose.orientation.z.atan2(od.pose.orientation.w) * 2.0;
                            core.step(position, yaw)
                        }
                        None => crate::StepOutput::no_path(),
                    }
                };
                let tw = Twist2 {
                    linear: MsgPoint2::new(out.command.linear, 0.0),
                    angular_z: out.command.angular,
                };
                if let Ok(mut g) = outputs.lock() {
                    g.cmd_vel = Some(tw);
                    g.goal_reached = out.goal_reached;
                }
                if out.goal_reached {
                    if let Ok(mut g) = inputs.lock() {
                        g.path = None;
                    }
                }
                let _ = cmd_vel_tx.send(tw);
            }
        });
        self.lifecycle_handle = Some(handle);
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NavError> {
        if !self.running {
            return Ok(()); // 幂等
        }
        if let Some(handle) = self.lifecycle_handle.take() {
            handle.abort();
        }
        self.running = false;
        self.lifecycle_handle = None;
        Ok(())
    }

    fn on_path(&mut self, path: Path2D) {
        if let Ok(mut g) = self.inputs.lock() {
            g.path = if path.points.is_empty() {
                None
            } else {
                Some(path.points.into_iter().map(msg_to_num_point).collect())
            };
        }
    }

    fn on_odom(&mut self, odom: PoseStamped) {
        if let Ok(mut g) = self.inputs.lock() {
            g.odom = Some(odom);
        }
    }

    fn cmd_vel(&self) -> Option<Twist2> {
        self.outputs.lock().ok().and_then(|g| g.cmd_vel)
    }

    fn goal_reached(&self) -> bool {
        self.outputs.lock().map(|g| g.goal_reached).unwrap_or(false)
    }

    fn subscribe_cmd_vel(&self) -> mpsc::Receiver<Twist2> {
        let (tx, rx) = mpsc::channel::<Twist2>(16);
        let mut watch_rx = self.cmd_vel_tx.subscribe();
        tokio::spawn(async move {
            loop {
                if watch_rx.changed().await.is_err() {
                    break;
                }
                let latest = *watch_rx.borrow();
                if tx.send(latest).await.is_err() {
                    break;
                }
            }
        });
        rx
    }
}

impl Drop for BasicPathFollowerModule {
    fn drop(&mut self) {
        if let Some(handle) = self.lifecycle_handle.take() {
            handle.abort();
        }
    }
}

#[allow(dead_code)]
fn _silence_unused_twist(_tw: &Twist2) -> f64 {
    twist_linear_x(_tw)
}