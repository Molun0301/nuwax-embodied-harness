//! P0-13 MovementManagerModule 副作用层。
//!
//! 把 `MovementManager::cancel_navigation()`(只返 `Vec<MovementEvent>`,由 caller 转发)
//! 包成真副作用 Module:直接通过 `tokio::sync::broadcast` 端口发
//! `MovementCommand::Stop` 和 NaN `PoseStamped` 给下游订阅者。
//!
//! 端口语义:
//! - `stop_out`:`broadcast::Sender<MovementCommand>`,发 `Stop` 取消运动
//! - `nan_goal_out`:`broadcast::Sender<PoseStamped>`,发 NaN pose 给 nav stack
//!   表示"无目标"
//!
//! 行为对账源:`dimos/navigation/movement_manager/movement_manager.py`
//! 的 `cancel_navigation` 路径(原 Python 在 cancel 时直接发布 stop_movement=true
//! 与 NaN goal,我们把这条 publish 链路下沉到 Module 里)。

#![allow(clippy::module_name_repetitions)]

use serde::{Deserialize, Serialize};
use tokio::sync::broadcast;

use nuwax_nav_base::NavError;
use nuwax_nav_types_messages::PoseStamped;

use crate::{CancelGoalPolicy, MovementManager, MovementManagerConfig};

/// 广播端口默认容量(broadcast channel size)
pub const DEFAULT_BROADCAST_CAPACITY: usize = 16;

/// Module 配置
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementManagerModuleConfig {
    /// 底层 MovementManager 配置
    pub manager_config: MovementManagerConfig,
    /// broadcast 端口容量(默认 16)
    pub broadcast_capacity: usize,
    /// robot_id(用于日志区分)
    pub robot_id: String,
}

impl Default for MovementManagerModuleConfig {
    fn default() -> Self {
        Self {
            manager_config: MovementManagerConfig::default(),
            broadcast_capacity: DEFAULT_BROADCAST_CAPACITY,
            robot_id: "default_robot".to_string(),
        }
    }
}

/// MovementManager 对外发出的高层命令
///
/// 设计:`Stop` 是 typed 命令(可扩展到 `Pause / Resume / SetGoal` 等);
/// NaN goal 走单独的 `PoseStamped` 通道,语义清晰。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MovementCommand {
    /// 立即停止运动(twist=0 + stop 标记)
    Stop,
    /// 暂停当前 nav 动作(stop 但保留当前 goal)
    Pause,
    /// 恢复被 pause 的 nav 动作
    Resume,
}

impl MovementCommand {
    /// 是否为 stop 类(Stop / Pause 都属于"现在不要往前走")
    #[must_use]
    pub fn is_stop_like(&self) -> bool {
        matches!(self, Self::Stop | Self::Pause)
    }
}

/// `MovementManagerModule` 真副作用对象。
///
/// 字段:
/// - `manager`:MovementManager 核心(纯函数式 cancel/click/cmd_vel 处理);
/// - `running`:lifecycle 状态(lifecycle 任务 spawn,本期保留字段);
/// - `stop_out`:广播 stop 命令端口;
/// - `nan_goal_out`:广播 NaN goal 端口。
pub struct MovementManagerModule {
    manager: MovementManager,
    config: MovementManagerModuleConfig,
    running: bool,
    stop_out: broadcast::Sender<MovementCommand>,
    nan_goal_out: broadcast::Sender<PoseStamped>,
}

impl MovementManagerModule {
    /// 构造
    pub fn new(config: MovementManagerModuleConfig) -> Result<Self, NavError> {
        let manager = MovementManager::new(config.manager_config.clone()).map_err(|e| {
            NavError::InvalidInput {
                field: "manager_config",
                reason: e.to_string(),
            }
        })?;
        let cap = if config.broadcast_capacity == 0 {
            DEFAULT_BROADCAST_CAPACITY
        } else {
            config.broadcast_capacity
        };
        let (stop_out, _) = broadcast::channel::<MovementCommand>(cap);
        let (nan_goal_out, _) = broadcast::channel::<PoseStamped>(cap);
        Ok(Self {
            manager,
            config,
            running: false,
            stop_out,
            nan_goal_out,
        })
    }

    /// robot_id
    #[must_use]
    pub fn robot_id(&self) -> &str {
        &self.config.robot_id
    }

    /// 暴露 stop_out 广播 sender(给外部订阅用)
    #[must_use]
    pub fn stop_out(&self) -> broadcast::Sender<MovementCommand> {
        self.stop_out.clone()
    }

    /// 暴露 nan_goal_out 广播 sender
    #[must_use]
    pub fn nan_goal_out(&self) -> broadcast::Sender<PoseStamped> {
        self.nan_goal_out.clone()
    }

    /// 订阅 stop 命令(返回新 receiver)
    pub fn subscribe_stop(&self) -> broadcast::Receiver<MovementCommand> {
        self.stop_out.subscribe()
    }

    /// 订阅 NaN goal
    pub fn subscribe_nan_goal(&self) -> broadcast::Receiver<PoseStamped> {
        self.nan_goal_out.subscribe()
    }

    /// 当前 cancel policy
    #[must_use]
    pub fn cancel_policy(&self) -> CancelGoalPolicy {
        self.manager.config().cancel_goal_policy
    }

    /// 构造一个 NaN-filled `PoseStamped`(无目标信号)。
    ///
    /// Foundation `Pose3` 各字段默认 0;为表达"NaN goal",position 与
    /// orientation 全部置 NaN,这样下游订阅者可凭 `is_nan()` 识别。
    fn build_nan_goal_stamped() -> PoseStamped {
        // 通过 `PoseStamped::default()` + 字段 patch 的方式构造。
        // 注意:`PoseStamped` 字段为 `frame_id / stamp / pose`,
        // frame_id 与 stamp 在 Foundation 中是 enum + Timestamp,不便写 NaN;
        // 这里只把 pose 部分写 NaN(position + orientation 全 NaN)。
        use nuwax_nav_types_messages::{NavFrameId, Pose3, Quaternion, Timestamp};
        let nan = f64::NAN;
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            pose: Pose3 {
                position: nuwax_nav_types_messages::Point3 {
                    x: nan,
                    y: nan,
                    z: nan,
                },
                orientation: Quaternion {
                    x: nan,
                    y: nan,
                    z: nan,
                    w: nan,
                },
            },
        }
    }

    /// 主动触发 cancel:依 `publish_stop` / `publish_nan_goal` 真副作用发布。
    ///
    /// - `publish_stop=true` -> `stop_out.broadcast(MovementCommand::Stop)`
    /// - `publish_nan_goal=true` -> `nan_goal_out.broadcast(build_nan_goal_stamped())`
    ///
    /// 即使 `running=false` 也允许 publish(端口与 lifecycle 解耦)。
    /// 重复调用幂等(每次都会 broadcast,下游 consumer 各自 dedupe)。
    pub fn trigger_cancel(
        &mut self,
        publish_stop: bool,
        publish_nan_goal: bool,
    ) -> Result<(), NavError> {
        // 同步推进 manager 内部 cooldown 状态(影响后续 nav_cmd_vel 抑制),
        // 即使外部没有 teleop 信号,也允许显式 cancel。
        let _events = self.manager.cancel_navigation();

        if publish_stop {
            // broadcast 失败(0 receivers)不算 error -- 仍属正常路径。
            let _ = self.stop_out.send(MovementCommand::Stop);
        }
        if publish_nan_goal {
            let _ = self.nan_goal_out.send(Self::build_nan_goal_stamped());
        }
        Ok(())
    }

    /// 启动 lifecycle(本期留 stub -- MovementManager 本身是事件驱动的,
    /// 不需要后台 ticker;但保留 API 兼容 BasicPathFollower 的对称性)。
    ///
    /// 幂等。
    pub fn start(&mut self) -> Result<(), NavError> {
        self.running = true;
        Ok(())
    }

    /// 停止 lifecycle(幂等)
    pub fn stop(&mut self) -> Result<(), NavError> {
        self.running = false;
        Ok(())
    }

    /// 是否 running
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.running
    }
}

impl Drop for MovementManagerModule {
    fn drop(&mut self) {
        // broadcast Sender 在 drop 时自动关闭 channel;receiver 端会收到 RecvError::Closed。
        // 不需要额外清理。
    }
}