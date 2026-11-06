//! 导航传输桥。会话与目标状态由 Session 唯一持有。
use std::sync::Arc;
#[cfg(test)]
use std::time::Duration;

use futures::StreamExt;
use nuwax_codec::lcm::schema;
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::{LcmFrameStream, LcmTransport};
use tokio::sync::{mpsc, oneshot};
use tonic::Status;

use crate::proto;

/// 只持有导航传输与当前目标的状态投递信息；会话状态由Session拥有。
pub struct NavBridge {
    commands: mpsc::UnboundedSender<Command>,
    task: tokio::task::JoinHandle<()>,
}

enum Command {
    Submit(PendingGoal, oneshot::Sender<Result<(), Status>>),
    Cancel(String, oneshot::Sender<Result<(), Status>>),
    InputReady(bool),
    Shutdown(oneshot::Sender<()>),
}

struct PendingGoal {
    id: String,
    goal: proto::NavigationGoal,
    exploration_child: bool,
    boundary: Vec<[f64; 2]>,
}

impl NavBridge {
    pub async fn start(
        status_tx: mpsc::UnboundedSender<proto::NavigationStatus>,
    ) -> Result<Self, String> {
        let url = std::env::var("NUWAX_LCM_URL")
            .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=0".into());
        Self::connect(
            Arc::new(move || LcmTransport::new_real(&url).map_err(|error| error.to_string())),
            status_tx,
        )
        .await
    }

    #[cfg(test)]
    pub(crate) async fn with_transport(
        transport: LcmTransport,
        status_tx: mpsc::UnboundedSender<proto::NavigationStatus>,
    ) -> Result<Self, String> {
        Self::connect(Arc::new(move || Ok(transport.clone())), status_tx).await
    }

    async fn connect(
        create_transport: Arc<dyn Fn() -> Result<LcmTransport, String> + Send + Sync>,
        status_tx: mpsc::UnboundedSender<proto::NavigationStatus>,
    ) -> Result<Self, String> {
        let state_topic = topic("NUWAX_LCM_CHANNEL_NAV_STATE", "/nav/state")?;
        let goal_topic = topic("NUWAX_LCM_CHANNEL_GOAL_POSE", "/nav/goal")?;
        let cancel_topic = topic("NUWAX_LCM_CHANNEL_NAV_CANCEL", "/nav/cancel")?;
        let initial = create_transport()?;
        let initial_states = subscribe_states(&initial, &state_topic).await?;
        publish_cancel(&initial, &cancel_topic, "")
            .await
            .map_err(|error| error.to_string())?;
        let (commands, mut rx) = mpsc::unbounded_channel();
        let task = tokio::spawn(async move {
            let mut transport = Some(initial);
            let mut states = Some(initial_states);
            let mut pending: Option<PendingGoal> = None;
            let mut active: Option<proto::NavigationStatus> = None;
            let mut input_ready = false;
            let mut stack_ready = false;
            loop {
                tokio::select! {
                    command = rx.recv() => {
                        let Some(command) = command else { break; };
                        // 状态断流后只在下一条目标或取消命令重建，不重发旧目标。
                        if states.is_none() && matches!(&command, Command::Submit(..) | Command::Cancel(..)) {
                            match create_transport() {
                                Ok(next) => match subscribe_states(&next, &state_topic).await {
                                    Ok(next_states) => { transport = Some(next); states = Some(next_states); }
                                    Err(reason) => { reply_disconnected(command, &status_tx, &reason); continue; }
                                },
                                Err(reason) => { reply_disconnected(command, &status_tx, &reason); continue; }
                            }
                        }
                        match command {
                            Command::Submit(goal, reply) => {
                                let waiting = goal.status(proto::GoalStatus::WaitingInput, "等待导航输入");
                                active = Some(waiting.clone());
                                pending = Some(goal);
                                let _ = status_tx.send(waiting);
                                let result = dispatch_pending(transport.as_ref().unwrap(), &goal_topic, &status_tx,
                                    &mut pending, input_ready, stack_ready).await;
                                if result.is_err() { active = None; }
                                let _ = reply.send(result);
                            }
                            Command::Cancel(id, reply) => {
                                if pending.as_ref().is_some_and(|goal| id.is_empty() || goal.id == id) { pending = None; }
                                if active.as_ref().is_some_and(|goal| id.is_empty() || goal.goal_id == id) { active = None; }
                                let _ = reply.send(publish_cancel(transport.as_ref().unwrap(), &cancel_topic, &id).await);
                            }
                            Command::InputReady(ready) => {
                                input_ready = ready;
                                if let Some(transport) = &transport {
                                    if let Err(error) = dispatch_pending(transport, &goal_topic, &status_tx,
                                        &mut pending, input_ready, stack_ready).await {
                                        active = None;
                                        tracing::warn!(%error, "待发送导航目标已经失败");
                                    }
                                }
                            }
                            Command::Shutdown(reply) => { let _ = reply.send(()); break; }
                        }
                    },
                    frame = async { states.as_mut().unwrap().next().await }, if states.is_some() => {
                        let frame = match frame {
                            Some(Ok(frame)) => frame,
                            failed => {
                                let reason = match failed {
                                    Some(Err(error)) => format!("导航状态传输失败：{error}"),
                                    None => "导航状态流已关闭".into(),
                                    _ => unreachable!(),
                                };
                                pending = None;
                                if let Some(mut goal) = active.take() {
                                    goal.status = proto::GoalStatus::Failed as i32;
                                    goal.message = reason;
                                    let _ = status_tx.send(goal);
                                }
                                states = None;
                                transport = None;
                                stack_ready = false;
                                continue;
                            }
                        };
                        let Ok(state) = schema::decode_navigation_status_body(&frame.payload) else { continue; };
                        stack_ready = state.input_ready;
                        if !state.session_id.is_empty() {
                            if let Some(update) = state_update(state) {
                                if active.as_ref().is_some_and(|goal| goal.goal_id == update.goal_id)
                                    && matches!(proto::GoalStatus::try_from(update.status), Ok(proto::GoalStatus::Succeeded | proto::GoalStatus::Failed | proto::GoalStatus::Cancelled | proto::GoalStatus::Stopped)) {
                                    active = None;
                                }
                                let _ = status_tx.send(update);
                            }
                        }
                        if let Err(error) = dispatch_pending(transport.as_ref().unwrap(), &goal_topic, &status_tx,
                            &mut pending, input_ready, stack_ready).await {
                            active = None;
                            tracing::warn!(%error, "待发送导航目标已经失败");
                        }
                    }
                }
            }
        });
        Ok(Self { commands, task })
    }

    pub async fn submit(
        &self,
        goal_id: String,
        goal: proto::NavigationGoal,
        exploration_child: bool,
    ) -> Result<(), Status> {
        self.submit_with_boundary(goal_id, goal, exploration_child, Vec::new())
            .await
    }

    pub async fn submit_with_boundary(
        &self,
        goal_id: String,
        goal: proto::NavigationGoal,
        exploration_child: bool,
        boundary: Vec<[f64; 2]>,
    ) -> Result<(), Status> {
        let (tx, rx) = oneshot::channel();
        self.commands
            .send(Command::Submit(
                PendingGoal {
                    id: goal_id,
                    goal,
                    exploration_child,
                    boundary,
                },
                tx,
            ))
            .map_err(|_| Status::unavailable("导航传输已停止"))?;
        rx.await
            .map_err(|_| Status::unavailable("导航传输已停止"))?
    }

    pub async fn cancel(&self, goal_id: &str) -> Result<(), Status> {
        let (tx, rx) = oneshot::channel();
        self.commands
            .send(Command::Cancel(goal_id.into(), tx))
            .map_err(|_| Status::unavailable("导航传输已停止"))?;
        rx.await
            .map_err(|_| Status::unavailable("导航传输已停止"))?
    }

    pub fn set_input_ready(&self, ready: bool) {
        let _ = self.commands.send(Command::InputReady(ready));
    }

    #[cfg(test)]
    pub(crate) async fn stop_for_test(&self) {
        self.task.abort();
        self.commands.closed().await;
    }
    pub async fn shutdown(self) {
        let (tx, rx) = oneshot::channel();
        let _ = self.commands.send(Command::Shutdown(tx));
        let _ = rx.await;
        let _ = self.task.await;
    }
}

impl PendingGoal {
    fn status(&self, status: proto::GoalStatus, message: &str) -> proto::NavigationStatus {
        proto::NavigationStatus {
            goal_id: self.id.clone(),
            status: status as i32,
            message: message.into(),
            target: self.goal.pose.clone(),
            exploration_child: self.exploration_child,
            ..Default::default()
        }
    }
}

async fn dispatch_pending(
    transport: &LcmTransport,
    topic: &LcmTopic,
    status_tx: &mpsc::UnboundedSender<proto::NavigationStatus>,
    pending: &mut Option<PendingGoal>,
    input_ready: bool,
    stack_ready: bool,
) -> Result<(), Status> {
    if !input_ready || !stack_ready {
        return Ok(());
    }
    let Some(goal) = pending.take() else {
        return Ok(());
    };
    let result = async {
        let payload = encode_goal(&goal.id, &goal.goal, &goal.boundary)?;
        transport
            .publish(
                RawLcmFrameBuilder::new(
                    topic.clone(),
                    message_type("nuwax_msgs.NavigationGoal").map_err(Status::internal)?,
                )
                .payload(payload)
                .build(),
            )
            .await
            .map_err(|error| Status::unavailable(format!("导航目标发送失败：{error}")))
    }
    .await;
    match result {
        Ok(()) => {
            let _ = status_tx.send(goal.status(proto::GoalStatus::Planning, "目标已交给导航栈"));
            Ok(())
        }
        Err(error) => {
            let _ = status_tx.send(goal.status(proto::GoalStatus::Failed, error.message()));
            Err(error)
        }
    }
}

async fn subscribe_states(
    transport: &LcmTransport,
    topic: &LcmTopic,
) -> Result<LcmFrameStream, String> {
    Ok(transport
        .subscribe(
            topic.clone(),
            Some(message_type("nuwax_msgs.NavigationStatus")?),
        )
        .await)
}

fn reply_disconnected(
    command: Command,
    status_tx: &mpsc::UnboundedSender<proto::NavigationStatus>,
    reason: &str,
) {
    let error = Status::unavailable(format!("导航通道重建失败：{reason}"));
    match command {
        Command::Submit(goal, reply) => {
            let _ = status_tx.send(goal.status(proto::GoalStatus::Failed, error.message()));
            let _ = reply.send(Err(error));
        }
        Command::Cancel(_, reply) => {
            let _ = reply.send(Err(error));
        }
        _ => unreachable!(),
    }
}

/// B3 对接载荷：目标号、运动代次、PoseStamped、三个可选执行约束；整数与浮点均为大端序。
pub(crate) fn encode_goal(
    id: &str,
    goal: &proto::NavigationGoal,
    boundary: &[[f64; 2]],
) -> Result<Vec<u8>, Status> {
    let pose = goal
        .pose
        .as_ref()
        .ok_or_else(|| Status::invalid_argument("导航目标缺少位姿"))?;
    let position = pose
        .position
        .as_ref()
        .ok_or_else(|| Status::invalid_argument("导航目标缺少位置"))?;
    let orientation = pose
        .orientation
        .as_ref()
        .ok_or_else(|| Status::invalid_argument("导航目标缺少朝向"))?;
    schema::encode_navigation_goal_body(&schema::NavigationGoal {
        session_id: id.into(),
        motion_generation: goal.motion_generation,
        goal: schema::PoseStamped {
            header: schema::Header {
                seq: 0,
                stamp_sec: (pose.timestamp_ns / 1_000_000_000) as i32,
                stamp_nsec: (pose.timestamp_ns % 1_000_000_000) as i32,
                frame_id: pose.frame_id.clone(),
            },
            pose: schema::Pose {
                position: [position.x, position.y, position.z],
                orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
            },
        },
        allow_reverse: goal.allow_reverse,
        position_tolerance_m: goal.position_tolerance_m,
        yaw_tolerance_rad: goal.yaw_tolerance_rad,
        boundary_points: boundary
            .iter()
            .map(|point| [point[0], point[1], 0.0])
            .collect(),
    })
    .map_err(|error| Status::invalid_argument(error.to_string()))
}

async fn publish_cancel(
    transport: &LcmTransport,
    topic: &LcmTopic,
    id: &str,
) -> Result<(), Status> {
    let payload = schema::encode_cancel_navigation_body(&schema::CancelNavigation {
        session_id: id.into(),
        ..Default::default()
    })
    .map_err(|error| Status::internal(error.to_string()))?;
    transport
        .publish(
            RawLcmFrameBuilder::new(
                topic.clone(),
                message_type("nuwax_msgs.CancelNavigation").map_err(Status::internal)?,
            )
            .payload(payload)
            .build(),
        )
        .await
        .map_err(|error| Status::unavailable(format!("导航取消发送失败：{error}")))
}

fn state_update(state: schema::NavigationStatus) -> Option<proto::NavigationStatus> {
    let status = proto::GoalStatus::try_from(state.goal_status).ok()?;
    if status == proto::GoalStatus::Unspecified {
        return None;
    }
    let terminal = matches!(
        status,
        proto::GoalStatus::Succeeded
            | proto::GoalStatus::Failed
            | proto::GoalStatus::Cancelled
            | proto::GoalStatus::Stopped
    );
    Some(proto::NavigationStatus {
        goal_id: state.session_id,
        status: status as i32,
        no_feasible_path: state.no_feasible_path,
        message: state.reason,
        remaining_distance_m: state.remaining_distance_m,
        planned_path: if terminal {
            Vec::new()
        } else {
            state
                .local_path
                .into_iter()
                .map(|point| proto::Vec3 {
                    x: point[0],
                    y: point[1],
                    z: point[2],
                })
                .collect()
        },
        ..Default::default()
    })
}

fn message_type(value: &str) -> Result<LcmMessageType, String> {
    LcmMessageType::new(value).map_err(|error| error.to_string())
}
fn topic(key: &str, default: &str) -> Result<LcmTopic, String> {
    let value = std::env::var(key).unwrap_or_else(|_| default.into());
    LcmTopic::new(value.split('#').next().unwrap_or(default)).map_err(|error| error.to_string())
}
#[cfg(test)]
fn unix_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

#[cfg(test)]
#[path = "nav_bridge_tests.rs"]
mod tests;
