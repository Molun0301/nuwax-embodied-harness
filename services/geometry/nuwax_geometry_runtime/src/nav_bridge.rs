//! 导航传输桥。会话与目标状态由 Session 唯一持有。
use std::time::{Duration, Instant};

use futures::StreamExt;
use nuwax_codec::lcm::schema;
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::LcmTransport;
use tokio::sync::{mpsc, oneshot};
use tonic::Status;

use crate::proto;

/// 仅保存尚未发送的传输任务，不保存会话状态或已发送目标的副本。
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
        let transport = LcmTransport::new_real(&url).map_err(|error| error.to_string())?;
        Self::with_transport(transport, status_tx).await
    }

    async fn with_transport(
        transport: LcmTransport,
        status_tx: mpsc::UnboundedSender<proto::NavigationStatus>,
    ) -> Result<Self, String> {
        let state_topic = topic("NUWAX_LCM_CHANNEL_NAV_STATE", "/nav/state")?;
        let mut states = transport
            .subscribe(
                state_topic,
                Some(message_type("nuwax_msgs.NavigationState")?),
            )
            .await;
        let goal_topic = topic("NUWAX_LCM_CHANNEL_GOAL_POSE", "/nav/goal")?;
        let cancel_topic = topic("NUWAX_LCM_CHANNEL_NAV_CANCEL", "/nav/cancel")?;
        // 启动清目标只在这里发送一次。旧导航栈尚不支持空目标，B3 必须接入此语义。
        publish_cancel(&transport, &cancel_topic, "")
            .await
            .map_err(|error| error.to_string())?;
        let (commands, mut rx) = mpsc::unbounded_channel();
        let task = tokio::spawn(async move {
            let mut pending: Option<PendingGoal> = None;
            let mut input_ready = false;
            let mut stack_ready_at: Option<Instant> = None;
            loop {
                tokio::select! {
                    command = rx.recv() => match command {
                        Some(Command::Submit(goal, reply)) => {
                            let waiting = goal.status(proto::GoalStatus::WaitingInput, "等待导航输入");
                            pending = Some(goal);
                            let _ = status_tx.send(waiting);
                            let result = dispatch_pending(&transport, &goal_topic, &status_tx,
                                &mut pending, input_ready, stack_ready_at).await;
                            let _ = reply.send(result);
                        }
                        Some(Command::Cancel(id, reply)) => {
                            if pending.as_ref().is_some_and(|goal| id.is_empty() || goal.id == id) {
                                pending = None;
                            }
                            let _ = reply.send(publish_cancel(&transport, &cancel_topic, &id).await);
                        }
                        Some(Command::InputReady(ready)) => {
                            input_ready = ready;
                            if let Err(error) = dispatch_pending(&transport, &goal_topic, &status_tx,
                                &mut pending, input_ready, stack_ready_at).await {
                                tracing::warn!(%error, "待发送导航目标已经失败");
                            }
                        }
                        Some(Command::Shutdown(reply)) => { let _ = reply.send(()); break; }
                        None => break,
                    },
                    frame = states.next() => {
                        let frame = match frame {
                            Some(Ok(frame)) => frame,
                            Some(Err(error)) => {
                                fail_pending(&mut pending, &status_tx, &format!("导航状态传输失败：{error}"));
                                break;
                            }
                            None => {
                                fail_pending(&mut pending, &status_tx, "导航状态流已关闭");
                                break;
                            }
                        };
                        let Ok(state) = schema::decode_navigation_state_body(&frame.payload) else { continue; };
                        stack_ready_at = stack_inputs_ready(&state).then(Instant::now);
                        if !state.session_id.is_empty() {
                            if let Some(update) = state_update(state) { let _ = status_tx.send(update); }
                        }
                        if let Err(error) = dispatch_pending(&transport, &goal_topic, &status_tx,
                            &mut pending, input_ready, stack_ready_at).await {
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
    stack_ready_at: Option<Instant>,
) -> Result<(), Status> {
    if !input_ready
        || !stack_ready_at
            .is_some_and(|at| at.elapsed() <= stack_state_ttl().min(motion_sink_ttl()))
    {
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
                    message_type("nuwax_msgs.GoalEnvelope").map_err(Status::internal)?,
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

fn fail_pending(
    pending: &mut Option<PendingGoal>,
    status_tx: &mpsc::UnboundedSender<proto::NavigationStatus>,
    message: &str,
) {
    if let Some(goal) = pending.take() {
        let _ = status_tx.send(goal.status(proto::GoalStatus::Failed, message));
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
    let mut payload = Vec::new();
    encode_string(id, &mut payload)?;
    payload.extend_from_slice(&goal.motion_generation.to_be_bytes());
    payload.extend_from_slice(
        &schema::encode_pose_stamped_body(&schema::PoseStamped {
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
        })
        .map_err(|error| Status::invalid_argument(error.to_string()))?,
    );
    match goal.allow_reverse {
        Some(value) => {
            payload.push(1);
            payload.push(u8::from(value));
        }
        None => payload.push(0),
    }
    for option in [goal.position_tolerance_m, goal.yaw_tolerance_rad] {
        match option {
            Some(value) => {
                payload.push(1);
                payload.extend_from_slice(&value.to_be_bytes());
            }
            None => payload.push(0),
        }
    }
    let count =
        i32::try_from(boundary.len()).map_err(|_| Status::invalid_argument("探索边界过大"))?;
    payload.extend_from_slice(&count.to_be_bytes());
    for point in boundary {
        for coordinate in point {
            payload.extend_from_slice(&coordinate.to_be_bytes());
        }
    }
    Ok(payload)
}

fn encode_string(value: &str, payload: &mut Vec<u8>) -> Result<(), Status> {
    let length =
        i32::try_from(value.len() + 1).map_err(|_| Status::invalid_argument("目标号过长"))?;
    payload.extend_from_slice(&length.to_be_bytes());
    payload.extend_from_slice(value.as_bytes());
    payload.push(0);
    Ok(())
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

fn stack_inputs_ready(state: &schema::NavigationState) -> bool {
    use nuwax_nav_types_messages::source_time;
    let now = unix_ns();
    state.sink_ready == 1
        && !state.planner_map_frame.is_empty()
        && state.planner_map_frame == state.planner_pose_frame
        && [
            state.pose_source_timestamp_ns,
            state.costmap_source_timestamp_ns,
        ]
        .into_iter()
        .all(|timestamp| {
            timestamp > 0
                && source_time::data_age_ns_at(timestamp as u64, now)
                    .is_ok_and(|age| age <= 2_000_000_000)
        })
}

fn state_update(state: schema::NavigationState) -> Option<proto::NavigationStatus> {
    let status = match state.state {
        0 if state.message.starts_with("CANCELLED:") => proto::GoalStatus::Cancelled,
        0 => return None,
        1 => proto::GoalStatus::Planning,
        2 => proto::GoalStatus::Executing,
        3 => proto::GoalStatus::Succeeded,
        4 if state.message.starts_with("LEASE_NOT_GRANTED:")
            || state.message.starts_with("STOPPED:") =>
        {
            proto::GoalStatus::Stopped
        }
        4 => proto::GoalStatus::Failed,
        _ => return None,
    };
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
        no_feasible_path: status == proto::GoalStatus::Planning
            && state.message.contains("NO_FEASIBLE_PATH"),
        message: state.message,
        // 旧状态消息只有直线距离，不能当成沿路线剩余距离。
        remaining_distance_m: 0.0,
        planned_path: if terminal {
            Vec::new()
        } else {
            state
                .local_path
                .into_iter()
                .map(|p| proto::Vec3 {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                })
                .collect()
        },
        ..Default::default()
    })
}

fn configured_duration(name: &str, default: Duration) -> Duration {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .map(Duration::from_millis)
        .unwrap_or(default)
}
fn stack_state_ttl() -> Duration {
    configured_duration("NUWAX_NAV_STACK_STATE_TTL_MS", Duration::from_secs(2))
}
fn motion_sink_ttl() -> Duration {
    configured_duration("NUWAX_NAV_MOTION_SINK_TTL_MS", Duration::from_secs(1))
}

fn message_type(value: &str) -> Result<LcmMessageType, String> {
    LcmMessageType::new(value).map_err(|error| error.to_string())
}
fn topic(key: &str, default: &str) -> Result<LcmTopic, String> {
    let value = std::env::var(key).unwrap_or_else(|_| default.into());
    LcmTopic::new(value.split('#').next().unwrap_or(default)).map_err(|error| error.to_string())
}
fn unix_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

#[cfg(test)]
#[path = "nav_bridge_tests.rs"]
mod tests;
