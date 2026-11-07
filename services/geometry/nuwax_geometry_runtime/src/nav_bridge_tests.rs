use super::*;
use nuwax_protocol::RawLcmFrame;
use nuwax_transport::{LcmBackend, LcmFrameStream, TransportResult};
use std::sync::{
    atomic::{AtomicBool, AtomicUsize, Ordering},
    Arc, Mutex,
};

struct Capture {
    published: Mutex<Vec<RawLcmFrame>>,
    tx: Mutex<Option<tokio::sync::broadcast::Sender<Result<RawLcmFrame, String>>>>,
    fail_goal_publication: AtomicBool,
    goal_publish_attempts: AtomicUsize,
}
impl Capture {
    fn new() -> Self {
        let (tx, _) = tokio::sync::broadcast::channel(32);
        Self {
            published: Mutex::new(Vec::new()),
            tx: Mutex::new(Some(tx)),
            fail_goal_publication: AtomicBool::new(false),
            goal_publish_attempts: AtomicUsize::new(0),
        }
    }
    fn inject_ready(&self) {
        self.inject_ready_named("");
    }
    fn inject_ready_named(&self, id: &str) {
        self.inject_ready_generation(id, 17);
    }
    fn inject_ready_generation(&self, id: &str, runtime_generation: i64) {
        let now = unix_ns() as i64;
        let state = schema::NavigationStatus {
            session_id: id.into(),
            runtime_generation,
            state: 1,
            goal_status: schema::NAVIGATION_GOAL_STATUS_PLANNING,
            input_ready: true,
            sink_ready: 1,
            planner_map_frame: "map".into(),
            planner_pose_frame: "map".into(),
            pose_source_timestamp_ns: now,
            costmap_source_timestamp_ns: now,
            ..Default::default()
        };
        let frame = RawLcmFrameBuilder::new(
            LcmTopic::new("/nav/state").unwrap(),
            message_type("nuwax_msgs.NavigationStatus").unwrap(),
        )
        .payload(schema::encode_navigation_status_body(&state).unwrap())
        .build();
        self.tx
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .send(Ok(frame))
            .unwrap();
    }
    fn goals(&self) -> Vec<RawLcmFrame> {
        self.published
            .lock()
            .unwrap()
            .iter()
            .filter(|frame| frame.topic.channel() == "/nav/goal")
            .cloned()
            .collect()
    }
}
#[async_trait::async_trait]
impl LcmBackend for Capture {
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        if frame.topic.channel() == "/nav/goal" {
            self.goal_publish_attempts.fetch_add(1, Ordering::SeqCst);
            if self.fail_goal_publication.load(Ordering::SeqCst) {
                return Err(nuwax_transport::TransportError::LcmBackend(
                    "测试导航发布链路已断开".into(),
                ));
            }
        }
        self.published.lock().unwrap().push(frame);
        Ok(())
    }
    async fn subscribe(&self, topic: LcmTopic, _type: Option<LcmMessageType>) -> LcmFrameStream {
        let rx = self.tx.lock().unwrap().as_ref().unwrap().subscribe();
        Box::pin(
            futures::stream::unfold((rx, topic), |(mut rx, topic)| async move {
                loop {
                    match rx.recv().await {
                        Ok(Ok(frame)) if frame.topic == topic => {
                            return Some((Ok(frame), (rx, topic)))
                        }
                        Ok(Ok(_)) => continue,
                        Ok(Err(error)) => {
                            return Some((
                                Err(nuwax_transport::TransportError::LcmBackend(error)),
                                (rx, topic),
                            ))
                        }
                        Err(_) => return None,
                    }
                }
            })
            .boxed(),
        )
    }
    fn name(&self) -> &str {
        "导航传输行为测试"
    }
}
fn goal() -> proto::NavigationGoal {
    proto::NavigationGoal {
        motion_generation: 77,
        pose: Some(proto::Pose {
            position: Some(proto::Vec3 {
                x: 1.0,
                y: 2.0,
                z: 0.4,
            }),
            orientation: Some(proto::Quat {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
            frame_id: "map".into(),
            timestamp_ns: unix_ns(),
        }),
        ..Default::default()
    }
}
#[tokio::test]
async fn startup_sends_one_cancel_all() {
    let capture = Arc::new(Capture::new());
    let (tx, _) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    let frames = capture.published.lock().unwrap();
    assert_eq!(frames.len(), 1);
    let cancel = schema::decode_cancel_navigation_body(&frames[0].payload).unwrap();
    assert!(cancel.session_id.is_empty());
    drop(frames);
    bridge.shutdown().await;
    assert_eq!(capture.published.lock().unwrap().len(), 1);
}
#[tokio::test]
async fn waiting_goal_automatically_sends_when_both_inputs_ready() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    bridge
        .submit("pending".into(), goal(), false)
        .await
        .unwrap();
    assert_eq!(
        rx.recv().await.unwrap().status,
        proto::GoalStatus::WaitingInput as i32
    );
    assert!(capture.goals().is_empty());
    bridge.set_input_ready(Ok(()));
    capture.inject_ready();
    let status = tokio::time::timeout(Duration::from_secs(1), rx.recv())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(status.status, proto::GoalStatus::Planning as i32);
    assert_eq!(status.goal_id, "pending");
    assert_eq!(capture.goals().len(), 1);
    bridge.shutdown().await;
}
#[tokio::test]
async fn replaced_or_cancelled_waiting_goal_is_never_sent() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    bridge.submit("old".into(), goal(), false).await.unwrap();
    rx.recv().await.unwrap();
    bridge.submit("new".into(), goal(), false).await.unwrap();
    rx.recv().await.unwrap();
    bridge.set_input_ready(Ok(()));
    capture.inject_ready();
    let status = tokio::time::timeout(Duration::from_secs(1), rx.recv())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(status.goal_id, "new");
    assert_eq!(capture.goals().len(), 1);
    bridge.set_input_ready(Err("占据栅格缺失，来源年龄未知".into()));
    bridge
        .submit("cancelled".into(), goal(), false)
        .await
        .unwrap();
    rx.recv().await.unwrap();
    bridge.cancel("cancelled").await.unwrap();
    bridge.set_input_ready(Ok(()));
    bridge.shutdown().await;
    assert_eq!(capture.goals().len(), 1);
}

async fn next_status(
    rx: &mut mpsc::UnboundedReceiver<proto::NavigationStatus>,
) -> proto::NavigationStatus {
    tokio::time::timeout(Duration::from_secs(1), rx.recv())
        .await
        .unwrap()
        .unwrap()
}

#[tokio::test]
async fn deferred_publish_failure_reports_failed_from_each_readiness_trigger_without_retry() {
    for from_stack_state in [false, true] {
        let capture = Arc::new(Capture::new());
        let (tx, mut rx) = mpsc::unbounded_channel();
        let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
            .await
            .unwrap();
        bridge
            .submit("waiting".into(), goal(), false)
            .await
            .unwrap();
        assert_eq!(
            next_status(&mut rx).await.status,
            proto::GoalStatus::WaitingInput as i32
        );
        capture.fail_goal_publication.store(true, Ordering::SeqCst);
        if from_stack_state {
            bridge.set_input_ready(Ok(()));
            bridge.cancel("unrelated").await.unwrap();
            capture.inject_ready();
        } else {
            capture.inject_ready_named("heartbeat");
            assert_eq!(next_status(&mut rx).await.goal_id, "heartbeat");
            bridge.set_input_ready(Ok(()));
        }
        let failed = next_status(&mut rx).await;
        assert_eq!(failed.goal_id, "waiting");
        assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
        assert!(failed.message.contains("测试导航发布链路已断开"));
        bridge.set_input_ready(Ok(()));
        bridge.cancel("unrelated").await.unwrap();
        assert_eq!(capture.goal_publish_attempts.load(Ordering::SeqCst), 1);
        assert!(capture.goals().is_empty());
        bridge.shutdown().await;
    }
}

#[tokio::test]
async fn immediate_publish_failure_returns_unavailable_and_reports_failed() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    bridge.set_input_ready(Ok(()));
    bridge.cancel("unrelated").await.unwrap();
    capture.inject_ready_named("heartbeat");
    next_status(&mut rx).await;
    capture.fail_goal_publication.store(true, Ordering::SeqCst);
    let error = bridge
        .submit("immediate".into(), goal(), false)
        .await
        .unwrap_err();
    assert_eq!(error.code(), tonic::Code::Unavailable);
    assert_eq!(
        next_status(&mut rx).await.status,
        proto::GoalStatus::WaitingInput as i32
    );
    let failed = next_status(&mut rx).await;
    assert_eq!(failed.goal_id, "immediate");
    assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
    assert!(failed.message.contains("测试导航发布链路已断开"));
    assert_eq!(capture.goal_publish_attempts.load(Ordering::SeqCst), 1);
    bridge.shutdown().await;
}

#[tokio::test]
async fn state_stream_error_or_close_fails_goal_and_next_command_reconnects() {
    for error in [None, Some("测试状态接收链路已断开")] {
        let capture = Arc::new(Capture::new());
        let (tx, mut rx) = mpsc::unbounded_channel();
        let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
            .await
            .unwrap();
        bridge
            .submit("disconnected".into(), goal(), false)
            .await
            .unwrap();
        next_status(&mut rx).await;
        if let Some(message) = error {
            capture
                .tx
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .send(Err(message.into()))
                .unwrap();
        } else {
            capture.tx.lock().unwrap().take();
        }
        let failed = next_status(&mut rx).await;
        assert_eq!(failed.goal_id, "disconnected");
        assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
        assert!(failed.message.contains(error.unwrap_or("导航状态流已关闭")));
        if error.is_none() {
            let (tx, _) = tokio::sync::broadcast::channel(32);
            *capture.tx.lock().unwrap() = Some(tx);
        }
        bridge.submit("new".into(), goal(), false).await.unwrap();
        assert_eq!(
            next_status(&mut rx).await.status,
            proto::GoalStatus::WaitingInput as i32
        );
        bridge.set_input_ready(Ok(()));
        capture.inject_ready();
        assert_eq!(
            next_status(&mut rx).await.status,
            proto::GoalStatus::Planning as i32
        );
        assert_eq!(capture.goal_publish_attempts.load(Ordering::SeqCst), 1);
        bridge.shutdown().await;
    }
}

#[tokio::test]
async fn deferred_encoding_failure_reports_failed_instead_of_leaving_waiting_input() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    let mut invalid = goal();
    invalid.pose.as_mut().unwrap().orientation = None;
    bridge
        .submit("invalid".into(), invalid, false)
        .await
        .unwrap();
    next_status(&mut rx).await;
    bridge.set_input_ready(Ok(()));
    capture.inject_ready();
    let failed = next_status(&mut rx).await;
    assert_eq!(failed.goal_id, "invalid");
    assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
    assert!(failed.message.contains("导航目标缺少朝向"));
    assert_eq!(capture.goal_publish_attempts.load(Ordering::SeqCst), 0);
    bridge.shutdown().await;
}

#[tokio::test]
async fn exploration_goal_envelope_carries_complete_map_boundary() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    let boundary = vec![[-3.2, -0.4], [1.1, -0.7], [3.4, 2.8], [-2.5, 4.2]];
    let mut request = goal();
    request.allow_reverse = Some(false);
    request.position_tolerance_m = Some(0.15);
    request.yaw_tolerance_rad = Some(0.2);
    bridge
        .submit_with_boundary("explore-child".into(), request, true, boundary.clone())
        .await
        .unwrap();
    next_status(&mut rx).await;
    bridge.set_input_ready(Ok(()));
    capture.inject_ready();
    next_status(&mut rx).await;
    let frame = capture.goals().pop().unwrap();
    assert_eq!(frame.msg_type.fqn(), "nuwax_msgs.NavigationGoal");
    let received = schema::decode_navigation_goal_body(&frame.payload).unwrap();
    assert_eq!(
        received.boundary_points,
        boundary
            .into_iter()
            .map(|p| [p[0], p[1], 0.0])
            .collect::<Vec<_>>()
    );
    assert_eq!(received.motion_generation, 77);
    assert_eq!(received.goal.pose.position, [1.0, 2.0, 0.4]);
    assert_eq!(received.allow_reverse, Some(false));
    assert_eq!(received.position_tolerance_m, Some(0.15));
    assert_eq!(received.yaw_tolerance_rad, Some(0.2));
    bridge.shutdown().await;
}

#[tokio::test]
async fn cancel_with_no_active_goal_is_safe() {
    let capture = Arc::new(Capture::new());
    let (tx, _) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    bridge.cancel("").await.unwrap();
    bridge.cancel("absent").await.unwrap();
    assert!(capture.goals().is_empty());
    let frames = capture.published.lock().unwrap().clone();
    assert_eq!(frames.len(), 3);
    assert_eq!(
        schema::decode_cancel_navigation_body(&frames[2].payload)
            .unwrap()
            .session_id,
        "absent"
    );
    bridge.shutdown().await;
}

#[test]
fn typed_navigation_status_preserves_reason_and_route_distance_without_prefixes() {
    for (wire, expected) in [
        (
            schema::NAVIGATION_GOAL_STATUS_CANCELLED,
            proto::GoalStatus::Cancelled,
        ),
        (
            schema::NAVIGATION_GOAL_STATUS_STOPPED,
            proto::GoalStatus::Stopped,
        ),
        (
            schema::NAVIGATION_GOAL_STATUS_PLANNING,
            proto::GoalStatus::Planning,
        ),
    ] {
        let received = state_update(schema::NavigationStatus {
            session_id: "target".into(),
            goal_status: wire,
            reason: "现场原因".into(),
            message: "无状态标记的诊断文本".into(),
            remaining_distance_m: 4.75,
            no_feasible_path: true,
            ..Default::default()
        })
        .unwrap();
        assert_eq!(received.status, expected as i32);
        assert_eq!(received.message, "现场原因");
        assert_eq!(received.remaining_distance_m, 4.75);
        assert!(received.no_feasible_path);
    }
}

#[tokio::test]
async fn disconnected_sent_goal_reports_reason_and_cancel_rebuilds_channel() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    bridge.submit("执行中".into(), goal(), false).await.unwrap();
    next_status(&mut rx).await;
    bridge.set_input_ready(Ok(()));
    capture.inject_ready();
    assert_eq!(
        next_status(&mut rx).await.status,
        proto::GoalStatus::Planning as i32
    );
    capture
        .tx
        .lock()
        .unwrap()
        .as_ref()
        .unwrap()
        .send(Err("执行中链路断开".into()))
        .unwrap();
    let failed = next_status(&mut rx).await;
    assert_eq!(failed.goal_id, "执行中");
    assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
    assert!(failed.message.contains("执行中链路断开"));
    bridge.cancel("执行中").await.unwrap();
    bridge
        .submit("重连目标".into(), goal(), false)
        .await
        .unwrap();
    assert_eq!(
        next_status(&mut rx).await.status,
        proto::GoalStatus::WaitingInput as i32
    );
    capture.inject_ready();
    assert_eq!(
        next_status(&mut rx).await.status,
        proto::GoalStatus::Planning as i32
    );
    assert_eq!(capture.goals().len(), 2);
    bridge.shutdown().await;
}


#[tokio::test]
async fn restart_fails_dispatched_goal_and_does_not_resend_it() {
    let capture = Arc::new(Capture::new());
    let (tx, mut rx) = mpsc::unbounded_channel();
    let bridge = NavBridge::with_transport(LcmTransport::new(capture.clone()), tx)
        .await
        .unwrap();
    bridge.set_input_ready(Ok(()));
    capture.inject_ready_generation("", 42);
    bridge.submit("在途目标".into(), goal(), false).await.unwrap();
    assert_eq!(rx.recv().await.unwrap().status, proto::GoalStatus::WaitingInput as i32);
    assert_eq!(rx.recv().await.unwrap().status, proto::GoalStatus::Planning as i32);
    assert_eq!(capture.goals().len(), 1);

    // 同一进程的目标编号可变化；启动标识即使变小，也表示进程变了。
    capture.inject_ready_generation("另一个目标", 42);
    let unrelated = rx.recv().await.unwrap();
    assert_eq!(unrelated.goal_id, "另一个目标");
    capture.inject_ready_generation("", 9);
    let failed = tokio::time::timeout(Duration::from_secs(1), rx.recv())
        .await.unwrap().unwrap();
    assert_eq!(failed.goal_id, "在途目标");
    assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
    assert_eq!(failed.message, "导航栈已重启，目标中断");
    assert_eq!(capture.goals().len(), 1);
    bridge.shutdown().await;
}
