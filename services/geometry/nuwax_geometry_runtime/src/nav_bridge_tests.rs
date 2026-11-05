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
        let now = unix_ns() as i64;
        let state = schema::NavigationState {
            session_id: id.into(),
            state: 1,
            sink_ready: 1,
            planner_map_frame: "map".into(),
            planner_pose_frame: "map".into(),
            pose_source_timestamp_ns: now,
            costmap_source_timestamp_ns: now,
            ..Default::default()
        };
        let frame = RawLcmFrameBuilder::new(
            LcmTopic::new("/nav/state").unwrap(),
            message_type("nuwax_msgs.NavigationState").unwrap(),
        )
        .payload(schema::encode_navigation_state_body(&state).unwrap())
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
    bridge.set_input_ready(true);
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
    bridge.set_input_ready(true);
    capture.inject_ready();
    let status = tokio::time::timeout(Duration::from_secs(1), rx.recv())
        .await
        .unwrap()
        .unwrap();
    assert_eq!(status.goal_id, "new");
    assert_eq!(capture.goals().len(), 1);
    bridge.set_input_ready(false);
    bridge
        .submit("cancelled".into(), goal(), false)
        .await
        .unwrap();
    rx.recv().await.unwrap();
    bridge.cancel("cancelled").await.unwrap();
    bridge.set_input_ready(true);
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
            bridge.set_input_ready(true);
            bridge.cancel("unrelated").await.unwrap();
            capture.inject_ready();
        } else {
            capture.inject_ready_named("heartbeat");
            assert_eq!(next_status(&mut rx).await.goal_id, "heartbeat");
            bridge.set_input_ready(true);
        }
        let failed = next_status(&mut rx).await;
        assert_eq!(failed.goal_id, "waiting");
        assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
        assert!(failed.message.contains("测试导航发布链路已断开"));
        bridge.set_input_ready(true);
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
    bridge.set_input_ready(true);
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
async fn state_stream_error_or_close_fails_pending_goal_and_ends_bridge() {
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
        assert_eq!(
            bridge
                .submit("new".into(), goal(), false)
                .await
                .unwrap_err()
                .code(),
            tonic::Code::Unavailable
        );
        assert_eq!(capture.goal_publish_attempts.load(Ordering::SeqCst), 0);
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
    bridge.set_input_ready(true);
    capture.inject_ready();
    let failed = next_status(&mut rx).await;
    assert_eq!(failed.goal_id, "invalid");
    assert_eq!(failed.status, proto::GoalStatus::Failed as i32);
    assert!(failed.message.contains("导航目标缺少朝向"));
    assert_eq!(capture.goal_publish_attempts.load(Ordering::SeqCst), 0);
    bridge.shutdown().await;
}
