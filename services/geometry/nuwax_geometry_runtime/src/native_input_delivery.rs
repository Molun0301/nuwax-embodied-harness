//! 一次只交付一帧，回执与当前私有代际、序列和源窗口一致才允许继续。

use crate::proto::PublishPointCloudRequest;
use nuwax_codec::lcm::{input_ack, input_frame};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::LcmTransport;
use std::time::Duration;
use tokio::sync::Mutex;
use tonic::Status;

type Binding = (LcmTopic, LcmMessageType);
const DELIVERY_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Default)]
struct DeliveryState {
    sequence: i32,
    last_scan_begin: Option<u64>,
    failure: Option<String>,
}

pub(crate) struct NativeInputDelivery {
    ack: Binding,
    frame: Binding,
    state: Mutex<DeliveryState>,
    shutdown: Option<tokio::sync::watch::Receiver<bool>>,
}

impl NativeInputDelivery {
    pub(crate) fn new(ack: Binding, frame: Binding) -> Self {
        Self {
            ack,
            frame,
            state: Mutex::new(DeliveryState::default()),
            shutdown: None,
        }
    }

    pub(crate) fn with_shutdown(mut self, shutdown: tokio::sync::watch::Receiver<bool>) -> Self {
        self.shutdown = Some(shutdown);
        self
    }

    pub(crate) async fn publish(
        &self,
        transport: &LcmTransport,
        request: &PublishPointCloudRequest,
        imu_payloads: Vec<Vec<u8>>,
    ) -> Result<i32, Status> {
        let mut state = tokio::select! {
            biased;
            _ = wait_for_shutdown(self.shutdown.clone()) => return Err(Status::cancelled("几何拥有方正在关闭")),
            state = self.state.lock() => state,
        };
        if let Some(reason) = &state.failure {
            return Err(Status::failed_precondition(format!(
                "原生输入已锁定失败，需重启几何运行时：{reason}"
            )));
        }
        if state
            .last_scan_begin
            .is_some_and(|last| request.timestamp_ns <= last)
        {
            return Err(Status::invalid_argument("扫描源时间必须严格递增"));
        }
        let sequence = state
            .sequence
            .checked_add(1)
            .ok_or_else(|| Status::out_of_range("原生扫描序列耗尽，需重启拥有方"))?;
        let end = scan_end(request)?;
        let payload =
            crate::server::encode_pointcloud_request_payload_with_sequence(request, sequence)
                .map_err(|error| Status::invalid_argument(error.to_string()))?;
        let packet = RawLcmFrameBuilder::new(self.frame.0.clone(), self.frame.1.clone())
            .payload(
                input_frame::encode_body(&payload, &imu_payloads)
                    .map_err(|error| Status::invalid_argument(error.to_string()))?,
            )
            .build();
        // 先锁定未决状态。取消、任务丢弃或未知传输结果不能使后续请求假定本帧完整。
        state.failure = Some(format!(
            "扫描{sequence}源时间{}尚未确认",
            request.timestamp_ns
        ));
        let mut subscription = match tokio::time::timeout(
            DELIVERY_TIMEOUT,
            transport.subscribe_owned(self.ack.0.clone(), Some(self.ack.1.clone())),
        )
        .await
        {
            Ok(Ok(subscription)) => subscription,
            other => return Err(Status::unavailable(format!("原生回执订阅失败：{other:?}"))),
        };
        let outcome = tokio::select! {
            biased;
            _ = wait_for_shutdown(self.shutdown.clone()) => Err(Status::cancelled("关闭中止了原生处理确认")),
            result = async { tokio::time::timeout(DELIVERY_TIMEOUT, async {
            transport
                .publish(packet.clone())
                .await
                .map_err(|error| Status::unavailable(format!("完整测量发布失败：{error}")))?;
            let mut attempts = 1;
            loop {
                let frame =
                    match tokio::time::timeout(Duration::from_millis(200), subscription.recv())
                        .await
                    {
                        Ok(Ok(frame)) => frame,
                        Ok(Err(error)) => {
                            return Err(Status::unavailable(format!("原生回执接收失败：{error}")))
                        }
                        Err(_) if attempts < 8 => {
                            // 重发原样整组；原生缓存完整载荷并仅重发已完成回执，绝不重复计算。
                            attempts += 1;
                            transport.publish(packet.clone()).await.map_err(|error| {
                                Status::unavailable(format!("完整测量重发失败：{error}"))
                            })?;
                            continue;
                        }
                        Err(_) => return Err(Status::deadline_exceeded("完整测量重试后仍未确认")),
                    };
                let ack = input_ack::decode_body(&frame.payload)
                    .map_err(|error| Status::data_loss(format!("原生回执非法：{error}")))?;
                if ack.sequence < sequence {
                    continue;
                }
                if ack.sequence != sequence || ack.scan_begin_timestamp_ns != request.timestamp_ns {
                    return Err(Status::data_loss("原生回执序列或扫描身份不匹配"));
                }
                if !ack.success {
                    return Err(Status::failed_precondition(format!(
                        "原生扫描拒绝：{}",
                        ack.message
                    )));
                }
                if ack.scan_end_timestamp_ns != end {
                    return Err(Status::data_loss("原生处理的扫描末端不匹配"));
                }
                return Ok(());
            }
        })
        .await
        .unwrap_or_else(|_| {
            Err(Status::deadline_exceeded(
                "原生扫描处理确认超过2秒，禁止继续跳帧",
            ))
        }) } => result,
        };
        let cleanup =
            tokio::time::timeout(Duration::from_millis(250), subscription.unsubscribe()).await;
        let outcome = outcome.and_then(|()| match cleanup {
            Ok(Ok(())) => Ok(()),
            _ => Err(Status::unavailable("原生回执订阅未确认回收")),
        });
        match outcome {
            Ok(()) => {
                state.sequence = sequence;
                state.last_scan_begin = Some(request.timestamp_ns);
                state.failure = None;
                Ok(sequence)
            }
            Err(error) => {
                state.failure = Some(format!(
                    "扫描{sequence}源时间{}：{error}",
                    request.timestamp_ns
                ));
                Err(error)
            }
        }
    }
}

async fn wait_for_shutdown(receiver: Option<tokio::sync::watch::Receiver<bool>>) {
    if let Some(mut receiver) = receiver {
        if !*receiver.borrow() {
            let _ = receiver.wait_for(|closed| *closed).await;
        }
    } else {
        std::future::pending::<()>().await;
    }
}

fn scan_end(request: &PublishPointCloudRequest) -> Result<u64, Status> {
    let offset = request
        .fields
        .iter()
        .find(|field| field.name == "offset_ns")
        .ok_or_else(|| Status::invalid_argument("扫描缺少逐点时间"))?
        .offset as usize;
    let step = request.point_step as usize;
    if step == 0
        || offset.checked_add(4).is_none_or(|end| end > step)
        || request.points.len() % step != 0
    {
        return Err(Status::invalid_argument("扫描逐点时间布局非法"));
    }
    let maximum = request
        .points
        .chunks_exact(step)
        .map(|point| u32::from_le_bytes(point[offset..offset + 4].try_into().unwrap()) as u64)
        .max()
        .unwrap_or(0);
    request
        .timestamp_ns
        .checked_add(maximum)
        .ok_or_else(|| Status::out_of_range("扫描末端时间溢出"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_codec::lcm::input_ack::LioInputAck;
    use nuwax_protocol::RawLcmFrameBuilder;
    use std::sync::Arc;
    use tokio::sync::oneshot;

    fn binding(topic: &str, name: &str) -> Binding {
        (
            LcmTopic::new(topic).unwrap(),
            LcmMessageType::new(name).unwrap(),
        )
    }

    fn request(stamp: u64) -> PublishPointCloudRequest {
        let mut points = Vec::new();
        for value in [1_f32, 2., 3.] {
            points.extend_from_slice(&value.to_le_bytes());
        }
        points.extend_from_slice(&5_u32.to_le_bytes());
        PublishPointCloudRequest {
            points,
            point_step: 16,
            frame_id: "sensor".into(),
            timestamp_ns: stamp,
            point_count: 1,
            is_dense: true,
            fields: [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("offset_ns", 12, 6)]
                .into_iter()
                .map(|(name, offset, datatype)| crate::proto::PointCloudField {
                    name: name.into(),
                    offset,
                    datatype,
                    count: 1,
                })
                .collect(),
            ..Default::default()
        }
    }

    async fn acknowledge(transport: &LcmTransport, channel: &Binding, ack: LioInputAck) {
        transport
            .publish(
                RawLcmFrameBuilder::new(channel.0.clone(), channel.1.clone())
                    .payload(input_ack::encode_body(&ack).unwrap())
                    .build(),
            )
            .await
            .unwrap();
    }

    #[tokio::test]
    async fn native_delivery_waits_for_processing_and_sequences_concurrent_requests() {
        let transport = LcmTransport::new_real("memq://").unwrap();
        let cloud = binding("/delivery/cloud", "nuwax_msgs.LioInputFrame");
        let _imu = binding("/delivery/imu", "sensor_msgs.Imu");
        let ack = binding("/delivery/ack", "nuwax_msgs.LioInputAck");
        let mut input = transport
            .subscribe_owned(cloud.0.clone(), Some(cloud.1.clone()))
            .await
            .unwrap();
        let delivery = Arc::new(NativeInputDelivery::new(ack.clone(), cloud.clone()));
        let (seen_tx, seen_rx) = oneshot::channel();
        let (release_tx, release_rx) = oneshot::channel();
        let native_transport = transport.clone();
        let responder = tokio::spawn(async move {
            let first = input.recv().await.unwrap();
            let first = input_frame::decode_body(&first.payload).unwrap().cloud;
            assert_eq!(first.header.seq, 1);
            seen_tx.send(()).unwrap();
            release_rx.await.unwrap();
            acknowledge(
                &native_transport,
                &ack,
                LioInputAck {
                    sequence: 1,
                    scan_begin_timestamp_ns: 100,
                    scan_end_timestamp_ns: 105,
                    success: true,
                    message: "initialized_without_pose".into(),
                },
            )
            .await;
            let second = input.recv().await.unwrap();
            let second = input_frame::decode_body(&second.payload).unwrap().cloud;
            assert_eq!(second.header.seq, 2);
            // 已处理的旧回执不能确认后续扫描。
            acknowledge(
                &native_transport,
                &ack,
                LioInputAck {
                    sequence: 1,
                    scan_begin_timestamp_ns: 100,
                    scan_end_timestamp_ns: 105,
                    success: true,
                    message: "old".into(),
                },
            )
            .await;
            acknowledge(
                &native_transport,
                &ack,
                LioInputAck {
                    sequence: 2,
                    scan_begin_timestamp_ns: 200,
                    scan_end_timestamp_ns: 205,
                    success: true,
                    message: "processed".into(),
                },
            )
            .await;
            input.unsubscribe().await.unwrap();
        });
        let d = delivery.clone();
        let t = transport.clone();
        let first = tokio::spawn(async move { d.publish(&t, &request(100), vec![]).await });
        seen_rx.await.unwrap();
        assert!(!first.is_finished(), "内核发送返回不能代替原生处理确认");
        let d = delivery.clone();
        let t = transport.clone();
        let second = tokio::spawn(async move { d.publish(&t, &request(200), vec![]).await });
        release_tx.send(()).unwrap();
        assert_eq!(first.await.unwrap().unwrap(), 1);
        assert_eq!(second.await.unwrap().unwrap(), 2);
        responder.await.unwrap();
    }

    #[tokio::test]
    async fn native_delivery_cancelled_after_publish_cannot_resume_as_complete() {
        let transport = LcmTransport::new_real("memq://").unwrap();
        let cloud = binding("/cancel/cloud", "nuwax_msgs.LioInputFrame");
        let _imu = binding("/cancel/imu", "sensor_msgs.Imu");
        let ack = binding("/cancel/ack", "nuwax_msgs.LioInputAck");
        let mut input = transport
            .subscribe_owned(cloud.0.clone(), Some(cloud.1.clone()))
            .await
            .unwrap();
        let delivery = Arc::new(NativeInputDelivery::new(ack, cloud.clone()));
        let d = delivery.clone();
        let t = transport.clone();
        let work = tokio::spawn(async move { d.publish(&t, &request(100), vec![]).await });
        input.recv().await.unwrap();
        work.abort();
        assert!(work.await.unwrap_err().is_cancelled());
        let error = delivery
            .publish(&transport, &request(200), vec![])
            .await
            .unwrap_err();
        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("尚未确认"));
        input.unsubscribe().await.unwrap();
    }

    #[tokio::test]
    async fn native_delivery_rejects_wrong_identity_and_latches_failure() {
        let transport = LcmTransport::new_real("memq://").unwrap();
        let cloud = binding("/identity/cloud", "nuwax_msgs.LioInputFrame");
        let _imu = binding("/identity/imu", "sensor_msgs.Imu");
        let ack = binding("/identity/ack", "nuwax_msgs.LioInputAck");
        let mut input = transport
            .subscribe_owned(cloud.0.clone(), Some(cloud.1.clone()))
            .await
            .unwrap();
        let delivery = NativeInputDelivery::new(ack.clone(), cloud.clone());
        let t = transport.clone();
        let responder = tokio::spawn(async move {
            input.recv().await.unwrap();
            acknowledge(
                &t,
                &ack,
                LioInputAck {
                    sequence: 1,
                    scan_begin_timestamp_ns: 99,
                    scan_end_timestamp_ns: 105,
                    success: true,
                    message: "wrong source".into(),
                },
            )
            .await;
            input.unsubscribe().await.unwrap();
        });
        let error = delivery
            .publish(&transport, &request(100), vec![])
            .await
            .unwrap_err();
        assert_eq!(error.code(), tonic::Code::DataLoss);
        assert_eq!(
            delivery
                .publish(&transport, &request(200), vec![])
                .await
                .unwrap_err()
                .code(),
            tonic::Code::FailedPrecondition
        );
        responder.await.unwrap();
    }
    #[tokio::test]
    async fn native_delivery_timeout_and_rejection_never_allow_later_input() {
        for reject in [false, true] {
            let transport = LcmTransport::new_real("memq://").unwrap();
            let cloud = binding("/failure/cloud", "nuwax_msgs.LioInputFrame");
            let _imu = binding("/failure/imu", "sensor_msgs.Imu");
            let ack = binding("/failure/ack", "nuwax_msgs.LioInputAck");
            let mut input = transport
                .subscribe_owned(cloud.0.clone(), Some(cloud.1.clone()))
                .await
                .unwrap();
            let delivery = NativeInputDelivery::new(ack.clone(), cloud.clone());
            let t = transport.clone();
            let responder = tokio::spawn(async move {
                input.recv().await.unwrap();
                if reject {
                    acknowledge(
                        &t,
                        &ack,
                        LioInputAck {
                            sequence: 1,
                            scan_begin_timestamp_ns: 100,
                            scan_end_timestamp_ns: 100,
                            success: false,
                            message: "惯性覆盖不足".into(),
                        },
                    )
                    .await;
                }
                input.unsubscribe().await.unwrap();
            });
            let error = delivery
                .publish(&transport, &request(100), vec![])
                .await
                .unwrap_err();
            assert_eq!(
                error.code(),
                if reject {
                    tonic::Code::FailedPrecondition
                } else {
                    tonic::Code::DeadlineExceeded
                }
            );
            assert_eq!(
                delivery
                    .publish(&transport, &request(200), vec![])
                    .await
                    .unwrap_err()
                    .code(),
                tonic::Code::FailedPrecondition
            );
            responder.await.unwrap();
        }
    }
    #[tokio::test]
    async fn native_delivery_retries_identical_frame_when_first_receipt_is_lost() {
        let transport = LcmTransport::new_real("memq://").unwrap();
        let frame = binding("/retry/frame", "nuwax_msgs.LioInputFrame");
        let ack = binding("/retry/ack", "nuwax_msgs.LioInputAck");
        let mut input = transport
            .subscribe_owned(frame.0.clone(), Some(frame.1.clone()))
            .await
            .unwrap();
        let delivery = NativeInputDelivery::new(ack.clone(), frame);
        let t = transport.clone();
        let native = tokio::spawn(async move {
            let first = input.recv().await.unwrap();
            let repeated = input.recv().await.unwrap();
            assert_eq!(first.payload, repeated.payload, "重试不能改动原始输入");
            let request = input_frame::decode_body(&repeated.payload).unwrap();
            assert_eq!(request.cloud.header.seq, 1);
            acknowledge(
                &t,
                &ack,
                LioInputAck {
                    sequence: 1,
                    scan_begin_timestamp_ns: 100,
                    scan_end_timestamp_ns: 105,
                    success: true,
                    message: "processed once".into(),
                },
            )
            .await;
            input.unsubscribe().await.unwrap();
        });
        assert_eq!(
            delivery
                .publish(&transport, &request(100), vec![])
                .await
                .unwrap(),
            1
        );
        native.await.unwrap();
    }

    #[tokio::test]
    async fn native_delivery_shutdown_wakes_receipt_wait_and_rejects_new_input() {
        let transport = LcmTransport::new_real("memq://").unwrap();
        let frame = binding("/shutdown/frame", "nuwax_msgs.LioInputFrame");
        let ack = binding("/shutdown/ack", "nuwax_msgs.LioInputAck");
        let mut input = transport
            .subscribe_owned(frame.0.clone(), Some(frame.1.clone()))
            .await
            .unwrap();
        let (stop, receiver) = tokio::sync::watch::channel(false);
        let delivery = Arc::new(NativeInputDelivery::new(ack, frame).with_shutdown(receiver));
        let d = delivery.clone();
        let t = transport.clone();
        let work = tokio::spawn(async move { d.publish(&t, &request(100), vec![]).await });
        input.recv().await.unwrap();
        let started = std::time::Instant::now();
        stop.send(true).unwrap();
        assert_eq!(
            work.await.unwrap().unwrap_err().code(),
            tonic::Code::Cancelled
        );
        assert!(started.elapsed() < Duration::from_millis(500));
        assert_eq!(
            delivery
                .publish(&transport, &request(200), vec![])
                .await
                .unwrap_err()
                .code(),
            tonic::Code::Cancelled
        );
        input.unsubscribe().await.unwrap();
    }
}
