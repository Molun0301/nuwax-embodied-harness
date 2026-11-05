use super::*;
use crate::proto::{LioImuSample, PointCloudField};
use nuwax_protocol::{LcmMessageType, LcmTopic};

fn pair(topic: &str, kind: &str) -> (LcmTopic, LcmMessageType) {
    (
        LcmTopic::new(topic).unwrap(),
        LcmMessageType::new(kind).unwrap(),
    )
}
fn request(stamps: &[u64]) -> PublishPointCloudRequest {
    let mut points = Vec::new();
    for value in [1_f32, 2.0, 3.0] {
        points.extend_from_slice(&value.to_le_bytes());
    }
    points.extend_from_slice(&0_u32.to_le_bytes());
    PublishPointCloudRequest {
        points,
        point_step: 16,
        point_count: 1,
        frame_id: "laser".into(),
        timestamp_ns: stamps[0],
        fields: [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("offset_ns", 12, 6)]
            .into_iter()
            .map(|(name, offset, datatype)| PointCloudField {
                name: name.into(),
                offset,
                datatype,
                count: 1,
            })
            .collect(),
        imu_samples: stamps
            .iter()
            .map(|timestamp| LioImuSample {
                timestamp_ns: *timestamp,
                frame_id: "laser".into(),
                accel_mps2_z: 9.81,
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    }
}
#[tokio::test]
async fn physical_imu_source_gap_uses_native_existing_threshold() {
    let transport = LcmTransport::new_real("memq://native-source-gap-test").unwrap();
    let channels = NativePrivateChannels {
        source_odometry: pair("/test/gap/o", "nav_msgs.Odometry"),
        source_cloud: pair("/test/gap/c", "sensor_msgs.PointCloud2"),
        input_cloud: pair("/test/gap/ic", "sensor_msgs.PointCloud2"),
        input_imu: pair("/test/gap/ii", "sensor_msgs.Imu"),
        input_ack: None,
        input_frame: None,
    };
    let mut delivery = NativeInputDelivery::default();
    delivery
        .publish(&transport, &channels, &request(&[1000, 2000]))
        .await
        .unwrap();
    delivery
        .publish(&transport, &channels, &request(&[2000, 500_002_000]))
        .await
        .unwrap();
    let error = delivery
        .publish(
            &transport,
            &channels,
            &request(&[500_002_000, 1_000_002_001]),
        )
        .await
        .unwrap_err();
    assert_eq!(error.code(), tonic::Code::FailedPrecondition);
    assert!(error.message().contains("500000001"));
    assert_eq!(delivery.last_imu_timestamp_ns, Some(500_002_000));
}

#[derive(Default)]
struct Capture {
    frames: std::sync::Mutex<Vec<nuwax_protocol::RawLcmFrame>>,
    fail_at: Option<usize>,
}
#[async_trait::async_trait]
impl nuwax_transport::LcmBackend for Capture {
    async fn publish(
        &self,
        frame: nuwax_protocol::RawLcmFrame,
    ) -> nuwax_transport::TransportResult<()> {
        let mut frames = self.frames.lock().unwrap();
        if self.fail_at == Some(frames.len()) {
            return Err(nuwax_transport::TransportError::LcmBackend(
                "惯性发布失败".into(),
            ));
        }
        frames.push(frame);
        Ok(())
    }
    async fn subscribe(
        &self,
        _: LcmTopic,
        _: Option<LcmMessageType>,
    ) -> nuwax_transport::LcmFrameStream {
        Box::pin(futures::stream::empty())
    }
    fn name(&self) -> &str {
        "原生输入发送顺序"
    }
}
fn channels() -> NativePrivateChannels {
    NativePrivateChannels {
        source_odometry: pair("/order/o", "nav_msgs.Odometry"),
        source_cloud: pair("/order/c", "sensor_msgs.PointCloud2"),
        input_cloud: pair("/order/ic", "sensor_msgs.PointCloud2"),
        input_imu: pair("/order/ii", "sensor_msgs.Imu"),
        input_ack: None,
        input_frame: None,
    }
}
#[tokio::test]
async fn native_raw_pair_publishes_all_imu_before_cloud() {
    let capture = std::sync::Arc::new(Capture::default());
    let transport = LcmTransport::new(capture.clone());
    let mut delivery = NativeInputDelivery::default();
    delivery
        .publish(&transport, &channels(), &request(&[1000, 2000, 3000]))
        .await
        .unwrap();
    let frames = capture.frames.lock().unwrap();
    assert_eq!(
        frames
            .iter()
            .map(|frame| frame.topic.channel())
            .collect::<Vec<_>>(),
        ["/order/ii", "/order/ii", "/order/ii", "/order/ic"]
    );
    for (frame, stamp) in frames.iter().zip([1000, 2000, 3000]) {
        let mut bytes = nuwax_codec::lcm::schema::IMU_FINGERPRINT.to_vec();
        bytes.extend_from_slice(&frame.payload);
        let imu = nuwax_codec::lcm::schema::decode_imu(&bytes).unwrap();
        assert_eq!(imu.header.stamp_nsec, stamp);
    }
    assert_eq!(delivery.last_imu_timestamp_ns, Some(3000));
}
#[tokio::test]
async fn native_raw_pair_failure_keeps_cloud_unpublished() {
    let capture = std::sync::Arc::new(Capture {
        fail_at: Some(1),
        ..Default::default()
    });
    let transport = LcmTransport::new(capture.clone());
    let mut delivery = NativeInputDelivery::default();
    let error = delivery
        .publish(&transport, &channels(), &request(&[1000, 2000, 3000]))
        .await
        .unwrap_err();
    assert_eq!(error.code(), tonic::Code::Unavailable);
    let frames = capture.frames.lock().unwrap();
    assert_eq!(frames.len(), 1);
    assert!(frames
        .iter()
        .all(|frame| frame.topic.channel() == "/order/ii"));
    assert_eq!(delivery.last_imu_timestamp_ns, None);
}
#[tokio::test]
async fn imu_gap_latches_one_failure_until_delivery_is_recreated() {
    let capture = std::sync::Arc::new(Capture::default());
    let transport = LcmTransport::new(capture.clone());
    let mut delivery = NativeInputDelivery::default();
    delivery
        .publish(&transport, &channels(), &request(&[1]))
        .await
        .unwrap();
    let first = delivery
        .publish(&transport, &channels(), &request(&[500_000_002]))
        .await
        .unwrap_err();
    assert!(delivery.has_failed());
    let count = capture.frames.lock().unwrap().len();
    for stamp in [500_000_003, 1_000_000_000] {
        let repeated = delivery
            .publish(&transport, &channels(), &request(&[stamp]))
            .await
            .unwrap_err();
        assert_eq!(repeated.code(), first.code());
        assert_eq!(repeated.message(), first.message());
    }
    assert_eq!(capture.frames.lock().unwrap().len(), count);
    let mut recovered = NativeInputDelivery::default();
    recovered
        .publish(&transport, &channels(), &request(&[1_000_000_000]))
        .await
        .unwrap();
    assert!(!recovered.has_failed());
}
