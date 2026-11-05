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
