use super::*;
use crate::mapper::{Mapper, MapperConfig};
use nuwax_protocol::RawLcmFrame;
use nuwax_transport::{LcmBackend, LcmFrameStream, TransportResult};
use std::sync::Mutex;

#[derive(Default)]
struct Capture(Mutex<Vec<RawLcmFrame>>);
#[async_trait::async_trait]
impl LcmBackend for Capture {
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        self.0.lock().unwrap().push(frame);
        Ok(())
    }
    async fn subscribe(&self, _: LcmTopic, _: Option<LcmMessageType>) -> LcmFrameStream {
        Box::pin(futures::stream::empty())
    }
    fn name(&self) -> &str {
        "人物跟随全局图行为测试"
    }
}

#[tokio::test]
async fn committed_cumulative_map_reaches_person_follow_decoder_once_per_revision() {
    let capture = Arc::new(Capture::default());
    let transport = LcmTransport::new(capture.clone());
    let mapper = Mapper::start(811, MapperConfig::default()).unwrap();
    let mut sent = None;
    for (stamp, offset) in [(1_000_000_000, 0.0), (2_000_000_000, 0.5)] {
        let mut frame = crate::mapper::tests::room_frame(811, stamp);
        frame.odom_pose.position.as_mut().unwrap().x = offset;
        mapper.feed(frame);
        let committed = mapper.snapshot().await.unwrap();
        let snapshot = mapper.display_cloud().await.unwrap();
        assert_eq!(snapshot.revision, committed.cloud.revision);
        assert!(!snapshot.display.is_empty());
        publish_follow_cloud(&transport, "map", 811, snapshot.clone(), &mut sent)
            .await
            .unwrap();
        publish_follow_cloud(&transport, "map", 811, snapshot, &mut sent)
            .await
            .unwrap();
        let messages = capture.0.lock().unwrap();
        let received = messages.last().unwrap();
        assert_eq!(received.topic.channel(), "/global_map");
        // 人物跟随解码器给消息体补上PointCloud2模式指纹后按同一路径解码。
        let mut packet = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
        packet.extend_from_slice(&received.payload);
        let decoded = schema::decode_point_cloud2(&packet).unwrap();
        assert_eq!(decoded.data.as_slice(), committed.cloud.display.as_ref());
        assert_eq!(decoded.header.seq as u64, committed.cloud.revision);
        assert_eq!(decoded.header.frame_id, "map");
        assert_eq!(decoded.width as usize * 12, decoded.data.len());
    }
    assert_eq!(capture.0.lock().unwrap().len(), 2);
    mapper.shutdown().await.unwrap();
}

#[test]
fn three_dimensional_occupancy_is_sent_at_most_once_per_second_keeping_the_latest() {
    let map = |stamp| proto::MapUpdate {
        meta: Some(proto::OccupancyGridMeta { timestamp_ns: stamp, ..Default::default() }),
        ..Default::default()
    };
    let start = tokio::time::Instant::now();
    let at = |ms| start + std::time::Duration::from_millis(ms);
    let mut rate = OccupancyRate { limited: true, sent_at: None, pending: None };
    assert_eq!(rate.offer(map(1), at(0)), Some(map(1)));
    assert_eq!(rate.offer(map(2), at(300)), None);
    assert_eq!(rate.offer(map(3), at(600)), None);
    assert_eq!(rate.due_at(), Some(at(1000)));
    assert_eq!(rate.take_due(at(1000)), Some(map(3)), "到点补发间隔内最新的一张");
    assert_eq!(rate.due_at(), None);
    assert_eq!(rate.offer(map(4), at(1500)), None);
    assert_eq!(rate.due_at(), Some(at(2000)));
    assert_eq!(rate.offer(map(5), at(2000)), Some(map(5)), "间隔已满的新栅格直接发并替换待发");
    assert_eq!(rate.due_at(), None);
    let mut planar = OccupancyRate { limited: false, sent_at: None, pending: None };
    for (stamp, ms) in [(1, 0), (2, 10), (3, 20)] {
        assert_eq!(planar.offer(map(stamp), at(ms)), Some(map(stamp)), "二维链每次都发");
    }
    assert_eq!(planar.due_at(), None);
}
