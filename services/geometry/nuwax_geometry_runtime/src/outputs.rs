//! 对外旁路输出：坐标树、导航地图与按配置发布的人物跟随点云。
use crate::{proto, session::SessionHandle};
use nuwax_codec::lcm::schema;
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::LcmTransport;
use std::sync::Arc;
use tokio::sync::oneshot;
#[path = "navigation_map_transfer.rs"]
mod navigation_map_transfer;

pub struct Outputs {
    stop: oneshot::Sender<()>,
    task: tokio::task::JoinHandle<()>,
}
#[derive(Default)]
struct MapOutput {
    session_id: u64,
    frame_id: String,
    revision: i64,
    prior: Option<Arc<nuwax_map_chunks::Manifest>>,
    mask: Option<proto::OccupancyGrid>,
    mask_sent_at: Option<tokio::time::Instant>,
}

/// 三维链下 `/map/occupancy` 两次发布至少相隔的时间。
const OCCUPANCY_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);

/// 三维链下 `/map/occupancy` 每秒至多发一次，间隔内只留最新一张，到点补发；二维链每次都发。
struct OccupancyRate {
    limited: bool,
    sent_at: Option<tokio::time::Instant>,
    pending: Option<proto::MapUpdate>,
}

impl OccupancyRate {
    /// 返回现在要发的一张；间隔未到时替换待发的一张。
    fn offer(&mut self, map: proto::MapUpdate, now: tokio::time::Instant) -> Option<proto::MapUpdate> {
        if self.limited && self.sent_at.is_some_and(|sent| now < sent + OCCUPANCY_INTERVAL) {
            self.pending = Some(map);
            return None;
        }
        self.pending = None;
        self.sent_at = Some(now);
        Some(map)
    }
    fn due_at(&self) -> Option<tokio::time::Instant> {
        self.pending.as_ref().map(|_| self.sent_at.unwrap() + OCCUPANCY_INTERVAL)
    }
    fn take_due(&mut self, now: tokio::time::Instant) -> Option<proto::MapUpdate> {
        self.sent_at = Some(now);
        self.pending.take()
    }
}

impl MapOutput {
    /// 内容、栅格布局或会话变化立即携带；否则距上次携带满一秒的下一包携带。
    fn mask_due(&self, session_id: u64, mask: Option<&proto::OccupancyGrid>, now: tokio::time::Instant) -> bool {
        let layout = |mask: &proto::OccupancyGrid| mask.meta.as_ref().map(|m|
            (m.origin_x, m.origin_y, m.resolution, m.width, m.height));
        let same = match (self.mask.as_ref(), mask) {
            (Some(old), Some(next)) => layout(old) == layout(next) && old.data == next.data,
            (None, None) => true,
            _ => false,
        };
        self.session_id != session_id || !same
            || self.mask_sent_at.is_none_or(|sent| now >= sent + std::time::Duration::from_secs(1))
    }
    async fn clear(&mut self, transport: &LcmTransport) -> Result<(), String> {
        self.mask = None;
        self.mask_sent_at = None;
        if self.prior.take().is_none() {
            return Ok(());
        }
        self.revision += 1;
        send_navigation_map(transport, &schema::NavigationMap3D {
            session_id: self.session_id.to_string(),
            revision: self.revision,
            frame_id: self.frame_id.clone(),
            cleared: true,
            cloud: schema::PointCloud2 {
                header: header(0, &self.frame_id, 0),
                ..Default::default()
            },
            ..Default::default()
        }).await
    }
}

impl Outputs {
    pub fn start(session: SessionHandle) -> Result<Self, String> {
        let url = nuwax_config::transport()?.lcm_url.clone();
        let transport = LcmTransport::new_real(&url).map_err(|e| e.to_string())?;
        let follow = nuwax_config::person_follow()?.autostart;
        Ok(Self::start_with_transport(session, transport, follow))
    }
    fn start_with_transport(session: SessionHandle, transport: LcmTransport, follow: bool) -> Self {
        let (stop, mut stopped) = oneshot::channel();
        let recent_transport = transport.clone();
        let recent_session = session.clone();
        let recent_task = tokio::spawn(async move {
            let mut routes = recent_session.route_rx();
            loop {
                let route = routes.borrow_and_update().clone();
                if let Some(route) = route {
                    let mut recent = route.mapper.recent_cloud_rx();
                    let mut cleared = 0;
                    loop {
                        tokio::select! {
                            changed=routes.changed()=>{if changed.is_err(){return;}break;},
                            changed=recent.changed()=>{
                                if changed.is_err(){break;}
                                let bytes=recent.borrow_and_update().clone();
                                if routes.borrow().as_ref().is_some_and(|current|current.id==route.id) {
                                    if let Some(value) = bytes {
                                        if let Some((sequence, bytes)) = &value.clear {
                                            if *sequence > cleared {
                                                cleared = *sequence;
                                                if let Err(reason) = publish(&recent_transport, nuwax_codec::lcm::recent_cloud::DEFAULT_TOPIC,
                                                    nuwax_codec::lcm::recent_cloud::TYPE_NAME, bytes.clone()).await { recent_session.report_output_error(route.id, reason); }
                                            }
                                        }
                                        if let Some(bytes) = value.frame {
                                            if let Err(reason)=publish(&recent_transport, nuwax_codec::lcm::recent_cloud::DEFAULT_TOPIC,
                                                nuwax_codec::lcm::recent_cloud::TYPE_NAME, bytes).await { recent_session.report_output_error(route.id, reason); }
                                        }
                                    }
                                }
                            },
                        }
                    }
                } else if routes.changed().await.is_err() {
                    return;
                }
            }
        });
        let pose_session = session.clone();
        let pose_transport = transport.clone();
        let pose_task = tokio::spawn(async move {
            let mut poses = pose_session.pose_rx();
            while poses.changed().await.is_ok() {
                let id = pose_session.route_rx().borrow().as_ref().map_or(0, |route| route.id);
                let tree = pose_session.frame_tree();
                let message = schema::TfMessage { transforms: tree.transforms.iter().map(|edge| {
                    let p = edge.translation.clone().unwrap_or_default();
                    let q = edge.rotation.clone().unwrap_or_default();
                    schema::TransformStamped { header: header(edge.timestamp_ns, &edge.parent_frame, 0),
                        child_frame_id: edge.child_frame.clone(), transform: schema::Transform {
                            translation: [p.x, p.y, p.z], rotation_xyzw: [q.x, q.y, q.z, q.w] } }
                }).collect() };
                let result = match schema::encode_tf_message_body(&message) {
                    Ok(bytes) => publish(&pose_transport, "/tf", "tf2_msgs.TFMessage", bytes).await,
                    Err(error) => Err(error.to_string()),
                };
                if let Err(reason) = result { pose_session.report_output_error(id, reason); }
            }
        });
        let task = tokio::spawn(async move {
            let routes = session.route_rx();
            let mut maps = session.map_rx();
            let mut output = MapOutput::default();
            let mut follow_sent: Option<(u64, u64)> = None;
            let mut occupancy = OccupancyRate {
                limited: session.state().info.is_some_and(|info| info.pipeline == "lio_3d"),
                sent_at: None,
                pending: None,
            };
            loop {
                let due = occupancy.due_at();
                let (id, result) = tokio::select! {
                    _=&mut stopped=>break,
                    _=tokio::time::sleep_until(due.unwrap_or_else(tokio::time::Instant::now)), if due.is_some()=>{
                        let id = routes.borrow().as_ref().map_or(0, |route| route.id);
                        let map = occupancy.take_due(tokio::time::Instant::now()).unwrap();
                        (id, publish_occupancy(&transport, &map).await)
                    },
                    changed=maps.changed()=>{
                        if changed.is_err(){break;}
                        let id = routes.borrow().as_ref().map_or(0, |route| route.id);
                        let map=maps.borrow_and_update().clone();
                        let result = if let Some(map)=map {
                            match publish_map(&transport, &session, &map, &mut output, &mut occupancy).await {
                                Ok(Some((sealed, changed))) if follow && changed => {
                                    publish_follow(&transport, &session, &sealed, &mut follow_sent).await
                                }
                                Ok(_) => Ok(()),
                                Err(reason) => Err(reason),
                            }
                        } else {
                            follow_sent = None;
                            // 地图撤下后不再补发撤下前的栅格。
                            occupancy.pending = None;
                            output.clear(&transport).await
                        };
                        (id, result)
                    },
                };
                if let Err(reason) = result {
                    session.report_output_error(id, reason);
                }
            }
            pose_task.abort();
            let _ = pose_task.await;
            recent_task.abort();
            let _ = recent_task.await;
        });
        Self { stop, task }
    }
    pub async fn shutdown(self) {
        let _ = self.stop.send(());
        let _ = self.task.await;
    }
}
async fn publish_map(
    transport: &LcmTransport,
    session: &SessionHandle,
    _map: &proto::MapUpdate,
    output: &mut MapOutput,
    occupancy: &mut OccupancyRate,
) -> Result<Option<(Arc<nuwax_map_chunks::SealedMap>, bool)>, String> {
    let mapper = match session.mapper() {
        Ok(mapper) => mapper,
        Err(_) => return Ok(None),
    };
    let id = mapper.session_id();
    let publication = mapper.publication().await;
    if session
        .mapper()
        .ok()
        .is_none_or(|current| current.session_id() != id)
    {
        return Ok(None);
    }
    let publication = publication?;
    let Some(map) = publication.map else {
        return Ok(None);
    };
    let frame_id = map.meta.as_ref().ok_or("栅格缺少元数据")?.frame_id.clone();
    if let Some(map) = occupancy.offer(map, tokio::time::Instant::now()) {
        publish_occupancy(transport, &map).await?;
    }
    let Some(sealed) = publication.fine else {
        return Ok(None);
    };
    let root = sealed.manifest();
    let stamp = root.source_timestamp_ns;
    let revision = if output.session_id == id {
        output.revision + 1
    } else {
        1
    };
    let send_mask = output.mask_due(id, publication.forbidden.as_ref(), tokio::time::Instant::now());
    let mut virtual_obstacles = publication
        .forbidden.as_ref().filter(|_| send_mask)
        .map(|mask| {
            grid(
                &proto::MapUpdate {
                    map_name: mask.map_name.clone(),
                    meta: mask.meta.clone(),
                    data: mask.data.clone(),
                    raw_data: Vec::new(),
                },
                true,
            )
        })
        .transpose()?;
    if let Some(mask) = &mut virtual_obstacles {
        mask.header = header(stamp, &frame_id, root.revision);
    }
    let message = schema::NavigationMap3D {
        session_id: root.domain.session.clone(),
        revision,
        frame_id: frame_id.clone(),
        source_timestamp_ns: stamp as i64,
        source_voxel_size: root.voxel_size_m,
        virtual_obstacles,
        fine: Some(navigation_map_transfer::prepare(&sealed)?),
        cloud: cloud(Vec::new(), &frame_id, stamp, 0),
        ..Default::default()
    };
    send_navigation_map(transport, &message).await?;
    if send_mask && publication.forbidden.is_some() {
        output.mask = publication.forbidden;
        output.mask_sent_at = Some(tokio::time::Instant::now());
    }
    let changed = output
        .prior
        .as_ref()
        .is_none_or(|old| old.identity() != root.identity());
    output.session_id = id;
    output.frame_id = message.frame_id;
    output.revision = revision;
    output.prior = Some(root.clone());
    Ok(Some((sealed, changed)))
}

async fn publish_occupancy(transport: &LcmTransport, map: &proto::MapUpdate) -> Result<(), String> {
    publish(
        transport,
        "/map/occupancy",
        "nav_msgs.OccupancyGrid",
        schema::encode_occupancy_grid_body(&grid(map, true)?).map_err(|e| e.to_string())?,
    )
    .await
}

async fn send_navigation_map(
    transport: &LcmTransport,
    message: &schema::NavigationMap3D,
) -> Result<(), String> {
    let bytes = schema::encode_navigation_map_3d_body(message).map_err(|e| e.to_string())?;
    publish(transport, "/nav/map3d", "nuwax_msgs.NavigationMap3D", bytes).await
}
fn header(stamp: u64, frame: &str, revision: u64) -> schema::Header {
    schema::Header {
        seq: revision as i32,
        stamp_sec: (stamp / 1_000_000_000) as i32,
        stamp_nsec: (stamp % 1_000_000_000) as i32,
        frame_id: frame.into(),
    }
}
fn grid(map: &proto::MapUpdate, effective: bool) -> Result<schema::OccupancyGrid, String> {
    let meta = map.meta.as_ref().ok_or("栅格缺少元数据")?;
    Ok(schema::OccupancyGrid {
        header: header(meta.timestamp_ns, &meta.frame_id, 0),
        info: schema::MapMetaData {
            resolution: meta.resolution,
            width: meta.width as i32,
            height: meta.height as i32,
            origin: schema::Pose {
                position: [meta.origin_x, meta.origin_y, 0.],
                orientation_xyzw: [0., 0., 0., 1.],
            },
            ..Default::default()
        },
        data: if effective { &map.data } else { &map.raw_data }
            .iter()
            .map(|v| *v as i8)
            .collect(),
    })
}
fn cloud(bytes: Vec<u8>, frame: &str, stamp: u64, revision: u64) -> schema::PointCloud2 {
    schema::PointCloud2 {
        header: header(stamp, frame, revision),
        height: 1,
        width: (bytes.len() / 12) as i32,
        point_step: 12,
        row_step: bytes.len() as i32,
        fields: ["x", "y", "z"]
            .into_iter()
            .enumerate()
            .map(|(index, name)| schema::PointField {
                name: name.into(),
                offset: index as i32 * 4,
                datatype: 7,
                count: 1,
            })
            .collect(),
        data: bytes,
        is_dense: true,
        ..Default::default()
    }
}
async fn publish_follow(
    transport: &LcmTransport,
    session: &SessionHandle,
    sealed: &nuwax_map_chunks::SealedMap,
    sent: &mut Option<(u64, u64)>,
) -> Result<(), String> {
    let mapper = match session.mapper() {
        Ok(mapper) => mapper,
        Err(_) => return Ok(()),
    };
    let id = mapper.session_id();
    if sealed.manifest().domain.session != id.to_string() {
        return Ok(());
    }
    let snapshot = mapper.display_cloud().await;
    if session
        .mapper()
        .ok()
        .is_none_or(|current| current.session_id() != id)
    {
        return Ok(());
    }
    let snapshot = snapshot?;
    let frame = session.state().info.unwrap().map_frame_id;
    publish_follow_cloud(transport, &frame, id, snapshot, sent).await
}

async fn publish_follow_cloud(
    transport: &LcmTransport,
    frame: &str,
    session_id: u64,
    snapshot: crate::pointcloud::CloudSnapshot,
    sent: &mut Option<(u64, u64)>,
) -> Result<(), String> {
    let revision = (session_id, snapshot.revision);
    if *sent == Some(revision) {
        return Ok(());
    }
    let message = cloud(
        snapshot.display.to_vec(),
        frame,
        snapshot.source_timestamp_ns,
        snapshot.revision,
    );
    publish(
        transport,
        "/global_map",
        "sensor_msgs.PointCloud2",
        schema::encode_point_cloud2_body(&message).map_err(|error| error.to_string())?,
    )
    .await?;
    *sent = Some(revision);
    Ok(())
}

async fn publish(
    transport: &LcmTransport,
    topic: &str,
    kind: &str,
    bytes: Vec<u8>,
) -> Result<(), String> {
    let topic = topic.split('#').next().unwrap();
    let frame = RawLcmFrameBuilder::new(
        LcmTopic::new(topic).map_err(|e| e.to_string())?,
        LcmMessageType::new(kind).map_err(|e| e.to_string())?,
    )
    .payload(bytes)
    .build();
    transport.publish(frame).await.map_err(|e| e.to_string())
}

#[cfg(test)]
#[path = "outputs_tests.rs"]
mod tests;

#[cfg(test)]
mod pose_output_tests {
    use super::*;
    use nuwax_transport::{LcmBackend, LcmFrameStream, TransportResult};

    struct BlockMap {
        entered: tokio::sync::Notify,
        release: tokio::sync::Notify,
        tf: tokio::sync::mpsc::UnboundedSender<Vec<u8>>,
    }
    #[async_trait::async_trait]
    impl LcmBackend for BlockMap {
        async fn publish(&self, frame: nuwax_protocol::RawLcmFrame) -> TransportResult<()> {
            if frame.topic.channel() == "/map/occupancy" {
                self.entered.notify_one();
                self.release.notified().await;
            }
            if frame.topic.channel() == "/tf" { self.tf.send(frame.payload).unwrap(); }
            Ok(())
        }
        async fn subscribe(&self, _: LcmTopic, _: Option<LcmMessageType>) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }
        fn name(&self) -> &str { "地图阻塞时位姿仍发布" }
    }
    #[tokio::test]
    async fn blocked_map_publication_does_not_delay_current_pose_tf() {
        let mapper = crate::mapper::Mapper::start(812, Default::default()).unwrap();
        mapper.feed(crate::mapper::tests::room_frame(812, 1_000_000_000));
        mapper.snapshot().await.unwrap();
        let map = mapper.map_rx().borrow().clone();
        let (session, poses, maps) = crate::session::test_output_session(mapper.clone());
        let (sender, mut received) = tokio::sync::mpsc::unbounded_channel();
        let backend = Arc::new(BlockMap { entered: Default::default(), release: Default::default(), tf: sender });
        let output = Outputs::start_with_transport(session, LcmTransport::new(backend.clone()), false);
        maps.send_replace(map);
        tokio::time::timeout(std::time::Duration::from_secs(2), backend.entered.notified()).await.unwrap();
        poses.send_replace(Some(crate::mapper::pose_from_isometry(
            nalgebra::Isometry3::identity(), "map", 2_000_000_000)));
        let bytes = tokio::time::timeout(std::time::Duration::from_secs(1), received.recv()).await
            .expect("地图尚未解除阻塞时必须收到当前位姿TF").unwrap();
        let mut packet = schema::TF_MESSAGE_FINGERPRINT.to_vec(); packet.extend(bytes);
        let tf = schema::decode_tf_message(&packet).unwrap();
        assert_eq!(tf.transforms.len(), 2);
        assert!(tf.transforms.iter().all(|edge| edge.header.stamp_sec == 2));
        backend.release.notify_one();
        output.shutdown().await;
        mapper.shutdown().await.unwrap();
    }
}


#[cfg(test)]
mod mask_delivery_tests {
    use super::*;
    #[test]
    fn changed_layout_content_session_and_one_second_refresh_carry_mask() {
        let now = tokio::time::Instant::now();
        let mask = proto::OccupancyGrid { meta: Some(proto::OccupancyGridMeta {
            resolution: 0.05, width: 2, height: 2, ..Default::default()
        }), data: vec![0;4], ..Default::default() };
        let mut output = MapOutput::default();
        assert!(output.mask_due(1,Some(&mask),now));
        output.session_id=1;output.mask=Some(mask.clone());output.mask_sent_at=Some(now);
        let mut clock_only=mask.clone();clock_only.meta.as_mut().unwrap().timestamp_ns=20;
        assert!(!output.mask_due(1,Some(&clock_only),now+std::time::Duration::from_millis(999)));
        assert!(output.mask_due(1,Some(&mask),now+std::time::Duration::from_secs(1)));
        assert!(output.mask_due(2,Some(&mask),now));
        for changed in 0..6 {
            let mut next=mask.clone(); let meta=next.meta.as_mut().unwrap();
            match changed { 0=>next.data[0]=100,1=>meta.origin_x=1.,2=>meta.origin_y=1.,
                3=>meta.width=1,4=>meta.height=1,_=>meta.resolution=0.1 }
            assert!(output.mask_due(1,Some(&next),now),"变化项{changed}必须携带");
        }
    }
    #[test]
    fn lost_change_packet_is_carried_again_within_one_second() {
        let now=tokio::time::Instant::now();
        let mask=proto::OccupancyGrid {data:vec![100],..Default::default()};
        let mut output=MapOutput::default();output.session_id=1;
        // 变化包在0毫秒发布但丢失；发送端不感知丢包，下一秒主动补带。
        assert!(output.mask_due(1,Some(&mask),now));
        output.mask=Some(mask.clone());output.mask_sent_at=Some(now);
        for ms in (100..1000).step_by(100) {
            assert!(!output.mask_due(1,Some(&mask),now+std::time::Duration::from_millis(ms)));
        }
        assert!(output.mask_due(1,Some(&mask),now+std::time::Duration::from_secs(1)));
    }
}
