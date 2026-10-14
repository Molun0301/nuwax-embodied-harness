//! 辅助点云按采集时间汇入原激光建图拥有方，不另起地图或定位前端。
use super::*;
use nalgebra::{Isometry3, Point3};
use prost::Message;
use std::collections::{BTreeMap, VecDeque};

// 只借用已有地图代次，不引入新的生命周期。
type MapIdentity = (String, u64, u64, u64);

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Observation {
    timestamp_ns: u64,
    body_frame: String,
    points_body: crate::correction_points::CorrectionPoints,
    origin_body: [f64; 3],
    digest: [u8; 32],
    sensor_id: String,
    source_sequence: u64,
    clock_id: String,
    calibration_ref: String,
}

#[derive(Default)]
pub(crate) struct Inbox {
    identity: Option<MapIdentity>,
    previous_lidar_ns: u64,
    mode_epoch: u64,
    latest: BTreeMap<String, VecDeque<Observation>>,
}

impl Inbox {
    fn select_mode(&mut self, epoch: u64) {
        if self.mode_epoch != epoch {
            self.latest.clear();
            self.previous_lidar_ns = 0;
            self.mode_epoch = epoch;
        }
    }

    fn select_map(&mut self, identity: &MapIdentity) {
        if self.identity.as_ref() != Some(identity) {
            self.latest.clear();
            self.previous_lidar_ns = 0;
            self.identity = Some(identity.clone());
        }
    }

    fn take_ready(&mut self, identity: &MapIdentity, lidar_ns: u64) -> Vec<Observation> {
        self.select_map(identity);
        let mut ready = Vec::new();
        for queue in self.latest.values_mut() {
            let Some(index) = queue
                .iter()
                .rposition(|frame| frame.timestamp_ns <= lidar_ns)
            else {
                continue;
            };
            let frame = queue.drain(..=index).last().unwrap();
            if frame.timestamp_ns > self.previous_lidar_ns {
                ready.push(frame);
            }
        }
        self.previous_lidar_ns = self.previous_lidar_ns.max(lidar_ns);
        ready.sort_by_key(|frame| frame.timestamp_ns);
        ready
    }
}

pub(crate) fn identity(adapter: &MappingAdapter) -> MapIdentity {
    let session = adapter.snapshot();
    let correction = adapter
        .map_odom_transform_snapshot()
        .map_or(0, |t| t.correction_revision);
    (
        session.session_id,
        session.session_generation,
        adapter.global_frame_generation(),
        correction,
    )
}

/// 与LiDAR准入类型分离；只有实际辅助入口和采集位姿配对能构造。
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RegisteredObservation {
    observation: Arc<Observation>,
    map: MapIdentity,
    odom_from_body: Isometry3<f64>,
    received_at_ns: u64,
}

impl RegisteredObservation {
    pub(crate) fn timestamp_ns(&self) -> u64 { self.observation.timestamp_ns }
    pub(crate) fn received_at_ns(&self) -> u64 { self.received_at_ns }
    pub(crate) fn odom_from_body(&self) -> Isometry3<f64> { self.odom_from_body }
    pub(crate) fn origin_body(&self) -> Point3<f64> { Point3::from(self.observation.origin_body) }
    pub(crate) fn body_points(&self) -> &crate::correction_points::CorrectionPoints { &self.observation.points_body }
    pub(crate) fn key(&self) -> (String, String, u64) {
        (self.observation.sensor_id.clone(), self.observation.clock_id.clone(), self.observation.source_sequence)
    }
    #[cfg(test)]
    pub(crate) fn for_test(timestamp_ns: u64, sensor_id: &str, points: Vec<Point3<f64>>,
        pose: Isometry3<f64>, origin: [f64; 3]) -> Self {
        Self { observation: Arc::new(Observation { timestamp_ns, body_frame: "base_link".into(),
            points_body: points.into(), origin_body: origin, digest: [1;32],
            sensor_id: sensor_id.into(), source_sequence: timestamp_ns,
            clock_id: "test-clock".into(), calibration_ref: "test-camera-calibration".into() }),
            map: ("test-session".into(),1,1,0), odom_from_body: pose, received_at_ns: timestamp_ns+1 }
    }

    pub(crate) fn belongs_to(&self, adapter: &MappingAdapter) -> bool {
        adapter.is_active() && self.map == identity(adapter)
    }
}

pub(crate) fn observe_mode(handle: &GeometryRuntimeHandle, epoch: u64) {
    if let Some(adapter) = &handle.mapping_adapter {
        adapter.lock().auxiliary_observations.select_mode(epoch);
    }
}

pub(crate) fn enqueue(
    handle: &GeometryRuntimeHandle,
    request: crate::proto::PublishPointCloudRequest,
    body_frame: &str,
) -> Result<crate::proto::PublishResult, String> {
    if request.timestamp_ns == 0 || request.sensor_id.trim().is_empty()
        || request.clock_id.trim().is_empty() || request.calibration_ref.trim().is_empty()
        || !request.imu_samples.is_empty() {
        return Err("辅助点云需要真实采集时间/来源/标定，且不能携带LIO惯性配组".into());
    }
    if request.frame_id != body_frame {
        return Err(format!("辅助点云需按相机外参注册到{body_frame}"));
    }
    let origin: [f64; 3] = request
        .sensor_origin_xyz
        .as_slice()
        .try_into()
        .map_err(|_| "辅助点云需要三维传感器光心")?;
    if !origin.iter().all(|v| v.is_finite()) { return Err("辅助光心包含非有限数值".into()); }
    let payload = crate::server::encode_pointcloud_request_payload_with_sequence(&request, 0)
        .map_err(|e| e.to_string())?;
    let parsed = super::parse_pointcloud_payload(&payload).ok_or("辅助点云XYZ格式无效")?;
    let mut points = parsed
        .data
        .chunks_exact(parsed.point_step as usize)
        .map(|p| {
            Point3::new(
                f32::from_le_bytes(p[0..4].try_into().unwrap()) as f64,
                f32::from_le_bytes(p[4..8].try_into().unwrap()) as f64,
                f32::from_le_bytes(p[8..12].try_into().unwrap()) as f64,
            )
        })
        .filter(|p| p.coords.iter().all(|v| v.is_finite()))
        .collect::<Vec<_>>();
    if let Some(filter) = crate::self_cloud::RobotSelfFilter::from_env(body_frame)? {
        filter.retain_external(&mut points, &Isometry3::identity());
    }
    let observation = Observation {
        timestamp_ns: request.timestamp_ns,
        body_frame: request.frame_id.clone(),
        points_body: points.into(),
        origin_body: origin,
        digest: crate::map_checksum::sha256_bytes(&request.encode_to_vec()),
        sensor_id: request.sensor_id.clone(),
        source_sequence: request.source_sequence,
        clock_id: request.clock_id.clone(),
        calibration_ref: request.calibration_ref.clone(),
    };
    let mut adapter = handle.mapping_adapter.as_ref().ok_or("建图未启动")?.lock();
    let map = identity(&adapter);
    adapter.auxiliary_observations.select_map(&map);
    // 保留短历史完成异步位姿配对；始终只将已配对的最新一帧送入地图。
    let queue = adapter
        .auxiliary_observations
        .latest
        .entry(request.sensor_id)
        .or_default();
    queue.push_back(observation);
    while queue.len() > 8 {
        queue.pop_front();
    }
    Ok(crate::proto::PublishResult {
        success: true,
        message: "辅助点云已受理，等待激光位姿配对后更新同一地图".into(),
    })
}

pub(crate) async fn flush(
    handle: &GeometryRuntimeHandle,
    lidar_ns: u64,
    expected_generations: Option<(u64, u64, u64)>,
    mode: Option<(
        u64,
        Arc<crate::correction_coordinator::RelocalizationModeFence>,
    )>,
) -> Result<Vec<RegisteredObservation>, String> {
    let (frames, transform, captured_map) = {
        let mut adapter = handle
            .mapping_adapter
            .as_ref()
            .ok_or("建图拥有方缺失")?
            .lock();
        let map = identity(&adapter);
        if expected_generations.is_some_and(|g| g != (map.1, map.2, map.3)) {
            return Ok(Vec::new());
        }
        adapter.auxiliary_observations.select_mode(mode.as_ref().map_or(0, |m| m.0));
        let frames = adapter.auxiliary_observations.take_ready(&map, lidar_ns);
        if frames.is_empty() {
            return Ok(Vec::new());
        }
        let Some(transform) = adapter.map_odom_transform_snapshot() else {
            return Ok(Vec::new());
        };
        (frames, transform, map)
    };
    let tf = Arc::clone(&handle.state.read().await.tf_tree);
    let mut archived = Vec::new();
    for frame in frames {
        let pose = match tf
            .lookup_interpolated_at(&frame.body_frame, frame.timestamp_ns as f64 / 1e9)
        {
            Ok(pose) if pose.parent == transform.odom_frame_id => pose.transform,
            result => {
                tracing::warn!(sensor_id=frame.sensor_id, source_timestamp_ns=frame.timestamp_ns,
                    reason=?result, "辅助观测缺少采集时刻位姿，跳过本帧");
                continue;
            }
        };
        let map_from_body = transform.effective_isometry() * pose;
        let points = frame
            .points_body
            .iter()
            .map(|p| map_from_body * p)
            .collect::<Vec<_>>();
        let origin = map_from_body * Point3::from(frame.origin_body);
        let received_at = crate::server::now_ns();
        let commit = super::commit_registered_lio_cloud_inner(
            handle,
            &points,
            12,
            received_at,
            frame.timestamp_ns,
            &transform.map_frame_id,
            MappingCloudSource::Auxiliary,
            Some((captured_map.1,captured_map.2,captured_map.3)),
            mode.as_ref().map(|m| m.0),
            mode.as_ref().map(|m| Arc::clone(&m.1)),
            Some(origin.coords.into()),
            None,
        )
        .await?;
        let navigation_context=handle.navigation_map_3d_publish_context();
        super::publish_mapping_ingest_commit_with_lifecycle_fence(
            &handle.transport, handle.mapping_adapter.as_ref().ok_or("建图拥有方缺失")?,
            &handle.mapping_lifecycle_lock, &handle.cloud_map_writer, &handle.state,
            &handle.cloud_map_tx, &handle.map_revision_tx, &transform.map_frame_id, &commit, None,
            mode.as_ref().map(|(epoch,fence)| ExpectedRelocalizationMode {
                fence: Arc::clone(fence), epoch:*epoch, active:false,
            }), navigation_context.as_ref(),
        ).await?;
        if !commit.outcome.accepted_map_points.is_empty() {
            tracing::info!(
                event = "mapping.auxiliary.accepted",
                sensor_id = frame.sensor_id,
                source_sequence = frame.source_sequence,
                source_timestamp_ns = frame.timestamp_ns,
                point_count = points.len(),
                "深度点云已进入统一几何地图"
            );
            archived.push(RegisteredObservation {
                observation: Arc::new(frame), map: captured_map.clone(),
                odom_from_body: pose, received_at_ns: received_at,
            });
        }
    }
    Ok(archived)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn frame(stamp: u64, source: &str) -> Observation {
        Observation {
            timestamp_ns: stamp,
            body_frame: "body".into(),
            points_body: vec![Point3::new(1.0, 0.0, -1.0)].into(),
            origin_body: [0.0, 0.0, 0.0],
            digest: [0; 32],
            sensor_id: source.into(),
            source_sequence: stamp,
            clock_id: "clock".into(),
            calibration_ref: "camera-calibration".into(),
        }
    }
    #[test]
    fn auxiliary_frames_join_lidar_in_capture_order_and_do_not_cross_maps() {
        let mut inbox = Inbox::default();
        let map = ("house".into(), 1, 1, 0);
        inbox.select_map(&map);
        inbox
            .latest
            .insert("depth".into(), VecDeque::from([frame(150, "depth")]));
        inbox
            .latest
            .insert("other".into(), VecDeque::from([frame(250, "other")]));
        inbox
            .latest
            .get_mut("depth")
            .unwrap()
            .push_back(frame(350, "depth"));
        assert!(inbox.take_ready(&map, 100).is_empty());
        assert_eq!(inbox.take_ready(&map, 200)[0].timestamp_ns, 150);
        inbox
            .latest
            .insert("depth".into(), VecDeque::from([frame(190, "depth")]));
        let ready = inbox.take_ready(&map, 300);
        assert_eq!(ready.len(), 1);
        assert_eq!(ready[0].timestamp_ns, 250);
        inbox
            .latest
            .insert("depth".into(), VecDeque::from([frame(350, "depth")]));
        assert!(inbox
            .take_ready(&("other-map".into(), 2, 2, 0), 400)
            .is_empty());
    }
    #[test]
    fn auxiliary_same_capture_keeps_two_sensors_and_mode_edges_clear_pending() {
        let mut inbox=Inbox::default();
        let map=("house".into(),1,1,0);
        inbox.select_map(&map);
        for source in ["depth-front","depth-left"] {
            inbox.latest.insert(source.into(),VecDeque::from([frame(150,source)]));
        }
        assert_eq!(inbox.take_ready(&map,200).len(),2);
        inbox.latest.insert("depth-front".into(),VecDeque::from([frame(250,"depth-front")]));
        inbox.select_mode(1);
        assert!(inbox.take_ready(&map,300).is_empty());
        inbox.latest.insert("depth-front".into(),VecDeque::from([frame(350,"depth-front")]));
        inbox.select_mode(2);
        assert!(inbox.take_ready(&map,400).is_empty());
        inbox.latest.insert("depth-front".into(),VecDeque::from([frame(450,"depth-front")]));
        assert!(inbox.take_ready(&("house".into(),1,1,1),500).is_empty());
    }

    struct UnitTransport;
    #[async_trait::async_trait]
    impl nuwax_transport::LcmBackend for UnitTransport {
        async fn publish(&self,_:RawLcmFrame)->nuwax_transport::TransportResult<()> { Ok(()) }
        async fn subscribe(&self,_:LcmTopic,_:Option<LcmMessageType>)->nuwax_transport::LcmFrameStream {
            Box::pin(futures::stream::pending())
        }
        fn name(&self)->&str { "auxiliary-unit" }
    }

    #[tokio::test]
    async fn auxiliary_interpolates_capture_pose_origin_and_obeys_current_owner() {
        let (handle,owner)=build_handle_for_tests_with_unit_transport(true,
            Some(MappingBackendSpec::Voxel),LcmTransport::new(Arc::new(UnitTransport))).await.unwrap();
        let map={
            let mut adapter=handle.mapping_adapter.as_ref().unwrap().lock();
            adapter.start_session("aux-unit","house").unwrap();
            adapter.feed_pose_in_frame(0.,0.,0.,0.,"odom").unwrap();
            identity(&adapter)
        };
        handle.cloud_map_writer.lock().as_mut().unwrap().begin_session(map.1).unwrap();
        let tf=handle.state.read().await.tf_tree.clone();
        for (seconds,x) in [(1,0.),(2,2.)] {
            let mut pose=TransformStamped::identity("odom","base_link");
            pose.stamp_secs=seconds;
            pose.transform=Isometry3::translation(x,0.,0.);
            tf.set(pose);
        }
        let request=crate::proto::PublishPointCloudRequest {
            points:[2_f32,0.,0.].iter().flat_map(|v|v.to_le_bytes()).collect(),
            point_step:12,point_count:1,frame_id:"base_link".into(),timestamp_ns:1_500_000_000,
            mapping_only:true,sensor_origin_xyz:vec![0.2,0.,0.],sensor_id:"depth-front".into(),
            source_sequence:8,clock_id:"clock".into(),calibration_ref:"actual-mount".into(),
            ..Default::default()
        };
        enqueue(&handle,request.clone(),"base_link").unwrap();
        let registered=flush(&handle,2_000_000_000,Some((map.1,map.2,map.3)),None).await.unwrap();
        assert_eq!(registered.len(),1);
        assert_eq!(registered[0].timestamp_ns(),1_500_000_000);
        assert!((registered[0].odom_from_body().translation.x-1.).abs()<1e-9);
        assert!((registered[0].origin_body().x-0.2).abs()<1e-9);
        assert_eq!(registered[0].body_points()[0],Point3::new(2.,0.,0.));
        assert!(registered[0].belongs_to(&handle.mapping_adapter.as_ref().unwrap().lock()));
        enqueue(&handle,crate::proto::PublishPointCloudRequest{timestamp_ns:2_100_000_000,source_sequence:9,..request},"base_link").unwrap();
        assert!(flush(&handle,2_200_000_000,Some((map.1+1,map.2,map.3)),None).await.unwrap().is_empty());
        handle.mapping_adapter.as_ref().unwrap().lock().stop_session("aux-unit").unwrap();
        assert!(!registered[0].belongs_to(&handle.mapping_adapter.as_ref().unwrap().lock()));
        drop(handle);owner.finish().await.unwrap();
    }

}
