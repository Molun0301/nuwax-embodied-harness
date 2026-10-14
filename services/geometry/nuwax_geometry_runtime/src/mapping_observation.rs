//! 辅助点云按采集时间汇入原激光建图拥有方，不另起地图或定位前端。
use super::*;
use nalgebra::{Isometry3, Point3};
use std::collections::{BTreeMap, VecDeque};

// 只借用已有地图代次，不引入新的生命周期。
type MapIdentity = (String, u64, u64, u64);

#[derive(Debug, Clone)]
pub(crate) struct Observation {
    timestamp_ns: u64,
    body_frame: String,
    points_body: crate::correction_points::CorrectionPoints,
    origin_body: [f64; 3],
    original_indices: Vec<u32>,
    sensor_id: String,
    source_sequence: u64,
    clock_id: String,
    calibration_ref: String,
    mapping_only: bool,
}

// 等价准入观测按实际点/索引与绑定比较；线布局、填充和被过滤返回不再制造摘要冲突。
// 不保存第二份原始请求，也不序列化或散列请求来判断相等。
impl PartialEq for Observation {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_ns == other.timestamp_ns && self.body_frame == other.body_frame
            && self.points_body == other.points_body && self.original_indices == other.original_indices
            && self.origin_body.iter().zip(other.origin_body).all(|(a, b)| a.to_bits() == b.to_bits())
            && self.sensor_id == other.sensor_id && self.source_sequence == other.source_sequence
            && self.clock_id == other.clock_id && self.calibration_ref == other.calibration_ref
            && self.mapping_only == other.mapping_only
    }
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
#[derive(Debug, Clone)]
pub(crate) struct RegisteredObservation {
    observation: Arc<Observation>,
    map: MapIdentity,
    odom_from_body: Isometry3<f64>,
    map_from_body: Isometry3<f64>,
    map_frame: String,
    odom_frame: String,
    layer_source: Arc<nuwax_map_chunks::LayerSource>,
    received_at_ns: u64,
}

impl PartialEq for RegisteredObservation {
    fn eq(&self, other: &Self) -> bool {
        fn same_pose(a: &Isometry3<f64>, b: &Isometry3<f64>) -> bool {
            a.translation.vector.iter().chain(a.rotation.coords.iter())
                .zip(b.translation.vector.iter().chain(b.rotation.coords.iter()))
                .all(|(a, b)| a.to_bits() == b.to_bits())
        }
        self.observation == other.observation && self.map == other.map
            && self.map_frame == other.map_frame && self.odom_frame == other.odom_frame
            && self.layer_source == other.layer_source
            && same_pose(&self.odom_from_body, &other.odom_from_body)
            && same_pose(&self.map_from_body, &other.map_from_body)
            && self.received_at_ns == other.received_at_ns
    }
}

impl RegisteredObservation {
    pub(crate) fn timestamp_ns(&self) -> u64 { self.observation.timestamp_ns }
    pub(crate) fn source_sequence(&self) -> u64 { self.observation.source_sequence }
    pub(crate) fn original_indices(&self) -> &[u32] { &self.observation.original_indices }
    pub(crate) fn layer_source(&self) -> &Arc<nuwax_map_chunks::LayerSource> { &self.layer_source }
    pub(crate) fn map_from_body(&self) -> Isometry3<f64> { self.map_from_body }
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
            original_indices: (0..points.len() as u32).collect(),
            points_body: points.into(), origin_body: origin,
            sensor_id: sensor_id.into(), source_sequence: timestamp_ns,
            clock_id: "test-clock".into(), calibration_ref: "test-camera-calibration".into(), mapping_only: true }),
            map: ("test-session".into(),1,1,0), odom_from_body: pose, map_from_body: pose,
            map_frame: "map".into(), odom_frame: "odom".into(),
            layer_source: Arc::new(nuwax_map_chunks::LayerSource::Auxiliary { sensor_id: sensor_id.into(),
                clock_id: "test-clock".into(), runtime_generation: 1, session_generation: 1 }),
            received_at_ns: timestamp_ns+1 }
    }

    #[cfg(test)]
    pub(crate) fn for_test_source(timestamp_ns: u64, sequence: u64, source: nuwax_map_chunks::LayerSource,
        points: Vec<Point3<f64>>, original_indices: Vec<u32>, pose: Isometry3<f64>) -> Self {
        let nuwax_map_chunks::LayerSource::Auxiliary { sensor_id, clock_id, session_generation, .. } = &source
            else { panic!("auxiliary test source required"); };
        assert_eq!(points.len(), original_indices.len());
        let mut value = Self::for_test(timestamp_ns, sensor_id, points, pose, [0.0; 3]);
        let observation = Arc::get_mut(&mut value.observation).unwrap();
        observation.source_sequence = sequence;
        observation.clock_id = clock_id.clone();
        observation.original_indices = original_indices;
        value.map.1 = *session_generation;
        value.layer_source = Arc::new(source);
        value
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
    let filter = crate::self_cloud::RobotSelfFilter::from_env(body_frame)?;
    let (points, original_indices) = capture_body_points(&parsed.data, parsed.point_step as usize, filter.as_ref());
    let observation = Observation {
        timestamp_ns: request.timestamp_ns,
        body_frame: request.frame_id.clone(),
        points_body: points.into(),
        origin_body: origin,
        original_indices,
        sensor_id: request.sensor_id.clone(),
        source_sequence: request.source_sequence,
        clock_id: request.clock_id.clone(),
        calibration_ref: request.calibration_ref.clone(),
        mapping_only: request.mapping_only,
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

/// 已校验的 XYZ 布局按原 emitted 顺序读取，过滤只删样本，不重编号。
fn capture_body_points(data: &[u8], point_step: usize, filter: Option<&crate::self_cloud::RobotSelfFilter>)
    -> (Vec<Point3<f64>>, Vec<u32>) {
    data.chunks_exact(point_step).enumerate().filter_map(|(index, bytes)| {
        let point = Point3::new(
            f32::from_le_bytes(bytes[0..4].try_into().unwrap()) as f64,
            f32::from_le_bytes(bytes[4..8].try_into().unwrap()) as f64,
            f32::from_le_bytes(bytes[8..12].try_into().unwrap()) as f64,
        );
        (point.coords.iter().all(|v| v.is_finite())
            && !filter.is_some_and(|filter| filter.contains_body(&point)))
            .then_some((point, index as u32))
    }).unzip()
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
        let received_at = crate::server::now_ns();
        let layer_source = Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
            sensor_id: frame.sensor_id.clone(), clock_id: frame.clock_id.clone(),
            runtime_generation: handle.nav.producer_runtime_generation(), session_generation: captured_map.1,
        });
        let registered = RegisteredObservation {
            observation: Arc::new(frame), map: captured_map.clone(), odom_from_body: pose,
            map_from_body: transform.effective_isometry() * pose,
            map_frame: transform.map_frame_id.clone(), odom_frame: transform.odom_frame_id.clone(),
            layer_source, received_at_ns: received_at,
        };
        let origin = registered.map_from_body() * registered.origin_body();
        let accepted = crate::admitted_cloud::AcceptedMappingCloud::auxiliary(registered.clone());
        let points = accepted.map_points();
        let commit = super::commit_registered_lio_cloud_inner(
            handle,
            points,
            12,
            received_at,
            registered.timestamp_ns(),
            &transform.map_frame_id,
            MappingCloudSource::Auxiliary,
            Some((captured_map.1,captured_map.2,captured_map.3)),
            mode.as_ref().map(|m| m.0),
            mode.as_ref().map(|m| Arc::clone(&m.1)),
            Some(origin.coords.into()),
            Some(accepted.clone()),
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
                sensor_id = registered.observation.sensor_id,
                source_sequence = registered.source_sequence(),
                source_timestamp_ns = registered.timestamp_ns(),
                point_count = points.len(),
                "深度点云已进入统一几何地图"
            );
            archived.push(registered);
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
            original_indices: vec![0],
            sensor_id: source.into(),
            source_sequence: stamp,
            clock_id: "clock".into(),
            calibration_ref: "camera-calibration".into(),
            mapping_only: true,
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

    #[test]
    fn auxiliary_filters_keep_original_emitted_indices_and_identical_return_values() {
        let points = [[0_f32, 0., 0.], [f32::NAN, 0., 0.], [0.8, 0.1, -0.4], [1.2, 0.3, 0.2]];
        let bytes: Vec<_> = points.iter().flatten().flat_map(|value| value.to_le_bytes()).collect();
        let filter = crate::self_cloud::tests::filter();
        let (filtered, indices) = capture_body_points(&bytes, 12, Some(&filter));
        assert_eq!(indices, vec![2, 3]);
        for (point, index) in filtered.iter().zip(indices) {
            assert_eq!(*point, Point3::from(points[index as usize].map(f64::from)));
        }
    }

    #[test]
    fn auxiliary_same_key_compares_effective_points_indices_and_capture_binding_without_hashes() {
        let original = RegisteredObservation::for_test(10, "depth-front", vec![Point3::new(1., 0., -0.4)],
            Isometry3::identity(), [0.; 3]);
        let mut history = crate::pgo_map::ScanHistory::default();
        history.record_auxiliary(original.clone()).unwrap();
        let equivalent = RegisteredObservation::for_test(10, "depth-front", vec![Point3::new(1., 0., -0.4)],
            Isometry3::identity(), [0.; 3]);
        history.record_auxiliary(equivalent).unwrap();
        assert_eq!(history.snapshot().unwrap().len(), 1);
        for change in 0..7 {
            let mut changed = original.clone();
            let mut observation = (*changed.observation).clone();
            match change {
                0 => observation.points_body = vec![Point3::new(1.1, 0., -0.4)].into(),
                1 => observation.original_indices = vec![7],
                2 => observation.calibration_ref = "different-mount".into(),
                3 => observation.origin_body[0] = -0.0,
                4 => observation.body_frame = "other-body".into(),
                5 => observation.mapping_only = false,
                _ => changed.odom_from_body.translation.vector.x = 0.1,
            }
            changed.observation = Arc::new(observation);
            assert!(history.record_auxiliary(changed).unwrap_err().contains("来源冲突"));
        }
        // 不同 NaN 负载及每点末尾填充都被规范过滤；原保留索引与有效点完全相同。
        let make = |nan_bits: u32, padding: u32| {
            let mut bytes = Vec::new();
            for point in [[f32::from_bits(nan_bits), 0., 0.], [1., 0., -0.4]] {
                bytes.extend(point.iter().flat_map(|value| value.to_le_bytes()));
                bytes.extend_from_slice(&padding.to_le_bytes());
            }
            let (points, indices) = capture_body_points(&bytes, 16, None);
            let mut observation = frame(10, "depth-front");
            observation.points_body = points.into(); observation.original_indices = indices;
            observation
        };
        assert_eq!(make(0x7fc0_0001, 1), make(0x7fc0_0002, 9));
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

    #[tokio::test]
    async fn auxiliary_admission_publishes_measured_floor_and_keeps_wall_occupancy() {
        let (handle, owner) = build_handle_for_tests_with_unit_transport(true,
            Some(MappingBackendSpec::Voxel), LcmTransport::new(Arc::new(UnitTransport))).await.unwrap();
        let map = {
            let mut adapter = handle.mapping_adapter.as_ref().unwrap().lock();
            adapter.start_session("aux-ground-unit", "house").unwrap();
            adapter.feed_pose_in_frame(0., 0., 0., 0., "odom").unwrap();
            identity(&adapter)
        };
        handle.cloud_map_writer.lock().as_mut().unwrap().begin_session(map.1).unwrap();
        let tf = handle.state.read().await.tf_tree.clone();
        for seconds in [1, 2] {
            let mut pose = TransformStamped::identity("odom", "base_link");
            pose.stamp_secs = seconds;
            tf.set(pose);
        }
        let mut points = vec![[f32::NAN, 0., 0.]];
        points.extend((0..8).flat_map(|x| (0..8).map(move |y|
            [0.008_f32 + x as f32 * 0.004, 0.008 + y as f32 * 0.004, -0.41])));
        points.extend((0..8).flat_map(|y| (0..8).map(move |z|
            [0.28_f32, 0.008 + y as f32 * 0.02, 0.21 + z as f32 * 0.02])));
        let payload: Vec<_> = points.iter().flatten().flat_map(|value| value.to_le_bytes()).collect();
        for sensor in ["depth-front", "depth-left"] {
            enqueue(&handle, crate::proto::PublishPointCloudRequest {
                points: payload.clone(), point_step: 12, point_count: points.len() as u32,
                frame_id: "base_link".into(), timestamp_ns: 1_500_000_000, mapping_only: true,
                sensor_origin_xyz: vec![0., 0., 0.], sensor_id: sensor.into(), source_sequence: 7,
                clock_id: "clock".into(), calibration_ref: "actual-mount".into(), ..Default::default()
            }, "base_link").unwrap();
        }
        let registered = flush(&handle, 2_000_000_000, Some((map.1, map.2, map.3)), None).await.unwrap();
        assert_eq!(registered.len(), 2);
        assert_eq!(registered[0].original_indices()[0], 1);
        let root = handle.cloud_map_writer.lock().as_ref().unwrap().committed_fine_cloud().unwrap();
        let mut measured = Vec::new();
        let mut wall_occupied = false;
        for block in &root.manifest().blocks {
            let bytes = root.repository.read_object(block.object, nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
            let occupancy = nuwax_map_chunks::decode_block(&bytes).unwrap();
            wall_occupied |= occupancy.keys().any(|key| {
                let cell = nuwax_map_chunks::global_key(block.key, *key).unwrap();
                cell[0] == 5 && cell[2] >= 4
            });
            measured.extend(nuwax_map_chunks::decode_layer_observations(&bytes).unwrap().into_values());
        }
        assert!(wall_occupied);
        assert!(!measured.is_empty());
        for sample in measured.iter().flat_map(|layer| &layer.samples) {
            assert!((sample.point[2] + 0.41).abs() < 1e-6, "wall cannot enter measured ground");
            assert!(sample.point_index > 0 && sample.point_index <= 64);
            assert_eq!(sample.point, points[sample.point_index as usize].map(f64::from));
            assert!(matches!(sample.source.as_ref(), nuwax_map_chunks::LayerSource::Auxiliary {
                runtime_generation, session_generation, ..
            } if *runtime_generation == handle.nav.producer_runtime_generation() && *session_generation == map.1));
        }
        // 首次根可能早于第二传感器的发布节拍；两帧实际已归档且指向各自真实来源。
        assert_ne!(registered[0].layer_source(), registered[1].layer_source());
        drop(handle);
        owner.finish().await.unwrap();
    }

}
