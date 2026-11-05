//! LIO（激光惯性里程计）输入、子进程和持续位姿的唯一拥有方。
use crate::{
    lio_ingress::*,
    mapper::MappingFrame,
    native_input_delivery::NativeInputDelivery,
    native_lio_owner::{BoundNativeLioRuntime, NativeLioPreflight, NativeProcessOwner},
    proto,
    session::FrameRouter,
};
use nalgebra::Point3;
use nuwax_codec::lcm::schema;
use nuwax_lio_backends::native::NativeBackend;
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::LcmTransport;
use std::{collections::VecDeque, sync::Arc};
use tokio::{
    sync::{mpsc, watch, Mutex},
    task::JoinHandle,
};
use tonic::Status;

type Binding = (LcmTopic, LcmMessageType);
struct AuxiliaryFrame {
    session_id: u64,
    timestamp_ns: u64,
    points: Vec<Point3<f64>>,
    origin: [f64; 3],
    sensor_id: String,
}
#[derive(Clone)]
struct InputRuntime {
    transport: LcmTransport,
    native: Option<BoundNativeLioRuntime>,
    planar: Option<Arc<crate::slam2d_owner::Owner>>,
    odometry_input: mpsc::UnboundedSender<proto::Pose>,
    cloud_input: mpsc::UnboundedSender<(u64, schema::PointCloud2)>,
}
struct Runtime {
    planar: Option<Arc<crate::slam2d_owner::Owner>>,
    shutdown: watch::Sender<bool>,
    tasks: Vec<JoinHandle<()>>,
}

pub(crate) struct Lio {
    router: FrameRouter,
    info: proto::RuntimeInfo,
    native: Option<BoundNativeLioRuntime>,
    planar: Option<crate::slam2d_owner::Settings>,
    runtime: Mutex<Option<Runtime>>,
    input_runtime: parking_lot::RwLock<Option<InputRuntime>>,
    input: Mutex<NativeInputDelivery>,
    native_self_filter: Option<crate::self_cloud::NativeSelfFilter>,
    sensor_frame: String,
    body_from_sensor: Option<crate::native_lio_owner::BodyFromSensorTransform>,
    auxiliary: Arc<parking_lot::Mutex<VecDeque<AuxiliaryFrame>>>,
}
impl Lio {
    pub(crate) async fn start(router: FrameRouter) -> Result<Self, String> {
        let planar = crate::slam2d_owner::Settings::from_env()?;
        let map = env("NUWAX_MAP_FRAME", "map");
        let odom = env("NUWAX_GEOMETRY_ODOM_FRAME", "odom");
        let body = env("NUWAX_BASE_FRAME", "base_link");
        let sensor = env("NUWAX_GEOMETRY_LIDAR_FRAME", "utlidar_lidar");
        let native = if planar.is_none() {
            match env("NUWAX_GEOMETRY_LIO_BACKEND", "device").as_str() {
                "device" => None,
                "fastlio2" => Some(
                    NativeLioPreflight::from_env(NativeBackend::FastLio2)?
                        .bind(&sensor, &body, &odom)?,
                ),
                "pointlio" => Some(
                    NativeLioPreflight::from_env(NativeBackend::PointLio)?
                        .bind(&sensor, &body, &odom)?,
                ),
                value => return Err(format!("不支持的LIO后端：{value}")),
            }
        } else {
            None
        };
        let info = if let Some(settings) = &planar {
            proto::RuntimeInfo {
                pipeline: "slam_2d".into(),
                map_frame_id: settings.template.map_frame_id.clone(),
                odom_frame_id: settings.template.odom_frame_id.clone(),
                base_frame_id: settings.template.body_frame_id.clone(),
                planar_input_config_json: serde_json::to_string(&settings.template)
                    .map_err(|e| e.to_string())?,
                planar_unix_offset_ns: i64::try_from(settings.unix_offset_ns)
                    .map_err(|e| e.to_string())?,
            }
        } else {
            proto::RuntimeInfo {
                pipeline: "lio_3d".into(),
                map_frame_id: map,
                odom_frame_id: odom,
                base_frame_id: body.clone(),
                ..Default::default()
            }
        };
        let body_from_sensor = if let Some(native) = &native {
            Some(native.body_from_sensor.clone())
        } else if planar.is_none()
            && std::env::var_os(crate::native_lio_owner::BODY_FROM_SENSOR_ENV).is_some()
        {
            Some(crate::native_lio_owner::BodyFromSensorTransform::from_env()?)
        } else {
            None
        };
        let result = Self {
            router,
            info,
            native_self_filter: if native.is_some() {
                crate::self_cloud::NativeSelfFilter::from_env(&body)?
            } else {
                None
            },
            native,
            sensor_frame: sensor,
            body_from_sensor,
            planar,
            runtime: Mutex::new(None),
            input_runtime: parking_lot::RwLock::new(None),
            input: Mutex::new(NativeInputDelivery::default()),
            auxiliary: Arc::new(parking_lot::Mutex::new(VecDeque::new())),
        };
        result.recover().await?;
        Ok(result)
    }
    pub(crate) fn info(&self) -> proto::RuntimeInfo {
        self.info.clone()
    }

    pub(crate) fn static_transforms(&self) -> Vec<proto::Transform> {
        let (translation, rotation, sensor) = if let Some(body_from_sensor) = &self.body_from_sensor
        {
            let matrix = nalgebra::Matrix3::from_row_slice(&body_from_sensor.rotation_row_major());
            let rotation = nalgebra::UnitQuaternion::from_rotation_matrix(
                &nalgebra::Rotation3::from_matrix_unchecked(matrix),
            );
            (
                body_from_sensor.translation_m(),
                rotation,
                self.sensor_frame.clone(),
            )
        } else if let Some(planar) = &self.planar {
            let values = planar.template.core_config["base_from_laser"]
                .as_array()
                .expect("已验证二维外参");
            (
                [
                    values[0].as_f64().unwrap(),
                    values[1].as_f64().unwrap(),
                    0.0,
                ],
                nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, values[2].as_f64().unwrap()),
                planar.template.frame_id.clone(),
            )
        } else {
            return Vec::new();
        };
        vec![proto::Transform {
            parent_frame: self.info.base_frame_id.clone(),
            child_frame: sensor,
            translation: Some(proto::Vec3 {
                x: translation[0],
                y: translation[1],
                z: translation[2],
            }),
            rotation: Some(proto::Quat {
                x: rotation.i,
                y: rotation.j,
                z: rotation.k,
                w: rotation.w,
            }),
            timestamp_ns: 0,
        }]
    }

    pub(crate) async fn is_running(&self) -> bool {
        let planar = {
            let runtime = self.runtime.lock().await;
            let Some(runtime) = runtime.as_ref() else {
                return false;
            };
            if *runtime.shutdown.borrow() || runtime.tasks.iter().any(JoinHandle::is_finished) {
                return false;
            }
            runtime.planar.clone()
        };
        if let Some(planar) = planar {
            return planar.is_running().await;
        }
        !self.input.lock().await.has_failed()
    }
    pub(crate) async fn recover(&self) -> Result<(), String> {
        if self.is_running().await {
            return Ok(());
        }
        self.shutdown().await;
        *self.input.lock().await = NativeInputDelivery::default();
        self.auxiliary.lock().clear();
        let transport =
            LcmTransport::new_real(&env("NUWAX_LCM_URL", "udpm://239.255.76.67:7667?ttl=0"))
                .map_err(|e| e.to_string())?;
        let (shutdown, rx) = watch::channel(false);
        let (odometry_input, odometry_rx) = mpsc::unbounded_channel();
        let (cloud_input, cloud_rx) = mpsc::unbounded_channel();
        let mut tasks = Vec::new();
        let planar = if let Some(settings) = &self.planar {
            Some(
                crate::slam2d_owner::Owner::start(
                    settings.clone(),
                    transport.clone(),
                    self.router.clone(),
                )
                .await?,
            )
        } else {
            let odometry_binding = match &self.native {
                Some(native) => native.channels.source_odometry.clone(),
                None => binding(
                    &env("NUWAX_LCM_CHANNEL_ODOM", "/odom#nav_msgs.Odometry"),
                    "nav_msgs.Odometry",
                )?,
            };
            let cloud_binding = match &self.native {
                Some(native) => native.channels.source_cloud.clone(),
                None => binding(
                    &env(
                        "NUWAX_LCM_CHANNEL_POINTCLOUD",
                        "/lidar/points#sensor_msgs.PointCloud2",
                    ),
                    "sensor_msgs.PointCloud2",
                )?,
            };
            let odometry = transport
                .subscribe_owned(odometry_binding.0, Some(odometry_binding.1))
                .await
                .map_err(|e| e.to_string())?;
            let cloud = transport
                .subscribe_owned(cloud_binding.0, Some(cloud_binding.1))
                .await
                .map_err(|e| e.to_string())?;
            let ingress = Ingress {
                router: self.router.clone(),
                native: self.native.clone(),
                self_filter: crate::self_cloud::RobotSelfFilter::from_env(
                    &self.info.base_frame_id,
                )?,
                transport: transport.clone(),
                auxiliary: self.auxiliary.clone(),
            };
            tasks.push(tokio::spawn(ingress.run(
                odometry,
                cloud,
                odometry_rx,
                cloud_rx,
                rx.clone(),
            )));
            if let Some(native) = &self.native {
                let process = NativeProcessOwner::spawn(&native.runtime)?;
                let router = self.router.clone();
                let stopped = shutdown.clone();
                tasks.push(tokio::spawn(crate::lio_owner::supervise_native(
                    process,
                    rx,
                    move |reason| {
                        stopped.send_replace(true);
                        router.fault(reason);
                    },
                )));
            }
            None
        };
        *self.input_runtime.write() = Some(InputRuntime {
            transport,
            native: self.native.clone(),
            planar: planar.clone(),
            odometry_input,
            cloud_input,
        });
        *self.runtime.lock().await = Some(Runtime {
            planar,
            shutdown,
            tasks,
        });
        Ok(())
    }
    pub(crate) async fn shutdown(&self) {
        *self.input_runtime.write() = None;
        let runtime = self.runtime.lock().await.take();
        if let Some(runtime) = runtime {
            runtime.shutdown.send_replace(true);
            if let Some(planar) = runtime.planar {
                if let Err(error) = planar.shutdown().await {
                    self.router.fault(error);
                }
            }
            for task in runtime.tasks {
                if let Err(error) = task.await {
                    self.router.fault(error.to_string());
                }
            }
        }
    }
    pub(crate) async fn publish_point_cloud(
        &self,
        request: proto::PublishPointCloudRequest,
    ) -> Result<(), Status> {
        if self.info.pipeline != "lio_3d" {
            return Err(Status::failed_precondition("二维执行链不接收三维点云"));
        }
        if request.mapping_only {
            return self.publish_auxiliary(request);
        }
        let runtime = self
            .input_runtime
            .read()
            .clone()
            .ok_or_else(|| Status::unavailable("LIO拥有方已关闭"))?;
        if runtime.planar.is_some() {
            return Err(Status::failed_precondition("二维执行链不接收三维点云"));
        }
        if let Some(native) = &runtime.native {
            if request.frame_id != self.sensor_frame
                || request
                    .imu_samples
                    .iter()
                    .any(|sample| sample.frame_id != self.sensor_frame)
            {
                return Err(Status::invalid_argument(
                    "原生点云和惯性坐标必须与配置传感器一致",
                ));
            }
            let request = match &self.native_self_filter {
                Some(filter) => filter
                    .mask_request(&request)
                    .map_err(Status::invalid_argument)?,
                None => request,
            };
            let mut input = self.input.lock().await;
            let failed_before = input.has_failed();
            let result = input
                .publish(&runtime.transport, &native.channels, &request)
                .await;
            if let Err(error) = &result {
                if !failed_before && error.code() != tonic::Code::InvalidArgument {
                    self.router.fault(error.to_string());
                }
            }
            result
        } else {
            if !request.imu_samples.is_empty() {
                return Err(Status::failed_precondition("设备LIO没有原生惯性输入通道"));
            }
            if request.frame_id != self.info.odom_frame_id {
                return Err(Status::invalid_argument("设备LIO配准点云必须在里程计坐标"));
            }
            let payload = encode_pointcloud_request_payload(&request)
                .map_err(|e| Status::invalid_argument(e.to_string()))?;
            let cloud = decode_cloud(&payload).map_err(Status::invalid_argument)?;
            runtime
                .cloud_input
                .send((self.router.session_id(), cloud))
                .map_err(|_| Status::unavailable("LIO输入任务已停止"))
        }
    }
    fn publish_auxiliary(&self, request: proto::PublishPointCloudRequest) -> Result<(), Status> {
        let session_id = self.router.session_id();
        if request.frame_id != self.info.base_frame_id {
            return Err(Status::invalid_argument("辅助点云必须在机体坐标"));
        }
        let payload = encode_pointcloud_request_payload(&request)
            .map_err(|e| Status::invalid_argument(e.to_string()))?;
        let cloud = decode_cloud(&payload).map_err(|e| Status::invalid_argument(e.to_string()))?;
        let points = cloud_points(&cloud).map_err(Status::invalid_argument)?;
        let origin: [f64; 3] = request
            .sensor_origin_xyz
            .try_into()
            .map_err(|_| Status::invalid_argument("辅助点云需要物理传感器原点"))?;
        if origin.iter().any(|value| !value.is_finite()) {
            return Err(Status::invalid_argument("辅助点云原点含非有限数值"));
        }
        let mut queue = self.auxiliary.lock();
        queue.push_back(AuxiliaryFrame {
            session_id,
            timestamp_ns: request.timestamp_ns,
            points,
            origin,
            sensor_id: request.sensor_id,
        });
        while queue.len() > 16 {
            queue.pop_front();
        }
        Ok(())
    }
    pub(crate) async fn publish_odom(
        &self,
        request: proto::PublishOdomRequest,
    ) -> Result<(), Status> {
        let pose = request
            .pose
            .clone()
            .ok_or_else(|| Status::invalid_argument("缺少里程计位姿"))?;
        pose_isometry(&pose).map_err(Status::invalid_argument)?;
        let runtime = self
            .input_runtime
            .read()
            .clone()
            .ok_or_else(|| Status::unavailable("LIO拥有方已关闭"))?;
        if let Some(planar) = &runtime.planar {
            if self.planar.as_ref().is_some_and(|settings| {
                settings.template.odometry_source
                    != nuwax_slam_2d_runtime::protocol::OdometrySource::Device
            }) {
                return Err(Status::failed_precondition(
                    "激光里程计模式不接收设备里程计",
                ));
            }
            return planar
                .publish_odom(request)
                .await
                .map_err(Status::invalid_argument);
        }
        if runtime.native.is_some() {
            return Err(Status::failed_precondition("原生LIO不接收设备里程计"));
        }
        if pose.frame_id != self.info.odom_frame_id {
            return Err(Status::invalid_argument("设备里程计坐标与配置不一致"));
        }
        runtime
            .odometry_input
            .send(pose)
            .map_err(|_| Status::unavailable("LIO输入任务已停止"))?;
        Ok(())
    }
    pub(crate) async fn publish_laser_scan(
        &self,
        request: proto::PublishLaserScanRequest,
    ) -> Result<(), Status> {
        let planar = self
            .input_runtime
            .read()
            .as_ref()
            .and_then(|runtime| runtime.planar.clone())
            .ok_or_else(|| Status::failed_precondition("当前不是二维执行链"))?;
        planar
            .publish_scan(&request.payload)
            .await
            .map_err(Status::invalid_argument)
    }
    pub(crate) async fn start_session(
        &self,
        id: u64,
        name: &str,
        checkpoint: Option<Vec<u8>>,
        localization: bool,
        seed: Option<proto::Pose>,
    ) -> Result<(), String> {
        let planar = self
            .runtime
            .lock()
            .await
            .as_ref()
            .and_then(|r| r.planar.clone());
        if let Some(planar) = planar {
            planar
                .start_session(id, name, checkpoint, localization, seed)
                .await?;
        }
        Ok(())
    }
    pub(crate) async fn stop_session(&self) -> Result<(), String> {
        let planar = self
            .runtime
            .lock()
            .await
            .as_ref()
            .and_then(|r| r.planar.clone());
        if let Some(planar) = planar {
            planar.stop_session().await?;
        }
        Ok(())
    }
    pub(crate) async fn planar_checkpoint(&self) -> Result<Option<Vec<u8>>, String> {
        let planar = self
            .runtime
            .lock()
            .await
            .as_ref()
            .and_then(|r| r.planar.clone());
        match planar {
            Some(planar) => planar.checkpoint().await.map(Some),
            None => Ok(None),
        }
    }
    pub(crate) async fn relocalize_planar(
        &self,
        seed: Option<proto::Pose>,
    ) -> Result<proto::RelocalizationResult, String> {
        let planar = self
            .runtime
            .lock()
            .await
            .as_ref()
            .and_then(|r| r.planar.clone())
            .ok_or("当前不是二维执行链")?;
        planar.relocalize(seed).await
    }
}
fn env(name: &str, default: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| default.into())
}
fn binding(value: &str, default_type: &str) -> Result<Binding, String> {
    let (topic, message) = value.split_once('#').unwrap_or((value, default_type));
    Ok((
        LcmTopic::new(topic).map_err(|e| e.to_string())?,
        LcmMessageType::new(message).map_err(|e| e.to_string())?,
    ))
}

struct Ingress {
    router: FrameRouter,
    native: Option<BoundNativeLioRuntime>,
    self_filter: Option<crate::self_cloud::RobotSelfFilter>,
    transport: LcmTransport,
    auxiliary: Arc<parking_lot::Mutex<VecDeque<AuxiliaryFrame>>>,
}
impl Ingress {
    async fn run(
        self,
        mut odometry: nuwax_transport::OwnedLcmSubscription,
        mut cloud: nuwax_transport::OwnedLcmSubscription,
        mut odometry_input: mpsc::UnboundedReceiver<proto::Pose>,
        mut cloud_input: mpsc::UnboundedReceiver<(u64, schema::PointCloud2)>,
        mut shutdown: watch::Receiver<bool>,
    ) {
        let mut poses = VecDeque::new();
        let mut clouds = VecDeque::new();
        let result: Result<(), String> = async {
            loop {
                tokio::select! {biased;
                    _=async {let _=shutdown.wait_for(|stop|*stop).await;}=>return Ok(()),
                    Some(pose)=odometry_input.recv()=>{
                        self.router.publish_pose(self.router.session_id(),pose.clone());
                        poses.push_back(pose);
                        while poses.len()>256{poses.pop_front();}
                        self.deliver_auxiliary(&poses)?;
                    },
                    Some((session_id,cloud))=cloud_input.recv()=>{
                        clouds.push_back((session_id,header_stamp(&cloud.header)?,cloud));
                        while clouds.len()>16{clouds.pop_front();}
                    },
                    frame=odometry.recv()=>{
                        let frame=frame.map_err(|e|e.to_string())?;
                        let message=decode_odometry(&frame.payload).map_err(|e|e.to_string())?;
                        let pose=pose_from_odometry(&message)?;
                        self.router.publish_pose(self.router.session_id(),pose.clone());
                        if self.native.is_some(){self.publish_pose_bypass(&message).await?;}
                        poses.push_back(pose);
                        while poses.len()>256{poses.pop_front();}
                        self.deliver_auxiliary(&poses)?;
                    },
                    frame=cloud.recv()=>{
                        let frame=frame.map_err(|e|e.to_string())?;
                        let cloud=decode_cloud(&frame.payload).map_err(|e|e.to_string())?;
                        let stamp=header_stamp(&cloud.header)?;
                        clouds.push_back((self.router.session_id(),stamp,cloud));
                        while clouds.len()>16{clouds.pop_front();}
                    },
                }
                while let Some((session, pose, cloud)) = take_pair(
                    &poses,
                    &mut clouds,
                    self.native.as_ref().map(|native| native.pair_max_skew),
                ) {
                    self.deliver(session, pose, cloud).await?;
                }
            }
        }
        .await;
        if let Err(error) = result {
            self.router.fault(error);
        }
        let _ = odometry.unsubscribe().await;
        let _ = cloud.unsubscribe().await;
    }
    fn deliver_auxiliary(&self, poses: &VecDeque<proto::Pose>) -> Result<(), String> {
        let Some(latest) = poses.back() else {
            return Ok(());
        };
        let frames = {
            let mut queue = self.auxiliary.lock();
            let mut frames = Vec::new();
            let mut pending = VecDeque::new();
            while let Some(frame) = queue.pop_front() {
                if frame.timestamp_ns <= latest.timestamp_ns {
                    frames.push(frame);
                } else {
                    pending.push_back(frame);
                }
            }
            *queue = pending;
            frames
        };
        for mut frame in frames {
            let Some(pose) = capture_pose(poses, frame.timestamp_ns)? else {
                continue;
            };
            if let Some(filter) = &self.self_filter {
                filter.retain_external(&mut frame.points, &nalgebra::Isometry3::identity());
            }
            self.router.publish_frame(MappingFrame {
                session_id: frame.session_id,
                odom_pose: pose,
                points_body: frame.points.iter().map(|p| [p.x, p.y, p.z]).collect(),
                ray_origin_body: frame.origin,
                mapping_only: true,
                sensor_id: frame.sensor_id,
                registered_cloud: None,
            });
        }
        Ok(())
    }
    async fn deliver(
        &self,
        session_id: u64,
        pose: proto::Pose,
        cloud: schema::PointCloud2,
    ) -> Result<(), String> {
        let mut points = cloud_points(&cloud)?;
        if self.native.is_none() && cloud.header.frame_id != pose.frame_id {
            return Err("设备点云与里程计坐标不一致".into());
        }
        let odom_from_body = pose_isometry(&pose)?;
        let ray_origin = if let Some(native) = &self.native {
            for point in &mut points {
                *point = native.body_from_sensor.transform_point(point);
            }
            native.body_from_sensor.ray_origin_body()
        } else {
            let body_from_odom = odom_from_body.inverse();
            for point in &mut points {
                *point = body_from_odom.transform_point(point);
            }
            Point3::origin()
        };
        if let Some(filter) = &self.self_filter {
            filter.retain_external(&mut points, &nalgebra::Isometry3::identity());
        }
        if self.native.is_some() {
            let mut bytes = Vec::with_capacity(points.len() * 12);
            for point in &points {
                let value = odom_from_body.transform_point(point);
                for value in [value.x, value.y, value.z] {
                    bytes.extend_from_slice(&(value as f32).to_le_bytes());
                }
            }
            let message = schema::PointCloud2 {
                header: schema::Header {
                    frame_id: pose.frame_id.clone(),
                    ..cloud.header.clone()
                },
                height: 1,
                width: points.len() as i32,
                point_step: 12,
                row_step: bytes.len() as i32,
                data: bytes,
                fields: vec![
                    schema::PointField {
                        name: "x".into(),
                        offset: 0,
                        datatype: 7,
                        count: 1,
                    },
                    schema::PointField {
                        name: "y".into(),
                        offset: 4,
                        datatype: 7,
                        count: 1,
                    },
                    schema::PointField {
                        name: "z".into(),
                        offset: 8,
                        datatype: 7,
                        count: 1,
                    },
                ],
                is_bigendian: false,
                is_dense: true,
            };
            let binding = binding(
                &env(
                    "NUWAX_LCM_CHANNEL_POINTCLOUD",
                    "/lidar/points#sensor_msgs.PointCloud2",
                ),
                "sensor_msgs.PointCloud2",
            )?;
            let payload = schema::encode_point_cloud2_body(&message).map_err(|e| e.to_string())?;
            self.transport
                .publish(
                    RawLcmFrameBuilder::new(binding.0, binding.1)
                        .payload(payload)
                        .build(),
                )
                .await
                .map_err(|e| e.to_string())?;
        }
        self.router.publish_frame(MappingFrame {
            session_id,
            odom_pose: pose,
            points_body: points.iter().map(|p| [p.x, p.y, p.z]).collect(),
            ray_origin_body: [ray_origin.x, ray_origin.y, ray_origin.z],
            mapping_only: false,
            sensor_id: cloud.header.frame_id.clone(),
            registered_cloud: Some(cloud),
        });
        Ok(())
    }
    async fn publish_pose_bypass(&self, message: &schema::Odometry) -> Result<(), String> {
        let binding = binding(
            &env("NUWAX_LCM_CHANNEL_ODOM", "/odom#nav_msgs.Odometry"),
            "nav_msgs.Odometry",
        )?;
        let payload = schema::encode_odometry_body(message).map_err(|e| e.to_string())?;
        self.transport
            .publish(
                RawLcmFrameBuilder::new(binding.0, binding.1)
                    .payload(payload)
                    .build(),
            )
            .await
            .map_err(|e| e.to_string())
    }
}

fn take_pair(
    poses: &VecDeque<proto::Pose>,
    clouds: &mut VecDeque<(u64, u64, schema::PointCloud2)>,
    native_skew: Option<std::time::Duration>,
) -> Option<(u64, proto::Pose, schema::PointCloud2)> {
    while let Some((_, stamp, _)) = clouds.front() {
        let stamp = *stamp;
        let pose = poses
            .iter()
            .rev()
            .find(|pose| pose.timestamp_ns <= stamp)
            .cloned();
        if poses.is_empty()
            || (native_skew.is_some() && poses.back().is_none_or(|pose| pose.timestamp_ns < stamp))
        {
            return None;
        }
        let (session, _, cloud) = clouds.pop_front().unwrap();
        let Some(pose) = pose else {
            continue;
        };
        if native_skew.is_some_and(|skew| stamp - pose.timestamp_ns > skew.as_nanos() as u64) {
            continue;
        }
        return Some((session, pose, cloud));
    }
    None
}

fn capture_pose(
    poses: &VecDeque<proto::Pose>,
    timestamp_ns: u64,
) -> Result<Option<proto::Pose>, String> {
    if let Some(exact) = poses.iter().find(|pose| pose.timestamp_ns == timestamp_ns) {
        return Ok(Some(exact.clone()));
    }
    let Some((left, right)) = poses
        .iter()
        .zip(poses.iter().skip(1))
        .find(|(left, right)| {
            left.timestamp_ns < timestamp_ns && timestamp_ns < right.timestamp_ns
        })
    else {
        return Ok(None);
    };
    let left_iso = pose_isometry(left)?;
    let right_iso = pose_isometry(right)?;
    let ratio =
        (timestamp_ns - left.timestamp_ns) as f64 / (right.timestamp_ns - left.timestamp_ns) as f64;
    let translation = left_iso
        .translation
        .vector
        .lerp(&right_iso.translation.vector, ratio);
    let rotation = left_iso.rotation.slerp(&right_iso.rotation, ratio);
    Ok(Some(proto::Pose {
        position: Some(proto::Vec3 {
            x: translation.x,
            y: translation.y,
            z: translation.z,
        }),
        orientation: Some(proto::Quat {
            x: rotation.i,
            y: rotation.j,
            z: rotation.k,
            w: rotation.w,
        }),
        frame_id: left.frame_id.clone(),
        timestamp_ns,
    }))
}

#[cfg(test)]
#[path = "lio_tests.rs"]
mod tests;
