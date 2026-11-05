//! 校正计算：直接调用既有回环优化或全局配准内核，不拥有会话状态。
use crate::{
    mapper::{pose_from_isometry, pose_isometry, Mapper, MappingFrame},
    proto,
};
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use nuwax_mapping::loop_closure::options::{PgoOptions, PGO_OPTIONS_ENV};
use nuwax_mapping::loop_closure::{
    CorrectionKeyframe, CorrectionTimeline, LoopClosureCoordinator, LoopClosurePipelineConfig,
    ProcessFrameResult,
};
use nuwax_mapping::relocalization::{
    evaluate_alignment, is_upright, relocation_pipeline_with_config, IcpConfig, IcpSolver,
    RelocalizationConfig, GRAVITY_TILT_MAX_DEG,
};
pub struct Corrector {
    mode: Mode,
    first_stamp: Option<u64>,
    sequence: u64,
}
enum Mode {
    Relocalization,
    Library(LoopClosureCoordinator),
    Native(Native),
}
struct Native {
    runtime: Option<tokio::runtime::Runtime>,
    owner: Option<nuwax_nav_pgo_bridge::OnlinePgoOwner>,
    ingress: Option<nuwax_nav_pgo_bridge::OnlinePgoIngress>,
    local_poses: std::collections::BTreeMap<u64, Isometry3<f64>>,
}
#[derive(Clone)]
pub(crate) struct PgoUpdate {
    pub latest: Isometry3<f64>,
    timeline: CorrectionTimeline,
    first_stamp: u64,
}
impl PgoUpdate {
    pub fn correction_at(&self, stamp: u64) -> Isometry3<f64> {
        self.timeline
            .correction_at(stamp.saturating_sub(self.first_stamp) as f64 / 1e9)
            .expect("优化轨迹已由数值内核校验")
    }
}
impl Corrector {
    pub fn new(backend: &str) -> Result<Self, String> {
        let mode = match backend {
            "none" | "relocalization" => Mode::Relocalization,
            "library_pgo" => {
                let base = if std::env::var("NUWAX_GEOMETRY_LIO_BACKEND").ok().as_deref()
                    == Some("pointlio")
                {
                    LoopClosurePipelineConfig::for_point_lio()
                } else {
                    LoopClosurePipelineConfig::default()
                };
                let raw = std::env::var(PGO_OPTIONS_ENV).unwrap_or_default();
                let config = PgoOptions::from_json(&raw)?.apply_to(base)?;
                Mode::Library(LoopClosureCoordinator::new(config).map_err(|e| e.to_string())?)
            }
            "native_pgo" => Mode::Native(Native {
                runtime: None,
                owner: None,
                ingress: None,
                local_poses: Default::default(),
            }),
            value => return Err(format!("未知校正后端：{value}")),
        };
        Ok(Self {
            mode,
            first_stamp: None,
            sequence: 0,
        })
    }
    pub(crate) fn process_frame(
        &mut self,
        frame: &MappingFrame,
        indices: &[u32],
        grid: Option<&nuwax_mapping::OccupancyGrid>,
        anchor: Isometry3<f64>,
    ) -> Result<Option<PgoUpdate>, String> {
        if matches!(self.mode, Mode::Relocalization) {
            return Ok(None);
        }
        let first = *self.first_stamp.get_or_insert(frame.odom_pose.timestamp_ns);
        self.sequence += 1;
        let time = frame.odom_pose.timestamp_ns.saturating_sub(first) as f64 / 1e9;
        let odom = pose_isometry(&frame.odom_pose);
        let points: Vec<_> = indices
            .iter()
            .map(|i| Point3::from(frame.points_body[*i as usize]))
            .collect();
        match &mut self.mode {
            Mode::Library(owner) => {
                let result = owner
                    .process_frame(time, odom, &points)
                    .map_err(|e| e.to_string())?;
                if !matches!(result, ProcessFrameResult::LoopClosed { .. }) {
                    return Ok(None);
                }
                let timeline = owner.corrections().clone();
                let correction = timeline.correction_at(time).map_err(|e| e.to_string())?;
                let frames = timeline
                    .keyframes()
                    .iter()
                    .map(|k| CorrectionKeyframe {
                        timestamp: k.timestamp,
                        local_pose: k.local_pose,
                        optimized_pose: anchor * k.optimized_pose,
                    })
                    .collect();
                let timeline = CorrectionTimeline::new(frames).map_err(|e| e.to_string())?;
                Ok(Some(PgoUpdate {
                    latest: anchor * correction,
                    timeline,
                    first_stamp: first,
                }))
            }
            Mode::Native(owner) => {
                let Some(grid) = grid else { return Ok(None) };
                owner.process(frame, &points, grid, self.sequence, anchor, first)
            }
            Mode::Relocalization => Ok(None),
        }
    }
    pub async fn relocalize(
        mapper: &Mapper,
        seed: Option<proto::PlanarSeed>,
    ) -> Result<proto::RelocalizationResult, tonic::Status> {
        if seed.is_some() {
            return Err(tonic::Status::invalid_argument("三维重定位不接受初值"));
        }
        let backend = mapper.correction_backend();
        if matches!(backend, "library_pgo" | "native_pgo") {
            return Err(tonic::Status::failed_precondition(
                "当前配置使用回环校正，不能执行重定位",
            ));
        }
        let frames = mapper
            .recent_frames()
            .await
            .map_err(tonic::Status::unavailable)?;
        let target = mapper
            .surface_map()
            .await
            .map_err(tonic::Status::unavailable)?;
        let initial = mapper.map_from_odom();
        let result = tokio::task::spawn_blocking(move || {
            let Some(target) = target else {
                return Ok((failed("已加载地图没有可匹配的点云", 0), None));
            };
            if frames.is_empty() {
                return Ok((failed("最近两秒没有可用雷达帧", 0), None));
            }
            let mut receiver = nuwax_map_chunks::Receiver::new(target.repository.clone());
            let mut target_points = Vec::new();
            let size = f64::from(target.manifest().voxel_size_m);
            for reference in &target.manifest().blocks {
                let block = receiver.block(reference)?;
                if (target_points.len() + block.len()) * std::mem::size_of::<Point3<f64>>()
                    > 120 * 1024 * 1024
                {
                    return Err("重定位目标点云超过原120MiB预算".into());
                }
                target_points.extend(
                    block
                        .keys()
                        .map(|key| nuwax_map_chunks::global_key(reference.key, *key))
                        .collect::<Result<Vec<_>, _>>()?
                        .into_iter()
                        .map(|key| {
                            Point3::new(
                                (f64::from(key[0]) + 0.5) * size,
                                (f64::from(key[1]) + 0.5) * size,
                                (f64::from(key[2]) + 0.5) * size,
                            )
                        }),
                );
            }
            let source: Vec<_> = frames
                .iter()
                .flat_map(|frame| {
                    let odom = pose_isometry(&frame.odom_pose);
                    frame
                        .points_body
                        .iter()
                        .map(move |p| odom * Point3::from(*p))
                })
                .collect();
            let config = RelocalizationConfig::default();
            let seeded: Vec<_> = source.iter().map(|point| initial * point).collect();
            let local = IcpSolver::new(IcpConfig {
                max_correspondence_distance: config.voxel_sizes.into_iter().fold(0., f64::max),
                ..config.icp
            })
            .align(&seeded, &target_points);
            let local_transform = local.transformation * initial;
            let score = evaluate_alignment(
                &source,
                &target_points,
                local_transform,
                config.fine_correspondence_distance,
            );
            let (transform, fitness, succeeded) = if local.mse.is_finite()
                && score.inlier_count >= config.min_final_inliers
                && score.fitness >= config.min_final_fitness
                && score.rmse.is_finite()
                && is_upright(&local_transform, GRAVITY_TILT_MAX_DEG)
            {
                (local_transform, score.rmse * score.rmse, true)
            } else {
                let global = relocation_pipeline_with_config(&source, &target_points, config);
                (
                    global.registration.transformation,
                    global.registration.mse,
                    global.registration.converged,
                )
            };
            let last = frames.last().unwrap();
            let result = proto::RelocalizationResult {
                succeeded,
                message: if succeeded {
                    "重定位成功".into()
                } else {
                    "重定位未匹配到已加载地图".into()
                },
                pose: succeeded.then(|| {
                    pose_from_isometry(
                        transform * pose_isometry(&last.odom_pose),
                        &target.manifest().domain.session,
                        last.odom_pose.timestamp_ns,
                    )
                }),
                fitness: if fitness.is_finite() { fitness } else { 0. },
                frames_used: frames.len() as u32,
                finished_at_ns: now_ns(),
            };
            Ok::<_, String>((result, succeeded.then_some(transform)))
        })
        .await
        .map_err(|e| tonic::Status::internal(e.to_string()))?
        .map_err(tonic::Status::unavailable)?;
        let (mut result, transform) = result;
        if let Some(transform) = transform {
            mapper
                .set_map_from_odom(transform)
                .await
                .map_err(tonic::Status::unavailable)?;
            if let Some(pose) = result.pose.as_mut() {
                pose.frame_id = mapper.map_frame_id().into();
            }
        }
        Ok(result)
    }
}
fn failed(message: &str, frames_used: u32) -> proto::RelocalizationResult {
    proto::RelocalizationResult {
        succeeded: false,
        message: message.into(),
        pose: None,
        fitness: 0.,
        frames_used,
        finished_at_ns: now_ns(),
    }
}
fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}
impl Native {
    fn process(
        &mut self,
        frame: &MappingFrame,
        points: &[Point3<f64>],
        grid: &nuwax_mapping::OccupancyGrid,
        sequence: u64,
        anchor: Isometry3<f64>,
        first: u64,
    ) -> Result<Option<PgoUpdate>, String> {
        use nuwax_codec::lcm::native_pgo as wire;
        if self.runtime.is_none() {
            self.runtime = Some(
                tokio::runtime::Builder::new_multi_thread()
                    .worker_threads(1)
                    .enable_all()
                    .build()
                    .map_err(|e| e.to_string())?,
            );
            let config = nuwax_nav_pgo_bridge::OnlinePgoConfig::new(
                std::env::var("NUWAX_LCM_URL")
                    .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=0".into()),
                frame.session_id as i64,
                1,
            );
            let (owner, ingress) = self
                .runtime
                .as_ref()
                .unwrap()
                .block_on(nuwax_nav_pgo_bridge::OnlinePgoOwner::start(config))
                .map_err(|e| e.to_string())?;
            self.owner = Some(owner);
            self.ingress = Some(ingress);
        }
        let pose = pose_isometry(&frame.odom_pose);
        self.local_poses.insert(frame.odom_pose.timestamp_ns, pose);
        let p = pose.translation.vector;
        let q = pose.rotation.quaternion();
        let nonce = u128::from(frame.session_id).to_be_bytes();
        let identity = wire::NativePgoRequestIdentity {
            backend: wire::NATIVE_PGO_BACKEND,
            runtime_generation: frame.session_id as i64,
            backend_generation: 1,
            request_sequence: 0,
            mapping_session_id: frame.session_id.to_string(),
            mapping_session_generation: 1,
            mapping_backend_kind: std::env::var("NUWAX_GEOMETRY_MAPPING_BACKEND")
                .unwrap_or_else(|_| "voxel".into()),
            mapping_backend_generation: 1,
            map_frame_generation: 1,
            mapping_commit_id: wire::NativePgoMappingCommitId {
                boot_nonce: nonce,
                sequence: sequence as i64,
            },
            occupancy_revision: sequence as i64,
            cloud_revision: sequence as i64,
            registered_cloud: wire::NativePgoRegisteredCloudProof {
                backend: match std::env::var("NUWAX_GEOMETRY_LIO_BACKEND").ok().as_deref() {
                    Some("pointlio") => 3,
                    Some("fastlio2") => 2,
                    _ => 1,
                },
                runtime_generation: frame.session_id as i64,
                source_sequence: sequence as i64,
                source_timestamp_ns: frame.odom_pose.timestamp_ns as i64,
                source_point_count: frame.points_body.len() as i64,
                point_count: points.len() as i64,
            },
            correlated_odom: wire::NativePgoCorrelatedOdomProof {
                source_sequence: sequence as i64,
                source_timestamp_ns: frame.odom_pose.timestamp_ns as i64,
                translation_m: [p.x, p.y, p.z],
                rotation_xyzw: [q.i, q.j, q.k, q.w],
            },
            committed_map: wire::NativePgoCommittedMapProof {
                source_timestamp_ns: (grid.ts * 1e9) as i64,
                width: grid.width as i64,
                height: grid.height as i64,
                resolution_m: grid.resolution,
                origin_x_m: grid.origin[0],
                origin_y_m: grid.origin[1],
            },
            source_timestamp_ns: frame.odom_pose.timestamp_ns as i64,
            map_frame_id: grid.frame_id.clone(),
            odom_frame_id: frame.odom_pose.frame_id.clone(),
            body_frame_id: std::env::var("NUWAX_BASE_FRAME").unwrap_or_else(|_| "base_link".into()),
            points_frame_id: std::env::var("NUWAX_BASE_FRAME")
                .unwrap_or_else(|_| "base_link".into()),
            correction_lease: wire::NativePgoCorrectionLeaseId {
                boot_nonce: nonce,
                generation: 1,
            },
        };
        let request = wire::NativePgoRequest {
            schema_version: wire::NATIVE_PGO_SCHEMA_VERSION_V3,
            identity,
            registered_body_coordinate_count: (points.len() * 3) as i32,
            registered_body_points_xyz: points.iter().flat_map(|p| [p.x, p.y, p.z]).collect(),
        };
        let response = self
            .runtime
            .as_ref()
            .unwrap()
            .block_on(self.ingress.as_mut().unwrap().submit(request))
            .map_err(|e| e.to_string())?;
        match response.disposition() {
            nuwax_nav_pgo_bridge::ValidatedOnlinePgoDisposition::Correction => {
                let result = response.result();
                let keyframes = result
                    .trajectory
                    .iter()
                    .map(|frame| {
                        let local = *self
                            .local_poses
                            .get(&(frame.source_timestamp_ns as u64))
                            .ok_or("PGO轨迹缺少对应原始位姿")?;
                        let q = frame.rotation_xyzw;
                        let optimized = Isometry3::from_parts(
                            Translation3::from(frame.translation_m),
                            UnitQuaternion::new_normalize(Quaternion::new(q[3], q[0], q[1], q[2])),
                        );
                        Ok(CorrectionKeyframe {
                            timestamp: (frame.source_timestamp_ns as u64).saturating_sub(first)
                                as f64
                                / 1e9,
                            local_pose: local,
                            optimized_pose: anchor * optimized,
                        })
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                let timeline = CorrectionTimeline::new(keyframes).map_err(|e| e.to_string())?;
                let latest = timeline
                    .correction_at(frame.odom_pose.timestamp_ns.saturating_sub(first) as f64 / 1e9)
                    .map_err(|e| e.to_string())?;
                Ok(Some(PgoUpdate {
                    latest,
                    timeline,
                    first_stamp: first,
                }))
            }
            nuwax_nav_pgo_bridge::ValidatedOnlinePgoDisposition::NoCandidate => Ok(None),
            _ => Err(format!(
                "原生PGO拒绝本帧：{}",
                response.result().rejection_reason
            )),
        }
    }
}
impl Drop for Native {
    fn drop(&mut self) {
        if let (Some(runtime), Some(owner)) = (&self.runtime, self.owner.take()) {
            let _ = runtime.block_on(owner.shutdown());
        }
    }
}
