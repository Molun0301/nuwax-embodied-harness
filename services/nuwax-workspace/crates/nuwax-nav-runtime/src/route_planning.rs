//! Planning-only state. This module has no controller or motion port.
#[path="recent_guide_overlay.rs"]
mod recent_guide_overlay;
use crate::planning_owner::Cancellation;
use crate::{SurfaceFollowerConfig, SurfaceSnapshot};
use nuwax_nav_scan::{BodyPose, LocalPlanError, ScanConfig};
use nuwax_nav_types_messages::{GoalOptions, Path3D, Point3, PoseStamped, Quaternion};
use std::sync::Arc;

/// Immutable global reference roots. Clipping/materialization runs only on the
/// planning owner; dense input memory scales with the supplied route, not a new cap.
pub(crate) struct ReferenceSource {
    pub route: Arc<Path3D>,
    pub cumulative: Arc<Vec<f64>>,
    pub goal: Option<PoseStamped>,
    pub options: GoalOptions,
    pub offset: f64,
    pub progress: f64,
}
impl ReferenceSource {
    /// 进度沿原全局窗口累计；实际当前位置的接头只供轨迹生成。
    fn nominal_reference(&self, materialized:&[[f64;3]]) -> Vec<[f64;3]> {
        let mut reference=materialized.to_vec();
        let i=self.cumulative.partition_point(|&value|value<self.progress).saturating_sub(1);
        if let (Some(first),Some(a))=(reference.first_mut(),self.route.poses.get(i).map(|p|p.pose.position)) {
            let p=if let Some(b)=self.route.poses.get(i+1).map(|p|p.pose.position) {
                let length=self.cumulative[i+1]-self.cumulative[i];
                let t=((self.progress-self.cumulative[i])/length.max(1e-12)).clamp(0.0,1.0);
                Point3::new(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t)
            }else{a};
            *first=[p.x,p.y,p.z-self.offset];
        }
        reference
    }
    pub fn materialize(
        &self,
        current: &PoseStamped,
        horizon: f64,
    ) -> Result<(Vec<[f64; 3]>, f64, bool, bool), String> {
        let route = &self.route;
        let offset = self.offset;
        let mut result = vec![[
            current.pose.position.x,
            current.pose.position.y,
            current.pose.position.z - offset,
        ]];
        let end = (self.progress + horizon).min(*self.cumulative.last().ok_or("全局路线为空")?);
        let mut direction: Option<[f64; 2]> = None;
        let mut global_straight = true;
        for (i, pair) in route.poses.windows(2).enumerate().skip(
            self.cumulative
                .partition_point(|&v| v < self.progress)
                .saturating_sub(1),
        ) {
            if nuwax_nav_pct::planning_cancelled() {return Err("reference planning superseded".into());}
            if self.cumulative[i + 1] <= self.progress + 0.02 {
                continue;
            }
            let p = pair[1].pose.position;
            // 路段形状由原全局路线决定，实际机体的横向偏差不能把直线变成弯道。
            let delta = [p.x - pair[0].pose.position.x, p.y - pair[0].pose.position.y];
            let length = delta[0].hypot(delta[1]);
            if length > 1e-9 {
                let next = delta.map(|v| v / length);
                if let Some(first) = direction {
                    global_straight &= first[0] * next[0] + first[1] * next[1] > 0.0
                        && (first[0] * next[1] - first[1] * next[0]).abs() < 1e-9;
                } else {
                    direction = Some(next);
                }
            }
            if self.cumulative[i + 1] <= end {
                result.push([p.x, p.y, p.z - offset]);
            } else {
                let a = pair[0].pose.position;
                let length = self.cumulative[i + 1] - self.cumulative[i];
                let t = ((end - self.cumulative[i]) / length.max(1e-9)).clamp(0.0, 1.0);
                result.push([
                    a.x + (p.x - a.x) * t,
                    a.y + (p.y - a.y) * t,
                    a.z + (p.z - a.z) * t - offset,
                ]);
                break;
            }
        }
        let at_end = end >= *self.cumulative.last().unwrap() - 1e-6;
        if at_end {
            if let Some(goal) = &self.goal {
                if current.pose.position.distance(goal.pose.position)
                    >= self.options.position_tolerance_m
                {
                    // 层析末点是名义地面参考，实际站姿仍须进入用户三维到达球。
                    // 保留指向原目标的末段，允许投影在真正入球时结束。
                    let target = [
                        goal.pose.position.x,
                        goal.pose.position.y,
                        goal.pose.position.z - offset,
                    ];
                    if result.last() != Some(&target) {
                        result.push(target);
                    }
                }
            }
        }
        Ok((result, end, at_end, global_straight))
    }
}
pub(crate) struct PlanRequest {
    pub source: Arc<SurfaceSnapshot>,
    pub pose: PoseStamped,
    pub boundary: Arc<Vec<[f64; 2]>>,
    pub reference: Vec<[f64; 3]>,
    pub reference_source: Option<ReferenceSource>,
    pub final_yaw: Option<f64>,
    pub arrival: Option<([f64; 3], f64)>,
    pub end: f64,
    pub progress: f64,
    pub generation: u64,
    pub straight: bool,
    pub detour: bool,
    pub scan: ScanConfig,
}
impl PlanRequest {
    pub fn materialize_reference(&mut self) -> Result<(), LocalPlanError> {
        // 保留不可变原路线，供有限候选比较真实覆盖进度。
        if let Some(source) = self.reference_source.as_ref() {
            let (reference, end, at_end, straight) =
                source.materialize(&self.pose, self.scan.horizon_m)?;
            self.reference = reference;
            self.end = end;
            self.straight = straight;
            self.final_yaw = if at_end && source.options.requires_orientation() {
                source.goal.as_ref().map(|g| g.pose.orientation.yaw())
            } else {
                None
            };
            self.arrival = if at_end {
                source
                    .goal
                    .as_ref()
                    .map(|g| (body(g).0, source.options.position_tolerance_m))
            } else {
                None
            };
        }
        Ok(())
    }
}
pub(crate) struct Candidate {
    pub partial: bool,
    pub path: Path3D,
    pub elapsed_ms: f64,
}
pub(crate) struct PlanResult {
    pub request: PlanRequest,
    pub outcome: Result<Candidate, Vec<LocalPlanError>>,
}
struct FailedLocalPlan {
    coverage_context: [u64;2],
    source: Arc<SurfaceSnapshot>,
    tomogram: Arc<nuwax_nav_pct::Tomogram>,
    boundary: Arc<Vec<[f64; 2]>>,
    start: BodyPose,
    reference: Vec<[f64; 3]>,
    final_yaw: Option<f64>,
    arrival: Option<([f64; 3], f64)>,
    failure: LocalPlanError,
    detour: bool,
    straight: bool,
}
pub(crate) fn body(p: &PoseStamped) -> BodyPose {
    let q = p.pose.orientation;
    (
        [p.pose.position.x, p.pose.position.y, p.pose.position.z],
        [q.x, q.y, q.z, q.w],
    )
}
pub(crate) fn exact_points<const N: usize>(a: &[[f64; N]], b: &[[f64; N]]) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(a, b)| a.map(f64::to_bits) == b.map(f64::to_bits))
}
pub(crate) fn pose_bits(p: BodyPose) -> ([u64; 3], [u64; 4]) {
    (p.0.map(f64::to_bits), p.1.map(f64::to_bits))
}
const LOCAL_MAP_REUSE_DISTANCE_M: f64 = 0.25;
fn local_map_radius(horizon: f64, source: &SurfaceSnapshot) -> f64 {
    let (resolution, _) = source.navigation_source_columns(None);
    let halo = source.body_envelope().map_or(f64::from(source.config().wall_clearance_m), |body| body.radius())
        + resolution * 0.5;
    // Reuse may shift the search center; keep the entire search disk and physical
    // query halo inside the materialized source after that shift.
    (horizon + 1.5).max(horizon * 1.5 + LOCAL_MAP_REUSE_DISTANCE_M + halo)
}
pub(crate) struct PlanKernel {
    config: SurfaceFollowerConfig,
    local_map: Option<(Arc<SurfaceSnapshot>, Point3, Arc<nuwax_nav_pct::Tomogram>)>,
    failed: Vec<FailedLocalPlan>,
    epoch: Option<u64>,
    #[cfg(test)]
    pub hook: Option<Box<dyn FnMut(&PlanRequest) + Send>>,
}
impl PlanKernel {
    pub fn new(config: SurfaceFollowerConfig) -> Self {
        Self {
            config,
            local_map: None,
            failed: Vec::new(),
            epoch: None,
            #[cfg(test)]
            hook: None,
        }
    }
    pub fn compute(&mut self, mut request: PlanRequest, cancel: &Cancellation) -> PlanResult {
        #[cfg(test)]
        if let Some(hook) = &mut self.hook {
            hook(&request);
        }
        if self.epoch != Some(cancel.expected) {
            self.failed.clear();
            self.epoch = Some(cancel.expected);
        }
        let token = nuwax_nav_pct::PlanningCancellation::new(cancel.epoch.clone(), cancel.expected);
        let outcome = nuwax_nav_pct::with_planning_cancellation(&token, || {
            if cancel.cancelled() {return Err(vec!["planning superseded".into()]);}
            request.scan.reference_corridor_m = self.config.max_path_deviation_m;
            request.scan.horizon_m = request
                .scan
                .horizon_m
                .max(
                    request
                        .source
                        .body_envelope()
                        .map_or(0.0, |b| b.radius() * 2.0),
                )
                .max(
                    self.config
                        .kinematics()
                        .map_or(0.0, |k| k.minimum_turning_radius_m * 2.0),
                )
                .max(self.config.lookahead_m * 2.0);
            request.materialize_reference().map_err(|e| vec![e])?;
            self.compute_inner(&request, cancel)
        });
        PlanResult { request, outcome }
    }
    fn progress_minimum_distance(&self, request:&PlanRequest) -> f64 {
        let tolerance=request.reference_source.as_ref().map_or_else(
            ||request.arrival.map_or(self.config.controller.goal_tolerance,|(_,tolerance)|tolerance),
            |source|source.options.position_tolerance_m);
        tolerance.max(request.source.body_envelope().map_or(0.0,|model|model.radius()))
            +f64::from(request.source.source_voxel_size_m())*0.5
    }
    fn compute_inner(
        &mut self,
        r: &PlanRequest,
        cancel: &Cancellation,
    ) -> Result<Candidate, Vec<LocalPlanError>> {
        // Fork on this CPU owner. No cache permission travels back to control.
        let map = r.source.fork_for_planning();
        let prepare = || -> Result<(Arc<nuwax_nav_pct::Tomogram>, Point3), LocalPlanError> {
            if let Some((_, center, local)) =
                self.local_map.as_ref().filter(|(source, center, _)| {
                    nuwax_nav_pct::same_source_geometry(source, &r.source)
                        && center.distance(r.pose.pose.position) < LOCAL_MAP_REUSE_DISTANCE_M
                })
            {
                return Ok((local.clone(), *center));
            }
            Ok((
                Arc::new(nuwax_nav_pct::Tomogram::local_from_surface(
                    &map,
                    [r.pose.pose.position.x, r.pose.pose.position.y],
                    local_map_radius(r.scan.horizon_m, &map),
                )?),
                r.pose.pose.position,
            ))
        };
        let (tomogram, center) = prepare().map_err(|e| vec![e])?;
        self.local_map = Some((r.source.clone(), center, tomogram.clone()));
        let kinematics = self
            .config
            .kinematics()
            .ok_or_else(|| vec!["近场执行缺少运动能力".into()])?;
        let costs = self.config.pct.local_costs();
        // 首次规划也读取本次近期源；静态层析缓存不包含这些障碍。
        // 范围覆盖完整绕行搜索圆及机体外包，避免只导出原直线附近的回波。
        let recent_guide = if let Some(model) = map.body_envelope() {
            let center = [r.pose.pose.position.x, r.pose.pose.position.y];
            let radius = r.scan.horizon_m * 1.5 + model.radius();
            let bounds = [center.map(|v|v-radius), center.map(|v|v+radius)];
            let hints = map.recent_obstacle_hints(bounds, body(&r.pose))
                .map_err(|e|vec![LocalPlanError::from(e.to_string())])?;
            Some(recent_guide_overlay::RecentGuideOverlay::new(hints, tomogram.resolution, f64::from(map.source_voxel_size_m()), model)
                .map_err(|e|vec![LocalPlanError::from(e)])?)
        } else { None };
        let nominal_reference=self.config.motion_limits.as_ref().map(|_|r.reference_source.as_ref()
            .map_or_else(||r.reference.clone(),|source|source.nominal_reference(&r.reference)));
        let progress_policy=nominal_reference.as_deref().map(|reference|nuwax_nav_scan::ProgressPrefixPolicy {
            reference,
            minimum_distance_m:self.progress_minimum_distance(r),
        });
        let mut failures = Vec::new();
        for detour in if r.detour {
            &[true][..]
        } else {
            &[false, true][..]
        } {
            if cancel.cancelled() {
                return Err(vec![LocalPlanError { body_witness: None,
                    kind: nuwax_nav_scan::LocalPlanFailureKind::Cancelled,
                    closest_pose: None,
                    blocked: None,
                    reason: "planning superseded".into(),
                }]);
            }
            let mut reference = r.reference.clone();
            if *detour && reference.len() > 1 {
                if let Ok(route) = tomogram.plan_bounded(
                    reference[0],
                    *reference.last().unwrap(),
                    tomogram.resolution,
                    &r.boundary,
                    None,
                    Some(r.scan.horizon_m * 1.5),
                    &costs,
                ) {
                    reference = route.points;
                }
            }
            let mut follow_reference = *detour;
            if let Some(overlay) = recent_guide.as_ref().filter(|overlay|
                !overlay.is_empty() && reference.windows(2).any(|p|!overlay.accepts(p[0],p[1]))) {
                match tomogram.plan_bounded_with_segment_filter(
                    reference[0], *reference.last().unwrap(), tomogram.resolution,
                    &r.boundary, None, Some(r.scan.horizon_m*1.5), &costs,
                    &|a,b|overlay.accepts(a,b),
                ) {
                    Ok(route) => {reference = route.points; follow_reference = true;}
                    Err(reason) => {
                        // 外接圆比完整姿态保守，窄门仍交给原机体扫掠判断。
                        // 引导失败不等于真实不可通行；非法提示输入则在上方明确返回。
                        tracing::debug!(target:"navigation.scan",%reason,
                            "recent guide unavailable; retaining exact Scan fallback");
                    }
                }
            }
            let start = body(&r.pose);
            let straight = r.straight && !follow_reference;
            let cached = self.failed.iter().find(|f| {
                Arc::ptr_eq(&f.tomogram, &tomogram)
                    && f.coverage_context==[r.progress.to_bits(),r.end.to_bits()]
                    && f.source.same_body_query_view(&r.source)
                    && exact_points(&f.boundary, &r.boundary)
                    && pose_bits(f.start) == pose_bits(start)
                    && exact_points(&f.reference, &reference)
                    && f.final_yaw.map(f64::to_bits) == r.final_yaw.map(f64::to_bits)
                    && f.arrival.map(|(p, t)| (p.map(f64::to_bits), t.to_bits()))
                        == r.arrival.map(|(p, t)| (p.map(f64::to_bits), t.to_bits()))
                    && f.detour == follow_reference
                    && f.straight == straight
            });
            let result = if let Some(f) = cached {
                Err(f.failure.clone())
            } else {
                let result = nuwax_nav_scan::plan_with_progress(
                    &map,
                    &tomogram,
                    start,
                    &reference,
                    &kinematics,
                    &r.scan,
                    r.final_yaw,
                    r.arrival,
                    &r.boundary,
                    follow_reference,
                    straight,
                    progress_policy,
                );
                if let Err(failure) = &result {
                    if failure.kind != nuwax_nav_scan::LocalPlanFailureKind::Cancelled {
                        if self.failed.len() == 2 {
                            self.failed.remove(0);
                        }
                        self.failed.push(FailedLocalPlan {
                            coverage_context:[r.progress.to_bits(),r.end.to_bits()],
                            source: r.source.clone(),
                            tomogram: tomogram.clone(),
                            boundary: r.boundary.clone(),
                            start,
                            reference,
                            final_yaw: r.final_yaw,
                            arrival: r.arrival,
                            failure: failure.clone(),
                            detour: follow_reference,
                            straight,
                        });
                    }
                }
                result
            };
            match result {
                Ok(trajectory) => {
                    let path = Path3D {
                        poses: trajectory
                            .poses
                            .into_iter()
                            .map(|(p, q)| {
                                let mut pose = r.pose.clone();
                                pose.pose.position = Point3::new(p[0], p[1], p[2]);
                                pose.pose.orientation = Quaternion {
                                    x: q[0],
                                    y: q[1],
                                    z: q[2],
                                    w: q[3],
                                };
                                pose
                            })
                            .collect(),
                    };
                    return Ok(Candidate { partial: trajectory.partial,
                        path,
                        elapsed_ms: trajectory.elapsed_ms,
                    });
                }
                Err(failure) => failures.push(failure),
            }
        }
        Err(failures)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    include!("route_progress_policy_tests.rs");
    use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::{MotionKinematics, MotionModel, NavFrameId, Pose3, Timestamp};
    use std::sync::atomic::AtomicU64;

    fn fixture() -> (PlanKernel, PlanRequest, Cancellation) {
        let mut config = SurfaceFollowerConfig::new(0.4);
        config.basic_kinematics = Some(MotionKinematics {
            model: MotionModel::Differential,
            max_forward_mps: 0.5,
            max_reverse_mps: 0.0,
            max_lateral_mps: 0.0,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.0,
            can_rotate_in_place: true,
        });
        let source = crate::surface_follower::test_snapshot(0.0, false)
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                z_bounds_m: [-0.05, 0.2],
                body_to_support_z_m: 0.4,
                max_step_up_m: 0.2,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap();
        let mut observations = SurfaceObservations::new(source.source_voxel_size_m(), 32).unwrap();
        let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
        for (x, y, z) in source.cell_keys() {
            let key = [x, y, z];
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 1_000_000_000,
                        hits: 2,
                    },
                );
        }
        for (key, block) in blocks {
            observations.replace_block(key, block).unwrap();
        }
        let source = Arc::new(source.with_observations(Some(Arc::new(observations))));
        let pose = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(1, 0),
            pose: Pose3 {
                position: Point3::new(0.5, 1.0, 0.5),
                orientation: Quaternion::IDENTITY,
            },
        };
        let request = PlanRequest {
            source,
            pose,
            boundary: Arc::new(Vec::new()),
            reference: vec![[0.5, 1.0, 0.1], [3.0, 1.0, 0.1]],
            reference_source: None,
            final_yaw: None,
            arrival: Some(([3.0, 1.0, 0.5], 0.35)),
            end: 2.5,
            progress: 0.0,
            generation: 1,
            straight: true,
            detour: false,
            scan: config.scan.clone(),
        };
        let cancel = Cancellation {
            epoch: Arc::new(AtomicU64::new(1)),
            expected: 1,
        };
        (PlanKernel::new(config), request, cancel)
    }
    #[test]
    fn local_materialization_covers_detours_after_cached_center_drift() {
        let (_, request, _) = fixture();
        let horizon = 5.0;
        let halo = request.source.body_envelope().unwrap().radius();
        let radius = local_map_radius(horizon, &request.source);
        assert!(radius > horizon + 1.5, "the old crop was smaller than the detour disk");
        for drift in [0.0, 0.1, LOCAL_MAP_REUSE_DISTANCE_M - 1e-9] {
            assert!(drift + horizon * 1.5 + halo <= radius);
        }
    }
    fn seed_exact_failures(kernel: &mut PlanKernel, request: &PlanRequest) {
        let tomogram = Arc::new(
            nuwax_nav_pct::Tomogram::local_from_surface(
                &request.source,
                [request.pose.pose.position.x, request.pose.pose.position.y],
                request.scan.horizon_m + 1.5,
            )
            .unwrap(),
        );
        kernel.local_map = Some((
            request.source.clone(),
            request.pose.pose.position,
            tomogram.clone(),
        ));
        kernel.failed.clear();
        for detour in [false, true] {
            let mut reference = request.reference.clone();
            if detour {
                if let Ok(route) = tomogram.plan_bounded(
                    reference[0],
                    *reference.last().unwrap(),
                    tomogram.resolution,
                    &request.boundary,
                    None,
                    Some(request.scan.horizon_m * 1.5),
                    &kernel.config.pct.local_costs(),
                ) {
                    reference = route.points;
                }
            }
            kernel.failed.push(FailedLocalPlan {
                coverage_context:[request.progress.to_bits(),request.end.to_bits()],
                source: request.source.clone(),
                tomogram: tomogram.clone(),
                boundary: request.boundary.clone(),
                start: body(&request.pose),
                reference,
                final_yaw: request.final_yaw,
                arrival: request.arrival,
                failure: LocalPlanError { body_witness: None,
                    kind: nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted,
                    closest_pose: Some(body(&request.pose)),
                    blocked: None,
                    reason: "exact cached failure".into(),
                },
                detour,
                straight: request.straight && !detour,
            });
        }
    }
    #[test]
    fn first_plan_routes_around_recent_wall_before_exhausting_scan() {
        let (mut kernel, mut request, cancel) = fixture();
        request.source = Arc::new(request.source.with_recent_obstacle_points(
            1, 1_000_000_000, 0, &[],
            (0..5).map(|i| (i, [0.73, 0.8 + f64::from(i) * 0.1, 0.5])),
        ).unwrap());
        request.final_yaw = Some(0.0);
        let began = std::time::Instant::now();
        let result = kernel.compute_inner(&request, &cancel).unwrap();
        let elapsed_ms = began.elapsed().as_secs_f64() * 1000.0;
        let length_m: f64 = result.path.poses.windows(2)
            .map(|p|p[0].pose.position.distance(p[1].pose.position)).sum();
        let turns: Vec<_> = result.path.poses.windows(2).map(|p|
            nuwax_nav_astar::controllers::angle_diff(p[1].pose.orientation.yaw(),p[0].pose.orientation.yaw()).abs()).collect();
        eprintln!("FIRST_RECENT_PLAN elapsed_ms={elapsed_ms} scan_ms={} failed_attempts={} poses={} length_m={length_m} turn_rad={} max_turn_rad={}",
            result.elapsed_ms, kernel.failed.len(), result.path.poses.len(),
            turns.iter().sum::<f64>(), turns.iter().copied().fold(0.0,f64::max));
        assert!(kernel.failed.is_empty(), "首次规划须先避开近期墙，不能消耗一次失败搜索后才绕行");
        assert!(result.path.poses.iter().any(|p| (p.pose.position.y - 1.0).abs() > 0.1));
        let end = result.path.poses.last().unwrap();
        assert!(end.pose.position.distance(Point3::new(3.0, 1.0, 0.5)) < 0.35);
        for pair in result.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]), body(&pair[1]), &[]).unwrap();
        }
    }
    #[test]
    fn recent_detour_overlay_reuses_static_tomogram_and_clears_without_rebuild() {
        let (mut kernel,mut request,cancel)=fixture();
        kernel.compute_inner(&request,&cancel).unwrap_or_else(|e|panic!("baseline: {e:?}"));
        let cached=kernel.local_map.as_ref().unwrap().2.clone();
        let original=request.source.clone();
        request.source=Arc::new(original.with_recent_obstacle_points(1,1_000_000_000,0,&[],
            (0..5).map(|i|(i,[0.73,0.8+f64::from(i)*0.1,0.5]))).unwrap());
        request.detour=true;request.final_yaw=Some(0.0);
        let detour=kernel.compute_inner(&request,&cancel).unwrap_or_else(|e|panic!("detour: {e:?}"));
        assert!(Arc::ptr_eq(&cached,&kernel.local_map.as_ref().unwrap().2));
        assert!(nuwax_nav_pct::same_source_geometry(&original,&request.source));
        assert!(detour.path.poses.iter().any(|p|(p.pose.position.y-1.0).abs()>0.1));
        for pair in detour.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]),body(&pair[1]),&[]).unwrap();
        }
        request.source=Arc::new(request.source.with_recent_obstacle_points(2,2_000_000_000,0,&[],std::iter::empty()).unwrap());
        let clear=kernel.compute_inner(&request,&cancel).unwrap_or_else(|e|panic!("clear: {e:?}"));
        assert!(Arc::ptr_eq(&cached,&kernel.local_map.as_ref().unwrap().2));
        assert!(clear.path.poses.iter().all(|p|(p.pose.position.y-1.0).abs()<1e-6));
    }
    #[test]
    fn recent_radius_guide_failure_preserves_pose_feasible_narrow_door() {
        let (mut kernel,mut request,cancel)=fixture();
        let mut model=request.source.body_envelope().unwrap().clone();
        model.footprint_xy_m=vec![[-0.3,-0.1],[0.3,-0.1],[0.3,0.1],[-0.3,0.1]];
        request.source=Arc::new(request.source.with_body_envelope(model).unwrap()
            .with_recent_obstacle_points(1,1_000_000_000,0,&[],
                [(0,[1.25,0.75,0.5]),(1,[1.25,1.25,0.5])]).unwrap());
        request.boundary=Arc::new(vec![[0.0,0.82],[4.0,0.82],[4.0,1.18],[0.0,1.18]]);
        request.detour=true;request.final_yaw=Some(0.0);
        let map=&request.source;
        let start=body(&request.pose);let end=([3.0,1.0,0.5],start.1);
        map.validate_body_motion(start,end,&request.boundary).expect("real long body fits the narrow door");
        let tomo=nuwax_nav_pct::Tomogram::local_from_surface(map,[0.5,1.0],6.5).unwrap();
        let hints=map.recent_obstacle_hints([[-6.5,-6.5],[6.5,6.5]],start).unwrap();
        let overlay=recent_guide_overlay::RecentGuideOverlay::new(hints,tomo.resolution,f64::from(map.source_voxel_size_m()),map.body_envelope().unwrap()).unwrap();
        assert!(!overlay.is_empty());
        assert!(tomo.plan_bounded_with_segment_filter(request.reference[0],*request.reference.last().unwrap(),
            tomo.resolution,&request.boundary,None,Some(7.5),&[],&|a,b|overlay.accepts(a,b)).is_err(),
            "radius-only guide must be demonstrably more conservative than this pose-feasible door");
        let result=kernel.compute_inner(&request,&cancel).unwrap_or_else(|e|panic!("exact fallback: {e:?}"));
        assert!(result.path.poses.iter().all(|p|(p.pose.position.y-1.0).abs()<1e-6));
        for pair in result.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]),body(&pair[1]),&request.boundary).unwrap();
        }
    }
    #[test]
    fn cache_key_preserves_signed_zero_and_quaternion_antipodes() {
        let pose = ([0.0, 1.0, 2.0], [0.0, 0.0, 0.0, 1.0]);
        let mut changed = pose;
        changed.0[0] = -0.0;
        assert_ne!(pose_bits(pose), pose_bits(changed));
        changed = pose;
        changed.1 = changed.1.map(|x| -x);
        assert_ne!(pose_bits(pose), pose_bits(changed));
        assert!(!exact_points(&[[0.0, 1.0]], &[[-0.0, 1.0]]));
    }
    #[test]
    fn exact_failed_attempt_cache_requires_owner_pose_boundary_source_and_semantics() {
        for change in 0..12 {
            let (mut kernel, mut request, cancel) = fixture();
            seed_exact_failures(&mut kernel, &request);
            let failures = match kernel.compute_inner(&request, &cancel) {
                Err(failures) => failures,
                Ok(_) => panic!("identical requests must reuse exact failures"),
            };
            assert_eq!(failures.len(), 2);
            assert!(failures
                .iter()
                .all(|failure| failure.reason == "exact cached failure"));
            match change {
                0 => {
                    let (source, center, _) = kernel.local_map.take().unwrap();
                    let owner = Arc::new(
                        nuwax_nav_pct::Tomogram::local_from_surface(
                            &source,
                            [center.x, center.y],
                            request.scan.horizon_m + 1.5,
                        )
                        .unwrap(),
                    );
                    kernel.local_map = Some((source, center, owner));
                }
                1 => request.pose.pose.position.x += 0.01,
                2 => {
                    request.boundary =
                        Arc::new(vec![[0.0, 0.0], [6.0, 0.0], [6.0, 3.0], [0.0, 3.0]])
                }
                3 => {
                    // A metadata-only source has the same Tomogram geometry, but cannot
                    // inherit a failure qualified against a different body query view.
                    let observations =
                        Arc::new(request.source.observations().unwrap().as_ref().clone());
                    let changed = Arc::new(request.source.with_observations(Some(observations)));
                    assert!(nuwax_nav_pct::same_source_geometry(
                        &request.source,
                        &changed
                    ));
                    assert!(!request.source.same_body_query_view(&changed));
                    request.source = changed;
                }
                4 => request.arrival.as_mut().unwrap().1 *= 0.9,
                5 => request.arrival.as_mut().unwrap().0[0] += 0.1,
                6 => request.pose.pose.orientation.x = -0.0,
                7 => request.final_yaw = Some(0.0),
                8 => {
                    // Both attempts have their own reference after the optional PCT pass.
                    for cached in &mut kernel.failed {
                        cached.reference[0][0] += 0.01;
                    }
                }
                9 => {
                    for cached in &mut kernel.failed {
                        cached.detour = !cached.detour;
                    }
                }
                10 => {
                    for cached in &mut kernel.failed {
                        cached.straight = !cached.straight;
                    }
                }
                _ => {
                    let source = crate::surface_follower::test_snapshot(0.0, false)
                        .with_body_envelope(request.source.body_envelope().unwrap().clone())
                        .unwrap();
                    request.source = Arc::new(source);
                }
            }
            if let Err(failures) = kernel.compute_inner(&request, &cancel) {
                assert!(
                    failures
                        .iter()
                        .all(|failure| failure.reason != "exact cached failure"),
                    "dependency change {change} reused a stale negative"
                );
            }
            assert!(kernel.failed.len() <= 2);
        }
    }
    #[test]
    fn lifecycle_epoch_retires_exact_failures() {
        let (mut kernel, request, mut cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        kernel.epoch = Some(cancel.expected);
        cancel.expected += 1;
        cancel
            .epoch
            .store(cancel.expected, std::sync::atomic::Ordering::Release);
        let result = kernel.compute(request, &cancel);
        if let Err(failures) = result.outcome {
            assert!(failures
                .iter()
                .all(|failure| failure.reason != "exact cached failure"));
        }
        assert_eq!(kernel.epoch, Some(cancel.expected));
        assert!(kernel
            .failed
            .iter()
            .all(|failure| failure.failure.reason != "exact cached failure"));
    }
    #[test]
    fn local_tomogram_reuse_keeps_its_original_construction_center() {
        for z_only in [false, true] {
            let (mut kernel, mut request, cancel) = fixture();
            let _ = kernel.compute_inner(&request, &cancel);
            let (_, original_center, original_tomogram) =
                kernel.local_map.as_ref().unwrap().clone();
            for distance in [0.1, 0.2] {
                request.pose.pose.position = original_center;
                if z_only {
                    request.pose.pose.position.z += distance;
                } else {
                    request.pose.pose.position.x += distance;
                }
                let _ = kernel.compute_inner(&request, &cancel);
                let (_, center, tomogram) = kernel.local_map.as_ref().unwrap();
                assert_eq!(
                    *center, original_center,
                    "cache origin drifted after a sub-threshold move"
                );
                assert!(Arc::ptr_eq(tomogram, &original_tomogram));
            }
            request.pose.pose.position = original_center;
            if z_only {
                request.pose.pose.position.z += 0.3;
            } else {
                request.pose.pose.position.x += 0.3;
            }
            let _ = kernel.compute_inner(&request, &cancel);
            let (_, center, tomogram) = kernel.local_map.as_ref().unwrap();
            assert_eq!(*center, request.pose.pose.position);
            assert!(
                !Arc::ptr_eq(tomogram, &original_tomogram),
                "cumulative displacement must reconstruct the local Tomogram"
            );
        }
    }
}
