//! 停车证明和停车后的目标位置分别成立，才允许发布成功。
use super::*;

pub(super) struct ArrivalConfirmation {
    pub(super) proof: FenceProof,
    pub(super) confirmed_at: Instant,
    pub(super) resume_requested: bool,
    pub(super) execution_identity: Option<Arc<PathExecutionIdentity>>,
    pub(super) kind: ArrivalKind,
}

pub(super) enum ArrivalKind {
    Body,
    Ground {
        evidence: Arc<GroundArrivalEvidence>,
        source: Arc<SurfaceSnapshot>,
        retired: bool,
    },
}
impl ArrivalKind {
    pub(super) fn ground(evidence: Arc<GroundArrivalEvidence>) -> Self {
        let source = Arc::clone(&evidence.actual().source);
        Self::Ground { evidence, source, retired: false }
    }
}
impl ArrivalConfirmation {
    /// Call only after the original map setter accepts the publication. Every
    /// accepted ingress is observed, including changes later coalesced away.
    pub(super) fn observe_accepted_map(&mut self, map: &ExecutionMap) {
        let ArrivalKind::Ground { source, retired, .. } = &mut self.kind else { return; };
        let next = match map {
            ExecutionMap::Surface(next) => {
                *retired |= !nuwax_nav_pct::same_source_geometry(source, next)
                    || !surface_changes::same_query_semantics(source, next);
                next
            }
            ExecutionMap::SurfaceUpdate(update) => {
                let linked = update.change_basis.as_ref().is_some_and(|basis|
                    nuwax_nav_pct::same_source_geometry(source, basis));
                let unchanged = matches!(update.changes.as_ref(), surface_changes::SurfaceGeometryChanges::Empty)
                    && nuwax_nav_pct::same_source_geometry(source, &update.snapshot);
                *retired |= !linked || !unchanged
                    || !surface_changes::same_query_semantics(source, &update.snapshot);
                &update.snapshot
            }
            ExecutionMap::Planar(_) => { *retired = true; return; }
        };
        // Retirement is absorbing. The latest accepted source still supplies
        // freshness qualification; a later restore cannot revive old relation.
        *source = Arc::clone(next);
    }
}

impl<M: NavMotionPort, P: NavPlanner> NavScheduler<M, P> {
    pub(super) fn arrival_resume_requested(&self) -> bool {
        self.arrival_confirmation
            .as_ref()
            .is_some_and(|pending| pending.resume_requested)
    }

    pub(super) async fn confirm_stopped_arrival(
        &mut self,
        status: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        self.check_path_wait_timeout(status).await?;
        if self.arrival_confirmation.is_some() && !self.obstacle_source_available {
            return Err(self
                .fail_closed(
                    status,
                    RuntimeError::StaleInput("停车复核期间近期障碍来源已撤销".into()),
                )
                .await);
        }
        let Some(pending) = self.arrival_confirmation.as_ref() else {
            return Ok(());
        };
        if pending.resume_requested {
            return self.resume_after_arrival_drift(status).await;
        }
        let after_stop = self
            .pose_acquired_at
            .is_some_and(|time| time > pending.confirmed_at);
        if !after_stop || self.stale_input_reason(Instant::now()).is_some() {
            self.status.message = "已确认停稳，等待停车后采样的有效位姿复核目标".into();
            self.publish_status(status);
            return Ok(());
        }
        let pending = self.arrival_confirmation.as_ref().unwrap();
        let (Some(session), Some(pose)) = (&self.session, &self.latest_pose) else {
            return Err(self
                .fail_closed(
                    status,
                    RuntimeError::Planner("停车后目标复核缺少原目标或有效位姿".into()),
                )
                .await);
        };
        let assessment = match (&session.goal, &pending.kind) {
            (ExecutionGoal::Body(goal), ArrivalKind::Body) => {
                let actual = pose.pose.position;
                let target = goal.pose.position;
                let position_ok = if session.options.position_tolerance_m == 0.0 {
                    nuwax_nav_types_messages::same_finite_position(
                        [actual.x, actual.y, actual.z], [target.x, target.y, target.z])
                } else {
                    pose.pose.position.distance(goal.pose.position)
                        <= session.options.position_tolerance_m
                };
                let orientation_ok = !session.options.requires_orientation()
                    || if session.options.yaw_tolerance_rad == 0.0 {
                        nuwax_nav_types_messages::same_finite_yaw(
                            pose.pose.orientation.yaw(), goal.pose.orientation.yaw())
                    } else {
                        let yaw = pose.pose.orientation.yaw() - goal.pose.orientation.yaw();
                        yaw.sin().atan2(yaw.cos()).abs() <= session.options.yaw_tolerance_rad
                    };
                if position_ok && orientation_ok {
                    nuwax_nav_types_messages::GoalRegionAssessment::WithinRegion
                } else { nuwax_nav_types_messages::GoalRegionAssessment::OutsideRegion }
            }
            (ExecutionGoal::Ground(goal), ArrivalKind::Ground { evidence, source, retired })
                if goal == evidence.goal() => {
                let Some(context) = self.planner.refresh_execution_context(Twist2::ZERO) else {
                    self.status.message = "已确认停稳，等待与当前输入配对的实际地面观察".into();
                    self.publish_status(status);
                    return Ok(());
                };
                // A timestamp-only fresh sample deliberately keeps the worker's
                // geometry version. Match full pose/frame, excluding stamp only.
                if !replan_feedback::same_pose(&context.odometry, pose)
                    || !nuwax_nav_pct::same_source_geometry(&context.source, source)
                    || !surface_changes::same_query_semantics(&context.source, source) {
                    self.status.message = "已确认停稳，等待当前来源与完整位姿的地面复核".into();
                    self.publish_status(status);
                    return Ok(());
                }
                evidence.assess(&context, session.options,
                    nuwax_nav_types_messages::ArrivalBoundary::Closed, *retired)
            }
            _ => return Err(self.fail_closed(status,
                RuntimeError::Planner("停车复核的原目标类型或来源已失配".into())).await),
        };
        match assessment {
            nuwax_nav_types_messages::GoalRegionAssessment::WithinRegion => {
                return self.finish_terminal(status, TerminalOutcome::Succeeded, SchedulerPhase::Idle).await;
            }
            nuwax_nav_types_messages::GoalRegionAssessment::TargetLayerUnresolved => {
                return self.finish_terminal(status, TerminalOutcome::GoalIncomplete {
                    reason: GoalIncompleteReason::TargetLayerUnresolved, xy_reached: true,
                }, SchedulerPhase::Idle).await;
            }
            nuwax_nav_types_messages::GoalRegionAssessment::KnownLayerConflict => {
                return self.finish_terminal(status, TerminalOutcome::GoalIncomplete {
                    reason: GoalIncompleteReason::KnownLayerConflict, xy_reached: true,
                }, SchedulerPhase::Idle).await;
            }
            nuwax_nav_types_messages::GoalRegionAssessment::OutsideRegion => {}
        }
        self.arrival_confirmation.as_mut().unwrap().resume_requested = true;
        self.status.phase = SchedulerPhase::Acquiring;
        self.status.message = "停稳后位姿未满足原目标，重新核对原运动准入后接续".into();
        self.publish_status(status);
        Ok(())
    }

    async fn resume_after_arrival_drift(
        &mut self,
        status: &watch::Sender<SchedulerStatus>,
    ) -> Result<(), RuntimeError> {
        // 尚未重新取租约时保持原停车证明；来源失效不能恢复运动。
        if let Some(reason) = self.stale_input_reason(Instant::now()) {
            return Err(self
                .fail_closed(status, RuntimeError::StaleInput(reason))
                .await);
        }
        let Some(session) = self.session.clone() else {
            return Err(RuntimeError::Planner("停车接续缺少原目标".into()));
        };
        let operation_id = self.next_operation_id("arrival-resume")?;
        let grant = self
            .motion
            .acquire(&operation_id, &session.motion_admission)
            .await;
        let proof = match grant {
            Ok(proof)
                if proof.lease_id != 0
                    && proof.guard_generation == session.motion_admission.guard_generation =>
            {
                proof
            }
            Ok(_) => {
                self.lease_active = true;
                self.arrival_confirmation = None;
                return Err(self
                    .fail_closed(
                        status,
                        RuntimeError::Motion("停车接续取得的租约不属于原准入代次".into()),
                    )
                    .await);
            }
            Err(error) => {
                if error.lease_may_be_active {
                    self.lease_active = true;
                    self.arrival_confirmation = None;
                }
                return Err(self
                    .fail_closed(status, RuntimeError::Motion(error.message))
                    .await);
            }
        };
        self.lease_active = true;
        let previous = self
            .arrival_confirmation
            .take()
            .expect("接续仍拥有原停车状态");
        self.status.guard_generation = proof.guard_generation;
        self.status.motion_sink_ready = true;
        self.status.last_zero_ack_id = None;
        self.status.last_fence_monotonic_ns = 0;
        self.status.last_terminal = None;
        self.local_intent_sequence = 0;
        self.planner.set_goal_log_context(&session.session_id);
        let initial = match &previous.kind {
            ArrivalKind::Body => None,
            ArrivalKind::Ground { evidence, .. } => match evidence.initial_capture() {
                Ok(capture) => Some(capture),
                Err(error) => return Err(self.fail_closed(status, error).await),
            },
        };
        if let Err(error) = self.planner.configure_execution_goal(&session.goal, session.options, initial) {
            return Err(self.fail_closed(status, error).await);
        }
        self.planner.configure_boundary(&session.boundary);
        let accepted = if let (Some((path, yaws)), Some(identity)) =
            (&self.current_path, previous.execution_identity)
        {
            match self
                .planner
                .replace_typed_execution_path(path.clone(), yaws.clone(), session.goal)
            {
                Ok(()) => {
                    // 新的指令身份对象阻止停车前已经选出的意图跨租约复活。
                    self.current_execution_identity = Some(Arc::new(identity.as_ref().clone()));
                    true
                }
                Err(RuntimeError::PathRejected { .. } | RuntimeError::StaleInput(_)) => false,
                Err(error) => return Err(self.fail_closed(status, error).await),
            }
        } else {
            false
        };
        self.status.phase = if accepted {
            SchedulerPhase::Controlling
        } else {
            SchedulerPhase::AwaitingPath
        };
        self.status.message = if accepted {
            "停稳后越界，原目标及路线已按当前输入重新接纳".into()
        } else {
            "停稳后越界，原目标等待更高修订的可执行路线".into()
        };
        self.publish_status(status);
        Ok(())
    }
}
