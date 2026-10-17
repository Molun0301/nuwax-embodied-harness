//! 停车证明和停车后的目标位置分别成立，才允许发布成功。
use super::*;

pub(super) struct ArrivalConfirmation {
    pub(super) proof: FenceProof,
    pub(super) confirmed_at: Instant,
    pub(super) resume_requested: bool,
    pub(super) execution_identity: Option<Arc<PathExecutionIdentity>>,
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
        let (Some(session), Some(pose)) = (&self.session, &self.latest_pose) else {
            return Err(self
                .fail_closed(
                    status,
                    RuntimeError::Planner("停车后目标复核缺少原目标或有效位姿".into()),
                )
                .await);
        };
        let position_ok = pose.pose.position.distance(session.goal_pose.pose.position)
            <= session.options.position_tolerance_m;
        let yaw = pose.pose.orientation.yaw() - session.goal_pose.pose.orientation.yaw();
        let orientation_ok = !session.options.requires_orientation()
            || yaw.sin().atan2(yaw.cos()).abs() <= session.options.yaw_tolerance_rad;
        if position_ok && orientation_ok {
            return self
                .finish_terminal(status, TerminalOutcome::Succeeded, SchedulerPhase::Idle)
                .await;
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
        if let Err(error) = self
            .planner
            .configure_goal(&session.goal_pose, session.options)
        {
            return Err(self.fail_closed(status, error).await);
        }
        self.planner.configure_boundary(&session.boundary);
        let accepted = if let (Some((path, yaws)), Some(identity)) =
            (&self.current_path, previous.execution_identity)
        {
            match self
                .planner
                .replace_execution_path(path.clone(), yaws.clone(), session.goal_pose)
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
