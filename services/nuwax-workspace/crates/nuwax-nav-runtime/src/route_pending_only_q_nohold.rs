// OUT-OF-REPOSITORY DRAFT. Included in route_pending_only_nohold_qualification.
// Same bounded R11/N2 sampler and channels. No hold, worker wait, second consumer,
// receipt trace, new production field, or request/payload clone in the Q probe.
// Q2 qualifies observations, not owner linearization or performance acceptance.

#[derive(Clone, Copy, Debug)]
pub struct Q2State {
    pub epoch: u64,
    pub next: u64,
    pub submitted: Option<(u64, bool)>,
    pub tickets: [Option<(u64, bool, usize)>; 2],
    pub retry_after: Instant,
    pub plans: u64,
    pub admission: Option<u64>,
    pub generation: u64,
}
fn q2_state(f: &RouteFollower) -> Q2State {
    Q2State { epoch: f.owner.as_ref().map_or(0, PlanningOwner::epoch), next: f.next_request,
        submitted: f.submitted.map(|(id, _, mode)| (id, mode)),
        tickets: std::array::from_fn(|i| f.source_tickets[i].as_ref()
            .map(|(id, ticket)| (*id, ticket.unknown, ticket.changes.len()))),
        retry_after: f.retry_after, plans: f.local_plans,
        admission: f.admitted_generation, generation: f.route_generation }
}
#[derive(Clone, Copy, Debug)]
pub struct Q2PendingObservation {
    pub expected_id: u64,
    pub expected_epoch: u64,
    pub before: Instant,
    pub after: Instant,
    pub queued_state: Option<(bool, bool)>,
}
#[derive(Clone, Copy, Debug)]
pub struct Q2StepObservation {
    pub due: Instant,
    pub entered: Instant,
    pub returned: Instant,
    pub before: Q2State,
    pub after: Q2State,
}
pub struct Q2Report {
    pub common: N2Report,
    pub s1_source_ns: Option<i64>,
    pub s1_finished: Option<Instant>,
    pub p_receipt: Option<(u64, u64, Instant)>, // original P id, epoch, retry
    pub step250: Option<Q2StepObservation>,
    pub before_s2: Option<Q2State>,
    pub after_s2: Option<Q2State>,
    pub pending_observation: Option<Q2PendingObservation>,
    pub step300: Option<Q2StepObservation>,
    pub reasons: Vec<String>,
}
impl Q2Report {
    pub fn validate_integrity(&self) -> Result<(), String> {
        let common = &self.common;
        if !common.cleanup_returned { return Err("original follower cleanup did not return".into()); }
        if common.observer_drops != 0 { return Err("bounded kernel observation lost events".into()); }
        if !common.issues.is_empty() { return Err(common.issues.join("; ")); }
        // Only this existing natural-Q reason relaxes full entry attribution:
        // the entire multi-acceptance interval deliberately has no receipt.
        if !self.reasons.iter().any(|r| r == "Q_UNCOVERED_MULTIPLE_ACCEPTANCES") {
            for event in &common.kernel {
                if let N2KernelEvent::Entry { identity, source_ns, .. } = event {
                    common.entry_receipt(identity, *source_ns)?;
                }
            }
        }
        if let Some(probe) = self.pending_observation {
            let damage = common.damage().ok_or("probe without original S2 endpoint")?;
            if probe.before < damage.finished || probe.after < probe.before {
                return Err("Q probe moved before D.finished or has reversed interval".into());
            }
        }
        if let (Some(before), Some(after)) = (self.before_s2, self.after_s2) {
            if before.epoch == after.epoch && before.retry_after != after.retry_after {
                return Err("uncancelled original S2 update changed retry".into());
            }
        }
        if let (Some(step), Some((_, epoch, retry))) = (self.step300, self.p_receipt) {
            if step.returned < retry && step.before.epoch == epoch && step.after.epoch == epoch
                && step.before.retry_after == retry && step.after.retry_after != retry {
                return Err("same-epoch cooldown step changed original retry".into());
            }
            if common.arm == N2Arm::Candidate && step.returned < retry
                && step.before.epoch == epoch && step.after.epoch == epoch
                && step.after.next.checked_sub(step.before.next) == Some(1)
                && step.before.submitted.map(|s| s.1) != step.after.submitted.map(|s| s.1) {
                return Err("single same-epoch cooldown replacement changed original P mode".into());
            }
        }
        Ok(())
    }
    // Fixed classification only. It never changes any input or retries a case.
    pub fn target_uncovered_reasons(&self) -> Vec<String> {
        let mut reasons = self.reasons.clone();
        let common = &self.common;
        if common.observer_drops != 0 { reasons.push("EVENT_LOSS".into()); }
        let Some((p_id, p_epoch, p_retry)) = self.p_receipt else {
            reasons.push("NO_ORIGINAL_P_RECEIPT_AT_250".into()); return reasons;
        };
        let Some(p) = self.step250 else { reasons.push("MISSING_250_STEP".into()); return reasons; };
        if p.due != common.origin + Duration::from_millis(250)
            || p.returned >= common.origin + Duration::from_millis(275) {
            reasons.push("250_STEP_MISSED_FIXED_S2_WINDOW".into());
        }
        if !p.after.tickets.iter().flatten().any(|(id, _, _)| *id == common.initial_id)
            || p.after.tickets.iter().flatten().count() != 2 {
            reasons.push("A_ALREADY_CONSUMED_OR_NO_TWO_TICKETS_AT_P".into());
        }
        let Some(damage) = common.damage() else { reasons.push("NO_S2_INSTALL".into()); return reasons; };
        if self.s1_finished.is_none_or(|at| at > p.entered) || p.returned > damage.began
            || damage.due != common.origin + Duration::from_millis(275) {
            reasons.push("S1_P_STEP_S2_EVENT_ORDER_NOT_ESTABLISHED".into());
        }
        if common.maps.iter().take_while(|map| !map.damage).any(|map|
            Some(map.source.source_ns) != self.s1_source_ns || map.damage)
            || common.maps.iter().filter(|map| !map.damage && map.finished < damage.began).count() != 1 {
            reasons.push("UNEXPECTED_PRE_S2_PUBLICATION".into());
        }
        let (Some(before), Some(after)) = (self.before_s2, self.after_s2) else {
            reasons.push("NO_S2_INSTALL_STATE".into()); return reasons;
        };
        if before.retry_after != p_retry || after.retry_after != p_retry {
            reasons.push("ORIGINAL_P_RETRY_NOT_PRESERVED_ACROSS_S2".into());
        }
        if before.epoch != p_epoch || after.epoch != p_epoch {
            reasons.push("S2_OR_PRIOR_ORIGINAL_POLICY_CANCELLED_EPOCH".into());
        }
        if before.submitted.map(|s| s.0) != Some(p_id) || after.submitted.map(|s| s.0) != Some(p_id)
            || after.tickets.iter().flatten().count() != 2
            || after.tickets.iter().flatten().any(|(_, unknown, _)| *unknown)
            || !after.tickets.iter().flatten().any(|(id, _, batches)| *id == p_id && *batches > 0) {
            reasons.push("P_ID_OR_TWO_KNOWN_TICKETS_OR_OWN_CHANGES_MISSING".into());
        }
        if !self.pending_observation.is_some_and(|probe| {
            probe.expected_id == p_id && probe.expected_epoch == p_epoch
                && probe.queued_state.is_some_and(|(active, completion)| active != completion)
        }) { reasons.push("EXACT_P_NOT_OBSERVED_QUEUED_BEHIND_A".into()); }
        let Some(step) = self.step300 else { reasons.push("MISSING_ORIGINAL_300_STEP".into()); return reasons; };
        if step.due != common.origin + Duration::from_millis(300)
            || step.before.retry_after != p_retry
            || step.entered >= p_retry || step.returned >= p_retry {
            reasons.push("300_STEP_OUTSIDE_ORIGINAL_P_COOLDOWN".into());
        }
        if step.before.epoch != p_epoch || step.after.epoch != p_epoch {
            reasons.push("300_STEP_EPOCH_CHANGED".into());
        }
        let accepted = step.after.next.checked_sub(step.before.next);
        match common.arm {
            N2Arm::Baseline => {
                if accepted != Some(0) || step.after.retry_after != p_retry
                    || step.after.submitted != step.before.submitted
                    || step.before.submitted.map(|s| s.0) != Some(p_id) {
                    reasons.push("A_DID_NOT_KEEP_ORIGINAL_P_WITH_ZERO_ACCEPTANCE".into());
                }
            }
            N2Arm::Candidate => {
                if accepted != Some(1) || step.after.retry_after != p_retry
                    || step.before.submitted.map(|s| s.0) != Some(p_id)
                    || step.after.submitted.map(|s| s.0) != Some(step.after.next)
                    || step.after.submitted.map(|s| s.1) != step.before.submitted.map(|s| s.1)
                    || step.after.tickets.iter().flatten().any(|(id, _, _)| *id == p_id) {
                    reasons.push("B_NO_SINGLE_COOLDOWN_REPLACEMENT_STATE_EVIDENCE".into());
                }
                // These are post-step receipt identities. Exact retired return
                // value and exchange linearization time are NOT observed here.
                let replacement = common.receipts.iter().find(|r| r.identity.id == step.after.next);
                if !replacement.is_some_and(|r| r.after_damage && r.epoch == p_epoch) {
                    reasons.push("B_REPLACEMENT_LACKS_UNAMBIGUOUS_LATEST_RECEIPT".into());
                }
            }
        }
        let expected_started_id = match common.arm {
            N2Arm::Baseline => p_id,
            N2Arm::Candidate => step.after.next,
        };
        if !common.kernel.iter().any(|event| match event {
            N2KernelEvent::Entry { at, identity, source_ns } if *at <= common.ended =>
                common.entry_receipt(identity, *source_ns).is_ok_and(|r| r.identity.id == expected_started_id),
            _ => false,
        }) { reasons.push("EXPECTED_P_OR_REPLACEMENT_ACTUAL_ENTRY_NOT_OBSERVED".into()); }
        if common.kernel.iter().any(|event| match event {
            N2KernelEvent::Entry { identity, source_ns, .. } => common.entry_receipt(identity, *source_ns).is_err(),
            _ => false,
        }) { reasons.push("ACTUAL_KERNEL_ENTRY_RECEIPT_ASSOCIATION_INCOMPLETE".into()); }
        reasons
    }
}
pub struct Q2NoHoldCase {
    common: N2NoHoldCase,
    s1_source_ns: Option<i64>,
    s1_finished: Option<Instant>,
    p_receipt: Option<(u64, u64, Instant)>,
    step250: Option<Q2StepObservation>,
    before_s2: Option<Q2State>,
    after_s2: Option<Q2State>,
    pending_observation: Option<Q2PendingObservation>,
    step300: Option<Q2StepObservation>,
    reasons: Vec<String>,
}
impl Q2NoHoldCase {
    pub fn prepare(arm: N2Arm, config: SurfaceFollowerConfig, initial: Arc<SurfaceSnapshot>,
        initial_meta: &N2SourceMeta, pose: PoseStamped, route: Path3D, goal: PoseStamped)
        -> Result<Self, String> {
        let common = N2NoHoldCase::prepare(arm, N2CaseKind::NaturalQ,
            config, initial, initial_meta, pose, route, goal)?;
        Ok(Self { common, s1_source_ns: None, s1_finished: None, p_receipt: None,
            step250: None, before_s2: None, after_s2: None, pending_observation: None,
            step300: None, reasons: Vec::new() })
    }
    pub fn begin(&mut self) -> Result<Instant, String> { self.common.begin() }
    pub fn publish_s1(&mut self, publication: N2Publication, due: Instant) -> Result<(), String> {
        if self.s1_source_ns.is_some() || self.common.damage_installed { return Err("S1 order violated".into()); }
        if !matches!(publication.update.changes.as_ref(), SurfaceGeometryChanges::Empty) {
            return Err("S1 must be an actual Writer Empty successor".into());
        }
        // S1 is the one allowed pre-damage Empty; use exactly the ordinary
        // installation path, with only the original post-damage ordering guard
        // bypassed in this fixture. No policy/owner/ticket assignment occurs.
        let source_ns = publication.meta.source_ns;
        self.install(publication, false, due)?;
        self.s1_source_ns = Some(source_ns);
        self.s1_finished = self.common.maps.last().map(|m| m.finished);
        Ok(())
    }
    fn install(&mut self, publication: N2Publication, damage: bool, due: Instant) -> Result<(), String> {
        if publication.meta.source_ns <= self.common.current_source_ns { return Err("source did not advance".into()); }
        let actual_ns = publication.update.snapshot.observations().and_then(|o| o.get(SOURCE_PROBE)).map(|o| o.observed_ns);
        if actual_ns != u64::try_from(publication.meta.source_ns).ok() || actual_ns != Some(publication.meta.observed_ns) {
            return Err("actual Writer/source/observation mismatch".into());
        }
        if self.common.maps.len() == MAP_CAP { return Err("map cap exceeded".into()); }
        let f = self.common.follower.as_mut().ok_or("already cleaned")?;
        let before = q2_state(f);
        let began = Instant::now();
        let result = f.update_execution_map(ExecutionMap::SurfaceUpdate(publication.update));
        let finished = Instant::now(); // ORIGINAL D endpoint: BEFORE any Q probe
        let after = q2_state(f);
        if damage {
            self.before_s2 = Some(before);
            self.after_s2 = Some(after);
            // Exactly once, with the already-recorded ORIGINAL P id/epoch.
            // No P at 250 => no probe; never substitute 275's latest submitted.
            if let Some((expected_id, expected_epoch, _)) = self.p_receipt {
                let before = Instant::now();
                let queued_state = f.owner.as_ref().and_then(|owner|
                    owner.pending_only_q_state_for_qualification(expected_id, expected_epoch));
                let after = Instant::now();
                self.pending_observation = Some(Q2PendingObservation {
                    expected_id, expected_epoch, before, after, queued_state });
            }
        }
        self.common.maps.push(N2MapInstall { damage, due, began, finished,
            source: publication.meta.clone(), epoch_before: before.epoch, epoch_after: after.epoch,
            tickets_before: before.tickets.iter().flatten().count(),
            tickets_after: after.tickets.iter().flatten().count(),
            submitted_before: before.submitted.map(|s| s.0), submitted_after: after.submitted.map(|s| s.0) });
        result.map_err(|e| e.to_string())?;
        self.common.current_source_ns = publication.meta.source_ns;
        self.common.damage_installed |= damage;
        Ok(())
    }
    pub fn publish_s2(&mut self, publication: N2Publication, due: Instant) -> Result<(), String> {
        if self.s1_source_ns.is_none() || self.common.damage_installed { return Err("S2 order violated".into()); }
        if !matches!(publication.update.changes.as_ref(), SurfaceGeometryChanges::Columns { .. }) {
            return Err("S2 must be actual Writer Columns".into());
        }
        self.install(publication, true, due)
    }
    pub fn publish_empty(&mut self, publication: N2Publication, due: Instant) -> Result<(), String> {
        self.common.publish(publication, false, due)
    }
    pub fn update_pose(&mut self, pose: PoseStamped) -> Result<(), String> { self.common.update_pose(pose) }
    pub fn move_pose_once(&mut self, pose: PoseStamped) -> Result<(), String> {
        let f = self.common.follower.as_mut().ok_or("already cleaned")?;
        let old = f.pose.as_ref().ok_or("missing old pose")?;
        if old.pose.position.x.to_bits() != 12.55_f64.to_bits()
            || pose.pose.position.x.to_bits() != 13.55_f64.to_bits()
            || old.frame_id != pose.frame_id || old.stamp.clock != pose.stamp.clock
            || pose.stamp.nanos < old.stamp.nanos { return Err("fixed Q pose/source-clock contract violated".into()); }
        let map = f.map.as_ref().ok_or("missing map")?;
        map.validate_body_pose(body(old).0, body(old).1).map_err(|e| e.to_string())?;
        map.validate_body_pose(body(&pose).0, body(&pose).1).map_err(|e| e.to_string())?;
        map.validate_body_motion(body(old), body(&pose), &f.boundary).map_err(|e| e.to_string())?;
        f.update_odometry(pose);
        Ok(())
    }
    pub fn tick(&mut self, due: Instant) -> Result<(), String> {
        let origin = self.common.origin.ok_or("tick before begin")?;
        let special = due == origin + Duration::from_millis(250) || due == origin + Duration::from_millis(300);
        let before = special.then(|| q2_state(self.common.follower.as_ref().unwrap()));
        let outcome = self.common.tick_recorded(due, Some(&mut self.reasons));
        // ORIGINAL step; missing interval receipts never gate future input.
        if let Some(before) = before {
            if let Some(control) = self.common.controls.last().filter(|c| c.due == due) {
                let after = q2_state(self.common.follower.as_ref().unwrap());
                let observation = Q2StepObservation { due, entered: control.entered,
                    returned: control.returned, before, after };
                if due == origin + Duration::from_millis(250) {
                    self.step250 = Some(observation);
                    let accepted = after.next.checked_sub(before.next);
                    let receipt = self.common.receipts.iter().find(|r| r.identity.id == after.next);
                    if accepted == Some(1) && receipt.is_some_and(|r|
                        Some(r.source_ns) == self.s1_source_ns && r.identity.id != self.common.initial_id)
                        && after.submitted.map(|s| s.0) == Some(after.next) {
                        self.p_receipt = Some((after.next, after.epoch, after.retry_after));
                    } else { self.reasons.push("250_DID_NOT_ACCEPT_ONE_ORIGINAL_S1_P".into()); }
                } else { self.step300 = Some(observation); }
            }
        }
        outcome
    }
    pub fn drain_events(&mut self) -> Result<(), String> { self.common.drain_events() }
    pub fn endpoints_seen(&self) -> bool {
        self.step300.is_some() && self.common.endpoints_seen()
    }
    pub fn skipped_pose(&mut self, count: u64) { self.common.skipped_pose(count); }
    pub fn skipped_control(&mut self, count: u64) { self.common.skipped_control(count); }
    pub fn note_issue(&mut self, issue: String) { self.common.note_issue(issue); }
    pub fn note_uncovered(&mut self, reason: String) { self.reasons.push(reason); }
    pub fn end_measurement(&mut self) { self.common.end_measurement(); }
    pub fn finish(self) -> Q2Report {
        Q2Report { common: self.common.finish(), s1_source_ns: self.s1_source_ns,
            s1_finished: self.s1_finished, p_receipt: self.p_receipt,
            step250: self.step250, before_s2: self.before_s2, after_s2: self.after_s2,
            pending_observation: self.pending_observation, step300: self.step300, reasons: self.reasons }
    }
}
