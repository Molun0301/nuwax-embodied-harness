// TESTS-ONLY, OUT-OF-REPOSITORY DRAFT. NOT COMPILED OR RUN.
// Include inside route_follower::tests after the selective refresh include.
// New pending-only v2 policy: original update_execution_map is unchanged.
// Uses the real step/owner/receipt/proof/current body and execution guard.
// The candidate payloads are synthetic F-layer status fixtures. The separate
// Writer/PlanKernel Q1 tests establish actual planning geometry and source chains.
// Gate durations are causal test controls, NEVER performance measurements.
mod pending_only_v2_tests {
    use super::*;

    #[derive(Clone)]
    struct Entry {
        request_id: u64,
        request_epoch: u64,
        source: Arc<SurfaceSnapshot>,
        pose: PoseStamped,
        detour: bool,
        generation: u64,
        route: Arc<Path3D>,
        cumulative: Arc<Vec<f64>>,
        boundary: Arc<Vec<[f64;2]>>,
        unmaterialized: bool,
        progress: f64,
        reference_progress: f64,
        goal: Option<PoseStamped>,
        options: GoalOptions,
        offset: f64,
        scan: ScanConfig,
    }
    #[derive(Clone, Copy)]
    enum ResultKind { Full, Partial, Error }
    struct Gate {
        entered: mpsc::Receiver<Entry>,
        release: Option<mpsc::Sender<()>>,
    }
    impl Gate {
        fn entry(&self) -> Entry {
            self.entered.recv_timeout(Duration::from_secs(5)).expect("original worker entry")
        }
        fn release(&self) { self.release.as_ref().unwrap().send(()).unwrap(); }
    }
    impl Drop for Gate {
        fn drop(&mut self) {
            if let Some(tx) = self.release.take() {
                let _ = tx.send(()); let _ = tx.send(());
            }
        }
    }
    fn install(f: &mut RouteFollower, kind: ResultKind) -> Gate {
        let (entry_tx, entered) = mpsc::channel();
        let (release, wait) = mpsc::channel();
        let offset = f.config.body_to_support_z_m;
        f.cancel_planning();
        f.owner = Some(PlanningOwner::new(move |mut request: PlanRequest, _| {
            let reference=request.reference_source.as_ref().expect("shared normal request source");
            let (request_id,request_epoch)=crate::planning_owner::current_request_for_test()
                .expect("real owner compute identity");
            let _ = entry_tx.send(Entry {request_id,request_epoch,source:request.source.clone(), pose:request.pose.clone(),
                detour:request.detour, generation:request.generation,
                route:reference.route.clone(),cumulative:reference.cumulative.clone(),
                boundary:request.boundary.clone(),unmaterialized:request.reference.is_empty(),
                progress:request.progress,reference_progress:reference.progress,
                goal:reference.goal.clone(),options:reference.options,offset:reference.offset,
                scan:request.scan.clone()});
            // F-layer only. Disconnection and timeout both release unwind.
            match wait.recv_timeout(Duration::from_secs(30)) {
                Ok(()) | Err(mpsc::RecvTimeoutError::Disconnected)=>{},
                Err(mpsc::RecvTimeoutError::Timeout)=>panic!("F-layer gate timed out before explicit release"),
            }
            request.materialize_reference().unwrap();
            let outcome = match kind {
                ResultKind::Error => Err(typed_failures()),
                ResultKind::Full | ResultKind::Partial => {
                    let path = if matches!(kind, ResultKind::Partial) {
                        let mut end = request.pose.clone(); end.pose.position.x += 0.7;
                        Path3D {poses:vec![request.pose.clone(),end]}
                    } else {
                        Path3D {poses:request.reference.iter().map(|p| {
                            let mut pose=request.pose.clone();
                            pose.pose.position=Point3::new(p[0],p[1],p[2]+offset); pose
                        }).collect()}
                    };
                    Ok(Candidate {partial:matches!(kind,ResultKind::Partial),path,elapsed_ms:0.0})
                }
            };
            PlanResult {request,outcome}
        }).unwrap());
        f.dirty=true;
        Gate {entered,release:Some(release)}
    }
    fn ids(f: &RouteFollower) -> Vec<u64> {
        f.source_tickets.iter().flatten().map(|(id,_)|*id).collect()
    }
    fn ticket(f: &RouteFollower, id:u64) -> SurfaceJobTicket {
        f.source_tickets.iter().flatten().find(|(got,_)|*got==id).unwrap().1.clone()
    }
    fn same_ticket(actual:&SurfaceJobTicket, expected:&SurfaceJobTicket) {
        assert_eq!((actual.columns,actual.unknown),(expected.columns,expected.unknown));
        match (&actual.basis,&expected.basis) {
            (Some(a),Some(b))=>assert!(Arc::ptr_eq(a,b)),
            (None,None)=>{}, _=>panic!("ticket basis changed"),
        }
        assert_eq!(actual.changes.len(),expected.changes.len());
        for (a,b) in actual.changes.iter().zip(&expected.changes) { assert!(Arc::ptr_eq(a,b)); }
    }
    fn tick_work(f:&mut RouteFollower, expected:(u64,u64)) -> PlannerStep {
        // Counts cover the ENTIRE real step. Expected values explicitly include
        // any legal original schedule; (1,2) means refresh plus normal schedule.
        // Most cases exclude normal schedule by actual cooldown or deduplication.
        assert!(f.controller.current_body_error().is_none(),"must reach coalesce/consume");
        let before=f.pending_source_work_for_qualification();
        let step=f.step(0.3);
        let after=f.pending_source_work_for_qualification();
        let delta=(after.0-before.0,after.1-before.1);
        eprintln!("PENDING_WORK before={before:?} after={after:?} delta={delta:?} expected={expected:?}");
        assert_eq!(delta,expected,"replace-lock attempts, shared request-builder entries");
        step
    }
    fn wait_retry(f:&RouteFollower) {
        let deadline=f.retry_after;
        if let Some(wait)=deadline.checked_duration_since(Instant::now()) { std::thread::sleep(wait); }
        assert!(Instant::now()>=deadline);
    }
    fn two_tickets(f:&mut RouteFollower, gate:&Gate) -> (u64,u64,Entry) {
        assert!(matches!(f.step(0.3),PlannerStep::Pending(_)) || f.admitted_generation.is_some());
        let a=f.submitted.unwrap().0;
        let entry=gate.entry();
        assert_eq!(entry.request_id,a);assert_eq!(entry.request_epoch,f.owner.as_ref().unwrap().epoch());
        assert!(Arc::ptr_eq(&entry.source,f.map.as_ref().unwrap()));
        wait_retry(f);
        let mut pose=f.pose.clone().unwrap();
        pose.pose.position.x+=f.scan.refresh_distance_m.max(0.25);
        pose.stamp=Timestamp::from_unix(2,0);
        f.update_odometry(pose);
        f.step(0.3);
        let p=f.submitted.unwrap().0;
        assert_ne!(a,p);
        assert_eq!(ids(f),vec![a,p]);
        assert!(gate.entered.try_recv().is_err(),"queued P must not start behind held A");
        (a,p,entry)
    }
    // Test-only evaluation of the UNCHANGED original affected expression.
    // Observe cloned tickets exactly as update_execution_map does, then assert
    // the real publication retained/cancelled the epoch and original identities.
    fn publish_checked(f:&mut RouteFollower, update:SurfaceMapUpdate, expected_affected:bool) {
        let old=f.map.clone().expect("existing source");
        let mut change=SurfaceJobTicket::new(Some(old));
        change.observe(&update.snapshot,update.change_basis.as_deref(),&update.changes);
        let mut observed:Vec<SurfaceJobTicket>=f.source_tickets.iter().flatten()
            .map(|(_,proof)|proof.clone()).collect();
        for proof in &mut observed {
            proof.observe(&update.snapshot,update.change_basis.as_deref(),&update.changes);
        }
        let ticket_unknown=observed.iter().any(|proof|proof.unknown);
        let global_related=f.route.as_ref().is_some_and(|path|!change.permits_path(&update.snapshot,path));
        let local_related=f.local_path.as_ref().is_some_and(|path|!change.permits_path(&update.snapshot,path));
        let affected=change.unknown || ticket_unknown
            || (!matches!(update.changes.as_ref(),SurfaceGeometryChanges::Empty)
                && (global_related || local_related));
        assert_eq!(affected,expected_affected,
            "original affected predicate: delta_unknown={}, ticket_unknown={ticket_unknown}, global_related={global_related}, local_related={local_related}",change.unknown);
        if !expected_affected && !matches!(update.changes.as_ref(),SurfaceGeometryChanges::Empty) {
            assert!(!change.unknown && !ticket_unknown && !global_related && !local_related,
                "far Columns must be disjoint from BOTH retained paths and keep every proof complete");
        }
        let epoch=f.owner.as_ref().unwrap().epoch();
        let old_ids=ids(f);let old_submitted=f.submitted;
        let before=Instant::now();
        f.update_execution_map(ExecutionMap::SurfaceUpdate(update)).unwrap();
        if expected_affected {
            assert!(f.owner.as_ref().unwrap().epoch()>epoch);
            assert!(ids(f).is_empty() && f.submitted.is_none());
            assert!(f.retry_after>=before && f.retry_after<=Instant::now(),
                "original cancel sets retry to the actual cancellation time");
            assert!(f.dirty);
        } else {
            assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
            assert_eq!(ids(f),old_ids,"publication itself must retain the original A/P identities");
            assert_eq!(f.submitted,old_submitted);
            for (id,expected) in old_ids.iter().zip(&observed) {
                same_ticket(&ticket(f,*id),expected);
            }
        }
    }
    fn publish(f: &mut RouteFollower, next:Arc<SurfaceSnapshot>, columns:&[[i32;2]]) {
        let old=f.map.clone().unwrap();
        publish_checked(f,selective_update(&old,next,columns),false);
    }
    fn empty(f:&mut RouteFollower) {
        let old=f.map.clone().unwrap();
        f.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
            snapshot:old.clone(),change_basis:Some(old),planar_reference:None,
            changes:Arc::new(SurfaceGeometryChanges::Empty),
        })).unwrap();
    }
    fn release_and_step(f:&mut RouteFollower,gate:&Gate) -> PlannerStep {
        gate.release();
        wait_for(||f.owner.as_ref().unwrap().completion_ready_for_test());
        f.step(0.3)
    }

    #[test]
    fn pending_only_v2_far_chain_replaces_only_p_with_latest_pose_progress_and_exact_retry() {
        let mut f=open_follower();
        let s0=selective_scene(&[]);
        f.update_execution_map(ExecutionMap::Surface(s0.clone())).unwrap();
        // Entirely NEW disjoint fixture. This does not replace the old [20,10]
        // related-chain witness or prove that A's actual detour is invalidated.
        let publications=[
            (selective_scene(&[[100,100,5]]),vec![[100,100]]),
            (selective_scene(&[]),vec![[100,100]]),
            (selective_scene(&[[110,100,5]]),vec![[110,100]]),
        ];
        let gate=install(&mut f,ResultKind::Full);
        let (a,p1,a_entry)=two_tickets(&mut f,&gate);
        let epoch=f.owner.as_ref().unwrap().epoch();
        let deadline=f.retry_after;let calls=f.local_planning_calls;
        let mut previous=p1;
        for (index,(source,columns)) in publications.into_iter().enumerate() {
            publish(&mut f,source.clone(),&columns);
            let a_before=ticket(&f,a);
            assert_eq!(a_before.changes.len(),index+1);
            // update_odometry does NOT advance RouteFollower.progress. The real
            // step must advance it before making the replacement request.
            let x=1.6+index as f64*0.1;
            let pose=comparison_pose(x,22+index as u32);
            let stale_progress=f.progress;
            f.update_odometry(pose.clone());
            assert_eq!(f.progress,stale_progress);
            assert!(Instant::now()<deadline,"original cooldown expired; never extend it");
            assert!(matches!(tick_work(&mut f,(1,1)),PlannerStep::Pending(_)));
            assert!(f.progress>stale_progress);
            assert!((f.progress-(x-0.5)).abs()<1e-9);
            let current=f.submitted.unwrap().0;
            assert_eq!(current,previous+1);assert_eq!(f.next_request,current);
            assert_eq!(ids(&f),vec![a,current]);assert!(!ids(&f).contains(&previous));
            same_ticket(&ticket(&f,a),&a_before);
            let p=ticket(&f,current);
            assert!(!p.unknown && p.changes.is_empty() && p.columns==0);
            assert!(p.basis.as_ref().is_some_and(|basis|Arc::ptr_eq(basis,&source)));
            assert_eq!(f.retry_after,deadline);
            assert_eq!(f.local_planning_calls,calls+index as u64+1);
            assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
            assert!(gate.entered.try_recv().is_err(),"retired P variants cannot enter the worker");
            previous=current;
        }
        for _ in 0..3 {
            empty(&mut f);let proof=ticket(&f,a);
            assert!(matches!(tick_work(&mut f,(0,0)),PlannerStep::Pending(_)));
            same_ticket(&ticket(&f,a),&proof);
            assert_eq!(f.submitted.unwrap().0,previous);assert_eq!(f.retry_after,deadline);
        }
        assert!(Arc::ptr_eq(&a_entry.source,&s0));
        assert_eq!(a_entry.progress,0.0);
        assert!(ticket(&f,a).permits_path(f.map.as_ref().unwrap(),f.route.as_ref().unwrap()),
            "far proof deliberately does NOT reject A; Q1 separately covers the real detour witness");
        release_and_step(&mut f,&gate);
        assert_eq!(f.local_plans,1);assert_eq!(ids(&f),vec![previous]);
        let p_entry=gate.entry();
        assert_eq!(p_entry.request_id,previous);assert_eq!(p_entry.request_epoch,epoch);
        assert!(Arc::ptr_eq(&p_entry.source,f.map.as_ref().unwrap()));
        assert_eq!(p_entry.pose,f.pose.clone().unwrap());
        assert_eq!(p_entry.progress,f.progress);
        assert_eq!(p_entry.reference_progress,f.progress);
        assert_eq!(p_entry.generation,f.route_generation);
        assert_eq!(p_entry.goal,f.goal);assert_eq!(p_entry.options,f.options);
        assert_eq!(p_entry.offset,f.config.body_to_support_z_m);
        assert_eq!(p_entry.scan.horizon_m,f.scan.horizon_m);
        assert_eq!(p_entry.scan.refresh_distance_m,f.scan.refresh_distance_m);
        assert_eq!(p_entry.scan.optimization_iterations,f.scan.optimization_iterations);
        assert_eq!(p_entry.scan.reference_corridor_m,f.scan.reference_corridor_m);
        assert_eq!(p_entry.scan.search_expansion_limit,f.scan.search_expansion_limit);
        assert!(p_entry.unmaterialized);
        assert!(Arc::ptr_eq(&p_entry.route,f.route.as_ref().unwrap()));
        assert!(Arc::ptr_eq(&p_entry.cumulative,&f.cumulative));
        assert!(Arc::ptr_eq(&p_entry.boundary,&f.boundary));
        assert!(!p_entry.detour);
        release_and_step(&mut f,&gate);
        assert_eq!(f.local_plans,2);assert_eq!(f.admitted_generation,Some(f.route_generation));
        assert!(ids(&f).is_empty());f.stop();
    }

    #[test]
    fn pending_only_v2_single_far_active_and_completion_have_zero_refresh_work() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        let gate=install(&mut f,ResultKind::Full);
        f.step(0.3);gate.entry(); // Observed ACTIVE, not a claimed initial-pending gate.
        let id=f.submitted.unwrap().0;let epoch=f.owner.as_ref().unwrap().epoch();
        let next=f.next_request;let calls=f.local_planning_calls;let deadline=f.retry_after;
        publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
        assert!(f.dirty && ticket(&f,id).changes.len()==1);
        for _ in 0..3 {
            tick_work(&mut f,(0,0));
            assert_eq!(ids(&f),vec![id]);assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
            assert_eq!((f.next_request,f.local_planning_calls),(next,calls));
            assert_eq!(f.retry_after,deadline);
        }
        gate.release();wait_for(||f.owner.as_ref().unwrap().completion_ready_for_test());
        assert_eq!(ids(&f),vec![id]);
        tick_work(&mut f,(0,0)); // Real single completion is consumed with no new helper work.
        assert_eq!((f.next_request,f.local_planning_calls),(next,calls));
        assert_eq!(f.local_plans,1);assert!(ids(&f).is_empty());f.stop();
    }

    #[test]
    fn pending_only_v2_two_ticket_empty_and_dirty_only_do_not_replace() {
        let mut f=open_follower();
        let gate=install(&mut f,ResultKind::Full);
        let (a,p,_)=two_tickets(&mut f,&gate);
        let calls=f.local_planning_calls;let next=f.next_request;let deadline=f.retry_after;
        assert!(f.dirty && f.local_path.is_none());
        for _ in 0..3 {
            empty(&mut f);
            tick_work(&mut f,(0,0));
            assert_eq!(ids(&f),vec![a,p]);assert_eq!(f.submitted.unwrap().0,p);
            assert_eq!((f.next_request,f.local_planning_calls),(next,calls));
            assert_eq!(f.retry_after,deadline);
        }
        f.stop();gate.release();
        wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
        assert!(gate.entered.try_recv().is_err());
    }

    fn recent_wall(source:&Arc<SurfaceSnapshot>, x:f64) -> Arc<SurfaceSnapshot> {
        Arc::new(source.with_recent_obstacle_points(2,2_000_000_000,0,&[],
            (0..5).map(|i|(i,[x,0.8+f64::from(i)*0.1,0.5]))).unwrap())
    }
    fn publish_empty_source(f:&mut RouteFollower,next:Arc<SurfaceSnapshot>) {
        let old=f.map.clone().unwrap();
        assert!(nuwax_nav_pct::same_source_geometry(&old,&next));
        publish_checked(f,SurfaceMapUpdate {
            snapshot:next,change_basis:Some(old),planar_reference:None,
            changes:Arc::new(SurfaceGeometryChanges::Empty),
        },false);
    }

    // Real initial PlanKernel admission + actual recent stopping obstacle.
    // No admitted_generation/local_path/retry field is fabricated to make this true.
    fn revoked_nominal_pair() -> (RouteFollower,Gate,u64,u64,Instant) {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        assert!(matches!(ready_step(&mut f,0.3),PlannerStep::Command(c) if c.linear.x>0.0));
        let generation=f.route_generation;
        let gate=install(&mut f,ResultKind::Full);
        let (a,p1,_)=two_tickets(&mut f,&gate);
        assert_eq!(f.admitted_generation,Some(generation));assert!(!f.submitted.unwrap().2);
        let deadline=f.retry_after;
        let x=f.pose.as_ref().unwrap().pose.position.x;
        assert!((x-1.5).abs()<1e-9);
        let wall=recent_wall(f.map.as_ref().unwrap(),x+0.23);
        publish_empty_source(&mut f,wall);
        assert!(f.controller.current_body_error().is_none(),"wall must intersect stopping tail, not the current body");
        f.observe_applied_command(Twist2 {
            linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0,
        });
        assert!(Instant::now()<deadline,"fixture missed actual original cooldown");
        assert!(matches!(tick_work(&mut f,(0,0)),PlannerStep::Pending(_)));
        // This text is written only by current_rejection after the REAL
        // controller.step returned ReplanRequired, not by test state injection.
        assert!(f.execution_wait_reason().is_some_and(|reason|reason.contains("当前轨迹受阻")),
            "must exercise the actual current command guard rejection");
        assert!(f.local_path.is_some() && f.admitted_generation.is_none());
        assert_eq!(f.submitted.unwrap().0,p1);assert!(!f.submitted.unwrap().2);
        assert_eq!(f.retry_after,deadline);
        // Revoked/dirty alone still cannot bypass the original retry gate.
        empty(&mut f);tick_work(&mut f,(0,0));
        assert_eq!(f.submitted.unwrap().0,p1);assert!(!f.submitted.unwrap().2);
        assert_eq!(f.retry_after,deadline);
        (f,gate,a,p1,deadline)
    }

    #[test]
    fn pending_only_v2_real_guard_rejection_keeps_nominal_inside_original_cooldown() {
        let (mut f,gate,a,p1,deadline)=revoked_nominal_pair();
        let epoch=f.owner.as_ref().unwrap().epoch();let calls=f.local_planning_calls;
        let x=f.pose.as_ref().unwrap().pose.position.x;
        let changed=recent_wall(&selective_scene(&[[100,100,5]]),x+0.23);
        publish(&mut f,changed,&[[100,100]]); // NEW far fixture, BOTH retained paths checked.
        let original_a=ticket(&f,a);
        assert!(f.local_path.is_some() && f.admitted_generation.is_none());
        assert!(Instant::now()<deadline,"original nominal cooldown must still be active");
        assert!(matches!(tick_work(&mut f,(1,1)),PlannerStep::Pending(_)));
        let p2=f.submitted.unwrap().0;
        assert_eq!(p2,p1+1);assert!(!f.submitted.unwrap().2,"refresh must preserve nominal despite genuine revocation");
        assert_eq!(ids(&f),vec![a,p2]);same_ticket(&ticket(&f,a),&original_a);
        assert_eq!(f.retry_after,deadline);assert_eq!(f.local_planning_calls,calls+1);
        assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
        assert!(f.admitted_generation.is_none(),"refresh alone cannot re-admit the path");
        assert!(ticket(&f,a).permits_path(f.map.as_ref().unwrap(),f.route.as_ref().unwrap()),
            "far changes must not be falsely described as rejecting A's straight source proof");
        // Only take the genuine completed A to expose the actual queued request.
        // Do not let a later ordinary schedule obscure P2's mode capture. A still
        // passes through unchanged consume_plan/latest connector; this test makes
        // NO unsupported claim that the far proof itself rejects A.
        gate.release();wait_for(||f.owner.as_ref().unwrap().completion_ready_for_test());
        f.consume_plan();let entry=gate.entry();
        assert_eq!(entry.request_id,p2);assert_eq!(entry.request_epoch,epoch);
        assert!(!entry.detour);assert!(Arc::ptr_eq(&entry.source,f.map.as_ref().unwrap()));
        assert_eq!(entry.pose,f.pose.clone().unwrap());
        f.stop();gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
    }

    #[test]
    fn pending_only_v2_expired_retry_allows_refresh_then_original_detour_schedule_in_one_step() {
        let (mut f,gate,a,p1,deadline)=revoked_nominal_pair();
        let x=f.pose.as_ref().unwrap().pose.position.x;
        wait_retry(&f); // Real passage of the original 250ms, never write retry_after.
        assert_eq!(f.retry_after,deadline);
        publish(&mut f,recent_wall(&selective_scene(&[[100,100,5]]),x+0.23),&[[100,100]]);
        let proof=ticket(&f,a);let calls=f.local_planning_calls;
        let epoch=f.owner.as_ref().unwrap().epoch();let before=Instant::now();
        assert!(matches!(tick_work(&mut f,(1,2)),PlannerStep::Pending(_)));
        // First acceptance is pending-only nominal refresh, second is original
        // legal schedule(true). Never incorrectly cap all step acceptances at one.
        let p3=f.submitted.unwrap().0;
        assert_eq!(p3,p1+2);assert_eq!(f.next_request,p3);assert!(f.submitted.unwrap().2);
        assert_eq!(f.local_planning_calls,calls+2);assert_eq!(ids(&f),vec![a,p3]);
        same_ticket(&ticket(&f,a),&proof);
        assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
        assert!(f.retry_after>=before+Duration::from_millis(250));
        assert!(f.retry_after<=Instant::now()+Duration::from_millis(250));
        assert!(f.retry_after>deadline);
        assert!(f.admitted_generation.is_none());assert!(gate.entered.try_recv().is_err());
        gate.release();wait_for(||f.owner.as_ref().unwrap().completion_ready_for_test());
        f.consume_plan();let entry=gate.entry();assert!(entry.detour);
        assert_eq!(entry.request_id,p3);assert_eq!(entry.request_epoch,epoch);
        assert!(Arc::ptr_eq(&entry.source,f.map.as_ref().unwrap()));
        f.stop();gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
    }

    #[test]
    fn pending_only_v2_existing_detour_is_never_downgraded_without_recovery_state() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        let gate=install(&mut f,ResultKind::Full);
        f.step(0.3);gate.entry();let a=f.submitted.unwrap().0;
        wait_retry(&f);
        // Explicit F-layer mode stimulus, not a claimed real guard recovery.
        // There is no local path; v2 must preserve the existing true mode exactly.
        f.schedule(true);
        let p1=f.submitted.unwrap().0;
        assert!(f.submitted.unwrap().2 && f.local_path.is_none());
        assert_eq!(ids(&f),vec![a,p1]);
        let deadline=f.retry_after;
        publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
        tick_work(&mut f,(1,1));
        let p2=f.submitted.unwrap().0;
        assert_eq!(p2,p1+1);assert!(f.submitted.unwrap().2);
        assert_eq!(f.retry_after,deadline);
        release_and_step(&mut f,&gate);
        let entry=gate.entry();assert!(entry.detour);
        f.stop();gate.release();
        wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
    }

    #[test]
    fn pending_only_v2_unknown_then_empty_and_stop_keep_hard_retirement() {
        for stop in [false,true] {
            let mut f=open_follower();
            let gate=install(&mut f,ResultKind::Full);
            let (_a,_p,_)=two_tickets(&mut f,&gate);
            let epoch=f.owner.as_ref().unwrap().epoch();
            let before=f.pending_source_work_for_qualification();
            if stop { f.stop(); }
            else {
                let source=f.map.clone().unwrap();
                f.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
                    snapshot:source.clone(),change_basis:Some(source),planar_reference:None,
                    changes:Arc::new(SurfaceGeometryChanges::Unknown),
                })).unwrap();
                empty(&mut f);
            }
            assert!(f.owner.as_ref().unwrap().epoch()>epoch);
            assert!(ids(&f).is_empty() && f.submitted.is_none());
            assert_eq!(f.pending_source_work_for_qualification(),before);
            gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
            assert_eq!(f.local_plans,0);
            assert!(gate.entered.try_recv().is_err(),"retired P cannot run");
            // Do not step Unknown here: a new normal request is legitimate and
            // is not the retired completion. Stop preserves late-result denial.
            f.stop();f.step(0.3);assert_eq!(f.local_plans,0);
        }
    }

    #[test]
    fn pending_only_v2_partial_and_error_keep_original_completion_boundaries() {
        for kind in [ResultKind::Partial,ResultKind::Error] {
            let mut f=open_follower();
            f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
            let gate=install(&mut f,kind);
            f.step(0.3);gate.entry();
            let a=f.submitted.unwrap().0;let epoch=f.owner.as_ref().unwrap().epoch();
            publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
            tick_work(&mut f,(0,0));
            assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
            release_and_step(&mut f,&gate);
            assert!(!ids(&f).contains(&a));
            match kind {
                ResultKind::Partial=>{
                    assert_eq!(f.local_plans,1);
                    assert!((f.local_end-0.7).abs()<1e-9);
                    assert!(f.local_end<*f.cumulative.last().unwrap());
                },
                ResultKind::Error=>{
                    assert_eq!(f.local_plans,0);assert!(f.admitted_generation.is_none());
                    assert!(f.local_failures.is_empty() && f.qualified_failure.is_none(),
                        "an old-source Err has no successful path for selective acceptance");
                },
                ResultKind::Full=>unreachable!(),
            }
            f.stop();
        }
    }

    #[test]
    fn pending_only_v2_partial_behind_current_progress_stays_rejected_after_p_replace() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        let gate=install(&mut f,ResultKind::Partial);
        let (a,p1,_)=two_tickets(&mut f,&gate);
        assert!((f.progress-1.0).abs()<1e-9);
        publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
        tick_work(&mut f,(1,1));
        let p2=f.submitted.unwrap().0;assert_eq!(p2,p1+1);
        release_and_step(&mut f,&gate);
        assert!(!ids(&f).contains(&a));assert_eq!(f.local_plans,0);
        assert!(f.admitted_generation.is_none(),"A's 0.7m prefix is behind current 1.0m progress");
        gate.entry();release_and_step(&mut f,&gate);
        assert_eq!(f.local_plans,1);
        assert!((f.local_end-1.7).abs()<1e-9);
        f.stop();
    }

    #[test]
    fn pending_only_v2_same_source_err_remains_a_current_typed_failure() {
        let mut f=open_follower();let gate=install(&mut f,ResultKind::Error);
        f.step(0.3);gate.entry();empty(&mut f);
        let step=release_and_step(&mut f,&gate);
        assert!(matches!(step,PlannerStep::Pending(_)));
        assert_eq!(f.local_plans,0);assert!(f.admitted_generation.is_none());
        assert_eq!(f.local_failures.len(),2);
        assert_eq!(f.local_failures[0].kind,nuwax_nav_scan::LocalPlanFailureKind::BudgetExhausted);
        assert_eq!(f.local_failures[1].kind,nuwax_nav_scan::LocalPlanFailureKind::SearchExhausted);
        f.stop();
    }

    #[test]
    fn pending_only_v2_recent_body_error_returns_before_replace_or_take_with_far_columns() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        let gate=install(&mut f,ResultKind::Full);
        let (a,p,_)=two_tickets(&mut f,&gate);
        gate.release();wait_for(||f.owner.as_ref().unwrap().completion_ready_for_test());
        // NEW split fixture: disjoint terrain delta retains both proof slots;
        // real live occupancy then makes the actual current body invalid.
        publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
        assert!(f.controller.current_body_error().is_none());
        let x=f.pose.as_ref().unwrap().pose.position.x;
        let body_wall=recent_wall(f.map.as_ref().unwrap(),x);
        publish_empty_source(&mut f,body_wall);
        assert!(f.controller.current_body_error().is_some(),"real recent occupancy must invalidate current body");
        assert_eq!(ticket(&f,p).changes.len(),1);
        let a_proof=ticket(&f,a);let p_proof=ticket(&f,p);
        let epoch=f.owner.as_ref().unwrap().epoch();
        let before=f.pending_source_work_for_qualification();
        let next=f.next_request;let calls=f.local_planning_calls;let deadline=f.retry_after;
        assert!(matches!(f.step(0.3),PlannerStep::Pending(_)));
        assert!(f.execution_wait_reason().is_some_and(|reason|reason.contains("当前机体几何检查未通过")));
        let after=f.pending_source_work_for_qualification();
        assert_eq!((after.0-before.0,after.1-before.1),(0,0));
        assert!(f.owner.as_ref().unwrap().completion_ready_for_test(),"body early return cannot take A");
        assert_eq!(ids(&f),vec![a,p]);assert_eq!((f.next_request,f.local_planning_calls),(next,calls));
        same_ticket(&ticket(&f,a),&a_proof);same_ticket(&ticket(&f,p),&p_proof);
        assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);assert_eq!(f.retry_after,deadline);
        assert!(gate.entered.try_recv().is_err(),"P cannot start while A completion remains");
        f.stop();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());assert_eq!(f.local_plans,0);
    }

    #[test]
    fn pending_only_v2_admitted_background_refresh_keeps_current_guarded_motion() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        assert!(matches!(ready_step(&mut f,0.3),PlannerStep::Command(c) if c!=Twist2::ZERO));
        let admitted=f.local_plans;let generation=f.route_generation;
        let gate=install(&mut f,ResultKind::Full);
        let (a,p1,_)=two_tickets(&mut f,&gate);
        let epoch=f.owner.as_ref().unwrap().epoch();let deadline=f.retry_after;
        assert_eq!(f.admitted_generation,Some(generation));
        assert!(f.local_path.is_some(),"actual retained local path must participate in original affected check");
        publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
        let a_proof=ticket(&f,a);
        let step=tick_work(&mut f,(1,1));
        assert!(matches!(step,PlannerStep::Command(c) if c!=Twist2::ZERO),"{step:?}");
        let p2=f.submitted.unwrap().0;assert_eq!(p2,p1+1);
        assert_eq!(f.admitted_generation,Some(generation));assert_eq!(f.local_plans,admitted);
        assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);assert_eq!(f.retry_after,deadline);
        same_ticket(&ticket(&f,a),&a_proof);
        let step=release_and_step(&mut f,&gate);
        assert!(matches!(step,PlannerStep::Command(c) if c!=Twist2::ZERO),"{step:?}");
        assert_eq!(f.local_plans,admitted+1);assert_eq!(f.admitted_generation,Some(generation));
        assert_eq!(ids(&f),vec![p2]);
        let entry=gate.entry();assert!(!entry.detour);
        assert!(Arc::ptr_eq(&entry.source,f.map.as_ref().unwrap()));
        release_and_step(&mut f,&gate);assert_eq!(f.local_plans,admitted+2);
        assert_eq!(f.admitted_generation,Some(generation));f.stop();
    }
    #[test]
    fn pending_only_v2_original_related_single_20_10_cancels_then_normal_schedule_remains_legal() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        let gate=install(&mut f,ResultKind::Full);f.step(0.3);gate.entry();
        let a=f.submitted.unwrap().0;let calls=f.local_planning_calls;
        let work=f.pending_source_work_for_qualification();
        let old=f.map.clone().unwrap();
        publish_checked(&mut f,selective_update(&old,selective_scene(&[[20,10,5]]),&[[20,10]]),true);
        assert_eq!(f.pending_source_work_for_qualification(),work);
        assert_eq!(f.local_planning_calls,calls);
        // [20,10] stays at the ORIGINAL related point. Original cancellation
        // legitimately resets retry, so a full step may build one normal request.
        tick_work(&mut f,(0,1));
        let fresh=f.submitted.unwrap().0;assert_eq!(fresh,a+1);assert_eq!(ids(&f),vec![fresh]);
        assert_eq!(f.local_planning_calls,calls+1);assert!(f.local_path.is_none());
        f.stop();gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
        assert_eq!(f.local_plans,0);assert!(gate.entered.try_recv().is_err());
    }

    #[test]
    fn pending_only_v2_original_related_two_slot_points_keep_epoch_cancellation() {
        // These fixed points belong to three OLD rejected-policy fixtures.
        // They are not moved/reinterpreted as new disjoint positive coverage.
        for point in [[20,10,5],[10,10,5],[15,10,5]] {
            let mut f=open_follower();
            f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
            let gate=install(&mut f,ResultKind::Full);
            let (_a,_p,_)=two_tickets(&mut f,&gate);
            let old=f.map.clone().unwrap();let work=f.pending_source_work_for_qualification();
            publish_checked(&mut f,selective_update(&old,selective_scene(&[point]),&[[point[0],point[1]]]),true);
            assert_eq!(f.pending_source_work_for_qualification(),work);
            empty(&mut f);assert!(ids(&f).is_empty() && f.submitted.is_none());
            gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
            assert!(gate.entered.try_recv().is_err(),"old pending request cannot enter after original cancellation");
            f.consume_plan();assert_eq!(f.local_plans,0);assert!(f.admitted_generation.is_none());f.stop();
        }
    }

    #[test]
    fn pending_only_v2_dirty_a_with_clean_p_does_not_attempt_refresh() {
        let mut f=open_follower();
        f.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
        let gate=install(&mut f,ResultKind::Full);f.step(0.3);gate.entry();
        let a=f.submitted.unwrap().0;
        publish(&mut f,selective_scene(&[[100,100,5]]),&[[100,100]]);
        assert_eq!(ticket(&f,a).changes.len(),1);
        wait_retry(&f);
        f.update_odometry(comparison_pose(1.5,20));
        tick_work(&mut f,(0,1)); // Existing normal schedule creates P at latest source.
        let p=f.submitted.unwrap().0;assert_eq!(ids(&f),vec![a,p]);
        assert_eq!(ticket(&f,a).changes.len(),1);assert!(ticket(&f,p).changes.is_empty());
        let epoch=f.owner.as_ref().unwrap().epoch();let deadline=f.retry_after;
        let calls=f.local_planning_calls;let next=f.next_request;
        for _ in 0..3 {
            empty(&mut f);let proof=ticket(&f,a);
            tick_work(&mut f,(0,0));
            same_ticket(&ticket(&f,a),&proof);
            assert_eq!(ids(&f),vec![a,p]);assert!(ticket(&f,p).changes.is_empty());
            assert_eq!((f.next_request,f.local_planning_calls),(next,calls));
            assert_eq!(f.retry_after,deadline);assert_eq!(f.owner.as_ref().unwrap().epoch(),epoch);
        }
        f.stop();gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
    }

    #[test]
    fn pending_only_v2_empty_slots_preserve_the_original_initial_normal_submission() {
        let mut f=open_follower();let gate=install(&mut f,ResultKind::Full);
        assert!(f.submitted.is_none() && ids(&f).is_empty());
        let calls=f.local_planning_calls;
        assert!(matches!(tick_work(&mut f,(0,1)),PlannerStep::Pending(_)));
        assert_eq!(f.local_planning_calls,calls+1);assert_eq!(ids(&f).len(),1);
        gate.entry(); // This is initial submission + observed active, not initial-pending proof.
        f.stop();gate.release();wait_for(||f.owner.as_ref().unwrap().is_idle_for_test());
    }

}
