// 受控更新发生在昂贵准备阶段，使用实际事务、表面选择器和原生查询，不靠宿主速度构造时序。
mod continuous_map_transaction_tests {
    use super::*;

    struct UpdatingSource {
        inner: GatedRealSurfaceSource,
        expensive_gates: std::collections::VecDeque<Arc<OwnerRuntimeGate>>,
        allow_advice: bool,
        qualification_enabled: bool,
        gate_every_prepare: bool,
    }

    impl ExplorationGoalSource for UpdatingSource {
        fn source_name(&self) -> std::borrow::Cow<'static, str> {
            self.inner.source_name()
        }
        fn begin(&mut self) -> nuwax_exploration::ExploreResult<()> {
            self.inner.begin()
        }
        fn end(&mut self) -> nuwax_exploration::ExploreResult<()> {
            self.inner.end()
        }
        fn supports_prepared_poll(&self) -> bool {
            true
        }
        fn poll_goal(
            &mut self,
            state: &ExplorationStateSnapshot<'_>,
        ) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
            self.prepare_poll(state)?;
            self.commit_prepared_poll()
        }
        fn prepare_poll(
            &mut self,
            state: &ExplorationStateSnapshot<'_>,
        ) -> nuwax_exploration::ExploreResult<nuwax_exploration::PreparedExplorationPoll> {
            let prepared = if self.allow_advice && self.qualification_enabled {
                self.inner.prepare_poll(state)?
            } else {
                // 同一原输入的旧策略对照；重新绑定私有查询，不改几何、位姿或业务历史。
                let mut input = state
                    .frozen_surface_input
                    .unwrap()
                    .original_input()
                    .fork_for_planning();
                if self.allow_advice {
                    input = input.with_revalidation_advice();
                }
                let alternate = ExplorationStateSnapshot {
                    robot_pose: state.robot_pose.clone(),
                    now: state.now,
                    global_costmap: state.global_costmap,
                    active_goal: state.active_goal.clone(),
                    latest_frontier_goal: state.latest_frontier_goal.clone(),
                    frozen_surface_input: Some(&input),
                };
                self.inner.prepare_poll(&alternate)?
            };
            let expensive = match prepared.outcome() {
                Ok(ExplorationPoll::Goal(goal)) => {
                    goal.metadata["route_source_searches"]
                        .parse::<usize>()
                        .unwrap()
                        > 0
                }
                _ => true,
            };
            if expensive || self.gate_every_prepare {
                if let Some(gate) = self.expensive_gates.pop_front() {
                    gate.hold();
                }
            }
            Ok(prepared)
        }
        fn commit_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
            self.inner.commit_prepared_poll()
        }
        fn discard_prepared_poll(&mut self) -> nuwax_exploration::ExploreResult<()> {
            self.inner.discard_prepared_poll()
        }
        fn report_goal_feedback(
            &mut self,
            id: &str,
            feedback: ExplorationGoalFeedback,
        ) -> nuwax_exploration::ExploreResult<()> {
            self.inner.report_goal_feedback(id, feedback)
        }
        fn frontier_snapshot(&self) -> Vec<FrontierGoal> {
            self.inner.frontier_snapshot()
        }
        fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
            self.inner.diagnostics()
        }
        fn checkpoint(&self) -> nuwax_exploration::ExploreResult<String> {
            self.inner.checkpoint()
        }
    }

    async fn publish_unrelated_geometry(
        handle: &ExploreRuntimeHandle,
        message: &schema::NavigationMap3D,
        snapshot: &SurfaceSnapshot,
        committed: &mut schema::CommittedMapIdentity,
        revision: &mut i32,
    ) {
        *revision += 1;
        let rebuilt = SurfaceSnapshot::from_points(
            &schema::navigation_map_3d_points(message).unwrap(),
            snapshot.config().clone(),
            snapshot.source_voxel_size_m(),
        )
        .unwrap();
        let changed = Arc::new(
            rebuilt
                .with_recent_obstacle_points(
                    *revision as u64,
                    *revision as u64,
                    0,
                    &[],
                    [(0, [50.0 + f64::from(*revision), 50.0, 0.3])],
                )
                .unwrap(),
        );
        committed.revision += 1;
        committed.commit_sequence += 1;
        committed.source_timestamp_ns = now_unix_nanos() as i64;
        publish_surface_owner_receipt(handle, committed, *revision, changed).await;
    }

    #[derive(Clone, Copy)]
    enum MapChange { Unrelated, Target, Refinement, RouteBlocked }

    async fn publish_route_update(
        handle: &ExploreRuntimeHandle,
        message: &schema::NavigationMap3D,
        snapshot: &SurfaceSnapshot,
        audit: &Arc<StdMutex<SurfaceOwnerAudit>>,
        committed: &mut schema::CommittedMapIdentity,
        revision: &mut i32,
        blocked: bool,
    ) {
        *revision += 1;
        let (start, end) = {
            let guard = audit.lock().unwrap();
            let capture = guard.captures.iter().rev().find(|capture|
                matches!(&capture.preview, Ok(ExplorationPoll::Goal(_))))
                .expect("路线更新前须已有真实候选，Pending不是目标");
            let goal = surface_owner_goal(capture.preview.as_ref().unwrap());
            (capture.pose.pose.position, goal.pose.pose.position)
        };
        let middle = [(start.x + end.x) * 0.5, (start.y + end.y) * 0.5];
        let changed = if blocked {
            let axis = usize::from((end.y-start.y).abs() > (end.x-start.x).abs());
            let points: Vec<_> = (-20..=70).flat_map(|i| (2..=5).map(move |z| {
                let mut p = [middle[0],middle[1],f64::from(z)*0.1];
                p[1-axis] = f64::from(i)*0.1;
                p
            })).enumerate().map(|(i,p)| (i as u32,p)).collect();
            snapshot.with_recent_obstacle_points(*revision as u64,*revision as u64,0,&[],points).unwrap()
        } else {
            let points: Vec<_> = schema::navigation_map_3d_points(message).unwrap().into_iter().map(|(x,y,z)| {
                let near = (f64::from(x)-start.x).hypot(f64::from(y)-start.y) < 0.2
                    || (f64::from(x)-middle[0]).hypot(f64::from(y)-middle[1]) < 0.15;
                (x,y,z+if z < 1.0 && near { *revision as f32*0.001 } else { 0.0 })
            }).collect();
            let refined = SurfaceSnapshot::from_points(&points,snapshot.config().clone(),snapshot.source_voxel_size_m()).unwrap();
            assert_ne!(refined.navigation_measured_points(), snapshot.navigation_measured_points(),
                "起点和路线中段须实际细化实测高度");
            refined
        };
        if blocked {
            let offset = handle.inner.lock().await.surface_mode.as_ref().unwrap()
                .config.as_ref().unwrap().follower.body_to_support_z_m;
            let target = [end.x,end.y,end.z];
            let result = nuwax_nav_stack::surface_oriented::plan(&changed,
                ([start.x,start.y,start.z], [0.,0.,0.,1.]), (target,[0.,0.,0.,1.]),
                nuwax_nav_types_messages::GoalOptions::default(), offset, &[]);
            assert!(match result {
                Err(_) => true,
                Ok(route) => route.last().is_none_or(|p| (p.0[0]-end.x).hypot(p.0[1]-end.y).hypot(p.0[2]-end.z)
                    > f64::from(changed.config().voxel_size)*0.25),
            }, "横墙须实际阻断本轮所选候选的完整路线，不能只封堵第一轮旧路线");
        }
        committed.revision += 1;
        committed.commit_sequence += 1;
        committed.source_timestamp_ns = now_unix_nanos() as i64;
        publish_surface_owner_receipt(handle,committed,*revision,Arc::new(changed)).await;
    }

    async fn update_case(
        allow_advice: bool,
        change: MapChange,
        qualification_enabled: bool,
        gate_every_prepare: bool,
        interval: Duration,
        follow_up: bool,
        minimum_prepare: Duration,
    ) -> (bool, usize) {
        let change_target = matches!(change, MapChange::Target);
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request = "continuous-map-advice";
        handle.start(request, "prefer_wavefront").await;
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let gates: Vec<_> = (0..if follow_up { 6 } else { 3 })
            .map(|_| Arc::new(OwnerRuntimeGate::default()))
            .collect();
        let _releases: Vec<_> = gates.iter().cloned().map(OwnerRuntimeGateRelease).collect();
        let (receiver, config, offset) = {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            (
                mode.receiver.clone(),
                guard.active_config.as_deref().unwrap().clone(),
                mode.config.as_ref().unwrap().follower.body_to_support_z_m,
            )
        };
        let mut source = SurfaceGoalSourceAdapter::new("surface", offset, receiver).unwrap();
        source
            .configure_constraints(
                config.boundary,
                config.max_navigation_retries,
                config.blacklist_radius_m,
            )
            .unwrap();
        let mut controller =
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        controller.add_source(UpdatingSource {
            inner: GatedRealSurfaceSource {
                source,
                audit: audit.clone(),
                after_prepare: None,
                second_after_prepare: None,
                before_commit: None,
            },
            expensive_gates: gates.iter().cloned().collect(),
            allow_advice,
            qualification_enabled,
            gate_every_prepare,
        });
        controller.begin_exploration().unwrap();
        owner_watchdog(handle.install_compute_controller(controller))
            .await
            .unwrap();
        let provider = Arc::new(RecoveringPoseProvider {
            fresh: AtomicBool::new(true),
            pose: surface_pose(&snapshot, 0.0, Quaternion::IDENTITY),
        });
        let scheduler = handle.spawn_scheduler(provider, Duration::from_millis(5));
        let mut committed = message.committed_map.clone();
        let mut delivered = false;
        let mut original_wait = None;
        let mut handled_dispatches = 0;
        let mut gates_since_dispatch = 0;
        let mut publication_revision = 1;
        let mut published_maps = std::collections::BTreeMap::from([(1_u64, committed.clone())]);
        let mut timer = tokio::time::interval(interval.max(Duration::from_millis(1)));
        timer.tick().await;
        for (index, gate) in gates.iter().enumerate() {
            let entered = loop {
                let entered = tokio::time::timeout(Duration::from_secs(5), async {
                    let mut prepared = false;
                    let mut prepared_at = None;
                    loop {
                        tokio::select! {
                            biased;
                            _=async {loop {if sink.submitted.lock().unwrap().len()>handled_dispatches {break;}
                                tokio::task::yield_now().await;}}=>break false,
                            _=gate.entered(), if !prepared => {
                                prepared = true;
                                prepared_at = Some(Instant::now());
                                if interval.is_zero() { break true; }
                            },
                            _=timer.tick(), if !interval.is_zero() => {
                                // 接收节拍独立于计算；计算在途时地图仍每1～3秒更新。
                                if matches!(change, MapChange::Refinement | MapChange::RouteBlocked) {
                                    publish_route_update(&handle,&message,&snapshot,&audit,
                                        &mut committed,&mut publication_revision,matches!(change,MapChange::RouteBlocked)).await;
                                } else {
                                    publish_unrelated_geometry(&handle, &message, &snapshot,
                                        &mut committed, &mut publication_revision).await;
                                }
                                published_maps.insert(publication_revision as u64, committed.clone());
                                if prepared && prepared_at.is_some_and(|at| at.elapsed() >= minimum_prepare) { break true; }
                            },
                        }
                    }
                }).await.unwrap_or_else(|_| panic!("等待阶段超时：间隔={interval:?}，阶段={index}，已处理派发={handled_dispatches}，状态={:?}，最后候选={:?}",
                    handle.snapshot(),
                    audit.lock().unwrap().captures.last().map(|capture| &capture.preview)));
                if entered || !follow_up || handled_dispatches != 0 {
                    break entered;
                }
                owner_watchdog(async {
                    while handle.inner.lock().await.active_child.is_none() {
                        tokio::task::yield_now().await;
                    }
                })
                .await;
                let lease = active_child_lease(&handle).await;
                handle
                    .report_navigation_result(
                        &lease.session_id,
                        ChildNavigationResult::Reached(terminal_proof(&lease)),
                    )
                    .await
                    .unwrap();
                // 终态调用已等待原反馈确认；调度器可立即领取下一轮，不能再等待全局空闲。
                // 下一轮可能已持有下一个受控准备门，等待空闲会让夹具自己卡住该门。
                handled_dispatches = 1;
                gates_since_dispatch = 0;
                original_wait = None;
            };
            if !entered {
                delivered = true;
                break;
            }
            let since = handle.inner.lock().await.selection_wait_since.unwrap();
            if let Some(original) = original_wait {
                assert_eq!(since, original, "换图不能重启原进度期限");
            }
            original_wait = Some(since);
            gates_since_dispatch += 1;
            if interval.is_zero() && matches!(change,MapChange::Refinement | MapChange::RouteBlocked) {
                publish_route_update(&handle,&message,&snapshot,&audit,
                    &mut committed,&mut publication_revision,matches!(change,MapChange::RouteBlocked)).await;
            } else if interval.is_zero() && gate_every_prepare && !change_target {
                publish_unrelated_geometry(
                    &handle,
                    &message,
                    &snapshot,
                    &mut committed,
                    &mut publication_revision,
                )
                .await;
            } else if interval.is_zero() {
                let center = if change_target {
                    let guard = audit.lock().unwrap();
                    let goal = surface_owner_goal(
                        guard.captures.last().unwrap().preview.as_ref().unwrap(),
                    );
                    let p = goal.pose.pose.position;
                    [p.x, p.y, p.z - offset + 0.2]
                } else {
                    [50.0 + index as f64, 50.0, 0.3]
                };
                let points: Vec<_> = if change_target {
                    (-3..=3)
                        .flat_map(|x| {
                            (-3..=3).map(move |y| {
                                [
                                    center[0] + f64::from(x) * 0.1,
                                    center[1] + f64::from(y) * 0.1,
                                    center[2],
                                ]
                            })
                        })
                        .enumerate()
                        .map(|(index, point)| (u32::try_from(index).unwrap(), point))
                        .collect()
                } else {
                    vec![(0, center)]
                };
                let changed = Arc::new(
                    snapshot
                        .with_recent_obstacle_points(
                            index as u64 + 2,
                            index as u64 + 2,
                            0,
                            &[],
                            points,
                        )
                        .unwrap(),
                );
                committed.revision += 1;
                committed.commit_sequence += 1;
                committed.source_timestamp_ns += 1;
                publish_surface_owner_receipt(&handle, &committed, index as i32 + 2, changed).await;
                publication_revision = index as i32 + 2;
            }
            published_maps.insert(publication_revision as u64, committed.clone());
            if gates_since_dispatch == 3 {
                handle.inner.lock().await.selection_wait_since =
                    Some(Instant::now() - DEFAULT_SELECTION_PROGRESS_TIMEOUT);
                owner_watchdog(async {
                    while handle.snapshot().status != STATUS_FAILED {
                        tokio::task::yield_now().await;
                    }
                })
                .await;
                assert!(handle
                    .snapshot()
                    .message
                    .contains("exploration_selection_stalled"));
                assert!(sink.submitted.lock().unwrap().is_empty());
            }
            gate.release();
        }
        for gate in &gates {
            gate.release();
        }
        if delivered {
            owner_watchdog(async {
                while handle.inner.lock().await.active_child.is_none() {
                    tokio::task::yield_now().await;
                }
            })
            .await;
            assert_eq!(
                sink.submitted.lock().unwrap().len(),
                if follow_up { 2 } else { 1 }
            );
            if gate_every_prepare {
                assert!(
                    audit.lock().unwrap().captures.iter().any(
                        |capture| matches!(&capture.preview, Ok(ExplorationPoll::Goal(goal))
                        if goal.metadata["route_exact_queries"].parse::<usize>().unwrap() > 0)
                    ),
                    "候选须经过真实原生精确路线查询"
                );
            }
            let submitted_map = sink
                .submitted
                .lock()
                .unwrap()
                .last()
                .unwrap()
                .intent
                .committed_map
                .clone();
            let captured = audit.lock().unwrap().captures.last().unwrap().original.notification();
            assert!(publication_revision as u64 > captured,
                "完整查询产出后必须实际再换图，不能退化为原查询相等");
            assert_eq!(submitted_map, published_maps[&captured],
                "当前输入只复核资格，请求必须保留实际冻结计算身份");
            assert!(crate::nav_runtime::same_committed_map_domain(&submitted_map, &committed));
            stop_owner_runtime_child(&handle, request).await;
        } else {
            handle.stop(request).await;
        }
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
        let discards = audit.lock().unwrap().discards;
        owner_watchdog(scheduler.shutdown()).await.unwrap();
        (delivered, discards)
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn changing_unrelated_maps_revalidate_then_dispatch_without_accepting_old_transaction() {
        let old = update_case(false, MapChange::Unrelated, true, false, Duration::ZERO, false, Duration::ZERO).await;
        let new = update_case(true, MapChange::Unrelated, true, false, Duration::ZERO, false, Duration::ZERO).await;
        assert!(!old.0 && old.1 >= 2, "旧策略应在受控持续换图下反复撤销");
        assert!(
            new.0 && new.1 == 0,
            "完整查询须直接完成当前输入复核并派发，不能先撤销一轮"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn target_changes_still_discard_results_and_keep_original_stall_deadline() {
        let result = update_case(true, MapChange::Target, true, true, Duration::ZERO, false, Duration::ZERO).await;
        assert!(
            !result.0 && result.1 >= 2,
            "实体持续覆盖候选时不得提交过时结果"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn completed_revalidation_dispatches_with_unrelated_updates_every_one_to_three_seconds() {
        let old = update_case(true, MapChange::Unrelated, false, true, Duration::ZERO, false, Duration::ZERO).await;
        assert!(
            !old.0 && old.1 >= 2,
            "原资格即使完成候选复核仍反复撤销并在原期限结束"
        );
        for seconds in 1..=3 {
            let current =
                update_case(true, MapChange::Unrelated, true, true, Duration::from_secs(seconds), true, Duration::ZERO).await;
            assert!(
                current.0 && current.1 == 0,
                "每{seconds}秒更新无关区域时，重核候选必须进入实际子导航派发"
            );
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn continuous_start_and_route_refinement_dispatches_within_original_deadline() {
        for seconds in 1..=3 {
            let result = update_case(true,MapChange::Refinement,true,true,Duration::from_secs(seconds),true,Duration::ZERO).await;
            assert!(result.0 && result.1 == 0,
                "起点及沿线每{seconds}秒细化后，应完成复核并持续派发两个子导航");
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn crossing_route_obstacles_withhold_and_reevaluate_without_resetting_deadline() {
        let result = update_case(true,MapChange::RouteBlocked,true,true,Duration::ZERO,false,Duration::ZERO).await;
        assert!(!result.0 && result.1 >= 2,
            "候选完整复核后，沿线新实体应扣下结果并重新评估，原期限内不得误派发");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn multi_second_full_queries_dispatch_continuously_during_one_to_three_hz_refinement() {
        let baseline_started = Instant::now();
        let baseline = update_case(true, MapChange::Refinement, false, true,
            Duration::from_millis(333), false, Duration::from_secs(3)).await;
        assert!(!baseline.0 && baseline.1 >= 2,
            "相同真实细化和慢准备输入下，关闭所选结果依据的原策略须撤销并触发原有界终止");
        println!("原策略：交付=0，撤销={}，运行={:?}，第三轮显式推进到原停滞边界", baseline.1, baseline_started.elapsed());
        for hz in 1..=3 {
            let started = Instant::now();
            let result = update_case(true, MapChange::Refinement, true, true,
                Duration::from_millis(1000 / hz), true, Duration::from_secs(3)).await;
            assert!(result.0 && result.1 == 0,
                "每秒{hz}次真实沿线细化、每轮至少3秒时须直接提交并连续派发两个子导航");
            println!("持续细化 hz={hz}，完整查询最短3秒，撤销={}，两次交付耗时={:?}", result.1, started.elapsed());
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn three_second_recheck_keeps_receipts_and_costmap_fanout_live() {
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request = "slow-selected-recheck";
        handle.start(request, "prefer_wavefront").await;
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let prepared = Arc::new(OwnerRuntimeGate::default());
        let _release = OwnerRuntimeGateRelease(prepared.clone());
        install_gated_real_surface_source(&handle, audit.clone(), prepared.clone(), None).await;
        let entered = Arc::new(tokio::sync::Notify::new());
        handle.compute_service.set_qualification_hook(Arc::new({
            let entered = entered.clone();
            let first = AtomicBool::new(true);
            move || { if first.swap(false, Ordering::SeqCst) {
                entered.notify_one(); std::thread::sleep(Duration::from_secs(3));
            } }
        }));
        let tick = tokio::spawn({
            let handle = handle.clone();
            let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
            async move { handle.tick_with_pose(pose).await }
        });
        owner_watchdog(prepared.entered()).await;
        let mut committed = message.committed_map.clone();
        let mut revision = 1;
        publish_unrelated_geometry(&handle, &message, &snapshot, &mut committed, &mut revision).await;
        prepared.release();
        owner_watchdog(entered.notified()).await;
        let started = Instant::now();
        let mut maximum_receipt = Duration::ZERO;
        let mut maximum_fanout = Duration::ZERO;
        while started.elapsed() < Duration::from_secs(3) {
            revision += 1;
            let next = surface_message("map", revision);
            let at = Instant::now();
            tokio::time::timeout(Duration::from_millis(200),
                handle.begin_surface_map_update(&next, now_monotonic_nanos())).await.unwrap().unwrap();
            maximum_receipt = maximum_receipt.max(at.elapsed());
            let at = Instant::now();
            tokio::time::timeout(Duration::from_millis(200),
                handle.ingest_committed_costmap_snapshot_in_frame(frontier_costmap(1), next.committed_map, 1)).await.unwrap().unwrap();
            maximum_fanout = maximum_fanout.max(at.elapsed());
            tokio::time::sleep(Duration::from_millis(333)).await;
        }
        let result = owner_watchdog(tick).await.unwrap();
        assert_eq!(result.status, STATUS_ACTIVE, "{}", result.message);
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        assert_eq!(audit.lock().unwrap().discards, 0);
        println!("复核至少3秒，接收最长={maximum_receipt:?}，二维分发最长={maximum_fanout:?}，交付={:?}", started.elapsed());
        stop_owner_runtime_child(&handle, request).await;
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn real_surface_consumer_receives_while_completed_install_waits_for_recheck() {
        use futures::FutureExt;
        use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
        let (handle, _) = configured_handle();
        let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
        let result = std::panic::AssertUnwindSafe(async {
            input.wait_finished(input.first.cloud_revision).await.unwrap();
            let install = handle.surface_install_gate.lock().await;
            let waiting = input.prepare_complete_candidate().await.unwrap();
            input.publish_candidate(&waiting).await.unwrap();
            f1_wait_latest(&handle, &waiting).await;
            // 读取原消费者实际收到构建结果的事件，不能靠睡眠推断已完成计算。
            let pending = format!("surface-map-install-pending-cloud:{}", waiting.cloud_revision);
            owner_watchdog(async {
                while !input.probe.events().iter().any(|event| event == &pending) {
                    tokio::task::yield_now().await;
                }
            }).await;
            for _ in 0..3 {
                let next = input.prepare_complete_candidate().await.unwrap();
                input.publish_candidate(&next).await.unwrap();
                tokio::time::timeout(Duration::from_millis(500), f1_wait_latest(&handle, &next)).await.unwrap();
                let guard = handle.inner.lock().await;
                let mode = guard.surface_mode.as_ref().unwrap();
                assert_eq!(mode.installed_revision, Some((input.first.cloud_revision, input.first.committed_map.revision)));
                assert!(guard.map_input_is_fresh(now_monotonic_nanos()));
                drop(guard);
                tokio::time::sleep(Duration::from_millis(333)).await;
            }
            let final_revision = handle.inner.lock().await.surface_mode.as_ref().unwrap().latest.as_ref().unwrap().cloud_revision;
            drop(install);
            input.wait_finished(final_revision).await.unwrap();
            assert_eq!(handle.inner.lock().await.surface_mode.as_ref().unwrap().installed_revision.unwrap().0, final_revision);
        }).catch_unwind().await;
        f1_finish_all_owners(&mut input, &handle, "install-source-live").await;
        if let Err(panic) = result { std::panic::resume_unwind(panic); }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn cancelled_delivery_retains_recheck_until_original_owner_shutdown() {
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request = "cancel-selected-recheck";
        handle.start(request, "prefer_wavefront").await;
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let prepared = Arc::new(OwnerRuntimeGate::default());
        let recheck = Arc::new(OwnerRuntimeGate::default());
        let _release_prepare = OwnerRuntimeGateRelease(prepared.clone());
        let _release_recheck = OwnerRuntimeGateRelease(recheck.clone());
        install_gated_real_surface_source(&handle, audit, prepared.clone(), None).await;
        handle.compute_service.set_qualification_hook(Arc::new({
            let recheck = recheck.clone(); move || recheck.hold()
        }));
        let tick = tokio::spawn({
            let handle = handle.clone();
            let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
            async move { handle.tick_with_pose(pose).await }
        });
        owner_watchdog(prepared.entered()).await;
        let mut committed = message.committed_map.clone();
        let mut revision = 1;
        publish_unrelated_geometry(&handle, &message, &snapshot, &mut committed, &mut revision).await;
        prepared.release();
        owner_watchdog(recheck.entered()).await;
        tick.abort();
        assert!(tick.await.unwrap_err().is_cancelled());
        handle.request_compute_shutdown().await;
        let joined = tokio::spawn({
            let handle = handle.clone(); async move { handle.compute_service.join().await }
        });
        tokio::task::yield_now().await;
        assert!(!joined.is_finished(), "不能在原阻塞复核仍持有地图时宣称关闭完成");
        recheck.release();
        owner_watchdog(joined).await.unwrap().unwrap();
        assert!(sink.submitted.lock().unwrap().is_empty(), "取消后不得交付候选");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn failed_recheck_uses_original_fault_drain_and_joins_worker() {
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request = "failed-selected-recheck";
        handle.start(request, "prefer_wavefront").await;
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let prepared = Arc::new(OwnerRuntimeGate::default());
        let _release = OwnerRuntimeGateRelease(prepared.clone());
        install_gated_real_surface_source(&handle, audit.clone(), prepared.clone(), None).await;
        let calls = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        handle.compute_service.set_qualification_hook(Arc::new({
            let calls = calls.clone();
            move || { calls.fetch_add(1, Ordering::SeqCst); panic!("注入当前输入复核工作错误"); }
        }));
        let tick = tokio::spawn({
            let handle = handle.clone();
            let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
            async move { handle.tick_with_pose(pose).await }
        });
        owner_watchdog(prepared.entered()).await;
        let mut committed = message.committed_map.clone();
        let mut revision = 1;
        publish_unrelated_geometry(&handle, &message, &snapshot, &mut committed, &mut revision).await;
        prepared.release();
        let failed = owner_watchdog(tick).await.unwrap();
        assert_eq!(failed.status, STATUS_FAILED, "{}", failed.message);
        let joined = owner_watchdog(handle.compute_service.join()).await;
        assert!(joined.is_err(), "工作错误必须进入原故障终态，不能伪装为正常关闭");
        let guard = handle.inner.lock().await;
        assert!(!guard.logical_active);
        assert!(guard.compute.fault.as_ref().unwrap().contains("qualification join failed"));
        assert!(guard.compute.status().unwrap().closed);
        assert_eq!(calls.load(Ordering::SeqCst), 1, "故障后不能重新发起复核");
        assert_eq!(audit.lock().unwrap().discards, 1);
        assert!(sink.submitted.lock().unwrap().is_empty());
    }

}
