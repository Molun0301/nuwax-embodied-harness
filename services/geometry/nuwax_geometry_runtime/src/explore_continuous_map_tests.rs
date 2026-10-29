// 受控更新发生在昂贵准备阶段，使用实际事务、表面选择器和原生查询，不靠宿主速度构造时序。
mod continuous_map_transaction_tests {
    use super::*;

    struct UpdatingSource {
        inner: GatedRealSurfaceSource,
        expensive_gates: std::collections::VecDeque<Arc<OwnerRuntimeGate>>,
        allow_advice: bool,
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
            let prepared = if self.allow_advice {
                self.inner.prepare_poll(state)?
            } else {
                // 同一原输入的旧策略对照；重新绑定私有查询，不改几何、位姿或业务历史。
                let input = state
                    .frozen_surface_input
                    .unwrap()
                    .original_input()
                    .fork_for_planning();
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
            if expensive {
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

    async fn update_case(allow_advice: bool, change_target: bool) -> (bool, usize) {
        let (handle, sink) = configured_handle();
        let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
        let request = "continuous-map-advice";
        handle.start(request, "prefer_wavefront").await;
        let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
        let gates: Vec<_> = (0..3)
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
        let mut committed = message.committed_map;
        let mut delivered = false;
        let mut original_wait = None;
        for (index, gate) in gates.iter().enumerate() {
            let entered = owner_watchdog(async {
                tokio::select! {
                    _=gate.entered()=>true,
                    _=async {loop {if !sink.submitted.lock().unwrap().is_empty(){break;}
                        tokio::task::yield_now().await;}}=>false,
                }
            })
            .await;
            if !entered {
                delivered = true;
                break;
            }
            let since = handle.inner.lock().await.selection_wait_since.unwrap();
            if let Some(original) = original_wait {
                assert_eq!(since, original, "换图不能重启原进度期限");
            }
            original_wait = Some(since);
            let center = if change_target {
                let guard = audit.lock().unwrap();
                let goal =
                    surface_owner_goal(guard.captures.last().unwrap().preview.as_ref().unwrap());
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
                    .with_recent_obstacle_points(index as u64 + 2, index as u64 + 2, 0, &[], points)
                    .unwrap(),
            );
            committed.revision += 1;
            committed.commit_sequence += 1;
            committed.source_timestamp_ns += 1;
            publish_surface_owner_receipt(&handle, &committed, index as i32 + 2, changed).await;
            if index == 2 {
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
            assert_eq!(sink.submitted.lock().unwrap().len(), 1);
            assert_eq!(
                sink.submitted.lock().unwrap()[0].intent.committed_map,
                committed,
                "新目标须保留当前核验输入的提交身份"
            );
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
        let old = update_case(false, false).await;
        let new = update_case(true, false).await;
        assert!(!old.0 && old.1 >= 2, "旧策略应在受控持续换图下反复撤销");
        assert!(
            new.0 && new.1 >= 1,
            "旧事务撤销后应以当前输入完成新核验并派发一次"
        );
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn target_changes_still_discard_results_and_keep_original_stall_deadline() {
        let result = update_case(true, true).await;
        assert!(
            !result.0 && result.1 >= 2,
            "实体持续覆盖候选时不得提交过时结果"
        );
    }
}
