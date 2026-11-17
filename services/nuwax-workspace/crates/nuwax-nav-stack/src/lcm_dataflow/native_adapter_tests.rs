    static NATIVE_GENERATION_TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    use super::*;

    fn goal() -> schema::NavigationGoal {
        let mut goal = schema::NavigationGoal {
session_id: "native-session".into(),
boundary_points: vec![
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 5.0, 0.0],
                [0.0, 5.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
motion_generation: 8,
goal: schema::PoseStamped {
                header: schema::Header {
                    seq: 9,
                    stamp_sec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [4.0, 4.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
..schema::NavigationGoal::default()
};

        schema::validate_navigation_goal(&goal).expect("seal native goal");
        goal
    }

    fn path(points: Vec<Point2>) -> Path2D {
        Path2D::stamped(NavFrameId::Map, Timestamp::from_unix(7, 0), points)
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    fn goal_generation(generation: i64) -> schema::NavigationGoal {
        let mut value = goal();
        value.motion_generation = generation as u64;
        value.goal.header.seq = i32::try_from(generation).expect("test generation fits i32");
        value.goal.header.stamp_sec = i32::try_from(generation).expect("test generation fits i32");
        schema::validate_navigation_goal(&value).expect("reseal native goal generation");
        value
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    fn far_key(goal: &schema::NavigationGoal) -> FarJoinKey {
        FarJoinKey {
            backend: "far-visibility-graph".into(),
            runtime_generation: 12,
            session_id: goal.session_id.clone(),
            motion_generation: goal.motion_generation,
            path_revision: 4,
            planner_runtime_generation: 12,
        }
    }

    #[cfg(all(
        feature = "native-runner",
        any(feature = "cmu-native", feature = "explore")
    ))]



    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn duplicate_goal_preserves_far_join_and_consumed_state() {
        let goal = goal_generation(9);
        let key = far_key(&goal);
        let mut latest = None;
        assert!(admit_native_adapter_goal(&mut latest, goal.clone()));
        let mut pending_results = HashMap::from([(
            key.clone(),
            path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]),
        )]);
        let mut pending_terminals = HashSet::new();
        let mut consumed = HashSet::new();

        assert!(!admit_native_adapter_goal(&mut latest, goal));
        pending_terminals.insert(key.clone());
        let mut published = usize::from(
            take_far_join(
                &key,
                &mut pending_results,
                &mut pending_terminals,
                &mut consumed,
            )
            .is_some(),
        );
        pending_terminals.insert(key.clone());
        published += usize::from(
            take_far_join(
                &key,
                &mut pending_results,
                &mut pending_terminals,
                &mut consumed,
            )
            .is_some(),
        );
        assert_eq!(published, 1);
        assert!(consumed.contains(&key));
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn same_motion_generation_keeps_distinct_far_goal_identity() {
        let first=goal_generation(9);
        let mut current=first.clone();current.session_id="next-child".into();
        let old_key=far_key(&first);let key=far_key(&current);
        let mut latest=None;
        assert!(admit_native_adapter_goal(&mut latest,first));
        assert!(admit_native_adapter_goal(&mut latest,current.clone()));
        assert!(!far_key_matches_goal(&old_key,&current,12));
        assert!(far_key_matches_goal(&key,&current,12));
        let mut results=HashMap::from([(key.clone(),path(vec![Point2::new(1.,1.),Point2::new(4.,4.)]))]);
        let mut terminals=HashSet::from([old_key.clone()]);let mut consumed=HashSet::new();
        assert!(take_far_join(&old_key,&mut results,&mut terminals,&mut consumed).is_none());
        terminals.insert(key.clone());
        assert!(take_far_join(&key,&mut results,&mut terminals,&mut consumed).is_some());
        assert_eq!(latest,Some(current));
    }

    #[test]
    fn native_path_gate_rejects_wrong_map_boundary_and_frame() {
        let goal = goal();
        let inside = path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]);
        assert!(native_path_respects_goal(&inside, &goal));

        let outside = path(vec![Point2::new(1.0, 1.0), Point2::new(6.0, 4.0)]);
        assert!(!native_path_respects_goal(&outside, &goal));

        let wrong_path_frame = Path2D::stamped(
            NavFrameId::Odom,
            Timestamp::from_unix(7, 0),
            inside.points.clone(),
        );
        assert!(!native_path_respects_goal(&wrong_path_frame, &goal));

        let mut wrong_map = goal.clone();
        wrong_map.goal.header.frame_id = "odom".into();
        assert!(!native_path_respects_goal(&inside, &wrong_map));

    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn far_terminal_gate_rejects_old_future_and_wrong_goal_identity() {
        let goal = goal();
        let state = schema::NavigationStatus {
            session_id: goal.session_id.clone(),
            robot_frame: "far-visibility-graph".into(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED,
            runtime_generation: 12,
            motion_generation: goal.motion_generation as i64,
            path_revision: 4,
            planner_runtime_generation: 12,
            ..schema::NavigationStatus::default()
        };
        assert!(far_terminal_matches_goal(&state, &goal, 12));
        assert!(!far_terminal_matches_goal(&state, &goal, 11));
        assert!(!far_terminal_matches_goal(&state, &goal, 13));

        let mut wrong_goal = goal;
        wrong_goal.motion_generation += 1;
        schema::validate_navigation_goal(&wrong_goal).expect("reseal wrong goal");
        assert!(!far_terminal_matches_goal(&state, &wrong_goal, 12));
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[test]
    fn far_join_wrong_key_does_not_consume_matching_pending_result() {
        let goal = goal();
        let good = FarJoinKey {
            backend: "far-visibility-graph".into(),
            runtime_generation: 12,
            session_id: goal.session_id.clone(),
            motion_generation: goal.motion_generation,
            path_revision: 4,
            planner_runtime_generation: 12,
        };
        let mut pending_results = HashMap::from([(
            good.clone(),
            path(vec![Point2::new(1.0, 1.0), Point2::new(4.0, 4.0)]),
        )]);
        let mut pending_terminals = HashSet::new();
        let mut consumed = HashSet::new();

        let mut wrong_revision = good.clone();
        wrong_revision.path_revision += 1;
        pending_terminals.insert(wrong_revision.clone());
        assert!(take_far_join(
            &wrong_revision,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_none());
        assert!(pending_results.contains_key(&good));

        let mut wrong_backend = good.clone();
        wrong_backend.backend = "another-backend".into();
        pending_terminals.insert(wrong_backend.clone());
        assert!(take_far_join(
            &wrong_backend,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_none());
        assert!(pending_results.contains_key(&good));

        pending_terminals.insert(good.clone());
        assert!(take_far_join(
            &good,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_some());
        assert!(take_far_join(
            &good,
            &mut pending_results,
            &mut pending_terminals,
            &mut consumed,
        )
        .is_none());
    }

    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    #[tokio::test]
    async fn far_real_lcm_adapter_pairs_ready_path_and_terminal_without_crossing_headers() {
        static NEXT_ADAPTER_ID: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let id = NEXT_ADAPTER_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let prefix = format!("/t/fa/{}/{id}", std::process::id());
        let node = TaskChannels { id: format!("far-adapter-{id}"), inputs: vec![format!("{prefix}/g"),format!("{prefix}/e"),format!("{prefix}/p")], outputs: vec![format!("{prefix}/o")] };
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq transport");
        let mut paths = typed_stream(&transport, &node.outputs[0], "nuwax_msgs.PathEnvelope")
            .await
            .expect("canonical path subscription");
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (ready_tx, ready_rx) = std_mpsc::sync_channel(1);
        let owner = tokio::spawn(run_native_planner_path_adapter_with_transport(
            node.clone(),
            Arc::from("base_link"),
            transport.clone(),
            shutdown_rx,
            ready_tx,
        ));
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                match ready_rx.try_recv() {
                    Ok(Ok(())) => break,
                    Ok(Err(reason)) => panic!("FAR adapter startup failed: {reason}"),
                    Err(std_mpsc::TryRecvError::Empty) => {
                        assert!(!owner.is_finished(), "FAR adapter exited before ready");
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                    Err(std_mpsc::TryRecvError::Disconnected) => {
                        panic!("FAR adapter dropped readiness channel")
                    }
                }
            }
        })
        .await
        .expect("FAR adapter readiness timeout");

        let mut goal = goal();
        goal.motion_generation=u64::MAX;
        publish_body(
            &transport,
            &node.inputs[0],
            "nuwax_msgs.NavigationGoal",
            schema::encode_navigation_goal_body(&goal).expect("encode FAR goal"),
        )
        .await
        .expect("publish FAR goal");
        FAR_READY_GENERATION.store(12, Ordering::Release);

        let state = |revision| schema::NavigationStatus {
            session_id: goal.session_id.clone(),
            robot_frame: "far-visibility-graph".into(),
            state: nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED,
            runtime_generation: 12,
            planner_map_frame: goal.goal.header.frame_id.clone(),
            planner_pose_frame: goal.goal.header.frame_id.clone(),
            motion_generation: goal.motion_generation as i64,
            path_revision: revision,
            planner_runtime_generation: 12,
            ..schema::NavigationStatus::default()
        };

        publish_body(
            &transport,
            &node.inputs[2],
            "nav_msgs.Path",
            schema::encode_path_body(&schema::Path {
                header: goal.goal.header.clone(),
                poses: vec![goal.goal.clone()],
            })
            .expect("encode legacy FAR path"),
        )
        .await
        .expect("publish legacy FAR path");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), paths.next())
                .await
                .is_err(),
            "legacy raw FAR path must not become canonical"
        );

        let mut wrong_header = goal.goal.header.clone();
        wrong_header.frame_id = "odom".into();
        let wrong_result = nuwax_nav_far_planner_bridge::worker::seal_private_result_v3(
            state(1),
            &schema::Path {
                header: wrong_header.clone(),
                poses: vec![schema::PoseStamped {
                    header: wrong_header,
                    pose: goal.goal.pose.clone(),
                }],
            },
        )
        .expect("seal wrong-header FAR result");
        let mut wrong_terminal = wrong_result.clone();
        wrong_terminal.message = "succeeded:private-result-v3-published".into();
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&wrong_result).expect("encode wrong FAR result"),
        )
        .await
        .expect("publish wrong FAR result");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&wrong_terminal)
                .expect("encode wrong FAR terminal"),
        )
        .await
        .expect("publish wrong FAR terminal");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), paths.next())
                .await
                .is_err(),
            "wrong-header FAR result must not become canonical"
        );

        let accepted_result = nuwax_nav_far_planner_bridge::worker::seal_private_result_v3(
            state(2),
            &schema::Path {
                header: goal.goal.header.clone(),
                poses: vec![goal.goal.clone()],
            },
        )
        .expect("seal accepted FAR result");
        let mut accepted_terminal = accepted_result.clone();
        accepted_terminal.message = "succeeded:private-result-v3-published".into();
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&accepted_terminal)
                .expect("encode accepted FAR terminal"),
        )
        .await
        .expect("publish accepted FAR terminal first");
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&accepted_result)
                .expect("encode accepted FAR result"),
        )
        .await
        .expect("publish accepted FAR result second");
        let frame = tokio::time::timeout(Duration::from_secs(2), paths.next())
            .await
            .expect("FAR canonical path timeout")
            .expect("FAR canonical path stream closed")
            .expect("FAR canonical path transport");
        let envelope = schema::decode_path_envelope_body(&frame.payload)
            .expect("decode FAR canonical PathEnvelope");
        assert_eq!(envelope.goal_key(), goal.goal_key());
        assert_eq!(envelope.planner_runtime_generation, 12);
        assert_eq!(envelope.path_revision, 2);

        let result_first = nuwax_nav_far_planner_bridge::worker::seal_private_result_v3(
            state(3),
            &schema::Path {
                header: goal.goal.header.clone(),
                poses: vec![goal.goal.clone()],
            },
        )
        .expect("seal result-first FAR result");
        let mut terminal_second = result_first.clone();
        terminal_second.message = "succeeded:private-result-v3-published".into();
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&result_first)
                .expect("encode result-first FAR result"),
        )
        .await
        .expect("publish FAR result first");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&terminal_second)
                .expect("encode terminal-second FAR event"),
        )
        .await
        .expect("publish FAR terminal second");
        let frame = tokio::time::timeout(Duration::from_secs(2), paths.next())
            .await
            .expect("result-first FAR canonical path timeout")
            .expect("FAR canonical path stream closed")
            .expect("FAR canonical path transport");
        let envelope = schema::decode_path_envelope_body(&frame.payload)
            .expect("decode result-first FAR canonical PathEnvelope");
        assert_eq!(envelope.path_revision, 3);

        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&result_first).expect("encode replay FAR result"),
        )
        .await
        .expect("publish replay FAR result");
        publish_body(
            &transport,
            &node.inputs[1],
            "nuwax_msgs.NavigationStatus",
            schema::encode_navigation_status_body(&terminal_second)
                .expect("encode replay FAR terminal"),
        )
        .await
        .expect("publish replay FAR terminal");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), paths.next())
                .await
                .is_err(),
            "replayed FAR pair must publish at most once"
        );

        shutdown_tx.send(()).expect("shutdown FAR adapter");
        owner
            .await
            .expect("join FAR adapter")
            .expect("FAR adapter shutdown");
    }

    #[test]
    fn restart_generation_is_cleared_before_a_new_ready_generation() {
        let _generation_guard=NATIVE_GENERATION_TEST_LOCK.lock().unwrap();
        #[cfg(any(feature = "cmu-native", feature = "explore"))]
        {
            FAR_READY_GENERATION.store(12, Ordering::Release);
            assert_eq!(native_ready_generation(), 12);
            invalidate_native_planner_generation("far_planner");
            assert_eq!(native_ready_generation(), 0);
            FAR_READY_GENERATION.store(13, Ordering::Release);
            assert_eq!(native_ready_generation(), 13);
        }

    }
