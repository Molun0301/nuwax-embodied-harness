// 验证运行时拥有方的状态边界；地面成员和高度合同由共享层定向回归覆盖。
fn owner_pose(x: f64, tick: i64) -> PoseStamped {
    PoseStamped {
        frame_id: nuwax_nav_types_messages::NavFrameId::Map,
        stamp: nuwax_nav_types_messages::Timestamp::from_unix(1 + tick, 0),
        pose: Pose3 { position: Point3::new(x, 1.05, 0.4), orientation: Quaternion::IDENTITY },
    }
}

fn owner_follower(source: Arc<SurfaceSnapshot>) -> SurfaceFollower {
    let mut config = SurfaceFollowerConfig::new(0.3);
    config.controller.simulation = true;
    config.motion_limits = Some(MotionLimits {
        model: nuwax_nav_types_messages::MotionModel::Differential,
        max_forward_mps: 0.5, max_reverse_mps: 0.0, max_lateral_mps: 0.0,
        max_angular_rps: 0.8, linear_acceleration_mps2: 1.0,
        guaranteed_braking_mps2: 1.0, angular_acceleration_rps2: 2.0,
        guaranteed_angular_braking_rps2: 2.0, reaction_time_sec: 0.05,
        minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
    });
    let mut follower = SurfaceFollower::new(config).unwrap();
    follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    follower.update_odometry(owner_pose(1.05, 0));
    follower.refresh_current_body_view();
    follower
}

#[test]
fn runtime_ground_owner_predictions_do_not_consume_actual_association() {
    let source = Arc::new(map(0.0));
    let mut follower = owner_follower(source.clone());
    let actual = owner_pose(1.05, 0);
    let target = owner_pose(1.45, 0);
    let (query, initial) = follower.current_ground_query().map(|(q, g)| (q.clone(), g.clone())).unwrap();
    let (_, predicted) = connection_endpoint_in_query(&source, &query, body(&actual), &initial, &target)
        .expect("平地连接可执行");
    assert_ne!(predicted, initial, "预测确实进入另一源地面列");
    assert_eq!(follower.current_ground_query().unwrap().1, &initial,
        "计划连接末点不能被当作实际已到层");
    follower.with_body_motion_checks(|clear, period, _, _| {
        assert!(clear(planar(&actual), planar(&actual), 0.0));
        assert!(clear(planar(&actual), planar(&target), 0.2));
        assert!(period(planar(&actual), planar(&actual), 0.0));
        assert!(period(planar(&actual), planar(&target), 0.2));
    }).unwrap();
    assert_eq!(follower.current_ground_query().unwrap().1, &initial,
        "制动细分与整拍预测都不得回写实际关联");
    follower.update_odometry(owner_pose(1.45, 1));
    follower.refresh_current_body_view();
    assert_ne!(follower.current_ground_query().unwrap().1, &initial,
        "新实际位姿才消费新地面列");
    assert!(follower.current_ground_query().unwrap().0.same_body_query_view(&source));
}

#[test]
fn runtime_ground_owner_memo_distinguishes_future_ground_dependencies() {
    let source = map(0.0);
    let query = source.body_query();
    let from = body(&owner_pose(1.05, 0));
    let known = query.start_ground_state(from).unwrap();
    let unknown = query.start_ground_state(([100.0, 100.0, 0.4], from.1)).unwrap();
    assert_ne!(known, unknown);
    let cache = RefCell::new(HashMap::new());
    let proof = ObservationProofAccumulator::new(None);
    let target = planar(&owner_pose(1.15, 0));
    let calls = Cell::new(0);
    for state in [&known, &unknown, &known, &unknown] {
        let result: Result<_, nuwax_nav_mls_planner::SurfaceError> = memoized_body_motion_impl(
            &cache, &proof, from, target, false, None, Some(state), || {
                calls.set(calls.get() + 1);
                Ok(BodyMotionSuccess { pose: from, ground: Some(state.clone()), latest_start_ns: None })
            });
        assert_eq!(result.unwrap().ground.as_ref(), Some(state));
    }
    assert_eq!(calls.get(), 2, "同姿态但未来地面依赖不同不能合并，重复状态应复用");
}

#[test]
fn runtime_ground_owner_recent_wall_rechecks_delay_and_full_stop_tail() {
    let clear = Arc::new(map(0.0));
    let mut follower = owner_follower(clear.clone());
    assert!(follower.current_body_error().is_none());
    let previous_query = follower.current_ground_query().unwrap().0.clone();
    let moving = Twist2 { linear: Point2::new(0.5, 0.0), angular_z: 0.0 };
    assert!(follower.predicted_body_motion_is_traversable(moving, 0.5));
    let blocked = Arc::new(clear.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
        [(0, [1.3, 1.05, 0.4])]).unwrap());
    follower.update_execution_map(ExecutionMap::Surface(blocked.clone())).unwrap();
    follower.observe_applied_command(moving);
    assert!(follower.current_body_error().is_none(), "前方障碍未进入当前机体");
    assert!(!previous_query.same_body_query_view(&blocked));
    assert!(follower.current_ground_query().unwrap().0.same_body_query_view(&blocked));
    assert!(!follower.predicted_body_motion_is_traversable(moving, 0.5),
        "新近期障碍必须进入基础延迟轨迹检查");
    assert!(!follower.check_selected_candidate_for_test(moving),
        "新近期障碍必须进入完整停车尾段，不能复用旧源答案");
}

#[test]
fn runtime_ground_owner_readonly_entry_uses_current_source_before_mutable_refresh() {
    let clear = Arc::new(map(0.0));
    let mut follower = owner_follower(clear.clone());
    let actual = owner_pose(1.05, 0);
    let target = owner_pose(1.45, 0);
    let retained = follower.current_ground_query().unwrap().0.clone();
    let blocked = Arc::new(clear.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
        [(0, [1.3, 1.05, 0.4])]).unwrap());
    follower.update_execution_map(ExecutionMap::Surface(blocked.clone())).unwrap();
    assert!(follower.body_view_dirty, "固定 setter 后尚未进入可变刷新入口的状态");
    assert!(!retained.same_body_query_view(&blocked));
    follower.with_body_motion_checks(|clear, period, _, _| {
        assert!(clear(planar(&actual), planar(&actual), 0.0));
        assert!(!clear(planar(&actual), planar(&target), 0.2),
            "直接细分检查必须看到新墙，不能使用旧 source 的绑定");
        assert!(period(planar(&actual), planar(&actual), 0.0));
        assert!(!period(planar(&actual), planar(&target), 0.2),
            "直接整拍检查也必须从同一当前 source 开始");
    }).unwrap();
    assert!(!follower.predicted_body_motion_is_traversable(
        Twist2 { linear: Point2::new(0.5, 0.0), angular_z: 0.0 }, 0.5));
    assert!(follower.body_view_dirty, "只读回退不伪装已完成可变状态刷新");
}

#[test]
fn runtime_ground_owner_readonly_entry_uses_current_actual_pose_without_consuming_prediction() {
    let source = Arc::new(map(0.0));
    let mut follower = owner_follower(source);
    let initial = follower.current_ground_query().unwrap().1.clone();
    let actual = owner_pose(1.45, 1);
    follower.update_odometry(actual.clone());
    let (_, current) = follower.current_ground_query_owned().unwrap();
    assert_ne!(current, initial, "只读入口须匹配新实际列，不能复用旧位姿的关联");
    follower.with_body_motion_checks(|clear, _, _, _| {
        assert!(clear(planar(&actual), planar(&owner_pose(1.65, 1)), 0.2));
    }).unwrap();
    assert_eq!(follower.current_ground_query_owned().unwrap().1, current,
        "直接预测之后仍从实际位姿取得关联，预测末点没有回写");
}

#[test]
fn runtime_ground_owner_callback_orders_share_only_exact_motion_proofs() {
    use crate::control_profile::Counter;
    for period_first in [false, true] {
        for blocked in [false, true] {
            let source = map(0.0);
            let source = if blocked {
                source.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
                    [(0, [1.4, 1.05, 0.4])]).unwrap()
            } else { source };
            let follower = owner_follower(Arc::new(source));
            let a = planar(&owner_pose(1.05, 0));
            let b = planar(&owner_pose(1.15, 0));
            let c = planar(&owner_pose(1.45, 0));
            crate::control_profile::begin();
            follower.with_body_motion_checks(|clear, period, failure, _| {
                let witness = if period_first {
                    assert!(period(a, b, 0.1));
                    assert!(clear(a, b, 0.1));
                    assert_eq!(period(b, c, 0.2), !blocked);
                    let witness = failure.borrow().clone();
                    assert_eq!(clear(b, c, 0.2), !blocked);
                    witness
                } else {
                    assert!(clear(a, b, 0.1));
                    assert!(period(a, b, 0.1));
                    assert_eq!(clear(b, c, 0.2), !blocked);
                    let witness = failure.borrow().clone();
                    assert_eq!(period(b, c, 0.2), !blocked);
                    witness
                };
                assert_eq!(*failure.borrow(), witness,
                    "共享失败缓存必须保留同一个真实拒绝样本，不受回调先后顺序影响");
            }).unwrap();
            let profile = crate::control_profile::end();
            assert_eq!(profile.counters[Counter::MotionMemoMisses as usize], 2,
                "两条独立轨迹只执行两个唯一的实际运动查询");
            assert_eq!(profile.counters[Counter::MotionMemoHits as usize], 2,
                "另一种采样的精确相同输入必须复用，包含失败结果");
        }
    }
}

#[test]
fn runtime_ground_owner_actual_unknown_gap_keeps_height_bias_in_mutable_and_readonly_entries() {
    use nuwax_nav_mls_planner::surface::GroundQueryState;
    for readonly in [false, true] {
        let source = Arc::new(map(0.0));
        let mut follower = owner_follower(source);
        let mut a = owner_pose(6.45, 1);
        a.pose.position.z = 0.45;
        let mut b = a.clone();
        b.pose.position.x = 6.65;
        b.stamp = owner_pose(6.65, 2).stamp;
        follower.update_odometry(a.clone());
        follower.refresh_current_body_view();
        let initial = follower.current_ground_query().unwrap().1.clone();
        assert_ne!(initial.query_state(), GroundQueryState::Unknown);
        let support_z = a.pose.position.z - follower.config.body_to_support_z_m;
        assert!((initial.height() - support_z).abs() > 0.025,
            "固定实际机体与已知地面之间的非零高度偏差");
        follower.update_odometry(b.clone());
        let (query, carried) = if readonly {
            follower.current_ground_query_owned().unwrap()
        } else {
            follower.refresh_current_body_view();
            follower.current_ground_query().map(|(q, s)| (q.clone(), s.clone())).unwrap()
        };
        eprintln!("ACTUAL_GROUND_UNKNOWN readonly={readonly} source_h={} actual_support_z={support_z} carried_h={}",
            initial.height(), carried.height());
        assert_eq!(carried.query_state(), GroundQueryState::Unknown);
        assert_eq!(carried.height().to_bits(), initial.height().to_bits(),
            "实际进入未知列只能携带原h，不能从bodyZ重新种下新地面高度");
        let projected = query.project_motion(body(&b), &carried,
            [a.pose.position.x, a.pose.position.y], 0.0, &mut None).unwrap();
        assert!((projected.pose().0[2] - b.pose.position.z).abs() < 1e-12,
            "返回同一平地不能因未知列重种子而凭空降低机体Z");
        query.validate_projected_motion(body(&b), &projected, &[], &mut None).unwrap();
        if readonly {
            assert_eq!(follower.actual_ground.as_ref().unwrap().0, a.pose,
                "只读临时关联和预测均不得写回实际缓存");
        }
    }
}

#[test]
fn runtime_ground_owner_new_source_reanchors_unknown_actual_without_reusing_old_key() {
    let mut follower = owner_follower(Arc::new(map(0.0)));
    let mut a = owner_pose(6.45, 1);
    a.pose.position.z = 0.45;
    follower.update_odometry(a.clone());
    follower.refresh_current_body_view();
    let old = follower.current_ground_query().unwrap().0.clone();
    let old_height = follower.current_ground_query().unwrap().1.height();
    let mut actual = a;
    actual.pose.position.x = 6.65;
    follower.update_odometry(actual.clone());
    let source = Arc::new(map(0.0));
    assert!(!old.same_body_query_view(&source), "相同内容但不同拥有方不共享源身份");
    let expected = source.body_query().start_ground_state(body(&actual)).unwrap();
    assert_ne!(expected.height().to_bits(), old_height.to_bits(), "夹具可区分新源重建与旧h携带");
    follower.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
    let (query, temporary) = follower.current_ground_query_owned().unwrap();
    assert!(query.same_body_query_view(&source));
    assert_eq!(temporary, expected);
    follower.refresh_current_body_view();
    assert_eq!(follower.current_ground_query().unwrap().1, &expected,
        "新源继续使用既有重建规则，不能硬套旧sourcekey");
}


#[test]
fn runtime_ground_owner_new_recent_preserves_unknown_height_and_rechecks_real_collision() {
    use nuwax_nav_mls_planner::surface::GroundQueryState;
    for readonly in [false, true] {
        for actual_moves in [false, true] {
            let source = Arc::new(map(0.0));
            let mut follower = owner_follower(source.clone());
            let mut known = owner_pose(6.45, 1);
            known.pose.position.z = 0.45;
            follower.update_odometry(known.clone());
            follower.refresh_current_body_view();
            let height = follower.current_ground_query().unwrap().1.height();
            let mut unknown = known;
            unknown.pose.position.x = 6.65;
            follower.update_odometry(unknown.clone());
            follower.refresh_current_body_view();
            let (old_query, old_ground) = follower.current_ground_query()
                .map(|(q, s)| (q.clone(), s.clone())).unwrap();
            assert_eq!(old_ground.query_state(), GroundQueryState::Unknown);
            assert_eq!(old_ground.height().to_bits(), height.to_bits());
            let mut actual = unknown.clone();
            if actual_moves { actual.pose.position.x = 6.75; }
            old_query.validate_motion_from_ground(body(&actual), body(&actual),
                &old_ground, &[], &mut None).expect("旧碰撞视图对此站姿可行");
            let blocked = Arc::new(source.with_recent_obstacle_points(2, 2_000_000_000, 0, &[],
                [(0, xyz(actual.pose.position))]).unwrap());
            assert!(old_query.same_ground_source(&blocked), "近期障碍不更换地面来源");
            assert!(!old_query.same_body_query_view(&blocked), "近期障碍必须淘汰旧碰撞答案");
            follower.update_execution_map(ExecutionMap::Surface(blocked.clone())).unwrap();
            follower.update_odometry(actual.clone());
            let (query, ground) = if readonly {
                follower.current_ground_query_owned().unwrap()
            } else {
                follower.refresh_current_body_view();
                follower.current_ground_query().map(|(q, s)| (q.clone(), s.clone())).unwrap()
            };
            assert!(query.same_body_query_view(&blocked));
            assert_eq!(ground.query_state(), GroundQueryState::Unknown);
            assert_eq!(ground.height().to_bits(), height.to_bits(),
                "同地面新近期视图仍携带既有h，不能按实际支撑Z重种子");
            assert_eq!(follower.odometry.as_ref().unwrap().pose, actual.pose,
                "实际关联不能修改已经观测到的机体姿态");
            assert!(query.validate_motion_from_ground(body(&actual), body(&actual),
                &ground, &[], &mut None).is_err(), "保留地面不等于保留旧碰撞许可");
            assert!(!follower.predicted_body_motion_is_traversable(Twist2::ZERO, 0.05),
                "实际控制入口必须拒绝新插入的真实障碍");
            if readonly {
                assert_eq!(follower.actual_ground.as_ref().unwrap().0, unknown.pose,
                    "只读关联仍不回写实际缓存，即使实际运动已穿过障碍");
                assert!(follower.body_view_dirty);
            }
            assert!(follower.current_body_error().is_some(), "当前站姿使用新碰撞视图拒绝");
            assert_eq!(follower.current_ground_query().unwrap().1.height().to_bits(), height.to_bits());
        }
    }
}
