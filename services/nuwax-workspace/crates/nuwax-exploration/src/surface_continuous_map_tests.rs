// 使用真实表面选择器和原生完整路线；旧事务撤销后不提交历史或目标编号。
use super::*;

#[test]
fn observed_cells_behind_calibrated_goal_do_not_invalidate_unchanged_visible_gain() {
    let template = map();
    let points: Vec<_> = (0..60).flat_map(|x| (0..60).filter_map(move |y|
        ((x, y) != (37, 37)).then_some(((x as f32 + 0.5) * 0.1,
            (y as f32 + 0.5) * 0.1, 0.05)))).collect();
    let original = Arc::new(SurfaceSnapshot::from_points(&points, template.config().clone(), 0.1).unwrap());
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(original.clone()).unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let mut model = volume_model();
    model.max_range_m = 6.0;
    model.horizontal_fov_rad = std::f64::consts::FRAC_PI_2;
    adapter.set_observation_model(model.clone()).unwrap();
    let mut current = state(0.4, 1);
    current.robot_pose.pose.position.x = 0.55;
    current.robot_pose.pose.position.y = 0.55;
    let first = receiver.freeze().unwrap().fork_for_planning().with_revalidation_advice();
    goal(adapter.prepare_poll_with_planning_input(&current, &first)
        .unwrap().outcome().as_ref().unwrap().clone());
    adapter.discard_prepared_poll().unwrap();
    let input = receiver.freeze().unwrap().fork_for_planning().with_revalidation_advice();
    let selected = goal(adapter.prepare_poll_with_planning_input(&current, &input)
        .unwrap().outcome().as_ref().unwrap().clone());
    let mut filled = points;
    filled.push((3.75, 3.75, 0.05));
    let changed = Arc::new(SurfaceSnapshot::from_points(&filled, template.config().clone(), 0.1).unwrap());
    let before = original.observation_targets(&[]).unwrap();
    let after = changed.observation_targets(&[]).unwrap();
    assert_ne!(before.iter().map(|target| target.cell).collect::<Vec<_>>(),
        after.iter().map(|target| target.cell).collect::<Vec<_>>(), "夹具须真正观测掉远处未知单元");
    let p = selected.pose.pose.position;
    let q = selected.pose.pose.orientation;
    let visible = |map: &SurfaceSnapshot, targets: &[_]| map.estimate_observation_view(
        &model, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], targets).unwrap().cells;
    let before_visible = visible(&original, &before);
    assert!(!before_visible.is_empty());
    assert_eq!(before_visible, visible(&changed, &after), "标定目标的实际可见收益须保持相同");
    receiver.observe(changed).unwrap();
    assert_eq!(input.selection_qualification().unwrap().invalidation(&receiver.freeze().unwrap()), None,
        "已观测单元在相机当前视域外，不应撤销实际可见收益未变的候选");
    adapter.discard_prepared_poll().unwrap();
}

#[test]
fn unrelated_change_inside_route_bounding_box_preserves_revalidated_selection() {
    let template = map();
    let points: Vec<_> = (0..100).flat_map(|x| (0..100).map(move |y|
        ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05))).collect();
    let original = Arc::new(SurfaceSnapshot::from_points(&points, template.config().clone(), 0.1).unwrap());
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(original.clone()).unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let first = receiver.freeze().unwrap().fork_for_planning().with_revalidation_advice();
    let mut corner = state(0.4, 1);
    corner.robot_pose.pose.position.x = 9.05;
    corner.robot_pose.pose.position.y = 9.05;
    let old = goal(adapter.prepare_poll_with_planning_input(&corner, &first)
        .unwrap().outcome().as_ref().unwrap().clone());
    adapter.discard_prepared_poll().unwrap();
    let input = receiver.freeze().unwrap().fork_for_planning().with_revalidation_advice();
    let mut current = state(0.4, 2);
    current.robot_pose.pose.position.x = if old.pose.pose.position.x < 5.0 { 9.05 } else { 0.55 };
    current.robot_pose.pose.position.y = if old.pose.pose.position.y < 5.0 { 9.05 } else { 0.55 };
    let selected = goal(adapter.prepare_poll_with_planning_input(&current, &input)
        .unwrap().outcome().as_ref().unwrap().clone());
    assert_eq!(selected.pose.pose.position, old.pose.pose.position);
    assert_eq!(selected.metadata["route_exact_queries"], "1");
    let qualification = input.selection_qualification().unwrap();
    let start = current.robot_pose.pose.position;
    let end = selected.pose.pose.position;
    assert!((start.x - end.x).abs() > 4.0 && (start.y - end.y).abs() > 4.0,
        "夹具须形成两个方向均跨越半张已测地图的路线");
    let obstacle = [start.x, end.y, 0.4];
    let changed = Arc::new(original.with_recent_obstacle_points(2, 2, 0, &[],
        [(0, obstacle)]).unwrap());
    let native = NativeFrontierCache::default().get(&changed, 0.3, &[]).unwrap();
    let route = native.planner.lock().unwrap().plan([start.x, start.y, start.z],
        [end.x, end.y, end.z]).expect("更新后的原完整路线查询须仍然通过");
    let minimum = route.path.windows(2).map(|pair| {
        let delta = [pair[1][0] - pair[0][0], pair[1][1] - pair[0][1]];
        let length2 = delta[0] * delta[0] + delta[1] * delta[1];
        let t = if length2 == 0.0 { 0.0 } else {
            (((obstacle[0] - pair[0][0]) * delta[0] + (obstacle[1] - pair[0][1]) * delta[1]) / length2).clamp(0.0, 1.0)
        };
        (obstacle[0] - pair[0][0] - t * delta[0]).hypot(obstacle[1] - pair[0][1] - t * delta[1])
    }).fold(f64::INFINITY, f64::min);
    assert!(minimum > 2.0, "更新实体须离完整路线超过2米，实际{minimum}米");
    receiver.observe(changed).unwrap();
    assert_eq!(qualification.invalidation(&receiver.freeze().unwrap()), None,
        "地图角上的实体在路线外接矩形内，但远离实际对角路线及目标，不应撤销");
    adapter.discard_prepared_poll().unwrap();
}

#[test]
fn revalidated_selection_compares_current_route_and_target_instead_of_whole_map_identity() {
    let original = map();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(original.clone()).unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let first = receiver
        .freeze()
        .unwrap()
        .fork_for_planning()
        .with_revalidation_advice();
    adapter
        .prepare_poll_with_planning_input(&state(0.4, 1), &first)
        .unwrap();
    assert!(first.selection_qualification().is_none());
    adapter.discard_prepared_poll().unwrap();
    let input = receiver
        .freeze()
        .unwrap()
        .fork_for_planning()
        .with_revalidation_advice();
    let selected = goal(
        adapter
            .prepare_poll_with_planning_input(&state(0.4, 2), &input)
            .unwrap()
            .outcome()
            .as_ref()
            .unwrap()
            .clone(),
    );
    let qualification = input
        .selection_qualification()
        .expect("完整精确路线复核的实际产物");
    let mut points: Vec<_> = original
        .navigation_measured_points()
        .unwrap()
        .iter()
        .map(|p| (p[0] as f32, p[1] as f32, p[2] as f32))
        .collect();
    points.push((50.05, 50.05, points[0].2));
    let unrelated = Arc::new(
        SurfaceSnapshot::from_points(
            &points,
            original.config().clone(),
            original.source_voxel_size_m(),
        )
        .unwrap()
        .with_ground_evidence(original.ground_evidence()),
    );
    receiver.observe(unrelated).unwrap();
    let latest = receiver.freeze().unwrap();
    assert_eq!(
        input.original_input().planning_query_change(&latest),
        Some("surface_geometry")
    );
    assert_eq!(
        qualification.invalidation(&latest),
        None,
        "实际重建整图并新增远处地面，不能撤销路线和目标都未变化的已重核结果"
    );
    let p = selected.pose.pose.position;
    let blocked = Arc::new(
        original
            .with_recent_obstacle_points(3, 3, 0, &[], [(0, [p.x, p.y, p.z + 0.1])])
            .unwrap(),
    );
    receiver.observe(blocked).unwrap();
    assert_eq!(
        qualification.invalidation(&receiver.freeze().unwrap()),
        Some("selected_route_geometry")
    );
    adapter.discard_prepared_poll().unwrap();
}

#[test]
fn fresh_receipt_rechecks_discarded_goal_even_when_query_geometry_is_unchanged() {
    let receiver = SurfaceInputReceiver::new();
    let snapshot = map();
    receiver.observe(snapshot.clone()).unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let captured = receiver.freeze().unwrap();
    let planning = captured.fork_for_planning().with_revalidation_advice();
    let original = goal(
        adapter
            .prepare_poll_with_planning_input(&state(0.4, 1), &planning)
            .unwrap()
            .outcome()
            .as_ref()
            .unwrap()
            .clone(),
    );
    adapter.discard_prepared_poll().unwrap();
    receiver.observe(snapshot).unwrap();
    let fresh = receiver.freeze().unwrap();
    assert!(captured.same_planning_query_view(&fresh));
    assert!(fresh.notification() > captured.notification());
    let planning = fresh.fork_for_planning().with_revalidation_advice();
    let current = goal(
        adapter
            .prepare_poll_with_planning_input(&state(0.4, 2), &planning)
            .unwrap()
            .outcome()
            .as_ref()
            .unwrap()
            .clone(),
    );
    assert_eq!(current.pose.pose, original.pose.pose);
    assert_eq!(current.metadata["route_exact_queries"], "1");
    assert_eq!(current.metadata["route_source_searches"], "0");
    assert_eq!(adapter.next_goal_id, 0);
    adapter.discard_prepared_poll().unwrap();
}

#[test]
fn discarded_goal_is_revalidated_on_current_map_without_repeating_source_search() {
    let original = map();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(original.clone()).unwrap();
    let captured = receiver.freeze().unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let before = adapter.checkpoint().unwrap();
    let planning = captured.fork_for_planning().with_revalidation_advice();
    let old = goal(
        adapter
            .prepare_poll_with_planning_input(&state(0.4, 1), &planning)
            .unwrap()
            .outcome()
            .as_ref()
            .unwrap()
            .clone(),
    );
    let changed = Arc::new(
        original
            .with_recent_obstacle_points(2, 2, 0, &[], [(0, [10.05, 10.05, 0.3])])
            .unwrap(),
    );
    receiver.observe(changed).unwrap();
    assert!(
        !captured.same_planning_query_view(&receiver.freeze().unwrap()),
        "夹具必须真正改变原完整查询，不能只增加发布修订"
    );
    adapter.discard_prepared_poll().unwrap();
    assert_eq!(adapter.checkpoint().unwrap(), before);
    assert_eq!(adapter.next_goal_id, 0);
    let current = state(0.4, 2);
    let current_input = receiver
        .freeze()
        .unwrap()
        .fork_for_planning()
        .with_revalidation_advice();
    let fresh = goal(
        adapter
            .prepare_poll_with_planning_input(&current, &current_input)
            .unwrap()
            .outcome()
            .as_ref()
            .unwrap()
            .clone(),
    );
    let mut original_strategy = source(receiver.clone(), 1);
    let reference = goal(original_strategy.poll_goal(&current).unwrap());
    assert_eq!(fresh.pose, reference.pose);
    assert_eq!(fresh.information_gain, reference.information_gain);
    assert_eq!(fresh.distance_cost, reference.distance_cost);
    assert_ne!(fresh.id, old.id);
    assert_eq!(fresh.pose.stamp, current.now);
    assert!(fresh.information_gain.unwrap() > 0.0);
    let source_searches = fresh.metadata["route_source_searches"]
        .parse::<usize>()
        .unwrap();
    let exact_queries = fresh.metadata["route_exact_queries"]
        .parse::<usize>()
        .unwrap();
    let accepted = goal(adapter.commit_prepared_poll().unwrap());
    assert_eq!(accepted, fresh);
    assert_eq!(adapter.next_goal_id, 1);
    assert_eq!(exact_queries, 1, "新输入仍须完成原完整路线核验");
    assert_eq!(
        source_searches, 0,
        "已选候选应在新输入上直接复核，不重复整图单源搜索与所有候选排序"
    );
}

#[test]
fn changed_target_is_checked_again_and_stop_drops_search_advice() {
    let original = map();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(original.clone()).unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let input = receiver
        .freeze()
        .unwrap()
        .fork_for_planning()
        .with_revalidation_advice();
    let old = goal(
        adapter
            .prepare_poll_with_planning_input(&state(0.4, 1), &input)
            .unwrap()
            .outcome()
            .as_ref()
            .unwrap()
            .clone(),
    );
    adapter.discard_prepared_poll().unwrap();
    assert!(adapter.search_hint.as_ref().unwrap().candidate.is_some());
    let target = old.pose.pose.position;
    let wall: Vec<_> = (-3..=3)
        .flat_map(|x| {
            (-3..=3).map(move |y| {
                [
                    target.x + f64::from(x) * 0.1,
                    target.y + f64::from(y) * 0.1,
                    target.z,
                ]
            })
        })
        .enumerate()
        .map(|(index, point)| (u32::try_from(index).unwrap(), point))
        .collect();
    let blocked = Arc::new(
        original
            .with_recent_obstacle_points(2, 2, 0, &[], wall)
            .unwrap(),
    );
    let native = NativeFrontierCache::default()
        .get(&blocked, 0.3, &[])
        .unwrap();
    let p = state(0.4, 2).robot_pose.pose.position;
    let direct = native
        .planner
        .lock()
        .unwrap()
        .plan([p.x, p.y, p.z], [target.x, target.y, target.z]);
    assert!(
        direct.as_ref().map_or(true, |route| route
            .path
            .last()
            .is_none_or(|end| distance(*end, [target.x, target.y, target.z]) > 0.2)),
        "夹具须由原生查询证明实体确实使原目标不可用，不能把可跨低阶当作阻断"
    );
    receiver.observe(blocked).unwrap();
    let current = receiver
        .freeze()
        .unwrap()
        .fork_for_planning()
        .with_revalidation_advice();
    let outcome = adapter
        .prepare_poll_with_planning_input(&state(0.4, 2), &current)
        .unwrap();
    if let Ok(ExplorationPoll::Goal(fresh)) = outcome.outcome() {
        assert_ne!(
            fresh.metadata["selection_method"],
            "current_map_revalidation"
        );
        assert!(
            fresh.pose.pose.position.distance(target) > 0.2,
            "新实体覆盖原候选时不得交付旧位置"
        );
        assert!(
            fresh.metadata["route_query_count"]
                .parse::<usize>()
                .unwrap()
                <= MAX_FRONTIER_ROUTE_QUERIES
        );
        assert!(
            fresh.metadata["evaluated_candidate_count"]
                .parse::<usize>()
                .unwrap()
                <= MAX_FRONTIER_CANDIDATE_EVALUATIONS
        );
    }
    adapter.discard_prepared_poll().unwrap();
    assert_eq!(adapter.next_goal_id, 0);
    adapter.end().unwrap();
    assert!(adapter.search_hint.is_none());
    adapter.begin().unwrap();
    assert!(adapter.search_hint.is_none());
}

#[test]
fn changed_queries_rotate_unfinished_work_without_committing_old_absence() {
    let points: Vec<_> = (0..120)
        .flat_map(|x| (0..100).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.0)))
        .collect();
    let floor = SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap();
    let mut mask = vec![false; 160 * 140];
    for y in 0..140 {
        for x in 0..160 {
            if ((x == 29 || x == 41) && (29..=41).contains(&y))
                || ((y == 29 || y == 41) && (29..=41).contains(&x))
            {
                mask[y * 160 + x] = true;
            }
        }
    }
    let original = Arc::new(
        floor
            .with_virtual_obstacles([-2., -2.], 0.1, 160, 140, &mask)
            .unwrap(),
    );
    assert!(original.frontier_candidates(&[]).unwrap().len() > 128);
    let mut offsets = Vec::new();
    for allow in [false, true] {
        let receiver = SurfaceInputReceiver::new();
        let mut adapter = source(receiver.clone(), 1);
        let checkpoint = adapter.checkpoint().unwrap();
        let mut positions = Vec::new();
        let mut first_unfinished = None;
        for sequence in 1..=4 {
            receiver
                .observe(Arc::new(
                    original
                        .with_recent_obstacle_points(
                            sequence,
                            sequence,
                            0,
                            &[],
                            [(0, [50. + sequence as f64, 50., 0.3])],
                        )
                        .unwrap(),
                ))
                .unwrap();
            let mut planning = receiver.freeze().unwrap().fork_for_planning();
            if allow {
                planning = planning.with_revalidation_advice();
            }
            let mut pose = state(0.3, sequence as i64);
            pose.robot_pose.pose.position.x = 1.55;
            pose.robot_pose.pose.position.y = 1.55;
            let prepared = adapter
                .prepare_poll_with_planning_input(&pose, &planning)
                .unwrap();
            assert_eq!(
                prepared.outcome().as_ref().unwrap(),
                &ExplorationPoll::Pending,
                "未完成一幅当前图的全部查询前不能合并旧批次宣布无前沿"
            );
            let route = adapter
                .prepared_poll
                .as_ref()
                .unwrap()
                .delta
                .deferred_route
                .as_ref()
                .unwrap()
                .as_ref()
                .unwrap();
            positions.push(route.rotation_offset);
            if sequence == 1 {
                first_unfinished = Some(route.resume_cursor);
                assert!(route.resume_cursor > 0);
            }
            adapter.discard_prepared_poll().unwrap();
            assert_eq!(adapter.checkpoint().unwrap(), checkpoint);
            assert_eq!(adapter.route_cursor, 0, "撤销事务不能提交旧候选游标");
        }
        if allow {
            assert_eq!(
                positions[1],
                first_unfinished.unwrap(),
                "下轮须从尚未查询项继续，不能跳过预算边界候选"
            );
        }
        offsets.push(positions);
    }
    assert!(offsets[0].iter().all(|offset| *offset == 0));
    assert!(
        offsets[1].iter().skip(1).all(|offset| *offset > 0),
        "新查询须轮转工作顺序，不能永久重做第一批"
    );
}

#[test]
fn failed_navigation_keeps_unfinished_search_order_across_changed_maps() {
    let points: Vec<_> = (0..120)
        .flat_map(|x| (0..100).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.0)))
        .collect();
    let floor = SurfaceSnapshot::from_points(&points, map().config().clone(), 0.1).unwrap();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(Arc::new(floor.clone())).unwrap();
    let mut adapter = source(receiver.clone(), 1);
    let mut pose = state(0.3, 1);
    pose.robot_pose.pose.position.x = 1.55;
    pose.robot_pose.pose.position.y = 1.55;
    let selected = goal(adapter.poll_goal(&pose).unwrap());
    adapter.report_goal_feedback(&selected.id,
        ExplorationGoalFeedback::Failed("已核验停车的导航失败".into())).unwrap();
    let retry = adapter.retry_support.expect("真实失败反馈应保留一次重试资格");
    let checkpoint = adapter.checkpoint().unwrap();
    // 新图把原实际站位围住，剩余前沿只能分批判定；远处更新不能反复重做第一批。
    let mut mask = vec![false; 160 * 140];
    for y in 0..140 {
        for x in 0..160 {
            if ((x == 29 || x == 41) && (29..=41).contains(&y))
                || ((y == 29 || y == 41) && (29..=41).contains(&x)) {
                mask[y * 160 + x] = true;
            }
        }
    }
    let blocked = floor.with_virtual_obstacles([-2., -2.], 0.1, 160, 140, &mask).unwrap();
    assert!(blocked.frontier_candidates(&[]).unwrap().len() > MAX_FRONTIER_CANDIDATE_EVALUATIONS);
    let mut next_offset = 0;
    for sequence in 2..=4 {
        receiver.observe(Arc::new(blocked.with_recent_obstacle_points(sequence, sequence, 0,
            &[], [(0, [50. + sequence as f64, 50., 0.3])]).unwrap())).unwrap();
        let planning = receiver.freeze().unwrap().fork_for_planning().with_revalidation_advice();
        let prepared = adapter.prepare_poll_with_planning_input(&pose, &planning).unwrap();
        assert_eq!(prepared.outcome().as_ref().unwrap(), &ExplorationPoll::Pending);
        let continuation = adapter.prepared_poll.as_ref().unwrap().delta.deferred_route
            .as_ref().unwrap().as_ref().unwrap();
        assert_eq!(continuation.rotation_offset, next_offset,
            "导航失败重试也应从上一批尚未查询项开始，不能因重试资格重置搜索顺序");
        next_offset = continuation.rotation_offset + continuation.resume_cursor;
        assert!(continuation.resume_cursor > 0);
        adapter.discard_prepared_poll().unwrap();
        assert_eq!(adapter.checkpoint().unwrap(), checkpoint, "撤销不得增加失败或提交旧无前沿结论");
        assert_eq!(adapter.retry_support, Some(retry));
        assert!(adapter.selected.is_none());
    }
    // 新图停止变化后，仍须把这一幅当前图完整检查到无前沿；旧批次不能拼成结论。
    let planning = receiver.freeze().unwrap().fork_for_planning().with_revalidation_advice();
    let maximum_batches = blocked.frontier_candidates(&[]).unwrap().len()
        .div_ceil(MAX_FRONTIER_ROUTE_QUERIES - 1) + 2;
    let mut exhausted = false;
    for _ in 0..maximum_batches {
        adapter.prepare_poll_with_planning_input(&pose, &planning).unwrap();
        match adapter.commit_prepared_poll().unwrap() {
            ExplorationPoll::NoFrontier => { exhausted = true; break; }
            ExplorationPoll::Pending => {}
            other => panic!("封闭站位不应产生可执行路线：{other:?}"),
        }
    }
    assert!(exhausted, "当前输入完整处理后必须自然耗尽，不能继续空转");
    assert_eq!(adapter.history[0].failures, 1, "规划不可达不能冒充又一次子导航失败");
}
