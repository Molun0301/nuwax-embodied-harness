// 使用真实表面选择器和原生完整路线；旧事务撤销后不提交历史或目标编号。
use super::*;

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
