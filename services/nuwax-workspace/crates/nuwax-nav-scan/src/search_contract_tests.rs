// 固定默认搜索合同；重复请求不改变地图、能力、预算或排序。

#[test]
fn rejected_bound_sweep_does_not_publish_candidate_ground_state() {
    let map = coarse_open_floor();
    let query = map.body_query();
    let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
    let mut ground = query.start_ground_state(start).unwrap();
    let before = ground.clone();
    let excluded = [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]];
    assert!(advance_bound(&query, &mut ground, start, [-0.95, 0.2], 0.0, &excluded).is_err());
    assert_eq!(ground, before, "扫掠拒绝后起点关联保持，候选地面不能泄漏给下一分支");
    let end = advance_bound(&query, &mut ground, start, [-0.95, 0.2], 0.0, &[]).unwrap();
    assert_eq!(end.0[2].to_bits(), start.0[2].to_bits());
}

#[test]
fn bound_unknown_rotation_retains_carried_height_without_rebinding() {
    let map = coarse_open_floor();
    let query = map.body_query();
    let start = ([100.0, 100.0, 0.55], [0.0, 0.0, 0.0, 1.0]);
    let mut ground = query.start_ground_state(start).unwrap();
    assert_eq!(ground.query_state(), nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
    let before = ground.clone();
    let mut poses = vec![start];
    rotate_bound(&query, &mut ground, &mut poses, 0.5, &motion(), &[]).unwrap();
    assert!(poses.len() > 2);
    assert!(poses.iter().all(|pose| pose.0.map(f64::to_bits) == start.0.map(f64::to_bits)));
    assert_eq!(ground, before, "未知列不补零、不创造目标坡面或按旋转采样重新绑定");
    let mut checked_ground = before;
    for pair in poses.windows(2) {
        checked_ground = query.validate_motion_from_ground(pair[0], pair[1], &checked_ground,
            &[], &mut None).unwrap();
    }
    assert_eq!(checked_ground, ground);
}

#[test]
fn default_search_outcomes_are_stable_across_repeated_requests() {
    let base = coarse_open_floor();
    let tomo = Tomogram::from_surface(&base).unwrap();
    let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
    let reference = [[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]];
    let excluded = [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]];
    let config = ScanConfig::default();
    assert_eq!(config.search_expansion_limit, 256);
    for (name, arrival, boundary, expected) in [
        ("open", None, &[][..], None),
        (
            "budget",
            Some(([0.1, 0.2, 2.0], 0.1)),
            &[][..],
            Some(LocalPlanFailureKind::BudgetExhausted),
        ),
        (
            "drained",
            None,
            &excluded[..],
            Some(LocalPlanFailureKind::SearchExhausted),
        ),
    ] {
        let map = base.fork_for_planning();
        let mut previous = None;
        for repeat in 0..2 {
            local_search::reset_last_work();
            let result = plan(
                &map,
                &tomo,
                start,
                &reference,
                &motion(),
                &config,
                None,
                arrival,
                boundary,
                true,
                false,
            );
            let work = local_search::last_work();
            assert_eq!(result.as_ref().err().map(|error| error.kind), expected, "{name}");
            if let Err(error) = &result {
                assert!(error.blocked.is_none(), "{name} 的搜索结果不是物理障碍测量");
            } else {
                let path = result.as_ref().unwrap();
                assert!(!path.partial, "未请求有限前缀");
                for edge in path.poses.windows(2) {
                    map.validate_body_motion(edge[0], edge[1], boundary).unwrap();
                }
            }
            if expected == Some(LocalPlanFailureKind::BudgetExhausted) {
                assert_eq!(work.0, config.search_expansion_limit);
            }
            if let Some((previous_result, previous_work)) = &previous {
                crate::polyline_reuse_tests::assert_same_result(previous_result, &result);
                assert_eq!(*previous_work, work, "{name} 第 {repeat} 次请求的搜索工作不同");
            }
            previous = Some((result, work));
        }
    }
}
