// Test-only full-selector frozen oracle. LAZY=false retains the original eager order.
thread_local! { static INFLATION_CALLS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) }; }
fn probe_inflate(map: &OccupancyGrid2D, radius: f64) -> OccupancyGrid2D {
    INFLATION_CALLS.with(|n| n.set(n.get()+1));
    inflate_obstacles(map, radius)
}
impl WavefrontGoalSelector {
    fn probe_prepare_inflation<const LAZY: bool>(
        &mut self,
        robot_pose: Point2,
        costmap: &OccupancyGrid2D,
        observations: Option<&OccupancyGrid2D>,
        session: &crate::session::ExplorationSession,
        now: Timestamp,
    ) -> PreparedWavefrontSelection {
        let cfg = self.detector.config().clone();
        let physical_costmap = costmap;
        let mut inflated_costmap = if LAZY && observations.is_some() { None } else {
            Some(probe_inflate(costmap, cfg.obstacle_inflation_radius_m))
        };

        // 低增益是局部目标的反馈，不能在检查其它可达前沿之前结束全图探索。
        let (no_gain_reason, information_gain) =
            session.prepare_information_gain(observations.unwrap_or_else(|| inflated_costmap.as_ref().expect("None observations remains eager")), &cfg);
        let mut regional_presence = None;
        let outcome = (|| {
            // 2. detect
            let outcome = self
                .detector
                .detect_candidates(robot_pose, physical_costmap, session)?;

            // 3. map NoReachableFreeSpace
            let candidates = match outcome {
                DetectionOutcome::NoReachableFreeSpace => {
                    return Ok(PreparedGoalSelection::Stop(
                        StopReason::NoReachableFreeSpace,
                    ));
                }
                DetectionOutcome::Candidates(c) => c,
            };

            // 4. 评分 + 排序
            let mut scored: Vec<(
                crate::FrontierScoreBreakdown,
                crate::detector::FrontierCandidate,
            )> = Vec::with_capacity(candidates.len());
            let mut views = std::collections::BTreeMap::new();
            for cand in &candidates {
                if session.is_blacklisted(cand.goal_position) {
                    continue;
                }
                if no_gain_reason.is_some()
                    && session.explored_goals.iter().any(|visited| {
                        visited.distance(cand.goal_position)
                            <= cfg.obstacle_inflation_radius_m + costmap.resolution
                    })
                {
                    continue;
                }
                let geom = CandidateGeometry {
                    centroid: cand.goal_position,
                    cell_count: cand.cluster.cell_count,
                    perimeter_m: cand.cluster.perimeter_m,
                    area_m2: cand.cluster.area_m2,
                };
                let costmap = inflated_costmap.get_or_insert_with(||
                    probe_inflate(physical_costmap, cfg.obstacle_inflation_radius_m));
                let breakdown = self.scorer.score(
                    &geom,
                    robot_pose,
                    costmap,
                    &session.explored_goals,
                    session.exploration_direction(),
                    &cfg,
                );
                let mut candidate = cand.clone();
                if let Some(model) = &self.observation_model {
                    let mut best = None;
                    let turns = if model.horizontal_fov_rad >= std::f64::consts::TAU - 1e-6 {
                        1
                    } else {
                        4
                    };
                    for i in 0..turns {
                        let yaw = cand.goal_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                        if !self.detector.goal_rotation_is_clear(
                            physical_costmap,
                            cand.goal_position,
                            yaw,
                        ) {
                            continue;
                        }
                        let view = nuwax_explore_routing::observation::estimate_planar_view(
                            model,
                            physical_costmap,
                            [cand.goal_position.x, cand.goal_position.y],
                            yaw,
                            self.detector.observation_targets(),
                        )
                        .map_err(WavefrontError::InvalidObservation)?;
                        if best.as_ref().is_none_or(
                            |(_, previous): &(
                                f64,
                                nuwax_explore_routing::observation::ViewEstimate,
                            )| view.cells.len() > previous.cells.len(),
                        ) {
                            best = Some((yaw, view));
                        }
                    }
                    let Some((yaw, view)) = best.filter(|(_, view)| !view.cells.is_empty()) else {
                        continue;
                    };
                    candidate.goal_yaw = yaw;
                    views.insert(candidate.debug_index, view);
                }
                scored.push((breakdown, candidate));
            }
            // 区域核心压缩代表并计算访问顺序，避免先对所有候选进行另一遍排序。
            let routing: Vec<_> = scored
                .iter()
                .map(|(score, candidate)| nuwax_explore_routing::Candidate {
                    position: [candidate.goal_position.x, candidate.goal_position.y, 0.0],
                    gain: views.get(&candidate.debug_index).map_or(
                        candidate.cluster.area_m2 * f64::from(score.total.max(0.001)),
                        |view| view.cells.len() as f64 * physical_costmap.resolution.powi(2),
                    ),
                    travel_cost: candidate.travel_cost,
                })
                .collect();
            let observed: Vec<_> = scored
                .iter()
                .map(|(_, candidate)| {
                    views
                        .get(&candidate.debug_index)
                        .map(|view| view.cells.clone())
                        .unwrap_or_default()
                })
                .collect();
            let (selection, presence) = session.regional_schedule.prepare_select_observed(
                &routing,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    let targets: Vec<_> =
                        goals.iter().map(|&i| scored[i].1.goal_position).collect();
                    self.detector
                        .travel_costs(scored[source].1.goal_position, &targets)
                },
            );
            regional_presence = Some(presence);
            let Some((breakdown, cand)) = selection
                .as_ref()
                .and_then(|selection| scored.get(selection.index))
            else {
                return Ok(match no_gain_reason {
                    Some(reason) => PreparedGoalSelection::Stop(reason),
                    None => PreparedGoalSelection::NoFrontier,
                });
            };

            // 6. 生成 id
            let seed = FrontierGoalIdSeed {
                centroid: cand.goal_position,
                score: breakdown.total,
                frontier_area_m2: Some(cand.cluster.area_m2 as f32),
                stamp: now,
                source_name: std::borrow::Cow::Borrowed("nuwax-explore-wavefront"),
            };

            // 7. 构造 goal(crate-private helper)
            let mut goal = frontier_candidate_to_draft(cand, seed);
            goal.distance_cost = Some(cand.travel_cost as f32);
            if let Some(view) = views.get(&cand.debug_index) {
                goal.information_gain =
                    Some((view.cells.len() as f64 * physical_costmap.resolution.powi(2)) as f32);
                goal.metadata.insert(
                    "gain_basis".into(),
                    "calibrated_visible_frontier_cells".into(),
                );
                goal.metadata
                    .insert("visibility_rays".into(), view.rays.to_string());
                goal.metadata.insert(
                    "visibility_budget_limited".into(),
                    view.budget_limited.to_string(),
                );
            }
            goal.metadata
                .insert("selection_method".into(), "regional_route".into());
            if let Some(selection) = selection {
                goal.metadata.insert(
                    "region_wait_rounds".into(),
                    selection.priority_wait_rounds.to_string(),
                );
                goal.metadata.insert(
                    "route_regions".into(),
                    selection.regional_order.len().to_string(),
                );
                goal.metadata.insert(
                    "available_regions".into(),
                    selection.region_count.to_string(),
                );
                goal.metadata
                    .insert("route_cost_m".into(), selection.route_cost.to_string());
            }
            Ok(PreparedGoalSelection::Goal(goal))
        })();
        PreparedWavefrontSelection {
            outcome,
            information_gain,
            regional_presence,
        }
    }

}

mod lazy_inflation_probe {
    use super::*;
    use crate::session::ExplorationSession;
    #[derive(Default)]
    struct Ids(Vec<FrontierGoalIdSeed>);
    impl FrontierGoalIdSource for Ids {
        fn next_frontier_goal_id(&mut self, seed:&FrontierGoalIdSeed)->String {
            self.0.push(seed.clone()); format!("probe-{}",self.0.len())
        }
    }
    fn fixture(case:usize,n:u32)->(OccupancyGrid2D,Point2,ExplorationSession) {
        let mut map=OccupancyGrid2D::filled(Point2::new(0.,0.),0.25,n,n,OccupancyCell::Free);
        let robot=Point2::new((n/2) as f64*0.25+0.125,(n/2) as f64*0.25+0.125);
        if [2,3,4,5,7].contains(&case) {
            for y in n/3..=2*n/3 {for x in [n/4,3*n/4] {map.set(x,y,OccupancyCell::Unknown);}}
            // Distant real obstacles exercise actual inflation on normal and filtered cohorts.
            for x in 3..n-3 {map.set(x,3,OccupancyCell::Occupied);}
        }
        if case==1 {map.cells.fill(OccupancyCell::Occupied);}
        if case==6 {map.cells.fill(OccupancyCell::Unknown);}
        let mut session=ExplorationSession::new();
        session.record_goal_published();
        if case==3 {session.record_goal_failed(robot,0,1000.);}
        if case==4 {
            session.last_information_count=Some(u64::MAX/2);
            session.no_gain_counter=1;
            for y in 0..n {for x in 0..n {session.mark_explored_goal(Point2::new(x as f64*0.25+0.125,y as f64*0.25+0.125));}}
        }
        (map,if case==7 {Point2::new(-1.,0.)} else {robot},session)
    }
    fn make_selector(calibrated:bool,invalid:bool)->WavefrontGoalSelector {
        let mut s=WavefrontGoalSelector::new(WavefrontConfig::default());
        if calibrated {
            s.observation_model=Some(nuwax_nav_types_messages::ObservationModel {
                kind:nuwax_nav_types_messages::ObservationKind::Planar,min_range_m:0.,max_range_m:200.,
                horizontal_fov_rad:std::f64::consts::TAU,vertical_min_rad:0.,vertical_max_rad:0.,
                sensor_translation_body_m:[0.;3],sensor_rotation_body_xyzw:[0.,0.,0.,1.],max_rays:if invalid {0} else {16},
            });
        } s
    }
    fn call<const LAZY:bool>(selector:&mut WavefrontGoalSelector,map:&OccupancyGrid2D,robot:Point2,
        separate:bool,session:&ExplorationSession)->(PreparedWavefrontSelection,usize) {
        INFLATION_CALLS.with(|n|n.set(0));
        let out=selector.probe_prepare_inflation::<LAZY>(robot,map,separate.then_some(map),session,Timestamp::from_unix(123,456));
        (out,INFLATION_CALLS.with(|n|n.get()))
    }
    #[test]
    fn lazy_inflation_full_selector_equivalence() {
        let mut cases=0;
        for separate in [false,true] {for calibrated in [false,true] {for case in 0..8 {
            let (map,robot,session)=fixture(case,101);
            let mut a=make_selector(calibrated,case==5);
            let mut b=make_selector(calibrated,case==5);
            let mut c=make_selector(calibrated,case==5);
            let mut sa=session.clone(); let mut sb=session.clone(); let mut sc=session;
            let mut ia=Ids::default();let mut ib=Ids::default();let mut ic=Ids::default();
            for round in 0..3 {
                let before=serde_json::to_value(&sa).unwrap();
                let (pa,ca)=call::<false>(&mut a,&map,robot,separate,&sa);
                let (pb,cb)=call::<true>(&mut b,&map,robot,separate,&sb);
                let pc=c.prepare_goal_with_observations(robot,&map,separate.then_some(&map),&sc,Timestamp::from_unix(123,456));
                assert_eq!(format!("{pa:?}"),format!("{pb:?}"),"case={case} separate={separate} round={round}");
                assert_eq!(format!("{pa:?}"),format!("{pc:?}"));
                assert_eq!(before,serde_json::to_value(&sa).unwrap());
                assert_eq!(a.diagnostics(),b.diagnostics());assert_eq!(a.diagnostics(),c.diagnostics());
                let oa=pa.resolve(&mut sa,&mut ia); let ob=pb.resolve(&mut sb,&mut ib);let oc=pc.resolve(&mut sc,&mut ic);
                assert_eq!(oa,ob);assert_eq!(oa,oc);
                assert_eq!(ia.0,ib.0);assert_eq!(ia.0,ic.0);
                assert_eq!(serde_json::to_value(&sa).unwrap(),serde_json::to_value(&sb).unwrap());
                assert_eq!(serde_json::to_value(&sa).unwrap(),serde_json::to_value(&sc).unwrap());
                assert_eq!(ca,1);
                if !separate {assert_eq!(cb,1);} else if [0,1,3,6,7].contains(&case) {assert_eq!(cb,0);}
                if case==2 {assert!(matches!(oa,Ok(GoalSelection::Goal(_))),"normal control needs goal: {oa:?}");assert_eq!(cb,1);}
                if case==3 {assert!(matches!(oa,Ok(GoalSelection::NoFrontier)));assert!(!a.diagnostics().last_candidates.is_empty());}
                cases+=1;
            }
        }}}
        println!("exact full-selector comparisons={cases}; includes production baseline, prepared deltas, diagnostics, session, IDs, repeated polls");
    }
    // Fixed process order and cohorts. Complete selector preparation is timed; setup/formatting are not.
    #[test]
    #[ignore="manual paired full-selector performance diagnostic"]
    fn lazy_inflation_full_selector_profile() {
        let reverse=std::env::var_os("LAZY_REVERSE").is_some();
        for (name,case,n,separate,calibrated) in [
            ("free101",0,101,true,false),("free401",0,401,true,false),
            ("blocked401",1,401,true,false),("filtered401",3,401,true,false),
            ("normal101",2,101,true,false),("normal401",2,401,true,false),
            ("calibrated101",2,101,true,true),("none_free401",0,401,false,false),
        ] {
            let(map,robot,session)=fixture(case,n);
            let mut a=make_selector(calibrated,false);let mut b=make_selector(calibrated,false);
            let _=call::<false>(&mut a,&map,robot,separate,&session);
            let _=call::<true>(&mut b,&map,robot,separate,&session);
            for sample in 0..5 {
                let mut ans=[None,None];let mut ns=[0u128;2];let mut counts=[0;2];
                for idx in if reverse {[1,0]} else {[0,1]} {
                    let start=std::time::Instant::now();
                    let (out,count)=if idx==0 {call::<false>(&mut a,&map,robot,separate,&session)}else{call::<true>(&mut b,&map,robot,separate,&session)};
                    ns[idx]=start.elapsed().as_nanos();counts[idx]=count;
                    ans[idx]=Some(out);
                }
                assert_eq!(format!("{:?}",ans[0]),format!("{:?}",ans[1]));
                assert_eq!(a.diagnostics(),b.diagnostics());
                println!("LAZY_PROFILE name={name} sample={sample} baseline_ns={} candidate_ns={} baseline_inflations={} candidate_inflations={}",ns[0],ns[1],counts[0],counts[1]);
                std::hint::black_box(ans);
            }
        }
    }
}
