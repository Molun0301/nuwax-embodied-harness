//! Bounded diagnostic for the runtime's fixed-start recent-obstacle reproducer.
//! All body admission, action sets, costs and the 256-expansion limit are production values.
use super::*;
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope};
use nuwax_nav_types_messages::MotionModel;

fn fixture() -> (SurfaceSnapshot, SurfaceSnapshot, BodyPose, MotionKinematics) {
    let cfg = Config {
        world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.6,
        surface_closing_radius: 0.0, node_spacing_m: 0.4, wall_clearance_m: 0.2,
        wall_buffer_m: 0.2, wall_buffer_weight: 0.0, step_threshold_m: 0.2,
        max_slope_deg: 35.0, min_step_tread_depth_m: 0.3, step_penalty_weight: 0.0,
        goal_tolerance: 0.12, viz_publish_hz: 0.0,
    };
    let mut points = Vec::new();
    for ix in 0..65 { for iy in 0..35 {
        let x = (ix as f64 + 0.5) * 0.1;
        points.push((x as f32, (iy as f32 + 0.5) * 0.1, 0.05_f32));
    }}
    let base = SurfaceSnapshot::from_points(&points, cfg, 0.1).unwrap()
        .with_body_envelope(SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
            max_step_up_m: 0.2, max_step_down_m: 0.2,
            max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
        }).unwrap();
    let far = base.with_recent_obstacle_points(1,1_000_000_000,0,&[],[(0,[10.,10.,0.5])]).unwrap();
    let recent = far.with_recent_obstacle_points(2,2_000_000_000,0,&[],
        (0..5).map(|i|(i,[0.73,0.8+f64::from(i)*0.1,0.5]))).unwrap();
    let motion = MotionKinematics { model: MotionModel::Differential,
        max_forward_mps: 0.5, max_reverse_mps: 0.0, max_lateral_mps: 0.0,
        max_angular_rps: 0.8, minimum_turning_radius_m: 0.0, can_rotate_in_place: true };
    (base,recent,([0.5,1.,0.5],[0.,0.,0.,1.]),motion)
}

#[test]
#[ignore="explicit fixed-start A/B diagnostic; does not simulate controller motion"]
fn recent_wide_obstacle_fixed_start_reference_ab() {
    let (base,map,start,motion) = fixture();
    assert!(nuwax_nav_pct::same_source_geometry(&base,&map));
    map.validate_body_pose(start.0,start.1).unwrap();
    let waypoints: [[f64;2];5] = [[0.5,1.],[0.5,1.5],[1.,1.5],[1.,1.],[3.,1.]];
    let mut witness = vec![start];
    for target in &waypoints[1..] {
        let from = *witness.last().unwrap();
        let dx = target[0]-from.0[0]; let dy = target[1]-from.0[1];
        rotate(&map,&mut witness,dy.atan2(dx),&motion,&[]).expect("witness turn");
        let from = *witness.last().unwrap();
        witness.extend(probe_motion(&map,from,dx.hypot(dy),0.,&[]).expect("witness forward sweep"));
    }
    rotate(&map,&mut witness,0.,&motion,&[]).unwrap();
    for pair in witness.windows(2) { map.validate_body_motion(pair[0],pair[1],&[]).expect("full witness revalidation"); }
    eprintln!("RECENT_WITNESS samples={} end={:?}",witness.len(),witness.last());
    // A uses the static surface only. Recent obstacles never enter the tomogram
    // cost field (navigation_height_columns reads occupied_columns), so building
    // this from `map` instead of `base` yields the identical straight reference:
    // recent returns reach the search only through hints + recent guide overlay.
    let tomogram = Tomogram::local_from_surface(&base,[0.5,1.],6.5).unwrap();
    let input_reference = [[0.5,1.,0.1],[3.,1.,0.1]];
    let actual = tomogram.plan_bounded(input_reference[0],input_reference[1],tomogram.resolution,
        &[],None,Some(7.5),&[]).unwrap().points;
    let detour: Vec<_> = waypoints.iter().map(|p|[p[0],p[1],0.1]).collect();
    eprintln!("RECENT_REFERENCES actual={actual:?} witness={detour:?}");
    let arrival = Some(([3.,1.,0.5],0.35));
    // Optional probe: report which of the production action set's primitives
    // even project out of one fixed pose, so a stall can be attributed to the
    // swept-body admission rather than to queue ordering.
    if let Ok(pose) = std::env::var("NEH_RECENT_PROBE") {
        let values: Vec<f64> = pose.split(',').filter_map(|v| v.trim().parse().ok()).collect();
        if values.len() >= 4 {
            let (px, py, pz, heading) = (values[0], values[1], values[2], values[3]);
            let (s, c) = (heading * 0.5).sin_cos();
            let from = ([px, py, pz], [0.0, 0.0, s, c]);
            eprintln!("RECENT_PROBE_BEGIN {from:?}");
            // 区分两种失败：位姿本身非法（不能到达）还是位姿合法但无一步可走（死位姿）。
            match map.validate_body_pose(from.0, from.1) {
                Ok(_) => eprintln!("RECENT_PROBE standing=legal"),
                Err(error) => eprintln!("RECENT_PROBE standing=ILLEGAL {error:?}"),
            }
            let body = map.body_envelope().unwrap();
            let fine = (f64::from(map.source_voxel_size_m()) * 0.5).clamp(0.01, 0.05);
            let radius = body.radius().max(motion.minimum_turning_radius_m);
            let body_step = (radius * 0.35).clamp(fine * 2.0, 0.2);
            let body_turn = body_step / radius.max(fine);
            for (length, turn) in [
                (0.25_f64, 0.0_f64), (0.2, 0.25), (0.2, -0.25), (0.12, 0.45), (0.12, -0.45),
                (body_step, body_turn), (body_step, -body_turn),
                (0.05, 0.1), (0.05, -0.1), (0.05, 0.0),
                (0.0, 0.3), (0.0, -0.3), (0.0, 0.08), (0.0, -0.08),
            ] {
                let projection = local_search::probe_motion(&map, from, length, turn, &[]);
                let swept = local_search::probe_swept_admission(&map, from, length, turn, &[]);
                eprintln!("RECENT_PROBE length={length:.4} turn={turn:.4} projected={} swept={}",
                    projection.as_ref().map(|p| p.len()).map_or("none".into(), |n| n.to_string()),
                    match &swept { Ok(_) => "ok".to_string(), Err(e) => format!("{:?}", e.kind) });
                // 诊断：原始错误文本与首个被拒样本，用于区分支撑、净空、边界和接触。
                if let Err(error) = &swept {
                    eprintln!("RECENT_PROBE_REASON length={length:.4} turn={turn:.4} reason={} witness={:?}",
                        error.reason, error.body_witness);
                }
            }
        }
    }
    // Heuristic and budget are separate factors; keep them separable in the
    // attribution run instead of changing both at once.
    let modes: [(&str, local_search::HeuristicMode); 3] = [
        ("orig4", local_search::HeuristicMode::ScaledSum(4.0)),
        ("admMax", local_search::HeuristicMode::AdmissibleMax),
        ("admSum", local_search::HeuristicMode::AdmissibleSum),
    ];
    // Optional budget sweep: NEH_RECENT_BUDGETS="256,1024,8192" restricts the
    // run without touching the production constant or the heuristic.
    let budgets: Vec<usize> = std::env::var("NEH_RECENT_BUDGETS").ok()
        .filter(|value| !value.trim().is_empty())
        .map_or_else(|| vec![DEFAULT_SEARCH_EXPANSION_LIMIT],
            |value| value.split(',').filter_map(|v| v.trim().parse().ok()).collect());
    let mut results = Vec::new();
    for budget in &budgets {
    for (mode_name, mode) in modes {
    for (name,reference) in [("A_static_pct",&actual),("B_witness_guide",&detour)] {
        eprintln!("RECENT_CASE_BEGIN {mode_name} {name} budget={budget}");
        let began = std::time::Instant::now();
        let result = local_search::with_heuristic_mode(mode, || local_search::with_expansion_limit(
            *budget, || local_search::plan(
            &map,&tomogram,start,reference,&motion,Some(0.),arrival,&[],5.,0.9,true,
            DEFAULT_SEARCH_EXPANSION_LIMIT)));
        let work = local_search::last_work();
        if let Ok(path) = &result {
            for pair in path.windows(2) { map.validate_body_motion(pair[0],pair[1],&[]).unwrap(); }
        }
        eprintln!("RECENT_CASE_RESULT {mode_name} {name} budget={budget} work={work:?} ms={} result={:?}",
            began.elapsed().as_secs_f64()*1000.,result.as_ref().map(|p|p.len()));
        results.push(((mode_name,budget),name,result));
    }
    }
    }
    // The original weighted heuristic must keep reproducing the reported failure
    // at the original 256-expansion bound, otherwise the diagnostic stopped
    // measuring the historical case.
    if budgets.iter().any(|value| *value == 256usize) {
        let find = |mode:&str, name:&str| results.iter()
            .find(|((m,b),n,_)| *m==mode && **b==256usize && *n==name).map(|(_,_,r)| r).unwrap();
        assert_eq!(find("orig4","A_static_pct").as_ref().unwrap_err().kind,
            LocalPlanFailureKind::BudgetExhausted,
            "原始加权启发式必须复现预算耗尽");
        assert!(find("orig4","B_witness_guide").is_ok(),
            "原始加权启发式在见证参考上必须成功");
    }
}

// Reuse the production request-local overlay instead of a benchmark-only copy.
// This is a test-only source dependency, not a dependency on the runtime crate.
#[cfg(feature="profile")]
#[path="../../nuwax-nav-runtime/src/recent_guide_overlay.rs"]
mod recent_guide_overlay;

#[cfg(feature="profile")]
#[test]
#[ignore="bounded release A/B; choose A or B for independent fresh-process measurements"]
fn recent_wide_obstacle_exported_guide_ab() {
    let requested=std::env::var("NEH_RECENT_AB_CASE").unwrap_or_else(|_|"both".into());
    assert!(["A","B","both"].contains(&requested.as_str()));
    let (base,map,start,motion)=fixture();
    map.validate_body_pose(start.0,start.1).unwrap();
    let tomogram=Tomogram::local_from_surface(&base,[0.5,1.0],6.5).unwrap();
    for label in ["A","B"] {
        if requested!="both" && requested!=label {continue;}
        let began=std::time::Instant::now();
        let mut reference=tomogram.plan_bounded([0.5,1.0,0.1],[3.0,1.0,0.1],
            tomogram.resolution,&[],None,Some(7.5),&[]).unwrap().points;
        let mut hints_count=0;
        if label=="B" {
            let model=map.body_envelope().unwrap();
            let radius=6.5+model.radius();
            let hints=map.recent_obstacle_hints([[0.5-radius,1.0-radius],[0.5+radius,1.0+radius]],start).unwrap();
            hints_count=hints.len();
            let overlay=recent_guide_overlay::RecentGuideOverlay::new(hints,tomogram.resolution,f64::from(map.source_voxel_size_m()),model).unwrap();
            assert!(!overlay.is_empty(),"real recent returns must reach the guide");
            reference=tomogram.plan_bounded_with_segment_filter(reference[0],*reference.last().unwrap(),
                tomogram.resolution,&[],None,Some(7.5),&[],&|a,b|overlay.accepts(a,b)).unwrap().points;
            assert!(reference.iter().any(|p|(p[1]-1.0).abs()>0.1));
        }
        let guide_ms=began.elapsed().as_secs_f64()*1000.0;
        eprintln!("RECENT_EXPORTED_CASE_BEGIN {label} hints={hints_count} reference={reference:?}");
        let scan_began=std::time::Instant::now();
        let result=plan(&map,&tomogram,start,&reference,&motion,&ScanConfig::default(),Some(0.0),
            Some(([3.0,1.0,0.5],0.35)),&[],true,false);
        let scan_ms=scan_began.elapsed().as_secs_f64()*1000.0;
        let total_ms=began.elapsed().as_secs_f64()*1000.0;
        if label=="A" {
            assert_eq!(result.as_ref().unwrap_err().kind,LocalPlanFailureKind::BudgetExhausted);
        } else {
            let path=&result.as_ref().unwrap().poses;
            for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
        }
        eprintln!("RECENT_EXPORTED_RESULT {}",serde_json::json!({"case":label,"guide_ms":guide_ms,
            "scan_ms":scan_ms,"total_ms":total_ms,"hints":hints_count,"reference":reference,
            "success":result.is_ok(),"path_poses":result.as_ref().ok().map(|r|r.poses.len()),
            "failure":result.as_ref().err().map(|e|format!("{:?}",e.kind))}));
    }
}

#[cfg(feature="profile")]
#[path="search_priority_diagnostic_tests.rs"]
mod search_priority_diagnostic_tests;

#[test]
fn recent_budget_failure_recovers_after_current_source_clears() {
    let (base,map,start,motion)=fixture();
    let tomogram=Tomogram::local_from_surface(&base,[0.5,1.0],6.5).unwrap();
    let reference=[[0.5,1.0,0.1],[3.0,1.0,0.1]];
    let arrival=Some(([3.0,1.0,0.5],0.35));
    let began=std::time::Instant::now();
    let failed=local_search::plan(&map,&tomogram,start,&reference,&motion,Some(0.0),arrival,
        &[],5.0,0.9,true,DEFAULT_SEARCH_EXPANSION_LIMIT).unwrap_err();
    let failure_ms=began.elapsed().as_secs_f64()*1000.0;
    assert_eq!(failed.kind,LocalPlanFailureKind::BudgetExhausted);
    assert!(failed.blocked.is_none(),"预算耗尽不能冒充全走廊不可通行的证据");
    let clear=map.with_recent_obstacle_points(3,3_000_000_000,0,&[],[]).unwrap();
    assert!(nuwax_nav_pct::same_source_geometry(&map,&clear));
    let began=std::time::Instant::now();
    let path=local_search::plan(&clear,&tomogram,start,&reference,&motion,Some(0.0),arrival,
        &[],5.0,0.9,true,DEFAULT_SEARCH_EXPANSION_LIMIT).unwrap();
    let recovery_ms=began.elapsed().as_secs_f64()*1000.0;
    assert!(distance3(path.last().unwrap().0,arrival.unwrap().0)<0.35);
    for pair in path.windows(2) {clear.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
    assert!(path.windows(2).any(|p|map.validate_body_motion(p[0],p[1],&[]).is_err()),
        "旧快照仍必须挡住旧障碍，不能通过改写地图使恢复通过");
    eprintln!("RECENT_BUDGET_RECOVERY failure_ms={failure_ms} recovery_ms={recovery_ms} poses={}",path.len());
}

#[cfg(feature="profile")]
#[path="recent_fine_escape_diagnostic_tests.rs"]
mod recent_fine_escape_diagnostic_tests;

#[cfg(feature="profile")]
#[path="recent_pipeline_eligibility_tests.rs"]
mod recent_pipeline_eligibility_tests;
