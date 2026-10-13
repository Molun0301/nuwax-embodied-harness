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
    let tomogram = Tomogram::local_from_surface(&base,[0.5,1.],6.5).unwrap();
    let input_reference = [[0.5,1.,0.1],[3.,1.,0.1]];
    let actual = tomogram.plan_bounded(input_reference[0],input_reference[1],tomogram.resolution,
        &[],None,Some(7.5),&[]).unwrap().points;
    let detour: Vec<_> = waypoints.iter().map(|p|[p[0],p[1],0.1]).collect();
    eprintln!("RECENT_REFERENCES actual={actual:?} witness={detour:?}");
    let arrival = Some(([3.,1.,0.5],0.35));
    let mut results = Vec::new();
    for (name,reference) in [("A_static_pct",&actual),("B_witness_guide",&detour)] {
        eprintln!("RECENT_CASE_BEGIN {name}");
        let began = std::time::Instant::now();
        let result = local_search::plan(&map,&tomogram,start,reference,&motion,Some(0.),arrival,&[],5.,0.9,true);
        let work = local_search::last_work();
        if let Ok(path) = &result {
            for pair in path.windows(2) { map.validate_body_motion(pair[0],pair[1],&[]).unwrap(); }
        }
        eprintln!("RECENT_CASE_RESULT {name} work={work:?} ms={} result={:?}",
            began.elapsed().as_secs_f64()*1000.,result.as_ref().map(|p|p.len()));
        results.push(result);
    }
    assert_eq!(results[0].as_ref().unwrap_err().kind,LocalPlanFailureKind::BudgetExhausted,
        "A must reproduce the bounded-search failure");
    assert!(results[1].is_ok(),"verified detour guide still failed: {:?}",results[1]);
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
            let overlay=recent_guide_overlay::RecentGuideOverlay::new(hints,tomogram.resolution,model).unwrap();
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
