//! Eligibility experiment: real recent guide, static body-sensitive departure.
use super::*;

#[test]
fn recent_guide_fine_escape_enters_search_and_succeeds() {
    check_recent_guide_fine_escape(Some(0.015));
}
#[test]
fn recent_guide_nonarrival_fine_escape_reuses_exact_polyline_failure() {
    check_recent_guide_fine_escape(None);
}
fn check_recent_guide_fine_escape(tolerance: Option<f64>) {
    // Same two rear pins and forward wall as fine_escape_scene(false,[0,0],false,6).
    let mut extra=vec![];
    for z in 25..55 {
        extra.push([-29,6,z]);extra.push([-29,-7,z]);
        for y in -50..20 {extra.push([37,y,z]);}
    }
    // Extend only independent flat support, so a downstream recent obstacle and
    // the complete witnessed detour fit inside the test surface.
    extra.extend((-130..131).flat_map(|x|(100..230).map(move |y|[x,y,-1])));
    let base=crate::tests::scene_with_extra_cells(None,0.0,&extra)
        .with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m:vec![[-0.3,-0.0599],[0.3,-0.0599],[0.3,0.0599],[-0.3,0.0599]],
            z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.4,max_step_up_m:0.0,max_step_down_m:0.0,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0,
        }).unwrap();
    let map=base.with_recent_obstacle_points(2,2_000_000_000,0,&[],
        (-3..=3).map(|i|((i+3) as u32,[f64::from(i)*0.1,1.0,0.4]))).unwrap();
    let start=([0.011,0.0,0.4],[0.0,0.0,0.0,1.0]);
    let motion=crate::tests::motion();
    map.validate_body_pose(start.0,start.1).unwrap();
    for turn in [0.08,-0.08,0.3,-0.3] { assert!(probe_motion(&map,start,0.0,turn,&[]).is_none()); }
    for turn in [0.1,-0.1] { assert!(probe_motion(&map,start,0.01,turn,&[]).is_none()); }
    assert!(probe_motion(&map,start,0.25,0.0,&[]).is_none());
    for (length,turn) in [(0.2,0.25),(0.2,-0.25),(0.12,0.45),(0.12,-0.45)] {
        assert!(probe_motion(&map,start,length,turn,&[]).is_none());
    }
    let points=[[0.011,0.0,0.0],[0.021,0.0,0.0],[0.021,0.45,0.0],
        [-0.65,0.45,0.0],[-0.65,1.8,0.0],[0.021,1.8,0.0]];
    let mut witness=vec![start];
    for point in &points[1..] {
        let from=*witness.last().unwrap();let dx=point[0]-from.0[0];let dy=point[1]-from.0[1];
        rotate(&map,&mut witness,dy.atan2(dx),&motion,&[]).expect("independent extended witness turn");
        witness.extend(probe_motion(&map,*witness.last().unwrap(),dx.hypot(dy),0.0,&[]).expect("independent extended witness translation"));
    }
    assert!(witness.iter().any(|p|distance3(p.0,[0.021,0.0,0.4])<1e-9),"witness retains exact 1 cm departure");
    let goal=[0.021,1.8,0.4];
    for pair in witness.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).expect("extended witness full resweep");}
    assert!(distance3(witness.last().unwrap().0,goal)<0.01);
    eprintln!("FALLBACK_ELIGIBILITY witness_samples={} independent_resweep=pass",witness.len());
    let tomo=Tomogram::local_from_surface(&base,[start.0[0],start.0[1]],3.0).unwrap();
    let from=[start.0[0],start.0[1],0.0];let to=[goal[0],goal[1],0.0];
    let original=tomo.plan_bounded(from,to,tomo.resolution,&[],None,Some(5.0),&[]).expect("static guide").points;
    let hints=map.recent_obstacle_hints([[-3.0,-3.0],[3.0,3.0]],start).unwrap();
    eprintln!("FALLBACK_ELIGIBILITY hints={} original={original:?}",hints.len());
    let overlay=recent_guide_overlay::RecentGuideOverlay::new(hints,tomo.resolution,f64::from(map.source_voxel_size_m()),map.body_envelope().unwrap()).unwrap();
    assert!(!overlay.is_empty());
    let reference=tomo.plan_bounded_with_segment_filter(from,to,tomo.resolution,&[],None,Some(5.0),&[],
        &|a,b|overlay.accepts(a,b)).expect("real recent guide must exist").points;
    assert_ne!(reference,original,"recent guide must materially differ");
    eprintln!("FALLBACK_ELIGIBILITY reference={reference:?}");
    let mut results=Vec::new();
    let mut works=Vec::new();
    let mut counts=Vec::new();
    for replay in [true,false] {
        let request_map=map.fork_for_planning();
        local_search::reset_last_work();
        let result=crate::polyline_reuse_tests::with_original(replay,||
            plan(&request_map,&tomo,start,&reference,&motion,&ScanConfig::default(),None,
                tolerance.map(|t|(goal,t)),&[],true,false));
        let work=local_search::last_work();
        let calls=crate::profile::calls();
        assert_eq!(crate::polyline_reuse_tests::projections(),if replay || tolerance.is_some() {2}else{1});
        if let Ok(path)=&result {
            for pair in path.poses.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            let end=*path.poses.last().unwrap();
            if let Some(tolerance)=tolerance {
                assert!(distance3(end.0,goal)<tolerance);
            } else {
                let a=reference[reference.len()-2];let b=*reference.last().unwrap();
                let tangent=(b[1]-a[1]).atan2(b[0]-a[0]);
                assert!((end.0[0]-b[0])*tangent.cos()+(end.0[1]-b[1])*tangent.sin()>=-1e-6,
                    "non-arrival window must connect to its terminal cross-section");
                assert!(angle(yaw(end.1)-tangent).cos()>0.0);
            }
        }
        eprintln!("FALLBACK_ELIGIBILITY_RESULT {}",serde_json::json!({"arrival":tolerance.is_some(),
            "replay_original":replay,"fallback_entered":work.1>0,"calls":calls,
            "work":{"expanded":work.0,"nodes":work.1,"frontier_peak":work.2},
            "success":result.is_ok(),"path_samples":result.as_ref().ok().map(|p|p.poses.len()),
            "failure":result.as_ref().err().map(|e|format!("{:?}",e.kind)),
            "detail":result.as_ref().err().map(|e|format!("{e:?}"))}));
        assert!(work.1>0,"fixture is ineligible if exported Scan never enters fallback");
        let path=result.as_ref().expect("current real-guide pipeline must retain witnessed fine escape");
        assert!(path.poses.iter().any(|p|distance3(p.0,[0.021,0.0,0.4])<1e-9),
            "search must retain the exact 1 cm departure before turning");
        counts.push(calls);works.push(work);results.push(result);
    }
    crate::polyline_reuse_tests::assert_same_result(&results[0],&results[1]);
    assert_eq!(works[0],works[1]);
    if tolerance.is_none() {
        assert!(counts[0].iter().zip(counts[1]).all(|(a,b)|*a>=b));
        assert!(counts[0].iter().zip(counts[1]).any(|(a,b)|*a>b));
    } else {assert_eq!(counts[0],counts[1]);}
}
