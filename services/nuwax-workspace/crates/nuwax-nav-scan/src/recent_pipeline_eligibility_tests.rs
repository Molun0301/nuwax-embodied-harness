//! Actual request-local recent guide for the five former forced-static fixtures.
use super::*;
#[test]
#[ignore="explicit current-pipeline eligibility diagnostic, no alternative heuristic"]
fn five_static_ablation_cases_through_actual_recent_guide() {
    let (base,_,original,motion)=fixture();
    let tomo=Tomogram::local_from_surface(&base,[0.5,1.0],6.5).unwrap();
    for (name,wall_x,wall_y,heading) in [
        ("static_a",0.73,1.0,0.0),
        ("shift_upper",0.73,1.15,0.0),
        ("shift_lower",0.73,0.85,0.0),
        ("reversed",0.73,1.0,std::f64::consts::PI),
        ("farther_wall",1.1,1.0,0.0),
    ] {
        let map=base.with_recent_obstacle_points(2,2_000_000_000,0,&[],
            (0..5).map(|i|(i,[wall_x,wall_y-0.2+f64::from(i)*0.1,0.5]))).unwrap();
        let start=(original.0,[0.0,0.0,(heading*0.5).sin(),(heading*0.5).cos()]);
        map.validate_body_pose(start.0,start.1).unwrap();
        let static_reference=tomo.plan_bounded([0.5,1.0,0.1],[3.0,1.0,0.1],
            tomo.resolution,&[],None,Some(7.5),&[]).unwrap().points;
        let radius=6.5+map.body_envelope().unwrap().radius();
        let hints=map.recent_obstacle_hints([[0.5-radius,1.0-radius],[0.5+radius,1.0+radius]],start).unwrap();
        assert!(!hints.is_empty());
        let overlay=recent_guide_overlay::RecentGuideOverlay::new(hints,tomo.resolution,f64::from(map.source_voxel_size_m()),map.body_envelope().unwrap()).unwrap();
        let guide=tomo.plan_bounded_with_segment_filter(static_reference[0],*static_reference.last().unwrap(),
            tomo.resolution,&[],None,Some(7.5),&[],&|a,b|overlay.accepts(a,b));
        let guide_success=guide.is_ok();
        // Match the real owner: keep static guidance if radius overlay has no route.
        let reference=guide.map_or_else(|_|static_reference.clone(),|r|r.points);
        local_search::reset_last_work();
        let result=plan(&map,&tomo,start,&reference,&motion,&ScanConfig::default(),Some(0.0),
            Some(([3.0,1.0,0.5],0.35)),&[],true,false);
        let work=local_search::last_work();
        if let Ok(path)=&result {
            for pair in path.poses.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).unwrap();}
            assert!(distance3(path.poses.last().unwrap().0,[3.0,1.0,0.5])<0.35);
            assert!(angle(yaw(path.poses.last().unwrap().1)).abs()<0.1);
        }
        eprintln!("RECENT_PIPELINE_ELIGIBILITY {}",serde_json::json!({"case":name,
            "guide_success":guide_success,"guide_changed":reference!=static_reference,"reference":reference,
            "success":result.is_ok(),"path_samples":result.as_ref().ok().map(|p|p.poses.len()),
            "fallback_entered":work.1>0,"work":{"expanded":work.0,"nodes":work.1,"frontier_peak":work.2},
            "failure":result.as_ref().err().map(|e|format!("{:?}",e.kind))}));
    }
}
