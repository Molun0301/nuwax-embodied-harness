//! Synthetic priority ablations. Oracle polyline is a positive control only.
use super::*;

fn witness(map: &SurfaceSnapshot, start: BodyPose, motion: &MotionKinematics,
    points: &[[f64;3]], final_yaw: f64) -> Vec<BodyPose> {
    let mut path=vec![start];
    for point in &points[1..] {
        let from=*path.last().unwrap();
        let dx=point[0]-from.0[0];let dy=point[1]-from.0[1];
        rotate(map,&mut path,dy.atan2(dx),motion,&[]).expect("independent witness rotation");
        path.extend(probe_motion(map,*path.last().unwrap(),dx.hypot(dy),0.0,&[]).expect("independent witness translation"));
    }
    rotate(map,&mut path,final_yaw,motion,&[]).unwrap();
    for pair in path.windows(2) {map.validate_body_motion(pair[0],pair[1],&[]).expect("independent witness resweep");}
    path
}

#[test]
#[ignore="explicit synthetic priority ablations; no production performance claim"]
fn synthetic_priority_ablation() {
    let (base,_,original,motion)=fixture();
    let tomo=Tomogram::local_from_surface(&base,[0.5,1.0],6.5).unwrap();
    let selected=std::env::var("NEH_PRIORITY_CASE").ok();
    let selected_mode=std::env::var("NEH_PRIORITY_MODE").ok();
    for (name,wall_x,wall_y,escape,heading,negative) in [
        ("static_a",0.73,1.0,1.0,0.0,0),
        ("shift_upper",0.73,1.15,-1.0,0.0,0),
        ("shift_lower",0.73,0.85,1.0,0.0,0),
        ("reversed",0.73,1.0,1.0,std::f64::consts::PI,0),
        ("farther_wall",1.1,1.0,1.0,0.0,0),
        ("clear",10.0,1.0,1.0,0.0,0),
        ("unreachable_height",10.0,1.0,1.0,0.0,1),
        ("excluded_boundary",10.0,1.0,1.0,0.0,2),
    ] {
        if selected.as_ref().is_some_and(|s|s!=name) {continue;}
        let map=base.with_recent_obstacle_points(2,2_000_000_000,0,&[],
            (0..5).map(|i|(i,[wall_x,wall_y-0.2+f64::from(i)*0.1,0.5]))).unwrap();
        let start=(original.0,[0.0,0.0,(heading*0.5).sin(),(heading*0.5).cos()]);
        map.validate_body_pose(start.0,start.1).unwrap();
        let reference=vec![[0.5,1.0,0.1],[3.0,1.0,0.1]];
        let oracle=if wall_x<5.0 {vec![[0.5,1.0,0.1],[0.5,1.0+escape*0.6,0.1],
            [wall_x+0.5,1.0+escape*0.6,0.1],[wall_x+0.5,1.0,0.1],[3.0,1.0,0.1]]} else {reference.clone()};
        if negative==0 {
            let path=witness(&map,start,&motion,&oracle,0.0);
            assert!(distance3(path.last().unwrap().0,[3.0,1.0,0.5])<0.01);
            eprintln!("PRIORITY_WITNESS case={name} samples={} resweep=pass",path.len());
        }
        let excluded=[[4.0,2.0],[5.0,2.0],[5.0,3.0],[4.0,3.0]];
        let boundary=if negative==2 {&excluded[..]} else {&[]};
        let goal=[3.0,1.0,if negative==1 {2.0} else {0.5}];
        for (mode,priority) in [
            ("baseline",local_search::DiagnosticPriority::Baseline),
            ("relaxed",local_search::DiagnosticPriority::RelaxedGoal(1.0)),
            ("weighted_relaxed",local_search::DiagnosticPriority::RelaxedGoal(4.0)),
            ("oracle",local_search::DiagnosticPriority::OracleGuide(oracle.clone())),
        ] {
            if selected_mode.as_ref().is_some_and(|s|s!=mode) {continue;}
            eprintln!("PRIORITY_BEGIN case={name} mode={mode}");
            let result=local_search::with_diagnostic_priority(priority,||local_search::plan(
                &map,&tomo,start,&reference,&motion,Some(0.0),Some((goal,0.35)),boundary,5.0,0.9,true, DEFAULT_SEARCH_EXPANSION_LIMIT));
            let work=local_search::last_work();
            if let Ok(path)=&result {
                for pair in path.windows(2) {
                    map.validate_body_motion(pair[0],pair[1],boundary).unwrap();
                    let d=[pair[1].0[0]-pair[0].0[0],pair[1].0[1]-pair[0].0[1]];
                    assert!(d[0]*yaw(pair[0].1).cos()+d[1]*yaw(pair[0].1).sin()>=-1e-9);
                }
                assert!(distance3(path.last().unwrap().0,goal)<0.35);
                assert!(angle(yaw(path.last().unwrap().1)).abs()<0.1);
                assert_eq!(negative,0,"negative control cannot succeed");
            }
            if negative!=0 { assert!(result.is_err()); assert!(result.as_ref().unwrap_err().blocked.is_none()); }
            eprintln!("PRIORITY_RESULT {}",serde_json::json!({"case":name,"mode":mode,
                "success":result.is_ok(),"work":{"expanded":work.0,"nodes":work.1,"frontier_peak":work.2},
                "path_samples":result.as_ref().ok().map(Vec::len),
                "failure":result.as_ref().err().map(|e|format!("{:?}",e.kind)),
                "closest":result.as_ref().err().and_then(|e|e.closest_pose)}));
        }
    }
}
