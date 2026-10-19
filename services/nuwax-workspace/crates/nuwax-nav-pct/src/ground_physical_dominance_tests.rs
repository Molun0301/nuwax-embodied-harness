//! Real source-backed production Ground entry, paired with its original-order
//! specialization. Full output comparison excludes only elapsed diagnostics.
use super::*;
use crate::ground_route::physical_test_capture::{capture,Call};

fn signature(result:&Result<PlannedGroundRoute,GroundRouteFailure>)->String {
    match result {
        Err(error)=>format!("{error:?}"),
        Ok(r)=>format!("{:?}|{:?}|{:?}|{:?}|{:?}|{:?}",
            r.points.iter().map(|p|(p.xy.map(f64::to_bits),p.ground_z.map(f64::to_bits))).collect::<Vec<_>>(),
            r.guide_body_poses.iter().map(|p|(p.0.map(f64::to_bits),p.1.map(f64::to_bits))).collect::<Vec<_>>(),
            (r.height,r.planned_layer),(r.stats.expanded,r.stats.cells),
            r.reference.connections().map(|c|(c.point_span,c.steps)).collect::<Vec<_>>(),
            trace(r).iter().map(|r|(r.0,r.1,r.2.xy.map(f64::to_bits),r.2.ground_z.map(f64::to_bits),r.3)).collect::<Vec<_>>()),
    }
}
fn actual_pair(source:impl Fn()->SurfaceSnapshot,actual:([f64;3],[f64;4]),start:GroundRouteStart,
    goal:GroundRouteGoal,boundary:&[[f64;2]],radius:Option<f64>,penalties:&[crate::RoutePenalty])
    ->[Vec<Call>;2] {
    let old_source=source();let new_source=source();
    let old=crate::PctPlanner::default();let new=crate::PctPlanner::default();
    let mut last=[Vec::new(),Vec::new()];
    for warm in [false,true] {
        let (a,ac)=capture(||old.ground_route_original_order(&old_source,actual,start,goal,0.,boundary,None,radius,
            SearchLimits::default(),penalties));
        let (b,bc)=capture(||new.ground_route_with_limits_and_penalties(&new_source,actual,start,goal,0.,boundary,None,radius,
            SearchLimits::default(),penalties));
        assert_eq!(signature(&a),signature(&b));
        let output=signature(&b);let a=a.unwrap();let b=b.unwrap();
        assert!(a.reference.matches_source(&old_source));assert!(b.reference.matches_source(&new_source));
        let mut tail=ac.as_slice();for call in &bc {let i=tail.iter().position(|old|old==call).unwrap();tail=&tail[i+1..];}
        // Validate every actual output segment after capture; not part of counts.
        for edge in b.guide_body_poses.windows(2) {new_source.body_query()
            .validate_unanchored_motion(edge[0],edge[1],boundary,&mut None).unwrap();}
        eprintln!("P1_PHYSICAL warm={warm} old={} new={} expanded={} old_ms={} new_ms={} output={output}",
            ac.len(),bc.len(),b.stats.expanded,a.stats.elapsed_ms,b.stats.elapsed_ms);
        last=[ac,bc];
    }
    last
}

#[test]
fn p1_01_actual_ground_corridor_skips_return_sweeps_cold_and_warm() {
    let xy=center([0,0]);let end=center([4,0]);let width=5.*q();
    let boundary=[[0.,0.],[width,0.],[width,q()],[0.,q()]];
    let actual=([xy[0],xy[1],0.5],[0.,0.,0.,1.]);
    let penalties=[crate::RoutePenalty{center:[10.,0.,0.5],radius:0.1,weight:1.}];
    let [old,new]=actual_pair(empty_source,actual,GroundRouteStart{xy,ground_z:None},
        GroundRouteGoal{xy:end,requested_ground_z:None},&boundary,Some(1.),&penalties);
    let start=[xy[0],xy[1],0.5].map(f64::to_bits);
    assert!(old.iter().any(|c|c.0[1]==start&&c.0[0]!=start));
    assert!(!new.iter().any(|c|c.0[1]==start&&c.0[0]!=start));
    assert!(new.len()<old.len());
}

#[test]
fn p1_02_actual_ground_gap_and_real_wall_keep_output_and_reducer_checks() {
    let xy=center([0,0]);let h=thin([0,0],0).height();
    let penalties=[crate::RoutePenalty{center:[10.,0.,h+0.4],radius:0.1,weight:1.}];
    let [old,new]=actual_pair(||known_source().0,([xy[0],xy[1],h+0.4],[0.,0.,0.,1.]),
        GroundRouteStart{xy,ground_z:Some(h)},GroundRouteGoal{xy:center([8,0]),requested_ground_z:Some(h)},
        &[[0.,0.],[9.*q(),0.],[9.*q(),q()],[0.,q()]],Some(1.),&penalties);
    assert!(new.len()<old.len());
    // Same C1-a real admitted wall, with its original source and bounded goal.
    let make=|| {
        let columns=(6..8).flat_map(|x|(-16..16).map(move|y|([x,y],ColumnGroundRecords{
            classified_empty:true,legacy:vec![],heightfields:vec![]}))).collect();
        let keys=(6..8).flat_map(|x|(-16..16).flat_map(move|y|(-2..20).map(move|z|[x,y,z])));
        scene(columns,keys,config(0.4),Some(model())).source
    };
    let [old,new]=actual_pair(make,([0.05,0.05,0.5],[0.,0.,0.,1.]),
        GroundRouteStart{xy:[0.05,0.05],ground_z:None},GroundRouteGoal{xy:[0.55,0.05],requested_ground_z:None},
        &[[-0.2,-1.],[1.,-1.],[1.,1.],[-0.2,1.]],Some(1.5),&[]);
    assert!(new.len()<old.len());assert!(old.iter().any(|c|!c.1));assert!(new.iter().any(|c|!c.1));
}
