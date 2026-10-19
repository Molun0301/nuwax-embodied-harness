//! C1-a fixed source-backed planning oracles. No execution/arrival authority.
use super::*;
use crate::ground_route::*;
use nuwax_types::navigation::GroundLayerResult;
use std::sync::atomic::AtomicBool;

fn model() -> SurfaceBodyEnvelope {
    SurfaceBodyEnvelope {fixed_collision:None,
        footprint_xy_m:vec![[-0.01,-0.01],[0.01,-0.01],[0.01,0.01],[-0.01,0.01]],
        z_bounds_m:[-0.1,0.1],body_to_support_z_m:0.4,
        max_step_up_m:0.15,max_step_down_m:0.15,max_cross_slope_deg:20.0,support_uncertainty_m:0.0}
}
fn empty_source()->SurfaceSnapshot {
    scene(vec![([30,30],ColumnGroundRecords {classified_empty:true,legacy:vec![],heightfields:vec![]})],
        [[30,30,0]],config(0.3),Some(model())).source
}
fn known_source()->(SurfaceSnapshot,f64) {
    let c=thin([0,0],0);let h=c.height();let mut cfg=config(0.3);cfg.max_slope_deg=20.0;cfg.step_threshold_m=0.15;
    (scene(vec![([0,0],native(vec![c])),([8,0],native(vec![thin([8,0],0)]))],
        [],cfg,Some(model())).source,h)
}
fn plan(source:&SurfaceSnapshot,actual:([f64;3],[f64;4]),start:GroundRouteStart,goal:GroundRouteGoal)
    ->Result<PlannedGroundRoute,GroundRouteFailure> {
    crate::PctPlanner::default().ground_route_with_limits_and_penalties(source,actual,start,goal,0.0,
        &[],None,None,SearchLimits::default(),&[])
}
fn trace(route:&PlannedGroundRoute)->Vec<(usize,usize,GroundRoutePoint,u8)> {
    route.reference.connections().enumerate().flat_map(|(c,connection)|(0..=connection.steps).map(move|i| {
        let sample=route.reference.sample(c,i).unwrap();
        (c,i,sample.point,match sample.association {PlannedGroundAssociation::Known {..}=>0,
            PlannedGroundAssociation::Carried=>1,PlannedGroundAssociation::Unanchored=>2})
    })).collect()
}

#[test]
fn c1a_01_requested_presence_and_unanchored_physical_coordinate_are_separate() {
    let source=empty_source();let start=GroundRouteStart {xy:[0.05,0.05],ground_z:None};
    for requested in [None,Some(0.0),Some(-0.0),Some(-0.25),Some(2.75)] {
        for z in [0.5,0.7] {
            let route=plan(&source,([0.05,0.05,z],[0.0,0.0,0.0,1.0]),start,
                GroundRouteGoal {xy:[0.25,0.05],requested_ground_z:requested}).unwrap();
            assert_eq!(route.height.requested_ground_z.map(f64::to_bits),requested.map(f64::to_bits));
            assert_eq!(route.height.resolved_ground_z,None);
            assert_eq!(route.planned_layer,GroundLayerResult::Unresolved);
            assert!(route.points.iter().all(|p|p.ground_z.is_none()));
            assert!(trace(&route).iter().all(|s|s.2.ground_z.is_none()&&s.3==2));
            assert!(route.guide_body_poses.iter().all(|p|p.0[2]==z));
        }
    }
    let mut other=model();other.body_to_support_z_m=0.6;
    let changed=source.clone().with_body_envelope(other).unwrap();
    let a=plan(&source,([0.05,0.05,0.7],[0.,0.,0.,1.]),start,GroundRouteGoal {xy:[0.25,0.05],requested_ground_z:None}).unwrap();
    let b=plan(&changed,([0.05,0.05,0.7],[0.,0.,0.,1.]),start,GroundRouteGoal {xy:[0.25,0.05],requested_ground_z:None}).unwrap();
    assert_eq!(a.points,b.points);assert_eq!(a.height,b.height);
    let actual=([0.07,0.09,0.5],[0.,0.,0.,1.]);
    let offset=plan(&source,actual,start,GroundRouteGoal {xy:[0.25,0.05],requested_ground_z:None}).unwrap();
    assert_eq!(offset.guide_body_poses[0].0.map(f64::to_bits),actual.0.map(f64::to_bits));
    assert_eq!(offset.guide_body_poses[0].1.map(f64::to_bits),actual.1.map(f64::to_bits));
}

#[test]
fn c1a_02_known_trace_borrows_original_native_records_and_plane_height() {
    let (source,h)=known_source();let xy=center([0,0]);
    let route=plan(&source,([xy[0],xy[1],h+0.4],[0.,0.,0.,1.]),GroundRouteStart {xy,ground_z:Some(h)},
        GroundRouteGoal {xy:[xy[0]+0.01,xy[1]],requested_ground_z:Some(h)}).unwrap();
    let original=source.observations().unwrap().ground_records([0,0]).next().unwrap();
    for row in trace(&route) {
        let sample=route.reference.sample(row.0,row.1).unwrap();
        let PlannedGroundAssociation::Known {source:Some(record)}=sample.association else {panic!("native Known lost its source")};
        assert_eq!(record.key(),original.key());
        match (record,original) {
            (nuwax_nav_mls_planner::surface::SourceGroundRecord::HeightfieldV1(a),
             nuwax_nav_mls_planner::surface::SourceGroundRecord::HeightfieldV1(b))=>assert!(std::ptr::eq(a,b)),
            _=>panic!("wrong original representation"),
        }
        assert_eq!(sample.point.ground_z,Some(h));
    }
    let plane=sloped_legacy_fixture();let from=center([0,0]);let to=[from[0]+0.01,from[1]];
    let original=plane.source.observations().unwrap().ground_records([0,0]).next().unwrap();
    let a=original.height_at(from).unwrap();let b=original.height_at(to).unwrap();
    let route=plan(&plane.source,([from[0],from[1],a+0.3],[0.,0.,0.,1.]),
        GroundRouteStart {xy:from,ground_z:Some(a)},GroundRouteGoal {xy:to,requested_ground_z:Some(b)}).unwrap();
    let last=route.reference.connections().enumerate().last().unwrap();
    let sample=route.reference.sample(last.0,last.1.steps).unwrap();
    assert_eq!(sample.point.ground_z,Some(b));assert!(b>a);
    let PlannedGroundAssociation::Known {source:Some(record)}=sample.association else {panic!("plane source lost")};
    match (record,original) {
        (nuwax_nav_mls_planner::surface::SourceGroundRecord::LegacyPlane(a),
         nuwax_nav_mls_planner::surface::SourceGroundRecord::LegacyPlane(b))=>assert!(std::ptr::eq(a,b)),
        _=>panic!("plane was replaced by a constant fake normal"),
    }
}

#[test]
fn c1a_03_unanchored_same_search_detours_real_wall_with_full_body_queries() {
    // Same fixed wall AABB as the existing bounded unknown-reference control;
    // actual Writer/Receiver records retire ground claims, never occupancy.
    let columns=(6..8).flat_map(|x|(-16..16).map(move|y|([x,y],ColumnGroundRecords {
        classified_empty:true,legacy:vec![],heightfields:vec![]}))).collect();
    let keys=(6..8).flat_map(|x|(-16..16).flat_map(move|y|(-2..20).map(move|z|[x,y,z])));
    let source=scene(columns,keys,config(0.4),Some(model())).source;
    let planner=crate::PctPlanner::default();assert!(planner.map(&source).unwrap().cells.is_empty());
    let actual=([0.05,0.05,0.5],[0.,0.,0.,1.]);
    let start=GroundRouteStart {xy:[0.05,0.05],ground_z:None};
    let goal=GroundRouteGoal {xy:[0.55,0.05],requested_ground_z:None};
    let boundary=[[-0.2,-1.0],[1.0,-1.0],[1.0,1.0],[-0.2,1.0]];
    let query=|radius,limit|planner.ground_route_with_limits_and_penalties(&source,actual,start,goal,0.0,
        &boundary,None,Some(radius),SearchLimits {max_states:limit},&[]);
    assert!(matches!(query(1.5,0),Err(GroundRouteFailure::Planning(PlanningFailure::BudgetExhausted(_)))));
    assert!(matches!(query(0.7,65536),Err(GroundRouteFailure::Planning(PlanningFailure::SearchDomainLimited(_)))));
    let route=query(1.5,65536).expect("fixed wall must be routed around by the same bounded A*");
    eprintln!("C1A_WALL expanded={} points={:?}",route.stats.expanded,route.points);
    assert!(route.stats.expanded>0);
    assert!(route.points.iter().any(|p|p.xy[1].abs()>0.8));
    assert!(route.points.iter().all(|p|p.ground_z.is_none()));
    assert!(trace(&route).iter().all(|s|s.3==2&&s.2.ground_z.is_none()));
    assert_eq!(route.height.resolved_ground_z,None);
    assert!(source.body_query().validate_unanchored_motion(actual,([0.55,0.05,0.5],actual.1),&boundary,&mut None).is_err());
    for edge in route.guide_body_poses.windows(2) {
        source.body_query().validate_unanchored_motion(edge[0],edge[1],&boundary,&mut None).unwrap();
    }
    let high=planner.ground_route_with_limits_and_penalties(&source,([0.05,0.05,1.5],actual.1),start,goal,0.0,
        &boundary,None,Some(1.5),SearchLimits {max_states:0},&[]).unwrap();
    assert_eq!(high.stats.expanded,0);assert!(high.points.iter().all(|p|p.ground_z.is_none()));
}

#[test]
fn c1a_04_explicit_target_layer_matches_conflicts_or_remains_unresolved() {
    let (source,h)=known_source();let xy=center([0,0]);let end=center([8,0]);
    let actual=([xy[0],xy[1],h+0.4],[0.,0.,0.,1.]);let start=GroundRouteStart {xy,ground_z:Some(h)};
    let matched=plan(&source,actual,start,GroundRouteGoal {xy:end,requested_ground_z:Some(h)}).unwrap();
    assert_eq!(matched.planned_layer,GroundLayerResult::Matched);assert_eq!(matched.height.resolved_ground_z,Some(h));
    let error=plan(&source,actual,start,GroundRouteGoal {xy:end,requested_ground_z:Some(h+3.0)}).unwrap_err();
    let GroundRouteFailure::KnownLayerConflict(intent)=error else {panic!("explicit conflict became physical NoPath")};
    assert_eq!(intent.requested_ground_z,Some(h+3.0));assert_eq!(intent.resolved_ground_z,None);
    let unresolved=plan(&source,actual,start,GroundRouteGoal {xy:center([5,0]),requested_ground_z:Some(0.0)}).unwrap();
    assert_eq!(unresolved.height.requested_ground_z,Some(0.0));assert_eq!(unresolved.height.resolved_ground_z,None);
    assert_eq!(unresolved.planned_layer,GroundLayerResult::Unresolved);
    // Fixed positive-XY center exit: the exact target column has no source.
    let center=crate::PctPlanner::default().ground_route_with_limits_and_penalties(&source,actual,start,
        GroundRouteGoal {xy:super::center([1,0]),requested_ground_z:Some(h+3.0)},1.01*q(),&[],None,None,
        SearchLimits::default(),&[]).unwrap();
    assert_eq!(center.points.last().unwrap().xy,xy);
    assert_eq!(center.height.requested_ground_z,Some(h+3.0));
    assert_eq!(center.height.resolved_ground_z,Some(h));
    assert_eq!(center.planned_layer,GroundLayerResult::Unresolved);
}

#[test]
fn c1a_05_known_gap_carry_is_independent_of_actual_bias_and_does_not_create_ground() {
    let (source,h)=known_source();let xy=center([0,0]);let end=center([8,0]);
    let start=GroundRouteStart {xy,ground_z:Some(h)};let goal=GroundRouteGoal {xy:end,requested_ground_z:None};
    let q=[(0.1f64/2.0).sin(),0.,0.,(0.1f64/2.0).cos()];
    let a=plan(&source,([xy[0],xy[1],h+0.4],q),start,goal).unwrap();
    let b=plan(&source,([xy[0],xy[1],h+0.46],q),start,goal).unwrap();
    assert_eq!(a.points,b.points);assert_eq!(trace(&a),trace(&b));
    let samples=trace(&a);assert_eq!(samples.first().unwrap().3,0);assert_eq!(samples.last().unwrap().3,0);
    assert!(samples.iter().any(|s|s.3==1));assert!(samples.iter().all(|s|s.2.ground_z==Some(h)));
    assert!(a.guide_body_poses.iter().all(|p|p.1==q&&(p.0[2]-(h+0.4)).abs()<1e-12));
    assert!(b.guide_body_poses.iter().all(|p|p.1==q&&(p.0[2]-(h+0.46)).abs()<1e-12));
    assert_eq!(source.observations().unwrap().ground_records([4,0]).count(),0);
}

#[test]
fn c1a_06_connections_keep_original_steps_spans_and_source_changes() {
    let (source,h)=known_source();let xy=center([0,0]);let goal=center([8,0]);
    let route=plan(&source,([xy[0],xy[1],h+0.4],[0.,0.,0.,1.]),GroundRouteStart {xy,ground_z:Some(h)},
        GroundRouteGoal {xy:goal,requested_ground_z:None}).unwrap();
    let connections:Vec<_>=route.reference.connections().collect();assert!(!connections.is_empty());
    for (index,c) in connections.iter().enumerate() {
        assert!(c.steps>=1);assert!(c.point_span[0]<=c.point_span[1]&&c.point_span[1]<route.points.len());
        assert!(route.reference.sample(index,c.steps+1).is_none());
        for step in 0..=c.steps {assert!(route.reference.sample(index,step).unwrap().point.ground_z.is_some());}
    }
    assert_eq!(route.points.first().unwrap().xy,xy);assert_eq!(route.points.last().unwrap().xy,goal);
    // Existing B1 exact controls separately exercise the real center/precise,
    // shortcut and folded-tail branches in this very same reducer/evaluator.
}

#[test]
fn c1a_07_cancellation_budget_and_captured_actual_coordinate_penalties() {
    let source=empty_source();let planner=crate::PctPlanner::default();
    let actual=([0.05,0.05,0.5],[0.,0.,0.,1.]);let start=GroundRouteStart {xy:[0.05,0.05],ground_z:None};
    let goal=GroundRouteGoal {xy:[0.25,0.05],requested_ground_z:None};let cancelled=AtomicBool::new(true);
    let bad=([f64::NAN,0.0,0.0],[0.,0.,0.,1.]);
    let error=planner.ground_route_with_limits_and_penalties(&source,bad,start,goal,0.0,&[],Some(&cancelled),None,
        SearchLimits {max_states:0},&[]).unwrap_err();
    assert!(matches!(error,GroundRouteFailure::Planning(PlanningFailure::Other(ref text)) if text=="层析规划已取消"));
    let captured=planner.local_costs();planner.report_local_blockage([0.15,0.05,0.5],0.1);
    let query=|penalties:&[crate::RoutePenalty]|planner.ground_route_with_limits_and_penalties(&source,actual,start,goal,
        0.0,&[],None,None,SearchLimits {max_states:0},penalties);
    assert_eq!(query(&captured).unwrap().stats.expanded,0);
    let current=planner.local_costs();assert!(matches!(query(&current),Err(GroundRouteFailure::Planning(PlanningFailure::BudgetExhausted(_)))));
    planner.clear_local_costs();assert!(matches!(query(&current),Err(GroundRouteFailure::Planning(PlanningFailure::BudgetExhausted(_)))));
}

#[test]
fn c1a_08_trace_retains_original_owner_after_planner_geometry_replacement() {
    let (source,h)=known_source();let planner=crate::PctPlanner::default();let xy=center([0,0]);
    let route=planner.ground_route_with_limits_and_penalties(&source,([xy[0],xy[1],h+0.4],[0.,0.,0.,1.]),
        GroundRouteStart {xy,ground_z:Some(h)},GroundRouteGoal {xy,requested_ground_z:Some(h)},0.0,&[],None,None,
        SearchLimits::default(),&[]).unwrap();
    let before=route.reference.sample(0,0).unwrap();let old=before.point.ground_z;
    let changed=scene(vec![([0,0],native(vec![thin([0,0],2)]))],[],config(0.3),Some(model())).source;
    let new_map=planner.map(&changed).unwrap();assert!(new_map.cell_at(point([0,0],thin([0,0],2).height())).is_some());
    assert!(route.reference.matches_source(&source));assert!(!route.reference.matches_source(&changed));
    let after=route.reference.sample(0,0).unwrap();assert_eq!(after.point.ground_z,old);
    let PlannedGroundAssociation::Known {source:Some(record)}=after.association else {panic!("old owner retired prematurely")};
    assert_eq!(record.key(),GroundKey::HeightfieldV1([0,0,0]));
    assert_eq!(record.members().collect::<Vec<_>>(),vec![0]);
}

#[path="ground_physical_dominance_tests.rs"]
mod physical_dominance_tests;

fn c1c1_signature(sample:nuwax_nav_mls_planner::surface::NavigationGroundSample<'_>)
    ->(u8,Option<u64>,Option<GroundKey>) {
    use nuwax_nav_mls_planner::surface::NavigationGroundSample as S;
    match sample {
        S::KnownNative{record,reference_xy}=>(0,record.height_at(reference_xy).map(f64::to_bits),Some(record.key())),
        S::KnownLegacy{height,..}=>(0,Some(height.to_bits()),None),
        S::Carried{height}=>(1,Some(height.to_bits()),None),S::Unanchored=>(2,None,None),
    }
}

#[test]
fn c1c1_recorded_navigation_adapter_keeps_discrete_addresses_and_owner() {
    let (source,h)=known_source();let from_xy=center([0,0]);let to_xy=center([8,0]);
    let actual=([from_xy[0],from_xy[1],h+0.4],[0.,0.,0.,1.]);
    let route=plan(&source,actual,GroundRouteStart{xy:from_xy,ground_z:Some(h)},
        GroundRouteGoal{xy:to_xy,requested_ground_z:None}).unwrap();
    let start=route.reference.position(0,0,0.0).unwrap().walk_state();let before=start.semantic_key();
    let edge=start.walk_xy(to_xy,&[],None).unwrap();let steps=edge.navigation_steps();
    let rows:Vec<_>=(0..=steps).map(|i|c1c1_signature(edge.navigation_at(i as f64/steps as f64))).collect();
    assert!(rows.iter().any(|r|r.0==1));assert_eq!(rows[0].0,0);assert_eq!(rows[steps].0,0);
    for i in 1..=steps {
        let t=i as f64/steps as f64;
        assert_eq!(c1c1_signature(edge.navigation_at(t.next_down())),rows[i-1]);
        assert_eq!(c1c1_signature(edge.navigation_at(t)),rows[i]);
        if i<steps {assert_eq!(c1c1_signature(edge.navigation_at(t.next_up())),rows[i]);}
    }
    edge.with_body_navigation(|owner,navigation| {
        assert!(owner.same_ground_source(&source));let state=owner.start_ground_state(actual).unwrap();
        let next=owner.project_motion_with_navigation(actual,&state,to_xy,0.0,navigation,&mut None).unwrap();
        assert_eq!(next.pose().0[2],actual.0[2]);owner.validate_projected_motion(actual,&next,&[],&mut None).unwrap();
    }).unwrap();
    assert_eq!(start.semantic_key(),before);
    let candidate=route.reference.freeze_candidate(vec![edge]).unwrap();
    for (i,row) in rows.iter().enumerate() {
        let sample=candidate.sample(0,i).unwrap();let (state,key)=match sample.association {
            PlannedGroundAssociation::Known{source}=>(0,source.map(|r|r.key())),
            PlannedGroundAssociation::Carried=>(1,None),PlannedGroundAssociation::Unanchored=>(2,None),
        };
        assert_eq!(*row,(state,sample.point.ground_z.map(f64::to_bits),key));
    }
    assert!(crate::ground_route::source_less_navigation_adapter_rejected());
    eprintln!("C1C1_RECORDED steps={steps} samples={} end={:?}",rows.len(),candidate.sample(0,steps));
}

#[test]
fn c1c1_recorded_legacy_plane_keeps_finite_attitude_pass_order() {
    use nuwax_nav_mls_planner::surface::SourceGroundRecord;
    // Same admitted LegacyPlane as sloped_legacy_fixture, plus one real flat
    // neighbor. Their direction change moves the calibrated support back across
    // the column boundary in the existing second (final) H3 pass.
    let original=sloped_legacy_fixture();let norm=1.0625_f64.sqrt();
    let columns=[([0,0],[-0.25/norm,0.,1./norm]),([1,0],[0.,0.,1.])].into_iter().map(|(xy,normal)| {
        let key=[xy[0],xy[1],0];let origin=point(xy,0.5*q());
        (xy,ColumnGroundRecords {classified_empty:false,heightfields:vec![],legacy:vec![(block_key(key),LayerObservation {
            members:vec![local_key(key)],plane_origin:origin,normal,samples:vec![LayerSample {source:Default::default(),
                point:origin,source_sequence:1,source_timestamp_ns:20,point_index:0}]} )]})
    }).collect();
    let source=scene(columns,[],original.source.config().clone(),original.source.body_envelope().cloned()).source;
    let angle=-0.25_f64.atan();let quaternion=[0.,(angle*0.5).sin(),0.,(angle*0.5).cos()];
    let xy=[-0.05,0.5*q()];let end=[0.5*q(),0.5*q()];
    let offset=source.support_position_from_body([0.;3],quaternion,0.3).unwrap();
    let support=[xy[0]+offset[0],xy[1]+offset[1]];
    let first=source.observations().unwrap().ground_records([0,0]).next().unwrap();
    assert!(matches!(first,SourceGroundRecord::LegacyPlane(_)));
    assert!((0.0..q()).contains(&support[0]));
    let h=first.height_at(support).unwrap();let actual=([xy[0],xy[1],h-offset[2]+0.02],quaternion);
    let route=plan(&source,actual,GroundRouteStart{xy,ground_z:Some(0.5*q())},
        GroundRouteGoal{xy:end,requested_ground_z:None}).unwrap();
    let start=route.reference.position(0,0,0.).unwrap().walk_state();
    let cancelled=AtomicBool::new(true);assert!(start.walk_xy(end,&[],Some(&cancelled)).is_err());
    let edge=start.walk_xy(end,&[],None).unwrap();let steps=edge.navigation_steps();
    let rows:Vec<_>=(0..=steps).map(|i|c1c1_signature(edge.navigation_at(i as f64/steps as f64))).collect();
    let changes=edge.navigation_changes();
    for i in 1..=steps {if rows[i]!=rows[i-1] {assert!(changes.contains(&(i as f64/steps as f64)));}}
    assert!(rows.windows(2).any(|pair|pair[0].0==0&&pair[1].0==0&&pair[0].2==pair[1].2&&pair[0].1!=pair[1].1),
        "fixed recorded plane must actually change h without changing id");
    edge.with_body_navigation(|owner,navigation| {
        let physical=owner.start_ground_state(actual).unwrap();
        let next=owner.project_motion_with_navigation(actual,&physical,end,0.0,navigation,&mut None).unwrap();
        let final_support=source.support_position_from_body(next.pose().0,next.pose().1,0.3).unwrap();
        let column=[final_support[0],final_support[1]].map(|v|(v/q()).floor() as i32);
        let record=source.observations().unwrap().ground_records(column).next().unwrap();
        assert_eq!(column,[0,0]);assert_eq!(next.ground().ground_key(),Some(record.key()));
        assert_eq!(next.ground().height(),record.height_at([final_support[0],final_support[1]]).unwrap());
        assert_ne!(next.pose().1,actual.1,"fixed direction change must enter the second pass");
        assert!(next.pose().1[1].abs()<1e-12,"finite transport uses the first-pass flat direction, no convergence iteration");
        owner.validate_projected_motion(actual,&next,&[],&mut None).unwrap();
        eprintln!("C1C1_LEGACY from={actual:?} support={support:?} next={:?} final_support={final_support:?} key={:?}",next.pose(),record.key());
    }).unwrap();
}
