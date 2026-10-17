// PROPOSAL: new explicit world contracts. Existing failed selectors are retained.
// No changed production policy, occupied voxel, speed, deadline, or collision epsilon.
use super::*;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(super) enum World {
    LegacyObservedSupport,
    ExplicitVirtualMaskDomain,
    UnobservedFlatContinuation,
}
fn declared_domain_mask(map:&schema::NavigationMap3D)->schema::OccupancyGrid {
    // This is an explicit permitted domain. It is not inferred from missing floor.
    // Existing production mask semantics reject its exterior and check whole-body cells.
    let mut mask=virtual_mask(map,false);
    mask.info.resolution=RESOLUTION;mask.info.width=100;mask.info.height=60;
    mask.info.origin.position=[0.0;3];mask.data=vec![0;6000];
    mask
}
impl World {
    pub(super) fn declare_on_wire(self,map:&mut schema::NavigationMap3D) {
        if self==Self::ExplicitVirtualMaskDomain {
            map.virtual_obstacles=Some(declared_domain_mask(map));
        }
    }
    pub(super) fn wire_matches(self,map:&schema::NavigationMap3D)->bool {
        if self==Self::ExplicitVirtualMaskDomain {
            map.virtual_obstacles.as_ref()==Some(&declared_domain_mask(map))
        } else {map.virtual_obstacles.is_none()}
    }
    pub(super) fn audit_built(self,map:&SurfaceSnapshot)->Result<(),String> {
        if self==Self::ExplicitVirtualMaskDomain && !map.matches_virtual_obstacles(
            [0.0;2],f64::from(RESOLUTION),100,60,(0..6000).map(|_|false)) {
            return Err("consumed snapshot does not retain the exact declared virtual mask".into());
        }
        Ok(())
    }
    pub(super) fn check_segment(self,segment:&OracleSegment,scene:Scene)->Result<usize,String> {
        match self {
            Self::LegacyObservedSupport|Self::ExplicitVirtualMaskDomain=>super::check_segment(segment,scene),
            Self::UnobservedFlatContinuation=>check_flat_continuation_segment(segment,scene),
        }
    }
}
fn check_collision_sample(pose:OraclePose,time_s:f64,geometry:OracleGeometry,
    obstacle_active:bool)->Result<OracleMargins,String> {
    let margins=oracle_margins(pose,geometry)?;
    if obstacle_active && margins.obstacle < -ORACLE_EPS {
        return Err(format!("occupied obstacle collision at t={time_s:.9}s (SAT gap={:.12}m)",margins.obstacle));
    }
    if obstacle_active && margins.obstacle <= ORACLE_EPS {
        return Err(format!("Unknown: near-zero obstacle margin at t={time_s:.9}s (SAT gap={:.12}m)",margins.obstacle));
    }
    Ok(margins)
}

// Same integration/endpoints/SAT/swept bound/budgets as the original oracle.
// Support over the observed rectangle is NOT claimed by this separate positive case.
fn check_flat_continuation_segment(segment: &OracleSegment, scene: Scene) -> Result<usize, String> {
    if !oracle_finite_pose(segment.start)
        || !oracle_finite_pose(segment.end)
        || !segment.twist.linear.x.is_finite()
        || !segment.twist.linear.y.is_finite()
        || !segment.twist.angular_z.is_finite()
        || !segment.duration_s.is_finite()
        || segment.duration_s < 0.0
    {
        return Err("invalid/non-finite actual trajectory segment".into());
    }
    if (segment.start.position[2] - ORACLE_CENTER_Z_M).abs() > ORACLE_EPS
        || (segment.end.position[2] - ORACLE_CENTER_Z_M).abs() > ORACLE_EPS
    {
        return Err("actual trajectory violates fixed z=0.5m SE(2) fixture".into());
    }
    if segment.end.yaw.abs() > 1.0e6 {
        return Err("Unknown: recorded endpoint angular numeric range exceeded".into());
    }
    let geometry = oracle_geometry(&scene)?;
    // Reintegrate the executed twist independently; an endpoint chord is not
    // the robot's path. Wrapped yaw agreement permits a recorded +/-pi wrap.
    let expected_end = oracle_integrate(segment.start, segment.twist, segment.duration_s)?;
    let yaw_delta = expected_end.yaw - segment.end.yaw;
    if !yaw_delta.is_finite() {
        return Err("invalid/non-finite endpoint angular difference".into());
    }
    let yaw_error = yaw_delta.sin().atan2(yaw_delta.cos()).abs();
    let position_error = expected_end
        .position
        .into_iter()
        .zip(segment.end.position)
        .map(|(expected, actual)| (expected - actual).abs())
        .fold(0.0, f64::max);
    if position_error > ORACLE_EPS || yaw_error > ORACLE_EPS {
        return Err(format!(
            "actual endpoint disagrees with independent SE(2) integration: position={position_error:.12}m, yaw={yaw_error:.12}rad"
        ));
    }
    // Check the actual recorded endpoints, not only the independently
    // reconstructed endpoint (which is allowed a small numeric tolerance).
    check_collision_sample(segment.start, 0.0, geometry, segment.obstacle_active)?;
    check_collision_sample(
        segment.end,
        segment.duration_s,
        geometry,
        segment.obstacle_active,
    )?;

    // For every point of the square, |p_dot| <= |v| + rho*|w|, where
    // rho=sqrt(2)*0.15. Thus every point moves at most h over half an interval.
    // A fixed midpoint separating axis with gap > h+eps remains separating;
    // The declared world has flat support beyond observed samples; certify collision only.
    let point_speed = segment.twist.linear.x.hypot(segment.twist.linear.y)
        + std::f64::consts::SQRT_2 * ORACLE_HALF_SIDE_M * segment.twist.angular_z.abs();
    if !point_speed.is_finite() {
        return Err("Unknown: non-finite point-speed bound".into());
    }
    let root = OracleInterval {
        lo_s: 0.0,
        hi_s: segment.duration_s,
        depth: 0,
    };
    // Depth-first traversal needs at most MAX_DEPTH+1 pending intervals.
    // No recursion, unbounded allocation, or controller feedback is involved.
    let mut stack = [root; ORACLE_MAX_DEPTH + 1];
    let mut stack_len = 1usize;
    let mut nodes = 0usize;
    while stack_len != 0 {
        if nodes == ORACLE_MAX_NODES {
            return Err(format!("Unknown: interval-node budget exhausted ({nodes})"));
        }
        stack_len -= 1;
        let interval = stack[stack_len];
        nodes += 1;
        let width_s = interval.hi_s - interval.lo_s;
        let half_width_s = 0.5 * width_s;
        let midpoint_s = interval.lo_s + half_width_s;
        let mut midpoint = None;
        for time_s in [interval.lo_s, midpoint_s, interval.hi_s] {
            let pose = oracle_integrate(segment.start, segment.twist, time_s)?;
            let margins = check_collision_sample(pose, time_s, geometry, segment.obstacle_active)?;
            if time_s == midpoint_s {
                midpoint = Some(margins);
            }
        }
        let midpoint = midpoint.ok_or_else(|| "Unknown: missing midpoint evidence".to_owned())?;
        let h = point_speed * half_width_s;
        if !h.is_finite() || !(h + ORACLE_EPS).is_finite() {
            return Err("Unknown: non-finite swept displacement bound".into());
        }
        if !segment.obstacle_active || midpoint.obstacle > h + ORACLE_EPS
        {
            continue;
        }
        if width_s <= ORACLE_MIN_INTERVAL_S {
            return Err(format!(
                "Unknown: unresolved interval at minimum width ({width_s:.12}s, nodes={nodes})"
            ));
        }
        if interval.depth >= ORACLE_MAX_DEPTH {
            return Err(format!("Unknown: subdivision depth exhausted (nodes={nodes})"));
        }
        if midpoint_s <= interval.lo_s || midpoint_s >= interval.hi_s {
            return Err("Unknown: subdivision cannot make numeric progress".into());
        }
        if stack_len + 2 > stack.len() {
            return Err("Unknown: fixed interval stack exhausted".into());
        }
        // Push right first so chronological left intervals are examined first.
        stack[stack_len] = OracleInterval {
            lo_s: midpoint_s,
            hi_s: interval.hi_s,
            depth: interval.depth + 1,
        };
        stack[stack_len + 1] = OracleInterval {
            lo_s: interval.lo_s,
            hi_s: midpoint_s,
            depth: interval.depth + 1,
        };
        stack_len += 2;
    }
    Ok(nodes)
}

#[test]
fn separated_world_oracles_keep_original_failure_and_wall_collision() {
    let outside=OraclePose {position:[2.2,-0.3,0.5],yaw:0.0};
    let segment=OracleSegment {start:outside,end:outside,twist:Twist2::ZERO,
        duration_s:1.0,obstacle_active:true};
    assert!(super::check_segment(&segment,Scene::Wall).unwrap_err().contains("unsupported footprint"));
    check_flat_continuation_segment(&segment,Scene::Wall).unwrap();
    let occupied=OraclePose {position:[2.2,1.5,0.5],yaw:0.0};
    let collision=OracleSegment {start:occupied,end:occupied,..segment};
    for world in [World::LegacyObservedSupport,World::ExplicitVirtualMaskDomain,
        World::UnobservedFlatContinuation] {
        assert!(world.check_segment(&collision,Scene::Wall).unwrap_err().contains("occupied obstacle collision"));
    }
    let contact=OraclePose {position:[2.2,-0.15,0.5],yaw:0.0};
    assert!(check_flat_continuation_segment(&OracleSegment{start:contact,end:contact,..segment},Scene::Wall).is_err());
}
#[test]
#[ignore="explicit codec/cache preflight; no live navigation or source-time fabrication"]
fn explicit_mask_wire_cache_full_footprint_preflight() {
    let repository=Repository::temporary().unwrap();
    let mut writer=Writer::new(repository.clone(),RESOLUTION,4*1024*1024).unwrap();
    let stamp=current_unix_timestamp_ns().unwrap();
    writer.observe(ground_keys(),stamp as u64).unwrap();
    let root=writer.snapshot(Domain {session:"wall-world-mask-preflight".into(),generation:1,
        frame_generation:1,producer_generation:17},1,stamp as u64).unwrap();
    let mut map=map_from_fine(&root,&repository);
    let mut cache=SurfaceGeometryCache::configured(&calibrated_config(true));
    let plain=cache.build(&map).unwrap();
    let outside=body_pose(1.0,-0.3,0.0);
    plain.validate_body_pose(outside.0,outside.1).unwrap();
    World::ExplicitVirtualMaskDomain.declare_on_wire(&mut map);
    let bytes=schema::encode_navigation_map_3d_body(&map).unwrap();
    let decoded=schema::decode_navigation_map_3d_body(&bytes).unwrap();
    assert_eq!(decoded,map);assert!(World::ExplicitVirtualMaskDomain.wire_matches(&decoded));
    let built=cache.build(&decoded).unwrap();
    World::ExplicitVirtualMaskDomain.audit_built(&built).unwrap();
    audit_observations(&built,Scene::Wall,Stage::Clear,stamp).unwrap();
    let inside=body_pose(1.0,1.5,0.0);built.validate_body_pose(inside.0,inside.1).unwrap();
    assert!(built.validate_body_pose(outside.0,outside.1).is_err());
    // Center remains inside the mask; the footprint crosses y=0.
    let footprint_outside=body_pose(1.0,0.1,0.0);
    plain.validate_body_pose(footprint_outside.0,footprint_outside.1).unwrap();
    assert!(built.validate_body_pose(footprint_outside.0,footprint_outside.1).is_err());
    let goal=goal_for_with_options(&decoded,1,[3.5,1.5,0.5],
        nuwax_nav_types_messages::GoalOptions{allow_reverse:false,position_tolerance_m:0.1,yaw_tolerance_rad:0.15});
    assert_eq!(goal.owner_kind,schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV);
    assert!(goal.boundary_id.is_empty()&&goal.boundary_frame_id.is_empty()&&goal.boundary_points.is_empty());
    schema::validate_goal_envelope(&goal).unwrap();
    assert!(navigation_boundary_from_goal(&goal).unwrap().is_none());
    eprintln!("WALL_WORLD_PREFLIGHT mask=wire_roundtrip_and_actual_cache_verified full_footprint=true public_nav_unchanged=true");
}
async fn qualify_world(world:World) {
    let mut harness=OccupancyHarness::start_in_world(true,Scene::Wall,world).await;
    let result=harness.exercise().await;let status=harness.scheduler.status();
    let cleanup=harness.shutdown().await;harness.plant.lock().unwrap().sample();
    let audit=harness.audit_after_cleanup();harness.dump_evidence();
    eprintln!("WORLD_FINAL world={world:?} result={result:?} status={status:?} cleanup={cleanup:?} audit={audit:?}");
    assert!(result.is_ok()&&cleanup.is_ok()&&audit.is_ok(),"world={world:?}; result={result:?}; cleanup={cleanup:?}; audit={audit:?}");
}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="new declared-mask-domain Wall negative; original failed selector retained"]
async fn explicit_mask_wall_preserves_active_goal() {qualify_world(World::ExplicitVirtualMaskDomain).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="new finite-Wall positive; true world is flat beyond observed floor; no mask"]
async fn finite_wall_unknown_flat_continuation_arrives() {qualify_world(World::UnobservedFlatContinuation).await;}


// Diagnostic wrappers preserve the original world, source, runtime and oracle.
// Only fixed-size scalar observation is added, and only after explicit activation.
async fn trace_world_input_timing(world:World) {
    let trace=nuwax_nav_runtime::qualification_timing::TraceSession::start(65536);
    let mut harness=OccupancyHarness::start_with_world_timing(true,Scene::Wall,None,None,world,Some(trace.sink())).await;
    let result=harness.exercise().await;let status=harness.scheduler.status();
    let cleanup=harness.shutdown().await;harness.plant.lock().unwrap().sample();
    let audit=harness.audit_after_cleanup();harness.dump_evidence();
    let mut report=trace.finish();
    if cleanup.is_err() {report.incomplete=true;}
    eprintln!("ACTOR_TIMING world={world:?} report={report:?}");
    eprintln!("WORLD_FINAL world={world:?} result={result:?} status={status:?} cleanup={cleanup:?} audit={audit:?}");
    assert!(result.is_ok()&&cleanup.is_ok()&&audit.is_ok(),"world={world:?}; result={result:?}; cleanup={cleanup:?}; audit={audit:?}");
}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="one opt-in bounded real-actor trace; same explicit-mask Wall assertions"]
async fn trace_explicit_mask_wall_input_timing() {trace_world_input_timing(World::ExplicitVirtualMaskDomain).await;}
#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="one opt-in bounded real-actor trace; same unmasked flat-continuation assertions"]
async fn trace_finite_wall_flat_input_timing() {trace_world_input_timing(World::UnobservedFlatContinuation).await;}
