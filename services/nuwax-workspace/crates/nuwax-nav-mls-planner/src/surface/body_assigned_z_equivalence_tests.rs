// Qualified against the original dense-Z kernel; optimization remains separate.
// Baseline HEAD: 332237e128a918376429a935cbe12c4e949a0538.
// Included in surface::body::tests, beside recent_query_tests.
// The complete frozen production function below differs ONLY in its name.
use super::*;
use std::collections::{BTreeMap, BTreeSet};

impl SurfaceSnapshot {
    fn check_body_pose_dense_z_oracle(&self,body:[f64;3],quaternion:[f64;4],slot:&mut Option<BodyRejection>)->Result<(),SurfaceError> {
        *slot=None;
        self.validate_complete_source_resolution()?;
        let model = self
            .body_envelope()
            .ok_or_else(|| SurfaceError::InvalidConfig("body envelope is unavailable".into()))?;
        let r=rotation(quaternion)?;
        if r[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        // Ground is useful for contact interpretation, not a second permission to
        // execute geometry checking. Missing samples are not occupied obstacles.
        let centre=match self.locate_support_from_body(body,quaternion,model.body_to_support_z_m) {
            Ok(value)=>Some(value),Err(SurfaceError::MissingSupport)=>None,Err(error)=>return Err(error),
        };
        let vertices: Vec<_> = model
            .footprint_xy_m
            .iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, body, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let polygon_query=PolygonQuery::new(&polygon);
        let prism_query=PrismQuery::new(model,r,body);
        let (min, max) = vertices.iter().fold(
            ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
            |(mut min, mut max), p| {
                for i in 0..3 {
                    min[i] = min[i].min(p[i]);
                    max[i] = max[i].max(p[i]);
                }
                (min, max)
            },
        );
        let size = f64::from(self.config().voxel_size);
        let x0 = (min[0] / size).floor() as i32;
        let x1 = (max[0] / size).floor() as i32;
        let y0 = (min[1] / size).floor() as i32;
        let y1 = (max[1] / size).floor() as i32;
        if (i64::from(x1) - i64::from(x0) + 1) * (i64::from(y1) - i64::from(y0) + 1) > 16_384 {
            return Err(SurfaceError::InvalidConfig(
                "body footprint query exceeds budget".into(),
            ));
        }
        // 查询半径取实际投影足迹到支撑格中心的最远距离；高度和支撑
        // 距离已经包含在旋转后的顶点里，不能再各加一次扩大地面遍历。
        // 向上对齐体素以复用同一格的邻域，并保留两格接触连接余量。
        let radius = ((vertices.iter().map(|p|(p[0]-centre.map_or(body[0],|c|c.surface_xyz[0])).hypot(p[1]-centre.map_or(body[1],|c|c.surface_xyz[1])))
            .fold(0.0,f64::max)+size*2.0)/size).ceil()*size;
        let patch=centre.map_or_else(||Arc::new(AHashMap::new()),|centre|self.ground_patch(centre,radius));
        let (patch,precise_ground)=self.refine_footprint_support(patch,&polygon_query,&prism_query,body,r,([x0,y0],[x1,y1]));
        if let Some(recent)=&self.recent_obstacles {
            let fine=recent.size;
            // All row inputs (pose, patch, source and prism)
            // stay fixed during this query. Reuse only the last coarse column;
            // every recent frame, fine XY and Z still executes in original order.
            let mut selected_column=None;
            let mut selected_rows:Option<Vec<FineGroundHeightRow>>=None;
            for frame in &recent.frames {
                let low=[(min[0]/fine).floor() as i32,(min[1]/fine).floor() as i32];
                let high=[(max[0]/fine).floor() as i32,(max[1]/fine).floor() as i32];
                for (&[x,y],zs) in frame.columns.range(low..=high) {
                    if y<low[1] || y>high[1] {continue;}
                    let first=zs.partition_point(|&z|f64::from(z+1)*fine<min[2]);
                    let last=zs.partition_point(|&z|f64::from(z)*fine<=max[2]);
                    let zs=&zs[first..last];if zs.is_empty() {continue;}
                    {
                            let coarse=f64::from(self.config().voxel_size);
                            let cx=(((f64::from(x)+0.5)*fine)/coarse).floor() as i32;
                            let cy=(((f64::from(y)+0.5)*fine)/coarse).floor() as i32;
                            let ground=patch.get(&(cx,cy)).copied();
                            if selected_column!=Some((cx,cy)) {
                                selected_rows=self.fine_ground_values::<FineGroundHeightRow>(
                                    ground,&polygon_query,&prism_query,body,r[2],cx,cy);
                                selected_column=Some((cx,cy));
                            }
                            let height=if let Some(rows)=&selected_rows {rows.iter().find(|row|row.xy==[x,y]).and_then(|row|row.height)}
                                else {ground.map(|g|self.ground_cell_position(g)[2])};
                            for &z in zs {
                                if let Some(points)=frame.surface_points.get(&[x,y,z]) {
                                    if !points.iter().any(|&point|!self.recent_surface_contact(point,height,body,r)
                                        &&prism_query.intersects(point,point)) {continue;}
                                }
                                let low=[x,y,z].map(|v|f64::from(v)*fine);
                                let high=[x,y,z].map(|v|f64::from(v+1)*fine);
                                if high[2]>=min[2] && low[2]<=max[2] && prism_query.intersects(low,high) {
                                    return Err(reject(slot,BodyRejection::Occupancy{cell:[x,y,z],
                                        source:OccupancySource::Recent,z_bounds:[low[2],high[2]],ground_height:height}));
                                }
                            }
                        }
                }
            }
        }
        self.check_fine_column_fringe(body,r,&polygon_query,&prism_query,&patch,min,max,([x0,y0],[x1,y1]),slot)?;
        let uncertainty = f64::from(self.source_voxel_size_m()) + model.support_uncertainty_m;
        let mut ground_points = Vec::new();
        for x in x0..=x1 {
            for y in y0..=y1 {
                if !polygon_query.intersects(x,y,size) {
                    continue;
                }
                let Some(&ground)=patch.get(&(x,y)) else {
                    // A sparse/unknown footprint column does not veto the pose.
                    // It still receives the exact same actual-volume collision pass.
                    if self.data.virtual_obstacles.as_ref().is_some_and(|mask|
                        !mask.permits_cell((x,y,0),size,0.0)) {return Err(SurfaceError::VirtualObstacle);}
                    self.check_unassigned_body_column(body,r,&polygon_query,&prism_query,x,y,slot)?;
                    continue;
                };
                if self
                    .data
                    .virtual_obstacles
                    .as_ref()
                    .is_some_and(|mask| !mask.permits_cell(ground, size, 0.0))
                {
                    return Err(SurfaceError::VirtualObstacle);
                }
                let px = (x as f64 + 0.5) * size;
                let py = (y as f64 + 0.5) * size;
                let contact = model.max_step_up_m.max(model.max_step_down_m) + uncertainty;
                let plane=polygon_query.contact_plane(body,r[2],x,y,size,self.ground_cell_position(ground)[2],
                    (-model.body_to_support_z_m/r[2][2]+contact+1e-6).min(uncertainty));
                let support_plane = plane - model.body_to_support_z_m / r[2][2];
                let selected=self.fine_ground_columns(Some(ground),&polygon_query,&prism_query,body,r[2],x,y);
                let ground_height = if selected.is_none() && precise_ground.contains_key(&(x,y)) {precise_ground[&(x,y)]} else {
                    self.footprint_ground_height(ground,&polygon_query,&prism_query,support_plane,contact,selected.as_deref())?
                };
                // Static height/ceiling feasibility belongs to PCT. Preserve the
                // original occupied voxels for actual-pose collision below instead
                // of repeating a coarse max-height veto and a fine rescue pass.
                ground_points.push([px, py, ground_height]);
                // This only decides whether an occupied ground band is contact.
                // A failed contact interpretation is rejected only if the actual
                // remaining occupied volume intersects the prism below.
                let coarse_contact=height_rejection(plane+model.z_bounds_m[0]/r[2][2],
                    support_plane,plane,contact,ground_height,uncertainty).is_none();
                if let Some(zs) = self.data.occupied_columns.get(&(x, y)) {
                    for &z in zs {
                        let bounds = self.data.height_bounds.get(&(x, y, z)).copied().unwrap_or([
                            z as f32 * self.config().voxel_size,
                            (z as f32 + 1.0) * self.config().voxel_size,
                        ]);
                        if (selected.is_none() && coarse_contact && f64::from(bounds[1]) <= ground_height + 1e-6)
                            || f64::from(bounds[0]) > max[2]
                            || f64::from(bounds[1]) < min[2]
                        {
                            continue;
                        }
                        let mut fine_hit=None;
                        let coarse_intersects = || prism_query.intersects(
                            [x as f64 * size, y as f64 * size, f64::from(bounds[0])],
                            [(x + 1) as f64 * size, (y + 1) as f64 * size, f64::from(bounds[1])],
                        );
                        let intersects = if let Some(observations) = &self.observations {
                            let fine = f64::from(observations.size_m);
                            let mut intersects = false;
                            let mut found = false;
                            let mut missing_slice_hit=false;
                            for ix in (f64::from(x)*size/fine).round() as i32..(f64::from(x+1)*size/fine).round() as i32 {
                                for iy in (f64::from(y)*size/fine).round() as i32..(f64::from(y+1)*size/fine).round() as i32 {
                                    let column=selected.as_ref().and_then(|columns|columns.iter().find(|c|c.xy==[ix,iy]));
                                    let fine_ground=if selected.is_some() {column.and_then(|c|c.height)} else {Some(ground_height)};
                                    // A partial fine source cannot erase the unknown
                                    // XY slice of an already occupied coarse volume.
                                    if selected.is_some() && column.is_none_or(|c|c.height.is_none())
                                        && !observations.is_complete_geometry_source()
                                        && (!coarse_contact || f64::from(bounds[1])>ground_height+1e-6)
                                        && prism_query.intersects([f64::from(ix)*fine,f64::from(iy)*fine,f64::from(bounds[0])],
                                            [f64::from(ix+1)*fine,f64::from(iy+1)*fine,f64::from(bounds[1])]) {
                                        missing_slice_hit=true;
                                    }
                                    for iz in (f64::from(bounds[0])/fine+1e-5).floor() as i32..(f64::from(bounds[1])/fine-1e-5).ceil() as i32 {
                                        // Both contact-run forms contain only real supported members
                                        // of this immutable observation owner. Preserve `found` so
                                        // an exempt fine contact cannot trigger coarse fallback.
                                        if column.is_some_and(|c|c.permits_contact(iz)) {
                                            found=true;
                                            continue;
                                        }
                                        if observations.get([ix,iy,iz]).is_none() { continue; }
                                        found = true;
                                        let contact=selected.is_none() && coarse_contact
                                            && fine_ground.is_some_and(|height|f64::from(iz+1)*fine<=height+1e-6);
                                        if contact {continue;}

                                        let hit=prism_query.intersects(
                                            [f64::from(ix)*fine,f64::from(iy)*fine,f64::from(iz)*fine],
                                            [f64::from(ix+1)*fine,f64::from(iy+1)*fine,f64::from(iz+1)*fine]);
                                        if hit && fine_hit.is_none() {fine_hit=Some(([ix,iy,iz],[f64::from(iz)*fine,f64::from(iz+1)*fine],fine_ground));}
                                        intersects |= hit;
                                    }
                                }
                            }
                            // 缺少该占据格的精细依据时保留粗格障碍，不把缺测当作自由。
                            if found { intersects || missing_slice_hit } else {  (!coarse_contact || f64::from(bounds[1])>ground_height+1e-6) && coarse_intersects() }
                        } else {  coarse_intersects() };
                        if intersects {
                            tracing::debug!(?body, x, y, z, ?bounds, ground_height, "机体与占据单元相交");
                            let (cell,z_bounds,source,ground_height)=match fine_hit {
                                Some((cell,bounds,height))=>(cell,bounds,OccupancySource::Fine,height),
                                None=>([x,y,z],bounds.map(f64::from),OccupancySource::CoarseFallback,Some(ground_height)),
                            };
                            return Err(reject(slot,BodyRejection::Occupancy{cell,source,z_bounds,ground_height}));
                        }
                    }
                }
            }
        }
        if let Some((a, b, _)) = fit_plane(&ground_points, centre.map_or(body,|c|c.support_xyz)) {
            let yaw = r[0][1].atan2(r[0][0]);
            let cross_slope = (-a * yaw.sin() + b * yaw.cos()).abs().atan().to_degrees();
            if cross_slope > model.max_cross_slope_deg + 1e-5 {
                tracing::debug!(?body, cross_slope, limit=model.max_cross_slope_deg, "实际机体支撑横坡超限");
                return Err(SurfaceError::UntraversableSegment);
            }
        }
        // Known terrain contradictions (including a measured drop) remain a
        // safety constraint. Absence of a layer returns Unknown and grants no
        // fabricated observation, but it is not a rejection.
        if self.body_ground_relation(body,quaternion)? == BodyGroundRelation::KnownIncompatible {
            return Err(SurfaceError::UntraversableSegment);
        }
        Ok(())
    }
}

// A small, deliberately sparse upper echo above a real floor. The coarse graph's
// 0.2m clearance keeps the original floor anchor under a 0.35m echo; the actual
// 0.3m body prism at z=0.4 must still reject the echo. This is a semantic test
// fixture, not a change to the original bounded performance fixture or budget.
fn indexed_z_fixture(fine: f32, coarse: f32, observed_mode: usize) -> SurfaceSnapshot {
    let nx=(0.8/fine).round() as i32;
    let ny=(0.4/fine).round() as i32;
    let mut all:BTreeSet<[i32;3]>=(-nx..=nx).flat_map(|x|
        (-ny..=ny).map(move|y|[x,y,-1])).collect();
    let low=(0.35/fine).round() as i32;
    let high=(0.39/fine).round() as i32;
    for x in [0,1] { for y in [0,1] { for z in [low,high] { all.insert([x,y,z]); } } }
    let points:Vec<_>=all.iter().map(|key|(
        (key[0] as f32+0.5)*fine,(key[1] as f32+0.5)*fine,(key[2] as f32+0.5)*fine)).collect();
    let mut blocks:BTreeMap<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>=BTreeMap::new();
    for key in all {
        // mode 1: a genuinely missing XY slice in partial fine input.
        // mode 2: floor-only partial input, so known coarse echoes must survive.
        if (observed_mode==1 && key[0]==0 && key[1]==0)
            || (observed_mode==2 && key[2]>=0) {continue;}
        blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
            .insert(key,SurfaceObservation{observed_ns:1,hits:2});
    }
    let mut observations=SurfaceObservations::new(fine,32).unwrap();
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
    if observed_mode==0 {observations.mark_complete_geometry_source();}
    let config=Config {world_frame:"map".into(),voxel_size:coarse,robot_height:0.2,
        surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
        wall_buffer_m:0.1,wall_buffer_weight:2.0,max_slope_deg:18.0,step_threshold_m:0.0,
        min_step_tread_depth_m:0.2,step_penalty_weight:1.0,goal_tolerance:0.1,viz_publish_hz:0.0};
    let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
        z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.4,max_step_up_m:0.0,max_step_down_m:0.0,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    SurfaceSnapshot::from_points(&points,config,fine).unwrap()
        .with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)))
}


type Pose = ([f64; 3], [f64; 4]);
type RejectionBits = ([i32; 3], OccupancySource, [u64; 2], Option<u64>);
fn rejection_bits(value: Option<BodyRejection>) -> Option<RejectionBits> {
    value.map(|value| match value {
        BodyRejection::Occupancy { cell, source, z_bounds, ground_height } =>
            (cell, source, z_bounds.map(f64::to_bits), ground_height.map(f64::to_bits)),
    })
}
fn witness_bits(value: Option<BodyFailureWitness>) -> Option<([u64; 3], [u64; 4], RejectionBits)> {
    value.map(|value| (value.pose.0.map(f64::to_bits), value.pose.1.map(f64::to_bits),
        rejection_bits(Some(value.rejection)).unwrap()))
}
fn assert_result_bits(actual: &Result<(), SurfaceError>, expected: &Result<(), SurfaceError>) {
    assert_eq!(actual, expected, "complete Result, including error payload");
    if let (Err(SurfaceError::InsufficientResolution { source_m: a, planner_m: b }),
        Err(SurfaceError::InsufficientResolution { source_m: c, planner_m: d })) = (actual, expected) {
        assert_eq!([a.to_bits(), b.to_bits()], [c.to_bits(), d.to_bits()]);
    }
}
fn poison() -> BodyRejection {
    BodyRejection::Occupancy { cell: [999, 998, 997], source: OccupancySource::Recent,
        z_bounds: [-0.0, 1.0], ground_height: Some(-0.0) }
}

// Every paired call executes the actual kernels, never a shared pose-cache hit.
// Planning forks isolate disposable body-query/ground-patch caches. Observation
// legacy_reference_cache remains shared by Arc (surface/observations.rs), so
// these phases do not claim that every source-owned cache is cold/reset.
fn compare_pose(source: &SurfaceSnapshot, pose: Pose) -> (Result<(), SurfaceError>, Option<BodyRejection>) {
    let mut candidate = source.fork_for_planning();
    let mut oracle = source.fork_for_planning();
    assert!(!Arc::ptr_eq(&candidate.body_query_cache, &oracle.body_query_cache));
    assert!(!Arc::ptr_eq(&candidate.data, &oracle.data));
    let mut last = None;
    for phase in ["cold", "warm helpers", "reset all caches"] {
        if phase == "reset all caches" {
            candidate = candidate.fork_for_planning();
            oracle = oracle.fork_for_planning();
        }
        let mut actual_rejection = Some(poison());
        let mut expected_rejection = Some(poison());
        let actual = candidate.check_body_pose(pose.0, pose.1, &mut actual_rejection);
        let expected = oracle.check_body_pose_dense_z_oracle(pose.0, pose.1, &mut expected_rejection);
        assert_result_bits(&actual, &expected);
        assert_eq!(rejection_bits(actual_rejection), rejection_bits(expected_rejection),
            "first rejection bits, pose={pose:?}, phase={phase}");
        assert!(candidate.body_query_cache.lock().unwrap().poses.is_empty());
        assert!(oracle.body_query_cache.lock().unwrap().poses.is_empty());
        last = Some((expected, expected_rejection));
    }
    let (expected, rejection) = last.unwrap();
    let expected_witness = rejection.map(|rejection| BodyFailureWitness { pose, rejection });
    let mut public = source.fork_for_planning();
    for phase in ["cold public", "cached public", "reset public"] {
        if phase == "reset public" { public = public.fork_for_planning(); }
        let mut witness = Some(BodyFailureWitness { pose: ([999.0; 3], [1.0; 4]), rejection: poison() });
        let actual = public.validate_body_pose_with_witness(pose.0, pose.1, &mut witness);
        assert_result_bits(&actual, &expected);
        assert_eq!(witness_bits(witness), witness_bits(expected_witness),
            "public pose, rejection and scalar bits, phase={phase}");
        assert_eq!(public.body_query_cache.lock().unwrap().poses.len(), 1);
    }
    (expected, rejection)
}

// Only the broad matrix uses the author's larger floor/echo fixture. Focused
// cases below keep the entire prism strictly inside coarse XY=(0,0).
fn single_column_fixture(fine: f32, coarse: f32, floor_z: i32,
    extra: &[[i32; 3]], omit_observations: &[[i32; 3]], complete: bool, measured: bool)
    -> (SurfaceSnapshot, Pose) {
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, block_key, local_key};
    assert!(!complete || omit_observations.is_empty(), "a complete owner cannot omit known geometry");
    let n = (coarse / fine).round() as i32;
    let all: BTreeSet<_> = (-3*n..4*n).flat_map(|x|
        (-3*n..4*n).map(move |y| [x, y, floor_z])).chain(extra.iter().copied()).collect();
    let points: Vec<_> = all.iter().map(|k| ((k[0] as f32 + 0.5)*fine,
        (k[1] as f32 + 0.5)*fine, (k[2] as f32 + 0.5)*fine)).collect();
    let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for &key in &all {
        if omit_observations.contains(&key) { continue; }
        blocks.entry(block_key(key)).or_default().insert(key, SurfaceObservation { observed_ns: 1, hits: 1 });
    }
    let mut observations = SurfaceObservations::new(fine, 32).unwrap();
    for (block, values) in blocks { observations.replace_block(block, values).unwrap(); }
    let q = f64::from(fine);
    let height = if measured { (f64::from(floor_z) + 0.5)*q } else { f64::from(floor_z + 1)*q };
    if measured {
        let mut layers = BTreeMap::<[i32; 3], LayerObservations>::new();
        for &key in all.iter().filter(|key| key[2] == floor_z && !omit_observations.contains(key)) {
            let local = local_key(key);
            let point = [(f64::from(key[0]) + 0.5)*q, (f64::from(key[1]) + 0.5)*q, height];
            layers.entry(block_key(key)).or_default().insert(local, LayerObservation {
                members: vec![local], normal: [0.0, 0.0, 1.0], plane_origin: point,
                samples: vec![LayerSample { source: Default::default(), point,
                    source_sequence: 1, source_timestamp_ns: 1, point_index: u32::from(local) }],
            });
        }
        for (block, values) in layers { observations.replace_ground_layers(block, Arc::new(values)).unwrap(); }
    }
    if complete { observations.mark_complete_geometry_source(); }
    let mut cfg = scene(&[], false).config().clone();
    cfg.voxel_size = coarse; cfg.robot_height = 0.1; cfg.wall_clearance_m = 0.0;
    let half = f64::from(coarse)*0.45;
    let envelope = SurfaceBodyEnvelope {
        footprint_xy_m: vec![[-half,-half],[half,-half],[half,half],[-half,half]],
        z_bounds_m: [-0.425,0.1], body_to_support_z_m: 0.4,
        max_step_up_m: 0.05, max_step_down_m: 0.05,
        max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
    };
    let map = SurfaceSnapshot::from_points(&points, cfg, fine).unwrap()
        .with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)));
    let pose = ([f64::from(coarse)*0.5, f64::from(coarse)*0.5, 0.4+height], [0.0,0.0,0.0,1.0]);
    (map, pose)
}

struct AssignedPath {
    rows: Option<Vec<FineGroundColumn>>,
    prism: PrismQuery,
    ground_height: f64,
    coarse_contact: bool,
}
// This is a fixture admission assertion, not a second collision implementation.
// It reconstructs the unchanged prelude and proves the only visited coarse XY
// has an assigned ground key, no recent obstacle, and no fringe rejection.
fn assert_single_assigned(source: &SurfaceSnapshot, pose: Pose) -> AssignedPath {
    let map = source.fork_for_planning();
    map.validate_complete_source_resolution().unwrap();
    assert!(map.recent_obstacles.is_none());
    assert!(map.data.virtual_obstacles.is_none());
    let model = map.body_envelope().unwrap();
    let r = rotation(pose.1).unwrap();
    let centre = map.locate_support_from_body(pose.0, pose.1, model.body_to_support_z_m).unwrap();
    let vertices: Vec<_> = model.footprint_xy_m.iter()
        .flat_map(|p| model.z_bounds_m.map(|z| world(r,pose.0,[p[0],p[1],z]))).collect();
    let polygon = hull(vertices.iter().map(|p| [p[0],p[1]]).collect());
    let query = PolygonQuery::new(&polygon);
    let prism = PrismQuery::new(model,r,pose.0);
    let (min,max) = vertices.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]), |(mut a,mut b),p| {
        for i in 0..3 { a[i]=a[i].min(p[i]); b[i]=b[i].max(p[i]); } (a,b)
    });
    let size = f64::from(map.config().voxel_size);
    let bounds = ([min[0],min[1]].map(|v|(v/size).floor() as i32),
        [max[0],max[1]].map(|v|(v/size).floor() as i32));
    assert_eq!(bounds, ([0,0],[0,0]), "fixture must have exactly one coarse XY path");
    let radius = ((vertices.iter().map(|p| (p[0]-centre.surface_xyz[0]).hypot(p[1]-centre.surface_xyz[1]))
        .fold(0.0,f64::max)+size*2.0)/size).ceil()*size;
    let (patch,precise) = map.refine_footprint_support(map.ground_patch(centre,radius),
        &query,&prism,pose.0,r,bounds);
    let ground = *patch.get(&(0,0)).expect("target must use assigned, not unassigned, column");
    // Every focused fixture has a floor at fine z=-1 or measured fine z=0.
    let floor_layer = if source.observations.as_ref().is_some_and(|o|o.ground_member([0,0,0])) { 0 } else { -1 };
    assert_eq!(ground.2,floor_layer,"fixture must assign its original floor layer");
    let mut fringe = None;
    assert_eq!(map.check_fine_column_fringe(pose.0,r,&query,&prism,&patch,min,max,bounds,&mut fringe),Ok(()));
    assert!(fringe.is_none());
    let uncertainty = f64::from(map.source_voxel_size_m())+model.support_uncertainty_m;
    let contact = model.max_step_up_m.max(model.max_step_down_m)+uncertainty;
    let plane = query.contact_plane(pose.0,r[2],0,0,size,map.ground_cell_position(ground)[2],
        (-model.body_to_support_z_m/r[2][2]+contact+1e-6).min(uncertainty));
    let support = plane-model.body_to_support_z_m/r[2][2];
    let rows = map.fine_ground_columns(Some(ground),&query,&prism,pose.0,r[2],0,0);
    let ground_height = if rows.is_none() && precise.contains_key(&(0,0)) { precise[&(0,0)] }
        else { map.footprint_ground_height(ground,&query,&prism,support,contact,rows.as_deref()).unwrap() };
    let coarse_contact = height_rejection(plane+model.z_bounds_m[0]/r[2][2],
        support,plane,contact,ground_height,uncertainty).is_none();
    #[cfg(feature = "body-query-profile")]
    if source.observations.is_some() { assert_assigned_execution(source,pose); }
    AssignedPath { rows, prism, ground_height, coarse_contact }
}

// The parent owns the three profile-only counter definitions/instrumentation.
// This assertion accepts both dense baseline and indexed candidate; it never
// asserts the old-only actual_candidates == dense_span relation.
#[cfg(feature = "body-query-profile")]
fn assert_assigned_execution(source: &SurfaceSnapshot, pose: Pose) {
    use crate::body_query_profile::{self as profile, Counter};
    struct Cleanup(bool);
    impl Drop for Cleanup { fn drop(&mut self) { if self.0 { let _ = profile::end(); } } }
    let map = source.fork_for_planning();
    profile::begin(); let mut cleanup = Cleanup(true);
    let mut rejection = None;
    let result = map.check_body_pose(pose.0,pose.1,&mut rejection);
    let record = profile::end(); cleanup.0 = false;
    let dense = record.counters[Counter::AssignedDenseZSpan as usize];
    let actual = record.counters[Counter::AssignedActualZCandidates as usize];
    let absent = record.counters[Counter::AssignedFineGetAbsent as usize];
    eprintln!("ASSIGNED_Z_CONTROL fine={} coarse={} complete={} dense_z_span={} actual_z_candidates={} fine_get_absent={} result={:?} rejection={:?}",
        map.source_voxel_size_m(),map.config().voxel_size,
        map.observations.as_ref().unwrap().is_complete_geometry_source(),dense,actual,absent,result,rejection);
    assert!(dense>0, "the production call must execute the actual assigned fine-Z branch");
    assert!(actual<=dense); assert!(absent<=actual);
}

fn assert_clear(map: &SurfaceSnapshot, pose: Pose) {
    let (result,witness) = compare_pose(map,pose);
    assert_eq!(result,Ok(())); assert!(witness.is_none());
}
fn assert_collision(map: &SurfaceSnapshot, pose: Pose, cell: [i32;3], expected_source: OccupancySource) {
    let (result,witness) = compare_pose(map,pose);
    assert_eq!(result,Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness,Some(BodyRejection::Occupancy { cell: actual, source, .. })
        if actual==cell && source==expected_source), "expected {cell:?}/{expected_source:?}, got {witness:?}");
}

#[test]
fn pose_assigned_z_frozen_kernel_and_public_cache_matrix() {
    // Original author's 3 x 3 x 5 x 3 matrix, retained instead of a second broad matrix.
    for (fine,coarse) in [(0.01,0.1),(0.05,0.05),(0.05,0.1)] {
        for mode in 0..3 {
            let map = indexed_z_fixture(fine,coarse,mode);
            for p in [[-0.55,0.05,0.4],[-0.05,0.05,0.4],[-0.0,0.0,0.4],
                [0.1,0.1,0.4],[0.3,0.05,0.55]] {
                for q in [[0.0,0.0,0.0,1.0],[0.0,0.0,0.2,1.0],[0.02,-0.03,0.1,1.0]] {
                    compare_pose(&map,(p,q));
                }
            }
            let collision = ([-0.05,0.05,0.4],[0.0,0.0,0.0,1.0]);
            let centre = map.locate_support_from_body(collision.0,collision.1,0.4).unwrap();
            assert_eq!(map.ground_patch(centre,0.6).get(&(0,0)).map(|cell|cell.2),Some(-1),
                "the echo must retain the original floor anchor, not an arbitrary upper layer");
            let (result,witness) = compare_pose(&map,collision);
            assert_eq!(result,Err(SurfaceError::InsufficientClearance));
            let source = if mode==2 || (mode==1 && fine.to_bits()==coarse.to_bits()) {
                OccupancySource::CoarseFallback
            } else { OccupancySource::Fine };
            assert!(matches!(witness,Some(BodyRejection::Occupancy { source: actual,.. }) if actual==source));
            assert_clear(&map,([-0.55,0.05,0.4],collision.1));
        }
        // Missing observations are an explicit coarse-only control, not an
        // empty fine owner masquerading as absent observations.
        let coarse_only = indexed_z_fixture(fine,coarse,0).with_observations(None);
        assert!(coarse_only.observations.is_none());
        assert_clear(&coarse_only,([-0.55,0.05,0.4],[0.0,0.0,0.0,1.0]));
        let (result,witness) = compare_pose(&coarse_only,([-0.05,0.05,0.4],[0.0,0.0,0.0,1.0]));
        assert_eq!(result,Err(SurfaceError::InsufficientClearance));
        assert!(matches!(witness,Some(BodyRejection::Occupancy { source: OccupancySource::CoarseFallback,.. })));
    }
}

#[test]
fn pose_assigned_z_contact_members_are_real_and_do_not_absorb_a_nonmember() {
    // Legacy contacts overlap the actual prism. Their real members must set
    // found=true and keep contact exempt during the actual assigned loop.
    for (fine,coarse) in [(0.01,0.1),(0.05,0.05),(0.05,0.1)] {
        for complete in [false,true] {
            let (map,pose) = single_column_fixture(fine,coarse,-1,&[],&[],complete,false);
            let path = assert_single_assigned(&map,pose);
            let observations = map.observations.as_ref().unwrap();
            let q = f64::from(fine);
            let rows = path.rows.unwrap(); assert!(!rows.is_empty());
            for row in rows {
                assert_eq!(row.contacts.as_slice(),&[[-1,-1]]);
                let key = [row.xy[0],row.xy[1],-1];
                assert!(observations.get(key).is_some());
                assert!(observations.column_levels(key[0],key[1]).unwrap().contains(&key[2]));
                assert!(path.prism.intersects(key.map(|v|f64::from(v)*q),key.map(|v|f64::from(v+1)*q)));
            }
            assert_clear(&map,pose);
        }
    }
    // Measured membership is exactly {0}; an adjacent real z=1 return in the
    // SAME coarse cell remains a rigid obstacle, despite its small height.
    for fine in [0.01,0.05] {
        let (clear,pose) = single_column_fixture(fine,0.1,0,&[],&[],true,true);
        let (blocked,blocked_pose) = single_column_fixture(fine,0.1,0,&[[0,0,1]],&[],true,true);
        assert_eq!(pose,blocked_pose);
        for map in [&clear,&blocked] {
            let path = assert_single_assigned(map,pose);
            let rows = path.rows.unwrap();
            let row = rows.iter().find(|row|row.xy==[0,0]).unwrap();
            assert_eq!(row.contacts.as_slice(),&[[0,0]]);
            assert!(row.permits_contact(0)); assert!(!row.permits_contact(1));
            assert!(map.observations.as_ref().unwrap().get([0,0,0]).is_some());
            assert!(map.observations.as_ref().unwrap().ground_member([0,0,0]));
            let bounds = map.data.height_bounds[&(0,0,0)];
            assert!(f64::from(bounds[1]) > path.ground_height+1e-6,
                "without found=true, this real measured contact WOULD trigger coarse fallback");
            let size = f64::from(map.config().voxel_size);
            assert!(path.prism.intersects([0.0,0.0,f64::from(bounds[0])],
                [size,size,f64::from(bounds[1])]));
        }
        assert!(blocked.observations.as_ref().unwrap().get([0,0,1]).is_some());
        assert!(!blocked.observations.as_ref().unwrap().ground_member([0,0,1]));
        assert_clear(&clear,pose);
        assert_collision(&blocked,pose,[0,0,1],OccupancySource::Fine);
    }
}

#[test]
fn pose_assigned_z_missing_slice_precedes_but_never_overrides_later_fine_hit() {
    let fine = 0.01_f32; let coarse = 0.1_f32;
    let hit = [1,1,22]; let missing = [0,0,-1];
    let (map,pose) = single_column_fixture(fine,coarse,-1,&[hit],&[missing],false,false);
    let path = assert_single_assigned(&map,pose);
    let observations = map.observations.as_ref().unwrap();
    assert!(!observations.is_complete_geometry_source());
    assert!(observations.column_levels(0,0).is_none(), "first XY is truly empty");
    let rows = path.rows.as_ref().unwrap();
    assert!(rows.iter().find(|r|r.xy==[0,0]).unwrap().height.is_none());
    assert!(rows.iter().find(|r|r.xy==[1,1]).unwrap().height.is_some());
    let bounds = map.data.height_bounds[&(0,0,2)];
    let q = f64::from(fine);
    assert!(!path.coarse_contact || f64::from(bounds[1])>path.ground_height+1e-6);
    assert!(path.prism.intersects([0.0,0.0,f64::from(bounds[0])],[q,q,f64::from(bounds[1])]));
    assert!([0,0] < [hit[0],hit[1]], "missing_slice is evaluated before the fine witness");
    assert!(observations.get(hit).is_some());
    assert_collision(&map,pose,hit,OccupancySource::Fine);
    // Same original coarse obstacle but no fine echo: conservative fallback is
    // a real negative control. This is an explicitly partial source.
    let (fallback,fallback_pose) = single_column_fixture(fine,coarse,-1,&[hit],&[missing,hit],false,false);
    assert_single_assigned(&fallback,fallback_pose);
    assert_collision(&fallback,fallback_pose,[0,0,2],OccupancySource::CoarseFallback);
    // Stronger missing-slice control: found=true comes from a later real fine
    // voxel OUTSIDE the shortened prism. Only the earlier empty XY slice makes
    // this reject. Skipping that XY before missing_slice_hit would wrongly pass.
    let upper = [1,1,28];
    let (source,short_pose) = single_column_fixture(fine,coarse,-1,&[hit,upper],&[missing,hit],false,false);
    let mut envelope = source.body_envelope().unwrap().clone();
    envelope.z_bounds_m[1] = -0.145;
    let missing_only = source.with_body_envelope(envelope).unwrap();
    let short = assert_single_assigned(&missing_only,short_pose);
    let observations = missing_only.observations.as_ref().unwrap();
    assert!(observations.column_levels(0,0).is_none());
    assert!(observations.get(upper).is_some());
    assert!(!short.prism.intersects(upper.map(|v|f64::from(v)*q),upper.map(|v|f64::from(v+1)*q)));
    let bounds = missing_only.data.height_bounds[&(0,0,2)];
    assert!(short.prism.intersects([0.0,0.0,f64::from(bounds[0])],[q,q,f64::from(bounds[1])]));
    let first = (f64::from(bounds[0])/q+1e-5).floor() as i32;
    let last = (f64::from(bounds[1])/q-1e-5).ceil() as i32;
    assert!(first<=upper[2] && upper[2]<last, "the nonhit really sets found in this same coarse-Z iteration");
    assert_collision(&missing_only,short_pose,[0,0,2],OccupancySource::CoarseFallback);
}

#[test]
fn pose_assigned_z_first_witness_keeps_coarse_z_then_fine_xyz_order() {
    let cases: [(f32,&[[i32;3]],[i32;3]);2] = [
        (0.01,&[[0,1,22],[0,1,24],[0,2,21],[1,0,20],[0,0,31]],[0,1,22]),
        (0.05,&[[0,1,4],[0,1,5],[1,0,4],[0,0,6]],[0,1,4]),
    ];
    for (fine,echoes,first) in cases {
        let (map,pose) = single_column_fixture(fine,0.1,-1,echoes,&[],true,false);
        let path = assert_single_assigned(&map,pose);
        let q = f64::from(fine);
        let coarse_levels = &map.data.occupied_columns[&(0,0)];
        assert!(coarse_levels.windows(2).all(|p|p[0]<p[1]));
        assert!(coarse_levels.contains(&2) && coarse_levels.contains(&3));
        for &key in echoes {
            assert!(map.observations.as_ref().unwrap().get(key).is_some());
            assert!(path.prism.intersects(key.map(|v|f64::from(v)*q),key.map(|v|f64::from(v+1)*q)));
            let row = path.rows.as_ref().unwrap().iter().find(|row|row.xy==[key[0],key[1]]).unwrap();
            assert!(!row.permits_contact(key[2]));
        }
        // The z=3 coarse cell contains lexicographically earlier fine XY=(0,0),
        // but coarse z=2 must reject first. Within z=2, X then Y outrank lower Z
        // elsewhere, and the first occupied Z of the selected XY wins.
        assert_collision(&map,pose,first,OccupancySource::Fine);
    }
}

#[test]
fn pose_assigned_z_integer_selector_preserves_half_open_order() {
    // Bounded selector-only supplemental control; it is not branch coverage.
    let levels = [i32::MIN,-11,-2,-1,0,1,2,9,i32::MAX];
    for (first,last) in [(i32::MIN,i32::MIN),(-20,-10),(-2,3),(0,0),(2,-2),(8,10),
        (i32::MAX-1,i32::MAX),(i32::MAX,i32::MAX)] {
        let expected: Vec<_> = levels.iter().copied().filter(|&z|first<=z && z<last).collect();
        let begin = levels.partition_point(|&z|z<first);
        let actual: Vec<_> = levels.iter().skip(begin).take_while(|&&z|z<last).copied().collect();
        assert_eq!(actual,expected,"first={first} last={last}");
    }
    // Also derive the half-open range from the original FLOAT bound formula,
    // including both adjacent f32 values around positive/negative boundaries.
    fn adjacent(value: f32, up: bool) -> f32 {
        if value==0.0 { return f32::from_bits(if up { 1 } else { (1_u32<<31)|1 }); }
        let bits=value.to_bits();
        f32::from_bits(if (value>0.0)==up { bits+1 } else { bits-1 })
    }
    let occupied = [-31,-30,-21,-20,-19,-11,-10,-9,-2,-1,0,1,2,9,10,19,20,21,29,30,31];
    for (fine,low,high) in [(0.01_f32,0.2_f32,0.3_f32),(0.01,-0.3,-0.2),
        (0.05,-0.15,0.05),(0.05,0.0,0.0),(0.05,0.1,-0.1)] {
        for low in [adjacent(low,false),low,adjacent(low,true)] {
            for high in [adjacent(high,false),high,adjacent(high,true)] {
                let first=(f64::from(low)/f64::from(fine)+1e-5).floor() as i32;
                let last=(f64::from(high)/f64::from(fine)-1e-5).ceil() as i32;
                let expected: Vec<_> = (first..last).filter(|z|occupied.binary_search(z).is_ok()).collect();
                let begin=occupied.partition_point(|&z|z<first);
                let actual: Vec<_> = occupied.iter().skip(begin).take_while(|&&z|z<last).copied().collect();
                assert_eq!(actual,expected,"fine={fine}, low={low:?}, high={high:?}");
            }
        }
    }
}


#[cfg(feature = "body-query-profile")]
fn capture_pose_path_scope(source: &SurfaceSnapshot, pose: Pose)
    -> (Result<(),SurfaceError>,Option<BodyRejection>,crate::body_query_profile::Record) {
    use crate::body_query_profile as profile;
    struct Cleanup(bool);
    impl Drop for Cleanup { fn drop(&mut self) { if self.0 { let _ = profile::end(); } } }
    let map=source.fork_for_planning();
    profile::begin(); let mut cleanup=Cleanup(true);
    let mut rejection=Some(poison());
    let result=map.check_body_pose(pose.0,pose.1,&mut rejection);
    let record=profile::end(); cleanup.0=false;
    (result,rejection,record)
}

#[cfg(feature = "body-query-profile")]
fn assert_pose_path_scope(record: &crate::body_query_profile::Record, columns: u32,
    assigned: u64, unassigned: u64) {
    use crate::body_query_profile::{Stage,Counter};
    assert_eq!(record.calls[Stage::PoseCoreTotal as usize],1);
    assert_eq!(record.calls[Stage::PosePrelude as usize],1);
    assert_eq!(record.calls[Stage::PoseCoarseColumns as usize],columns);
    assert_eq!(record.counters[Counter::PoseAssignedColumns as usize],assigned);
    assert_eq!(record.counters[Counter::PoseUnassignedColumns as usize],unassigned);
    let total=record.wall_nanos[Stage::PoseCoreTotal as usize];
    let prelude=record.wall_nanos[Stage::PosePrelude as usize];
    let coarse=record.wall_nanos[Stage::PoseCoarseColumns as usize];
    assert!(total>=prelude+coarse,"disjoint P/C must fit inside T: {total}, {prelude}, {coarse}");
}

#[test]
#[cfg(feature = "body-query-profile")]
fn pose_path_profile_success_records_real_assigned_and_unknown_columns() {
    let (source,pose)=single_column_fixture(0.01,0.1,-1,&[],&[],true,false);
    let (result,rejection,record)=capture_pose_path_scope(&source,pose);
    assert_eq!(result,Ok(())); assert!(rejection.is_none());
    assert_pose_path_scope(&record,1,1,0);
    // Same immutable source, actual unknown space beyond its retained floor.
    // Unknown ground remains traversable; no fabricated ground or witness.
    let size=f64::from(source.config().voxel_size);
    let unknown=([size*50.5,size*50.5,0.4],pose.1);
    assert!(matches!(source.locate_support_from_body(unknown.0,unknown.1,0.4),
        Err(SurfaceError::MissingSupport)));
    let (result,rejection,record)=capture_pose_path_scope(&source,unknown);
    assert_eq!(result,Ok(())); assert!(rejection.is_none());
    assert_pose_path_scope(&record,1,0,1);
}

#[test]
#[cfg(feature = "body-query-profile")]
fn pose_path_profile_early_prelude_and_column_errors_finish_scopes() {
    let (clear,pose)=single_column_fixture(0.01,0.1,-1,&[],&[],true,false);
    let (result,rejection,record)=capture_pose_path_scope(&clear,(pose.0,[0.0;4]));
    assert_eq!(result,Err(SurfaceError::InvalidPose)); assert!(rejection.is_none());
    assert_pose_path_scope(&record,0,0,0);
    // Reuse the already qualified true measured-member/nonmember collision.
    let (blocked,pose)=single_column_fixture(0.01,0.1,0,&[[0,0,1]],&[],true,true);
    let (expected,witness)=compare_pose(&blocked,pose);
    let (result,rejection,record)=capture_pose_path_scope(&blocked,pose);
    assert_result_bits(&result,&expected);
    assert_eq!(rejection_bits(rejection),rejection_bits(witness));
    assert_eq!(result,Err(SurfaceError::InsufficientClearance));
    assert!(matches!(rejection,Some(BodyRejection::Occupancy {cell:[0,0,1],source:OccupancySource::Fine,..})));
    assert_pose_path_scope(&record,1,1,0);
}

// A predicate/consumer admission check, not a replacement collision implementation.
// Original full rows and height-only rows are queried outside the profile interval.
struct LazyContactAdmission {
    assigned: u64,
    disjoint: u64,
    overlapping: u64,
    ground_points: Vec<[f64;3]>,
    min: [f64;3],
    max: [f64;3],
}
fn admit_lazy_contact_pose(source:&SurfaceSnapshot,pose:Pose)->LazyContactAdmission {
    let map=source.fork_for_planning();
    assert!(map.recent_obstacles.is_none() && map.data.virtual_obstacles.is_none());
    let model=map.body_envelope().unwrap(); let r=rotation(pose.1).unwrap();
    let centre=map.locate_support_from_body(pose.0,pose.1,model.body_to_support_z_m).unwrap();
    let vertices:Vec<_>=model.footprint_xy_m.iter()
        .flat_map(|p|model.z_bounds_m.map(|z|world(r,pose.0,[p[0],p[1],z]))).collect();
    let polygon=hull(vertices.iter().map(|p|[p[0],p[1]]).collect());
    let query=PolygonQuery::new(&polygon); let prism=PrismQuery::new(model,r,pose.0);
    let (min,max)=vertices.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(mut a,mut b),p| {
        for i in 0..3 { a[i]=a[i].min(p[i]); b[i]=b[i].max(p[i]); } (a,b)
    });
    let size=f64::from(map.config().voxel_size);
    let bounds=([min[0],min[1]].map(|v|(v/size).floor()as i32),
        [max[0],max[1]].map(|v|(v/size).floor()as i32));
    let radius=((vertices.iter().map(|p|(p[0]-centre.surface_xyz[0]).hypot(p[1]-centre.surface_xyz[1]))
        .fold(0.0,f64::max)+size*2.0)/size).ceil()*size;
    let (patch,precise)=map.refine_footprint_support(map.ground_patch(centre,radius),
        &query,&prism,pose.0,r,bounds);
    let mut fringe=None;
    assert_eq!(map.check_fine_column_fringe(pose.0,r,&query,&prism,&patch,min,max,bounds,&mut fringe),Ok(()));
    assert!(fringe.is_none(),"control must reach the original coarse loop");
    let mut result=LazyContactAdmission {assigned:0,disjoint:0,overlapping:0,ground_points:Vec::new(),min,max};
    let uncertainty=f64::from(map.source_voxel_size_m())+model.support_uncertainty_m;
    let contact=model.max_step_up_m.max(model.max_step_down_m)+uncertainty;
    for x in bounds.0[0]..=bounds.1[0] {for y in bounds.0[1]..=bounds.1[1] {
        if !query.intersects(x,y,size) {continue;}
        let ground=*patch.get(&(x,y)).expect("controlled footprint must use assigned ground");
        result.assigned+=1;
        let plane=query.contact_plane(pose.0,r[2],x,y,size,map.ground_cell_position(ground)[2],
            (-model.body_to_support_z_m/r[2][2]+contact+1e-6).min(uncertainty));
        let support=plane-model.body_to_support_z_m/r[2][2];
        let rows=map.fine_ground_columns(Some(ground),&query,&prism,pose.0,r[2],x,y);
        let old_height=if rows.is_none() && precise.contains_key(&(x,y)) {precise[&(x,y)]}
            else {map.footprint_ground_height(ground,&query,&prism,support,contact,rows.as_deref()).unwrap()};
        let height=map.fine_ground_height(ground,&query,&prism,pose.0,r[2],x,y);
        assert_eq!(height.is_some(),rows.is_some(),"Some(empty/all-None) must remain Some");
        let new_height=if let Some(height)=height {height}
            else if precise.contains_key(&(x,y)) {precise[&(x,y)]}
            else {map.footprint_ground_height(ground,&query,&prism,support,contact,None).unwrap()};
        assert_eq!(new_height.to_bits(),old_height.to_bits(),"ordered row maximum/fallback bits");
        result.ground_points.push([(f64::from(x)+0.5)*size,(f64::from(y)+0.5)*size,old_height]);
        let has_overlap=map.data.occupied_columns.get(&(x,y)).is_some_and(|zs|zs.iter().any(|&z| {
            let b=map.data.height_bounds.get(&(x,y,z)).copied().unwrap_or([
                z as f32*map.config().voxel_size,(z as f32+1.0)*map.config().voxel_size]);
            !(f64::from(b[0])>max[2] || f64::from(b[1])<min[2])
        }));
        if has_overlap {result.overlapping+=1;} else {result.disjoint+=1;}
    }}
    assert!(result.assigned>0);
    result
}
fn disjoint_contact_fixture(fine:f32,coarse:f32,measured:bool,complete:bool)->(SurfaceSnapshot,Pose) {
    let (source,pose)=single_column_fixture(fine,coarse,if measured {0}else{-1},&[],&[],complete,measured);
    let mut envelope=source.body_envelope().unwrap().clone(); envelope.z_bounds_m=[-0.15,0.15];
    (source.with_body_envelope(envelope).unwrap(),pose)
}
fn assert_independent_source_cold(actual:SurfaceSnapshot,oracle:SurfaceSnapshot,pose:Pose) {
    // Independently built owners, rather than forks sharing the source reference cache.
    assert!(!Arc::ptr_eq(&actual.observations.as_ref().unwrap().legacy_reference_cache,
        &oracle.observations.as_ref().unwrap().legacy_reference_cache));
    assert!(!Arc::ptr_eq(&actual.body_query_cache,&oracle.body_query_cache));
    let mut a=Some(poison()); let mut b=Some(poison());
    let ar=actual.check_body_pose(pose.0,pose.1,&mut a);
    let br=oracle.check_body_pose_dense_z_oracle(pose.0,pose.1,&mut b);
    assert_result_bits(&ar,&br); assert_eq!(rejection_bits(a),rejection_bits(b));
}

#[test]
fn pose_lazy_contacts_real_disjoint_assigned_matches_frozen_and_independent_cold() {
    for (fine,coarse) in [(0.01,0.1),(0.05,0.05),(0.05,0.1)] {
        for measured in [false,true] {for complete in [false,true] {
            let (map,pose)=disjoint_contact_fixture(fine,coarse,measured,complete);
            let admission=admit_lazy_contact_pose(&map,pose);
            assert_eq!((admission.assigned,admission.disjoint,admission.overlapping),(1,1,0));
            assert_clear(&map,pose); // Full frozen kernel plus cold/warm public witness parity.
            #[cfg(feature="body-query-profile")] {
                use crate::body_query_profile::Counter;
                let (result,witness,record)=capture_pose_path_scope(&map,pose);
                assert_eq!(result,Ok(())); assert!(witness.is_none());
                assert_pose_path_scope(&record,1,1,0);
                assert_eq!(record.counters[Counter::AssignedDenseZSpan as usize],0);
                eprintln!("LAZY_CONTACT_CONTROL fine={fine} coarse={coarse} measured={measured} complete={complete} assigned=1 independently_proved_disjoint=1");
            }
        }}
        // One representative genuinely independent source rebuild per resolution,
        // before any query/admission check warms these two new owners.
        let (actual,pose)=disjoint_contact_fixture(fine,coarse,false,true);
        let (oracle,oracle_pose)=disjoint_contact_fixture(fine,coarse,false,true);
        assert_eq!(pose,oracle_pose); assert_independent_source_cold(actual,oracle,pose);
    }
}

fn adjacent_positive(value:f64,up:bool)->f64 {
    assert!(value>0.0 && value.is_finite());
    f64::from_bits(if up {value.to_bits()+1}else{value.to_bits()-1})
}
#[test]
fn pose_lazy_contacts_closed_z_boundary_keeps_collision_path_and_first_witness() {
    for (fine,coarse,echo_z) in [(0.01,0.1,22),(0.05,0.05,4),(0.05,0.1,4)] {
        let echo=[0,0,echo_z];
        let (source,pose)=single_column_fixture(fine,coarse,-1,&[echo],&[],true,false);
        let coarse_z=(((f64::from(echo_z)+0.5)*f64::from(fine))/f64::from(coarse)).floor()as i32;
        let stored=source.data.height_bounds.get(&(0,0,coarse_z)).copied().unwrap_or([
            coarse_z as f32*coarse,(coarse_z as f32+1.0)*coarse]);
        let high=f64::from(stored[1]);
        for (bottom,overlap) in [(adjacent_positive(high,false),true),(high,true),(adjacent_positive(high,true),false)] {
            let mut envelope=source.body_envelope().unwrap().clone();
            envelope.z_bounds_m=[bottom-pose.0[2],0.15];
            let map=source.with_body_envelope(envelope).unwrap();
            let admission=admit_lazy_contact_pose(&map,pose);
            assert_eq!(admission.min[2].to_bits(),bottom.to_bits(),"real transformed bound, not a rounded target");
            assert_eq!(admission.overlapping,if overlap {1}else{0});
            if overlap {
                let (result,witness)=compare_pose(&map,pose);
                assert_eq!(result,Err(SurfaceError::InsufficientClearance));
                assert!(matches!(witness,Some(BodyRejection::Occupancy{source:OccupancySource::Fine,..})));
                #[cfg(feature="body-query-profile")] assert_assigned_execution(&map,pose);
            } else { assert_clear(&map,pose); }
        }
        // The other closed endpoint: body top touches the obstacle's lower bound.
        let low=f64::from(stored[0]);
        for (top,overlap) in [(adjacent_positive(low,false),false),(low,true),(adjacent_positive(low,true),true)] {
            let mut envelope=source.body_envelope().unwrap().clone();
            envelope.z_bounds_m=[-0.3,top-pose.0[2]];
            let map=source.with_body_envelope(envelope).unwrap();
            let admission=admit_lazy_contact_pose(&map,pose);
            assert_eq!(admission.max[2].to_bits(),top.to_bits(),"real transformed upper bound");
            assert_eq!(admission.overlapping,if overlap {1}else{0});
            if overlap {
                let (result,witness)=compare_pose(&map,pose);
                assert_eq!(result,Err(SurfaceError::InsufficientClearance));
                assert!(matches!(witness,Some(BodyRejection::Occupancy{source:OccupancySource::Fine,..})));
                #[cfg(feature="body-query-profile")] assert_assigned_execution(&map,pose);
            } else { assert_clear(&map,pose); }
        }
    }
    // Predicate-only supplement: NaN is not permission to skip the original path.
    for (lo,hi,min,max) in [(f64::NAN,1.0,0.0,2.0),(0.0,f64::NAN,0.0,2.0),
        (-0.0,0.0,0.0,1.0),(1.0,2.0,0.0,1.0)] {
        assert!(!(lo>max || hi<min));
    }
    // Real mixed footprint: clear assigned columns precede the original echo.
    for fine in [0.01,0.05] {
        let map=indexed_z_fixture(fine,0.1,0); let pose=([-0.05,0.05,0.4],[0.,0.,0.,1.]);
        let admission=admit_lazy_contact_pose(&map,pose);
        assert!(admission.disjoint>0 && admission.overlapping>0);
        let (result,witness)=compare_pose(&map,pose);
        assert_eq!(result,Err(SurfaceError::InsufficientClearance));
        assert!(matches!(witness,Some(BodyRejection::Occupancy{source:OccupancySource::Fine,..})));
    }
}

#[test]
fn pose_lazy_contacts_all_none_rows_keep_some_and_precise_precedence_contract() {
    for fine in [0.01,0.05] {
        let (base,pose)=disjoint_contact_fixture(fine,0.1,false,true);
        let empty=SurfaceObservations::new(fine,32).unwrap(); // Explicitly partial, not complete.
        let partial=base.with_observations(Some(Arc::new(empty)));
        let absent=base.with_observations(None);
        for source in [&partial,&absent] {
            let admission=admit_lazy_contact_pose(source,pose);
            assert_eq!((admission.disjoint,admission.overlapping),(1,0));
            assert_clear(source,pose);
        }
        let model=partial.body_envelope().unwrap();let r=rotation(pose.1).unwrap();
        let points:Vec<_>=model.footprint_xy_m.iter().map(|p|[p[0]+pose.0[0],p[1]+pose.0[1]]).collect();
        let query=PolygonQuery::new(&points);let prism=PrismQuery::new(model,r,pose.0);
        let ground=partial.locate_support_from_body(pose.0,pose.1,model.body_to_support_z_m).unwrap().cell;
        let rows=partial.fine_ground_columns(Some(ground),&query,&prism,pose.0,r[2],0,0).unwrap();
        assert!(!rows.is_empty() && rows.iter().all(|row|row.height.is_none()));
        let fallback=partial.ground_cell_position(ground)[2];
        let height=partial.fine_ground_height(ground,&query,&prism,pose.0,r[2],0,0);
        assert_eq!(height.map(f64::to_bits),Some(fallback.to_bits()));
        assert!(absent.fine_ground_height(ground,&query,&prism,pose.0,r[2],0,0).is_none());
        // Defensive fallback contract only, NOT a claim that the public path can
        // produce precise_ground while observations/model are absent.
        let precise_height=3.0;
        assert_eq!(height.unwrap_or(precise_height).to_bits(),fallback.to_bits());
        let absent_height=absent.fine_ground_height(ground,&query,&prism,pose.0,r[2],0,0);
        assert_eq!(absent_height.unwrap_or(precise_height).to_bits(),precise_height.to_bits());
    }
}

fn disjoint_cross_slope_fixture(fine:f32)->(SurfaceSnapshot,Pose) {
    let n=(0.5/fine).round()as i32;
    let keys:BTreeSet<_>=(-n..=n).flat_map(|x|(-n..=n).map(move|y|[x,y,y.div_euclid(5)-1])).collect();
    let points:Vec<_>=keys.iter().map(|k|((k[0]as f32+0.5)*fine,(k[1]as f32+0.5)*fine,(k[2]as f32+0.5)*fine)).collect();
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:1,hits:2});}
    let mut observations=SurfaceObservations::new(fine,32).unwrap();
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
    observations.mark_complete_geometry_source();
    let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.1;cfg.wall_clearance_m=0.0;
    let envelope=SurfaceBodyEnvelope{footprint_xy_m:vec![[-0.18,-0.18],[0.18,-0.18],[0.18,0.18],[-0.18,0.18]],
        z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
        max_cross_slope_deg:1.0,support_uncertainty_m:0.0};
    let map=SurfaceSnapshot::from_points(&points,cfg,fine).unwrap().with_body_envelope(envelope).unwrap()
        .with_observations(Some(Arc::new(observations)));
    let z=*map.data.physical_graph.surface_lookup[&(0,0)].first().unwrap();
    let height=map.ground_cell_position((0,0,z))[2];
    (map,([0.05,0.05,height+0.4],[0.,0.,0.,1.]))
}
#[test]
fn pose_lazy_contacts_disjoint_columns_still_reject_actual_cross_slope() {
    for fine in [0.01,0.05] {
        let (map,pose)=disjoint_cross_slope_fixture(fine);
        let admission=admit_lazy_contact_pose(&map,pose);
        assert!(admission.assigned>2 && admission.disjoint==admission.assigned && admission.overlapping==0);
        let (_,b,_)=fit_plane(&admission.ground_points,pose.0).expect("real multi-column ground plane");
        assert!(b.abs().atan().to_degrees()>map.body_envelope().unwrap().max_cross_slope_deg+1e-5);
        assert_eq!(map.body_ground_relation(pose.0,pose.1),Ok(BodyGroundRelation::KnownCompatible),
            "same error must not be explained by the later known-terrain tail");
        let (result,witness)=compare_pose(&map,pose);
        assert_eq!(result,Err(SurfaceError::UntraversableSegment));assert!(witness.is_none());
        #[cfg(feature="body-query-profile")] {
            use crate::body_query_profile::Counter;
            let (result,witness,record)=capture_pose_path_scope(&map,pose);
            assert_eq!(result,Err(SurfaceError::UntraversableSegment));assert!(witness.is_none());
            assert_pose_path_scope(&record,1,admission.assigned,0);
            assert_eq!(record.counters[Counter::AssignedDenseZSpan as usize],0);
        }
    }
}
