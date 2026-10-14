// Uncached complete pose comparator for recent height-only row reuse.
// Ground classification follows the current source-only contract.
use super::*;

impl SurfaceSnapshot {
    fn reference_recent_body_pose(&self,body:[f64;3],quaternion:[f64;4],slot:&mut Option<BodyRejection>)->Result<(),SurfaceError> {
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
                            let selected=self.fine_ground_columns(ground,&polygon_query,&prism_query,body,r[2],cx,cy);
                            let height=if let Some(rows)=&selected {rows.iter().find(|row|row.xy==[x,y]).and_then(|row|row.height)}
                                else {ground.map(|g|self.cell_position(g)[2])};
                            for &z in zs {
                                if let Some(points)=frame.surface_points.get(&[x,y,z]) {
                                    if !points.iter().any(|&point|!self.recent_surface_contact(point,height,body,r)
                                        &&prism_query.intersects(point,point)) {continue;}
                                }
                                let low=[x,y,z].map(|v|f64::from(v)*fine);
                                let high=[x,y,z].map(|v|f64::from(v+1)*fine);
                                if high[2]>=min[2] && low[2]<=max[2] && prism_query.intersects(low,high) {
                                    return Err(reject(slot,BodyRejection::Occupancy{cell:[x,y,z],
                                        source:OccupancySource::Recent,z_bounds:[low[2],high[2]],ground_height:None}));
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
                let plane=polygon_query.contact_plane(body,r[2],x,y,size,self.cell_position(ground)[2],
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
                                        if !observations.get([ix,iy,iz]).is_some() {continue;}
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
        if self.body_ground_relation(body,quaternion)?==BodyGroundRelation::KnownIncompatible {
            return Err(SurfaceError::UntraversableSegment);
        }
        Ok(())
    }

}

fn recent_fixture(coarse:f32,kind:&str,frames:u64)->SurfaceSnapshot {
    let fine=0.05_f32;
    let mut points:Vec<_>=(-25..25).flat_map(|x|(-20..20).filter_map(move|y| {
        (kind!="fallback" || x<0 || x>3 || y<0 || y>3)
            .then_some(((x as f32+0.5)*fine,(y as f32+0.5)*fine,0.025))
    })).collect();
    if kind=="door" {
        for x in -25..25 {for y in [-5,4] {for z in 1..14 {
            points.push(((x as f32+0.5)*fine,(y as f32+0.5)*fine,(z as f32+0.5)*fine));
        }}}
    }
    let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=coarse;cfg.wall_clearance_m=0.;
    let mut envelope=model();
    envelope.footprint_xy_m=vec![[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]];
    envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
    envelope.max_cross_slope_deg=18.;
    let mut map=SurfaceSnapshot::from_points(&points,cfg,fine).unwrap()
        .with_body_envelope(envelope).unwrap();
    if kind!="coarse_only" {
        map=map.with_observations(Some(witnessed_observations(&points,fine)));
    }
    if kind=="no_recent" {return map;}
    // Live floor returns overlap body Z bounds, but are actual floor contact.
    let mut live:Vec<_>=(-12..12).flat_map(|x|(-8..8).map(move|y|
        [(f64::from(x)+0.5)*f64::from(fine),(f64::from(y)+0.5)*f64::from(fine),0.04])).collect();
    let ground:Vec<_>=(0..live.len() as u32).collect();
    if kind=="obstacle" {live.push([0.075,0.025,0.45]);live.push([0.125,0.025,0.4]);}
    for sequence in 1..=frames {
        map=map.with_recent_obstacle_points(sequence,sequence*100,1000,&ground,
            live.iter().copied().enumerate().map(|(i,p)|(i as u32,p))).unwrap();
    }
    map
}

fn recent_poses()->Vec<([f64;3],[f64;4])> {
    (0..16).map(|i| {
        let yaw=(i as f64-7.0)*0.003;
        ([0.005+(i as f64)*0.0007,0.005-(i as f64)*0.0003,0.45],
            [0.,0.,(yaw*0.5).sin(),(yaw*0.5).cos()])
    }).collect()
}

#[test]
fn recent_query_matches_frozen_complete_pose_and_first_rejection() {
    let poses=recent_poses();let mut successes=0;let mut failures=0;
    for coarse in [0.05,0.075,0.1] {for kind in ["no_recent","clear","fallback","door","obstacle","coarse_only"] {
        for frames in [1,8] {
            let map=recent_fixture(coarse,kind,frames);
            for &pose in &poses {
                let mut old=None;let mut new=None;
                let expected=map.reference_recent_body_pose(pose.0,pose.1,&mut old);
                let actual=map.check_body_pose(pose.0,pose.1,&mut new);
                assert_eq!(actual,expected,"{coarse} {kind} {frames} {pose:?}");
                assert_eq!(new,old,"first rejection: {coarse} {kind} {frames} {pose:?}");
                if actual.is_ok() {successes+=1;}else{failures+=1;}
                if kind=="obstacle" {assert!(matches!(new,Some(BodyRejection::Occupancy{source:OccupancySource::Recent,..})));}
                else if kind!="fallback" {assert!(actual.is_ok(),"{coarse} {kind} {frames}: {actual:?}");}
            }
        }
    }}
    assert!(successes>0&&failures>0);
    eprintln!("RECENT_QUERY_PARITY successes={successes} failures={failures}");
}

#[test]
#[ignore="manual serial release A/B benchmark; includes full uncached body pose query"]
fn recent_query_complete_pose_ab_benchmark() {
    use std::{hint::black_box,time::Instant};
    let poses=recent_poses();
    let pairs:usize=std::env::var("NEH_RECENT_PAIRS").ok().and_then(|s|s.parse().ok()).unwrap_or(7);
    let repeats:usize=std::env::var("NEH_RECENT_REPEATS").ok().and_then(|s|s.parse().ok()).unwrap_or(8);
    for coarse in [0.05,0.075,0.1] {for kind in ["no_recent","clear","fallback","door","obstacle","coarse_only"] {
        for frames in [1,8] {
            let map=recent_fixture(coarse,kind,frames);
            let expected:Vec<_>=poses.iter().map(|&(p,q)| {let mut slot=None;
                let result=map.reference_recent_body_pose(p,q,&mut slot);(result,slot)}).collect();
            for (i,&(p,q)) in poses.iter().enumerate() {let mut slot=None;
                assert_eq!((map.check_body_pose(p,q,&mut slot),slot),expected[i]);}
            for pair in 0..pairs {
                let mut elapsed=[0u128;2];
                for arm in if pair%2==0 {[0,1]}else{[1,0]} {
                    let start=Instant::now();
                    for _ in 0..repeats {for (i,&(p,q)) in poses.iter().enumerate() {
                        let mut slot=None;
                        let result=if arm==0 {map.reference_recent_body_pose(black_box(p),black_box(q),&mut slot)}
                            else{map.check_body_pose(black_box(p),black_box(q),&mut slot)};
                        assert_eq!((result,slot),expected[i]);
                    }}
                    elapsed[arm]=start.elapsed().as_nanos();
                }
                eprintln!("RECENT_QUERY_AB coarse={coarse} kind={kind} frames={frames} pair={pair} calls={} old_ns={} new_ns={}",
                    repeats*poses.len(),elapsed[0],elapsed[1]);
            }
        }
    }}
}

#[test]
fn recent_height_rows_keep_fine_xy_unknown_layers_and_exact_height_bits() {
    use nuwax_map_chunks::{LayerObservation,LayerSample,local_key};
    let points=[(0.025,0.025,0.025),(0.025,0.075,0.075),
        (0.075,0.075,0.025),(0.075,0.075,0.275)];
    let mut observations=witnessed_observations(&points,0.05).as_ref().clone();
    let mut layers=std::collections::BTreeMap::new();
    for (key,z) in [([0,0,0],0.025),([0,1,1],0.075)] {
        let local=local_key(key);let origin=[(f64::from(key[0])+0.5)*0.05,(f64::from(key[1])+0.5)*0.05,z];
        layers.insert(local,LayerObservation{members:vec![local],normal:[0.,0.,1.],plane_origin:origin,
            samples:vec![LayerSample{source: Default::default(), point:origin,source_sequence:1,source_timestamp_ns:100,point_index:0}]});
    }
    observations.replace_ground_layers([0,0,0],Arc::new(layers)).unwrap();
    let base=recent_fixture(0.1,"no_recent",1).with_observations(Some(Arc::new(observations)));
    // Consecutive fine Ys share coarse XY but have different measured heights.
    // A high return at the second XY is contact there, then the unknown slice
    // must still reject; accidentally reusing the first scalar changes witness.
    let quantum=f64::from(base.source_voxel_size_m());
    let live=base.with_recent_obstacle_points(1,100,0,&[0,1,2],
        [(0,[0.025,0.025,0.025]),(1,[0.025,0.075,0.12]),(2,[0.075,0.025,0.1])]).unwrap();
    let body=[0.05,0.05,0.45];let q=[0.,0.,0.,1.];let mut old=None;let mut new=None;
    let expected=live.reference_recent_body_pose(body,q,&mut old);
    assert_eq!(live.check_body_pose(body,q,&mut new),expected);assert_eq!(new,old);
    assert!(matches!(new,Some(BodyRejection::Occupancy{source:OccupancySource::Recent,cell,..})
        if cell==[1,0,(0.1/quantum).floor() as i32]));
    let polygon=PolygonQuery::new(&[[0.001,0.001],[0.099,0.001],[0.099,0.099],[0.001,0.099]]);
    for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
        let map=base.clone().with_ground_evidence(evidence);
        for z in [0.4,0.45,0.55] {for pitch in [0.0_f64,0.08] {
            let body=[0.05,0.05,z];let r=rotation([0.,(pitch/2.).sin(),0.,(pitch/2.).cos()]).unwrap();
            let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
            for ground in [None,Some((0,0,0))] {
                let original=map.fine_ground_columns(ground,&polygon,&prism,body,r[2],0,0).unwrap();
                let heights=map.fine_ground_values::<FineGroundHeightRow>(ground,&polygon,&prism,body,r[2],0,0).unwrap();
                assert_eq!(original.iter().map(|r|(r.xy,r.height.map(f64::to_bits))).collect::<Vec<_>>(),
                    heights.iter().map(|r|(r.xy,r.height.map(f64::to_bits))).collect::<Vec<_>>());
                assert!(heights.iter().any(|r|r.xy==[1,0]&&r.height.is_none()));
                if evidence==GroundEvidence::MeasuredSurface && ground.is_some() {
                    assert_eq!(heights.iter().find(|r|r.xy==[0,0]).unwrap().height,Some(0.025));
                    assert_eq!(heights.iter().find(|r|r.xy==[0,1]).unwrap().height,Some(0.075));
                }
            }
        }}
    }
}

#[test]
fn recent_query_reuse_is_local_to_pose_and_source() {
    for coarse in [0.05,0.075,0.1] {
        let map=recent_fixture(coarse,"fallback",8);
        let q=[0.,0.,0.,1.];
        for position in [[0.005,0.005,0.45],[-0.025,0.025,0.45],[0.08,0.01,0.45]] {
            let mut cases=vec![map.clone(),map.clear_recent_obstacles()];
            cases.push(map.with_observations(None));
            cases.push(map.with_observations(Some(witnessed_observations(&[(0.025,0.025,0.325)],0.05))));
            cases.push(map.with_recent_obstacle_points(9,900,0,&[],[(0,[0.025,0.025,0.45])]).unwrap());
            for case in cases {
                let mut old_map=case.clone();old_map.body_query_cache=Default::default();
                let mut new_map=case;new_map.body_query_cache=Default::default();
                for offset in [0.,0.001,0.,-0.001] {
                    let p=[position[0]+offset,position[1],position[2]];
                    let mut old=None;let mut new=None;
                    assert_eq!(new_map.check_body_pose(p,q,&mut new),old_map.reference_recent_body_pose(p,q,&mut old));
                    assert_eq!(new,old);
                }
            }
        }
    }
}


#[test]
fn recent_query_preserves_first_frame_witness_across_a_reused_coarse_column() {
    let map=recent_fixture(0.1,"no_recent",1)
        .with_recent_obstacle_points(1,100,1000,&[0],[(0,[0.025,0.025,0.04])]).unwrap()
        .with_recent_obstacle_points(2,200,1000,&[],[(0,[0.075,0.025,0.425]),(1,[0.075,0.025,0.475])]).unwrap()
        .with_recent_obstacle_points(3,300,1000,&[],[(0,[-0.075,0.025,0.425])]).unwrap();
    let body=[0.005,0.005,0.45];let q=[0.,0.,0.,1.];
    let mut old=None;let mut new=None;
    assert_eq!(map.check_body_pose(body,q,&mut new),map.reference_recent_body_pose(body,q,&mut old));
    assert_eq!(new,old);
    assert!(matches!(new,Some(BodyRejection::Occupancy{source:OccupancySource::Recent,cell:[1,0,8],..})),"{new:?}");
}

#[test]
fn recent_query_preserves_real_current_overlay_and_cleared_voxels() {
    let mut points:Vec<_>=(-40..40).flat_map(|x|(-30..30).flat_map(move|y|(-1..3).map(move|z|
        ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05)))).collect();
    points.push((0.025,0.025,0.325));
    let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.;
    let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
    envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
    let source=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
        .with_observations(Some(witnessed_observations(&points,0.05)));
    let body=[0.,0.,0.4];let q=[0.,0.,0.,1.];
    assert!(source.with_current_body_support(body,q).is_err(),"real obstacle is never cleared");
    let view=source.clone();
    assert!(view.observations.as_ref().unwrap().get([0,0,2]).is_some());
    for obstacle in [false,true] {
        let point=if obstacle{[0.025,0.025,0.325]}else{[0.025,0.025,0.025]};
        let map=view.with_recent_obstacle_points(1,100,0,if obstacle{&[]}else{&[0]},[(0,point)]).unwrap();
        for dx in [0.,0.001,0.05] {
            let p=[dx,0.,0.4];let mut old=None;let mut new=None;
            assert_eq!(map.check_body_pose(p,q,&mut new),map.reference_recent_body_pose(p,q,&mut old));
            assert_eq!(new,old);
            if obstacle {assert!(matches!(new,Some(BodyRejection::Occupancy{source:OccupancySource::Recent,..})));}
        }
    }
}
