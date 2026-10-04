//! 原表面构建任务拥有的块接收、可靠补取和有界磁盘缓存。
use nuwax_codec::lcm::schema;
use nuwax_map_chunks::{Manifest, Receiver, RegionReader, Repository, MAX_BLOCK_BYTES};
use std::sync::Arc;
use std::time::{Duration, Instant};

pub(super) struct FineMapReader {
    receiver: Receiver,
    reader: RegionReader,
    client: Option<nuwax_rpc::RpcClient>,
    lcm_url: String,
}
impl FineMapReader {
    /// 按真实内容摘要刷新观察块；几何未变化的时间更新也必须交付。
    pub(super) fn observations(
        &mut self,
        root: &Manifest,
        previous: Option<(&Manifest, &Arc<nuwax_nav_mls_planner::SurfaceObservations>)>,
    ) -> Result<Arc<nuwax_nav_mls_planner::SurfaceObservations>, String> {
        use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
        use std::collections::BTreeMap;
        let previous = previous.filter(|(old, _)| old.domain == root.domain
            && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits());
        if let Some((_, grid)) = previous.filter(|(old, _)| *old == root) {
            return Ok(Arc::clone(grid));
        }
        let mut grid = match previous {
            Some((_, grid)) => grid.as_ref().clone(),
            None => SurfaceObservations::new(root.voxel_size_m, 32).map_err(|e| e.to_string())?,
        };
        let old: BTreeMap<_, _> = previous.into_iter().flat_map(|(r, _)| &r.blocks).map(|b| (b.key, b)).collect();
        let new: BTreeMap<_, _> = root.blocks.iter().map(|b| (b.key, b)).collect();
        for key in old.keys().filter(|key| !new.contains_key(*key)) {
            grid.replace_block(*key, BTreeMap::new()).map_err(|e| e.to_string())?;
        }
        for reference in &root.blocks {
            if old.get(&reference.key).is_some_and(|old| old.sha256 == reference.sha256) { continue; }
            let block = self.reader.block(reference)?;
            let mut values = BTreeMap::new();
            for (&local, evidence) in block.iter() {
                if evidence.observed_ns > root.source_timestamp_ns { return Err("精细观察晚于提交根".into()); }
                values.insert(nuwax_map_chunks::global_key(reference.key, local)?,
                    SurfaceObservation { observed_ns: evidence.observed_ns, hits: evidence.hits });
            }
            grid.replace_block(reference.key, values).map_err(|e| e.to_string())?;
        }
        grid.set_source_timestamp_ns(root.source_timestamp_ns);
        // 同一提交根的全部块已经读取并验证，缺少占据块不等于缺少已提交的几何源。
        grid.mark_complete_geometry_source();
        Ok(Arc::new(grid))
    }

    pub(super) fn new(lcm_url: String) -> Result<Self, String> {
        let repository = Repository::temporary()?;
        Ok(Self {
            receiver: Receiver::new(repository.clone()),
            reader: RegionReader::new(repository, 32 * 1024 * 1024),
            client: None,
            lcm_url,
        })
    }
    pub(super) fn receive(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<Arc<Manifest>, String> {
        let transfer = message.fine.as_ref().ok_or("消息缺少精细根")?;
        let candidate = Manifest::decode(&transfer.root, transfer.root_sha256)?;
        if let Some(current) = self.receiver.current() {
            if current.as_ref() == &candidate {
                return Ok(current);
            }
        }
        let expected = schema::navigation_map_3d_domain(message);
        let deadline = Instant::now() + Duration::from_secs(2);
        let client = &mut self.client;
        let url = &self.lcm_url;
        let mut fetched_blocks=0_usize;
        let received=self.receiver.receive(transfer, &expected, |hash| {
            fetched_blocks+=1;
            if client.is_none() {
                *client = Some(nuwax_rpc::RpcClient::new(
                    Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(url).map_err(|e| e.to_string())?),
                    "GeometryRuntimeService/get_navigation_map_block",
                ).map_err(|e| e.to_string())?);
            }
            let runtime = tokio::runtime::Handle::try_current().map_err(|_| "可靠补取必须在原受管阻塞构建任务中执行")?;
            let request = serde_json::json!({"sha256_hex": hash.iter().map(|v| format!("{v:02x}")).collect::<String>()});
            let mut last = "精细块补取期限耗尽".to_string();
            for _ in 0..3 {
                let Some(remaining) = deadline.checked_duration_since(Instant::now()) else { break; };
                let result = runtime.block_on(client.as_ref().unwrap().call_scoped((vec![request.clone()], Default::default()), remaining.min(Duration::from_millis(350))));
                match result {
                    Ok(value) => {
                        let hex = value.get("data_hex").and_then(|v| v.as_str()).ok_or("精细块响应格式错误")?;
                        if hex.len() > MAX_BLOCK_BYTES * 2 || hex.len() % 2 != 0 || !hex.is_ascii() { return Err("精细块响应长度错误".into()); }
                        return (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).map_err(|_| "精细块响应编码错误".into())).collect();
                    }
                    Err(error) => last = error.to_string(),
                }
            }
            Err(last)
        })?;
        if fetched_blocks>0 {tracing::debug!(fetched_blocks,blocks=received.blocks.len(),revision=received.revision,"精细地图可靠补取完成");}
        Ok(received)
    }
    /// 只展开几何变化块涉及的完整规划列；上下叠层和跨块列共同聚合。
    pub(super) fn columns(
        &mut self,
        root: &Manifest,
        previous: Option<&Manifest>,
        planner_size: f32,
    ) -> Result<(Vec<nuwax_nav_mls_planner::SurfaceColumnUpdate>, usize), String> {
        use std::collections::{BTreeMap, BTreeSet};
        if !planner_size.is_finite() || planner_size < root.voxel_size_m {
            return Err("规划精度小于精细源精度".into());
        }
        let compatible = previous.filter(|old| {
            old.domain == root.domain && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits()
        });
        let old: BTreeMap<_, _> = compatible
            .into_iter()
            .flat_map(|r| &r.blocks)
            .map(|b| (b.key, b))
            .collect();
        let new: BTreeMap<_, _> = root.blocks.iter().map(|b| (b.key, b)).collect();
        let mut dirty = BTreeSet::new();
        for key in old.keys().chain(new.keys()) {
            if old.get(key).map(|b| b.geometry_sha256) == new.get(key).map(|b| b.geometry_sha256) {
                continue;
            }
            let (min, max) = planning_columns(*key, root.voxel_size_m, planner_size)?;
            for x in min[0]..=max[0] {
                for y in min[1]..=max[1] {
                    dirty.insert([x, y]);
                }
            }
            if dirty.len() > 2_000_000 {
                return Err("脏列超过规划工作集预算".into());
            }
        }
        let mut columns: BTreeMap<[i32; 2], BTreeMap<i32, [f32; 2]>> = BTreeMap::new();
        let mut read_points = 0;
        for reference in &root.blocks {
            let (min, max) = planning_columns(reference.key, root.voxel_size_m, planner_size)?;
            if !dirty
                .range([min[0], i32::MIN]..=[max[0], i32::MAX])
                .any(|c| c[1] >= min[1] && c[1] <= max[1])
            {
                continue;
            }
            let block = self.reader.block(reference)?;
            read_points += block.len();
            for (&local, evidence) in block.iter() {
                if evidence.observed_ns > root.source_timestamp_ns {
                    return Err("精细证据晚于提交根".into());
                }
                let key = nuwax_map_chunks::global_key(reference.key, local)?;
                let xyz = key.map(|v| ((f64::from(v) + 0.5) * f64::from(root.voxel_size_m)) as f32);
                let cell = xyz.map(|v| (v / planner_size).floor() as i32);
                let column = [cell[0], cell[1]];
                if !dirty.contains(&column) {
                    continue;
                }
                // 与完整点输入使用相同的精细高度量化，避免浮点边界出现两套语义。
                let fine = (f64::from(xyz[2]) / f64::from(root.voxel_size_m)).floor();
                let low = (fine * f64::from(root.voxel_size_m)) as f32;
                let high = ((fine + 1.0) * f64::from(root.voxel_size_m)) as f32;
                columns
                    .entry(column)
                    .or_default()
                    .entry(cell[2])
                    .and_modify(|b| {
                        b[0] = b[0].min(low);
                        b[1] = b[1].max(high);
                    })
                    .or_insert([low, high]);
            }
        }
        let updates = dirty
            .into_iter()
            .map(|column| nuwax_nav_mls_planner::SurfaceColumnUpdate {
                column,
                cells: columns
                    .remove(&column)
                    .unwrap_or_default()
                    .into_iter()
                    .collect(),
            })
            .collect();
        Ok((updates, read_points))
    }
}

fn planning_columns(
    key: [i32; 3],
    source: f32,
    planner: f32,
) -> Result<([i32; 2], [i32; 2]), String> {
    let first = nuwax_map_chunks::global_key(key, 0)?;
    let last = nuwax_map_chunks::global_key(key, 32767)?;
    let convert = |v| {
        let point = ((f64::from(v) + 0.5) * f64::from(source)) as f32;
        let column = (point / planner).floor();
        if !column.is_finite() || column.abs() >= i32::MAX as f32 * 0.5 {
            return Err("精细块规划坐标越界".to_owned());
        }
        Ok(column as i32)
    };
    Ok((
        [convert(first[0])?, convert(first[1])?],
        [convert(last[0])?, convert(last[1])?],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_map_chunks::*;
    use std::collections::{BTreeMap, BTreeSet};

    fn install(
        repository: &Repository,
        cells: &BTreeSet<Key>,
        size: f32,
        revision: u64,
    ) -> Manifest {
        let mut blocks = BTreeMap::<Key, Block>::new();
        for &key in cells {
            blocks.entry(block_key(key)).or_default().insert(
                local_key(key),
                Evidence {
                    observed_ns: revision,
                    hits: 1,
                },
            );
        }
        let blocks = blocks
            .into_iter()
            .map(|(key, block)| {
                let bytes = encode_block(&block).unwrap();
                let sha256 = digest(&bytes);
                repository.put_object(sha256, &bytes).unwrap();
                BlockRef {
                    key,
                    sha256,
                    geometry_sha256: block_geometry_digest(&block),
                    points: block.len() as u32,
                }
            })
            .collect();
        Manifest {
            version: 1,
            domain: Domain {
                session: "test".into(),
                generation: 1,
                frame_generation: 1,
                producer_generation: 1,
            },
            revision,
            source_timestamp_ns: revision,
            voxel_size_m: size,
            blocks,
        }
    }
    fn reader(repository: Repository) -> FineMapReader {
        FineMapReader {
            receiver: Receiver::new(repository.clone()),
            reader: RegionReader::new(repository, 1024 * 1024),
            client: None,
            lcm_url: String::new(),
        }
    }
    fn assert_columns(
        updates: &[nuwax_nav_mls_planner::SurfaceColumnUpdate],
        cells: &BTreeSet<Key>,
        source: f32,
        planner: f32,
    ) {
        // 独立遍历全部源点作为参照，包含负坐标与不能整除的块边缘。
        let mut reference = BTreeMap::<[i32; 2], BTreeMap<i32, [f32; 2]>>::new();
        for key in cells {
            let p = key.map(|v| ((f64::from(v) + 0.5) * f64::from(source)) as f32);
            let c = p.map(|v| (v / planner).floor() as i32);
            let fine = (f64::from(p[2]) / f64::from(source)).floor();
            let bounds = [
                (fine * f64::from(source)) as f32,
                ((fine + 1.0) * f64::from(source)) as f32,
            ];
            reference
                .entry([c[0], c[1]])
                .or_default()
                .entry(c[2])
                .and_modify(|b| {
                    b[0] = b[0].min(bounds[0]);
                    b[1] = b[1].max(bounds[1]);
                })
                .or_insert(bounds);
        }
        for update in updates {
            assert_eq!(
                update.cells,
                reference
                    .remove(&update.column)
                    .unwrap_or_default()
                    .into_iter()
                    .collect::<Vec<_>>()
            );
        }
    }
    #[test]
    fn changed_blocks_merge_vertical_and_non_aligned_neighbors_and_keep_deletions() {
        let repository = Repository::temporary().unwrap();
        let mut reader = reader(repository.clone());
        let mut cells: BTreeSet<_> = (-70..70)
            .flat_map(|x| (-12..12).flat_map(move |y| [[x, y, 0], [x, y, 50]]))
            .collect();
        for size in [0.1, 0.13] {
            let first = install(&repository, &cells, 0.05, 1);
            let (updates, read) = reader.columns(&first, None, size).unwrap();
            assert_eq!(read, cells.len());
            assert_columns(&updates, &cells, 0.05, size);
            cells.remove(&[-32, 0, 0]);
            cells.insert([-33, 0, 8]);
            let second = install(&repository, &cells, 0.05, 2);
            let (updates, read) = reader.columns(&second, Some(&first), size).unwrap();
            assert!(read < cells.len());
            assert_columns(&updates, &cells, 0.05, size);
            let third = install(&repository, &cells, 0.05, 3);
            let (updates, read) = reader.columns(&third, Some(&second), size).unwrap();
            assert!(updates.is_empty());
            assert_eq!(read, 0);
            cells.retain(|key| block_key(*key) != [-2, 0, 0]);
            let fourth = install(&repository, &cells, 0.05, 4);
            let (updates, _) = reader.columns(&fourth, Some(&third), size).unwrap();
            assert_columns(&updates, &cells, 0.05, size);
            assert!(updates.iter().any(|u| u.cells.is_empty()));
        }
    }
    #[test]
    fn complete_fine_manifest_resolves_a_real_door_across_storage_blocks() {
        use nuwax_nav_mls_planner::{SurfaceMapBuilder,SurfaceBodyEnvelope,mls_planner::Config};
        let capture=std::env::var("NEH_DOOR_CAPTURE_PATH").ok();
        let repository=Repository::temporary().unwrap();let mut reader=reader(repository.clone());
        let fine=f64::from(0.01_f32);let width=32.0*fine;
        let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,
            surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
            wall_buffer_m:0.4,wall_buffer_weight:2.0,max_slope_deg:18.0,step_threshold_m:0.0,
            min_step_tread_depth_m:0.2,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0};
        let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.3,-width/2.0],[0.3,-width/2.0],
            [0.3,width/2.0],[-0.3,width/2.0]],z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.4,
            max_step_up_m:0.0,max_step_down_m:0.0,max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let mut builder=SurfaceMapBuilder::configured(config,Some(envelope));
        let mut previous:Option<(Manifest,Arc<nuwax_nav_mls_planner::SurfaceObservations>)>=None;
        for (revision,gap) in [(1,33),(2,32)] {
            let mut keys:BTreeSet<Key>=(-100..100).flat_map(|x|(-50..100).map(move|y|[x,y,-1])).collect();
            keys.extend((0..4).flat_map(|x|(-50..100).filter(move|&y|y<0||y>=gap)
                .flat_map(move|y|(0..80).map(move|z|[x,y,z]))));
            let root=install(&repository,&keys,0.01,revision);
            let observations=reader.observations(&root,previous.as_ref().map(|(root,grid)|(root,grid))).unwrap();
            let (updates,count)=reader.columns(&root,previous.as_ref().map(|(root,_)|root),0.1).unwrap();
            let built=builder.build_column_updates(&updates,0.01,previous.is_none(),count).unwrap();
            let map=built.snapshot.with_observations(Some(Arc::clone(&observations)));
            let y=gap as f64*fine/2.0;let q=[0.0,0.0,0.0,1.0];
            let result=map.validate_body_motion(([-0.5,y,0.4],q),([0.5,y,0.4],q),&[]);
            assert_eq!(result.is_ok(),gap==33,"门宽必须决定通行，存储块边缘不能改变它：{gap} {result:?}");
            if gap==33 {
                use nuwax_nav_runtime::{ExecutionMap,ExecutionPath,NavPlanner,PlannerStep,SurfaceFollower,SurfaceFollowerConfig};
                use nuwax_nav_types_messages::{GoalOptions,MotionLimits,MotionModel,NavFrameId,Path3D,Point2,Point3,Pose3,PoseStamped,Quaternion,Timestamp,Twist2};
                let map=Arc::new(map);
                // 未对准门中线时，先证明完整前向参照，再验直接全局请求。
                let point=|x,y|([x,y,0.4],[0.0,0.0,0.0,1.0]);
                let reference=[point(-0.65,0.30),point(-0.35,y),point(0.39,y),point(0.65,0.30)];
                let motion=MotionLimits {model:MotionModel::Omnidirectional,max_forward_mps:0.3,
                    max_reverse_mps:0.3,max_lateral_mps:0.3,max_angular_rps:0.8,
                    linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
                    angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,
                    reaction_time_sec:0.05,minimum_turning_radius_m:0.0,can_rotate_in_place:true};
                let options=GoalOptions {allow_reverse:true,position_tolerance_m:0.003,yaw_tolerance_rad:0.01};
                for pair in reference.windows(2) {
                    let segment=crate::surface_oriented::plan(&map,pair[0],pair[1],options,Some(&motion),&[],0)
                        .expect("非对齐窄门必须先有真实可执行的前向参照");
                    for edge in segment.windows(2) {map.validate_body_motion(edge[0],edge[1],&[]).unwrap();}
                }
                let route=crate::surface_oriented::plan(&map,reference[0],reference[3],options,Some(&motion),&[],1000)
                    .expect("未预先对准的真实可过窄门须找到完整路线");
                for edge in route.windows(2) {map.validate_body_motion(edge[0],edge[1],&[]).unwrap();}
                for (heading,aligned) in [(0.0_f64,true),(5.0_f64.to_radians(),true),(0.0,false)] {
                    let body=|x:f64|([x,if aligned {y+(x-2.0*fine)*heading.tan()} else {0.30},0.4],[0.0,0.0,(heading/2.0).sin(),(heading/2.0).cos()]);
                    let stamped=|body:([f64;3],[f64;4]),time:i64|PoseStamped {frame_id:NavFrameId::Map,
                        stamp:Timestamp::from_unix(time/1_000_000_000,(time%1_000_000_000) as u32),
                        pose:Pose3 {position:Point3::new(body.0[0],body.0[1],body.0[2]),
                            orientation:Quaternion{x:body.1[0],y:body.1[1],z:body.1[2],w:body.1[3]}}};
                    let motion=MotionLimits {model:MotionModel::Omnidirectional,max_forward_mps:0.3,
                        max_reverse_mps:0.3,max_lateral_mps:0.3,max_angular_rps:0.8,
                        linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
                        angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,
                        reaction_time_sec:0.05,minimum_turning_radius_m:0.0,can_rotate_in_place:true};
                    let options=GoalOptions {allow_reverse:true,position_tolerance_m:0.03,yaw_tolerance_rad:0.05};
                    let end_x=if aligned {0.5}else{0.65};
                    let mut current=body(-end_x);let goal=body(end_x);let mut time=1_000_000_000;
                    let route=crate::surface_oriented::plan(&map,current,goal,options,Some(&motion),&[],1000).unwrap();
                    let mut config=SurfaceFollowerConfig::new(0.4);config.controller.speed=0.3;
                    config.controller.control_frequency=20.0;config.controller.simulation=true;
                    config.motion_limits=Some(motion);config.max_angular_speed_rps=0.8;
                    let mut follower=SurfaceFollower::new(config).unwrap();
                    follower.configure_goal(&stamped(goal,time),options).unwrap();
                    follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
                    follower.update_odometry(stamped(current,time));
                    follower.replace_execution_path(ExecutionPath::Surface(Path3D {
                        poses:route.iter().copied().map(|p|stamped(p,time)).collect()}),None,stamped(goal,time)).unwrap();
                    let mut applied=Twist2::ZERO;let mut reached=false;
                    let mut inputs=Vec::new();
                    for tick in 0..240 {
                        if capture.is_some() {inputs.push((current,[applied.linear.x,applied.linear.y,applied.angular_z],time));}
                        let mut arriving=false;
                        let command=match follower.step(0.32) {
                            PlannerStep::Command(command)=>command,
                            PlannerStep::Succeeded=>{arriving=true;Twist2::ZERO},
                            other=>{
                                if let Some(destination)=&capture {
                                    let envelope=map.body_envelope().unwrap();
                                    let cfg=map.config();
                                    let body_bits=|p:([f64;3],[f64;4])|(p.0.map(f64::to_bits),p.1.map(f64::to_bits));
                                    let config_json=serde_json::json!({
                                        "world_frame":cfg.world_frame,"voxel_size":cfg.voxel_size,
                                        "robot_height":cfg.robot_height,"surface_closing_radius":cfg.surface_closing_radius,
                                        "node_spacing_m":cfg.node_spacing_m,"wall_clearance_m":cfg.wall_clearance_m,
                                        "wall_buffer_m":cfg.wall_buffer_m,"wall_buffer_weight":cfg.wall_buffer_weight,
                                        "max_slope_deg":cfg.max_slope_deg,"step_threshold_m":cfg.step_threshold_m,
                                        "min_step_tread_depth_m":cfg.min_step_tread_depth_m,
                                        "step_penalty_weight":cfg.step_penalty_weight,"goal_tolerance":cfg.goal_tolerance,
                                        "viz_publish_hz":cfg.viz_publish_hz});
                                    let config_bits=serde_json::json!({
                                        "voxel_size":cfg.voxel_size.to_bits(),"robot_height":cfg.robot_height.to_bits(),
                                        "surface_closing_radius":cfg.surface_closing_radius.to_bits(),
                                        "node_spacing_m":cfg.node_spacing_m.to_bits(),"wall_clearance_m":cfg.wall_clearance_m.to_bits(),
                                        "wall_buffer_m":cfg.wall_buffer_m.to_bits(),"wall_buffer_weight":cfg.wall_buffer_weight.to_bits(),
                                        "max_slope_deg":cfg.max_slope_deg.to_bits(),"step_threshold_m":cfg.step_threshold_m.to_bits(),
                                        "min_step_tread_depth_m":cfg.min_step_tread_depth_m.to_bits(),
                                        "step_penalty_weight":cfg.step_penalty_weight.to_bits(),"goal_tolerance":cfg.goal_tolerance.to_bits(),
                                        "viz_publish_hz":cfg.viz_publish_hz.to_bits()});
                                    let body_scalars=[envelope.z_bounds_m[0],envelope.z_bounds_m[1],
                                        envelope.body_to_support_z_m,envelope.max_step_up_m,envelope.max_step_down_m,
                                        envelope.max_cross_slope_deg,envelope.support_uncertainty_m];
                                    let recipe=serde_json::json!({"gap":gap,"revision":revision,
                                        "fine_resolution_bits":map.source_voxel_size_m().to_bits(),"keys_count":keys.len(),
                                        "config":config_json,"config_float_bits":config_bits,
                                        "footprint_xy_m":&envelope.footprint_xy_m,
                                        "footprint_xy_bits":envelope.footprint_xy_m.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>(),
                                        "body_scalars":body_scalars,"body_scalar_bits":body_scalars.map(f64::to_bits)});
                                    let mut record=serde_json::json!({
                                        "schema":"dense-doorway-input-trace-v1", "heading":heading,
                                        "aligned":aligned,"tick":tick,"time_ns":time,
                                        "current":current,"applied":[applied.linear.x,applied.linear.y,applied.angular_z],
                                        "goal":goal,"progress_m":follower.progress_m(),"outcome":format!("{other:?}"),
                                        "simulation":true,"control_frequency":20.0,"speed":0.3,
                                        "allow_reverse":options.allow_reverse,
                                        "position_tolerance_m":options.position_tolerance_m,
                                        "yaw_tolerance_rad":options.yaw_tolerance_rad,"source_recipe":recipe});
                                    record["route"]=serde_json::to_value(&route).unwrap();
                                    record["inputs"]=serde_json::to_value(&inputs).unwrap();
                                    record["current_bits"]=serde_json::to_value(body_bits(current)).unwrap();
                                    record["goal_bits"]=serde_json::to_value(body_bits(goal)).unwrap();
                                    record["applied_bits"]=serde_json::to_value([applied.linear.x,applied.linear.y,applied.angular_z].map(f64::to_bits)).unwrap();
                                    record["route_bits"]=serde_json::to_value(route.iter().map(|p|body_bits(*p)).collect::<Vec<_>>()).unwrap();
                                    record["input_bits"]=serde_json::to_value(inputs.iter().map(|(p,a,t)|(body_bits(*p),(*a).map(f64::to_bits),*t)).collect::<Vec<_>>()).unwrap();
                                    std::fs::write(destination,serde_json::to_vec_pretty(&record).unwrap()).unwrap();
                                    eprintln!("DOOR_CAPTURE path={destination} aligned={aligned} heading={heading:?} tick={tick} route_len={}",route.len());
                                }
                                panic!("真实窄门闭环不能卡住：角度{heading} aligned={aligned} tick={tick} {other:?} {current:?}");
                            },
                        };
                        assert!(command.linear.x>=0.0 && command.linear.y==0.0,"窄门也不能横走倒退");
                        let next_applied=Twist2 {linear:Point2::new(applied.linear.x+(command.linear.x-applied.linear.x).clamp(-0.05,0.05),0.0),
                            angular_z:applied.angular_z+(command.angular_z-applied.angular_z).clamp(-0.1,0.1)};
                        let mean=Twist2 {linear:Point2::new((applied.linear.x+next_applied.linear.x)*0.5,0.0),
                            angular_z:(applied.angular_z+next_applied.angular_z)*0.5};
                        let yaw=2.0*current.1[2].atan2(current.1[3]);
                        let next=nuwax_nav_control::integrate(nuwax_nav_control::PlanarPose{x:current.0[0],y:current.0[1],yaw},mean,0.05);
                        let actual=([next.x,next.y,0.4],[0.0,0.0,(next.yaw/2.0).sin(),(next.yaw/2.0).cos()]);
                        map.validate_body_motion(current,actual,&[]).expect("实际积分机体不能碰到门框");
                        current=actual;applied=next_applied;time+=50_000_000;
                        follower.observe_applied_command(applied);follower.update_odometry(stamped(current,time));
                        // 跟随器只报告规划到达，模拟执行器仍真实制动到停稳后确认。
                        if arriving && applied.linear.x.abs()<0.005 && applied.angular_z.abs()<0.01
                            && (current.0[0]-goal.0[0]).hypot(current.0[1]-goal.0[1])<options.position_tolerance_m {
                            reached=true;break;
                        }
                    }
                    assert!(reached && current.0[0]>0.46 && applied.linear.x.abs()<0.01,
                        "完整机体须通过门框并停稳：角度{heading}，到达{reached}，位姿{current:?}，速度{applied:?}");
                }
            }
            previous=Some((root,observations));
        }
    }

    #[test]
    fn fixed_dirty_region_reads_same_points_when_far_map_grows() {
        let repository = Repository::temporary().unwrap();
        let mut reader = reader(repository.clone());
        let mut reads = Vec::new();
        for far in [64, 512, 2048] {
            let mut cells: BTreeSet<_> = (0..far)
                .flat_map(|x| (0..8).map(move |y| [x, y, 0]))
                .collect();
            let first = install(&repository, &cells, 0.05, 1);
            cells.insert([10, 2, 2]);
            let next = install(&repository, &cells, 0.05, 2);
            let (_, count) = reader.columns(&next, Some(&first), 0.1).unwrap();
            reads.push(count);
        }
        assert_eq!(reads, vec![257, 257, 257]);
    }
include!("surface_chunks_doorway_diagnostics.rs");

}

#[cfg(test)]
#[path = "surface_chunk_process_tests.rs"]
mod rpc_process_tests;
