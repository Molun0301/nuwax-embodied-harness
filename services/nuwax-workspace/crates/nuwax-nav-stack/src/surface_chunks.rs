//! 原表面构建任务拥有的块接收、对象补取和有界磁盘缓存。
use nuwax_codec::lcm::schema;
use nuwax_map_chunks::{Manifest, Receiver, Repository, MAX_BLOCK_BYTES, MAX_BLOCK_TRANSFER_PART_BYTES};
use std::sync::Arc;
use std::time::Duration;

pub(super) struct FineMapReader {
    receiver: Receiver,
    client: Option<nuwax_rpc::RpcClient>,
    lcm_url: String,
}
impl FineMapReader {
    /// 按块证据修订刷新观察块；几何未变化的时间更新也必须交付。
    pub(super) fn observations(
        &mut self,
        root: &Manifest,
        previous: Option<(&Manifest, &Arc<nuwax_nav_mls_planner::SurfaceObservations>)>,
    ) -> Result<Arc<nuwax_nav_mls_planner::SurfaceObservations>, String> {
        self.observations_with_changes(root, previous).map(|(grid, _)| grid)
    }

    /// 按块修订替换观测，变化列相对兼容旧根。
    pub(super) fn observations_with_changes(
        &mut self,
        root: &Manifest,
        previous: Option<(&Manifest, &Arc<nuwax_nav_mls_planner::SurfaceObservations>)>,
    ) -> Result<(Arc<nuwax_nav_mls_planner::SurfaceObservations>, Option<Vec<[i32; 2]>>), String> {
        nuwax_nav_mls_planner::surface_chunks::observations_with_changes(
            &mut self.receiver, root, previous,
        )
    }

    pub(super) fn new(lcm_url: String) -> Result<Self, String> {
        let repository = Repository::temporary()?;
        Ok(Self {
            receiver: Receiver::with_cache_budget(repository, 32 * 1024 * 1024),
            client: None,
            lcm_url,
        })
    }
    pub(super) fn receive(
        &mut self,
        message: &schema::NavigationMap3D,
    ) -> Result<Arc<Manifest>, String> {
        let transfer = message.fine.as_ref().ok_or("消息缺少精细根")?;
        // Receiver must also inspect included bytes on duplicate publications:
        // an identical root cannot hide an immutable-object conflict.
        let expected = schema::navigation_map_3d_domain(message);
        let client = &mut self.client;
        let url = &self.lcm_url;
        let mut fetched_blocks=0_usize;
        let received=self.receiver.receive(transfer, &expected, |_, reference| {
            fetched_blocks+=1;
            if client.is_none() {
                *client = Some(nuwax_rpc::RpcClient::new(
                    Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(url).map_err(|e| e.to_string())?),
                    "GeometryRuntimeService/get_navigation_map_block",
                ).map_err(|e| e.to_string())?);
            }
            let runtime = tokio::runtime::Handle::try_current().map_err(|_| "可靠补取必须在原受管阻塞构建任务中执行")?;
            runtime.block_on(fetch_block_parts(client.as_ref().unwrap(), reference))
        })?;
        if fetched_blocks>0 {tracing::debug!(fetched_blocks,blocks=received.blocks.len(),revision=received.revision,"精细地图可靠补取完成");}
        Ok(received)
    }
    #[cfg(test)]
    fn columns(
        &mut self,
        root: &Manifest,
        previous: Option<(&Manifest, &Arc<nuwax_nav_mls_planner::SurfaceObservations>)>,
        planner_size: f32,
    ) -> Result<(Vec<nuwax_nav_mls_planner::SurfaceColumnUpdate>, usize, Arc<nuwax_nav_mls_planner::SurfaceObservations>), String> {
        let (grid, changes) = self.observations_with_changes(root, previous)?;
        let (updates, count) = nuwax_nav_mls_planner::surface_chunks::column_updates(
            root, previous.map(|(root, _)| root), planner_size, &grid, changes.as_deref(),
        );
        Ok((updates, count, grid))
    }

}

/// 同一对象最多四段并发；每段只请求一次，超时为350毫秒。
async fn fetch_block_parts(client: &nuwax_rpc::RpcClient,
    reference: &nuwax_map_chunks::BlockRef) -> Result<Vec<u8>, String> {
    async fn part(client: &nuwax_rpc::RpcClient, request: serde_json::Value) -> Result<(Vec<u8>, usize), String> {
        let reply = client.call_scoped((vec![request.clone()], Default::default()),
            Duration::from_millis(350)).await.map_err(|e| e.to_string())?;
        let data = block_response(&request, &reply)?;
        Ok((data, reply["total_bytes"].as_u64().expect("分段格式已检查") as usize))
    }
    let request = block_request(reference);
    let (mut joined, total) = part(client, request.clone()).await?;
    while joined.len() < total {
        let requests = (joined.len()..total).step_by(MAX_BLOCK_TRANSFER_PART_BYTES).take(4).map(|offset| {
            let mut request = request.clone();
            request["offset"] = serde_json::json!(offset);
            part(client, request)
        });
        for (bytes, length) in futures::future::try_join_all(requests).await? {
            if length != total { return Err("精细块分段总长变化".into()); }
            joined.extend_from_slice(&bytes);
        }
    }
    Ok(joined)
}

pub(super) fn block_request(reference: &nuwax_map_chunks::BlockRef) -> serde_json::Value {
    let nuwax_map_chunks::ObjectIdentity::Assigned(id) = reference.object;
    serde_json::json!({"object_id": id.0, "offset": 0})
}

fn block_response(request: &serde_json::Value, response: &serde_json::Value) -> Result<Vec<u8>, String> {
    use base64::{engine::general_purpose::STANDARD, Engine as _};
    let offset = request["offset"].as_u64().expect("本地分段偏移") as usize;
    let total = response["total_bytes"].as_u64().filter(|&v| v <= MAX_BLOCK_BYTES as u64)
        .ok_or("精细块响应总长错误")? as usize;
    let expected = total.saturating_sub(offset).min(MAX_BLOCK_TRANSFER_PART_BYTES);
    let compressed = response.get("data_lz4_base64");
    let encoded = compressed.or_else(|| response.get("data_base64")).and_then(serde_json::Value::as_str)
        .ok_or("精细块响应格式错误")?;
    let data = STANDARD.decode(encoded).map_err(|e| e.to_string())?;
    let data = if compressed.is_some() { nuwax_codec::lcm::map_block_part::decompress(&data, expected)? } else { data };
    if data.len() != expected { return Err("精细块响应长度错误".into()); }
    Ok(data)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;
    use nuwax_map_chunks::*;
    use std::collections::{BTreeMap, BTreeSet};
    include!("surface_layer_transfer_tests.rs");
    include!("surface_object_fetch_tests.rs");
    include!("surface_input_capture.rs");

    // Fresh-root compatibility for existing doorway diagnostic recipes.
    #[allow(dead_code)]
    fn install(repository: &Repository, cells: &BTreeSet<Key>, size: f32, revision: u64) -> Manifest {
        install_revisioned(repository, cells, size, revision, None)
    }

    fn install_revisioned(
        repository: &Repository,
        cells: &BTreeSet<Key>,
        size: f32,
        revision: u64,
        previous: Option<&Manifest>,
    ) -> Manifest {
        let mut writer = Writer::new(repository.clone(), size, 4 * 1024 * 1024).unwrap();
        if let Some(previous) = previous {
            writer.restore(previous).unwrap();
            let mut reader = RegionReader::new(repository.clone(), 4 * 1024 * 1024);
            let removed: Vec<_> = previous.blocks.iter().flat_map(|reference| {
                reader.block(reference).unwrap().keys().map(|&local|
                    global_key(reference.key, local).unwrap()).collect::<Vec<_>>()
            }).filter(|key| !cells.contains(key)).collect();
            writer.remove(removed).unwrap();
        }
        writer.observe(cells.iter().copied(), revision).unwrap();
        writer.snapshot(Domain {
            session: "test".into(), }, revision, revision).unwrap().as_ref().clone()
    }
    fn reader(repository: Repository) -> FineMapReader {
        FineMapReader {
            receiver: Receiver::with_cache_budget(repository, 1024 * 1024),
            client: None,
            lcm_url: String::new(),
        }
    }
    fn transfer_message(repository: &Repository, root: &Manifest) -> schema::NavigationMap3D {
        schema::NavigationMap3D {
            session_id: root.domain.session.clone(),

            fine: Some(Transfer {
                root: root.encode().unwrap(),
                included: root.blocks.iter().map(|block| (block.object,
                    repository.read_object(block.object, MAX_BLOCK_BYTES).unwrap())).collect(),
            }),
            ..Default::default()
        }
    }

    #[test]
    fn duplicate_root_still_rejects_conflicting_included_body_without_installing_it() {
        let repository = Repository::temporary().unwrap();
        let root = install_revisioned(&repository, &BTreeSet::from([[0, 0, 0]]), 0.05, 1, None);
        let mut receiver = FineMapReader::new(String::new()).unwrap();
        let message = transfer_message(&repository, &root);
        let committed = receiver.receive(&message).unwrap();
        assert!(Arc::ptr_eq(&receiver.receive(&message).unwrap(), &committed));
        let mut conflicting = message.clone();
        let bytes = conflicting.fine.as_mut().unwrap().included.values_mut().next().unwrap();
        let mut block = decode_block(bytes).unwrap();
        block.values_mut().next().unwrap().hits += 1;
        *bytes = encode_block(&block).unwrap();
        assert!(receiver.receive(&conflicting).unwrap_err().to_string().contains("identity conflicts"));
        assert!(Arc::ptr_eq(&receiver.receiver.current().unwrap(), &committed));
        let observations = receiver.observations(&committed, None).unwrap();
        assert_eq!(observations.get([0, 0, 0]).unwrap().hits, 1);
    }

    #[test]
    fn observations_and_columns_reuse_admitted_block_arcs() {
        let repository = Repository::temporary().unwrap();
        let cells: BTreeSet<_> = (0..64).flat_map(|x| (0..64).map(move |y| [x, y, 0])).collect();
        let root = install_revisioned(&repository, &cells, 0.05, 1, None);
        let message = transfer_message(&repository, &root);
        let mut reader = FineMapReader::new(String::new()).unwrap();
        let admitted = reader.receive(&message).unwrap();
        // 与计时测试分开，避免额外缓存访问改变其操作顺序。
        let blocks: Vec<_> = admitted.blocks.iter()
            .map(|reference| reader.receiver.block(reference).unwrap()).collect();
        let observations = reader.observations(&admitted, None).unwrap();
        let (updates, count, _) = reader.columns(&admitted, None, 0.1).unwrap();
        assert_eq!(count, cells.len());
        assert_columns(&updates, &cells, 0.05, 0.1);
        for &cell in &cells {
            assert_eq!(observations.get(cell).unwrap().observed_ns, 1);
            assert_eq!(observations.get(cell).unwrap().hits, 1);
        }
        let repeated = reader.receive(&message).unwrap();
        assert!(Arc::ptr_eq(&admitted, &repeated));
        for (reference, block) in admitted.blocks.iter().zip(&blocks) {
            assert!(Arc::ptr_eq(block, &reader.receiver.block(reference).unwrap()));
        }
        assert!(reader.client.is_none());
    }

    #[test]
    fn evidence_revision_refreshes_observations_without_dirtying_geometry() {
        let repository = Repository::temporary().unwrap();
        let cells: BTreeSet<_> = (0..64).flat_map(|x| (0..64).map(move |y| [x, y, 0])).collect();
        let first = install_revisioned(&repository, &cells, 0.05, 1, None);
        let second = install_revisioned(&repository, &cells, 0.05, 2, Some(&first));
        let first_message = transfer_message(&repository, &first);
        let second_message = transfer_message(&repository, &second);
        let mut included_bytes = Vec::new();
        for (revision, root, message) in [(1, &first, &first_message), (2, &second, &second_message)] {
            let transfer = message.fine.as_ref().unwrap();
            assert_eq!(root.revision, revision);
            assert_eq!(root.source_timestamp_ns, revision);
            assert_eq!(root.points(), 4096);
            assert_eq!(root.blocks.len(), 4);
            assert!(root.blocks.iter().all(|b| matches!(b.object, ObjectIdentity::Assigned(_))));
            let objects: BTreeSet<_> = root.blocks.iter().map(|b| b.object).collect();
            assert_eq!(objects.len(), root.blocks.len());
            assert_eq!(objects, transfer.included.keys().copied().collect::<BTreeSet<_>>());
            let bytes: usize = transfer.included.values().map(Vec::len).sum();
            assert_eq!(bytes, 49_216);
            assert!(bytes <= 128 * 1024);
            included_bytes.push(bytes);
        }
        let mut reader = FineMapReader::new(String::new()).unwrap();
        assert!(reader.client.is_none());
        let mut timings_us = Vec::new();
        let start = Instant::now();
        let first = reader.receive(&first_message).unwrap();
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        let start = Instant::now();
        let old = reader.observations(&first, None).unwrap();
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        let start = Instant::now();
        let (initial_updates, initial_count) = nuwax_nav_mls_planner::surface_chunks::column_updates(&first, None, 0.1, &old, None);
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        assert_eq!(initial_count, cells.len());
        assert!(!initial_updates.is_empty());
        assert_columns(&initial_updates, &cells, 0.05, 0.1);
        let start = Instant::now();
        let second = reader.receive(&second_message).unwrap();
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        let start = Instant::now();
        let (next, changes) = reader.observations_with_changes(&second, Some((&first, &old))).unwrap();
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        assert_eq!(first.geometry_revision, second.geometry_revision);
        assert!(first.blocks.iter().zip(&second.blocks).all(|(a, b)|
            a.block_revision < b.block_revision && a.geometry_revision == b.geometry_revision));
        let start = Instant::now();
        let (updates, count) = nuwax_nav_mls_planner::surface_chunks::column_updates(&second, Some(&first), 0.1, &next, changes.as_deref());
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        assert!(updates.is_empty());
        assert_eq!(count, 0);
        for &cell in &cells {
            assert_eq!(old.get(cell).unwrap().observed_ns, 1);
            assert_eq!(old.get(cell).unwrap().hits, 1);
            assert_eq!(next.get(cell).unwrap().observed_ns, 2);
            assert_eq!(next.get(cell).unwrap().hits, 2);
        }
        assert!(!Arc::ptr_eq(&old, &next));
        let start = Instant::now();
        let repeated = reader.receive(&second_message).unwrap();
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        assert!(Arc::ptr_eq(&second, &repeated));
        let start = Instant::now();
        let repeated_observations = reader.observations(&repeated, Some((&second, &next))).unwrap();
        timings_us.push(start.elapsed().as_micros());
        assert!(reader.client.is_none());
        assert!(Arc::ptr_eq(&next, &repeated_observations));
        eprintln!("inline fine cells={} blocks={} included_bytes={included_bytes:?} stages=receive1,observations1,columns1,receive2,observations2,columns2,repeat_receive2,repeat_observations2 operation_us={timings_us:?} columns_read_points=[{initial_count},{count}] client_initialized={}",
            cells.len(), second.blocks.len(), reader.client.is_some());
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
        let cells: BTreeSet<_> = (-70..70)
            .flat_map(|x| (-12..12).flat_map(move |y| [[x, y, 0], [x, y, 50]]))
            .collect();
        for size in [0.1, 0.13] {
            let mut cells = cells.clone();
            let first = install_revisioned(&repository, &cells, 0.05, 1, None);
            let (updates, read, first_observations) = reader.columns(&first, None, size).unwrap();
            assert_eq!(read, cells.len());
            assert_columns(&updates, &cells, 0.05, size);
            cells.remove(&[-32, 0, 0]);
            cells.insert([-33, 0, 8]);
            let second = install_revisioned(&repository, &cells, 0.05, 2, Some(&first));
            let (updates, read, second_observations) = reader.columns(&second, Some((&first, &first_observations)), size).unwrap();
            assert!(read < cells.len());
            assert_columns(&updates, &cells, 0.05, size);
            let third = install_revisioned(&repository, &cells, 0.05, 3, Some(&second));
            let (updates, read, third_observations) = reader.columns(&third, Some((&second, &second_observations)), size).unwrap();
            assert!(updates.is_empty());
            assert_eq!(read, 0);
            let mut previous_layers = BTreeMap::<[i32; 2], BTreeSet<i32>>::new();
            for key in &cells {
                let p = key.map(|v| ((f64::from(v) + 0.5) * f64::from(0.05_f32)) as f32);
                let c = p.map(|v| (v / size).floor() as i32);
                previous_layers.entry([c[0], c[1]]).or_default().insert(c[2]);
            }
            cells.retain(|key| block_key(*key) != [-2, 0, 0]);
            let fourth = install_revisioned(&repository, &cells, 0.05, 4, Some(&third));
            let (updates, _, fourth_observations) = reader.columns(&fourth, Some((&third, &third_observations)), size).unwrap();
            assert_columns(&updates, &cells, 0.05, size);
            assert!(!updates.is_empty());
            assert!(updates.iter().any(|u| u.cells.len() < previous_layers[&u.column].len()));
            cells.retain(|key| block_key(*key) != [-2, 0, 1]);
            let fifth = install_revisioned(&repository, &cells, 0.05, 5, Some(&fourth));
            let (updates, _, _) = reader.columns(&fifth, Some((&fourth, &fourth_observations)), size).unwrap();
            assert_columns(&updates, &cells, 0.05, size);
            assert!(updates.iter().any(|u| u.cells.is_empty()));
        }
    }
    #[test]
    fn complete_fine_manifest_resolves_a_real_door_across_storage_blocks() {
        use nuwax_nav_mls_planner::{SurfaceMapBuilder,SurfaceBodyEnvelope,mls_planner::Config};
        // 直接核对合成门框实体与真实矩形机身，避免旧地面投影成为新算法的二次门禁。
        fn body_intersects_wall(body:([f64;3],[f64;4]),width:f64,gap:f64,fine:f64)->bool {
            let yaw=2.*body.1[2].atan2(body.1[3]);let (sn,cs)=yaw.sin_cos();
            let body_xy:[[_;2];4]=[[-0.3,-width/2.],[0.3,-width/2.],[0.3,width/2.],[-0.3,width/2.]]
                .map(|p|[body.0[0]+cs*p[0]-sn*p[1],body.0[1]+sn*p[0]+cs*p[1]]);
            [[-0.5,0.],[gap,1.]].into_iter().any(|[lo,hi]| {
                let wall=[[0.,lo],[4.*fine,lo],[4.*fine,hi],[0.,hi]];
                [[1.,0.],[0.,1.],[cs,sn],[-sn,cs]].into_iter().all(|axis| {
                    let range=|poly:&[[f64;2];4]|poly.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|mut b,p| {
                        let v=p[0]*axis[0]+p[1]*axis[1];b[0]=b[0].min(v);b[1]=b[1].max(v);b});
                    let a=range(&body_xy);let b=range(&wall);a[1]>=b[0]&&b[1]>=a[0]
                })
            })
        }
        let capture=std::env::var("NEH_DOOR_CAPTURE_PATH").ok();
        let repository=Repository::temporary().unwrap();let mut reader=reader(repository.clone());
        let fine=f64::from(0.01_f32);let width=32.0*fine;
        let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,
            surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
            wall_buffer_m:0.4,wall_buffer_weight:2.0,max_slope_deg:18.0,step_threshold_m:0.0,
            min_step_tread_depth_m:0.2,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0};
        let envelope=SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.3,-width/2.0],[0.3,-width/2.0],
            [0.3,width/2.0],[-0.3,width/2.0]],z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.4,
            max_step_up_m:0.0,max_step_down_m:0.0,max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let mut builder=SurfaceMapBuilder::configured(config,Some(envelope));
        let mut previous:Option<(Manifest,Arc<nuwax_nav_mls_planner::SurfaceObservations>)>=None;
        for (revision,gap) in [(1,33),(2,32)] {
            let mut keys:BTreeSet<Key>=(-100..100).flat_map(|x|(-50..100).map(move|y|[x,y,-1])).collect();
            keys.extend((0..4).flat_map(|x|(-50..100).filter(move|&y|y<0||y>=gap)
                .flat_map(move|y|(0..80).map(move|z|[x,y,z]))));
            let root=install_revisioned(&repository,&keys,0.01,revision,previous.as_ref().map(|(root,_)| root));
            let (observations,changes)=reader.observations_with_changes(&root,previous.as_ref().map(|(root,grid)|(root,grid))).unwrap();
            let (updates,count)=nuwax_nav_mls_planner::surface_chunks::column_updates(&root,previous.as_ref().map(|(root,_)|root),0.1,&observations,changes.as_deref());
            let built=builder.build_column_updates(&updates,0.01,previous.is_none(),count).unwrap();
            let map=built.snapshot.with_observations(Some(Arc::clone(&observations)));
            let y=gap as f64*fine/2.0;let q=[0.0,0.0,0.0,1.0];
            let direct_clear=(0..=300).all(|i|!body_intersects_wall(([-0.5+i as f64/300.,y,0.4],q),width,gap as f64*fine,fine));
            assert_eq!(direct_clear,gap==33,"门宽必须由真实矩形与门框相交决定：{gap}");
            if gap==33 {
                use nuwax_nav_runtime::{ExecutionMap,ExecutionPath,NavPlanner,PlannerStep,SurfaceFollower,SurfaceFollowerConfig};
                use nuwax_nav_types_messages::{GoalOptions,MotionLimits,MotionModel,NavFrameId,Path3D,Point2,Point3,Pose3,PoseStamped,Quaternion,Timestamp,Twist2};
                let map=Arc::new(map);
                // 未对准门中线时，先证明完整前向参照，再验直接全局请求。
                let point=|x,y|([x,y,0.4],[0.0,0.0,0.0,1.0]);
                let reference=[point(-0.65,0.30),point(-0.35,y),point(0.39,y),point(0.65,0.30)];
                let options=GoalOptions {allow_reverse:true,position_tolerance_m:0.003,yaw_tolerance_rad:0.01};
                for pair in reference.windows(2) {
                    let segment=crate::surface_oriented::plan(&map,pair[0],pair[1],options,0.4,&[])
                        .expect("非对齐窄门必须先有真实可执行的前向参照");
                    assert!(segment.len()>=2);
                }
                let route=crate::surface_oriented::plan(&map,reference[0],reference[3],options,0.4,&[])
                    .expect("未预先对准的真实可过窄门须找到完整路线");
                assert!(route.len()>=2);
                for (heading,aligned) in [(0.0_f64,true),(5.0_f64.to_radians(),true),(0.0_f64,false)] {
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
                    let route=crate::surface_oriented::plan(&map,current,goal,options,0.4,&[]).unwrap();
                    // 此用例验证细地图/PCT（点云地形规划器）/SCAN（局部样条规划器）的数值闭环，
                    // 显式串行驱动地图、规划、跟踪三接口；不把初始化耗时充当物理时间。
                    let native=nuwax_nav_runtime::native_map::from_surface(&map).unwrap();
                    let reference:Vec<_>=route.iter().map(|p|p.0).collect();
                    let mut config=SurfaceFollowerConfig::new(0.4).scan;
                    config.resolution=map.navigation_source_columns(None).0;
                    config.body_length=0.6;config.body_width=width;config.body_height=0.3;
                    config.body_z_min=-0.15;config.body_z_max=0.15;
                    config.max_velocity=0.3;config.max_reverse_velocity=0.3;config.max_lateral_velocity=0.3;
                    config.max_acceleration=1.;config.max_yaw_rate=0.8;config.max_yaw_acceleration=2.;
                    config.max_step_up=0.;config.max_step_down=0.;config.max_slope=18_f64.to_radians();config.sample_dt=0.05;
                    let mut follower=crate::scan_numerical_fixture::Planner::new();follower.set_static_map_fixture(true);
                    let mut request=crate::scan_numerical_fixture::PlanRequest {
                        obstacles:&native.obstacles,ground:&native.static_collision_ground,reference:&reference,
                        start_position:current.0,start_velocity:Some([0.;3]),start_acceleration:None,start_yaw:heading,
                        start_yaw_rate:Some(0.),odometry_timestamp_ns:time,config,boundary:&[],
                        forbidden_columns:&native.forbidden_columns,goal_yaw:None,
                    };
                    let mut applied=Twist2::ZERO;let mut reached=false;let mut passed_opening=false;
                    let mut inputs=Vec::new();
                    for tick in 0..240 {
                        let yaw=stamped(current,time).pose.orientation.yaw();
                        request.start_position=current.0;request.start_yaw=yaw;request.odometry_timestamp_ns=time;
                        request.start_velocity=Some([yaw.cos()*applied.linear.x-yaw.sin()*applied.linear.y,
                            yaw.sin()*applied.linear.x+yaw.cos()*applied.linear.y,0.]);
                        request.start_yaw_rate=Some(applied.angular_z);
                        if capture.is_some() {inputs.push((current,[applied.linear.x,applied.linear.y,applied.angular_z],time));}
                        let position_reached=(0..3).map(|i|(current.0[i]-goal.0[i]).powi(2)).sum::<f64>().sqrt()<options.position_tolerance_m;
                        request.goal_yaw=position_reached.then_some(heading);
                        let arriving=position_reached&&nuwax_nav_astar::controllers::angle_diff(heading,yaw).abs()<options.yaw_tolerance_rad;
                        let command=match if arriving{Ok(Twist2::ZERO)}else{follower.step(&request).map(|out|Twist2 {
                            linear:Point2::new(out.command_forward,out.command_lateral),angular_z:out.command_yaw_rate})} {
                            Ok(command)=>command,
                            Err(other)=>{
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
                        // 当前作者A保留原声明的全向通道，独立响应模型也须完整执行这些通道。
                        assert!(command.linear.x >= -motion.max_reverse_mps && command.linear.x <= motion.max_forward_mps
                            && command.linear.y.abs() <= motion.max_lateral_mps && command.angular_z.abs() <= motion.max_angular_rps,
                            "命令必须满足原声明的各轴能力：tick={tick} command={command:?}");
                        let delta=Point2::new(command.linear.x-applied.linear.x,command.linear.y-applied.linear.y);
                        let scale=(0.05/delta.x.hypot(delta.y).max(f64::MIN_POSITIVE)).min(1.);
                        let next_applied=Twist2 {linear:Point2::new(applied.linear.x+delta.x*scale,applied.linear.y+delta.y*scale),
                            angular_z:applied.angular_z+(command.angular_z-applied.angular_z).clamp(-0.1,0.1)};
                        let mean=Twist2 {linear:Point2::new((applied.linear.x+next_applied.linear.x)*0.5,(applied.linear.y+next_applied.linear.y)*0.5),
                            angular_z:(applied.angular_z+next_applied.angular_z)*0.5};
                        let yaw=2.0*current.1[2].atan2(current.1[3]);
                        let next=nuwax_nav_control::integrate(nuwax_nav_control::PlanarPose{x:current.0[0],y:current.0[1],yaw},mean,0.05);
                        let actual=([next.x,next.y,0.4],[0.0,0.0,(next.yaw/2.0).sin(),(next.yaw/2.0).cos()]);
                        let sweep=(mean.linear.x.hypot(mean.linear.y)+mean.angular_z.abs()*0.3_f64.hypot(width*0.5))*0.05;
                        let divisions=(sweep/(fine/3.)).ceil().max(1.) as usize;
                        for sample in 0..=divisions {
                            let p=nuwax_nav_control::integrate(nuwax_nav_control::PlanarPose{x:current.0[0],y:current.0[1],yaw},mean,0.05*sample as f64/divisions as f64);
                            let body=([p.x,p.y,0.4],[0.,0.,(p.yaw*0.5).sin(),(p.yaw*0.5).cos()]);
                            if p.x>=0. && p.x<=4.*fine {
                                assert!(p.y>0. && p.y<gap as f64*fine,"必须穿过原门洞：{body:?}");
                                passed_opening=true;
                            }
                            assert!(!body_intersects_wall(body,width,gap as f64*fine,fine),"实际积分机身撞门框：tick={tick} body={body:?} command={command:?}");
                        }
                        current=actual;applied=next_applied;time+=50_000_000;
                        follower.observe_applied([applied.linear.x,applied.linear.y,applied.angular_z]);
                        // 业务到达直接返回；这里独立检验积分执行器的物理制动，不增加业务确认。
                        if arriving && applied.linear.x.hypot(applied.linear.y)<0.005 && applied.angular_z.abs()<0.01
                            && (current.0[0]-goal.0[0]).hypot(current.0[1]-goal.0[1])<options.position_tolerance_m {
                            eprintln!("窄门闭环通过 heading={heading} aligned={aligned} tick={tick} pose={current:?} actual={applied:?}");
                            reached=true;break;
                        }
                    }
                    assert!(reached && passed_opening && current.0[0]>0.46 && applied.linear.x.hypot(applied.linear.y)<0.01,
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
            let first = install_revisioned(&repository, &cells, 0.05, 1, None);
            let observations = reader.observations(&first, None).unwrap();
            cells.insert([10, 2, 2]);
            let next = install_revisioned(&repository, &cells, 0.05, 2, Some(&first));
            let (_, count, _) = reader.columns(&next, Some((&first, &observations)), 0.1).unwrap();
            reads.push(count);
        }
        assert_eq!(reads, vec![257, 257, 257]);
    }

}
