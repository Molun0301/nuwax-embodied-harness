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
}

#[cfg(test)]
#[path = "surface_chunk_process_tests.rs"]
mod rpc_process_tests;
