// 仅由显式实录基准包含；逐帧计时、终态规范导出和摘要不进入生产路径。
use crate::{persistent_occupancy::PersistentOccupancyGrid, pointcloud::CloudMapShmWriter};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    fs::File,
    io::{BufWriter, Read, Write},
    path::{Path, PathBuf},
    time::Instant,
};

pub(super) fn process_usage() -> (f64, u64) {
    let mut usage: libc::rusage = unsafe { std::mem::zeroed() };
    assert_eq!(unsafe { libc::getrusage(libc::RUSAGE_SELF, &mut usage) }, 0);
    let ms = |t: libc::timeval| t.tv_sec as f64 * 1e3 + t.tv_usec as f64 / 1e3;
    (
        (ms(usage.ru_utime) + ms(usage.ru_stime)),
        usage.ru_maxrss as u64,
    )
}

pub(super) struct FrameRecord {
    pub frame: usize,
    pub source_timestamp_ns: u64,
    pub source_points: usize,
    pub cumulative_source_points: u64,
    pub wall_ms: f64,
    pub cpu_ms: f64,
    pub classify_ms: f64,
    pub occupancy_ms: f64,
    pub fine_ms: f64,
    pub dense_ms: f64,
    pub visible_ms: f64,
    pub fine_revoke_ms: f64,
    pub dense_revoke_ms: f64,
    pub dense_append_ms: f64,
    pub visible_revoke_ms: f64,
    pub classify_cpu_ms: f64,
    pub occupancy_cpu_ms: f64,
    pub cloud_cpu_ms: f64,
    pub fine_blocks: Option<usize>,
    pub dense_points: u64,
    pub visible_points: usize,
    pub new_free_cells: usize,
    pub occupancy_cells: usize,
    pub dense_index_bytes: usize,
    pub fine_index_bytes: usize,
    pub visible_index_bytes: usize,
    pub fine_blocks_read: usize,
    pub dense_coordinate_pages_read: usize,
    pub dense_candidate_points: usize,
    pub dense_removed_points: usize,
    pub dense_index_pages_read: usize,
    pub peak_rss_kib: u64,
    pub dense_storage_memory: [usize; 5],
}

fn distribution(values: impl Iterator<Item = f64>) -> Value {
    let mut values: Vec<_> = values.collect();
    assert!(!values.is_empty());
    values.sort_by(f64::total_cmp);
    let at = |q: f64| values[((values.len() - 1) as f64 * q).ceil() as usize];
    json!({"count":values.len(),"mean":values.iter().sum::<f64>()/values.len() as f64,
        "p50":at(0.5),"p99":at(0.99),"max":values.last().unwrap()})
}

fn stage_distributions(records: &[FrameRecord]) -> Value {
    json!({
        "frame_wall_ms":distribution(records.iter().map(|r|r.wall_ms)),
        "frame_cpu_ms":distribution(records.iter().map(|r|r.cpu_ms)),
        "classify_wall_ms":distribution(records.iter().map(|r|r.classify_ms)),
        "fine_wall_ms":distribution(records.iter().map(|r|r.fine_ms)),
        "dense_wall_ms":distribution(records.iter().map(|r|r.dense_ms)),
        "visible_wall_ms":distribution(records.iter().map(|r|r.visible_ms)),
        "occupancy_wall_ms":distribution(records.iter().map(|r|r.occupancy_ms)),
        "fine_revoke_wall_ms":distribution(records.iter().map(|r|r.fine_revoke_ms)),
        "dense_revoke_wall_ms":distribution(records.iter().map(|r|r.dense_revoke_ms)),
        "dense_append_wall_ms":distribution(records.iter().map(|r|r.dense_append_ms)),
        "visible_revoke_wall_ms":distribution(records.iter().map(|r|r.visible_revoke_ms)),
        "classify_cpu_ms":distribution(records.iter().map(|r|r.classify_cpu_ms)),
        "occupancy_cpu_ms":distribution(records.iter().map(|r|r.occupancy_cpu_ms)),
        "cloud_cpu_ms":distribution(records.iter().map(|r|r.cloud_cpu_ms))})
}

fn io_distributions(records: &[FrameRecord]) -> Value {
    let stats = |field: fn(&FrameRecord) -> usize| {
        let mut value = distribution(records.iter().map(|r| field(r) as f64));
        value["total"] = json!(records.iter().map(|r| field(r) as u64).sum::<u64>());
        value
    };
    json!({"fine_blocks_read":stats(|r|r.fine_blocks_read),
        "dense_coordinate_pages_read":stats(|r|r.dense_coordinate_pages_read),
        "dense_candidate_points":stats(|r|r.dense_candidate_points),
        "dense_removed_points":stats(|r|r.dense_removed_points),
        "dense_index_pages_read":stats(|r|r.dense_index_pages_read)})
}

pub(super) struct Recorder {
    root: PathBuf,
    csv: BufWriter<File>,
    records: Vec<FrameRecord>,
    started: Instant,
    capture: bool,
    preloaded_input_peak_rss_kib: u64,
    frame_audit: Option<BufWriter<File>>,
}
impl Recorder {
    pub(super) fn from_env(capture: bool, multiple_captures: bool) -> Option<Self> {
        let root = std::env::var_os("NUWAX_G3_EVIDENCE_DIR")?;
        let mut root = PathBuf::from(root);
        if multiple_captures {
            root.push(if capture { "capture_on" } else { "capture_off" });
        }
        std::fs::create_dir_all(&root).unwrap();
        let mut csv = BufWriter::new(File::create(root.join("逐帧.csv")).unwrap());
        writeln!(csv,"frame,source_timestamp_ns,source_points,cumulative_source_points,wall_ms,cpu_ms,classify_wall_ms,occupancy_wall_ms,fine_wall_ms,dense_wall_ms,visible_wall_ms,fine_revoke_wall_ms,dense_revoke_wall_ms,dense_append_wall_ms,visible_revoke_wall_ms,classify_cpu_ms,occupancy_cpu_ms,cloud_cpu_ms,fine_blocks,dense_points,visible_points,new_free_cells,occupancy_cells,dense_index_bytes,fine_index_bytes,visible_index_bytes,fine_blocks_read,dense_coordinate_pages_read,dense_candidate_points,dense_removed_points,dense_index_pages_read,peak_rss_kib").unwrap();
        Some(Self {
            root: root.clone(),
            csv,
            records: Vec::new(),
            started: Instant::now(),
            capture,
            preloaded_input_peak_rss_kib: process_usage().1,
            frame_audit: std::env::var_os("NUWAX_G3_KEY_AUDIT").map(|_| {
                let mut file = BufWriter::new(File::create(root.join("逐帧撤销审计.bin")).unwrap());
                file.write_all(b"G3FAUD1\0").unwrap();
                file
            }),
        })
    }
    pub(super) fn audit_enabled(&self) -> bool {
        self.frame_audit.is_some()
    }
    pub(super) fn audit_frame(
        &mut self,
        frame_index: usize,
        timestamp_ns: u64,
        frame: &crate::dynamic_backend::MappedFrame,
        removed: &[[i32; 3]],
    ) {
        let Some(file) = self.frame_audit.as_mut() else {
            return;
        };
        // 仅显式量化批次记录现有帧编码及真正删掉的精细键，位于帧计时之后。
        let encoded = frame.encode().unwrap();
        file.write_all(&(frame_index as u32).to_le_bytes()).unwrap();
        file.write_all(&timestamp_ns.to_le_bytes()).unwrap();
        file.write_all(&(encoded.len() as u32).to_le_bytes())
            .unwrap();
        file.write_all(&(removed.len() as u32).to_le_bytes())
            .unwrap();
        file.write_all(&encoded).unwrap();
        for key in removed {
            for value in key {
                file.write_all(&value.to_le_bytes()).unwrap();
            }
        }
    }
    pub(super) fn push(&mut self, row: FrameRecord) {
        let fine_blocks = row.fine_blocks.map_or_else(String::new, |v| v.to_string());
        writeln!(self.csv,"{},{},{},{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            row.frame,row.source_timestamp_ns,row.source_points,row.cumulative_source_points,
            row.wall_ms,row.cpu_ms,row.classify_ms,row.occupancy_ms,row.fine_ms,row.dense_ms,row.visible_ms,
            row.fine_revoke_ms,row.dense_revoke_ms,row.dense_append_ms,row.visible_revoke_ms,
            row.classify_cpu_ms,row.occupancy_cpu_ms,row.cloud_cpu_ms,fine_blocks,row.dense_points,
            row.visible_points,row.new_free_cells,row.occupancy_cells,row.dense_index_bytes,
            row.fine_index_bytes,row.visible_index_bytes,row.fine_blocks_read,row.dense_coordinate_pages_read,
            row.dense_candidate_points,row.dense_removed_points,row.dense_index_pages_read,row.peak_rss_kib).unwrap();
        if row.frame % 100 == 0 {
            self.csv.flush().unwrap();
        }
        self.records.push(row);
    }
    pub(super) fn finish(
        mut self,
        cloud: &mut CloudMapShmWriter,
        persistent: Option<&PersistentOccupancyGrid>,
        source_timestamp_ns: u64,
        requested_frames: usize,
    ) {
        self.csv.flush().unwrap();
        assert!(!self.records.is_empty());
        let final_dense_index_allocated_disk_bytes = cloud.revocation_index_disk_bytes();
        let dense_storage_disk = cloud.dense_storage_disk_bytes();
        let dense_directory = cloud.dense_directory_stats();
        let storage_memory_names = [
            "page_table_directory",
            "page_visibility_metadata",
            "bloom_filters",
            "block_directory_estimated",
            "live_keys_estimated",
        ];
        let mut storage_memory_peak = serde_json::Map::new();
        let mut storage_memory_final = serde_json::Map::new();
        for (index, name) in storage_memory_names.iter().enumerate() {
            storage_memory_peak.insert(
                (*name).into(),
                json!(self
                    .records
                    .iter()
                    .map(|r| r.dense_storage_memory[index])
                    .max()
                    .unwrap()),
            );
            storage_memory_final.insert(
                (*name).into(),
                json!(self.records.last().unwrap().dense_storage_memory[index]),
            );
        }
        let before_export = process_usage();
        let audit_started = Instant::now();
        let key_audit = if let Some(file) = self.frame_audit.as_mut() {
            file.flush().unwrap();
            let path = self.root.join("完整原键审计.bin");
            cloud.export_dense_key_audit(&path).unwrap();
            Some(
                json!({"keys":artifact(&path, dense_storage_disk[0]/12,Some(25)),
                "frames":artifact(&self.root.join("逐帧撤销审计.bin"),self.records.len() as u64,None),
                "format":"每已存点25字节：xyz原f32位模式u32×3、降f32前i32×3体素键、u8当前可见性，含已撤销点；顺序与原追加文件相同"}),
            )
        } else {
            None
        };
        let key_audit_export_wall_ms = if key_audit.is_some() {
            audit_started.elapsed().as_secs_f64() * 1e3
        } else {
            0.0
        };
        let after_audit = process_usage();
        let export_started = Instant::now();
        let maps = export_maps(&self.root, cloud, persistent, source_timestamp_ns);
        let export_wall_ms = export_started.elapsed().as_secs_f64() * 1e3;
        let after_export = process_usage();
        let params = crate::dynamic_backend::online_params();
        let segments:Vec<_>=self.records.chunks(100).map(|chunk| {
            let last=chunk.last().unwrap();
            json!({"first_frame":chunk[0].frame,"last_frame":last.frame,"timing":stage_distributions(chunk),"io_counts":io_distributions(chunk),
                "dense_points":last.dense_points,"visible_points":last.visible_points,"fine_blocks":last.fine_blocks,
                "occupancy_cells":last.occupancy_cells,"dense_index_bytes":last.dense_index_bytes,
                "fine_index_bytes":last.fine_index_bytes,"visible_index_bytes":last.visible_index_bytes,
                "new_free_cells":chunk.iter().map(|r|r.new_free_cells).sum::<usize>(),
                "dense_storage_memory_peak_bytes":storage_memory_names.iter().enumerate().map(|(i,n)|((*n).to_string(),json!(chunk.iter().map(|r|r.dense_storage_memory[i]).max().unwrap()))).collect::<serde_json::Map<String,Value>>()})
        }).collect();
        let summary = json!({
            "backend":format!("{:?}",crate::dynamic_backend::backend()),
            "reference_bbox":crate::dynamic_backend::reference_revoke(),
            "capture":self.capture,"requested_frames":requested_frames,"frames":self.records.len(),
            "source_points":self.records.last().unwrap().cumulative_source_points,
            "recorded_path":std::env::var("NUWAX_PGO_RECORDED_SCANS").unwrap(),
            "sensor_origin_body":std::env::var("NUWAX_PGO_SENSOR_ORIGIN_BODY").unwrap(),
            "online_params":{"resolution":params.resolution,"unknown":params.inflate_unknown,
                "hits":params.inflate_hits_dist,"window_frames":params.sliding_window_size,
                "compensation":params.inflate_unknown_compensation},
            "timing":stage_distributions(&self.records),"io_counts":io_distributions(&self.records),"segments_100_frames":segments,
            "peak_rss_before_export_kib":before_export.1,"peak_rss_with_export_kib":after_export.1,
            "preloaded_input_peak_rss_kib":self.preloaded_input_peak_rss_kib,
            "final_dense_index_allocated_disk_bytes":final_dense_index_allocated_disk_bytes,
            "dense_storage_disk_bytes":{"xyz_logical":dense_storage_disk[0],"keys_logical":dense_storage_disk[1],"new_derived_logical":dense_storage_disk[2],
                "xyz_allocated":dense_storage_disk[3],"keys_allocated":dense_storage_disk[4],"new_derived_allocated":dense_storage_disk[5]},
            "dense_storage_memory_peak_bytes":storage_memory_peak,"dense_storage_memory_final_bytes":storage_memory_final,
            "dense_directory_terminal":{"level":dense_directory[0],"entries":dense_directory[1],"last_frame_compactions":dense_directory[2],
                "last_frame_compaction_entries":dense_directory[3],"last_frame_compaction_peak_bytes_estimated":dense_directory[4]},
            "key_audit":key_audit,"key_audit_export_wall_ms":key_audit_export_wall_ms,
            "key_audit_export_cpu_ms":after_audit.0-before_export.0,
            "peak_index_bytes":{"dense":self.records.iter().map(|r|r.dense_index_bytes).max().unwrap(),
                "fine":self.records.iter().map(|r|r.fine_index_bytes).max().unwrap(),
                "visible":self.records.iter().map(|r|r.visible_index_bytes).max().unwrap(),
                "total":self.records.iter().map(|r|r.dense_index_bytes+r.fine_index_bytes+r.visible_index_bytes).max().unwrap()},
            "export_wall_ms":export_wall_ms,
            "export_cpu_ms":after_export.0-after_audit.0,
            "whole_instrumented_run_s":self.started.elapsed().as_secs_f64(),"maps":maps,
            "clock_scope":"整帧含分类、二维计算/持久融合、精细、完整、可见；不含实录预读、当前帧变换/来源构建、CSV/规模统计和终态提交导出",
            "input_memory_scope":"沿用原recorded_input预读方式，进程RSS包含完整实录冻结输入；另报新增索引字节",
            "quantile_rule":"对排序后的实测值取ceil((N-1)*q)位置，不插值"});
        std::fs::write(
            self.root.join("汇总.json"),
            serde_json::to_vec_pretty(&summary).unwrap(),
        )
        .unwrap();
        eprintln!(
            "ONLINE_EVIDENCE root={} frames={} peak_rss_kib={}",
            self.root.display(),
            self.records.len(),
            after_export.1
        );
    }
}

fn artifact(path: &Path, records: u64, record_bytes: Option<usize>) -> Value {
    let mut file = File::open(path).unwrap();
    let mut hash = Sha256::new();
    let mut buffer = vec![0u8; 1024 * 1024];
    loop {
        let size = file.read(&mut buffer).unwrap();
        if size == 0 {
            break;
        }
        hash.update(&buffer[..size]);
    }
    json!({"file":path.file_name().unwrap().to_str().unwrap(),"bytes":file.metadata().unwrap().len(),
        "records":records,"record_bytes":record_bytes,"sha256":format!("{:x}",hash.finalize())})
}

fn export_maps(
    root: &Path,
    cloud: &mut CloudMapShmWriter,
    persistent: Option<&PersistentOccupancyGrid>,
    stamp: u64,
) -> Value {
    // 同一测量进程结束后只提交一次，获取真正冻结的精细、完整与可见来源。
    cloud.bind_fine_map_owner(1, 1).unwrap();
    let receipt = crate::mapping_commit::OccupancyCommitReceipt {
        commit_id: crate::mapping_commit::MappingCommitId {
            boot_nonce: 1,
            sequence: 1,
        },
        session_id: "online-cost".into(),
        session_generation: 1,
        occupancy_revision: 1,
        source_timestamp_ns: stamp,
        backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
        backend_generation: 1,
    };
    cloud
        .commit_accumulated_snapshot(&receipt)
        .unwrap()
        .unwrap();
    let fine = cloud
        .committed_fine_cloud()
        .expect("实录终态精细源应可提交");
    let manifest = fine.manifest();
    let fine_path = root.join("精细占据.bin");
    let ground_path = root.join("精细地面记录.bin");
    let mut fine_file = BufWriter::new(File::create(&fine_path).unwrap());
    let mut ground_file = BufWriter::new(File::create(&ground_path).unwrap());
    let mut blocks: Vec<_> = manifest.blocks.iter().collect();
    blocks.sort_by_key(|r| r.key);
    for reference in blocks {
        let bytes = fine
            .repository
            .read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES)
            .unwrap();
        let block = nuwax_map_chunks::decode_block(&bytes).unwrap();
        assert_eq!(
            block.len(),
            reference.points as usize,
            "精细块点数应与冻结清单相同"
        );
        for (&local, evidence) in &block {
            let key = nuwax_map_chunks::global_key(reference.key, local).unwrap();
            for value in key {
                fine_file.write_all(&value.to_le_bytes()).unwrap();
            }
            fine_file
                .write_all(&evidence.observed_ns.to_le_bytes())
                .unwrap();
            fine_file.write_all(&evidence.hits.to_le_bytes()).unwrap();
        }
        // 去掉随机对象身份，保留全部实际占据和地面字段，按块键/局部键规范输出。
        let records = nuwax_map_chunks::decode_ground_records(&bytes).unwrap();
        let canonical = nuwax_map_chunks::encode_ground_block(&block, &records).unwrap();
        for value in reference.key {
            ground_file.write_all(&value.to_le_bytes()).unwrap();
        }
        ground_file
            .write_all(&(canonical.len() as u32).to_le_bytes())
            .unwrap();
        ground_file.write_all(&canonical).unwrap();
    }
    fine_file.flush().unwrap();
    ground_file.flush().unwrap();
    let dense = cloud
        .committed_dense_cloud()
        .unwrap()
        .expect("实录终态完整源应可提交");
    let dense_path = root.join("完整原序点云.bin");
    let mut dense_file = BufWriter::new(File::create(&dense_path).unwrap());
    let mut offset = 0;
    while offset < dense.point_count {
        let bytes = dense
            .read(offset, crate::dense_cloud::CHUNK_POINTS)
            .unwrap();
        assert!(!bytes.is_empty());
        offset += bytes.len() as u64 / 12;
        dense_file.write_all(&bytes).unwrap();
    }
    assert_eq!(offset, dense.point_count);
    dense_file.flush().unwrap();
    let visible = cloud.shared_committed_snapshot().unwrap();
    let mut visible_bits: Vec<[u32; 3]> = visible
        .xyz_f32_le
        .chunks_exact(12)
        .map(|p| {
            std::array::from_fn(|i| u32::from_le_bytes(p[i * 4..i * 4 + 4].try_into().unwrap()))
        })
        .collect();
    visible_bits.sort_unstable();
    let visible_path = root.join("可见静态点集.bin");
    let mut visible_file = BufWriter::new(File::create(&visible_path).unwrap());
    for point in visible_bits {
        for value in point {
            visible_file.write_all(&value.to_le_bytes()).unwrap();
        }
    }
    visible_file.flush().unwrap();
    let occupancy = persistent.expect("实录二维累积结果应存在");
    let occupancy_path = root.join("二维占据.bin");
    std::fs::write(
        &occupancy_path,
        occupancy
            .grid
            .data
            .iter()
            .map(|&v| v as u8)
            .collect::<Vec<_>>(),
    )
    .unwrap();
    let metadata = json!({"fine":{"voxel_size_f32_bits":manifest.voxel_size_m.to_bits(),"points":manifest.points(),
            "blocks":manifest.blocks.len(),"source_timestamp_ns":manifest.source_timestamp_ns},
        "dense":{"points":dense.point_count,"bounds_min_f32_bits":dense.bounds_min.map(f32::to_bits),
            "bounds_max_f32_bits":dense.bounds_max.map(f32::to_bits)},
        "visible":{"points":visible.point_count,"voxel_size_f32_bits":visible.voxel_size.to_bits()},
        "occupancy":{"width":occupancy.grid.width,"height":occupancy.grid.height,
            "resolution_f64_bits":occupancy.grid.resolution.to_bits(),"origin_f64_bits":occupancy.grid.origin.map(f64::to_bits),
            "orientation_f64_bits":occupancy.grid.origin_orientation.map(f64::to_bits),
            "timestamp_f64_bits":occupancy.grid.ts.to_bits(),"frame_id":occupancy.grid.frame_id}});
    std::fs::write(
        root.join("地图元数据.json"),
        serde_json::to_vec_pretty(&metadata).unwrap(),
    )
    .unwrap();
    json!({"fine":artifact(&fine_path,manifest.points(),Some(22)),
        "fine_ground_records":artifact(&ground_path,manifest.blocks.len() as u64,None),
        "dense_original_order":artifact(&dense_path,dense.point_count,Some(12)),
        "visible_canonical_bits":artifact(&visible_path,visible.point_count as u64,Some(12)),
        "occupancy":artifact(&occupancy_path,occupancy.grid.data.len() as u64,Some(1)),"metadata":metadata})
}
