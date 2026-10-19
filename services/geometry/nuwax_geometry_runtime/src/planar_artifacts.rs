//! 完整图与精细三维资产共用有界分块传输，按真实格式校验后封存。
//! 未命名私有文件由描述符拥有，关闭即释放。
use crate::proto::{CheckpointMemberIdentity, CheckpointMemberRole, GraphArtifactRef, GraphArtifactTransferRequest, GraphArtifactTransferResponse};
use nuwax_slam_2d_runtime::archive;
use std::{collections::HashMap, fs::File, io::{Read, Seek, SeekFrom, Write}, os::unix::fs::OpenOptionsExt, time::{Duration, Instant}};

pub(crate) fn assigned_id(value: &str) -> bool {
    value.len() == 32 && value.bytes().all(|v| v.is_ascii_digit() || (b'a'..=b'f').contains(&v)) && value.bytes().any(|v| v != b'0')
}
pub(crate) fn validate_member_identity(member: &CheckpointMemberIdentity) -> Result<(), String> {
    if !assigned_id(&member.checkpoint_id) || member.frame_id.is_empty() || member.frame_id.len() > 1024
        || member.frame_id.trim() != member.frame_id || member.frame_id.chars().any(char::is_control) {
        return Err("检查点成员身份或坐标系非法".into());
    }
    let role = CheckpointMemberRole::try_from(member.role).map_err(|_| "检查点成员角色未知")?;
    let occupancy = member.occupancy_receipt.as_ref().ok_or("检查点成员缺少占据收据")?;
    if occupancy.backend_kind == "slam_toolbox" {
        if !matches!(role, CheckpointMemberRole::PlanarGraph | CheckpointMemberRole::CanonicalOccupancy) { return Err("二维检查点成员角色非法".into()); }
        crate::server::validate_mapping_commit_identity(occupancy.commit_id.as_ref(), "planar member")?;
        if member.cloud_receipt.is_some() || occupancy.backend_kind != "slam_toolbox"
            || occupancy.session_id.is_empty() || occupancy.session_generation == 0 || occupancy.backend_generation == 0
            || occupancy.occupancy_revision == 0 || occupancy.source_timestamp_ns == 0
            || member.frame_generation != 0 || member.producer_generation != 0 { return Err("二维成员来源身份非法".into()); }
    } else {
        if !matches!(role, CheckpointMemberRole::BackendState | CheckpointMemberRole::CanonicalOccupancy | CheckpointMemberRole::CanonicalCloud | CheckpointMemberRole::FineMap | CheckpointMemberRole::DenseCloud)
            || !matches!(occupancy.backend_kind.as_str(), "voxel" | "raytracing") { return Err("检查点成员角色或后端非法".into()); }
        crate::server::validate_mapping_checkpoint_receipts(&occupancy.session_id, occupancy.session_generation, &occupancy.backend_kind, Some(occupancy), member.cloud_receipt.as_ref())?;
        if occupancy.source_timestamp_ns == 0 || member.cloud_receipt.as_ref().unwrap().source_timestamp_ns == 0 { return Err("检查点成员来源时间非法".into()); }
        if role == CheckpointMemberRole::FineMap {
            if member.producer_generation == 0 { return Err("精细成员缺少生产者代际".into()); }
        } else if member.frame_generation != 0 || member.producer_generation != 0 { return Err("非精细成员不能携带精细根代际".into()); }
    }
    Ok(())
}
struct Entry {
    file: File, reference: GraphArtifactRef, written: u64, sealed: bool, touched: Instant,
    #[cfg(test)] validation_runs: u32,
}
#[derive(Default)]
pub(crate) struct Store { entries: HashMap<String, Entry> }
impl Store {
    fn collect(&mut self) { self.entries.retain(|_, e| e.touched.elapsed() < Duration::from_secs(600)); }
    pub(crate) fn validate_reference(reference: &GraphArtifactRef) -> Result<(), String> {
        if reference.size == 0 || reference.size > crate::dense_cloud::MAX_ARCHIVE_BYTES { return Err("制品大小非法".into()); }
        match reference.identity_schema_version {
            0 | 1 => {
                if reference.id.is_empty() || reference.id.len() > 128 || reference.sha256.len() != 32
                    || reference.sha256.iter().all(|v| *v == 0) || reference.member_identity.is_some() { return Err("旧制品身份或混合代际非法".into()); }
            }
            2 => {
                if !assigned_id(&reference.id) || !reference.sha256.is_empty() { return Err("制品新身份混入旧摘要或格式非法".into()); }
                let member = reference.member_identity.as_ref().ok_or("制品缺少成员身份")?;
                validate_member_identity(member)?;
                if !matches!(CheckpointMemberRole::try_from(member.role), Ok(CheckpointMemberRole::FineMap | CheckpointMemberRole::DenseCloud | CheckpointMemberRole::PlanarGraph)) { return Err("可传输制品角色非法".into()); }
                if member.role == CheckpointMemberRole::FineMap as i32 && reference.size > nuwax_map_chunks::MAX_ARCHIVE_BYTES { return Err("精细制品超过容量".into()); }
            }
            _ => return Err("不支持的制品身份代际".into()),
        }
        Ok(())
    }
    /// Legacy-only producer adapter. Never use to wrap an AR2/AR3/AR4/AR5 archive.
    pub fn insert_legacy(&mut self, mut file: File, size: u64, sha: [u8; 32]) -> Result<GraphArtifactRef, String> {
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        let mut magic = [0; 8]; file.read_exact(&mut magic).map_err(|e| e.to_string())?;
        if matches!(&magic, b"NUWAXAR2" | b"NUWAXAR3" | b"NUWAXAR4" | b"NUWAXAR5" | b"NWS2D002") { return Err("AR2/AR3/AR4/AR5/NWS2D002编号档案不能包装为旧摘要制品".into()); }
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        if crate::dense_cloud::hash_reader(&mut file)? != (size, sha) { return Err("旧制品导入摘要变化".into()); }
        self.admit_file(file, GraphArtifactRef { id: uuid::Uuid::new_v4().simple().to_string(), size, sha256: sha.to_vec(), ..Default::default() })
    }
    /// Takes ownership of the already frozen creator file; no identity hash/copy/full-body scan.
    pub fn insert(&mut self, mut file: File, size: u64, member: CheckpointMemberIdentity) -> Result<GraphArtifactRef, String> {
        let reference = GraphArtifactRef { id: uuid::Uuid::new_v4().simple().to_string(), size, sha256: Vec::new(), identity_schema_version: 2, member_identity: Some(member) };
        Self::validate_reference(&reference)?;
        Self::validate_archive_binding(&mut file, &reference, false)?;
        self.admit_file(file, reference)
    }
    fn admit_file(&mut self, mut file: File, reference: GraphArtifactRef) -> Result<GraphArtifactRef, String> {
        self.collect(); Self::validate_reference(&reference)?;
        let metadata = file.metadata().map_err(|e| e.to_string())?;
        if !metadata.is_file() || metadata.len() != reference.size { return Err("冻结制品实际文件大小非法".into()); }
        if self.entries.len() >= 4 { return Err("传输制品数量达到上限".into()); }
        if self.entries.contains_key(&reference.id) { return Err("新制品身份已存在".into()); }
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        self.entries.insert(reference.id.clone(), Entry { file, written: reference.size, reference: reference.clone(), sealed: true, touched: Instant::now(), #[cfg(test)] validation_runs: 0 });
        Ok(reference)
    }
    pub fn open(&mut self, reference: &GraphArtifactRef) -> Result<File, String> {
        self.collect(); let e = self.entry(reference)?;
        if !e.sealed { return Err("上传制品未封存".into()); }
        File::open(format!("/proc/self/fd/{}", std::os::fd::AsRawFd::as_raw_fd(&e.file))).map_err(|e| e.to_string())
    }
    fn entry(&mut self, r: &GraphArtifactRef) -> Result<&mut Entry, String> {
        Self::validate_reference(r)?;
        let e = self.entries.get_mut(&r.id).ok_or("制品不存在或已过期")?;
        if e.reference != *r { return Err("传输制品身份冲突".into()); }
        e.touched = Instant::now(); Ok(e)
    }
    fn validate_archive_binding(file: &mut File, r: &GraphArtifactRef, scan: bool) -> Result<(), String> {
        let metadata = file.metadata().map_err(|e| e.to_string())?;
        if !metadata.is_file() || metadata.len() != r.size { return Err("制品实际大小不符".into()); }
        file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        let mut magic = [0; 8]; file.read_exact(&mut magic).map_err(|e| e.to_string())?;
        let member = r.member_identity.as_ref();
        match &magic {
            b"NUWAXDC1" => {
                let (metadata, _) = crate::dense_cloud::read_metadata(file)?;
                if let Some(m) = member {
                    let o = m.occupancy_receipt.as_ref().unwrap(); let c = m.cloud_receipt.as_ref().ok_or("完整点云缺少云收据")?;
                    if m.role != CheckpointMemberRole::DenseCloud as i32 || metadata.session_id != o.session_id
                        || metadata.session_generation != c.session_generation || metadata.cloud_revision != c.cloud_revision
                        || metadata.source_timestamp_ns != c.source_timestamp_ns || metadata.frame_id != m.frame_id { return Err("完整点云档案与来源身份不符".into()); }
                }
                if scan { crate::dense_cloud::validate_archive(file)?; }
            }
            b"NUWAXAR1" | b"NUWAXAR2" | b"NUWAXAR3" | b"NUWAXAR4" | b"NUWAXAR5" => {
                if matches!(&magic, b"NUWAXAR2" | b"NUWAXAR3" | b"NUWAXAR4" | b"NUWAXAR5") && r.identity_schema_version != 2 { return Err("AR2/AR3/AR4/AR5需要新制品身份".into()); }
                // Assigned outer ownership may retain AR1 source bytes. The
                // real installation commit, not import, establishes its new baseline.
                let mut fine = nuwax_map_chunks::ArchiveReader::open(file)?;
                let (domain, revision, timestamp, keys) = match fine.root() {
                    nuwax_map_chunks::ArchivedRoot::Current(root) => (&root.domain, root.revision, root.source_timestamp_ns, root.blocks.iter().map(|b| b.key).collect::<Vec<_>>()),
                    nuwax_map_chunks::ArchivedRoot::Legacy(root) => (&root.manifest.domain, root.manifest.revision, root.manifest.source_timestamp_ns, root.manifest.blocks.iter().map(|b| b.key).collect::<Vec<_>>()),
                };
                if let Some(m) = member {
                    let o = m.occupancy_receipt.as_ref().unwrap(); let c = m.cloud_receipt.as_ref().ok_or("精细地图缺少云收据")?;
                    if m.role != CheckpointMemberRole::FineMap as i32 || domain.session != o.session_id
                        || domain.generation != c.session_generation || domain.frame_generation != m.frame_generation
                        || domain.producer_generation != m.producer_generation || revision != c.cloud_revision || timestamp != c.source_timestamp_ns { return Err("精细档案根与来源身份不符".into()); }
                }
                if scan { for key in keys { fine.block_by_key(key)?; } }
            }
            b"NWS2D001" | b"NWS2D002" => {
                let metadata = if scan { archive::grid_structure(file)?.0 } else { archive::read_structure(file)?.0 };
                match (&metadata, member) {
                    (archive::ArchiveMetadata::Legacy(_), None) if r.identity_schema_version != 2 => {},
                    (archive::ArchiveMetadata::Assigned(assigned), Some(m)) if r.identity_schema_version == 2 => {
                        let source = &assigned.source;
                        let o = m.occupancy_receipt.as_ref().ok_or("二维图缺少规范占据收据")?;
                        if m.role != CheckpointMemberRole::PlanarGraph as i32
                            || m.checkpoint_id != assigned.checkpoint_id || m.cloud_receipt.is_some()
                            || m.frame_id != source.session.map_frame_id
                            || o.session_id != source.session.session_id
                            || o.session_generation != source.session.session_generation
                            || o.source_timestamp_ns != source.source_unix_timestamp_ns {
                            return Err("二维档案与检查点或来源身份不符".into());
                        }
                    },
                    _ => return Err("二维档案与外层身份代际混合".into()),
                }
            }
            _ => return Err("未知制品档案格式".into()),
        }
        Ok(())
    }
    pub fn transfer(&mut self, request: GraphArtifactTransferRequest) -> Result<GraphArtifactTransferResponse, String> {
        self.collect(); let r = request.artifact.ok_or("缺少制品身份")?; Self::validate_reference(&r)?;
        if request.data.len() > archive::CHUNK_BYTES { return Err("制品分块超过预算".into()); }
        if request.action == 1 {
            if !request.data.is_empty() || request.offset != 0 { return Err("初始化不能携带制品数据".into()); }
            if !self.entries.contains_key(&r.id) {
                if self.entries.len() >= 4 { return Err("传输制品数量达到上限".into()); }
                let path = std::env::temp_dir().join(format!("nuwax-planar-transfer-{}", uuid::Uuid::new_v4().simple()));
                let file = std::fs::OpenOptions::new().read(true).write(true).create_new(true).mode(0o600).open(&path).map_err(|e| e.to_string())?;
                std::fs::remove_file(path).map_err(|e| e.to_string())?;
                self.entries.insert(r.id.clone(), Entry { file, reference: r.clone(), written: 0, sealed: false, touched: Instant::now(), #[cfg(test)] validation_runs: 0 });
            }
        }
        if request.action == 5 {
            if !request.data.is_empty() || request.offset != 0 { return Err("释放不能携带位置或数据".into()); }
            if self.entries.contains_key(&r.id) { self.entry(&r)?; self.entries.remove(&r.id); }
            return Ok(GraphArtifactTransferResponse { artifact: Some(r), ..Default::default() });
        }
        let e = self.entry(&r)?; let mut data = Vec::new();
        match request.action {
            1 => {}
            2 => {
                let end = request.offset.checked_add(request.data.len() as u64).ok_or("分块位置溢出")?;
                if request.data.is_empty() || end > e.reference.size || request.offset > e.written { return Err("上传分块位置非法".into()); }
                e.file.seek(SeekFrom::Start(request.offset)).map_err(|e| e.to_string())?;
                let overlap = (e.written - request.offset).min(request.data.len() as u64) as usize;
                if overlap > 0 {
                    let mut previous = vec![0; overlap]; e.file.read_exact(&mut previous).map_err(|e| e.to_string())?;
                    if previous != request.data[..overlap] { return Err("相同位置分块内容冲突".into()); }
                }
                if end > e.written {
                    if e.sealed { return Err("不可修改已封存制品".into()); }
                    e.file.write_all(&request.data[overlap..]).map_err(|e| e.to_string())?; e.written = end;
                }
            }
            3 => {
                if !request.data.is_empty() || request.offset != 0 { return Err("封存不能携带位置或数据".into()); }
                if e.written != e.reference.size { return Err("上传尚未完整，不能封存".into()); }
                if !e.sealed {
                    // Format dispatch precedes every legacy operation: no assigned fine archive hash bridge.
                    Self::validate_archive_binding(&mut e.file, &r, true)?;
                    #[cfg(test)] { e.validation_runs += 1; }
                    if r.identity_schema_version != 2 {
                        e.file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
                        if crate::dense_cloud::hash_reader(&mut e.file)? != (r.size, r.sha256.as_slice().try_into().unwrap()) { return Err("旧上传完整摘要不匹配".into()); }
                    }
                    e.file.sync_all().map_err(|e| e.to_string())?; e.sealed = true;
                }
            }
            4 => {
                if !e.sealed || request.offset >= e.reference.size || !request.data.is_empty() { return Err("下载状态或位置非法".into()); }
                data.resize((e.reference.size - request.offset).min(archive::CHUNK_BYTES as u64) as usize, 0);
                e.file.seek(SeekFrom::Start(request.offset)).map_err(|e| e.to_string())?; e.file.read_exact(&mut data).map_err(|e| e.to_string())?;
            }
            _ => return Err("未知制品操作".into()),
        }
        Ok(GraphArtifactTransferResponse { artifact: Some(r), offset: e.written, sealed: e.sealed, data })
    }
}

#[cfg(test)]
mod assigned_identity_tests {
    use super::*;
    use prost::Message;
    use crate::proto::{CanonicalCloudCommitReceipt, CanonicalOccupancyCommitReceipt, MappingCommitIdentity};
    fn member(role: CheckpointMemberRole) -> CheckpointMemberIdentity {
        CheckpointMemberIdentity {
            checkpoint_id:"11111111111111111111111111111111".into(),role:role as i32,frame_id:"map".into(),
            occupancy_receipt:Some(CanonicalOccupancyCommitReceipt {commit_id:Some(MappingCommitIdentity{boot_nonce:vec![1;16],sequence:5}),session_id:"source".into(),session_generation:2,occupancy_revision:7,source_timestamp_ns:100,backend_kind:"voxel".into(),backend_generation:3}),
            cloud_receipt:Some(CanonicalCloudCommitReceipt {commit_id:Some(MappingCommitIdentity{boot_nonce:vec![1;16],sequence:5}),session_generation:2,occupancy_revision:7,cloud_revision:9,source_timestamp_ns:100,point_count:1}),
            frame_generation:if role==CheckpointMemberRole::FineMap{4}else{0},producer_generation:if role==CheckpointMemberRole::FineMap{6}else{0},
        }
    }
    fn dense() -> (File,u64) {
        let mut source=crate::dense_cloud::DenseCloud::new().unwrap();
        source.append(&[nalgebra::Point3::new(1.0,2.0,3.0),nalgebra::Point3::new(1.01,2.01,3.01)]).unwrap();
        let receipt=crate::mapping_commit::CloudCommitReceipt{commit_id:crate::mapping_commit::MappingCommitId{boot_nonce:1,sequence:5},session_generation:2,occupancy_revision:7,cloud_revision:9,source_timestamp_ns:100,point_count:1};
        source.snapshot(&receipt,"source").unwrap().archive("map").unwrap()
    }
    fn fine() -> (File,u64) {
        let repo=nuwax_map_chunks::Repository::temporary().unwrap();
        let mut writer=nuwax_map_chunks::Writer::with_id_source(repo.clone(),0.05,4*1024*1024,||Ok(nuwax_map_chunks::ObjectId([1;16]))).unwrap();
        writer.observe([[0,0,0]],100).unwrap();
        let root=writer.snapshot(nuwax_map_chunks::Domain{session:"source".into(),generation:2,frame_generation:4,producer_generation:6},9,100).unwrap();
        nuwax_map_chunks::archive_file(&root,&repo).unwrap()
    }
    fn request(store:&mut Store,reference:&GraphArtifactRef,action:u32,offset:u64,data:&[u8])->Result<GraphArtifactTransferResponse,String> {
        let wire=GraphArtifactTransferRequest{action,artifact:Some(reference.clone()),offset,data:data.to_vec()}.encode_to_vec();
        let response=store.transfer(GraphArtifactTransferRequest::decode(wire.as_slice()).map_err(|e|e.to_string())?)?;
        GraphArtifactTransferResponse::decode(response.encode_to_vec().as_slice()).map_err(|e|e.to_string())
    }
    fn bytes(mut file:File)->Vec<u8>{file.seek(SeekFrom::Start(0)).unwrap();let mut bytes=Vec::new();file.read_to_end(&mut bytes).unwrap();bytes}
    fn reference(size:u64,role:CheckpointMemberRole)->GraphArtifactRef{GraphArtifactRef{id:"22222222222222222222222222222222".into(),size,sha256:Vec::new(),identity_schema_version:2,member_identity:Some(member(role))}}
    #[test]
    fn assigned_v2_store_dense_exact_retry_descriptor_overlap_seal_and_release() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let (file,size)=dense();let source=bytes(file);let r=reference(size,CheckpointMemberRole::DenseCloud);let mut store=Store::default();
        assert_eq!(request(&mut store,&r,1,0,&[]).unwrap().offset,0);request(&mut store,&r,1,0,&[]).unwrap();
        let mut changed=r.clone();changed.member_identity.as_mut().unwrap().checkpoint_id="3".repeat(32);
        assert!(request(&mut store,&changed,1,0,&[]).is_err());assert!(request(&mut store,&r,3,0,&[]).is_err());
        request(&mut store,&r,2,0,&source[..20]).unwrap();request(&mut store,&r,2,0,&source[..20]).unwrap();
        let mut conflict=source[..20].to_vec();conflict[4]^=1;assert!(request(&mut store,&r,2,0,&conflict).is_err());
        assert_eq!(request(&mut store,&r,2,10,&source[10..]).unwrap().offset,size);
        assert!(request(&mut store,&r,3,0,&[]).unwrap().sealed);request(&mut store,&r,3,0,&[]).unwrap();assert_eq!(store.entries[&r.id].validation_runs,1);
        request(&mut store,&r,2,0,&source).unwrap();assert_eq!(request(&mut store,&r,4,0,&[]).unwrap().data,source);assert_eq!(bytes(store.open(&r).unwrap()),source);assert_eq!(store.entries[&r.id].validation_runs,1);
        assert!(request(&mut store,&changed,4,0,&[]).is_err());assert!(request(&mut store,&changed,5,0,&[]).is_err());
        request(&mut store,&r,5,0,&[]).unwrap();request(&mut store,&r,5,0,&[]).unwrap();assert!(store.open(&r).is_err());
    }
    #[test]
    fn assigned_v2_store_fine_ar4_source_binding_and_unknown_format() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let (file,size)=fine();let source=bytes(file);assert_eq!(&source[..8],b"NUWAXAR4");
        for field in 0..6 {
            let mut r=reference(size,CheckpointMemberRole::FineMap);let m=r.member_identity.as_mut().unwrap();
            match field {0=>m.frame_generation+=1,1=>m.producer_generation+=1,2=>m.occupancy_receipt.as_mut().unwrap().session_id="other".into(),
                3=>{m.occupancy_receipt.as_mut().unwrap().session_generation+=1;m.cloud_receipt.as_mut().unwrap().session_generation+=1;},
                4=>m.cloud_receipt.as_mut().unwrap().cloud_revision+=1,_=>{m.occupancy_receipt.as_mut().unwrap().source_timestamp_ns+=1;m.cloud_receipt.as_mut().unwrap().source_timestamp_ns+=1;}}
            let mut store=Store::default();request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&source).unwrap();assert!(request(&mut store,&r,3,0,&[]).is_err(),"source field {field}");
        }
        let r=reference(size,CheckpointMemberRole::FineMap);let mut store=Store::default();request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&source).unwrap();request(&mut store,&r,3,0,&[]).unwrap();assert_eq!(request(&mut store,&r,4,0,&[]).unwrap().data,source);
        let mut unknown=source.clone();unknown[..8].copy_from_slice(b"NUWAXAR9");let mut store=Store::default();request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&unknown).unwrap();assert!(request(&mut store,&r,3,0,&[]).unwrap_err().contains("未知制品档案格式"));
    }
    #[test]
    fn assigned_v2_store_ar4_layers_keep_identity_and_reject_legacy_envelopes() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let repo=nuwax_map_chunks::Repository::temporary().unwrap();
        let mut writer=nuwax_map_chunks::Writer::new(repo.clone(),0.125,4*1024*1024).unwrap();
        writer.observe([[0,0,0]],100).unwrap();
        let producer=std::sync::Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
            sensor_id:"depth/front".into(),clock_id:"camera".into(),runtime_generation:17,session_generation:2,
        });
        writer.record_source_watermark(producer.clone(),900,0).unwrap();
        writer.replace_layer_observations([0,0,0],[(0,nuwax_map_chunks::LayerObservation {
            members:vec![0],plane_origin:[0.0625,0.0625,0.08],normal:[0.0,0.0,1.0],
            samples:vec![nuwax_map_chunks::LayerSample {source:producer.clone(),point:[0.04,0.06,0.08],source_sequence:0,source_timestamp_ns:90,point_index:0}],
        })].into()).unwrap();
        let root=writer.snapshot(nuwax_map_chunks::Domain {session:"source".into(),generation:2,frame_generation:4,producer_generation:6},9,100).unwrap();
        let(file,size)=nuwax_map_chunks::archive_file(&root,&repo).unwrap();let source=bytes(file);
        assert_eq!(&source[..8],b"NUWAXAR4");
        let r=reference(size,CheckpointMemberRole::FineMap);let mut store=Store::default();
        request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&source).unwrap();
        assert!(request(&mut store,&r,3,0,&[]).unwrap().sealed);
        assert_eq!(store.entries[&r.id].reference.identity_schema_version,2);
        assert_eq!(bytes(store.open(&r).unwrap()),source);
        let mut reopened=nuwax_map_chunks::ArchiveReader::open(store.open(&r).unwrap()).unwrap();
        assert_eq!(reopened.root(),&nuwax_map_chunks::ArchivedRoot::Current((*root).clone()));
        let layers=nuwax_map_chunks::decode_layer_observations(&reopened.block_by_key([0,0,0]).unwrap()).unwrap();
        assert_eq!(layers[&0].samples[0].source,producer);
        assert_eq!((layers[&0].samples[0].source_timestamp_ns,layers[&0].samples[0].source_sequence),(90,0));
        let(file,size)=nuwax_map_chunks::archive_file(&root,&repo).unwrap();
        let inserted=store.insert(file,size,member(CheckpointMemberRole::FineMap)).unwrap();
        assert_eq!(inserted.identity_schema_version,2);
        let(file,size)=nuwax_map_chunks::archive_file(&root,&repo).unwrap();
        assert!(store.insert_legacy(file,size,[9;32]).unwrap_err().contains("AR4"));
        let legacy=GraphArtifactRef {id:"legacy-ar4".into(),size,sha256:vec![9;32],..Default::default()};
        request(&mut store,&legacy,1,0,&[]).unwrap();request(&mut store,&legacy,2,0,&source).unwrap();
        assert!(request(&mut store,&legacy,3,0,&[]).unwrap_err().contains("AR4"));
    }
    #[test]
    fn assigned_v2_store_ar5_heightfield_uses_original_identity_and_transfer() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let repo=nuwax_map_chunks::Repository::temporary().unwrap();
        let mut writer=nuwax_map_chunks::Writer::new(repo.clone(),0.05,4*1024*1024).unwrap();
        writer.observe([[0,0,0],[0,0,32]],100).unwrap();
        let field=nuwax_map_chunks::heightfield_observations::HeightfieldColumn::from_counts([0,0],0.05,
            [(0,[1,0,0,0,0,0,0,0]),(32,[1,0,0,0,0,0,0,0])].into()).unwrap().unwrap();
        writer.replace_ground_column([0,0],&nuwax_map_chunks::ColumnGroundRecords {classified_empty: false, legacy:vec![],heightfields:vec![
            nuwax_map_chunks::HeightfieldSurface {column:field.clone(),samples:vec![]}]}).unwrap();
        let root=writer.snapshot(nuwax_map_chunks::Domain {session:"source".into(),generation:2,frame_generation:4,producer_generation:6},9,100).unwrap();
        let(file,size)=nuwax_map_chunks::archive_file(&root,&repo).unwrap();let source=bytes(file);
        assert_eq!(&source[..8],b"NUWAXAR5");
        let r=reference(size,CheckpointMemberRole::FineMap);let mut store=Store::default();
        request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&source).unwrap();
        assert!(request(&mut store,&r,3,0,&[]).unwrap().sealed);
        assert_eq!(bytes(store.open(&r).unwrap()),source);
        let mut archive=nuwax_map_chunks::ArchiveReader::open(store.open(&r).unwrap()).unwrap();
        let destination=nuwax_map_chunks::Repository::temporary().unwrap();
        let imported=archive.import(&destination).unwrap();
        let mut restored=nuwax_map_chunks::Writer::new(destination,0.05,4*1024*1024).unwrap();
        restored.restore_archived(&imported).unwrap();
        assert_eq!(restored.ground_column([0,0]).unwrap().heightfields[0].column,field);
        let(file,size)=nuwax_map_chunks::archive_file(&root,&repo).unwrap();
        assert!(store.insert_legacy(file,size,[9;32]).unwrap_err().contains("AR5"));
        let legacy=GraphArtifactRef {id:"legacy-ar5".into(),size,sha256:vec![9;32],..Default::default()};
        request(&mut store,&legacy,1,0,&[]).unwrap();request(&mut store,&legacy,2,0,&source).unwrap();
        assert!(request(&mut store,&legacy,3,0,&[]).unwrap_err().contains("AR5"));
    }
    #[test]
    fn assigned_v2_store_still_reads_ar2_ar3_without_source_claims() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let repo=nuwax_map_chunks::Repository::temporary().unwrap();
        let mut writer=nuwax_map_chunks::Writer::new(repo.clone(),0.05,4*1024*1024).unwrap();
        writer.observe([[0,0,0]],100).unwrap();
        let root=writer.snapshot(nuwax_map_chunks::Domain {session:"source".into(),generation:2,frame_generation:4,producer_generation:6},9,100).unwrap();
        for version in [2,3] {
            let mut historical=(*root).clone();historical.version=version;
            let(file,size)=nuwax_map_chunks::archive_file(&historical,&repo).unwrap();let source=bytes(file);
            let r=reference(size,CheckpointMemberRole::FineMap);let mut store=Store::default();
            request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&source).unwrap();
            assert!(request(&mut store,&r,3,0,&[]).unwrap().sealed);
            assert_eq!(bytes(store.open(&r).unwrap()),source);
        }
    }
    #[test]
    fn assigned_v2_store_mixed_fields_limits_truncation_and_finite_geometry() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let(file,size)=dense();let source=bytes(file);let good=reference(size,CheckpointMemberRole::DenseCloud);
        for mutation in 0..7 {let mut bad=good.clone();match mutation {0=>bad.sha256=vec![9;32],1=>bad.identity_schema_version=3,2=>bad.id="A".repeat(32),3=>bad.member_identity=None,4=>bad.size=0,5=>bad.member_identity.as_mut().unwrap().role=99,_=>bad.member_identity.as_mut().unwrap().frame_generation=1}assert!(Store::validate_reference(&bad).is_err());}
        let mut nan=source.clone();let n=nan.len();nan[n-4..].copy_from_slice(&f32::NAN.to_le_bytes());let mut store=Store::default();request(&mut store,&good,1,0,&[]).unwrap();
        assert!(request(&mut store,&good,2,1,&source[..20]).is_err());assert!(request(&mut store,&good,2,0,&vec![0;archive::CHUNK_BYTES+1]).is_err());
        request(&mut store,&good,2,0,&nan).unwrap();assert!(request(&mut store,&good,3,0,&[]).is_err());assert!(request(&mut store,&good,4,0,&[]).is_err());
    }
    #[test]
    fn assigned_v2_store_rejects_ar4_legacy_bridge_before_hash_and_corrupt_block() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));let(file,size)=fine();let source=bytes(file);
        let legacy=GraphArtifactRef{id:"legacy-upload".into(),size,sha256:vec![9;32],..Default::default()};let mut store=Store::default();
        request(&mut store,&legacy,1,0,&[]).unwrap();request(&mut store,&legacy,2,0,&source).unwrap();assert!(request(&mut store,&legacy,3,0,&[]).unwrap_err().contains("AR4"));
        let(file,size)=fine();assert!(store.insert_legacy(file,size,[9;32]).unwrap_err().contains("AR4"));
        let r=reference(size,CheckpointMemberRole::FineMap);let mut store=Store::default();let mut corrupt=source;let n=corrupt.len();corrupt[n-2..].fill(0);
        request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&corrupt).unwrap();assert!(request(&mut store,&r,3,0,&[]).is_err());
        let(file,size)=dense();let source=bytes(file);let mut r=reference(size,CheckpointMemberRole::DenseCloud);r.member_identity.as_mut().unwrap().frame_id="odom".into();let mut store=Store::default();
        request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&source).unwrap();assert!(request(&mut store,&r,3,0,&[]).is_err());
    }
    #[test]
    fn assigned_v2_store_rejects_one_fine_object_id_with_different_archive_bodies() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));let(file,_)=fine();let reader=nuwax_map_chunks::ArchiveReader::open(file).unwrap();
        let nuwax_map_chunks::ArchivedRoot::Current(mut root)=reader.root().clone()else{panic!("AR4 expected")};let mut second=root.blocks[0].clone();second.key=[1,0,0];root.blocks.push(second);
        let root_bytes=root.encode().unwrap();let blocks=[100,99].map(|observed_ns|nuwax_map_chunks::encode_block(&nuwax_map_chunks::Block::from([(0,nuwax_map_chunks::Evidence{observed_ns,hits:1})])).unwrap());
        let mut bytes=b"NUWAXAR4".to_vec();bytes.extend_from_slice(&(root_bytes.len()as u64).to_le_bytes());bytes.extend_from_slice(&root_bytes);bytes.extend_from_slice(&2_u64.to_le_bytes());
        for block in blocks{bytes.extend_from_slice(&(block.len()as u32).to_le_bytes());bytes.extend_from_slice(&block);}
        let r=reference(bytes.len()as u64,CheckpointMemberRole::FineMap);let mut store=Store::default();request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&bytes).unwrap();assert!(request(&mut store,&r,3,0,&[]).is_err());
    }
    #[test]
    fn assigned_v2_store_accepts_ar1_source_without_reinterpreting_legacy_aliases() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let root=nuwax_map_chunks::LegacyManifest{version:1,domain:nuwax_map_chunks::Domain{session:"source".into(),generation:2,frame_generation:4,producer_generation:6},revision:9,source_timestamp_ns:100,voxel_size_m:0.05,
            blocks:vec![nuwax_map_chunks::LegacyBlockRef{key:[0;3],sha256:nuwax_map_chunks::LegacyAlias([7;32]),geometry_sha256:nuwax_map_chunks::LegacyAlias([8;32]),points:1}]};
        let json=serde_json::to_vec(&root).unwrap();let block=nuwax_map_chunks::encode_block(&nuwax_map_chunks::Block::from([(0,nuwax_map_chunks::Evidence{observed_ns:100,hits:1})])).unwrap();
        let mut bytes=b"NUWAXAR1".to_vec();bytes.extend_from_slice(&(json.len()as u64).to_le_bytes());bytes.extend_from_slice(&[9;32]);bytes.extend_from_slice(&json);bytes.extend_from_slice(&1_u64.to_le_bytes());bytes.extend_from_slice(&(block.len()as u32).to_le_bytes());bytes.extend_from_slice(&block);
        assert!(matches!(nuwax_map_chunks::ArchiveReader::open(std::io::Cursor::new(&bytes)).unwrap().root(),nuwax_map_chunks::ArchivedRoot::Legacy(_)));
        let r=reference(bytes.len()as u64,CheckpointMemberRole::FineMap);let mut store=Store::default();request(&mut store,&r,1,0,&[]).unwrap();request(&mut store,&r,2,0,&bytes).unwrap();assert!(request(&mut store,&r,3,0,&[]).unwrap().sealed);assert_eq!(request(&mut store,&r,4,0,&[]).unwrap().data,bytes);
    }
    #[test]
    fn assigned_v2_store_owned_insert_and_retirement() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));let mut store=Store::default();let(file,size)=fine();let r=store.insert(file,size,member(CheckpointMemberRole::FineMap)).unwrap();assert!(assigned_id(&r.id));assert!(r.sha256.is_empty());
        let original=request(&mut store,&r,4,0,&[]).unwrap().data;
        for _ in 0..8{assert_eq!(request(&mut store,&r,4,0,&[]).unwrap().data,original);assert!(request(&mut store,&r,3,0,&[]).unwrap().sealed);}assert_eq!(store.entries[&r.id].validation_runs,0);
        let reader=store.open(&r).unwrap();store.entries.get_mut(&r.id).unwrap().touched=Instant::now()-Duration::from_secs(601);assert!(store.open(&r).is_err());assert_eq!(bytes(reader),original);
        let(file,size)=dense();assert!(store.insert(file,size+1,member(CheckpointMemberRole::DenseCloud)).is_err());
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum TargetIdentity {
    Legacy([u8; 32]),
    Assigned(GraphArtifactRef),
}
impl TargetIdentity {
    pub fn runtime_identity(&self) -> nuwax_slam_2d_runtime::protocol::ArtifactIdentity {
        match self {
            Self::Legacy(_) => nuwax_slam_2d_runtime::protocol::ArtifactIdentity::Legacy,
            Self::Assigned(reference) => nuwax_slam_2d_runtime::protocol::ArtifactIdentity::Assigned {
                checkpoint_id: reference.member_identity.as_ref().expect("validated graph member").checkpoint_id.clone(),
            },
        }
    }
    pub fn legacy_digest(&self) -> Option<[u8; 32]> {
        match self { Self::Legacy(sha) => Some(*sha), Self::Assigned(_) => None }
    }
}
#[derive(Debug)]
pub(crate) struct Target {
    pub file: std::sync::Arc<File>,
    pub size: u64,
    pub identity: TargetIdentity,
    pub metadata: archive::ArchiveMetadata,
}
impl PartialEq for Target {
    fn eq(&self, other: &Self) -> bool {
        self.size == other.size && self.identity == other.identity && self.metadata == other.metadata
    }
}
impl Target {
    pub fn open(&self) -> Result<File, String> {
        File::open(format!(
            "/proc/self/fd/{}",
            std::os::fd::AsRawFd::as_raw_fd(&*self.file)
        ))
        .map_err(|e| e.to_string())
    }
    pub fn receipt(
        &self,
        activation: [u8; 32],
        receipt: &crate::mapping_commit::OccupancyCommitReceipt,
    ) -> [u8; 32] {
        let mut data = b"nuwax.geometry.planar-target.v1\0".to_vec();
        data.extend_from_slice(&activation);
        data.extend_from_slice(&self.identity.legacy_digest().expect("legacy target receipt requires legacy target"));
        data.extend_from_slice(&receipt.commit_id.boot_nonce.to_be_bytes());
        for n in [
            self.size,
            receipt.commit_id.sequence,
            receipt.session_generation,
            receipt.occupancy_revision,
            receipt.source_timestamp_ns,
            receipt.backend_generation,
        ] {
            data.extend_from_slice(&n.to_be_bytes());
        }
        nuwax_codec::hash::sha256_bytes(&data)
    }
}

#[cfg(test)]
pub(crate) mod planar_assigned_pair_tests {
    use super::*;
    use crate::proto::{CanonicalOccupancyCommitReceipt, MappingBackendCheckpoint, MappingCommitIdentity, OccupancyGridData, OccupancyGridMeta};

    pub(crate) fn fixture() -> (MappingBackendCheckpoint, Vec<u8>) {
        let metadata = archive::AssignedMetadata {
            checkpoint_id: "1".repeat(32),
            source: archive::Metadata {
                version: 2,
                session: serde_json::from_str(r#"{"session_id":"source","map_name":"map","session_generation":2,"sensor_id":"laser","frame_id":"laser","map_frame_id":"map","odom_frame_id":"odom","body_frame_id":"base","clock_domain":"unix_ns","clock_id":"","calibration_ref":"calibration-1","odometry_source":"device","core_config":{"base_from_laser":[0,0,0]}}"#).unwrap(),
                estimate: nuwax_slam_2d_runtime::protocol::Estimate { graph_revision: 9, ..Default::default() },
                source_unix_timestamp_ns: 100, width: 3, height: 1, resolution: 0.05, origin: [1.0, 2.0],
                members: ["graph.posegraph", "graph.data", "grid.i8"].map(|name| archive::AssignedMember { name: name.into(), size: 3 }),
            },
        };
        let header = serde_json::to_vec(&metadata).unwrap();
        let bytes = [b"NWS2D002".as_slice(), &(header.len() as u32).to_le_bytes(), &header, b"abcdef\0\x64\xff"].concat();
        let receipt = CanonicalOccupancyCommitReceipt {
            commit_id: Some(MappingCommitIdentity { boot_nonce: vec![1;16], sequence: 5 }),
            session_id: "source".into(), session_generation: 2, occupancy_revision: 7,
            source_timestamp_ns: 100, backend_kind: "slam_toolbox".into(), backend_generation: 3,
        };
        let graph = CheckpointMemberIdentity { checkpoint_id: metadata.checkpoint_id.clone(), role: CheckpointMemberRole::PlanarGraph as i32,
            occupancy_receipt: Some(receipt.clone()), cloud_receipt: None, frame_id: "map".into(), frame_generation: 0, producer_generation: 0 };
        let mut occupancy = graph.clone(); occupancy.role = CheckpointMemberRole::CanonicalOccupancy as i32;
        (MappingBackendCheckpoint {
            schema_version: 2, backend_kind: "slam_toolbox".into(), source_session_id: "source".into(), source_session_generation: 2,
            created_at_ns: 101, checkpoint_id: metadata.checkpoint_id, backend_payload_schema_version: 2,
            payload_size: bytes.len() as u64, target_kind: "planar_graph".into(), occupancy_receipt: Some(receipt), occupancy_identity: Some(occupancy),
            graph_artifact: Some(GraphArtifactRef { id: "2".repeat(32), size: bytes.len() as u64, identity_schema_version: 2, member_identity: Some(graph), sha256: Vec::new() }),
            canonical_occupancy_checkpoint: Some(OccupancyGridData { meta: Some(OccupancyGridMeta { width: 3, height: 1, resolution: 0.05,
                origin_x: 1.0, origin_y: 2.0, frame_id: "map".into(), revision: 7, timestamp_ns: 100 }), data: vec![0,100,255] }),
            ..Default::default()
        }, bytes)
    }
    pub(crate) fn transfer(store: &mut Store, r: &GraphArtifactRef, action: u32, offset: u64, data: Vec<u8>) -> Result<GraphArtifactTransferResponse, String> {
        store.transfer(GraphArtifactTransferRequest { action, artifact: Some(r.clone()), offset, data })
    }
    pub(crate) fn upload(store: &mut Store, r: &GraphArtifactRef, bytes: &[u8]) -> Result<(), String> {
        transfer(store,r,1,0,vec![])?; transfer(store,r,2,0,bytes.to_vec())?; transfer(store,r,3,0,vec![])?; Ok(())
    }
    #[test]
    fn planar_assigned_pair_seal_retry_and_reader_survive_release_without_hash() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let (checkpoint, bytes) = fixture(); let r=checkpoint.graph_artifact.unwrap(); let mut store=Store::default();
        upload(&mut store,&r,&bytes).unwrap(); transfer(&mut store,&r,3,0,vec![]).unwrap();
        assert_eq!(store.entries[&r.id].validation_runs,1);
        let mut pinned=store.open(&r).unwrap(); transfer(&mut store,&r,5,0,vec![]).unwrap();
        let (metadata,cells)=archive::grid_structure(&mut pinned).unwrap();
        assert_eq!(metadata.identity(),nuwax_slam_2d_runtime::protocol::ArtifactIdentity::Assigned {checkpoint_id:"1".repeat(32)});
        assert_eq!(cells,vec![0,100,-1]); assert!(store.open(&r).is_err());
    }
    #[test]
    fn planar_assigned_pair_binds_graph_role_checkpoint_session_frame_and_source_time() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        for change in 0..6 {
            let (checkpoint, bytes)=fixture(); let mut r=checkpoint.graph_artifact.unwrap(); let m=r.member_identity.as_mut().unwrap();
            match change {0=>m.checkpoint_id="3".repeat(32),1=>m.occupancy_receipt.as_mut().unwrap().session_id="other".into(),
                2=>m.occupancy_receipt.as_mut().unwrap().session_generation+=1,3=>m.frame_id="odom".into(),
                4=>m.occupancy_receipt.as_mut().unwrap().source_timestamp_ns+=1,_=>m.role=CheckpointMemberRole::CanonicalOccupancy as i32}
            assert!(upload(&mut Store::default(),&r,&bytes).is_err(),"source field {change}");
        }
        let (checkpoint, mut bytes)=fixture(); let r=checkpoint.graph_artifact.unwrap();
        *bytes.last_mut().unwrap()=101; assert!(upload(&mut Store::default(),&r,&bytes).is_err());
        bytes.pop(); assert!(upload(&mut Store::default(),&r,&bytes).is_err());
    }
    #[test]
    fn planar_assigned_pair_legacy_envelope_cannot_hash_an_assigned_archive() {
        crate::dense_cloud::FORBID_ARCHIVE_HASH.with(|v|v.set(true));
        let (_,bytes)=fixture();
        let r=GraphArtifactRef {id:"legacy".into(),size:bytes.len() as u64,sha256:vec![7;32],..Default::default()};
        assert!(upload(&mut Store::default(),&r,&bytes).unwrap_err().contains("代际"));
    }
}
