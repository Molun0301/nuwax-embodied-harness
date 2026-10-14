// Included by archive.rs to exercise its private streaming primitive.
use super::*;
use std::io::{self, Cursor};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};

fn metadata() -> AssignedMetadata {
    AssignedMetadata {
        checkpoint_id: "11111111111111111111111111111111".into(),
        source: Metadata {
            version: 2,
            session: serde_json::from_str(r#"{"session_id":"s","map_name":"map","session_generation":1,"sensor_id":"laser","frame_id":"laser","map_frame_id":"map","odom_frame_id":"odom","body_frame_id":"base","clock_domain":"unix_ns","clock_id":"","calibration_ref":"calibration-1","odometry_source":"device","core_config":{"base_from_laser":[0,0,0]}}"#).unwrap(),
            estimate: Estimate { graph_revision: 9, ..Default::default() },
            source_unix_timestamp_ns: 100,
            width: 3, height: 1, resolution: 0.05, origin: [1.0, 2.0],
            members: ["graph.posegraph", "graph.data", "grid.i8"].map(|name| AssignedMember { name: name.into(), size: 3 }),
        },
    }
}
struct Directory(PathBuf);
impl Directory {
    fn new() -> Self {
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let path = std::env::temp_dir().join(format!("assigned-archive-{}-{}", std::process::id(), NEXT.fetch_add(1, Ordering::Relaxed)));
        std::fs::create_dir(&path).unwrap();
        Self(path)
    }
    fn join(&self, name: &str) -> PathBuf { self.0.join(name) }
}
impl Drop for Directory { fn drop(&mut self) { let _ = std::fs::remove_dir_all(&self.0); } }
fn members(dir: &Directory) -> [PathBuf; 3] {
    let paths = [dir.join("graph.posegraph"), dir.join("graph.data"), dir.join("grid.i8")];
    for (path, bytes) in paths.iter().zip([b"abc".as_slice(), b"def", &[0,100,255]]) { std::fs::write(path, bytes).unwrap(); }
    paths
}
fn encoded(m: &AssignedMetadata) -> Vec<u8> {
    let mut output = Vec::new();
    let mut a = Cursor::new(b"abc"); let mut b = Cursor::new(b"def"); let mut c = Cursor::new([0,100,255]);
    write_assigned_members(&mut output, m, &mut [&mut a, &mut b, &mut c]).unwrap();
    output
}
fn raw(magic: &[u8;8], header: &[u8], body: &[u8]) -> Vec<u8> {
    [magic.as_slice(), &(header.len() as u32).to_le_bytes(), header, body].concat()
}
#[test]
fn three_members_roundtrip_copy_and_reopen_keep_checkpoint() {
    let dir = Directory::new(); let paths = members(&dir); let m = metadata();
    let archive = dir.join("saved");
    let descriptor = write_assigned(&archive, &m, paths.each_ref().map(|p| p.as_path())).unwrap();
    assert_eq!(descriptor.checkpoint_id, m.checkpoint_id);
    assert_eq!(descriptor.payload_size, std::fs::metadata(&archive).unwrap().len());
    let copied = dir.join("copied"); std::fs::copy(&archive, &copied).unwrap();
    let mut file = open_regular(&copied).unwrap();
    assert_eq!(grid_structure(&mut file).unwrap(), (ArchiveMetadata::Assigned(m.clone()), vec![0,100,-1]));
    let unpacked = Directory::new();
    assert_eq!(unpack_structure(&mut file, &unpacked.0).unwrap(), ArchiveMetadata::Assigned(m.clone()));
    for path in paths { assert_eq!(std::fs::read(&path).unwrap(), std::fs::read(unpacked.join(path.file_name().unwrap().to_str().unwrap())).unwrap()); }
    assert!(read_header(&mut file).is_err()); // No accidental v2 consumption by active v1 APIs.
    let header = serde_json::to_string(&m).unwrap();
    assert!(!header.contains("sha256"));
}
#[test]
fn metadata_rejects_wrong_roles_order_budgets_geometry_and_ids() {
    let mut cases = Vec::new();
    let mut m=metadata(); m.source.members.swap(0,1); cases.push(m);
    let mut m=metadata(); m.source.members[0].name="../escape".into(); cases.push(m);
    let mut m=metadata(); m.source.members[0].size=MAX_MEMBER+1; cases.push(m);
    let mut m=metadata(); m.source.members[1].size=0; cases.push(m);
    let mut m=metadata(); m.source.members[2].size=MAX_GRID+1; cases.push(m);
    let mut m=metadata(); m.source.width=2; cases.push(m);
    let mut m=metadata(); m.source.resolution=f64::NAN; cases.push(m);
    let mut m=metadata(); m.source.version=1; cases.push(m);
    let mut m=metadata(); m.source.estimate.graph_revision=0; cases.push(m);
    let mut m=metadata(); m.checkpoint_id.clear(); cases.push(m);
    for bad in ["0".repeat(32), "A".repeat(32), "g".repeat(32), "1".repeat(31), "1".repeat(33)] {
        let mut m=metadata(); m.checkpoint_id=bad; cases.push(m);
    }
    for m in cases { assert!(m.validate().is_err(), "{m:?}"); }
    let m=metadata(); let mut other=m.source.session.clone(); other.sensor_id="other".into();
    assert!(m.source.compatible_with(&other).is_err());
}
#[test]
fn reader_rejects_truncation_extra_bytes_magic_and_header_budget() {
    let valid=encoded(&metadata());
    for length in [0,7,11,12,valid.len()-1] { assert!(read_structure(&mut Cursor::new(&valid[..length])).is_err()); }
    let mut bytes=valid.clone(); bytes.push(0); assert!(read_structure(&mut Cursor::new(bytes)).is_err());
    let mut bytes=valid.clone(); bytes[7]=b'3'; assert!(read_structure(&mut Cursor::new(bytes)).is_err());
    for size in [0,(MAX_HEADER+1) as u32] {
        let mut bytes=valid.clone(); bytes[8..12].copy_from_slice(&size.to_le_bytes());
        assert!(read_structure(&mut Cursor::new(bytes)).is_err());
    }
    let mut m=metadata(); m.source.members[0].size=MAX_MEMBER+1;
    assert!(read_structure(&mut Cursor::new(raw(ASSIGNED_MAGIC,&serde_json::to_vec(&m).unwrap(), b"abcdef\0\x64\xff"))).is_err());
    let mut value=serde_json::to_value(metadata()).unwrap(); value["source"]["members"][0]["sha256"]=serde_json::json!([1,2,3]);
    assert!(read_structure(&mut Cursor::new(raw(ASSIGNED_MAGIC,&serde_json::to_vec(&value).unwrap(),b"abcdef\0\x64\xff"))).is_err());
}
#[test]
fn illegal_grid_and_changed_member_lengths_clean_up_outputs() {
    for body in [vec![0,42,255],vec![0,100],vec![0,100,255,0]] {
        let dir=Directory::new(); let paths=members(&dir); std::fs::write(&paths[2],body).unwrap();
        let archive=dir.join("out");
        assert!(write_assigned(&archive,&metadata(),paths.each_ref().map(|p|p.as_path())).is_err());
        assert!(!archive.exists());
    }
    let mut bytes=encoded(&metadata()); *bytes.last_mut().unwrap()=42;
    assert!(grid_structure(&mut Cursor::new(&bytes)).is_err());
    let dir=Directory::new(); assert!(unpack_structure(&mut Cursor::new(bytes),&dir.0).is_err());
    assert_eq!(std::fs::read_dir(&dir.0).unwrap().count(),0);
}
#[test]
fn preexisting_files_are_not_overwritten_or_deleted_on_failure() {
    let dir=Directory::new(); let paths=members(&dir); let archive=dir.join("existing");
    std::fs::write(&archive,b"keep").unwrap();
    assert!(write_assigned(&archive,&metadata(),paths.each_ref().map(|p|p.as_path())).is_err());
    assert_eq!(std::fs::read(&archive).unwrap(),b"keep");
    let dest=Directory::new(); std::fs::write(dest.join("graph.data"),b"keep").unwrap();
    assert!(unpack_structure(&mut Cursor::new(encoded(&metadata())),&dest.0).is_err());
    assert!(!dest.join("graph.posegraph").exists());
    assert_eq!(std::fs::read(dest.join("graph.data")).unwrap(),b"keep");
}
#[test]
fn assigned_id_is_not_a_content_checksum() {
    let mut bytes=encoded(&metadata()); let (_,start)=read_structure(&mut Cursor::new(&bytes)).unwrap();
    bytes[start as usize]=b'X'; // Same length, valid structure: deliberately not detected.
    let (m,cells)=grid_structure(&mut Cursor::new(&bytes)).unwrap();
    assert_eq!(m,ArchiveMetadata::Assigned(metadata())); assert_eq!(cells,vec![0,100,-1]);
    let dir=Directory::new(); unpack_structure(&mut Cursor::new(bytes),&dir.0).unwrap();
    assert_eq!(std::fs::read(dir.join("graph.posegraph")).unwrap(),b"Xbc");
}
struct Counted { inner: Cursor<Vec<u8>>, read_bytes: usize, max_request: usize }
impl Counted { fn new(data: Vec<u8>) -> Self { Self { inner: Cursor::new(data),read_bytes:0,max_request:0 } } }
impl Read for Counted {
    fn read(&mut self,b:&mut [u8])->io::Result<usize> { self.max_request=self.max_request.max(b.len()); let n=self.inner.read(b)?; self.read_bytes+=n; Ok(n) }
}
impl Seek for Counted { fn seek(&mut self,p:SeekFrom)->io::Result<u64> { self.inner.seek(p) } }
#[test]
fn members_stream_once_and_header_grid_reads_skip_graph_bodies() {
    let mut m=metadata(); m.source.members[0].size=CHUNK_BYTES as u64+17;
    let mut a=Counted::new(vec![7;m.source.members[0].size as usize]);
    let mut b=Counted::new(b"def".to_vec()); let mut c=Counted::new(vec![0,100,255]);
    let mut bytes=Vec::new(); let size=write_assigned_members(&mut bytes,&m,&mut [&mut a,&mut b,&mut c]).unwrap();
    assert_eq!(size,bytes.len() as u64); assert_eq!(a.read_bytes,m.source.members[0].size as usize);
    assert_eq!(b.read_bytes,3); assert_eq!(c.read_bytes,3); assert!(a.max_request<=CHUNK_BYTES);
    let mut read=Counted::new(bytes); let (_,start)=read_structure(&mut read).unwrap();
    assert_eq!(read.read_bytes,start as usize);
    read.read_bytes=0; grid_structure(&mut read).unwrap(); assert_eq!(read.read_bytes,start as usize+3);
    read.read_bytes=0; let dir=Directory::new(); unpack_structure(&mut read,&dir.0).unwrap();
    assert_eq!(read.read_bytes,size as usize); assert!(read.max_request<=CHUNK_BYTES);
}
#[test]
fn legacy_static_digest_fields_are_structure_only() {
    // Fixed historical digest literals (abc and one NUL byte), never calculated here.
    // Only structural compatibility is asserted; legacy hash verification is intentionally unrun.
    const HEADER: &str = r#"{"version":1,"session":{"session_id":"s","map_name":"map","session_generation":1,"sensor_id":"laser","frame_id":"laser","map_frame_id":"map","odom_frame_id":"odom","body_frame_id":"base","clock_domain":"unix_ns","clock_id":"","calibration_ref":"calibration-1","odometry_source":"device","core_config":{"base_from_laser":[0,0,0]}},"estimate":{"source_timestamp_ns":0,"clock_domain":"","clock_id":"","source_sequence":0,"graph_revision":9,"optimizations":0,"anchor_odom_from_base":[0,0,0],"odom_from_base":[0,0,0],"map_from_base":[0,0,0],"map_from_odom":[0,0,0],"covariance":[0,0,0,0,0,0,0,0,0],"matching_response":0,"local_response":0,"disposition":"","node_count":0,"edge_count":0},"source_unix_timestamp_ns":100,"width":1,"height":1,"resolution":0.05,"origin":[1,2],"members":[{"name":"graph.posegraph","size":3,"sha256":[186,120,22,191,143,1,207,234,65,65,64,222,93,174,34,35,176,3,97,163,150,23,122,156,180,16,255,97,242,0,21,173]},{"name":"graph.data","size":3,"sha256":[186,120,22,191,143,1,207,234,65,65,64,222,93,174,34,35,176,3,97,163,150,23,122,156,180,16,255,97,242,0,21,173]},{"name":"grid.i8","size":1,"sha256":[110,52,11,156,255,179,122,152,156,165,68,230,187,120,10,44,120,144,29,63,179,55,56,118,133,17,163,6,23,175,160,29]}]}"#;
    let bytes=raw(MAGIC,HEADER.as_bytes(),b"abcabc\0");
    let (legacy,cells)=grid_structure(&mut Cursor::new(&bytes)).unwrap(); assert_eq!(cells,vec![0]);
    let ArchiveMetadata::Legacy(m)=legacy else { panic!("wrong version") };
    assert_eq!(m.members[0].sha256[0],0xba);
    let dir=Directory::new(); let path=dir.join("legacy"); std::fs::write(&path,&bytes).unwrap();
    assert_eq!(read_header(&mut open_regular(&path).unwrap()).unwrap().0,m);
    let dest=Directory::new(); unpack_structure(&mut Cursor::new(bytes),&dest.0).unwrap();
    assert_eq!(std::fs::read(dest.join("graph.posegraph")).unwrap(),b"abc");
}

#[test]
fn symlink_inputs_are_rejected() {
    let dir=Directory::new(); let paths=members(&dir); let link=dir.join("link");
    std::os::unix::fs::symlink(&paths[0],&link).unwrap();
    assert!(write_assigned(&dir.join("out"),&metadata(),[&link,&paths[1],&paths[2]]).is_err());
    assert!(open_regular(&dir.0).is_err());
}

#[test]
fn descriptor_binding_rejects_changed_checkpoint_format_and_size() {
    let m=metadata(); let bytes=encoded(&m);
    let id=ArtifactIdentity::Assigned {checkpoint_id:m.checkpoint_id.clone()};
    assert!(read_bound_structure(&mut Cursor::new(&bytes),bytes.len() as u64,&id).is_ok());
    assert!(read_bound_structure(&mut Cursor::new(&bytes),bytes.len() as u64,&ArtifactIdentity::Legacy).is_err());
    assert!(read_bound_structure(&mut Cursor::new(&bytes),bytes.len() as u64+1,&id).is_err());
    let other=ArtifactIdentity::Assigned {checkpoint_id:"2".repeat(32)};
    assert!(read_bound_structure(&mut Cursor::new(&bytes),bytes.len() as u64,&other).is_err());
}
#[test]
fn parsed_load_extracts_once_and_keeps_grid_for_native_comparison() {
    let bytes=encoded(&metadata());let size=bytes.len(); let mut reader=Counted::new(bytes);
    let (m,_)=read_bound_structure(&mut reader,size as u64,&ArtifactIdentity::Assigned {checkpoint_id:metadata().checkpoint_id}).unwrap();
    let dir=Directory::new(); let grid=unpack_parsed(&mut reader,&dir.0,&m).unwrap();
    assert_eq!(grid,vec![0,100,-1]);assert_eq!(reader.read_bytes,size);
    assert_eq!(m.identity(),ArtifactIdentity::Assigned {checkpoint_id:metadata().checkpoint_id});
}
