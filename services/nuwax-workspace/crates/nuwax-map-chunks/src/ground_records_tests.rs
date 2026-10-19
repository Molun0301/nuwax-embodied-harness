//! Storage qualification: counts are already associated input, not real capture.
use super::*;
use heightfield_observations::HeightfieldColumn;
use std::io::{Cursor, Read};
const BUDGET: usize = 4 * 1024 * 1024;
const Q: f32 = 0.05;
fn domain() -> Domain { Domain { session: "heightfield-storage".into(), generation: 1, frame_generation: 0, producer_generation: 1 } }
fn writer() -> Writer { Writer::new(Repository::temporary().unwrap(), Q, BUDGET).unwrap() }
fn surface(xy: [i32; 2], zs: &[i32]) -> HeightfieldSurface {
    HeightfieldSurface { column: HeightfieldColumn::from_counts(xy, Q,
        zs.iter().map(|&z| (z, [1,0,0,0,0,0,0,0])).collect()).unwrap().unwrap(), samples: vec![] }
}
fn column(s: HeightfieldSurface) -> ColumnGroundRecords { ColumnGroundRecords { legacy: vec![], heightfields: vec![s] } }
fn install(w: &mut Writer, value: &ColumnGroundRecords, stamp: u64) {
    for s in &value.heightfields {
        let xy = s.column.xy();
        w.observe(s.column.members().map(|(z,_)| [xy[0],xy[1],z]), stamp).unwrap();
    }
    w.replace_ground_column(value.heightfields[0].column.xy(), value).unwrap();
}
fn receive(r: &mut Receiver, root: &Manifest, source: &Repository) -> Result<Arc<Manifest>, String> {
    r.receive(&Transfer { root: root.encode()?, included: BTreeMap::new() }, &root.domain,
        |_, b| source.read_object(b.object, MAX_BLOCK_BYTES))
}
fn assembled(r: &mut Receiver, root: &Manifest, xy: [i32;2]) -> ColumnGroundRecords {
    let blocks: Vec<_> = root.blocks.iter().map(|b| (b.key,r.ground_records(b).unwrap())).collect();
    ground_records::assemble_column(xy, root.voxel_size_m, &blocks).unwrap()
}
fn archive_bytes(root: &Manifest, repo: &Repository) -> Vec<u8> {
    let (mut file, size) = archive_file(root, repo).unwrap();
    let mut bytes = Vec::new(); file.read_to_end(&mut bytes).unwrap(); assert_eq!(bytes.len() as u64,size); bytes
}
fn artifact(name: &str, bytes: &[u8]) {
    if let Some(dir) = std::env::var_os("NEH_HEIGHTFIELD_ARCHIVE_DIR") {
        std::fs::create_dir_all(&dir).unwrap(); std::fs::write(std::path::Path::new(&dir).join(name),bytes).unwrap();
    }
}

#[test]
fn heightfield_cross_block_writer_receiver_archive_restore_exact_members() {
    let mut w = writer();
    let xy = [-1, 3]; let mut value = column(surface(xy, &[-33,-1,0,32,65]));
    let source = Arc::new(LayerSource::Lidar { backend: 2, runtime_generation: 7, sensor_frame_id: "lidar".into() });
    value.heightfields[0].samples = vec![LayerSample { source: source.clone(), point: [-0.025,0.175,0.001],
        source_sequence: 3, source_timestamp_ns: 20, point_index: 4 }];
    install(&mut w,&value,20); w.record_source_watermark(source.clone(),20,3).unwrap();
    let root = w.snapshot(domain(),1,20).unwrap(); assert_eq!(root.version,5); assert_eq!(root.blocks.len(),5);
    let mut receiver = Receiver::new(Repository::temporary().unwrap());
    let pinned = receive(&mut receiver,&root,w.repository()).unwrap();
    assert_eq!(assembled(&mut receiver,&pinned,xy),value);
    assert!(root.blocks.iter().all(|r| w.repository().read_object(r.object,MAX_BLOCK_BYTES).unwrap().starts_with(b"NUWAXBL4")));
    let bytes = archive_bytes(&root,w.repository()); assert!(bytes.starts_with(b"NUWAXAR5")); artifact("cross-block.ar5",&bytes);
    let repo = Repository::temporary().unwrap(); let path = repo.path().to_path_buf();
    let restored = ArchiveReader::open(Cursor::new(bytes)).unwrap().import(&repo).unwrap();
    let mut copy = Writer::new(repo.clone(),Q,BUDGET).unwrap(); copy.restore_archived(&restored).unwrap();
    assert_eq!(copy.ground_column(xy).unwrap(),value); assert_eq!(copy.source_watermark(&source),Some((20,3)));
    assert_eq!(copy.snapshot(domain(),2,20).unwrap().version,5);
    drop(copy); drop(repo); assert!(!path.exists());
    eprintln!("H2 cross_block=5 complete_members=5 source_watermark=retained cleanup_returned=true");
}

#[test]
fn heightfield_deletion_reanchors_all_fragments_and_preserves_pinned_root() {
    let mut w=writer(); let xy=[0,0]; let value=column(surface(xy,&[-33,0,33])); install(&mut w,&value,20);
    let first=w.snapshot(domain(),1,20).unwrap(); let mut r=Receiver::new(Repository::temporary().unwrap());
    let pinned=receive(&mut r,&first,w.repository()).unwrap();
    w.remove([[0,0,-33]]).unwrap(); let second=w.snapshot(domain(),2,20).unwrap();
    let after=receive(&mut r,&second,w.repository()).unwrap();
    assert_eq!(assembled(&mut r,&pinned,xy),value);
    let next=assembled(&mut r,&after,xy); assert_eq!(next.heightfields[0].column.members().map(|p|p.0).collect::<Vec<_>>(),vec![0,33]);
    for b in &after.blocks { for (_,record) in r.ground_records(b).unwrap().iter() {
        if let GroundRecordRef::HeightfieldV1(f)=record { assert_eq!(f.anchor,[0,0,0]); assert_eq!(f.occupied_bounds,next.heightfields[0].column.occupied_bounds()); }
    }}
    w.remove([[0,0,0],[0,0,33]]).unwrap(); let empty=w.snapshot(domain(),3,20).unwrap();
    assert!(empty.blocks.is_empty()); assert!(w.ground_column(xy).unwrap().heightfields.is_empty());
    assert_eq!(receive(&mut r,&empty,w.repository()).unwrap().geometry_revision,3);
    assert_eq!(assembled(&mut r,&pinned,xy),value);
}

#[test]
fn heightfield_metadata_and_zero_mass_members_have_distinct_geometry_revisions() {
    let mut w=writer(); let xy=[0,0];
    let make=|a,b| column(HeightfieldSurface { column: HeightfieldColumn::from_normalized_members(xy,Q,
        [(0,[a,b,0.,0.,0.,0.,0.,0.]),(32,[0.;8]),(64,[0.;8])].into()).unwrap(), samples: vec![] });
    let first_value=make(0.75,0.25); install(&mut w,&first_value,20); let first=w.snapshot(domain(),1,20).unwrap();
    let metadata=make(0.625,0.375); assert!(first_value.heightfields[0].column.geometry_eq(&metadata.heightfields[0].column));
    w.replace_ground_column(xy,&metadata).unwrap(); let second=w.snapshot(domain(),2,20).unwrap();
    assert_eq!(second.geometry_revision,first.geometry_revision);
    assert_ne!(second.blocks[0].object,first.blocks[0].object);
    // A zero-mass middle fragment may legally withdraw ground membership while
    // its original occupancy stays. No count field invents a missing-packet error.
    let withdrawn=column(HeightfieldSurface {column:HeightfieldColumn::from_normalized_members(xy,Q,
        [(0,[0.625,0.375,0.,0.,0.,0.,0.,0.]),(64,[0.;8])].into()).unwrap(),samples:vec![]});
    w.replace_ground_column(xy,&withdrawn).unwrap(); let third=w.snapshot(domain(),3,20).unwrap();
    assert_eq!(third.geometry_revision,3); assert_eq!(third.points(),3);
    let mut r=Receiver::new(Repository::temporary().unwrap()); receive(&mut r,&second,w.repository()).unwrap();
    receive(&mut r,&third,w.repository()).unwrap(); assert_eq!(assembled(&mut r,&third,xy),withdrawn);
    let raised=column(HeightfieldSurface {column:HeightfieldColumn::from_normalized_members(xy,Q,
        [(0,[0.,0.,0.,0.,1.,0.,0.,0.]),(64,[0.;8])].into()).unwrap(),samples:vec![]});
    w.replace_ground_column(xy,&raised).unwrap(); let fourth=w.snapshot(domain(),4,20).unwrap(); assert_eq!(fourth.geometry_revision,4);
    artifact("zero-mass-withdrawn.ar5",&archive_bytes(&third,w.repository()));
}

#[test]
fn heightfield_legacy_arc_and_format_boundaries_remain_explicit() {
    let mut w=writer(); let xy=[1,0]; w.observe([[1,0,0]],20).unwrap();
    let sample=LayerSample {source:Arc::new(LayerSource::Legacy),point:[0.075,0.025,0.001],source_sequence:1,source_timestamp_ns:20,point_index:0};
    let legacy=LayerObservation {members:vec![1],plane_origin:[0.075,0.025,0.001],normal:[0.,0.,1.],samples:vec![sample]};
    w.replace_layer_observations([0,0,0],[(1,legacy.clone())].into()).unwrap(); let old=w.snapshot(domain(),1,20).unwrap();
    assert_eq!(old.version,4); let mut reader=RegionReader::new(w.repository().clone(),BUDGET);
    let a=reader.layers(&old.blocks[0]).unwrap(); let b=reader.ground_records(&old.blocks[0]).unwrap().legacy().unwrap(); assert!(Arc::ptr_eq(&a,&b));
    assert_eq!(a[&1],legacy); artifact("legacy.ar4",&archive_bytes(&old,w.repository()));
    let new=column(surface([0,0],&[0])); install(&mut w,&new,20); let root=w.snapshot(domain(),2,20).unwrap();
    assert!(reader.layers(&root.blocks[0]).is_err());
    let records=reader.ground_records(&root.blocks[0]).unwrap(); assert_eq!(records.iter().count(),2);
    assert_eq!(w.ground_column(xy).unwrap().legacy[0].1,legacy);
    let mut bad=(*root).clone(); bad.version=4;
    assert!(archive_file(&bad,w.repository()).is_err());
    let mut r=Receiver::new(Repository::temporary().unwrap()); assert!(receive(&mut r,&bad,w.repository()).is_err()); assert!(r.current().is_none());
    artifact("mixed.ar5",&archive_bytes(&root,w.repository()));
    // Actual zero-mass middle fragments have identical bytes. Root5/BL4
    // entry points must apply the existing Receiver spatial-binding rule.
    let mut owner=writer();
    let mass=HeightfieldColumn::from_normalized_members([0,0],Q,
        [(0,[1.,0.,0.,0.,0.,0.,0.,0.]),(32,[0.;8]),(64,[0.;8]),(96,[0.;8])].into()).unwrap();
    install(&mut owner,&column(HeightfieldSurface {column:mass,samples:vec![]}),20);
    let good=owner.snapshot(domain(),1,20).unwrap();
    let a=owner.repository().read_object(good.blocks[1].object,MAX_BLOCK_BYTES).unwrap();
    let b=owner.repository().read_object(good.blocks[2].object,MAX_BLOCK_BYTES).unwrap(); assert_eq!(a,b);
    let mut rebound=(*good).clone(); rebound.blocks[2].object=rebound.blocks[1].object;
    let mut restore=Writer::new(owner.repository().clone(),Q,BUDGET).unwrap();
    assert!(restore.restore(&rebound).unwrap_err().contains("spatial binding"));
    assert!(!restore.has_unpublished_changes()); assert_eq!(restore.block_count(),0);
    let bytes=archive_bytes(&rebound,owner.repository()); artifact("invalid-rebound.ar5",&bytes);
    assert!(ArchiveReader::open(Cursor::new(bytes)).unwrap().import(&Repository::temporary().unwrap()).unwrap_err().contains("spatial binding"));
    let mut receiver=Receiver::new(Repository::temporary().unwrap());
    let error=receive(&mut receiver,&rebound,owner.repository()).unwrap_err();
    eprintln!("H2 receiver_rebound_error={error}");
    assert_eq!(error,"layer object rebound to another spatial key/resolution");
    assert!(receiver.current().is_none());
}

#[test]
fn heightfield_root_admission_rejects_actual_mass_geometry_and_binding_errors() {
    let mut w=writer(); install(&mut w,&column(surface([0,0],&[0,32])),20); let root=w.snapshot(domain(),1,20).unwrap();
    for case in 0..5 {
        let mut bad=(*root).clone(); bad.revision=2; bad.geometry_revision=2;
        let original=&root.blocks[0]; let (block,records)=decode_block_parts(&w.repository().read_object(original.object,MAX_BLOCK_BYTES).unwrap()).unwrap();
        let mut changed=records.owned_versioned();
        let GroundRecord::HeightfieldV1(fragment)=changed.values_mut().next().unwrap() else { panic!("fixture"); };
        match case { 0=>fragment.members[0].1[0]*=0.5, 1=>fragment.height_m+=0.01,
            2=>fragment.occupied_bounds[1]+=0.05, 3=>fragment.anchor[2]-=1, _=>fragment.anchor[0]+=1 }
        let bytes=encode_ground_block(&block,&GroundRecords::Versioned(Arc::new(changed))).unwrap();
        let id=ObjectId::random().unwrap().into(); bad.blocks[0].object=id; bad.blocks[0].block_revision=2; bad.blocks[0].geometry_revision=2;
        let mut r=Receiver::new(Repository::temporary().unwrap()); let accepted=receive(&mut r,&root,w.repository()).unwrap();
        let result=r.receive(&Transfer {root:bad.encode().unwrap(),included:[(id,bytes)].into()},&domain(),|_,b|w.repository().read_object(b.object,MAX_BLOCK_BYTES));
        eprintln!("H2 malformed_case={case} error={result:?}"); assert!(result.is_err()); assert_eq!(r.current().unwrap(),accepted);
    }
    let mut r=Receiver::new(Repository::temporary().unwrap()); receive(&mut r,&root,w.repository()).unwrap();
    let mut rebound=(*root).clone(); rebound.revision=2; rebound.geometry_revision=2; for b in &mut rebound.blocks { b.key[0]+=1; b.block_revision=2; b.geometry_revision=2; }
    assert!(receive(&mut r,&rebound,w.repository()).unwrap_err().contains("rebound"));
}

#[test]
fn heightfield_ground_parts_use_one_decode_and_incremental_roots_skip_untouched_columns() {
    let mut w=writer(); install(&mut w,&column(surface([0,0],&[0,32])),20);
    install(&mut w,&column(surface([64,0],&[0,32])),20); let first=w.snapshot(domain(),1,20).unwrap();
    let repo=w.repository().clone(); let mut reader=RegionReader::new(repo.clone(),0); let reads=repo.counts().0;
    let (block,records)=reader.ground_parts(&first.blocks[0]).unwrap(); assert_eq!(repo.counts().0-reads,1); assert_eq!(block.len(),1); assert!(records.has_heightfield()); assert_eq!(reader.cached_bytes(),0);
    let mut warm=RegionReader::new(repo.clone(),BUDGET); let (_,a)=warm.ground_parts(&first.blocks[0]).unwrap(); let (_,b)=warm.ground_parts(&first.blocks[0]).unwrap();
    match(a,b) { (GroundRecords::Versioned(a),GroundRecords::Versioned(b))=>assert!(Arc::ptr_eq(&a,&b)), _=>panic!("typed fixture") }
    let target=Repository::temporary().unwrap(); let mut r=Receiver::with_cache_budget(target.clone(),0); receive(&mut r,&first,&repo).unwrap();
    let reads=target.counts().0; let duplicate=receive(&mut r,&first,&repo).unwrap(); assert_eq!(target.counts().0,reads); assert_eq!(duplicate,first);
    // No new root-wide payload pass on a metadata-only publication.
    let second=w.snapshot(domain(),2,20).unwrap(); let reads=target.counts().0; receive(&mut r,&second,&repo).unwrap(); assert_eq!(target.counts().0,reads);
    // Existing maximum-valid-block fixture makes the Writer's original 2MiB
    // reader cache too small. Measure the real seal/receive paths, not just the
    // accessor. The comparison needs two objects; root assembly also reads.
    let mut w=writer();
    w.observe((0..32768_u16).map(|local|global_key([0,0,0],local).unwrap()),20).unwrap();
    let make=|a,b| column(HeightfieldSurface { column:HeightfieldColumn::from_normalized_members([0,0],Q,
        [(0,[a,b,0.,0.,0.,0.,0.,0.])].into()).unwrap(),samples:vec![] });
    w.replace_ground_column([0,0],&make(0.75,0.25)).unwrap(); let first=w.snapshot(domain(),1,20).unwrap();
    let repo=w.repository().clone(); let target=Repository::temporary().unwrap(); let mut r=Receiver::with_cache_budget(target.clone(),0);
    receive(&mut r,&first,&repo).unwrap();
    w.replace_ground_column([0,0],&make(0.625,0.375)).unwrap();
    let reads=repo.counts().0; let second=w.snapshot(domain(),2,20).unwrap();
    let seal_reads=repo.counts().0-reads;
    assert_eq!(seal_reads,5,"2 comparison objects + current/previous changed groups + current assembly");
    let reads=target.counts().0; receive(&mut r,&second,&repo).unwrap(); let receive_reads=target.counts().0-reads;
    assert_eq!(receive_reads,6,"1 missing-object attempt + 3 root-validation reads + 2 comparison objects");
    assert_eq!(second.geometry_revision,first.geometry_revision);
    eprintln!("H2 uncacheable_points=32768 seal_reads={seal_reads} receiver_zero_cache_reads={receive_reads}");
}

#[test]
fn heightfield_column_preparation_and_original_flush_failure_do_not_publish_half_columns() {
    let mut w=writer(); let xy=[0,0]; let first_value=column(surface(xy,&[0,32,64])); install(&mut w,&first_value,20);
    let old=w.snapshot(domain(),1,20).unwrap(); let repo=w.repository().clone();
    let invalid=column(surface(xy,&[0,32,64,96])); assert!(w.replace_ground_column(xy,&invalid).is_err()); assert_eq!(w.ground_column(xy).unwrap(),first_value);
    w.remove([[0,0,0]]).unwrap(); let edited=w.ground_column(xy).unwrap(); repo.fail_put_after(Some(1));
    assert!(w.snapshot(domain(),2,20).is_err()); assert_eq!(w.ground_column(xy).unwrap(),edited);
    let mut old_reader=RegionReader::new(repo.clone(),0);
    let parts=old.blocks.iter().map(|b|(b.key,old_reader.ground_records(b).unwrap())).collect::<Vec<_>>();
    assert_eq!(ground_records::assemble_column(xy,Q,&parts).unwrap(),first_value);
    repo.fail_put_after(None); let next=w.snapshot(domain(),2,20).unwrap(); assert_eq!(next.version,5);
    let mut r=Receiver::new(Repository::temporary().unwrap()); receive(&mut r,&next,&repo).unwrap(); assert_eq!(assembled(&mut r,&next,xy),edited);
}

#[test]
fn heightfield_full_horizontal_block_and_existing_byte_budget_roundtrip() {
    let mut w=writer(); let keys=(0..32).flat_map(|x|(0..32).map(move |y|[x,y,0])).collect::<Vec<_>>();
    w.observe(keys,20).unwrap();
    let columns: Vec<_> = (0..32).flat_map(|x|(0..32).map(move |y|([x,y],column(surface([x,y],&[0]))))).collect();
    w.replace_ground_columns(columns.iter().map(|(xy,column)|(*xy,column))).unwrap();
    let root=w.snapshot(domain(),1,20).unwrap(); assert_eq!(root.blocks.len(),1);
    let bytes=w.repository().read_object(root.blocks[0].object,MAX_BLOCK_BYTES).unwrap(); let records=decode_ground_records(&bytes).unwrap();
    assert_eq!(records.iter().count(),1024); assert!(bytes.len()<MAX_BLOCK_BYTES); assert_eq!(block_summary(&bytes).unwrap(),(1024,20));
    let archive=archive_bytes(&root,w.repository()); artifact("full-1024.ar5",&archive);
    let repo=Repository::temporary().unwrap(); let restored=ArchiveReader::open(Cursor::new(archive)).unwrap().import(&repo).unwrap();
    let mut copy=Writer::new(repo,Q,BUDGET).unwrap(); copy.restore_archived(&restored).unwrap(); assert_eq!(copy.ground_column([31,31]).unwrap(),column(surface([31,31],&[0])));
    let (block,records)=decode_block_parts(&bytes).unwrap(); let mut oversized=records.owned_versioned();
    // Real codec byte bound, without changing the original 4MiB limit.
    for record in oversized.values_mut() { if let GroundRecord::HeightfieldV1(f)=record {
        f.samples=vec![LayerSample {source:Arc::new(LayerSource::Auxiliary {sensor_id:"x".repeat(5000),clock_id:"clock".into(),runtime_generation:1,session_generation:1}),
            point:[0.,0.,0.],source_sequence:0,source_timestamp_ns:20,point_index:0}];
    }}
    assert!(encode_ground_block(&block,&GroundRecords::Versioned(Arc::new(oversized))).unwrap_err().contains("budget"));
    eprintln!("H2 full_columns=1024 encoded_block_bytes={} byte_budget={MAX_BLOCK_BYTES}",bytes.len());
}
