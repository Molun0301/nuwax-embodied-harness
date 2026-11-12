//! Storage qualification: counts are already associated input, not real capture.
use super::*;
use heightfield_observations::HeightfieldColumn;
use std::io::{Cursor, Read};
const BUDGET: usize = 4 * 1024 * 1024;
const Q: f32 = 0.05;
fn domain() -> Domain { Domain { session: "heightfield-storage".into(), } }
fn writer() -> Writer { Writer::new(Repository::temporary().unwrap(), Q, BUDGET).unwrap() }
fn surface(xy: [i32; 2], zs: &[i32]) -> HeightfieldSurface {
    HeightfieldSurface { column: HeightfieldColumn::from_counts(xy, Q,
        zs.iter().map(|&z| (z, [1,0,0,0,0,0,0,0])).collect()).unwrap().unwrap(), samples: vec![] }
}
fn column(s: HeightfieldSurface) -> ColumnGroundRecords { ColumnGroundRecords { classified_empty: false,  heightfields: vec![s] } }
fn install(w: &mut Writer, value: &ColumnGroundRecords, stamp: u64) {
    for s in &value.heightfields {
        let xy = s.column.xy();
        w.observe(s.column.members().map(|(z,_)| [xy[0],xy[1],z]), stamp).unwrap();
    }
    w.replace_ground_column(value.heightfields[0].column.xy(), value).unwrap();
}
fn receive(r: &mut Receiver, root: &Manifest, source: &Repository) -> Result<Arc<Manifest>, String> {
    r.receive(&Transfer { root: root.encode()?, repository_path: source.path().to_string_lossy().into_owned() }, &root.domain)
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
fn compact_heightfield_preserves_logical_identity_and_float_bits() {
    let source = Arc::new(LayerSource::Lidar {
        backend: 1,
        runtime_generation: i64::MAX,
        sensor_frame_id: "sensor-原始".into(),
    });
    let mut value = column(surface([0, 0], &[0]));
    value.heightfields[0].samples = (0..3)
        .map(|i| LayerSample {
            source: Arc::new(source.as_ref().clone()),
            point: [
                -0.0,
                f64::from_bits(1),
                f64::from_bits(0x3f847ae147ae147b + i),
            ],
            source_sequence: u64::MAX - i,
            source_timestamp_ns: u64::MAX,
            point_index: u32::MAX - i as u32,
        })
        .collect();
    value.heightfields[0].samples.extend([
        LayerSample {
            source: Arc::new(LayerSource::Auxiliary {
                sensor_id: "sensor-原始".into(),
                clock_id: "独立时钟".into(),
                runtime_generation: u64::MAX,
                session_generation: u64::MAX,
            }),
            point: [0.001; 3],
            source_sequence: 0,
            source_timestamp_ns: u64::MAX,
            point_index: u32::MAX,
        },
        LayerSample {
            source: Arc::new(LayerSource::Lidar { backend: 1, runtime_generation: 1, sensor_frame_id: "test".into() }),
            point: [0.001; 3],
            source_sequence: 1,
            source_timestamp_ns: u64::MAX,
            point_index: u32::MAX,
        },
    ]);
    let mut w = writer();
    install(&mut w, &value, u64::MAX);
    w.record_source_watermark(source.clone(), u64::MAX, u64::MAX)
        .unwrap();
    let root = w.snapshot(domain(), 1, u64::MAX).unwrap();
    let compact = w
        .repository()
        .read_object(root.blocks[0].object, MAX_BLOCK_BYTES)
        .unwrap();
    let (block, actual) = decode_block_parts(&compact).unwrap();
    for records in [&actual] {
        let GroundRecordRef::HeightfieldV1(decoded) = records.iter().next().unwrap().1 else {
            panic!();
        };
        for (original, decoded) in value.heightfields[0].samples.iter().zip(&decoded.samples) {
            assert_eq!(
                original.point.map(f64::to_bits),
                decoded.point.map(f64::to_bits)
            );
            assert_eq!(original, decoded);
        }
        assert!(Arc::ptr_eq(
            &decoded.samples[0].source,
            &decoded.samples[1].source
        ));
    }
    // 两种字节表示使用独立接收库；同一清单、来源水位及完整逻辑记录相同。
    for (name, payload) in [("紧凑高度场.ar5", compact)] {
        let repository = Repository::temporary().unwrap();
        repository.put_object(root.blocks[0].object, &payload).unwrap();
        let mut receiver = Receiver::new(repository.read_only());
        let received = receiver
            .receive(
                &Transfer {
                    root: root.encode().unwrap(),
                    repository_path: repository.path().to_string_lossy().into_owned(),
                },
                &root.domain)
            .unwrap();
        assert_eq!(*received, *root);
        assert_eq!(assembled(&mut receiver, &received, [0, 0]), value);
        let bytes = archive_bytes(&received, &repository);
        artifact(name, &bytes);
        let restored_repository = Repository::temporary().unwrap();
        let restored = ArchiveReader::open(Cursor::new(bytes))
            .unwrap()
            .import(&restored_repository)
            .unwrap();
        let mut copy = Writer::new(restored_repository, Q, BUDGET).unwrap();
        copy.restore_archived(&restored).unwrap();
        assert_eq!(copy.ground_column([0, 0]).unwrap(), value);
        assert_eq!(copy.source_watermark(&source), Some((u64::MAX, u64::MAX)));
    }
}

#[test]
fn compact_heightfield_full_block_retains_all_32_samples_with_original_byte_budget() {
    let stamp = 1_792_583_310_123_456_789;
    let mut block = Block::new();
    let mut records = BTreeMap::new();
    for local in 0..1024_u16 {
        let xy = [i32::from(local % 32), i32::from(local / 32)];
        block.insert(
            local,
            Evidence {
                observed_ns: stamp,
                hits: 32,
            },
        );
        records.insert(
            local,
            GroundRecord::HeightfieldV1(Arc::new(HeightfieldFragment {
                anchor: [xy[0], xy[1], 0],
                height_m: f64::from(Q) / 16.0,
                occupied_bounds: [0.0, f64::from(Q)],
                members: vec![(local, [1., 0., 0., 0., 0., 0., 0., 0.])],
                samples: (0..32)
                    .map(|i| LayerSample {
                        source: Arc::new(LayerSource::Lidar {
                            backend: 1,
                            runtime_generation: 1,
                            sensor_frame_id: "sensor".into(),
                        }),
                        point: [
                            (f64::from(xy[0]) + 0.5) * f64::from(Q),
                            (f64::from(xy[1]) + 0.5) * f64::from(Q),
                            0.001,
                        ],
                        source_sequence: i + 1,
                        source_timestamp_ns: stamp - 32 + i,
                        point_index: u32::from(local),
                    })
                    .collect(),
            })),
        );
    }
    let records = GroundRecords::Versioned(Arc::new(records));
    let encoded = encode_ground_block(&block, &records).unwrap();
    assert!(encoded.len() < MAX_BLOCK_BYTES);
    assert_eq!(decode_ground_records(&encoded).unwrap(), records);
    assert_eq!(decoded_block_stats(&encoded).unwrap(), (1024, stamp));
    eprintln!(
        "COMPACT full_columns=1024 samples_per_column=32 bytes={}",
        encoded.len()
    );
}

#[test]
fn compact_heightfield_rejects_invalid_source_tables() {
    let fragment = HeightfieldFragment {
        anchor: [0, 0, 0],
        height_m: 0.001,
        occupied_bounds: [0., f64::from(Q)],
        members: vec![(0, [1., 0., 0., 0., 0., 0., 0., 0.])],
        samples: vec![LayerSample {
            source: Arc::new(LayerSource::Lidar {
                backend: 1,
                runtime_generation: 1,
                sensor_frame_id: "sensor".into(),
            }),
            point: [0.001; 3],
            source_sequence: 1,
            source_timestamp_ns: 20,
            point_index: 0,
        }],
    };
    let original = serde_json::to_value(&fragment).unwrap();
    for case in 0..6 {
        let mut value = original.clone();
        let samples = &mut value["samples"];
        let first_source = samples["sources"][0].clone();
        match case {
            0 => samples["sources"]
                .as_array_mut()
                .unwrap()
                .push(first_source),
            1 => samples["values"][0][0] = 1.into(),
            2 => samples["values"] = serde_json::json!([]),
            3 => samples["values"] = serde_json::json!(vec![samples["values"][0].clone(); 33]),
            4 => samples["sources"][0]["runtime_generation"] = 0.into(),
            _ => samples["extra"] = true.into(),
        }
        assert!(
            serde_json::from_value::<HeightfieldFragment>(value).is_err(),
            "case={case}"
        );
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
    assert!(root.blocks.iter().all(|r| w.repository().read_object(r.object,MAX_BLOCK_BYTES).unwrap().starts_with(b"NUWAXBL6")));
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
fn heightfield_receiver_keeps_object_binding() {
    let mut w=writer(); install(&mut w,&column(surface([0,0],&[0,32])),20); let root=w.snapshot(domain(),1,20).unwrap();
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
    // 最大合法块超过原读取缓存，仍须证明正式封图不回读、接收仅尝试缺块。
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
    assert_eq!(seal_reads,0,"封图只比较已保留的原块和脏块，不回读");
    receive(&mut r,&second,&repo).unwrap(); let receive_reads=r.reader.read_count();
    assert_eq!(receive_reads,1,"只读仓库中的对象一次，不为校验回读");
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
    assert_eq!(records.iter().count(),1024); assert!(bytes.len()<MAX_BLOCK_BYTES); assert_eq!(decoded_block_stats(&bytes).unwrap(),(1024,20));
    let archive=archive_bytes(&root,w.repository()); artifact("full-1024.ar5",&archive);
    let repo=Repository::temporary().unwrap(); let restored=ArchiveReader::open(Cursor::new(archive)).unwrap().import(&repo).unwrap();
    let mut copy=Writer::new(repo,Q,BUDGET).unwrap(); copy.restore_archived(&restored).unwrap(); assert_eq!(copy.ground_column([31,31]).unwrap(),column(surface([31,31],&[0])));
    let (block,records)=decode_block_parts(&bytes).unwrap(); let mut oversized=match &records { GroundRecords::Versioned(values) => values.as_ref().clone() };
    // Real codec byte bound, without changing the original 4MiB limit.
    for record in oversized.values_mut() { if let GroundRecord::HeightfieldV1(f)=record {
        Arc::make_mut(f).samples=vec![LayerSample {source:Arc::new(LayerSource::Auxiliary {sensor_id:"x".repeat(5000),clock_id:"clock".into(),runtime_generation:1,session_generation:1}),
            point:[0.,0.,0.],source_sequence:0,source_timestamp_ns:20,point_index:0}];
    }}
    assert!(ground_binary::encode(&block, &oversized).unwrap_err().contains("budget"));
    let records = GroundRecords::Versioned(Arc::new(oversized));
    let bytes = encode_ground_block(&block, &records).unwrap();
    assert_eq!(decode_ground_records(&bytes).unwrap(), records, "旧格式超限案例现在完整存储");
    eprintln!("H2 full_columns=1024 encoded_block_bytes={} byte_budget={MAX_BLOCK_BYTES}",bytes.len());
}

// Claim-only changes retain historical diagnostics backed by real occupancy.
struct H3DetachedFixture {
    writer: Writer,
    before: Arc<Manifest>,
    after: Arc<Manifest>,
    original: LayerObservation,
    detached: ColumnGroundRecords,
}



// Malformed-fixture builders only. Never used to manufacture the green archive.
// They deliberately bypass ground validation while retaining valid occupancy,
// JSON serialization, header lengths, and root envelopes.
fn h3_mutation_block(block: &Block, records: &BTreeMap<u16, GroundRecord>) -> Vec<u8> {
    let json = serde_json::to_vec(&records.iter().collect::<Vec<_>>()).unwrap();
    let mut bytes = encode_block(block).unwrap();
    bytes[..8].copy_from_slice(b"NUWAXBL4");
    bytes[12..16].copy_from_slice(&(json.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&json);
    bytes
}

fn h3_mutation_archive(root: &Manifest, block: &[u8]) -> Vec<u8> {
    assert_eq!(root.version, 5);
    assert_eq!(root.blocks.len(), 1);
    let json = root.encode().unwrap();
    let mut bytes = b"NUWAXAR5".to_vec();
    bytes.extend_from_slice(&(json.len() as u64).to_le_bytes());
    bytes.extend_from_slice(&json);
    bytes.extend_from_slice(&1_u64.to_le_bytes());
    bytes.extend_from_slice(&(block.len() as u32).to_le_bytes());
    bytes.extend_from_slice(block);
    bytes
}





#[test]
fn classified_empty_column_roundtrips_moves_carrier_and_replaces_atomically() {
    let xy = [0,0]; let empty = ColumnGroundRecords {classified_empty:true,heightfields:vec![]};
    let mut w=writer(); w.observe([[0,0,0],[0,0,32],[0,0,64],[1,0,0]],20).unwrap();
    let old=w.snapshot(domain(),1,20).unwrap();
    w.replace_ground_column(xy,&empty).unwrap();
    let marked=w.snapshot(domain(),2,20).unwrap();
    assert!(marked.geometry_revision>old.geometry_revision);
    let mut receiver=Receiver::new(Repository::temporary().unwrap());
    receive(&mut receiver,&old,w.repository()).unwrap();
    let pinned=receive(&mut receiver,&marked,w.repository()).unwrap();
    assert_eq!(assembled(&mut receiver,&pinned,xy),empty);
    assert!(!assembled(&mut receiver,&pinned,[1,0]).classified_empty);
    assert_eq!(marked.points(),4);
    let bytes=archive_bytes(&marked,w.repository());artifact("h3-classified-empty.ar5",&bytes);
    let repo=Repository::temporary().unwrap();
    let archived=ArchiveReader::open(Cursor::new(bytes)).unwrap().import(&repo).unwrap();
    let mut copy=Writer::new(repo,Q,BUDGET).unwrap();copy.restore_archived(&archived).unwrap();
    assert_eq!(copy.ground_column(xy).unwrap(),empty);
    for (revision,z,next_z) in [(3,0,32),(4,32,64)] {
        copy.remove([[0,0,z]]).unwrap();
        let root=copy.snapshot(domain(),revision,20).unwrap();
        let mut r=RegionReader::new(copy.repository().clone(),0);
        let markers:Vec<_>=root.blocks.iter().flat_map(|b| r.ground_records(b).unwrap().iter()
            .filter_map(|(_,record)|matches!(record,GroundRecordRef::ClassifiedEmptyColumn).then_some(b.key)).collect::<Vec<_>>()).collect();
        assert_eq!(markers,vec![[0,0,next_z/32]]);
        assert_eq!(copy.ground_column(xy).unwrap(),empty);
    }
    let actual=column(surface(xy,&[64]));
    copy.replace_ground_column(xy,&actual).unwrap();
    assert_eq!(copy.ground_column(xy).unwrap(),actual);
    copy.snapshot(domain(),5,20).unwrap();
    copy.replace_ground_column(xy,&empty).unwrap();
    assert_eq!(copy.ground_column(xy).unwrap(),empty);
    copy.snapshot(domain(),6,20).unwrap();
    copy.remove([[0,0,64]]).unwrap();
    let last=copy.snapshot(domain(),7,20).unwrap();
    assert_eq!(last.points(),1); // Unconverted neighbour remains ordinary occupancy.
    assert!(!copy.ground_column(xy).unwrap().classified_empty);
    assert!(copy.ground_column(xy).unwrap().heightfields.is_empty());
    assert_eq!(assembled(&mut receiver,&pinned,xy),empty);
    eprintln!("EMPTY_COLUMN persisted=true carrier=0,32,64 real_claim_replaced=true all_occupancy_deleted_clears=true old_neighbor_legacy=true");
}
