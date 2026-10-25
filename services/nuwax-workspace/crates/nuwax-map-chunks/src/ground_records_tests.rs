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
fn column(s: HeightfieldSurface) -> ColumnGroundRecords { ColumnGroundRecords { classified_empty: false, legacy: vec![], heightfields: vec![s] } }
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

fn old_heightfield_block(block: &Block, records: &GroundRecords) -> Vec<u8> {
    let mut entries = Vec::new();
    for (key, record) in records.iter() {
        let GroundRecordRef::HeightfieldV1(fragment) = record else {
            panic!("heightfield fixture");
        };
        let mut value = serde_json::to_value(fragment).unwrap();
        value["samples"] = serde_json::to_value(&fragment.samples).unwrap();
        entries.push(serde_json::json!([key, {"representation":"heightfield_v1","data":value}]));
    }
    let json = serde_json::to_vec(&entries).unwrap();
    let mut bytes = encode_block(block).unwrap();
    bytes[..8].copy_from_slice(b"NUWAXBL4");
    bytes[12..16].copy_from_slice(&(json.len() as u32).to_le_bytes());
    bytes.extend(json);
    bytes
}

#[test]
fn compact_heightfield_reads_old_bl4_and_preserves_logical_identity_and_float_bits() {
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
            source: Arc::new(LayerSource::Legacy),
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
    let old = old_heightfield_block(&block, &actual);
    let legacy = decode_ground_records(&old).unwrap();
    assert_eq!(legacy, actual);
    assert!(legacy.geometry_eq(&actual));
    for records in [&legacy, &actual] {
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
    for (name, payload) in [("旧高度场.ar5", old), ("紧凑高度场.ar5", compact)] {
        let repository = Repository::temporary().unwrap();
        let mut receiver = Receiver::new(repository.clone());
        let received = receiver
            .receive(
                &Transfer {
                    root: root.encode().unwrap(),
                    included: BTreeMap::from([(root.blocks[0].object, payload)]),
                },
                &root.domain,
                |_, _| Err("included fixture".into()),
            )
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
            GroundRecord::HeightfieldV1(HeightfieldFragment {
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
            }),
        );
    }
    let records = GroundRecords::Versioned(Arc::new(records));
    let old = old_heightfield_block(&block, &records);
    assert!(old.len() > MAX_BLOCK_BYTES, "复现原持续采样会超出块预算");
    let encoded = encode_ground_block(&block, &records).unwrap();
    assert!(encoded.len() < MAX_BLOCK_BYTES);
    assert_eq!(decode_ground_records(&encoded).unwrap(), records);
    assert_eq!(block_summary(&encoded).unwrap(), (1024, stamp));
    eprintln!(
        "COMPACT full_columns=1024 samples_per_column=32 old_bytes={} new_bytes={}",
        old.len(),
        encoded.len()
    );
}

#[test]
fn compact_heightfield_rejects_invalid_source_tables_and_keeps_sample_validation() {
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
    let mut value = original;
    value["samples"]["values"][0][2] = 0.into();
    let invalid: HeightfieldFragment = serde_json::from_value(value).unwrap();
    let block = BTreeMap::from([(
        0,
        Evidence {
            observed_ns: 20,
            hits: 1,
        },
    )]);
    assert!(
        encode_ground_block(
            &block,
            &GroundRecords::Versioned(Arc::new(BTreeMap::from([(
                0,
                GroundRecord::HeightfieldV1(invalid)
            ),])))
        )
        .is_err(),
        "紧凑编码不跳过原始样本身份验证"
    );
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

// Claim-only changes retain historical diagnostics backed by real occupancy.
struct H3DetachedFixture {
    writer: Writer,
    before: Arc<Manifest>,
    after: Arc<Manifest>,
    original: LayerObservation,
    detached: ColumnGroundRecords,
}

fn h3_detached_fixture() -> H3DetachedFixture {
    let mut writer = writer();
    // The precise counterexample: two real occupied members in the same block.
    writer.observe([[0, 0, 0]], 10).unwrap();
    writer.observe([[0, 0, 1]], 20).unwrap();
    let original = LayerObservation {
        members: vec![0, 1024],
        plane_origin: [0.025, 0.025, 0.025],
        normal: [0.0, 0.0, 1.0],
        samples: vec![LayerSample {
            source: Arc::new(LayerSource::Lidar {
                backend: 2, runtime_generation: 7, sensor_frame_id: "lidar".into(),
            }),
            point: [0.025, 0.025, 0.075],
            source_sequence: 1, source_timestamp_ns: 20, point_index: 4,
        }],
    };
    writer.replace_layer_observations([0, 0, 0], [(0, original.clone())].into()).unwrap();
    let before = writer.seal(domain(), 1, 20).unwrap().root;
    let mut retained = original.clone();
    retained.members = vec![0]; // Withdraw only the ground claim, not occupancy.
    let detached = ColumnGroundRecords {
        classified_empty: false, legacy: vec![([0, 0, 0], retained)], heightfields: vec![],
    };
    writer.replace_ground_column([0, 0], &detached).unwrap();
    let after = writer.seal(domain(), 2, 20).unwrap().root;
    H3DetachedFixture { writer, before, after, original, detached }
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
fn h3_legacy_claim_withdrawal_retains_original_sample_across_storage() {
    let mut f = h3_detached_fixture();
    assert_eq!(f.before.version, 4);
    assert_eq!(f.after.version, 5);
    assert_eq!(f.after.blocks.len(), 1);
    assert_eq!(f.after.points(), 2);
    assert_eq!(f.after.geometry_revision, 2);
    assert_eq!(f.writer.ground_column([0, 0]).unwrap(), f.detached);
    let old_bytes = f.writer.repository().read_object(f.before.blocks[0].object, MAX_BLOCK_BYTES).unwrap();
    let new_bytes = f.writer.repository().read_object(f.after.blocks[0].object, MAX_BLOCK_BYTES).unwrap();
    assert!(old_bytes.starts_with(b"NUWAXBL3"));
    assert!(new_bytes.starts_with(b"NUWAXBL4"), "pure Legacy must not compact into invalid BL3");
    let (old_block, _) = decode_block_parts(&old_bytes).unwrap();
    let (block, records) = decode_block_parts(&new_bytes).unwrap();
    assert_eq!(block, old_block); // Includes original observed_ns and hit counts.
    assert_eq!(block, BTreeMap::from([
        (0, Evidence { observed_ns: 10, hits: 1 }),
        (1024, Evidence { observed_ns: 20, hits: 1 }),
    ]));
    assert!(matches!(&records, GroundRecords::Versioned(_)));
    assert!(!records.has_heightfield(), "an unrelated HF must not hide broken compact");
    assert_eq!(records.iter().count(), 1);
    let GroundRecordRef::LegacyPlane(retained) = records.get(0).unwrap() else { panic!("Legacy fixture"); };
    assert_eq!(retained, &f.detached.legacy[0].1);
    assert_eq!(retained.samples, f.original.samples);
    assert_eq!(retained.plane_origin, f.original.plane_origin);
    assert_eq!(retained.normal, f.original.normal);
    assert_eq!(retained.members, vec![0]);

    let mut receiver = Receiver::new(Repository::temporary().unwrap());
    let pinned_before = receive(&mut receiver, &f.before, f.writer.repository()).unwrap();
    let pinned_after = receive(&mut receiver, &f.after, f.writer.repository()).unwrap();
    assert_eq!(*receiver.block(&pinned_after.blocks[0]).unwrap(), block);
    assert_eq!(assembled(&mut receiver, &pinned_after, [0, 0]), f.detached);
    assert_eq!(assembled(&mut receiver, &pinned_before, [0, 0]).legacy[0].1, f.original);
    assert_eq!(receive(&mut receiver, &f.after, f.writer.repository()).unwrap(), pinned_after);

    // Only these archives are positive qualification artifacts: real Writer,
    // real seal, real repository bytes, and the production archive_file path.
    let old_archive = archive_bytes(&f.before, f.writer.repository());
    let archive = archive_bytes(&f.after, f.writer.repository());
    artifact("h3-detached-before.ar4", &old_archive);
    artifact("h3-detached-legacy.ar5", &archive);
    let mut archive_reader = ArchiveReader::open(Cursor::new(archive.clone())).unwrap();
    assert_eq!(archive_reader.block_by_key([0, 0, 0]).unwrap(), new_bytes);
    assert_eq!(archive_reader.block(&f.after.blocks[0]).unwrap(), new_bytes);
    assert_eq!(archive_reader.region([0, 0, 0], [0, 0, 1], 2).unwrap(), vec![
        ([0, 0, 0], Evidence { observed_ns: 10, hits: 1 }),
        ([0, 0, 1], Evidence { observed_ns: 20, hits: 1 }),
    ]);
    let repository = Repository::temporary().unwrap();
    let restored = ArchiveReader::open(Cursor::new(archive)).unwrap().import(&repository).unwrap();
    assert_eq!(restored, ArchivedRoot::Current((*f.after).clone()));
    let mut copy = Writer::new(repository.clone(), Q, BUDGET).unwrap();
    copy.restore_archived(&restored).unwrap();
    assert_eq!(copy.ground_column([0, 0]).unwrap(), f.detached);
    let resealed = copy.seal(domain(), 3, 20).unwrap().root;
    assert_eq!(resealed.version, 5);
    assert_eq!(resealed.blocks, f.after.blocks);
    assert_eq!(repository.read_object(resealed.blocks[0].object, MAX_BLOCK_BYTES).unwrap(), new_bytes);

    // The same detached record is still outside the old BL3 contract. Exercise
    // both old encoding and old decoding, rather than merely changing a root tag.
    let legacy: LayerObservations = [(0, f.detached.legacy[0].1.clone())].into();
    assert_eq!(encode_layer_block(&block, &legacy).unwrap_err(), "layer observation exceeds occupancy evidence");
    let old_start = 16 + block.len() * 12;
    let mut old_json: serde_json::Value = serde_json::from_slice(&old_bytes[old_start..]).unwrap();
    old_json[0][1]["members"] = serde_json::json!([0]); // The sole BL3 mutation.
    let json = serde_json::to_vec(&old_json).unwrap();
    let mut bad_bl3 = old_bytes[..old_start].to_vec();
    bad_bl3[12..16].copy_from_slice(&(json.len() as u32).to_le_bytes());
    bad_bl3.extend_from_slice(&json);
    assert_eq!(decode_layer_observations(&bad_bl3).unwrap_err(), "layer observation exceeds occupancy evidence");

    // Subsequent real occupancy deletion must remove its only diagnostic sample.
    // The old pinned root still owns the unchanged detached historical record.
    f.writer.remove([[0, 0, 1]]).unwrap();
    assert!(f.writer.ground_column([0, 0]).unwrap().legacy.is_empty());
    let deleted = f.writer.seal(domain(), 3, 20).unwrap().root;
    assert_eq!(deleted.points(), 1);
    let live = receive(&mut receiver, &deleted, f.writer.repository()).unwrap();
    assert_eq!(*receiver.block(&live.blocks[0]).unwrap(), [(0, block[&0])].into());
    assert_eq!(assembled(&mut receiver, &pinned_after, [0, 0]), f.detached);
    eprintln!("H3_DETACHED green=writer_seal_receiver_archive_read_import_restore unchanged_sample=20 occupancy=0:10,1024:20 legacy_bl3_rejected=true");
}

#[test]
fn h3_detached_legacy_samples_reject_forged_evidence_at_admission() {
    let f = h3_detached_fixture();
    let original_bytes = f.writer.repository().read_object(f.after.blocks[0].object, MAX_BLOCK_BYTES).unwrap();
    let (original_block, original_records) = decode_block_parts(&original_bytes).unwrap();
    // All numbers below are fixed mutation controls, not calibration sweeps.
    for case in ["future-sample", "occupied-wrong-voxel", "missing-voxel",
                 "other-block", "f32-resolution", "invalid-source"] {
        let mut block = original_block.clone();
        let mut records = original_records.owned_versioned();
        let GroundRecord::LegacyPlane(layer) = records.get_mut(&0).unwrap() else { panic!("Legacy fixture"); };
        match case {
            "future-sample" => layer.samples[0].source_timestamp_ns = 21,
            "occupied-wrong-voxel" => {
                // The sample stays at z1/stamp20; its actual voxel is only 19.
                // An unrelated z2/stamp20 deliberately preserves block-max20.
                block.get_mut(&1024).unwrap().observed_ns = 19;
                block.insert(2048, Evidence { observed_ns: 20, hits: 1 });
            }
            "missing-voxel" => {
                // Move only the occupancy evidence to a decoy z2. The sample
                // stays at z1/stamp20 and member0/plane/source stay unchanged.
                block.remove(&1024);
                block.insert(2048, Evidence { observed_ns: 20, hits: 1 });
            }
            "other-block" => layer.samples[0].point[2] = f64::from(Q) * 33.5,
            "f32-resolution" => layer.samples[0].point[2] = 0.05,
            "invalid-source" => {
                let LayerSource::Lidar { backend, .. } = Arc::make_mut(&mut layer.samples[0].source) else { panic!("typed fixture"); };
                *backend = 0; // Explicit negative mutation of the original legal typed source.
            }
            _ => unreachable!(),
        }
        let candidate_column = ColumnGroundRecords { classified_empty: false, legacy: vec![([0, 0, 0], layer.clone())], heightfields: vec![] };
        assert_eq!(candidate_column.legacy[0].1.members, vec![0]);
        assert_eq!(candidate_column.legacy[0].1.plane_origin, f.original.plane_origin);
        assert_eq!(candidate_column.legacy[0].1.normal, f.original.normal);
        if case == "f32-resolution" {
            assert_eq!((0.05_f64 * (1.0 / f64::from(Q))).floor(), 0.0);
            assert_eq!((0.05_f64 * (1.0 / 0.05_f64)).floor(), 1.0);
        }
        let bytes = h3_mutation_block(&block, &records);
        let intrinsic_ok = !matches!(case, "future-sample" | "invalid-source");
        assert_eq!(decode_block_parts(&bytes).is_ok(), intrinsic_ok, "{case}: context-free boundary");
        assert_eq!(encode_ground_block(&block, &GroundRecords::Versioned(Arc::new(records))).is_ok(),
                   intrinsic_ok, "{case}: encoder boundary");

        // Writer admission sees exactly the mutated real occupied Block. Its
        // successful baseline has no ground records; no sample time is lowered
        // or synthesized to obtain that baseline. Failure must not publish any.
        let mut admission = writer();
        for (&local, evidence) in &block {
            admission.observe([global_key([0, 0, 0], local).unwrap()], evidence.observed_ns).unwrap();
        }
        let clean = admission.seal(domain(), 1, 20).unwrap().root;
        assert!(admission.replace_ground_column([0, 0], &candidate_column).is_err(), "{case}: Writer preparation");
        assert!(!admission.has_unpublished_changes(), "{case}: rejected preparation dirtied writer");
        assert!(admission.ground_column([0, 0]).unwrap().legacy.is_empty());
        let unchanged = admission.seal(domain(), 2, 20).unwrap().root;
        assert_eq!(unchanged.version, 4);
        assert_eq!(unchanged.blocks, clean.blocks);

        let id: ObjectIdentity = ObjectId::random().unwrap().into();
        let mut bad = (*f.after).clone();
        bad.revision = 3;
        bad.geometry_revision = 3;
        // Keep a future-sample mutation within the root envelope so failure
        // proves the block's occupancy bound, not an unrelated root-time check.
        if case == "future-sample" { bad.source_timestamp_ns = 21; }
        bad.blocks[0].object = id;
        bad.blocks[0].points = block.len() as u32;
        bad.blocks[0].block_revision = 3;
        bad.blocks[0].geometry_revision = 3;
        bad.validate().unwrap();
        for route in ["included", "fetched", "preexisting"] {
            let target = Repository::temporary().unwrap();
            let mut receiver = Receiver::new(target.clone());
            let accepted = receive(&mut receiver, &f.after, f.writer.repository()).unwrap();
            if route == "preexisting" { target.put_object(id, &bytes).unwrap(); }
            let included = if route == "included" { [(id, bytes.clone())].into() } else { BTreeMap::new() };
            let mut fetches = 0;
            let result = receiver.receive(&Transfer { root: bad.encode().unwrap(), included }, &domain(), |_, reference| {
                assert_eq!(reference.object, id);
                fetches += 1;
                Ok(bytes.clone())
            });
            assert!(result.is_err(), "{case}: Receiver {route}");
            assert_eq!(fetches, usize::from(route == "fetched"), "{case}: wrong Receiver route");
            assert_eq!(receiver.current().unwrap(), accepted, "{case}: replaced current root");
            assert_eq!(assembled(&mut receiver, &accepted, [0, 0]), f.detached);
        }

        let archive = h3_mutation_archive(&bad, &bytes);
        artifact(&format!("h3-detached-{case}.ar5"), &archive);
        // Each entry point gets a fresh reader. open() is intentionally lazy
        // and is expected to accept the well-formed envelope before block read.
        assert!(ArchiveReader::open(Cursor::new(archive.clone())).unwrap().block_by_key([0, 0, 0]).is_err(), "{case}: archive block_by_key");
        assert!(ArchiveReader::open(Cursor::new(archive.clone())).unwrap().block(&bad.blocks[0]).is_err(), "{case}: archive block");
        assert!(ArchiveReader::open(Cursor::new(archive.clone())).unwrap().region([0, 0, 0], [0, 0, 2], 3).is_err(), "{case}: archive region");
        assert!(ArchiveReader::open(Cursor::new(archive)).unwrap().import(&Repository::temporary().unwrap()).is_err(), "{case}: archive import");
        let target = Repository::temporary().unwrap();
        target.put_object(id, &bytes).unwrap();
        let mut restore = Writer::new(target, Q, BUDGET).unwrap();
        assert!(restore.restore(&bad).is_err(), "{case}: Writer restore");
        assert!(!restore.has_unpublished_changes());
        assert_eq!(restore.block_count(), 0);
        eprintln!("H3_DETACHED mutation={case} intrinsic_ok={intrinsic_ok} writer_receiver_archive_restore=REJECT");
    }
}

#[test]
fn classified_empty_column_roundtrips_moves_carrier_and_replaces_atomically() {
    let xy = [0,0]; let empty = ColumnGroundRecords {classified_empty:true,legacy:vec![],heightfields:vec![]};
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

#[test]
fn classified_empty_column_rejects_conflicts_and_unknown_tags_at_boundary() {
    let xy=[0,0];let mut w=writer();let actual=column(surface(xy,&[0]));install(&mut w,&actual,20);
    w.observe([[0,0,32]],20).unwrap();let root=w.snapshot(domain(),1,20).unwrap();
    let mut both=actual.clone();both.classified_empty=true;
    assert!(w.replace_ground_column(xy,&both).unwrap_err().contains("conflicts"));
    assert!(!w.has_unpublished_changes());
    let block=Block::from([(0,Evidence{observed_ns:20,hits:1})]);
    let good=GroundRecords::Versioned(Arc::new([(0,GroundRecord::ClassifiedEmptyColumn{})].into()));
    let bytes=encode_ground_block(&block,&good).unwrap();assert!(decode_block_parts(&bytes).is_ok());
    for json in [
        serde_json::json!([[0,{"representation":"unrecognized_empty","data":{}}]]),
        serde_json::json!([[0,{"representation":"classified_empty_column","data":{"height":0}}]]),
        serde_json::json!([[1,{"representation":"classified_empty_column","data":{}}]]),
        serde_json::json!([[1024,{"representation":"classified_empty_column","data":{}}]]),
    ] {
        let payload=serde_json::to_vec(&json).unwrap();let mut bad=bytes[..28].to_vec();
        bad[12..16].copy_from_slice(&(payload.len() as u32).to_le_bytes());bad.extend(payload);
        assert!(decode_block_parts(&bad).is_err(),"{json}");
    }
    // Each block is intrinsically valid, but a marker cannot coexist with a
    // ground claim in another vertical shard of the same actual column.
    let mut conflicting=root.as_ref().clone();let reference=conflicting.blocks.iter_mut().find(|r|r.key==[0,0,1]).unwrap();
    let object=ObjectId::random().unwrap().into();reference.object=object;
    let transfer=Transfer{root:conflicting.encode().unwrap(),included:[(object,bytes)].into()};
    let mut receiver=Receiver::new(Repository::temporary().unwrap());
    let error=receiver.receive(&transfer,&conflicting.domain,|_,b| w.repository().read_object(b.object,MAX_BLOCK_BYTES)).unwrap_err();
    assert!(error.contains("conflicts"),"{error}");
    // Hot incremental roots must also inspect a changed Legacy-only shard:
    // the empty marker object itself is unchanged and may be cached.
    let mut w=writer();w.observe([[0,0,0],[0,0,32]],20).unwrap();
    w.replace_ground_column(xy,&ColumnGroundRecords{classified_empty:true,legacy:vec![],heightfields:vec![]}).unwrap();
    let marked=w.snapshot(domain(),1,20).unwrap();
    let layer=LayerObservation{members:vec![0],normal:[0.,0.,1.],plane_origin:[0.025,0.025,32.5*f64::from(Q)],
        samples:vec![LayerSample{source:Default::default(),point:[0.025,0.025,32.5*f64::from(Q)],
            source_sequence:1,source_timestamp_ns:20,point_index:0}]};
    let mut hot=Receiver::new(Repository::temporary().unwrap());receive(&mut hot,&marked,w.repository()).unwrap();
    let mut next=marked.as_ref().clone();next.revision=2;next.geometry_revision=2;
    let changed=next.blocks.iter_mut().find(|r|r.key==[0,0,1]).unwrap();
    let object=ObjectId::random().unwrap().into();changed.object=object;changed.block_revision=2;changed.geometry_revision=2;
    let legacy_bytes=encode_layer_block(&block,&[(0,layer.clone())].into()).unwrap();
    let transfer=Transfer{root:next.encode().unwrap(),included:[(object,legacy_bytes)].into()};
    let error=hot.receive(&transfer,&next.domain,|_,b|w.repository().read_object(b.object,MAX_BLOCK_BYTES)).unwrap_err();
    assert!(error.contains("conflicts"),"{error}");
    w.replace_layer_observations([0,0,1],[(0,layer)].into()).unwrap();
    let error=w.snapshot(domain(),2,20).unwrap_err();assert!(error.contains("conflicts"),"{error}");
    eprintln!("EMPTY_COLUMN intrinsic_invalid=4 cross_shard_conflict=REJECT hot_changed_legacy=REJECT writer_seal=REJECT");
}

// 脏块记录表引用唯一时也只在全部校验通过后原地改：整块校验拒绝的一批替换，
// 不能留下同批中已暂存的合法列，也不能改字节统计、待封存状态或封存版本。
#[test]
fn rejected_batch_on_unique_dirty_block_changes_no_record_byte_or_version() {
    let build = || {
        let mut w = writer();
        for x in 0..3 { install(&mut w, &column(surface([x, 0], &[0, 1])), 20); }
        w
    };
    let (mut failing, mut control) = (build(), build());
    let before: Vec<_> = (0..3).map(|x| failing.ground_column([x, 0]).unwrap()).collect();
    let resident = failing.resident_bytes();
    let (first, second) = (column(surface([0, 0], &[0])), column(surface([1, 0], &[1])));
    // 诊断样本时间晚于成员的占据证据，暂存阶段不检查，只在整块校验时失败。
    let mut late = surface([2, 0], &[0]);
    late.samples = vec![LayerSample { source: Arc::new(LayerSource::Lidar { backend: 1, runtime_generation: 1,
        sensor_frame_id: "sensor".into() }), point: [0.125, 0.025, 0.01], source_sequence: 1,
        source_timestamp_ns: 99, point_index: 0 }];
    let late = column(late);
    let error = failing.replace_ground_columns([([0, 0], &first), ([1, 0], &second), ([2, 0], &late)]).unwrap_err();
    assert_eq!(error, "heightfield sample exceeds occupancy evidence");
    for x in 0..3 { assert_eq!(failing.ground_column([x, 0]).unwrap(), before[x as usize]); }
    assert_eq!(failing.resident_bytes(), resident);
    let (a, b) = (failing.seal(domain(), 1, 20).unwrap().root, control.seal(domain(), 1, 20).unwrap().root);
    assert_eq!((a.version, a.revision, a.geometry_revision), (b.version, b.revision, b.geometry_revision));
    let shape = |root: &Manifest| root.blocks.iter().map(|r| (r.key, r.block_revision, r.geometry_revision, r.points)).collect::<Vec<_>>();
    assert_eq!(shape(&a), shape(&b));
    // 同一批合法替换随后照常生效。
    failing.replace_ground_columns([([0, 0], &first), ([1, 0], &second)]).unwrap();
    assert_eq!(failing.ground_column([0, 0]).unwrap(), first);
    assert_eq!(failing.ground_column([1, 0]).unwrap(), second);
}
