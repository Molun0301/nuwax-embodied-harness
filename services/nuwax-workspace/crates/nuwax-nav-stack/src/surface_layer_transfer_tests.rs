// FineMapReader production transfer path: real BL3 provenance, v4 roots and deltas.
#[test]
fn measured_layer_transfer_distinguishes_plane_changes_metadata_and_deletions() {
    let repository=Repository::temporary().unwrap();
    let mut writer=Writer::new(repository.clone(),0.05,4*1024*1024).unwrap();
    let mut reader=FineMapReader::new(String::new()).unwrap();
    let domain=Domain {session:"test".into(),generation:1,frame_generation:1,producer_generation:1};
    let source=Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
        sensor_id:"depth-front".into(),clock_id:"camera-clock".into(),
        runtime_generation:7,session_generation:1,
    });
    let mut previous:Option<(Arc<Manifest>,Arc<nuwax_nav_mls_planner::SurfaceObservations>)>=None;
    for revision in 1..=6_u64 {
        writer.observe([[40,40,0]],revision).unwrap();
        if revision!=6 {writer.observe([[0,0,0]],revision).unwrap();}
        if revision==6 {writer.remove([[0,0,0]]).unwrap();}
        else {
            let layers=if revision==4 {BTreeMap::new()} else {
                let z=if revision<=2 {0.02}else{0.03};
                BTreeMap::from([(0,LayerObservation {members:vec![0],plane_origin:[0.025,0.025,z],normal:[0.0,0.0,1.0],
                    samples:vec![LayerSample {source:source.clone(), point:[0.025,0.025,z+revision as f64*0.0001],
                        source_sequence:revision,source_timestamp_ns:revision,point_index:0}],
                })])
            };
            writer.replace_layer_observations([0,0,0],layers).unwrap();
        }
        writer.record_source_watermark(source.clone(),revision,revision).unwrap();
        let root=writer.snapshot(domain.clone(),revision,revision).unwrap();
        assert_eq!(root.version,4);
        assert_eq!(root.source_watermarks.len(),1);
        assert_eq!(root.source_watermarks[0].source,source);
        assert_eq!(root.source_watermarks[0].source_timestamp_ns,revision);
        let admitted=reader.receive(&transfer_message(&repository,&root)).unwrap();
        if revision!=4 && revision!=6 {
            let reference=admitted.blocks.iter().find(|block|block.key==[0,0,0]).unwrap();
            let transferred=reader.receiver.layers(reference).unwrap();
            assert_eq!(transferred[&0].samples[0].source,source);
            assert_eq!(transferred[&0].samples[0].point_index,0);
            assert_eq!(transferred[&0].samples[0].source_timestamp_ns,revision);
        }
        let (observations,changed)=reader.observations_with_changes(&admitted,
            previous.as_ref().map(|(old,grid)|(old.as_ref(),grid))).unwrap();
        let layers:Vec<_>=observations.ground_layers([0,0]).collect();
        if revision==4 || revision==6 {assert!(layers.is_empty());}
        else {
            assert_eq!(layers.len(),1);
            assert_eq!(layers[0].members,vec![0]);
            assert!((layers[0].height_at([0.025,0.025]).unwrap()-if revision<=2 {0.02}else{0.03}).abs()<1e-12);
            assert_eq!(layers[0].statistics.moments().count(),1);
        }
        match revision {
            1=>assert!(changed.is_none(),"initial source has no delta baseline"),
            2=>{
                assert_eq!(changed,Some(vec![]),"time/sample diagnostics cannot dirty terrain");
                let (old,grid)=previous.as_ref().unwrap();
                assert_eq!(old.geometry_revision,root.geometry_revision);
                assert!(grid.same_geometry(&observations));
                assert!(!Arc::ptr_eq(grid, &observations),
                    "empty geometry delta must not discard real sample/evidence changes");
                assert_eq!(observations.get([0,0,0]).unwrap().observed_ns,revision);
                assert_ne!(grid.ground_layers([0,0]).next().unwrap().statistics.moments().mean(),layers[0].statistics.moments().mean());
            }
            _=>{
                assert_eq!(changed,Some(vec![[0,0]]),"plane/add/delete must carry exact dirty fine column");
                let (old,grid)=previous.as_ref().unwrap();
                assert!(root.geometry_revision>old.geometry_revision);
                assert!(!grid.same_geometry(&observations));
            }
        }
        previous=Some((admitted,observations));
    }
    // A layerless final block deletion must not leave measured geometry behind.
    assert!(previous.unwrap().1.get([0,0,0]).is_none());
}

#[test]
fn source_watermark_only_transfer_preserves_blocks_and_geometry() {
    let repository = Repository::temporary().unwrap();
    let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
    let mut reader = FineMapReader::new(String::new()).unwrap();
    let domain = Domain { session: "watermark-only".into(), generation: 1,
        frame_generation: 1, producer_generation: 1 };
    let source = Arc::new(LayerSource::Auxiliary { sensor_id: "depth-front".into(),
        clock_id: "camera-clock".into(), runtime_generation: 7, session_generation: 1 });
    let other_clock = Arc::new(LayerSource::Auxiliary { sensor_id: "depth-front".into(),
        clock_id: "other-clock".into(), runtime_generation: 7, session_generation: 1 });
    writer.observe([[0, 0, 0]], 90).unwrap();
    writer.replace_layer_observations([0, 0, 0], BTreeMap::from([(0, LayerObservation {
        members: vec![0], plane_origin: [0.025, 0.025, 0.02], normal: [0.0, 0.0, 1.0],
        samples: vec![LayerSample { source: source.clone(), point: [0.025, 0.025, 0.02],
            source_sequence: 0, source_timestamp_ns: 90, point_index: 17 }],
    })])).unwrap();
    writer.record_source_watermark(source.clone(), 90, 0).unwrap();
    let first = writer.snapshot(domain.clone(), 1, 100).unwrap();
    let admitted = reader.receive(&transfer_message(&repository, &first)).unwrap();
    let mut observations = reader.observations(&admitted, None).unwrap();
    let first_bytes: BTreeMap<_, _> = first.blocks.iter().map(|reference|
        (reference.object, repository.read_object(reference.object, MAX_BLOCK_BYTES).unwrap())).collect();
    let mut previous = admitted;
    for (revision, updated_source, timestamp, sequence) in [
        (2, source.clone(), 900, 1), (3, other_clock, 2, 0), (4, source.clone(), 900, 2),
    ] {
        writer.record_source_watermark(updated_source.clone(), timestamp, sequence).unwrap();
        let root = writer.snapshot(domain.clone(), revision, 100).unwrap();
        assert_eq!(root.blocks, first.blocks, "source bookkeeping must not rewrite block objects");
        assert_eq!(root.geometry_revision, first.geometry_revision);
        for reference in &root.blocks {
            assert_eq!(first_bytes[&reference.object],
                repository.read_object(reference.object, MAX_BLOCK_BYTES).unwrap());
        }
        assert_eq!(root.source_timestamp_ns, first.source_timestamp_ns);
        assert_ne!(root.source_watermarks, previous.source_watermarks);
        let admitted = reader.receive(&transfer_message(&repository, &root)).unwrap();
        assert!(admitted.source_watermarks.iter().any(|mark| mark.source == updated_source
            && mark.source_timestamp_ns == timestamp && mark.source_sequence == sequence));
        let (next, changed) = reader.observations_with_changes(&admitted,
            Some((previous.as_ref(), &observations))).unwrap();
        assert_eq!(changed, Some(vec![]));
        assert!(observations.same_geometry(&next));
        assert!(Arc::ptr_eq(&observations, &next),
            "F2: a true watermark-only root must reuse the complete observation owner");
        assert!(next.is_complete_geometry_source());
        assert_eq!(reader.receiver.current().unwrap().identity(), admitted.identity());
        assert_eq!(observations.get([0, 0, 0]), next.get([0, 0, 0]));
        let layers = reader.receiver.layers(&admitted.blocks[0]).unwrap();
        assert_eq!(layers[&0].samples[0].source, source);
        assert_eq!(layers[&0].samples[0].source_timestamp_ns, 90);
        assert_eq!(layers[&0].samples[0].point_index, 17);
        previous = admitted;
        observations = next;
    }
}

// F2: publication headers do not rewrite unchanged observations. This test uses
// real Writer/Receiver publications; no replacement Arc is injected as input.
#[test]
fn root_header_only_reuses_complete_observations() {
    let repository = Repository::temporary().unwrap();
    let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
    let mut reader = FineMapReader::new(String::new()).unwrap();
    let domain = Domain { session: "header-only".into(), generation: 1,
        frame_generation: 1, producer_generation: 1 };
    writer.observe([[0, 0, 0]], 90).unwrap();
    let first = writer.snapshot(domain.clone(), 1, 100).unwrap();
    let first = reader.receive(&transfer_message(&repository, &first)).unwrap();
    let observations = reader.observations(&first, None).unwrap();
    assert!(observations.is_complete_geometry_source());
    let second = writer.snapshot(domain, 2, 200).unwrap();
    assert_eq!(first.blocks, second.blocks);
    assert_eq!(first.geometry_revision, second.geometry_revision);
    assert_eq!(first.source_watermarks, second.source_watermarks);
    assert_ne!(first.source_timestamp_ns, second.source_timestamp_ns);
    let second = reader.receive(&transfer_message(&repository, &second)).unwrap();
    let (next, changes) = reader.observations_with_changes(&second,
        Some((first.as_ref(), &observations))).unwrap();
    assert_eq!(changes, Some(vec![]));
    assert_eq!(reader.receiver.current().unwrap().identity(), second.identity());
    assert_eq!(next.get([0, 0, 0]), observations.get([0, 0, 0]));
    assert!(Arc::ptr_eq(&observations, &next),
        "F2: a real header-only publication must retain its observation owner");
}

#[test]
fn root_header_no_block_mutation_still_completes_an_incomplete_baseline() {
    let repository = Repository::temporary().unwrap();
    let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
    let mut reader = FineMapReader::new(String::new()).unwrap();
    let domain = Domain { session: "empty-header-completeness".into(), generation: 1,
        frame_generation: 1, producer_generation: 1 };
    let first = writer.snapshot(domain.clone(), 1, 100).unwrap();
    let first = reader.receive(&transfer_message(&repository, &first)).unwrap();
    // Intentional narrow API guard: the previous empty index has not been marked
    // complete. The real root must still establish completeness in a new owner.
    let incomplete = Arc::new(nuwax_nav_mls_planner::SurfaceObservations::new(0.05, 32).unwrap());
    assert!(!incomplete.is_complete_geometry_source());
    let second = writer.snapshot(domain, 2, 200).unwrap();
    let second = reader.receive(&transfer_message(&repository, &second)).unwrap();
    let (complete, changes) = reader.observations_with_changes(&second,
        Some((first.as_ref(), &incomplete))).unwrap();
    assert_eq!(changes, Some(vec![]));
    assert!(complete.is_complete_geometry_source());
    assert!(!incomplete.is_complete_geometry_source());
    assert!(!Arc::ptr_eq(&incomplete, &complete));
}

fn hf_test_column(xy:[i32;2], first:[f64;8], middle:bool) -> ColumnGroundRecords {
    let members=[(0,first),(32,[0.;8]),(64,[0.;8])].into_iter()
        .filter(|(z,_)|*z!=32 || middle).collect::<Vec<_>>();
    let bytes=serde_json::to_vec(&serde_json::json!({"representation":"heightfield_v1",
        "xy":xy,"voxel_size_m":0.05_f32,"members":members})).unwrap();
    ColumnGroundRecords {legacy:vec![],heightfields:vec![HeightfieldSurface {
        column:heightfield_observations::decode_heightfield_column(&bytes).unwrap(),samples:vec![]}]}
}

#[test]
fn heightfield_reader_native_members_metadata_withdrawal_and_retirement() {
    use nuwax_nav_mls_planner::surface::GroundKey;
    for budget in [0,32*1024*1024] {
        let repo=Repository::temporary().unwrap();
        let mut writer=Writer::new(repo.clone(),0.05,4*1024*1024).unwrap();
        let mut reader=FineMapReader {receiver:Receiver::with_cache_budget(Repository::temporary().unwrap(),budget),
            client:None,lcm_url:String::new()};
        let domain=Domain {session:"hf-reader".into(),generation:1,frame_generation:1,producer_generation:1};
        let mut previous:Option<(Arc<Manifest>,Arc<nuwax_nav_mls_planner::SurfaceObservations>)>=None;
        let mut pinned=None;
        for revision in 1..=5 {
            if revision==1 {writer.observe([[0,0,0],[0,0,32],[0,0,64],[96,0,0]],20).unwrap();}
            if revision<=3 {
                let weights=if revision==1 {[0.75,0.25,0.,0.,0.,0.,0.,0.]}
                    else {[0.625,0.375,0.,0.,0.,0.,0.,0.]};
                writer.replace_ground_column([0,0],&hf_test_column([0,0],weights,revision!=3)).unwrap();
            }
            if revision==4 {writer.remove([[0,0,0]]).unwrap();}
            if revision==5 {writer.remove([[0,0,32],[0,0,64]]).unwrap();}
            let root=writer.snapshot(domain.clone(),revision,20).unwrap();
            let admitted=reader.receive(&transfer_message(&repo,&root)).unwrap();
            let (source,delta)=reader.observations_with_changes(&admitted,
                previous.as_ref().map(|(r,s)|(r.as_ref(),s))).unwrap();
            assert!(source.is_complete_geometry_source());
            match revision {
                1=>{assert!(delta.is_none());pinned=Some(source.clone());
                    let r=source.ground_record(GroundKey::HeightfieldV1([0,0,0])).unwrap();
                    assert_eq!(r.members().collect::<Vec<_>>(),vec![0,32,64]);assert_eq!(r.direction(),None);},
                2=>{assert_eq!(delta,Some(vec![]));let old=&previous.as_ref().unwrap().1;
                    assert!(source.same_geometry(old));assert!(!Arc::ptr_eq(old,&source));},
                3=>{assert_eq!(delta,Some(vec![[0,0]]));
                    assert!(source.get([0,0,32]).is_some());
                    assert_eq!(source.ground_record(GroundKey::HeightfieldV1([0,0,0])).unwrap().members().collect::<Vec<_>>(),vec![0,64]);},
                // Removing the only positive-mass member legitimately leaves no
                // height record; both zero-mass occupied members stay obstacles.
                4=>{assert_eq!(delta,Some(vec![[0,0]]));assert!(source.ground_records([0,0]).next().is_none());
                    assert!(source.get([0,0,64]).is_some());},
                _=>{assert_eq!(delta,Some(vec![[0,0]]));assert!(source.ground_records([0,0]).next().is_none());},
            }
            assert_eq!(source.get([96,0,0]).unwrap().observed_ns,20);
            previous=Some((admitted,source));
        }
        let pinned=pinned.unwrap();
        assert_eq!(pinned.ground_record(GroundKey::HeightfieldV1([0,0,0])).unwrap().members().collect::<Vec<_>>(),vec![0,32,64]);
    }
}

#[test]
fn heightfield_reader_preserves_mixed_legacy_cross_block_height_and_reanchor() {
    use nuwax_nav_mls_planner::surface::GroundKey;
    let repo=Repository::temporary().unwrap();let mut writer=Writer::new(repo.clone(),0.05,4*1024*1024).unwrap();
    writer.observe([[0,0,-1],[0,0,32],[1,0,0]],20).unwrap();
    writer.replace_layer_observations([0,0,0],[(1,LayerObservation {members:vec![1],normal:[0.,0.,1.],
        plane_origin:[0.075,0.025,0.02],samples:vec![LayerSample {source:Default::default(),
            point:[0.075,0.025,0.02],source_sequence:1,source_timestamp_ns:20,point_index:1}]} )].into()).unwrap();
    let column=heightfield_observations::HeightfieldColumn::from_counts([0,0],0.05,
        [(-1,[1,0,0,0,0,0,0,0]),(32,[1,0,0,0,0,0,0,0])].into()).unwrap().unwrap();
    let h=column.height();writer.replace_ground_column([0,0],&ColumnGroundRecords {legacy:vec![],
        heightfields:vec![HeightfieldSurface {column,samples:vec![]}]}).unwrap();
    let root=writer.snapshot(Domain {session:"mixed-reader".into(),generation:1,frame_generation:1,producer_generation:1},1,20).unwrap();
    let mut reader=FineMapReader::new(String::new()).unwrap();let root=reader.receive(&transfer_message(&repo,&root)).unwrap();
    let source=reader.observations(&root,None).unwrap();
    let native=source.ground_record(GroundKey::HeightfieldV1([0,0,-1])).unwrap();
    assert_eq!(native.members().collect::<Vec<_>>(),vec![-1,32]);assert_eq!(native.height_at([0.025,0.025]),Some(h));
    assert!(!native.contains_member([0,0,0]));
    let legacy=source.ground_record(GroundKey::LegacyPlane([1,0,0])).unwrap();
    assert_eq!(legacy.height_at([0.075,0.025]),Some(0.02));assert_eq!(legacy.direction(),Some([0.,0.,1.]));
    assert_eq!(source.ground_layers([1,0]).count(),1);
    writer.remove([[0,0,-1]]).unwrap();
    let second=writer.snapshot(root.domain.clone(),2,20).unwrap();
    let second=reader.receive(&transfer_message(&repo,&second)).unwrap();
    let (next,delta)=reader.observations_with_changes(&second,Some((&root,&source))).unwrap();
    assert_eq!(delta,Some(vec![[0,0]]));
    assert!(next.ground_record(GroundKey::HeightfieldV1([0,0,-1])).is_none());
    assert_eq!(next.ground_record(GroundKey::HeightfieldV1([0,0,32])).unwrap().members().collect::<Vec<_>>(),vec![32]);
    assert_eq!(source.ground_record(GroundKey::HeightfieldV1([0,0,-1])).unwrap().members().count(),2);
}

#[test]
fn heightfield_reader_duplicate_and_header_only_keep_observation_owner() {
    let repo=Repository::temporary().unwrap();let mut writer=Writer::new(repo.clone(),0.05,4*1024*1024).unwrap();
    writer.observe([[0,0,0],[0,0,32],[0,0,64]],20).unwrap();
    writer.replace_ground_column([0,0],&hf_test_column([0,0],[0.75,0.25,0.,0.,0.,0.,0.,0.],true)).unwrap();
    let domain=Domain {session:"hf-header".into(),generation:1,frame_generation:1,producer_generation:1};
    let root=writer.snapshot(domain.clone(),1,20).unwrap();let mut reader=FineMapReader::new(String::new()).unwrap();
    let first=reader.receive(&transfer_message(&repo,&root)).unwrap();let source=reader.observations(&first,None).unwrap();
    let (same,delta)=reader.observations_with_changes(&first,Some((&first,&source))).unwrap();
    assert!(Arc::ptr_eq(&same,&source));assert_eq!(delta,Some(vec![]));
    let second=writer.snapshot(domain,2,21).unwrap();assert_eq!(second.blocks,first.blocks);
    let second=reader.receive(&transfer_message(&repo,&second)).unwrap();
    let (same,delta)=reader.observations_with_changes(&second,Some((&first,&source))).unwrap();
    assert!(Arc::ptr_eq(&same,&source));assert_eq!(delta,Some(vec![]));
}

#[test]
fn heightfield_reader_accepts_same_block_fragments_without_losing_members() {
    use nuwax_nav_mls_planner::surface::GroundKey;
    let repo = Repository::temporary().unwrap();
    let mut writer = Writer::new(repo.clone(), 0.05, 4 * 1024 * 1024).unwrap();
    writer.observe((0..4).map(|z| [0, 0, z]), 20).unwrap();
    let column = heightfield_observations::HeightfieldColumn::from_counts([0, 0], 0.05,
        (0..4).map(|z| (z, [1,0,0,0,0,0,0,0])).collect()).unwrap().unwrap();
    writer.replace_ground_column([0,0], &ColumnGroundRecords { legacy: vec![],
        heightfields: vec![HeightfieldSurface { column, samples: vec![] }] }).unwrap();
    let root = writer.snapshot(Domain { session: "same-block-fragments".into(), generation: 1,
        frame_generation: 1, producer_generation: 1 }, 1, 20).unwrap();
    let mut region = RegionReader::new(repo.clone(), 0);
    let (block, records) = region.ground_parts(&root.blocks[0]).unwrap();
    let GroundRecordRef::HeightfieldV1(original) = records.get(0).unwrap() else { panic!("native fixture"); };
    // Same admitted surface, two disjoint interleaved fragments in one block.
    // The existing format permits this; the Writer's canonical split is not
    // an additional receiver admission rule.
    let mut even = original.clone();
    let mut odd = original.clone();
    even.members.retain(|(local, _)| (local / 1024) % 2 == 0);
    odd.members.retain(|(local, _)| (local / 1024) % 2 == 1);
    let split = GroundRecords::Versioned(Arc::new(BTreeMap::from([
        (even.members[0].0, GroundRecord::HeightfieldV1(even)),
        (odd.members[0].0, GroundRecord::HeightfieldV1(odd)),
    ])));
    let bytes = encode_ground_block(&block, &split).unwrap();
    let object = ObjectId::random().unwrap().into();
    let mut split_root = root.as_ref().clone();
    split_root.blocks[0].object = object;
    let mut message = transfer_message(&repo, &root);
    message.fine = Some(Transfer { root: split_root.encode().unwrap(), included: [(object, bytes)].into() });
    let mut reader = FineMapReader::new(String::new()).unwrap();
    let admitted = reader.receive(&message).unwrap();
    let admitted_records = reader.receiver.ground_records(&admitted.blocks[0]).unwrap();
    assert_eq!(admitted_records.iter().count(), 2);
    let source = reader.observations(&admitted, None).unwrap();
    let selected = source.ground_record(GroundKey::HeightfieldV1([0,0,0])).unwrap();
    let actual_members = selected.members().collect::<Vec<_>>();
    let contains = (0..4).map(|z| selected.contains_member([0,0,z])).collect::<Vec<_>>();
    eprintln!("H3 same_block_fragments=2 receiver_admitted=true members={actual_members:?} contains={contains:?}");
    assert_eq!(contains, vec![true; 4], "every admitted fragment must participate in membership");
    assert_eq!(actual_members, vec![0,1,2,3], "borrowed iteration remains sorted and unique");
    assert!(!selected.contains_member([0,0,4]));
    assert!(!selected.contains_member([1,0,0]));
}
