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
    let mut previous = admitted;
    for (revision, updated_source, timestamp, sequence) in [
        (2, source.clone(), 900, 1), (3, other_clock, 2, 0), (4, source.clone(), 900, 2),
    ] {
        writer.record_source_watermark(updated_source.clone(), timestamp, sequence).unwrap();
        let root = writer.snapshot(domain.clone(), revision, 100).unwrap();
        assert_eq!(root.blocks, first.blocks, "source bookkeeping must not rewrite block objects");
        assert_eq!(root.geometry_revision, first.geometry_revision);
        assert_eq!(root.source_timestamp_ns, first.source_timestamp_ns);
        assert_ne!(root.source_watermarks, previous.source_watermarks);
        let admitted = reader.receive(&transfer_message(&repository, &root)).unwrap();
        assert!(admitted.source_watermarks.iter().any(|mark| mark.source == updated_source
            && mark.source_timestamp_ns == timestamp && mark.source_sequence == sequence));
        let (next, changed) = reader.observations_with_changes(&admitted,
            Some((previous.as_ref(), &observations))).unwrap();
        assert_eq!(changed, Some(vec![]));
        assert!(observations.same_geometry(&next));
        assert_eq!(observations.get([0, 0, 0]), next.get([0, 0, 0]));
        let layers = reader.receiver.layers(&admitted.blocks[0]).unwrap();
        assert_eq!(layers[&0].samples[0].source, source);
        assert_eq!(layers[&0].samples[0].source_timestamp_ns, 90);
        assert_eq!(layers[&0].samples[0].point_index, 17);
        previous = admitted;
        observations = next;
    }
}
