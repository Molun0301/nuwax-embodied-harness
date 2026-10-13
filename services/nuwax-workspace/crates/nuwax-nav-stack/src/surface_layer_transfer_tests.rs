// FineMapReader production transfer path: real BL2 objects, admitted roots and deltas.
#[test]
fn measured_layer_transfer_distinguishes_plane_changes_metadata_and_deletions() {
    let repository=Repository::temporary().unwrap();
    let mut writer=Writer::new(repository.clone(),0.05,4*1024*1024).unwrap();
    let mut reader=FineMapReader::new(String::new()).unwrap();
    let domain=Domain {session:"test".into(),generation:1,frame_generation:1,producer_generation:1};
    let mut previous:Option<(Arc<Manifest>,Arc<nuwax_nav_mls_planner::SurfaceObservations>)>=None;
    for revision in 1..=6_u64 {
        writer.observe([[40,40,0]],revision).unwrap();
        if revision!=6 {writer.observe([[0,0,0]],revision).unwrap();}
        if revision==6 {writer.remove([[0,0,0]]).unwrap();}
        else {
            let layers=if revision==4 {BTreeMap::new()} else {
                let z=if revision<=2 {0.02}else{0.03};
                BTreeMap::from([(0,LayerObservation {members:vec![0],plane_origin:[0.025,0.025,z],normal:[0.0,0.0,1.0],
                    samples:vec![LayerSample {point:[0.025,0.025,z+revision as f64*0.0001],
                        source_sequence:revision,source_timestamp_ns:revision,point_index:0}],
                })])
            };
            writer.replace_layer_observations([0,0,0],layers).unwrap();
        }
        let root=writer.snapshot(domain.clone(),revision,revision).unwrap();
        assert_eq!(root.version,3);
        let admitted=reader.receive(&transfer_message(&repository,&root)).unwrap();
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
