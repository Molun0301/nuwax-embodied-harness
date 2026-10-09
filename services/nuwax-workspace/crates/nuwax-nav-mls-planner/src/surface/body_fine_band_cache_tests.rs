use super::*;

fn observed(keys:&[[i32;3]])->Arc<SurfaceObservations> {
    let mut groups=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
    for &key in keys {groups.entry(key.map(|v|v.div_euclid(32))).or_default()
        .insert(key,SurfaceObservation{observed_ns:0,hits:0});}
    let mut source=SurfaceObservations::new(0.05,32).unwrap();
    for (block,values) in groups {source.replace_block(block,values).unwrap();}
    Arc::new(source)
}

#[test]
fn fine_band_cache_preserves_runs_and_resets_with_source_owners() {
    let keys=[[0,0,-3],[0,0,-2],[0,0,-1],[0,0,2],[0,0,3]];
    let source=observed(&keys);
    let base=scene(&[],false).with_observations(Some(source.clone()));
    let bands=base.fine_contact_bands(0,0);
    assert_eq!(bands.iter().map(|b|b.run).collect::<Vec<_>>(),vec![[-3,-1],[2,3]]);
    let fine=f64::from(source.size_m);
    assert_eq!(bands.iter().map(|b|b.fit.map(f64::to_bits)).collect::<Vec<_>>(),
        vec![[-2.0*fine,0.0].map(f64::to_bits),[3.0*fine,4.0*fine].map(f64::to_bits)]);
    let cloned=base.clone();let same=base.with_observations(Some(source));
    assert!(Arc::ptr_eq(&base.body_query_cache,&cloned.body_query_cache));
    assert!(Arc::ptr_eq(&bands,&same.fine_contact_bands(0,0)));
    let replaced=base.with_observations(Some(observed(&[[0,0,6]])));
    assert!(!Arc::ptr_eq(&base.body_query_cache,&replaced.body_query_cache));
    assert!(replaced.body_query_cache.lock().unwrap().fine_bands.is_empty());
    assert_eq!(replaced.fine_contact_bands(0,0)[0].run,[6,6]);
    let volume=base.clone().with_ground_evidence(GroundEvidence::OccupiedVolume);
    assert!(volume.body_query_cache.lock().unwrap().fine_bands.is_empty());
    let vb=volume.fine_contact_bands(0,0);
    assert_eq!(vb.iter().map(|b|b.run).collect::<Vec<_>>(),vec![[-3,-1],[2,3]]);
    assert!(vb.iter().all(|b|b.fit[0].to_bits()==b.fit[1].to_bits()));
    let fork=base.fork_for_planning();
    assert!(!Arc::ptr_eq(&base.body_query_cache,&fork.body_query_cache));
    assert!(!Arc::ptr_eq(base.observations.as_ref().unwrap(),fork.observations.as_ref().unwrap()));
    assert!(fork.body_query_cache.lock().unwrap().fine_bands.is_empty());
    assert_eq!(*fork.fine_contact_bands(0,0),*bands);
    let envelope=base.with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
    assert!(envelope.body_query_cache.lock().unwrap().fine_bands.is_empty());
    assert_eq!(*envelope.fine_contact_bands(0,0),*bands);
    assert_eq!(std::mem::size_of::<FineContactBand>(),24);
}

#[test]
fn occupied_volume_cleared_top_never_selects_an_internal_height() {
    let keys=[[0,0,0],[0,0,1],[0,0,2]];
    let base=scene(&[],false).with_observations(Some(observed(&keys)));
    let fine=f64::from(base.observations.as_ref().unwrap().size_m);
    let q=[0.0,0.0,0.0,1.0];let r=rotation(q).unwrap();
    let body=[0.025,0.025,3.0*fine+base.body_envelope().unwrap().body_to_support_z_m];
    let polygon=vec![[0.01,0.01],[0.04,0.01],[0.04,0.04],[0.01,0.04]];
    for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
        let raw=base.clone().with_ground_evidence(evidence);
        let prism=PrismQuery::new(raw.body_envelope().unwrap(),r,body);
        let query=PolygonQuery::new(&polygon);
        let rows=raw.fine_ground_columns(None,&query,&prism,body,r[2],0,0).unwrap();
        assert_eq!(rows.iter().find(|c|c.xy==[0,0]).unwrap().height.map(f64::to_bits),Some((3.0*fine).to_bits()));
        // Focused private-mask fixture: immutable source members remain intact;
        // only its exposed top is excluded for this query's body view.
        let mut view=raw.clone();view.body_query_cache=Default::default();
        view.current_body_support=Some(Arc::new(CurrentBodySupport {
            source:raw.clone(),body,quaternion:q,polygon:Arc::new(polygon.clone()),
            cells:Arc::new(AHashSet::new()),cleared_voxels:Arc::new(AHashSet::from_iter([[0,0,2]])),
        }));
        let rows=view.fine_ground_columns(None,&query,&prism,body,r[2],0,0).unwrap();
        let height=rows.iter().find(|c|c.xy==[0,0]).unwrap().height;
        let expected=if evidence==GroundEvidence::OccupiedVolume {None}else{Some(2.0*fine)};
        assert_eq!(height.map(f64::to_bits),expected.map(f64::to_bits),"{evidence:?}");
        assert_eq!(view.fine_contact_bands(0,0)[0].run,[0,2]);
        for key in keys {assert!(view.observations.as_ref().unwrap().get(key).is_some());}
        assert!(!view.current_body_support.as_ref().unwrap().cleared_voxels.contains(&[0,0,1]));
    }
}

#[test]
fn evidence_only_new_arc_invalidates_warm_band_membership() {
    let source=|hits| {
        let mut o=SurfaceObservations::new(0.05,32).unwrap();
        o.set_source_timestamp_ns(100);
        o.replace_block([0,0,0],std::collections::BTreeMap::from([
            ([0,0,5],SurfaceObservation{observed_ns:1,hits})])).unwrap();
        Arc::new(o)
    };
    let strong=source(3);let weak=source(1);
    assert_eq!(strong.column_levels(0,0),weak.column_levels(0,0));
    assert!(strong.supports_obstacle([0,0,5]));
    assert!(!weak.supports_obstacle([0,0,5]));
    let map=scene(&[],false).with_observations(Some(strong.clone()));
    let warm=map.fine_contact_bands(0,0);assert_eq!(warm[0].run,[5,5]);
    let changed=map.with_observations(Some(weak));
    assert!(map.same_geometry(&changed));
    assert!(!Arc::ptr_eq(&map.body_query_cache,&changed.body_query_cache));
    assert!(changed.body_query_cache.lock().unwrap().fine_bands.is_empty());
    assert!(changed.fine_contact_bands(0,0).is_empty());
    assert_eq!(map.fine_contact_bands(0,0)[0].run,[5,5]);
    let restored=changed.with_observations(Some(strong));
    assert!(restored.body_query_cache.lock().unwrap().fine_bands.is_empty());
    assert_eq!(restored.fine_contact_bands(0,0)[0].run,[5,5]);
}

#[test]
fn fine_contact_runs_only_authorize_supported_source_members_across_gaps_and_masks() {
    let keys=[[0,0,-3],[0,0,-2],[0,0,-1],[0,0,1],[0,0,2]];
    let base=scene(&[],false).with_observations(Some(observed(&keys)));
    let q=[0.0,0.0,0.0,1.0];let r=rotation(q).unwrap();
    let body=[0.025,0.025,base.body_envelope().unwrap().body_to_support_z_m];
    let polygon=vec![[0.01,0.01],[0.04,0.01],[0.04,0.04],[0.01,0.04]];
    for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
        for masked in [false,true] {
            let mut map=base.clone().with_ground_evidence(evidence);map.body_query_cache=Default::default();
            if masked {map.current_body_support=Some(Arc::new(CurrentBodySupport {
                source:map.clone(),body,quaternion:q,polygon:Arc::new(polygon.clone()),
                cells:Arc::new(AHashSet::new()),cleared_voxels:Arc::new(AHashSet::from_iter([[0,0,-2],[0,0,2]])),
            }));}
            let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
            let rows=map.fine_ground_columns(None,&PolygonQuery::new(&polygon),&prism,body,r[2],0,0).unwrap();
            let row=rows.iter().find(|row|row.xy==[0,0]).unwrap();
            assert!(row.permits_contact(-3)&&row.permits_contact(-1),"selected original run");
            assert_eq!(row.permits_contact(-2),!masked,"selected run cannot bridge a cleared member");
            assert!(row.permits_contact(1)&&row.permits_contact(2),"extra original run remains source-backed, including a privately cleared member");
            for z in -5..=5 {
                assert!(!row.permits_contact(z)||keys.contains(&[0,0,z]),"cannot bridge a source gap");
                if row.permits_contact(z) {
                    let source=map.observations.as_ref().unwrap();
                    assert!(source.get([0,0,z]).is_some());assert!(source.supports_obstacle([0,0,z]));
                }
            }
            assert_eq!(map.fine_contact_bands(0,0).iter().map(|b|b.run).collect::<Vec<_>>(),vec![[-3,-1],[1,2]]);
        }
    }
}

#[test]
fn fine_contact_permission_is_recomputed_for_evidence_only_owner_replacement() {
    let source=|hits| {let mut o=SurfaceObservations::new(0.05,32).unwrap();o.set_source_timestamp_ns(100);
        o.replace_block([0,0,0],std::collections::BTreeMap::from([([0,0,0],SurfaceObservation{observed_ns:1,hits})])).unwrap();Arc::new(o)};
    let strong=source(3);let weak=source(1);
    let base=scene(&[],false).with_observations(Some(strong.clone()));
    let q=[0.0,0.0,0.0,1.0];let r=rotation(q).unwrap();let body=[0.025,0.025,base.body_envelope().unwrap().body_to_support_z_m];
    let polygon=vec![[0.01,0.01],[0.04,0.01],[0.04,0.04],[0.01,0.04]];
    for (map,expected) in [(base.clone(),true),(base.with_observations(Some(weak.clone())),false),(base.with_observations(None),false),(base.with_observations(Some(strong)),true)] {
        let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
        let rows=map.fine_ground_columns(None,&PolygonQuery::new(&polygon),&prism,body,r[2],0,0);
        let permitted=rows.as_ref().is_some_and(|rows|rows.iter().any(|c|c.xy==[0,0]&&c.permits_contact(0)));
        assert_eq!(permitted,expected);
        if permitted {let o=map.observations.as_ref().unwrap();assert!(o.get([0,0,0]).is_some()&&o.supports_obstacle([0,0,0]));}
    }
    assert!(weak.get([0,0,0]).is_some());assert!(!weak.supports_obstacle([0,0,0]));
}
