use super::*;

fn mixed_hotpath_map()->SurfaceSnapshot {
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
    let base=same_coarse_measured_map(&[]);
    let mut observations=base.observations.as_ref().unwrap().as_ref().clone();
    let q=f64::from(observations.size_m);
    let mut blocks=std::collections::BTreeMap::<[i32;3],LayerObservations>::new();
    for x in -12..12 {for y in -12..12 {
        if (x+y)%2!=0 {continue;}
        for z in [0,1] {
            let key=[x,y,z];let local=local_key(key);
            let origin=[(f64::from(x)+0.5)*q,(f64::from(y)+0.5)*q,(f64::from(z)+0.5)*q];
            blocks.entry(block_key(key)).or_default().insert(local,LayerObservation {
                members:vec![local],normal:[0.,0.,1.],plane_origin:origin,
                samples:vec![LayerSample{source:Default::default(),point:origin,
                    source_sequence:1,source_timestamp_ns:1,point_index:u32::from(local)}],
            });
        }
    }}
    for (block,layers) in blocks {observations.replace_ground_layers(block,Arc::new(layers)).unwrap();}
    observations.mark_complete_geometry_source();
    base.with_observations(Some(Arc::new(observations)))
}

fn column_bits(rows:Option<Vec<FineGroundColumn>>)->Option<Vec<([i32;2],Option<u64>,Vec<[i32;2]>)>> {
    rows.map(|rows|rows.into_iter().map(|row|(row.xy,row.height.map(f64::to_bits),row.contacts.as_slice().to_vec())).collect())
}
fn height_bits(rows:Option<Vec<FineGroundHeight>>)->Option<Vec<Option<u64>>> {
    rows.map(|rows|rows.into_iter().map(|row|row.0.map(f64::to_bits)).collect())
}
fn height_row_bits(rows:Option<Vec<FineGroundHeightRow>>)->Option<Vec<([i32;2],Option<u64>)>> {
    rows.map(|rows|rows.into_iter().map(|row|(row.xy,row.height.map(f64::to_bits))).collect())
}
fn warm_hotpath_cache(map:&SurfaceSnapshot,mode:usize,cx:i32,cy:i32) {
    if mode>0 {map.fine_contact_bands(cx*2,cy*2);}
    if mode==2 {
        let empty=Arc::new(Vec::new());
        let mut cache=map.body_query_cache.lock().unwrap();
        while cache.fine_bands.len()<8192 {
            let index=cache.fine_bands.len() as i32;
            cache.fine_bands.insert((1_000_000+index,0),Arc::clone(&empty));
        }
    }
}

#[test]
fn hotpath_batch_matches_frozen_selector_for_legacy_measured_and_mixed() {
    let fixtures=[same_coarse_measured_map(&[]),same_coarse_measured_map(&[0,1]),mixed_hotpath_map()];
    for (fixture,base) in fixtures.iter().enumerate() {
        for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
            for (cx,cy) in [(0,0),(-1,-1)] {
                for ground_z in [None,Some(0),Some(7)] {
                    let ground=ground_z.map(|z|(cx,cy,z));
                    for z in [0.425,0.475,1.8] {
                        for attitude in [[0.,0.,0.,1.],[0.10,-0.05,0.20,1.0]] {
                            for mode in 0..3 {
                                let map=base.fork_for_planning().with_ground_evidence(evidence);
                                let reference=base.fork_for_planning().with_ground_evidence(evidence);
                                warm_hotpath_cache(&map,mode,cx,cy);warm_hotpath_cache(&reference,mode,cx,cy);
                                let body=[f64::from(cx)*0.1+0.05,f64::from(cy)*0.1+0.05,z];
                                let polygon_points=[[body[0]-0.045,body[1]-0.045],
                                    [body[0]+0.045,body[1]-0.045],[body[0]+0.045,body[1]+0.045],
                                    [body[0]-0.045,body[1]+0.045]];
                                let polygon=PolygonQuery::new(&polygon_points);let r=rotation(attitude).unwrap();
                                let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
                                let actual=column_bits(map.fine_ground_values(ground,&polygon,&prism,body,r[2],cx,cy));
                                let expected=column_bits(reference.fine_ground_values_before_hotpath(ground,&polygon,&prism,body,r[2],cx,cy));
                                assert_eq!(actual,expected,"fixture={fixture} ground={ground:?} z={z} mode={mode}");
                                assert_eq!(height_bits(map.fine_ground_values(ground,&polygon,&prism,body,r[2],cx,cy)),
                                    height_bits(reference.fine_ground_values_before_hotpath(ground,&polygon,&prism,body,r[2],cx,cy)));
                                assert_eq!(height_row_bits(map.fine_ground_values(ground,&polygon,&prism,body,r[2],cx,cy)),
                                    height_row_bits(reference.fine_ground_values_before_hotpath(ground,&polygon,&prism,body,r[2],cx,cy)));
                                // The one-column guide entry must share exactly the same selector too.
                                for xy in [[cx*2,cy*2],[cx*2+1,cy*2+1]] {
                                    let observations=map.observations.as_ref().unwrap();let model=map.body_envelope().unwrap();
                                    let a:FineGroundColumn=map.select_fine_ground_value(observations,model,ground,&polygon,&prism,body,r[2],xy,None);
                                    let b:FineGroundColumn=reference.select_fine_ground_value_before_hotpath(observations,model,ground,&polygon,&prism,body,r[2],xy,None);
                                    assert_eq!(column_bits(Some(vec![a])),column_bits(Some(vec![b])));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn hotpath_measured_acceptance_and_rejection_skip_unused_band_builds() {
    let base=same_coarse_measured_map(&[0,1]);
    let polygon=PolygonQuery::new(&[[0.005,0.005],[0.095,0.005],[0.095,0.095],[0.005,0.095]]);
    let r=rotation([0.,0.,0.,1.]).unwrap();
    for (ground,z,reject) in [(Some((0,0,0)),0.475,false),(Some((0,0,7)),0.475,true),(None,1.8,true)] {
        for cache_full in [false,true] {
            let map=base.fork_for_planning();let reference=base.fork_for_planning();
            if cache_full {
                let empty=Arc::new(Vec::new());let mut cache=map.body_query_cache.lock().unwrap();
                for i in 0..8192 {cache.fine_bands.insert((1_000_000+i,0),Arc::clone(&empty));}
            }
            let body=[0.05,0.05,z];let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
            let rows=map.fine_ground_columns(ground,&polygon,&prism,body,r[2],0,0).unwrap();
            assert_eq!(rows.len(),4);
            if reject {assert!(rows.iter().all(|row|row.height.is_none()&&row.contacts.as_slice().is_empty()),
                "measured rejection is final, even though real compatibility occupancy exists");}
            else {assert!(rows.iter().all(|row|row.height.is_some()&&row.permits_contact(1)&&!row.permits_contact(0)));}
            let expected=reference.fine_ground_values_before_hotpath(ground,&polygon,&prism,body,r[2],0,0);
            assert_eq!(column_bits(Some(rows)),column_bits(expected));
            assert_eq!(reference.body_query_cache.lock().unwrap().fine_bands.len(),4,
                "frozen cold path really constructs four bands that this query does not consume");
            let cache=map.body_query_cache.lock().unwrap();
            assert_eq!(cache.fine_bands.len(),if cache_full {8192}else{0});
            for xy in [(0,0),(0,1),(1,0),(1,1)] {assert!(!cache.fine_bands.contains_key(&xy));}
        }
    }
}

#[test]
fn hotpath_mixed_columns_keep_original_order_and_only_build_legacy_bands() {
    let base=mixed_hotpath_map();
    let polygon=PolygonQuery::new(&[[0.005,0.005],[0.095,0.005],[0.095,0.095],[0.005,0.095]]);
    let r=rotation([0.,0.,0.,1.]).unwrap();let body=[0.05,0.05,0.475];
    for ground in [Some((0,0,0)),Some((0,0,7)),None] {
        let map=base.fork_for_planning();let reference=base.fork_for_planning();
        let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
        let rows=map.fine_ground_columns(ground,&polygon,&prism,body,r[2],0,0).unwrap();
        assert_eq!(rows.iter().map(|row|row.xy).collect::<Vec<_>>(),vec![[0,0],[0,1],[1,0],[1,1]]);
        assert_eq!(column_bits(Some(rows)),column_bits(reference.fine_ground_values_before_hotpath(ground,&polygon,&prism,body,r[2],0,0)));
        let mut keys:Vec<_>=map.body_query_cache.lock().unwrap().fine_bands.keys().copied().collect();keys.sort();
        assert_eq!(keys,vec![(0,1),(1,0)],"measured records and measured misses never enter compatibility cache");
        assert_eq!(reference.body_query_cache.lock().unwrap().fine_bands.len(),4);
    }
}

#[test]
fn hotpath_absent_observations_and_empty_footprint_keep_original_option_contract() {
    let q=[0.,0.,0.,1.];let r=rotation(q).unwrap();let body=[0.05,0.05,0.475];
    let polygon=PolygonQuery::new(&[[9.,9.],[10.,9.],[10.,10.],[9.,10.]]);
    for map in [same_coarse_measured_map(&[0,1]),same_coarse_measured_map(&[0,1]).with_observations(None)] {
        let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
        let a=column_bits(map.fine_ground_values(None,&polygon,&prism,body,r[2],0,0));
        let b=column_bits(map.fine_ground_values_before_hotpath(None,&polygon,&prism,body,r[2],0,0));
        assert_eq!(a,b);
        assert_eq!(a,map.observations.as_ref().map(|_|Vec::new()));
    }
}

fn witness_bits(witness:Option<BodyFailureWitness>)->String {
    witness.map_or_else(||"none".into(),|w|match w.rejection {
        BodyRejection::Occupancy{cell,source,z_bounds,ground_height}=>format!("{:?}/{:?}/{cell:?}/{source:?}/{:?}/{:?}",
            w.pose.0.map(f64::to_bits),w.pose.1.map(f64::to_bits),z_bounds.map(f64::to_bits),ground_height.map(f64::to_bits)),
    })
}

// Run unchanged against before/after binaries. Compare only HOTPATH_TRACE lines.
// This exercises actual pose, motion, recent-obstacle and external-boundary results;
// it is not a replacement for the existing collision and phase suites.
#[test]
#[ignore="explicit serial baseline/candidate real-output comparison"]
fn hotpath_pose_and_motion_real_output_trace() {
    let measured=same_coarse_measured_map(&[0,1]);
    let source=f64::from(measured.source_voxel_size_m());
    let lower=0.5*source+0.4;let upper=1.5*source+0.4;
    let q=[0.,0.,0.,1.];
    assert_eq!(measured.validate_body_pose([0.025,0.025,upper],q),Ok(()));
    assert_eq!(measured.validate_body_pose([0.025,0.025,lower],q),Err(SurfaceError::InsufficientClearance));
    let recent=measured.with_recent_obstacle_points(1,100,0,&[],[(0,[0.025,0.025,0.35])]).unwrap();
    let fixtures=[same_coarse_measured_map(&[]),measured.clone(),mixed_hotpath_map(),recent,measured.with_observations(None)];
    let boundary=[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]];
    for (case,base) in fixtures.iter().enumerate() {for warm in [false,true] {
        let map=base.fork_for_planning();
        for z in [lower,upper,0.8] {for attitude in [q,[0.,0.,0.05_f64.sin(),0.05_f64.cos()]] {
            let from=([0.025,0.025,z],attitude);
            let to=([0.225,0.025,z],attitude);
            if warm {let _=map.validate_body_pose(from.0,from.1);let _=map.validate_body_motion(from,to,&[]);}
            let mut witness=None;let result=map.validate_body_pose_with_witness(from.0,from.1,&mut witness);
            eprintln!("HOTPATH_TRACE case={case} warm={warm} kind=pose pose={:?}/{:?} result={result:?} witness={}",
                from.0.map(f64::to_bits),from.1.map(f64::to_bits),witness_bits(witness));
            for bounded in [false,true] {
                let mut witness=None;let result=map.validate_body_motion_with_witness(from,to,if bounded {&boundary}else{&[]},&mut witness);
                eprintln!("HOTPATH_TRACE case={case} warm={warm} kind=motion bounded={bounded} pose={:?}/{:?} result={result:?} witness={}",
                    from.0.map(f64::to_bits),from.1.map(f64::to_bits),witness_bits(witness));
            }
        }}
    }}
}

impl SurfaceSnapshot {
    fn fine_ground_values_before_hotpath<T:FineGroundValue>(&self,ground:Option<VoxelKey>,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],x:i32,y:i32)->Option<Vec<T>> {
        let observations=self.observations.as_ref()?;
        let model=self.body_envelope()?;
        let fine=f64::from(observations.size_m);let size=f64::from(self.config().voxel_size);
        let mut columns=Vec::new();
        for ix in (f64::from(x)*size/fine).floor() as i32..(f64::from(x+1)*size/fine).ceil() as i32 {
            for iy in (f64::from(y)*size/fine).floor() as i32..(f64::from(y+1)*size/fine).ceil() as i32 {
                if polygon.intersects(ix,iy,fine) {columns.push(([ix,iy],None));}
            }
        }
        // 与接触响应域相同的现有带缓存，单次批读；冷列仅扫描其原始列索引。
        {
            let cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
            for (xy,bands) in &mut columns {*bands=cache.fine_bands.get(&(xy[0],xy[1])).cloned();}
        }
        let mut missing=Vec::new();
        for (xy,bands) in &mut columns {
            if bands.is_none() {
                let values=Arc::new(self.read_fine_contact_bands(xy[0],xy[1]));
                missing.push(((xy[0],xy[1]),Arc::clone(&values)));*bands=Some(values);
            }
        }
        if !missing.is_empty() {
            let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
            if cache.fine_bands.len()>=8192 {cache.fine_bands.clear();}
            cache.fine_bands.extend(missing);
        }
        Some(columns.into_iter().map(|(xy,bands)| {
            let bands=bands.expect("批读已填充");
            self.select_fine_ground_value_before_hotpath::<T>(observations,model,ground,polygon,prism,
                body,normal,xy,Some(bands.as_slice()))
        }).collect())
    }

    /// One source-owned fine column, shared by batched body checks and live guide hints.
    /// Measured layers return before any compatibility-band cache access.
    fn select_fine_ground_value_before_hotpath<T:FineGroundValue>(&self,observations:&SurfaceObservations,
        model:&SurfaceBodyEnvelope,ground:Option<VoxelKey>,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],xy:[i32;2],bands:Option<&[FineContactBand]>)->T {
        // Without a source terrain anchor carry the calibrated support height
        // in the gravity frame. Body pitch/roll belongs to the physical prism,
        // and cannot relabel a horizontal ground band as an obstacle.
        let (body,normal)=if ground.is_none() {
            ([body[0],body[1],body[2]+model.body_to_support_z_m*(1.0-normal[2])],[0.0,0.0,1.0])
        } else {(body,normal)};
        let fine=f64::from(observations.size_m);let size=f64::from(self.config().voxel_size);
        let contact=model.max_step_up_m.max(model.max_step_down_m)+f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        // Newly measured layers have source-owned members and a fixed plane.
        // Legacy columns retain the existing compatibility behavior below.
        if self.ground_evidence==GroundEvidence::MeasuredSurface {
            let center=xy.map(|v|(f64::from(v)+0.5)*fine);
            let mut records=observations.ground_layers(xy).peekable();
            let any_measured=records.peek().is_some();
            // The physical anchor constrains membership to its coarse cell.
            // Within that cell retain the layer nearest the calibrated support
            // plane, as in support lookup; never merge the other layer's members.
            let reference=body[2]-(normal[0]*(center[0]-body[0])
                +normal[1]*(center[1]-body[1])+model.body_to_support_z_m)/normal[2];
            let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
            let record=records.filter(|record| {
                if let Some(ground)=ground {
                    record.members.iter().any(|&z|(((f64::from(z)+0.5)*fine)/size).floor() as i32==ground.2)
                } else {
                    record.height_at(center).is_some_and(|height|
                        height-reference<=model.max_step_up_m+uncertainty+1e-6
                        && reference-height<=model.max_step_down_m+uncertainty+1e-6
                        && record.normal[2]>=f64::from(self.config().max_slope_deg).to_radians().cos())
                }
            }).min_by(|a,b|(a.height_at(center).unwrap_or(f64::INFINITY)-reference).abs()
                .total_cmp(&(b.height_at(center).unwrap_or(f64::INFINITY)-reference).abs()));
            if let Some(record)=record {
                let height=record.height_at(center);
                let mut contacts=FineContacts::default();
                if T::CONTACTS {for &z in &record.members {
                    if let Some(last)=contacts.last_mut().filter(|last|last[1].checked_add(1)==Some(z)) {last[1]=z;}
                    else {contacts.push([z,z]);}
                }}
                return T::from_parts(xy,height,contacts);
            }
            if any_measured {return T::from_parts(xy,None,FineContacts::default());}
        }
        let support_offset=model.body_to_support_z_m/normal[2];
        let body_lower_offset=model.z_bounds_m[0]/normal[2];
        let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let allowance=(-support_offset+contact+1e-6).min(uncertainty);
        // contact_plane has only two query-local return values. The original
        // height-dependent fast branch still chooses between them for each Z.
        let flat=[-normal[0]/normal[2],-normal[1]/normal[2]]==[0.0,0.0];
        let (rectangular_plane,clipped_plane,upper_plane)=if flat {
            (body[2],body[2],body[2])
        } else {
            (polygon.contact_plane(body,normal,xy[0],xy[1],fine,f64::NEG_INFINITY,0.0),
             polygon.contact_plane(body,normal,xy[0],xy[1],fine,f64::INFINITY,0.0),
             -polygon.contact_plane([body[0],body[1],-body[2]],[-normal[0],-normal[1],normal[2]],
                xy[0],xy[1],fine,f64::INFINITY,0.0))
        };
        let lower=clipped_plane-support_offset;
        let upper=upper_plane-support_offset;
        let nominal_upper=lower+contact+1e-6;
        let contact_plane=|top:f64|if top<=rectangular_plane+allowance {rectangular_plane}else{clipped_plane};
        let reference=(lower+upper)*0.5;
        let distance=|b:[f64;2]|(b[0]-reference).max(reference-b[1]).max(0.0);
        // Batched body checks pass their existing prefetched bands. A single
        // guide column loads only its own legacy band, after the measured fast path.
        let cached;
        let bands=match bands {
            Some(bands)=>bands,
            None=>{cached=self.fine_contact_bands(xy[0],xy[1]);cached.as_slice()},
        };
        let band=bands.iter().min_by(|a,b|distance(a.fit).total_cmp(&distance(b.fit))
            .then_with(||(a.fit[1]-reference).abs().total_cmp(&(b.fit[1]-reference).abs()))
            .then_with(||a.fit[1].total_cmp(&b.fit[1])));
        // Associate the original fit interval with the existing actual-overlap
        // nominal window. Intersection is a predicate, never a truncated band.
        let band=band.filter(|candidate|
            candidate.fit[0].max(upper-contact-1e-6)
                <= candidate.fit[1].min(nominal_upper));
        let height=band.and_then(|band| {
            // OccupiedVolume still selects only the collapsed fit top here.
            let first=(band.fit[0]/fine-1.0).round() as i32;
            let last=band.run[1];
            (first..=last).rev().find_map(|iz| {
                let key=[xy[0],xy[1],iz];
                let top=f64::from(iz+1)*fine;
                (top<=nominal_upper || prism.intersects(key.map(|v|f64::from(v)*fine),
                    key.map(|v|f64::from(v+1)*fine))).then_some(top)
            })
        });
        let mut contacts=FineContacts::default();
        if T::CONTACTS {
            for candidate in bands.iter() {
                let [first,last]=candidate.run;
                if Some(candidate)==band {
                    // Check each original selected-band voxel with the existing
                    // nominal upper bound and height predicate. Higher intersecting
                    // voxels remain in `height` and in the original prism checks.
                    for iz in first..=last {
                        let top=f64::from(iz+1)*fine;
                        if top>nominal_upper {continue;}
                        let plane=contact_plane(top);
                        let support=plane-support_offset;
                        let body_lower=plane+body_lower_offset;
                        if height_rejection(body_lower,support,plane,contact,top,uncertainty).is_none() {
                            if let Some(run)=contacts.last_mut().filter(|run|run[1].checked_add(1)==Some(iz)) {
                                run[1]=iz;
                            } else {contacts.push([iz,iz]);}
                        }
                    }
                    continue;
                }
                // An extra band is not a new support anchor. Its complete exposed
                // top must fit the existing nominal-support travel at this overlap;
                // rigid-body bottom height must not enlarge that travel.
                let plane=contact_plane(candidate.fit[1]);
                let support=plane-support_offset;
                let body_lower=plane+body_lower_offset;
                if candidate.fit[1]<=support+contact+1e-6
                    && height_rejection(body_lower,support,plane,contact,candidate.fit[1],uncertainty).is_none() {
                    contacts.push([first,last]);
                }
            }
        }
        T::from_parts(xy,height,contacts)
    }

}
