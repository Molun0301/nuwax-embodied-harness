// Separate opt-in load case: the original scene and all original motion oracles
// remain unchanged. Only newly acquired recent ground-return density increases.
use super::*;
use nuwax_codec::lcm::{lio,recent_cloud};

const GROUND_RETURNS: usize = 96_000;
const INSERTED_RETURNS: usize = GROUND_RETURNS + 1_824;

#[derive(Clone,Debug)]
pub(super) struct PublishedDenseScan {
    source:i64, stage:Stage, published:Instant, points:usize, ground:usize, bytes:usize,
    encode_publish_ms:f64,
}

fn packet(map:&schema::NavigationMap3D,sequence:i32,stamp:i64,pose_stamp:i64,
    scene:Scene,stage:Stage)->recent_cloud::RecentCloud3D {
    assert_eq!(scene,Scene::Box,"this new load is defined only for the original box");
    let count=if stage==Stage::Clear {GROUND_RETURNS}else{INSERTED_RETURNS};
    let mut data=Vec::with_capacity(count*12);
    let mut ground_point_indices=Vec::with_capacity(GROUND_RETURNS);
    for key in ground_keys() {
        for ix in 0..4 {for iy in 0..4 {
            ground_point_indices.push((data.len()/12) as u32);
            // Strictly interior subcell lattice, with the original ground z.
            let point=[(key[0] as f32+(ix as f32+0.5)/4.0)*RESOLUTION,
                (key[1] as f32+(iy as f32+0.5)/4.0)*RESOLUTION,
                (key[2] as f32+0.5)*RESOLUTION];
            for coordinate in point {data.extend_from_slice(&coordinate.to_le_bytes());}
        }}
    }
    if stage==Stage::Inserted {
        for key in obstacle_keys(scene) {
            for coordinate in key {
                data.extend_from_slice(&((coordinate as f32+0.5)*RESOLUTION).to_le_bytes());
            }
        }
    }
    assert_eq!(data.len(),count*12);
    let cloud=schema::PointCloud2 {
        header:schema::Header {seq:sequence,stamp_sec:(stamp/1_000_000_000) as i32,
            stamp_nsec:(stamp%1_000_000_000) as i32,frame_id:"map".into()},
        height:1,width:count as i32,fields:["x","y","z"].into_iter().enumerate()
            .map(|(i,name)|schema::PointField {name:name.into(),offset:(i*4) as i32,datatype:7,count:1}).collect(),
        is_bigendian:false,point_step:12,row_step:(count*12) as i32,data,is_dense:true,
    };
    recent_cloud::RecentCloud3D {
        producer_runtime_generation:map.producer_runtime_generation,
        session_id:map.committed_map.session_id.clone(),
        session_generation:map.committed_map.session_generation,
        map_frame_generation:map.map_frame_generation,commit_boot_nonce:map.committed_map.commit_boot_nonce,
        ground_point_indices,cloud:Some(lio::LioCloudEnvelope {
            schema_version:1,backend:lio::LIO_BACKEND_DEVICE,runtime_generation:17,
            source_sequence:i64::from(sequence),source_timestamp_ns:stamp,
            correlated_odometry_source_sequence:i64::from(sequence),correlated_odometry_source_timestamp_ns:pose_stamp,
            sensor_frame_id:"synthetic_sensor".into(),body_frame_id:"base_link".into(),
            odom_frame_id:"odom".into(),map_frame_id:"map".into(),
            registration_state:lio::LIO_REGISTRATION_REGISTERED_MAP,
            source_point_count:count as i64,emitted_point_count:count as i64,cloud,
        }),
    }
}

pub(super) async fn publish(transport:&LcmTransport,map:&schema::NavigationMap3D,
    sequence:i32,stamp:i64,pose_stamp:i64,scene:Scene,stage:Stage)->PublishedDenseScan {
    let started=Instant::now();
    let packet=packet(map,sequence,stamp,pose_stamp,scene,stage);
    let points=packet.cloud.as_ref().unwrap().emitted_point_count as usize;
    let ground=packet.ground_point_indices.len();
    let body=recent_cloud::encode_body(&packet).unwrap();
    let bytes=body.len();
    publish_body(transport,recent_cloud::DEFAULT_TOPIC,recent_cloud::TYPE_NAME,body).await.unwrap();
    PublishedDenseScan {source:stamp,stage,published:Instant::now(),points,ground,bytes,
        encode_publish_ms:started.elapsed().as_secs_f64()*1000.0}
}

#[test]
#[ignore="new dense-scene wire/geometry preflight; save before the single live load run"]
fn dense_box_scene_wire_preflight() {
    let repository=Repository::temporary().unwrap();
    let mut writer=Writer::new(repository.clone(),RESOLUTION,4*1024*1024).unwrap();
    let mut cache=SurfaceGeometryCache::configured(&calibrated_config(true));
    let mut previous=None;
    for (index,stage) in [Stage::Clear,Stage::Inserted].into_iter().enumerate() {
        if let Some(at)=previous {let elapsed:Duration=Instant::now()-at;
            if elapsed<SAMPLE_PERIOD {std::thread::sleep(SAMPLE_PERIOD-elapsed);}}
        previous=Some(Instant::now());
        let stamp=current_unix_timestamp_ns().unwrap();
        let keys=scene_keys(Scene::Box,stage);
        writer.observe(keys.iter().copied(),stamp as u64).unwrap();
        let root=writer.snapshot(Domain {session:"dense-box-preflight".into(),generation:1,
            frame_generation:1,producer_generation:17},index as u64+1,stamp as u64).unwrap();
        let map=map_from_fine(&root,&repository);
        let packet=packet(&map,index as i32+1,stamp,stamp,Scene::Box,stage);
        let body=recent_cloud::encode_body(&packet).unwrap();
        assert!(body.len()<=recent_cloud::MAX_BYTES);
        let decoded=recent_cloud::decode_body(&body).unwrap();
        assert_eq!(decoded,packet);
        let cloud=decoded.cloud.as_ref().unwrap();
        let expected=if stage==Stage::Clear {GROUND_RETURNS}else{INSERTED_RETURNS};
        assert_eq!(cloud.emitted_point_count,expected as i64);
        assert_eq!(cloud.source_point_count,expected as i64);
        assert!(cloud.emitted_point_count<=lio::MAX_LIO_EMITTED_POINT_COUNT);
        assert_eq!(decoded.ground_point_indices,(0..GROUND_RETURNS as u32).collect::<Vec<_>>());
        let materialized=point_cloud_from_schema(cloud.cloud.clone()).unwrap();
        assert_eq!(materialized.points.len(),expected);
        let mut actual=BTreeSet::new();let mut ground_cells=BTreeMap::<[i32;3],usize>::new();
        let mut unique=BTreeSet::new();
        let points:Vec<_>=materialized.points.iter().map(|p|[p.x,p.y,p.z]).collect();
        for (i,point) in points.iter().enumerate() {
            assert!(point.iter().all(|v|v.is_finite()));
            let key=point.map(|v|(v/f64::from(RESOLUTION)).floor() as i32);
            assert!(keys.contains(&key));actual.insert(key);
            assert!(unique.insert(point.map(f64::to_bits)),"all dense returns must be distinct");
            if i<GROUND_RETURNS {
                assert_eq!(key[2],1);*ground_cells.entry(key).or_default()+=1;
                assert_eq!(point[2],f64::from(1.5*RESOLUTION));
            } else {assert!(obstacle_keys(Scene::Box).contains(&key));}
        }
        assert_eq!(actual,keys);
        assert_eq!(ground_cells.keys().copied().collect::<BTreeSet<_>>(),ground_keys());
        assert!(ground_cells.values().all(|&n|n==16));
        let built=cache.build(&map).unwrap();
        audit_observations(&built,Scene::Box,stage,stamp).unwrap();
        let dense=built.with_recent_obstacle_points(index as u64+1,stamp as u64,
            1_000_000_000,&decoded.ground_point_indices,
            points.into_iter().enumerate().map(|(i,p)|(i as u32,p))).unwrap();
        original_geometry_queries(&dense,Scene::Box,stage).unwrap();
        eprintln!("DENSE_PREFLIGHT stage={stage:?} points={expected} ground={} bytes={} columns={} exact_scene_keys=true distinct_points=true original_geometry_queries=passed",
            GROUND_RETURNS,body.len(),actual.iter().map(|k|[k[0],k[1]]).collect::<BTreeSet<_>>().len());
    }
    // Existing independent oracle counterexamples and analytic witness checks.
    stable_plant_preserves_recorded_near_zero_counterexample_and_normal_arcs();
    continuous_oracle_tests::preflight_regressions();
    independent_geometry_preflight().unwrap();
    eprintln!("DENSE_PREFLIGHT_PASS new_load_only=true original_oracles_unchanged=true live_runs=0");
}

#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="new dense recent box/ON load, one serial run after saved preflight; original cases unchanged"]
async fn dense_recent_box_local_planning_on() {
    let log=Arc::new(Mutex::new(Vec::new()));
    let mut harness=OccupancyHarness::start_with_dense_recent(true,Scene::Box,Some(log.clone())).await;
    let result=harness.exercise().await;let status=harness.scheduler.status();
    let cleanup=harness.shutdown().await;
    harness.plant.lock().unwrap().sample();
    let audit=harness.audit_after_cleanup();harness.dump_evidence();
    let scans=log.lock().unwrap();
    let mut load_ok=!scans.is_empty();
    for row in scans.iter() {
        eprintln!("DENSE_PUBLISHED {row:?}");
        load_ok &= row.ground==GROUND_RETURNS && row.bytes<=recent_cloud::MAX_BYTES
            && row.points==if row.stage==Stage::Clear {GROUND_RETURNS}else{INSERTED_RETURNS}
            && row.encode_publish_ms.is_finite();
    }
    load_ok &= scans.windows(2).all(|w|w[1].source>w[0].source && w[1].published>w[0].published);
    let events=harness.plant.lock().unwrap().events.clone();
    let moving:Vec<_>=events.iter().filter(|e|is_nonzero_applied(e)).collect();
    // A dense stream must coexist with actual movement over more than one
    // unchanged source TTL. Publication is not mislabeled a per-frame ACK.
    let moving_span=moving.first().zip(moving.last()).map(|(first,last)|last.at.duration_since(first.at));
    let concurrent=moving.first().zip(moving.last()).map_or(0,|(first,last)|
        scans.iter().filter(|row|row.published>=first.at && row.published<=last.at).count());
    load_ok &= moving_span.is_some_and(|span|span>Duration::from_secs(1)) && concurrent>=2
        && scans.iter().any(|row|row.stage==Stage::Inserted);
    eprintln!("DENSE_LOAD_AUDIT published={} concurrent_with_movement={} moving_span={moving_span:?} load_ok={load_ok} source_ttl_s=1 scan_period_ms=200 per_frame_ack_count=not_recorded",
        scans.len(),concurrent);
    eprintln!("DENSE_FINAL result={result:?} status={status:?} cleanup={cleanup:?} audit={audit:?} load_ok={load_ok}");
    assert!(result.is_ok() && cleanup.is_ok() && audit.is_ok() && load_ok,
        "result={result:?}; cleanup={cleanup:?}; audit={audit:?}; load_ok={load_ok}");
}
