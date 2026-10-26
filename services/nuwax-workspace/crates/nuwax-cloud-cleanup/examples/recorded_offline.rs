//! 实录离线参数基准：扫描不改输入，记录选择规则、日志磁盘和全源点候选。
use nalgebra::{Isometry3, Matrix4, Point3, Quaternion, Translation3, UnitQuaternion};
use nuwax_cloud_cleanup::{
    offline_dufomap,
    scan_log::{SavePolicy, ScanLog, ScanMetadata, ScanReader, Session},
};
use nuwax_dufomap_native::{DownSampling, DufoMap, Params};
use std::{
    collections::HashSet,
    fs::File,
    io::{BufReader, Read},
    path::PathBuf,
    time::Instant,
};
fn read_scan(reader: &mut impl Read) -> Option<(u64, Isometry3<f64>, Vec<[f32; 3]>)> {
    let mut h = [0; 68];
    if reader.read_exact(&mut h).is_err() {
        return None;
    }
    let stamp = u64::from_le_bytes(h[..8].try_into().unwrap());
    let p: [f64; 7] =
        std::array::from_fn(|i| f64::from_le_bytes(h[8 + i * 8..16 + i * 8].try_into().unwrap()));
    let pose = Isometry3::from_parts(
        Translation3::new(p[0], p[1], p[2]),
        UnitQuaternion::new_normalize(Quaternion::new(p[6], p[3], p[4], p[5])),
    );
    let n = u32::from_le_bytes(h[64..].try_into().unwrap()) as usize;
    let mut bytes = vec![0; n * 12];
    reader.read_exact(&mut bytes).unwrap();
    let points = bytes
        .chunks_exact(12)
        .map(|b| {
            std::array::from_fn(|a| f32::from_le_bytes(b[a * 4..a * 4 + 4].try_into().unwrap()))
        })
        .collect();
    Some((stamp, pose, points))
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let path = &args[1];
    let limit: usize = args[2].parse().unwrap();
    let rule = &args[3];
    let unknown: u32 = args[4].parse().unwrap();
    let sampling = &args[5];
    let range: f32 = args[6].parse().unwrap();
    let root = PathBuf::from(&args[7]);
    std::fs::create_dir_all(&root).unwrap();
    let session = Session {
        session_id: format!("recorded-{limit}-{rule}-{unknown}-{sampling}-{range}"),
        generation: 1,
    };
    let mut log = ScanLog::open(&root, session).unwrap();
    let policy = if rule == "every" {
        SavePolicy {
            translation_m: 0.,
            rotation_deg: 0.,
            interval_s: 0.,
        }
    } else {
        SavePolicy::default()
    };
    let mut last = None;
    let (mut frames, mut saved, mut count, mut first, mut last_stamp) = (0, 0, 0, 0, 0);
    let mut inp = BufReader::new(File::open(path).unwrap());
    let started = Instant::now();
    while frames < limit {
        let Some((stamp, pose, points)) = read_scan(&mut inp) else {
            break;
        };
        if first == 0 {
            first = stamp;
        }
        last_stamp = stamp;
        frames += 1;
        count += points.len();
        let metadata = ScanMetadata {
            source_timestamp_ns: stamp,
            source_sequence: frames as u64,
            sensor: "L2".into(),
            source_generation: 1,
            odom_from_body: pose.to_homogeneous().as_slice().try_into().unwrap(),
            map_from_odom: Matrix4::identity().as_slice().try_into().unwrap(),
            sensor_origin_body: Some([0.28945, 0., -0.046825]),
            observation_minimum_range_m: None,
            point_count: points.len(),
        };
        if policy.selects(last.as_ref(), &metadata) {
            log.append(&metadata, points.into_iter(), None).unwrap();
            saved += 1;
            last = Some(metadata);
        }
        log.progress(stamp, 0).unwrap();
    }
    log.sync().unwrap();
    let encode_s = started.elapsed().as_secs_f64();
    let bytes = std::fs::metadata(&log.path).unwrap().len();
    let mut params = if args.get(8).is_some_and(|v| v == "upstream") {
        Params::default()
    } else {
        offline_dufomap::params(50).unwrap()
    };
    params.inflate_unknown = unknown;
    params.down_sampling = match sampling.as_str() {
        "none" => DownSampling::None,
        "centroid" => DownSampling::Centroid,
        _ => DownSampling::Center,
    };
    params.max_range = range;
    let started = Instant::now();
    let mut map = DufoMap::new(&params).unwrap();
    let mut reader = ScanReader::open(&log.path).unwrap();
    while let Some(scan) = reader.next_scan().unwrap() {
        let pose = Matrix4::from_column_slice(&scan.metadata.odom_from_body);
        let world = |p: [f32; 3]| {
            let q = pose * Point3::new(p[0] as f64, p[1] as f64, p[2] as f64).to_homogeneous();
            [q.x as f32, q.y as f32, q.z as f32]
        };
        let points: Vec<_> = scan.points.into_iter().map(world).collect();
        map.insert(
            &points,
            world(scan.metadata.sensor_origin_body.unwrap()),
            false,
        )
        .unwrap();
    }
    map.propagate();
    let insert_s = started.elapsed().as_secs_f64();
    let mut inp = BufReader::new(File::open(path).unwrap());
    let started = Instant::now();
    let (mut dynamic, mut queried) = (0, 0);
    let (mut candidate_keys, mut static_keys) = (HashSet::new(), HashSet::new());
    let mut mid_free = 0;
    let mut mid_count = 0;
    for _ in 0..frames {
        let (_, pose, points) = read_scan(&mut inp).unwrap();
        let world: Vec<[f32; 3]> = points
            .iter()
            .map(|p| {
                let q = pose * Point3::new(p[0] as f64, p[1] as f64, p[2] as f64);
                [q.x as f32, q.y as f32, q.z as f32]
            })
            .collect();
        let flags = map.seen_free(&world).unwrap();
        for (p, d) in world.iter().zip(flags) {
            queried += 1;
            dynamic += d as usize;
            let k = p.map(|v| (v as f64 / (0.05f32 as f64)).floor() as i32);
            if d {
                candidate_keys.insert(k);
            } else {
                static_keys.insert(k);
            }
        }
        let origin = pose * Point3::new(0.28945, 0., -0.046825);
        let mids: Vec<_> = world
            .iter()
            .filter(|p| {
                ((p[0] as f64 - origin.x).powi(2)
                    + (p[1] as f64 - origin.y).powi(2)
                    + (p[2] as f64 - origin.z).powi(2))
                .sqrt()
                    > 1.5
            })
            .map(|p| {
                [
                    (p[0] + origin.x as f32) * 0.5,
                    (p[1] + origin.y as f32) * 0.5,
                    (p[2] + origin.z as f32) * 0.5,
                ]
            })
            .collect();
        mid_free += map.seen_free(&mids).unwrap().iter().filter(|v| **v).count();
        mid_count += mids.len();
    }
    let query_s = started.elapsed().as_secs_f64();
    candidate_keys.retain(|k| !static_keys.contains(k));
    let duration = (last_stamp - first) as f64 * 1e-9;
    println!(
        "{}",
        serde_json::json!({"frames":frames,"points":count,"saved_frames":saved,"rule":rule,"inflate_unknown":unknown,"down_sampling":sampling,"max_range":range,
        "duration_s":duration,"log_bytes":bytes,"growth_mib_min":bytes as f64/1048576.*60./duration,"encode_s":encode_s,"offline_insert_s":insert_s,"query_all_s":query_s,
        "offline_total_s":insert_s+query_s,"dynamic_points":dynamic,"queried_points":queried,"candidate_voxels":candidate_keys.len(),"resolution":params.resolution,"inflate_unknown_compensation":params.inflate_unknown_compensation,"inflate_hits_dist":params.inflate_hits_dist,"midpoint_free_ratio":mid_free as f64/mid_count as f64,
        "tree_bytes":map.memory_bytes(),"nodes":map.num_nodes()})
    );
}
