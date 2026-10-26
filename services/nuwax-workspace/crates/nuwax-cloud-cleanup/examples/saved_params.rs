//! 同一保存资产的参数复核，避免用减少候选掩盖静态结构误删。
use nalgebra::{Matrix4, Vector4};
use nuwax_cloud_cleanup::{
    offline_residual_cleanup::{asset_key, CompletedMap},
    scan_log::ScanReader,
};
use nuwax_dufomap_native::{DownSampling, DufoMap, Params};
use std::{collections::HashSet, fs::File, io::Read, path::Path};
fn world(m: &Matrix4<f64>, p: [f32; 3]) -> [f32; 3] {
    let q = m * Vector4::new(p[0] as f64, p[1] as f64, p[2] as f64, 1.);
    [q.x as f32, q.y as f32, q.z as f32]
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let folder = Path::new(&args[1]);
    let fine = CompletedMap::read(&folder.join("original-fine.bin")).unwrap();
    let mut file = File::open(folder.join("original-dense.bin")).unwrap();
    let mut h = [0; 12];
    file.read_exact(&mut h).unwrap();
    assert_eq!(&h[..8], b"NUWAXDC1");
    let mut header = vec![0; u32::from_le_bytes(h[8..].try_into().unwrap()) as usize];
    file.read_exact(&mut header).unwrap();
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).unwrap();
    let dense: Vec<[f32; 3]> = bytes
        .chunks_exact(12)
        .map(|p| {
            std::array::from_fn(|i| f32::from_le_bytes(p[i * 4..i * 4 + 4].try_into().unwrap()))
        })
        .collect();
    let centers: Vec<_> = fine
        .keys
        .iter()
        .map(|p| p.map(|v| ((v as f64 + 0.5) * fine.voxel_size_m as f64) as f32))
        .collect();
    let extended = args.get(2).is_some_and(|v| v == "extended");
    let resolutions = if extended {
        vec![0.1, 0.15, 0.2]
    } else {
        vec![0.1]
    };
    let compensations = if extended {
        vec![false, true]
    } else {
        vec![true]
    };
    let unknowns = if extended { vec![1] } else { vec![1, 2] };
    for resolution in resolutions {
        for &compensation in &compensations {
            for &unknown in &unknowns {
                for sampling in if extended {
                    vec![DownSampling::Center]
                } else {
                    vec![
                        DownSampling::Center,
                        DownSampling::Centroid,
                        DownSampling::None,
                    ]
                } {
                    for protect in [0.1, 0.2, 0.3] {
                        let params = Params {
                            resolution,
                            inflate_unknown_compensation: compensation,
                            inflate_unknown: unknown,
                            down_sampling: sampling,
                            inflate_hits_dist: protect,
                            ..Params::default()
                        };
                        let mut map = DufoMap::new(&params).unwrap();
                        let mut reader = ScanReader::open(&folder.join("raw-scans.scan")).unwrap();
                        while let Some(scan) = reader.next_scan().unwrap() {
                            let m = Matrix4::from_column_slice(&scan.metadata.odom_from_body);
                            let points: Vec<_> =
                                scan.points.into_iter().map(|p| world(&m, p)).collect();
                            map.insert(
                                &points,
                                world(&m, scan.metadata.sensor_origin_body.unwrap()),
                                false,
                            )
                            .unwrap();
                        }
                        map.propagate();
                        let flags = map.seen_free(&dense).unwrap();
                        let mut dynamic = HashSet::new();
                        let mut retained = HashSet::new();
                        for (&p, d) in dense.iter().zip(flags) {
                            let k = asset_key(p.map(|v| v as f64), fine.voxel_size_m);
                            if d {
                                dynamic.insert(k);
                            } else {
                                retained.insert(k);
                            }
                        }
                        for (&k, d) in fine.keys.iter().zip(map.seen_free(&centers).unwrap()) {
                            if d {
                                dynamic.insert(k);
                            }
                        }
                        dynamic.retain(|k| !retained.contains(k));
                        println!(
                            "{}",
                            serde_json::json!({"resolution":resolution,"compensation":compensation,"inflate_unknown":unknown,"down_sampling":format!("{sampling:?}"),"inflate_hits_dist":protect,"candidate_voxels":dynamic.len(),"keys":dynamic})
                        );
                    }
                }
            }
        }
    }
}
