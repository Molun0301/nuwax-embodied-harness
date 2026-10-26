//! 精确复用保存资产回归的射线、原始精细键和完整源回波保留规则。
use super::{
    support::{js, object},
    DufoMap, Options,
};
use std::{collections::HashSet, fs::File, io::Read, time::Instant};
type Key = [i32; 3];
const Q: f32 = 0.05;
const ORIGIN: [f64; 3] = [0.03, 0.03, 0.65];
fn asset_key(p: [f64; 3]) -> Key {
    let inv = 1. / f64::from(Q);
    p.map(|v| (v * inv).floor() as i32)
}
fn dense_keys(p: [f64; 3], present: &HashSet<Key>) -> Vec<Key> {
    let intervals = p.map(|x| {
        let v = x as f32;
        let down = v.next_down() as f64;
        let up = v.next_up() as f64;
        let inv = 1. / f64::from(Q);
        [
            ((x + down) * 0.5 * inv).floor() as i32,
            ((x + up) * 0.5 * inv).floor() as i32,
        ]
    });
    let mut out = vec![];
    for x in intervals[0][0]..=intervals[0][1] {
        for y in intervals[1][0]..=intervals[1][1] {
            for z in intervals[2][0]..=intervals[2][1] {
                let k = [x, y, z];
                if present.contains(&k) {
                    out.push(k);
                }
            }
        }
    }
    if out.is_empty() {
        out.push(asset_key(p));
    }
    out
}
fn corridor(person_y: Option<f64>, phase: f64) -> (Vec<[f64; 3]>, Vec<bool>) {
    let mut points = vec![];
    let mut truth = vec![];
    // 与dynamic_scene::tests::corridor逐行保持相同的循环、精度和求交顺序。
    for row in 0..61 {
        let elevation = -0.55 + row as f64 * 0.017 + phase * 0.002;
        for col in 0..141 {
            let azimuth = -0.7 + col as f64 * 0.01 + phase * 0.002;
            let direction = [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
            let mut distance = (8. - ORIGIN[0]) / direction[0];
            if direction[2] < 0. {
                distance = distance.min(-ORIGIN[2] / direction[2]);
            }
            if direction[2] > 0. {
                distance = distance.min((3. - ORIGIN[2]) / direction[2]);
            }
            if direction[1] != 0. {
                let side = if direction[1] > 0. { 3. } else { -3. };
                distance = distance.min((side - ORIGIN[1]) / direction[1]);
            }
            let mut dynamic = false;
            if let Some(y) = person_y {
                let dx = ORIGIN[0] - 3.;
                let dy = ORIGIN[1] - y;
                let a = direction[0].powi(2) + direction[1].powi(2);
                let b = 2. * (dx * direction[0] + dy * direction[1]);
                let c = dx * dx + dy * dy - 0.25_f64.powi(2);
                let discriminant = b * b - 4. * a * c;
                if discriminant >= 0. {
                    let hit = (-b - discriminant.sqrt()) / (2. * a);
                    let z = ORIGIN[2] + hit * direction[2];
                    if hit > 0. && hit < distance && (0.12..=1.8).contains(&z) {
                        distance = hit;
                        dynamic = true;
                    }
                }
            }
            points.push(std::array::from_fn(|i| ORIGIN[i] + distance * direction[i]));
            truth.push(dynamic);
        }
    }
    (points, truth)
}
pub fn evaluate(o: &Options) {
    assert!(
        o.mode == "offline" && o.sampling == "all",
        "保存回归固定离线语义与原始60帧全插入"
    );
    let started = Instant::now();
    let mut map = DufoMap::new(&o.params).unwrap();
    let mut human = HashSet::new();
    let mut static_keys = HashSet::new();
    let mut present = HashSet::new();
    let mut dense = vec![];
    for frame in 0..60 {
        let (points, truth) = corridor(
            (frame < 30).then_some(-1. + frame as f64 * 2. / 29.),
            (frame % 3) as f64,
        );
        for (&p, on_person) in points.iter().zip(truth) {
            let key = asset_key(p);
            present.insert(key);
            if on_person {
                if p[2] > 0.35 {
                    human.insert(key);
                }
            } else {
                static_keys.insert(key);
            }
        }
        let points_f32: Vec<_> = points.iter().map(|p| p.map(|v| v as f32)).collect();
        map.insert(&points_f32, ORIGIN.map(|v| v as f32), false)
            .unwrap();
        // 生产write_dense落盘为f32，随后read_dense再扩展为f64；保留这次精度往返。
        dense.extend(points_f32.iter().map(|p| p.map(f64::from)));
    }
    map.propagate();
    let mut keys: Vec<_> = present.iter().copied().collect();
    keys.sort_unstable();
    let centers: Vec<_> = keys
        .iter()
        .map(|k| k.map(|v| ((v as f64 + 0.5) * f64::from(Q)) as f32))
        .collect();
    let mut asset_exact = false;
    if let Some(path) = &o.path {
        let path = std::path::Path::new(path);
        let mut file = File::open(path.join("dense.bin")).unwrap();
        let mut magic = [0; 8];
        file.read_exact(&mut magic).unwrap();
        assert_eq!(&magic, b"NUWAXDC1");
        let mut n = [0; 4];
        file.read_exact(&mut n).unwrap();
        let mut header = vec![0; u32::from_le_bytes(n) as usize];
        file.read_exact(&mut header).unwrap();
        let mut bytes = vec![];
        file.read_to_end(&mut bytes).unwrap();
        let actual: Vec<[f32; 3]> = bytes
            .chunks_exact(12)
            .map(|p| {
                std::array::from_fn(|k| f32::from_le_bytes(p[k * 4..k * 4 + 4].try_into().unwrap()))
            })
            .collect();
        let generated: Vec<_> = dense.iter().map(|p| p.map(|v| v as f32)).collect();
        assert_eq!(actual, generated, "生成射线与失败资产dense.bin不一致");
        let source = std::fs::read(path.join("cleaned/source.xyzf32")).unwrap();
        let source_centers: HashSet<[u32; 3]> = source
            .chunks_exact(12)
            .map(|p| {
                std::array::from_fn(|k| u32::from_le_bytes(p[k * 4..k * 4 + 4].try_into().unwrap()))
            })
            .collect();
        assert_eq!(
            source_centers,
            centers.iter().map(|p| p.map(f32::to_bits)).collect(),
            "生成原始精细键与失败资产不一致"
        );
        asset_exact = true;
    }
    let fine_flags = map.seen_free(&centers).unwrap();
    let dense_f32: Vec<_> = dense.iter().map(|p| p.map(|v| v as f32)).collect();
    let dense_flags = map.seen_free(&dense_f32).unwrap();
    let mut removed: HashSet<_> = keys
        .iter()
        .zip(&fine_flags)
        .filter_map(|(&k, &free)| free.then_some(k))
        .collect();
    let fine_raw_removed = removed.len();
    let mut retained = HashSet::new();
    for (&point, &free) in dense.iter().zip(&dense_flags) {
        for k in dense_keys(point, &present) {
            if free {
                if present.contains(&k) {
                    removed.insert(k);
                }
            } else {
                retained.insert(k);
            }
        }
    }
    removed.retain(|k| !retained.contains(k));
    if let Some(path) = &o.final_mask {
        // 保存场景按原始精细键字典序输出最终删除掩码，完整复现retained保护后的结果。
        std::fs::write(
            path,
            keys.iter()
                .map(|k| u8::from(removed.contains(k)))
                .collect::<Vec<_>>(),
        )
        .unwrap();
    }
    let human: HashSet<_> = human.difference(&static_keys).copied().collect();
    let human_removed = human.intersection(&removed).count();
    let static_removed = static_keys.intersection(&removed).count();
    if let Some(path) = &o.metrics {
        use std::io::Write;
        let mut file = File::create(path).unwrap();
        writeln!(file, "x,y,z,removed,human,static").unwrap();
        for k in &keys {
            writeln!(
                file,
                "{},{},{},{},{},{}",
                k[0],
                k[1],
                k[2],
                removed.contains(k),
                human.contains(k),
                static_keys.contains(k)
            )
            .unwrap();
        }
    }
    println!(
        "{}",
        object(&[
            ("scene", js("saved_corridor")),
            ("frames", "60".into()),
            ("points", dense.len().to_string()),
            ("resolution", o.params.resolution.to_string()),
            ("unknown", o.params.inflate_unknown.to_string()),
            ("hits", o.params.inflate_hits_dist.to_string()),
            ("window_frames", o.params.sliding_window_size.to_string()),
            ("asset_exact_verified", asset_exact.to_string()),
            ("source_fine_keys", keys.len().to_string()),
            ("static_keys", static_keys.len().to_string()),
            ("static_removed", static_removed.to_string()),
            ("human_body_unique_keys", human.len().to_string()),
            ("human_removed", human_removed.to_string()),
            (
                "human_removed_fraction",
                (human_removed as f64 / human.len() as f64).to_string()
            ),
            (
                "passes_saved_regression",
                (human_removed as f64 / human.len() as f64 > 0.95 && static_removed == 0)
                    .to_string()
            ),
            (
                "fine_center_free_before_retained_guard",
                fine_raw_removed.to_string()
            ),
            ("removed_after_retained_guard", removed.len().to_string()),
            (
                "dense_points_free",
                dense_flags.iter().filter(|v| **v).count().to_string()
            ),
            ("whole_run_s", started.elapsed().as_secs_f64().to_string())
        ])
    );
}
