//! 合成走廊验证去动态行为；实录基准默认忽略，用发布构建单独运行。
use super::*;

/// 轴对齐盒子，用于构造墙、地面和行人。
#[derive(Clone, Copy)]
struct Aabb {
    min: [f32; 3],
    max: [f32; 3],
}

impl Aabb {
    /// 射线与盒子的最近正交点距离（slab 方法）。
    fn hit(&self, origin: [f32; 3], dir: [f32; 3]) -> Option<f32> {
        let (mut t0, mut t1) = (0.0f32, f32::INFINITY);
        for axis in 0..3 {
            if dir[axis].abs() < 1e-9 {
                if origin[axis] < self.min[axis] || origin[axis] > self.max[axis] {
                    return None;
                }
                continue;
            }
            let inv = 1.0 / dir[axis];
            let (mut a, mut b) = ((self.min[axis] - origin[axis]) * inv, (self.max[axis] - origin[axis]) * inv);
            if a > b {
                std::mem::swap(&mut a, &mut b);
            }
            t0 = t0.max(a);
            t1 = t1.min(b);
            if t0 > t1 {
                return None;
            }
        }
        (t0 > 1e-4).then_some(t0)
    }
}

// 封闭室内走廊：DUFOMap 只沿有回波的射线记录空闲，四周与天花板保证射线都有回波。
// 各表面错开 0.1 米体素边界：命中点若恰好落在体素面上，不同平台三角函数末位差异会把它分到两侧不同体素。
const WALLS: [Aabb; 6] = [
    Aabb { min: [-1.0, -1.63, -0.13], max: [12.0, 1.63, 0.03] },  // 地面
    Aabb { min: [-1.0, -1.63, 2.97], max: [12.0, 1.63, 3.13] },   // 天花板
    Aabb { min: [-1.0, -1.73, 0.03], max: [12.0, -1.63, 2.97] },  // 左墙
    Aabb { min: [-1.0, 1.63, 0.03], max: [12.0, 1.73, 2.97] },    // 右墙
    Aabb { min: [9.97, -1.63, 0.03], max: [10.07, 1.63, 2.97] },  // 尽头墙
    Aabb { min: [-1.13, -1.63, 0.03], max: [-1.03, 1.63, 2.97] }, // 后墙
];

/// 第 `frame` 帧的行人位置：前 30 帧从 y=-1.0 横穿到 y=1.0，之后离开。
fn person(frame: usize) -> Option<Aabb> {
    (frame < 30).then(|| {
        let y = -1.0 + 2.0 * frame as f32 / 29.0 + 0.017;
        Aabb { min: [5.83, y - 0.2, 0.03], max: [6.17, y + 0.2, 1.73] }
    })
}

/// 传感器从 x=0.5 匀速前进到 x=3.5，高 0.5 米；返回 (原点, 命中点, 每点是否打在行人上)。
fn scan(frame: usize) -> ([f32; 3], Vec<[f32; 3]>, Vec<bool>) {
    let origin = [0.53 + 3.0 * frame as f32 / 59.0, 0.013, 0.53];
    let walker = person(frame);
    let (mut points, mut on_person) = (Vec::new(), Vec::new());
    for az in -60..=60 {
        // 俯仰覆盖 ±30°，保证后续射线能扫过行人原位置的整个身高。
        for el in -30..=30 {
            let (a, e) = ((az as f32 * 0.75).to_radians(), (el as f32).to_radians());
            let dir = [a.cos() * e.cos(), a.sin() * e.cos(), e.sin()];
            let static_hit = WALLS.iter().filter_map(|wall| wall.hit(origin, dir)).fold(f32::INFINITY, f32::min);
            let person_hit = walker.and_then(|box_| box_.hit(origin, dir)).unwrap_or(f32::INFINITY);
            let t = static_hit.min(person_hit);
            if t.is_finite() && t < 30.0 {
                points.push([origin[0] + dir[0] * t, origin[1] + dir[1] * t, origin[2] + dir[2] * t]);
                on_person.push(person_hit <= static_hit);
            }
        }
    }
    (origin, points, on_person)
}

fn ratio(flags: impl Iterator<Item = bool>) -> f64 {
    let (mut yes, mut all) = (0usize, 0usize);
    for flag in flags {
        all += 1;
        yes += flag as usize;
    }
    yes as f64 / all.max(1) as f64
}

#[test]
fn offline_removes_crossing_person_and_keeps_static_structure() {
    let mut map = DufoMap::new(&Params::default()).unwrap();
    let (mut cloud, mut truth) = (Vec::new(), Vec::new());
    for frame in 0..60 {
        let (origin, points, on_person) = scan(frame);
        map.insert(&points, origin, false).unwrap();
        cloud.extend_from_slice(&points);
        truth.extend(on_person);
    }
    map.propagate();
    let dynamic = map.seen_free(&cloud).unwrap();
    let person = |above: bool| ratio(dynamic.iter().zip(&truth).zip(&cloud)
        .filter(|((_, p), q)| **p && (q[2] > 0.33) == above).map(|((d, _), _)| *d));
    let static_lost = ratio(dynamic.iter().zip(&truth).filter(|(_, p)| !**p).map(|(d, _)| *d));
    // 行人横穿后，原位置被后续射线观测为空；墙和地面不会被射线穿过。
    // 命中点周围 inflate_hits_dist（0.2 米）内不判为空，所以贴地的脚部按设计保留，只统计不断言。
    let (body, feet) = (person(true), person(false));
    eprintln!("DUFOMAP_SYNTHETIC person_body_removed={body:.3} person_feet_removed={feet:.3} static_lost={static_lost:.4}");
    assert!(body > 0.95, "行人身体（0.3米以上）删除比例过低：{body:.3}");
    assert!(static_lost < 0.01, "静态结构误删比例过高：{static_lost:.4}");
    assert!(map.memory_bytes() > 0 && map.num_nodes() > 0);
}

#[test]
fn online_marks_person_entering_space_seen_free_earlier() {
    let mut map = DufoMap::new(&Params::default()).unwrap();
    let (mut late_person, mut late_static) = (Vec::new(), Vec::new());
    for frame in 0..60 {
        let (origin, points, on_person) = scan(frame);
        map.insert(&points, origin, true).unwrap();
        // 在线模式：用已建地图判定本帧。行人后半程走进前面帧已观测为空的位置。
        let dynamic = map.seen_free(&points).unwrap();
        for (flag, person) in dynamic.into_iter().zip(on_person) {
            if frame >= 15 {
                if person { late_person.push(flag) } else { late_static.push(flag) }
            }
        }
    }
    let person_flagged = ratio(late_person.into_iter());
    let static_flagged = ratio(late_static.into_iter());
    assert!(person_flagged > 0.3, "在线模式未识别进入已知空区域的行人：{person_flagged:.3}");
    assert!(static_flagged < 0.01, "在线模式误判静态结构：{static_flagged:.4}");
}

#[test]
fn clear_resets_map_and_rejects_invalid_input() {
    assert!(DufoMap::new(&Params { resolution: 0.0, ..Params::default() }).is_err());
    assert!(DufoMap::new(&Params { levels: 1, ..Params::default() }).is_err());
    let mut map = DufoMap::new(&Params::default()).unwrap();
    assert!(map.insert(&[[1.0, 0.0, 0.0]], [f32::NAN, 0.0, 0.0], true).is_err());
    let (origin, points, _) = scan(40);
    map.insert(&points, origin, true).unwrap();
    assert!(map.num_nodes() > 0);
    map.clear();
    assert!(map.seen_free(&points).unwrap().iter().all(|flag| !flag), "清空后不应保留空闲证据");
}

/// 单位四元数 (w, x, y, z) 旋转向量。
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let t = [2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

/// 实录在线基准：每帧插入并判定当前帧，每 100 帧输出插入/判定耗时、八叉树内存和动态比例。
/// 输入格式同 `pgo_map::tests::pgo_recorded_real_scale_rebuild`（每帧 68 字节头 + 机体系 f32 点）。
#[test]
#[ignore = "需要显式提供实录配对扫描，release构建独立执行"]
fn recorded_online_cost() {
    use std::io::Read;
    let path = std::env::var("NUWAX_DUFOMAP_RECORDED_SCANS").expect("指定实录配对扫描文件");
    let frames: usize = std::env::var("NUWAX_DUFOMAP_RECORDED_FRAMES").ok().and_then(|v| v.parse().ok()).unwrap_or(3000);
    let origin_body: [f64; 3] = std::env::var("NUWAX_DUFOMAP_SENSOR_ORIGIN_BODY").ok()
        .map(|v| { let p: Vec<f64> = v.trim_matches(['[', ']']).split(',').map(|x| x.trim().parse().unwrap()).collect(); [p[0], p[1], p[2]] })
        .unwrap_or([0.28945, 0.0, -0.046825]);
    let mut file = std::io::BufReader::new(std::fs::File::open(path).unwrap());
    let mut map = DufoMap::new(&Params::default()).unwrap();
    let (mut insert_s, mut query_s, mut points_n, mut dynamic_n) = (0.0f64, 0.0f64, 0usize, 0usize);
    let mut last: Option<([f32; 3], Vec<[f32; 3]>)> = None;
    let started = std::time::Instant::now();
    for frame in 1..=frames {
        let mut header = [0u8; 68];
        if file.read_exact(&mut header).is_err() { break; }
        let pose: [f64; 7] = std::array::from_fn(|i| f64::from_le_bytes(header[8 + i * 8..16 + i * 8].try_into().unwrap()));
        let size = u32::from_le_bytes(header[64..68].try_into().unwrap()) as usize;
        let mut bytes = vec![0u8; size * 12];
        file.read_exact(&mut bytes).unwrap();
        let (t, q) = ([pose[0], pose[1], pose[2]], [pose[6], pose[3], pose[4], pose[5]]);
        let to_world = |p: [f64; 3]| { let r = rotate(q, p); [(r[0] + t[0]) as f32, (r[1] + t[1]) as f32, (r[2] + t[2]) as f32] };
        let points: Vec<[f32; 3]> = bytes.chunks_exact(12).map(|b| to_world([
            f32::from_le_bytes(b[..4].try_into().unwrap()) as f64,
            f32::from_le_bytes(b[4..8].try_into().unwrap()) as f64,
            f32::from_le_bytes(b[8..12].try_into().unwrap()) as f64])).collect();
        let origin = to_world(origin_body);
        let stage = std::time::Instant::now();
        map.insert(&points, origin, true).unwrap();
        insert_s += stage.elapsed().as_secs_f64();
        let stage = std::time::Instant::now();
        let dynamic = map.seen_free(&points).unwrap();
        query_s += stage.elapsed().as_secs_f64();
        points_n += points.len();
        last = Some((origin, points.clone()));
        dynamic_n += dynamic.iter().filter(|flag| **flag).count();
        if frame % 100 == 0 {
            eprintln!("DUFOMAP_ONLINE frames={frame} insert_ms={:.2} query_ms={:.3} memory_mb={:.1} nodes={} dynamic_ratio={:.4}",
                insert_s * 1e3 / 100.0, query_s * 1e3 / 100.0, map.memory_bytes() as f64 / 1048576.0, map.num_nodes(),
                dynamic_n as f64 / points_n.max(1) as f64);
            (insert_s, query_s, points_n, dynamic_n) = (0.0, 0.0, 0, 0);
        }
    }
    // 真实数据自检：射线中点位于传感器与命中点之间，离命中点超过保护距离时应被观测为空。
    if let Some((origin, points)) = last {
        let mids: Vec<[f32; 3]> = points.iter().filter(|p| {
            let d = ((p[0] - origin[0]).powi(2) + (p[1] - origin[1]).powi(2) + (p[2] - origin[2]).powi(2)).sqrt();
            d > 1.5
        }).map(|p| [(p[0] + origin[0]) / 2.0, (p[1] + origin[1]) / 2.0, (p[2] + origin[2]) / 2.0]).collect();
        let free = map.seen_free(&mids).unwrap().iter().filter(|flag| **flag).count();
        eprintln!("DUFOMAP_SANITY ray_midpoints={} seen_free_ratio={:.3}", mids.len(), free as f64 / mids.len().max(1) as f64);
    }
    eprintln!("DUFOMAP_ONLINE_TOTAL seconds={:.1}", started.elapsed().as_secs_f64());
}


/// 回归：负坐标转体素键曾依赖未定义行为，aarch64 上所有负坐标落到同一键，空闲证据全部错位。
#[test]
fn negative_coordinates_keep_distinct_voxels_and_free_space() {
    let mut map = DufoMap::new(&Params::default()).unwrap();
    let origin = [-20.03f32, -30.03, -4.97];
    // 朝 -x 方向的一面墙，全部点都在负坐标区。
    let wall: Vec<[f32; 3]> = (0..40).flat_map(|i| (0..20).map(move |j| [-26.03, -32.03 + i as f32 * 0.1, -5.97 + j as f32 * 0.1])).collect();
    map.insert(&wall, origin, true).unwrap();
    let free = map.seen_free(&[[-23.03, -30.03, -4.97], [-24.53, -31.03, -5.47]]).unwrap();
    assert!(free.iter().all(|flag| *flag), "传感器与墙之间应被观测为空：{free:?}");
    assert!(!map.seen_free(&[[-26.03, -30.03, -4.97]]).unwrap()[0], "墙面本身不应为空");
}
