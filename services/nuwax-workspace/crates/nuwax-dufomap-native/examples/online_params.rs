//! 有界样本的在线参数评估：合成真值和实录相对指标分别输出。
use nuwax_dufomap_native::{DownSampling, DufoMap, Params};
use std::{
    fs::File,
    io::{BufReader, Read},
    time::Instant,
};

const SAMPLE_CAP: usize = 100_000;
const GROUPS: [&str; 8] = [
    "floor",
    "ceiling",
    "wall",
    "post",
    "table_top",
    "table_leg",
    "person_body",
    "person_feet",
];

#[derive(Clone, Copy)]
struct Sample {
    point: [f32; 3],
    group: usize,
    online: bool,
}

struct Reservoir {
    points: Vec<Sample>,
    count: u64,
    state: u64,
}
impl Reservoir {
    fn new() -> Self {
        Self {
            points: Vec::with_capacity(SAMPLE_CAP),
            count: 0,
            state: 0x4d595df4d0f33173,
        }
    }
    fn push(&mut self, sample: Sample) {
        self.count += 1;
        // 固定种子的水塘抽样，所有参数使用相同样本，内存不随实录长度增长。
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        if self.points.len() < SAMPLE_CAP {
            self.points.push(sample);
        } else {
            let index = (self.state % self.count) as usize;
            if index < SAMPLE_CAP {
                self.points[index] = sample;
            }
        }
    }
}

struct Scan {
    origin: [f32; 3],
    points: Vec<[f32; 3]>,
    groups: Vec<usize>,
    timestamp: f64,
}

#[derive(Clone, Copy)]
struct Box3 {
    low: [f32; 3],
    high: [f32; 3],
    group: usize,
}
impl Box3 {
    fn hit(self, origin: [f32; 3], dir: [f32; 3]) -> Option<f32> {
        let (mut lo, mut hi) = (0.0f32, f32::INFINITY);
        for k in 0..3 {
            if dir[k].abs() < 1e-9 {
                if origin[k] < self.low[k] || origin[k] > self.high[k] {
                    return None;
                }
            } else {
                let (a, b) = (
                    (self.low[k] - origin[k]) / dir[k],
                    (self.high[k] - origin[k]) / dir[k],
                );
                lo = lo.max(a.min(b));
                hi = hi.min(a.max(b));
                if lo > hi {
                    return None;
                }
            }
        }
        (lo > 1e-4).then_some(lo)
    }
}

fn boxes(frame: usize) -> Vec<Box3> {
    let mut out = vec![
        Box3 {
            low: [-1., -1.63, -0.13],
            high: [12., 1.63, 0.03],
            group: 0,
        },
        Box3 {
            low: [-1., -1.63, 2.97],
            high: [12., 1.63, 3.13],
            group: 1,
        },
        Box3 {
            low: [-1., -1.73, 0.03],
            high: [12., -1.63, 2.97],
            group: 2,
        },
        Box3 {
            low: [-1., 1.63, 0.03],
            high: [12., 1.73, 2.97],
            group: 2,
        },
        Box3 {
            low: [9.97, -1.63, 0.03],
            high: [10.07, 1.63, 2.97],
            group: 2,
        },
        Box3 {
            low: [-1.13, -1.63, 0.03],
            high: [-1.03, 1.63, 2.97],
            group: 2,
        },
        // 细杆、桌面和桌腿独立记账，避免大量墙点淹没细结构误删。
        Box3 {
            low: [4.43, -1.05, 0.03],
            high: [4.49, -0.99, 1.75],
            group: 3,
        },
        Box3 {
            low: [4.85, 0.80, 0.77],
            high: [5.75, 1.45, 0.83],
            group: 4,
        },
    ];
    for x in [4.87, 5.65] {
        for y in [0.82, 1.35] {
            out.push(Box3 {
                low: [x, y, 0.03],
                high: [x + 0.06, y + 0.06, 0.77],
                group: 5,
            });
        }
    }
    // 前15帧建立空闲证据，接着30帧横穿，最后45帧观测离开后的场景。
    if (15..45).contains(&frame) {
        let y = -1.0 + 2.0 * (frame - 15) as f32 / 29.0 + 0.017;
        out.push(Box3 {
            low: [5.83, y - 0.2, 0.03],
            high: [6.17, y + 0.2, 1.73],
            group: 6,
        });
    }
    out
}

fn synthetic(frame: usize, dense: bool) -> Scan {
    let origin = [0.53 + 3.0 * frame as f32 / 89.0, 0.013, 0.53];
    let scene = boxes(frame);
    let (naz, nel) = if dense { (121, 61) } else { (64, 32) };
    let (mut points, mut groups) = (Vec::new(), Vec::new());
    for az in 0..naz {
        for el in 0..nel {
            // 固定扫描线，稀疏场景约2048点/帧，不靠跨帧抖动人为填满未知区域。
            let a = (-45.0 + 90.0 * az as f32 / (naz - 1) as f32).to_radians();
            let e = (-30.0 + 60.0 * el as f32 / (nel - 1) as f32).to_radians();
            let dir = [a.cos() * e.cos(), a.sin() * e.cos(), e.sin()];
            if let Some((distance, mut group)) = scene
                .iter()
                .filter_map(|b| b.hit(origin, dir).map(|d| (d, b.group)))
                .min_by(|a, b| a.0.total_cmp(&b.0))
            {
                let point = std::array::from_fn(|k| origin[k] + dir[k] * distance);
                if group == 6 && point[2] <= 0.33 {
                    group = 7;
                }
                points.push(point);
                groups.push(group);
            }
        }
    }
    Scan {
        origin,
        points,
        groups,
        timestamp: frame as f64 / 15.0,
    }
}

fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let t = [
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    ]
}

fn recorded(reader: &mut BufReader<File>) -> Option<Scan> {
    let mut header = [0u8; 68];
    // 只有完整文件结尾允许无下一帧，截断输入直接失败。
    if reader.read(&mut header[..1]).unwrap() == 0 {
        return None;
    }
    reader.read_exact(&mut header[1..]).unwrap();
    let timestamp = u64::from_le_bytes(header[..8].try_into().unwrap()) as f64 * 1e-9;
    let pose: [f64; 7] = std::array::from_fn(|k| {
        f64::from_le_bytes(header[8 + k * 8..16 + k * 8].try_into().unwrap())
    });
    let count = u32::from_le_bytes(header[64..68].try_into().unwrap()) as usize;
    assert!(count < 1_000_000, "单帧点数不合理");
    let mut bytes = vec![0; count * 12];
    reader.read_exact(&mut bytes).unwrap();
    let world = |point: [f64; 3]| {
        let r = rotate([pose[6], pose[3], pose[4], pose[5]], point);
        std::array::from_fn(|k| (r[k] + pose[k]) as f32)
    };
    let points: Vec<_> = bytes
        .chunks_exact(12)
        .map(|p| {
            world(std::array::from_fn(|k| {
                f32::from_le_bytes(p[k * 4..k * 4 + 4].try_into().unwrap()) as f64
            }))
        })
        .collect();
    Some(Scan {
        origin: world([0.28945, 0.0, -0.046825]),
        groups: vec![0; count],
        points,
        timestamp,
    })
}

#[repr(C)]
struct Timespec {
    seconds: i64,
    nanoseconds: i64,
}
unsafe extern "C" {
    fn clock_gettime(clock: i32, result: *mut Timespec) -> i32;
}
fn cpu_seconds() -> f64 {
    let mut t = Timespec {
        seconds: 0,
        nanoseconds: 0,
    };
    assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0);
    t.seconds as f64 + t.nanoseconds as f64 * 1e-9
}
fn peak_kib() -> usize {
    std::fs::read_to_string("/proc/self/status")
        .unwrap()
        .lines()
        .find_map(|line| {
            line.strip_prefix("VmHWM:")
                .map(|s| s.split_whitespace().next().unwrap().parse().unwrap())
        })
        .unwrap()
}

fn config(name: &str) -> (Params, usize) {
    let mut p = Params {
        resolution: 0.15,
        inflate_unknown_compensation: false,
        inflate_hits_dist: 0.25,
        ..Params::default()
    };
    let mut merge = 1;
    match name {
        "baseline" => {}
        "upstream" => p = Params::default(),
        "res010" => p.resolution = 0.1,
        "res020" => p.resolution = 0.2,
        "unknown0" => p.inflate_unknown = 0,
        "unknown2" => p.inflate_unknown = 2,
        "unknown3" => p.inflate_unknown = 3,
        "res020_unknown2" => {
            p.resolution = 0.2;
            p.inflate_unknown = 2;
        }
        "res020_unknown3" => {
            p.resolution = 0.2;
            p.inflate_unknown = 3;
        }
        "res025_unknown2" => {
            p.resolution = 0.25;
            p.inflate_unknown = 2;
        }
        "res025_unknown3" => {
            p.resolution = 0.25;
            p.inflate_unknown = 3;
        }
        "compensation" => p.inflate_unknown_compensation = true,
        "hits020" => p.inflate_hits_dist = 0.2,
        "hits030" => p.inflate_hits_dist = 0.3,
        "none" => p.down_sampling = DownSampling::None,
        "centroid" => p.down_sampling = DownSampling::Centroid,
        "uniform" => p.down_sampling = DownSampling::Uniform,
        "range10" => p.max_range = 10.0,
        "range20" => p.max_range = 20.0,
        "merge3" => merge = 3,
        _ => panic!("未知参数名称: {name}"),
    }
    (p, merge)
}

struct Eval {
    map: DufoMap,
    samples: Reservoir,
    total: [u64; 8],
    online: [u64; 8],
    frames: usize,
    insert_s: f64,
    query_s: f64,
    assemble_s: f64,
    cpu_s: f64,
    calls: usize,
    max_pending_points: usize,
    max_origin_shift: f32,
    first_time: Option<f64>,
    last_time: f64,
    last_points: Vec<[f32; 3]>,
    last_origin: [f32; 3],
}
impl Eval {
    fn new(p: &Params) -> Self {
        Self {
            map: DufoMap::new(p).unwrap(),
            samples: Reservoir::new(),
            total: [0; 8],
            online: [0; 8],
            frames: 0,
            insert_s: 0.,
            query_s: 0.,
            assemble_s: 0.,
            cpu_s: 0.,
            calls: 0,
            max_pending_points: 0,
            max_origin_shift: 0.,
            first_time: None,
            last_time: 0.,
            last_points: Vec::new(),
            last_origin: [0.; 3],
        }
    }
    fn batch(&mut self, scans: &[Scan]) {
        let cpu = cpu_seconds();
        let stage = Instant::now();
        let origin = scans.last().unwrap().origin;
        let points: Vec<_> = scans
            .iter()
            .flat_map(|s| s.points.iter().copied())
            .collect();
        self.assemble_s += stage.elapsed().as_secs_f64();
        self.max_pending_points = self.max_pending_points.max(points.len());
        for s in scans {
            self.max_origin_shift = self.max_origin_shift.max(
                (0..3)
                    .map(|k| (s.origin[k] - origin[k]).powi(2))
                    .sum::<f32>()
                    .sqrt(),
            );
        }
        let stage = Instant::now();
        self.map.insert(&points, origin, true).unwrap();
        self.insert_s += stage.elapsed().as_secs_f64();
        let stage = Instant::now();
        let flags = self.map.seen_free(&points).unwrap();
        self.query_s += stage.elapsed().as_secs_f64();
        self.cpu_s += cpu_seconds() - cpu;
        let mut index = 0;
        for scan in scans {
            self.frames += 1;
            self.first_time.get_or_insert(scan.timestamp);
            self.last_time = scan.timestamp;
            for (&point, &group) in scan.points.iter().zip(&scan.groups) {
                let online = flags[index];
                index += 1;
                self.total[group] += 1;
                self.online[group] += u64::from(online);
                self.samples.push(Sample {
                    point,
                    group,
                    online,
                });
            }
        }
        self.calls += 1;
        self.last_points.clone_from(&scans.last().unwrap().points);
        self.last_origin = origin;
    }
    fn report(&self, scene: &str, name: &str, p: &Params, merge: usize, elapsed: f64) {
        let points: Vec<_> = self.samples.points.iter().map(|s| s.point).collect();
        let flags = self.map.seen_free(&points).unwrap();
        let (mut sampled, mut final_free, mut late_free) = ([0u64; 8], [0u64; 8], [0u64; 8]);
        for (sample, free) in self.samples.points.iter().zip(flags) {
            sampled[sample.group] += 1;
            final_free[sample.group] += u64::from(free);
            late_free[sample.group] += u64::from(free && !sample.online);
        }
        // 合成输入可确定性重建，按帧复算完整终态，不累计全量历史点。
        let mut final_exact = [0u64; 8];
        if scene != "recorded" {
            for frame in 0..90 {
                let scan = synthetic(frame, scene == "dense");
                for (group, free) in scan
                    .groups
                    .iter()
                    .zip(self.map.seen_free(&scan.points).unwrap())
                {
                    final_exact[*group] += u64::from(free);
                }
            }
        }
        let midpoints: Vec<_> = self
            .last_points
            .iter()
            .filter(|p| {
                (0..3)
                    .map(|k| (p[k] - self.last_origin[k]).powi(2))
                    .sum::<f32>()
                    > 2.25
            })
            .map(|p| std::array::from_fn(|k| (p[k] + self.last_origin[k]) * 0.5))
            .collect();
        let midpoint_free = self
            .map
            .seen_free(&midpoints)
            .unwrap()
            .into_iter()
            .filter(|f| *f)
            .count();
        let group_json = (0..8).filter(|g| scene != "recorded" || *g == 0).map(|g| format!("\"{}\":{{\"total\":{},\"online_free\":{},\"sampled\":{},\"final_free\":{},\"late_free\":{},\"final_exact\":{}}}", if scene == "recorded" { "unlabelled" } else { GROUPS[g] }, self.total[g], self.online[g], sampled[g], final_free[g], late_free[g], if scene == "recorded" { "null".to_owned() } else { final_exact[g].to_string() })).collect::<Vec<_>>().join(",");
        println!("{{\"scene\":\"{scene}\",\"case\":\"{name}\",\"resolution\":{},\"unknown\":{},\"compensation\":{},\"hits\":{},\"down_sampling\":\"{:?}\",\"max_range\":{},\"merge_frames\":{merge},\"frames\":{},\"points\":{},\"duration_s\":{},\"insert_calls\":{},\"insert_ms_per_frame\":{},\"query_ms_per_frame\":{},\"assemble_ms_per_frame\":{},\"cpu_ms_per_frame\":{},\"whole_run_s\":{elapsed},\"map_bytes\":{},\"map_nodes\":{},\"peak_rss_kib\":{},\"sample_cap\":{SAMPLE_CAP},\"samples\":{},\"sample_bytes\":{},\"max_pending_points\":{},\"merged_points_bytes\":{},\"max_origin_shift_m\":{},\"ray_midpoints\":{},\"ray_midpoints_free\":{},\"groups\":{{{group_json}}}}}",
            p.resolution, p.inflate_unknown, p.inflate_unknown_compensation, p.inflate_hits_dist, p.down_sampling, p.max_range,
            self.frames, self.total.iter().sum::<u64>(), self.last_time - self.first_time.unwrap_or(self.last_time), self.calls,
            self.insert_s * 1e3 / self.frames as f64, self.query_s * 1e3 / self.frames as f64, self.assemble_s * 1e3 / self.frames as f64,
            self.cpu_s * 1e3 / self.frames as f64, self.map.memory_bytes(), self.map.num_nodes(), peak_kib(), self.samples.points.len(),
            self.samples.points.capacity() * std::mem::size_of::<Sample>(), self.max_pending_points, self.max_pending_points * 12,
            self.max_origin_shift, midpoints.len(), midpoint_free);
    }
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert!(
        args.len() >= 3,
        "用法: online_params sparse|dense|recorded 参数名称 [实录路径] [帧数，0表示全部]"
    );
    let (scene, name) = (args[1].as_str(), args[2].as_str());
    let (p, merge) = config(name);
    let mut eval = Eval::new(&p);
    let started = Instant::now();
    let mut pending = Vec::with_capacity(merge);
    let mut process = |scan: Scan| {
        pending.push(scan);
        if pending.len() == merge {
            eval.batch(&pending);
            pending.clear();
        }
    };
    match scene {
        "sparse" | "dense" => {
            for frame in 0..90 {
                process(synthetic(frame, scene == "dense"));
            }
        }
        "recorded" => {
            let mut file = BufReader::new(File::open(&args[3]).unwrap());
            let limit: usize = args.get(4).map_or(0, |s| s.parse().unwrap());
            let mut frames = 0;
            while let Some(scan) = recorded(&mut file) {
                process(scan);
                frames += 1;
                if limit != 0 && frames >= limit {
                    break;
                }
            }
        }
        _ => panic!("未知场景"),
    }
    if !pending.is_empty() {
        eval.batch(&pending);
    }
    eval.report(scene, name, &p, merge, started.elapsed().as_secs_f64());
}
