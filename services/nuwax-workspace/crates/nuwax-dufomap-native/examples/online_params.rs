//! 稀疏扫描与滑动窗口参数评估；真实方向合成有逐点真值，实录只报告相对指标。
#[path = "online_params/saved.rs"]
mod saved;
#[path = "online_params/support.rs"]
mod support;
use nuwax_dufomap_native::{DownSampling, DufoMap, Params};
use std::{fs::File, io::Write, time::Instant};
use support::{array, distribution, js, object, Density, Input, Scan, Selector};

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
        "range10" => p.max_range = 10.,
        "range20" => p.max_range = 20.,
        "merge3" => merge = 3,
        _ if name.starts_with("window") => {
            p.sliding_window_size = name[6..].parse().expect("窗口参数名称应为windowN")
        }
        _ => panic!("未知参数名称:{name}"),
    }
    (p, merge)
}

struct Options {
    scene: String,
    name: String,
    path: Option<String>,
    limit: usize,
    direction_start: usize,
    mode: String,
    sampling: String,
    params: Params,
    merge: usize,
    metrics: Option<String>,
    final_mask: Option<String>,
}
impl Options {
    fn parse() -> Self {
        let args: Vec<_> = std::env::args().skip(1).collect();
        assert!(args.len()>=2,"用法: online_params sparse|dense|l2|recorded|density|saved 参数名 [实录路径] [帧数] [--mode online|offline --sampling all|v2|keyframe --window N --unknown N --hits 米 --resolution 米 --direction-start 帧 --frame-metrics 路径 --final-mask 路径]");
        let (params, merge) = config(&args[1]);
        let mut out = Self {
            scene: args[0].clone(),
            name: args[1].clone(),
            path: None,
            limit: 0,
            direction_start: 0,
            mode: "online".into(),
            sampling: "all".into(),
            params,
            merge,
            metrics: None,
            final_mask: None,
        };
        let mut i = 2;
        if args.get(i).is_some_and(|v| !v.starts_with("--")) {
            out.path = Some(args[i].clone());
            i += 1;
        }
        if args.get(i).is_some_and(|v| !v.starts_with("--")) {
            out.limit = args[i].parse().unwrap();
            i += 1;
        }
        while i < args.len() {
            let key = &args[i];
            let value = args.get(i + 1).expect("选项缺少值");
            match key.as_str() {
                "--mode" => out.mode = value.clone(),
                "--sampling" => out.sampling = value.clone(),
                "--window" => out.params.sliding_window_size = value.parse().unwrap(),
                "--unknown" => out.params.inflate_unknown = value.parse().unwrap(),
                "--hits" => out.params.inflate_hits_dist = value.parse().unwrap(),
                "--resolution" => out.params.resolution = value.parse().unwrap(),
                "--direction-start" => out.direction_start = value.parse().unwrap(),
                "--frames" => out.limit = value.parse().unwrap(),
                "--frame-metrics" => out.metrics = Some(value.clone()),
                "--final-mask" => out.final_mask = Some(value.clone()),
                "--compensation" => {
                    out.params.inflate_unknown_compensation = value.parse().unwrap()
                }
                _ => panic!("未知选项:{key}"),
            }
            i += 2;
        }
        assert!(
            matches!(out.mode.as_str(), "online" | "offline"),
            "未知模式"
        );
        Selector::new(&out.sampling);
        out
    }
    fn input(&self) -> Input {
        Input::new(
            &self.scene,
            self.path.as_deref(),
            self.limit,
            self.direction_start,
        )
    }
}

struct Latency {
    points: Vec<[f32; 3]>,
    groups: Vec<usize>,
    first: Vec<Option<usize>>,
    curve: Vec<String>,
}
impl Latency {
    fn new() -> Self {
        Self {
            points: vec![],
            groups: vec![],
            first: vec![],
            curve: vec![],
        }
    }
    fn collect(&mut self, scan: &Scan) {
        for (&point, &group) in scan.points.iter().zip(&scan.groups) {
            if group >= 6 {
                self.points.push(point);
                self.groups.push(group);
                self.first.push(None);
            }
        }
    }
    fn observe(&mut self, map: &DufoMap, frame: usize) {
        // 第44帧仍有人；此时已判空记延迟0。第45帧是离开后的第1帧。
        if frame < 44 {
            return;
        }
        let delay = frame - 44;
        for (first, free) in self
            .first
            .iter_mut()
            .zip(map.seen_free(&self.points).unwrap())
        {
            if free && first.is_none() {
                *first = Some(delay);
            }
        }
        let counts = |g| {
            self.first
                .iter()
                .zip(&self.groups)
                .filter(|(first, group)| **group == g && first.is_some())
                .count()
        };
        self.curve.push(object(&[
            ("source_frame", frame.to_string()),
            ("frames_after_departure", delay.to_string()),
            ("body_deleted", counts(6).to_string()),
            ("feet_deleted", counts(7).to_string()),
        ]));
    }
    fn json(&self) -> String {
        let mut values = vec![
            ("departure_source_frame", 45.to_string()),
            ("curve", format!("[{}]", self.curve.join(","))),
        ];
        for (g, name) in [(6, "body"), (7, "feet")] {
            let delays: Vec<_> = self
                .first
                .iter()
                .zip(&self.groups)
                .filter(|(_, group)| **group == g)
                .map(|(first, _)| *first)
                .collect();
            let deleted: Vec<_> = delays.iter().filter_map(|v| v.map(|x| x as f64)).collect();
            values.push((
                name,
                object(&[
                    ("total", delays.len().to_string()),
                    ("deleted", deleted.len().to_string()),
                    ("not_deleted", (delays.len() - deleted.len()).to_string()),
                    ("delay_frames_deleted_only", distribution(&deleted)),
                    (
                        "already_deleted_at_departure",
                        deleted.iter().filter(|&&v| v == 0.).count().to_string(),
                    ),
                ]),
            ));
        }
        object(&values)
    }
}

struct FrameMetric {
    frame: usize,
    selected: bool,
    points: usize,
    insert_ms: f64,
    query_ms: f64,
    cpu_ms: f64,
    tree: usize,
    window: usize,
}
struct Eval {
    map: DufoMap,
    samples: Reservoir,
    midpoints: Reservoir,
    total: [u64; 8],
    online: [u64; 8],
    inserted: [u64; 8],
    frames: usize,
    selected: usize,
    calls: usize,
    first_time: Option<f64>,
    last_time: f64,
    insert_s: f64,
    query_s: f64,
    assemble_s: f64,
    cpu_s: f64,
    latency_s: f64,
    latency_cpu_s: f64,
    final_propagate_s: f64,
    final_propagate_cpu_s: f64,
    peak_tree: usize,
    peak_window: usize,
    max_pending: usize,
    max_origin_shift: f32,
    last_points: Vec<[f32; 3]>,
    last_origin: [f32; 3],
    metrics: Vec<FrameMetric>,
    latency: Latency,
}
impl Eval {
    fn new(p: &Params) -> Self {
        Self {
            map: DufoMap::new(p).unwrap(),
            samples: Reservoir::new(),
            midpoints: Reservoir::new(),
            total: [0; 8],
            online: [0; 8],
            inserted: [0; 8],
            frames: 0,
            selected: 0,
            calls: 0,
            first_time: None,
            last_time: 0.,
            insert_s: 0.,
            query_s: 0.,
            assemble_s: 0.,
            cpu_s: 0.,
            latency_s: 0.,
            latency_cpu_s: 0.,
            final_propagate_s: 0.,
            final_propagate_cpu_s: 0.,
            peak_tree: 0,
            peak_window: 0,
            max_pending: 0,
            max_origin_shift: 0.,
            last_points: vec![],
            last_origin: [0.; 3],
            metrics: vec![],
            latency: Latency::new(),
        }
    }
    fn batch(&mut self, scans: &[(Scan, bool)], online: bool, labelled: bool) {
        let cpu = cpu_seconds();
        let assembly = Instant::now();
        let selected: Vec<_> = scans
            .iter()
            .filter(|(_, selected)| *selected)
            .map(|(scan, _)| scan)
            .collect();
        let origin = selected.last().map(|s| s.origin);
        let points: Vec<_> = selected
            .iter()
            .flat_map(|s| s.points.iter().copied())
            .collect();
        self.assemble_s += assembly.elapsed().as_secs_f64();
        let assembly_cpu = cpu_seconds() - cpu;
        self.max_pending = self.max_pending.max(points.len());
        let mut insert = 0.;
        let mut insert_cpu = 0.;
        if let Some(origin) = origin {
            for scan in &selected {
                self.selected += 1;
                for &g in &scan.groups {
                    self.inserted[g] += 1;
                }
                self.max_origin_shift = self.max_origin_shift.max(
                    scan.origin
                        .iter()
                        .zip(origin)
                        .map(|(a, b)| (a - b).powi(2))
                        .sum::<f32>()
                        .sqrt(),
                );
            }
            let t = Instant::now();
            let c = cpu_seconds();
            self.map.insert(&points, origin, online).unwrap();
            insert_cpu = cpu_seconds() - c;
            insert = t.elapsed().as_secs_f64();
            self.insert_s += insert;
            self.calls += 1;
        }
        let mut batch_query = 0.;
        let mut batch_query_cpu = 0.;
        let mut frame_query = Vec::new();
        for (scan, _) in scans {
            self.frames += 1;
            self.first_time.get_or_insert(scan.timestamp);
            self.last_time = scan.timestamp;
            let t = Instant::now();
            let c = cpu_seconds();
            let flags = if online {
                self.map.seen_free(&scan.points).unwrap()
            } else {
                vec![false; scan.points.len()]
            };
            let query = if online {
                t.elapsed().as_secs_f64()
            } else {
                0.
            };
            let query_cpu = if online { cpu_seconds() - c } else { 0. };
            batch_query += query;
            batch_query_cpu += query_cpu;
            frame_query.push((query, query_cpu));
            for ((&point, &group), free) in scan.points.iter().zip(&scan.groups).zip(flags) {
                self.total[group] += 1;
                self.online[group] += u64::from(free);
                self.samples.push(Sample {
                    point,
                    group,
                    online: free,
                });
                if point
                    .iter()
                    .zip(scan.origin)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f32>()
                    > 2.25
                {
                    self.midpoints.push(Sample {
                        point: std::array::from_fn(|k| (point[k] + scan.origin[k]) * 0.5),
                        group: 0,
                        online: false,
                    });
                }
            }
            if labelled && online {
                self.latency.collect(scan);
            }
            self.last_points.clone_from(&scan.points);
            self.last_origin = scan.origin;
        }
        self.query_s += batch_query;
        // CPU仅计拼接、插入和当前点查询；采样与真值记账开销由whole_run_s体现。
        let cpu = assembly_cpu + insert_cpu + batch_query_cpu;
        self.cpu_s += cpu;
        self.peak_tree = self.peak_tree.max(self.map.memory_bytes());
        self.peak_window = self.peak_window.max(self.map.window_memory_bytes());
        if online && labelled {
            let t = Instant::now();
            let c = cpu_seconds();
            self.latency
                .observe(&self.map, scans.last().unwrap().0.frame);
            self.latency_s += t.elapsed().as_secs_f64();
            self.latency_cpu_s += cpu_seconds() - c;
        }
        for ((scan, selected), (query, query_cpu)) in scans.iter().zip(frame_query) {
            self.metrics.push(FrameMetric {
                frame: scan.frame,
                selected: *selected,
                points: scan.points.len(),
                insert_ms: insert * 1e3 / scans.len() as f64,
                query_ms: query * 1e3,
                cpu_ms: (assembly_cpu + insert_cpu) * 1e3 / scans.len() as f64 + query_cpu * 1e3,
                tree: self.map.memory_bytes(),
                window: self.map.window_memory_bytes(),
            });
        }
    }
    fn report(&self, o: &Options, started: Instant) {
        let final_started = Instant::now();
        let final_cpu = cpu_seconds();
        let mut final_exact = [0u64; 8];
        let mut final_total = [0u64; 8];
        // 重新逐帧读入完整输入。抽帧仅改变插入，不改变最终查询分母和真值。
        let mut input = o.input();
        let mut mask_file = o
            .final_mask
            .as_ref()
            .map(|path| File::create(path).unwrap());
        while let Some(scan) = input.next() {
            let flags = self.map.seen_free(&scan.points).unwrap();
            if let Some(file) = &mut mask_file {
                // 按完整源帧/源点顺序输出0/1字节，供连续力度的逐点掩码嵌套核对。
                file.write_all(&flags.iter().map(|&free| u8::from(free)).collect::<Vec<_>>())
                    .unwrap();
            }
            for (&group, free) in scan.groups.iter().zip(flags) {
                final_total[group] += 1;
                final_exact[group] += u64::from(free);
            }
        }
        assert_eq!(self.total, final_total, "终态查询没有覆盖相同输入全集");
        let points: Vec<_> = self.samples.points.iter().map(|s| s.point).collect();
        let flags = self.map.seen_free(&points).unwrap();
        let (mut sampled, mut sample_free, mut late_free) = ([0u64; 8], [0u64; 8], [0u64; 8]);
        for (sample, free) in self.samples.points.iter().zip(flags) {
            sampled[sample.group] += 1;
            sample_free[sample.group] += u64::from(free);
            late_free[sample.group] += u64::from(free && !sample.online);
        }
        let midpoints: Vec<_> = self
            .last_points
            .iter()
            .filter(|p| {
                p.iter()
                    .zip(self.last_origin)
                    .map(|(a, b)| (a - b).powi(2))
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
            .filter(|v| *v)
            .count();
        let sample_midpoints: Vec<_> = self.midpoints.points.iter().map(|s| s.point).collect();
        let sample_midpoint_free = self
            .map
            .seen_free(&sample_midpoints)
            .unwrap()
            .into_iter()
            .filter(|v| *v)
            .count();
        let final_s = final_started.elapsed().as_secs_f64();
        let final_cpu_s = cpu_seconds() - final_cpu;
        let groups: Vec<_> = (0..8)
            .filter(|g| o.scene != "recorded" || *g == 0)
            .map(|g| {
                let name = if o.scene == "recorded" {
                    "unlabelled"
                } else {
                    GROUPS[g]
                };
                (
                    name,
                    object(&[
                        ("total", self.total[g].to_string()),
                        ("inserted_total", self.inserted[g].to_string()),
                        (
                            "online_free",
                            if o.mode == "online" {
                                self.online[g].to_string()
                            } else {
                                "null".into()
                            },
                        ),
                        ("sampled", sampled[g].to_string()),
                        ("final_free", sample_free[g].to_string()),
                        (
                            "late_free",
                            if o.mode == "online" {
                                late_free[g].to_string()
                            } else {
                                "null".into()
                            },
                        ),
                        ("final_exact", final_exact[g].to_string()),
                    ]),
                )
            })
            .collect();
        let metric = |f: fn(&FrameMetric) -> f64| {
            distribution(&self.metrics.iter().map(f).collect::<Vec<_>>())
        };
        let p = &o.params;
        let mut values = vec![
            ("scene", js(&o.scene)),
            ("case", js(&o.name)),
            ("mode", js(&o.mode)),
            ("sampling", js(&o.sampling)),
            ("resolution", p.resolution.to_string()),
            ("unknown", p.inflate_unknown.to_string()),
            ("compensation", p.inflate_unknown_compensation.to_string()),
            ("hits", p.inflate_hits_dist.to_string()),
            ("window_frames", p.sliding_window_size.to_string()),
            ("down_sampling", js(&format!("{:?}", p.down_sampling))),
            ("max_range", p.max_range.to_string()),
            ("merge_frames", o.merge.to_string()),
            ("direction_start", o.direction_start.to_string()),
            ("sensor_origin_body", array(&support::SENSOR)),
            ("frames", self.frames.to_string()),
            ("selected_frames", self.selected.to_string()),
            ("points", self.total.iter().sum::<u64>().to_string()),
            (
                "duration_s",
                (self.last_time - self.first_time.unwrap_or(self.last_time)).to_string(),
            ),
            ("insert_calls", self.calls.to_string()),
            (
                "insert_ms_per_frame",
                (self.insert_s * 1e3 / self.frames as f64).to_string(),
            ),
            (
                "insert_ms_per_selected_frame",
                (self.insert_s * 1e3 / self.selected.max(1) as f64).to_string(),
            ),
            (
                "query_ms_per_frame",
                (self.query_s * 1e3 / self.frames as f64).to_string(),
            ),
            (
                "assemble_ms_per_frame",
                (self.assemble_s * 1e3 / self.frames as f64).to_string(),
            ),
            (
                "cpu_ms_per_frame",
                (self.cpu_s * 1e3 / self.frames as f64).to_string(),
            ),
            (
                "insert_ms_distribution_source_frames",
                metric(|m| m.insert_ms),
            ),
            (
                "query_ms_distribution_source_frames",
                metric(|m| m.query_ms),
            ),
            ("cpu_ms_distribution_source_frames", metric(|m| m.cpu_ms)),
            ("latency_query_s", self.latency_s.to_string()),
            ("latency_query_cpu_s", self.latency_cpu_s.to_string()),
            ("final_propagate_s", self.final_propagate_s.to_string()),
            (
                "final_propagate_cpu_s",
                self.final_propagate_cpu_s.to_string(),
            ),
            (
                "native_wall_s",
                (self.assemble_s + self.insert_s + self.query_s + self.final_propagate_s)
                    .to_string(),
            ),
            (
                "native_cpu_s",
                (self.cpu_s + self.final_propagate_cpu_s).to_string(),
            ),
            ("final_exact_replay_query_s", final_s.to_string()),
            ("final_exact_replay_query_cpu_s", final_cpu_s.to_string()),
            ("whole_run_s", started.elapsed().as_secs_f64().to_string()),
            ("map_bytes", self.map.memory_bytes().to_string()),
            ("map_peak_bytes", self.peak_tree.to_string()),
            ("window_bytes", self.map.window_memory_bytes().to_string()),
            ("window_peak_bytes", self.peak_window.to_string()),
            ("map_nodes", self.map.num_nodes().to_string()),
            ("peak_rss_kib", peak_kib().to_string()),
            ("sample_cap", SAMPLE_CAP.to_string()),
            ("samples", self.samples.points.len().to_string()),
            (
                "sample_bytes",
                (self.samples.points.capacity() * std::mem::size_of::<Sample>()).to_string(),
            ),
            ("max_pending_points", self.max_pending.to_string()),
            ("merged_points_bytes", (self.max_pending * 12).to_string()),
            ("max_origin_shift_m", self.max_origin_shift.to_string()),
            ("ray_midpoints", midpoints.len().to_string()),
            ("ray_midpoints_free", midpoint_free.to_string()),
            (
                "ray_midpoints_all_source_total",
                self.midpoints.count.to_string(),
            ),
            ("ray_midpoints_sampled", sample_midpoints.len().to_string()),
            (
                "ray_midpoints_sampled_free",
                sample_midpoint_free.to_string(),
            ),
            ("groups", object(&groups)),
            (
                "latency",
                if o.mode == "online" && o.scene != "recorded" {
                    self.latency.json()
                } else {
                    "null".into()
                },
            ),
        ];
        if o.scene != "recorded" {
            values.push((
                "static_total",
                self.total[..6].iter().sum::<u64>().to_string(),
            ));
            values.push((
                "static_online_false_free",
                if o.mode == "online" {
                    self.online[..6].iter().sum::<u64>().to_string()
                } else {
                    "null".into()
                },
            ));
            values.push((
                "static_final_false_free",
                final_exact[..6].iter().sum::<u64>().to_string(),
            ));
        }
        println!("{}", object(&values));
        if let Some(path) = &o.metrics {
            let mut file = File::create(path).unwrap();
            writeln!(
                file,
                "source_frame,selected,points,insert_ms,query_ms,cpu_ms,tree_bytes,window_bytes"
            )
            .unwrap();
            for m in &self.metrics {
                writeln!(
                    file,
                    "{},{},{},{},{},{},{},{}",
                    m.frame,
                    m.selected,
                    m.points,
                    m.insert_ms,
                    m.query_ms,
                    m.cpu_ms,
                    m.tree,
                    m.window
                )
                .unwrap();
            }
        }
    }
}

fn main() {
    let o = Options::parse();
    if o.scene == "saved" {
        saved::evaluate(&o);
        return;
    }
    let started = Instant::now();
    if o.scene == "density" {
        let mut values = Vec::new();
        for scene in ["sparse", "dense", "l2", "recorded"] {
            let mut input = Input::new(scene, o.path.as_deref(), o.limit, o.direction_start);
            let mut density = Density::new();
            while let Some(scan) = input.next() {
                density.observe(&scan);
            }
            values.push((scene, density.json()));
        }
        println!(
            "{}",
            object(&[
                ("analysis", js("input_density")),
                ("sensor_origin_body", array(&support::SENSOR)),
                ("min_range_m", "0.2".into()),
                ("bin_width_deg", "10".into()),
                ("sources", object(&values)),
                ("whole_run_s", started.elapsed().as_secs_f64().to_string())
            ])
        );
        return;
    }
    let mut eval = Eval::new(&o.params);
    let mut input = o.input();
    let mut selector = Selector::new(&o.sampling);
    let mut pending = Vec::new();
    let mut selected_pending = 0;
    while let Some(scan) = input.next() {
        let selected = selector.selects(&scan);
        selected_pending += usize::from(selected);
        pending.push((scan, selected));
        if o.merge == 1 || selected_pending >= o.merge {
            eval.batch(&pending, o.mode == "online", o.scene != "recorded");
            pending.clear();
            selected_pending = 0;
        }
    }
    if !pending.is_empty() {
        eval.batch(&pending, o.mode == "online", o.scene != "recorded");
    }
    assert!(eval.frames > 0, "输入没有帧");
    if o.mode == "offline" {
        let t = Instant::now();
        let c = cpu_seconds();
        eval.map.propagate();
        eval.final_propagate_s = t.elapsed().as_secs_f64();
        eval.final_propagate_cpu_s = cpu_seconds() - c;
        eval.peak_tree = eval.peak_tree.max(eval.map.memory_bytes());
    }
    eval.report(&o, started);
}
