//! G2 重建去动态策略比较：同一源点人口、冻结里程计终态和逐时校正映射。
#[path = "online_params/support.rs"]
#[allow(dead_code)]
mod support;
use nuwax_dufomap_native::{DufoMap, Params};
use std::{fs::File, io::Write, time::Instant};
use support::{array, js, object, rotate, Input, Pose, Scan, Selector, SENSOR};

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

// 与 G3 最终在线生产参数相同；评估工具不覆盖生产配置。
fn online_evidence_params() -> Params {
    Params {
        resolution: 0.15,
        inflate_unknown: 2,
        inflate_unknown_compensation: false,
        inflate_hits_dist: 0.30,
        sliding_window_size: 30,
        ..Params::default()
    }
}

// 与完成清理默认力度 50 的单精度计算相同。
fn corrected_offline_params() -> Params {
    Params {
        resolution: 0.10,
        inflate_unknown: 2,
        inflate_unknown_compensation: false,
        inflate_hits_dist: 0.40 - 0.001 * f32::from(50u8),
        sliding_window_size: 5,
        ..Params::default()
    }
}

fn params_json(p: &Params) -> String {
    object(&[
        ("resolution_m", p.resolution.to_string()),
        ("unknown", p.inflate_unknown.to_string()),
        ("window", p.sliding_window_size.to_string()),
        ("protect_m", p.inflate_hits_dist.to_string()),
        (
            "unknown_compensation",
            p.inflate_unknown_compensation.to_string(),
        ),
        ("downsampling", js("Center")),
        ("max_range_m", p.max_range.to_string()),
    ])
}

fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn correction(scan: &Scan, mode: &str) -> Pose {
    if mode == "identity" {
        return Pose {
            translation: [0.; 3],
            quaternion: [1., 0., 0., 0.],
        };
    }
    // 对源时间连续变化的刚体校正，包含滚转、俯仰、偏航和三个平移分量。
    let t = scan.timestamp / (89. / 15.);
    let [roll, pitch, yaw] = [0.04 + 0.02 * t, -0.06 + 0.01 * t.sin(), 0.20 + 0.05 * t];
    let axis = |angle: f64, index: usize| {
        let mut q = [0.; 4];
        q[0] = (angle / 2.).cos();
        q[index] = (angle / 2.).sin();
        q
    };
    Pose {
        translation: [2. + 0.08 * t, -1. + 0.03 * t.sin(), 0.4 + 0.02 * t],
        quaternion: multiply(multiply(axis(yaw, 3), axis(pitch, 2)), axis(roll, 1)),
    }
}

fn world64(pose: Pose, point: [f64; 3]) -> [f64; 3] {
    let rotated = rotate(pose.quaternion, point);
    std::array::from_fn(|k| rotated[k] + pose.translation[k])
}

fn mapped_points(scan: &Scan, c: Pose) -> (Vec<[f32; 3]>, Vec<[f32; 3]>) {
    let [w, x, y, z] = scan.pose.quaternion;
    let inverse_rotation = [w, -x, -y, -z];
    let mut frozen = Vec::with_capacity(scan.points.len());
    let mut corrected = Vec::with_capacity(scan.points.len());
    for &point in &scan.points {
        // 先还原同一机体原点，再分别走冻结 odom_from_body 与校正后的完整变换。
        let body = rotate(
            inverse_rotation,
            std::array::from_fn(|k| f64::from(point[k]) - scan.pose.translation[k]),
        );
        let odom = world64(scan.pose, body);
        let original = odom.map(|v| v as f32);
        assert_eq!(original, point, "冻结坐标往返改变源点身份");
        frozen.push(original);
        corrected.push(c.world(odom));
    }
    (frozen, corrected)
}

#[derive(Default)]
struct Population {
    total: [u64; 8],
    online_current: [u64; 8],
    offline_a: [u64; 8],
    online_final_b: [u64; 8],
    union: [u64; 8],
    a_lost_b: [u64; 8],
    a_added_to_b: [u64; 8],
    wrong_coordinate_b: [u64; 8],
    wrong_coordinate_mismatches: [u64; 8],
}
impl Population {
    fn current(&mut self, groups: &[usize], flags: &[bool]) {
        for (&g, &flag) in groups.iter().zip(flags) {
            self.total[g] += 1;
            self.online_current[g] += u64::from(flag);
        }
    }
    fn final_flags(&mut self, groups: &[usize], a: &[bool], b: &[bool], wrong_b: &[bool]) {
        for (((&g, &a), &b), &wrong_b) in groups.iter().zip(a).zip(b).zip(wrong_b) {
            self.offline_a[g] += u64::from(a);
            self.online_final_b[g] += u64::from(b);
            self.union[g] += u64::from(a || b);
            self.a_lost_b[g] += u64::from(b && !a);
            self.a_added_to_b[g] += u64::from(a && !b);
            self.wrong_coordinate_b[g] += u64::from(wrong_b);
            self.wrong_coordinate_mismatches[g] += u64::from(b != wrong_b);
        }
    }
    fn json(&self) -> String {
        let groups: Vec<_> = GROUPS
            .iter()
            .enumerate()
            .map(|(g, &name)| {
                (
                    name,
                    object(&[
                        ("total", self.total[g].to_string()),
                        ("online_current", self.online_current[g].to_string()),
                        ("a", self.offline_a[g].to_string()),
                        ("b", self.online_final_b[g].to_string()),
                        ("a_union_b", self.union[g].to_string()),
                        ("b_deleted_but_a_retained", self.a_lost_b[g].to_string()),
                        ("a_deleted_but_b_retained", self.a_added_to_b[g].to_string()),
                        ("wrong_coordinate_b", self.wrong_coordinate_b[g].to_string()),
                        (
                            "wrong_coordinate_mismatches",
                            self.wrong_coordinate_mismatches[g].to_string(),
                        ),
                    ]),
                )
            })
            .collect();
        let sum =
            |values: &[u64; 8], range: std::ops::Range<usize>| values[range].iter().sum::<u64>();
        object(&[
            ("groups", object(&groups)),
            ("static_total", sum(&self.total, 0..6).to_string()),
            (
                "static_online_current_false_free",
                sum(&self.online_current, 0..6).to_string(),
            ),
            (
                "static_a_false_free",
                sum(&self.offline_a, 0..6).to_string(),
            ),
            (
                "static_b_false_free",
                sum(&self.online_final_b, 0..6).to_string(),
            ),
            (
                "static_union_false_free",
                sum(&self.union, 0..6).to_string(),
            ),
            (
                "a_additional_static_false_free",
                sum(&self.a_added_to_b, 0..6).to_string(),
            ),
            ("a_lost_b_dynamic", sum(&self.a_lost_b, 6..8).to_string()),
            (
                "union_additional_dynamic",
                sum(&self.a_added_to_b, 6..8).to_string(),
            ),
            (
                "a_preserves_b_pointwise",
                self.a_lost_b[6..].iter().all(|&n| n == 0).to_string(),
            ),
            ("b_preserves_online_final_pointwise", "true".into()),
            ("union_preserves_b_pointwise", "true".into()),
            (
                "union_adds_no_static_false_free",
                self.a_added_to_b[..6].iter().all(|&n| n == 0).to_string(),
            ),
        ])
    }
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

fn main() {
    let args: Vec<_> = std::env::args().skip(1).collect();
    assert!(
        args.len() >= 2,
        "用法: g2_rebuild_params sparse|dense|l2 identity|time-varying [实录路径] [输出JSON路径]"
    );
    let scene = &args[0];
    let mode = &args[1];
    assert!(matches!(scene.as_str(), "sparse" | "dense" | "l2"));
    assert!(matches!(mode.as_str(), "identity" | "time-varying"));
    let path = args
        .get(2)
        .filter(|p| p.as_str() != "-")
        .map(String::as_str);
    let input = || Input::new(scene, path, 90, 0);
    let started = Instant::now();
    let online_params = online_evidence_params();
    let offline_params = corrected_offline_params();
    let mut online = DufoMap::new(&online_params).unwrap();
    let mut offline = DufoMap::new(&offline_params).unwrap();
    let mut all = Population::default();
    let mut selected = Population::default();
    let mut selector = Selector::new("v2");
    let mut stream = input();
    let mut selected_frames = Vec::new();
    let mut source_frames = 0;
    let mut transforms = Vec::new();
    let mut online_wall_s = 0.;
    let mut offline_wall_s = 0.;
    while let Some(scan) = stream.next() {
        source_frames += 1;
        let c = correction(&scan, mode);
        let (frozen, corrected) = mapped_points(&scan, c);
        let t = Instant::now();
        online
            .insert(&frozen, scan.pose.world(SENSOR), true)
            .unwrap();
        let current = online.seen_free(&frozen).unwrap();
        online_wall_s += t.elapsed().as_secs_f64();
        all.current(&scan.groups, &current);
        if selector.selects(&scan) {
            selected_frames.push(scan.frame);
            selected.current(&scan.groups, &current);
            let origin = c.world(world64(scan.pose, SENSOR));
            let t = Instant::now();
            offline.insert(&corrected, origin, false).unwrap();
            offline_wall_s += t.elapsed().as_secs_f64();
            transforms.push(object(&[
                ("source_frame", scan.frame.to_string()),
                ("timestamp_s", scan.timestamp.to_string()),
                ("map_from_odom_translation", array(&c.translation)),
                ("map_from_odom_quaternion_wxyz", array(&c.quaternion)),
                ("corrected_sensor_origin", array(&origin)),
            ]));
        }
    }
    let t = Instant::now();
    offline.propagate();
    offline_wall_s += t.elapsed().as_secs_f64();
    let t = Instant::now();
    let snapshot = online.clone();
    let snapshot_clone_s = t.elapsed().as_secs_f64();
    let mut stream = input();
    let mut selector = Selector::new("v2");
    let mut second_total = [0u64; 8];
    let mut second_selected = [0u64; 8];
    let mut snapshot_points_checked = 0;
    let mut sentinel = None;
    while let Some(scan) = stream.next() {
        let c = correction(&scan, mode);
        let (frozen, corrected) = mapped_points(&scan, c);
        let a = offline.seen_free(&corrected).unwrap();
        let b = snapshot.seen_free(&frozen).unwrap();
        assert_eq!(b, online.seen_free(&frozen).unwrap(), "克隆终态不一致");
        snapshot_points_checked += frozen.len();
        let wrong_b = snapshot.seen_free(&corrected).unwrap();
        all.final_flags(&scan.groups, &a, &b, &wrong_b);
        let is_selected = selector.selects(&scan);
        for &g in &scan.groups {
            second_total[g] += 1;
            second_selected[g] += u64::from(is_selected);
        }
        if is_selected {
            selected.final_flags(&scan.groups, &a, &b, &wrong_b);
        }
        if b.iter().any(|&v| v) {
            sentinel = Some((frozen, b));
        }
    }
    assert_eq!(second_total, all.total, "第二次输入的全源真值人口改变");
    assert_eq!(
        second_selected, selected.total,
        "第二次输入的选帧真值人口改变"
    );
    assert!(all
        .online_current
        .iter()
        .zip(all.online_final_b)
        .all(|(&a, b)| a <= b));
    let original_tree_bytes = online.memory_bytes();
    let snapshot_tree_bytes = snapshot.memory_bytes();
    let snapshot_window_bytes = snapshot.window_memory_bytes();
    online.clear();
    let (sentinel_points, sentinel_flags) = sentinel.expect("合成终态未形成空闲证据");
    assert_eq!(
        snapshot.seen_free(&sentinel_points).unwrap(),
        sentinel_flags,
        "原树清空影响冻结快照"
    );
    assert!(online
        .seen_free(&sentinel_points)
        .unwrap()
        .iter()
        .all(|&v| !v));
    let result = object(&[
        ("scene", js(scene)),
        ("correction", js(mode)),
        ("source_frames", source_frames.to_string()),
        ("selected_frames", array(&selected_frames)),
        ("online_params", params_json(&online_params)),
        ("offline_params", params_json(&offline_params)),
        ("all_source_population", all.json()),
        ("v2_selected_source_population", selected.json()),
        (
            "missing_source_points",
            array(&std::array::from_fn::<_, 8, _>(|g| {
                all.total[g] - selected.total[g]
            })),
        ),
        (
            "online_insert_and_current_query_s",
            online_wall_s.to_string(),
        ),
        ("offline_insert_and_propagate_s", offline_wall_s.to_string()),
        ("snapshot_clone_s", snapshot_clone_s.to_string()),
        (
            "snapshot_points_checked",
            snapshot_points_checked.to_string(),
        ),
        ("snapshot_survived_original_clear", "true".into()),
        ("online_tree_bytes", original_tree_bytes.to_string()),
        ("snapshot_tree_bytes", snapshot_tree_bytes.to_string()),
        (
            "snapshot_window_bytes_estimate",
            snapshot_window_bytes.to_string(),
        ),
        ("offline_tree_bytes", offline.memory_bytes().to_string()),
        (
            "offline_window_bytes_estimate",
            offline.window_memory_bytes().to_string(),
        ),
        ("peak_rss_kib", peak_kib().to_string()),
        ("whole_run_s", started.elapsed().as_secs_f64().to_string()),
        (
            "selected_correction_transforms",
            format!("[{}]", transforms.join(",")),
        ),
    ]);
    println!("{result}");
    if let Some(path) = args.get(3) {
        writeln!(File::create(path).unwrap(), "{result}").unwrap();
    }
}
