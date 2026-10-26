use super::{boxes, Box3};
use std::{
    collections::HashSet,
    fs::File,
    io::{BufReader, Read},
};

pub const SENSOR: [f64; 3] = [0.28945, 0., -0.046825];

pub fn js(value: &str) -> String {
    format!(
        "\"{}\"",
        value
            .replace('\\', "\\\\")
            .replace('"', "\\\"")
            .replace('\n', "\\n")
    )
}
pub fn array<T: ToString>(values: &[T]) -> String {
    format!(
        "[{}]",
        values
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(",")
    )
}
pub fn object(values: &[(&str, String)]) -> String {
    format!(
        "{{{}}}",
        values
            .iter()
            .map(|(k, v)| format!("{}:{}", js(k), v))
            .collect::<Vec<_>>()
            .join(",")
    )
}
pub fn distribution(values: &[f64]) -> String {
    if values.is_empty() {
        return "null".into();
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let quantile = |q: f64| sorted[((sorted.len() - 1) as f64 * q).ceil() as usize];
    object(&[
        ("min", sorted[0].to_string()),
        (
            "mean",
            (values.iter().sum::<f64>() / values.len() as f64).to_string(),
        ),
        ("p50", quantile(0.5).to_string()),
        ("p95", quantile(0.95).to_string()),
        ("p99", quantile(0.99).to_string()),
        ("max", sorted.last().unwrap().to_string()),
    ])
}

#[derive(Clone, Copy)]
pub struct Pose {
    pub translation: [f64; 3],
    pub quaternion: [f64; 4],
}
pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    let t = [
        2. * (y * v[2] - z * v[1]),
        2. * (z * v[0] - x * v[2]),
        2. * (x * v[1] - y * v[0]),
    ];
    [
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    ]
}
impl Pose {
    pub fn world(&self, point: [f64; 3]) -> [f32; 3] {
        let r = rotate(self.quaternion, point);
        std::array::from_fn(|k| (r[k] + self.translation[k]) as f32)
    }
    pub fn angle(&self, other: &Self) -> f64 {
        // 与v2选择器一致：先得到完整旋转矩阵，再求 R前^T R今 的迹。
        let trace = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
            .iter()
            .map(|&axis| {
                let a = rotate(self.quaternion, axis);
                let b = rotate(other.quaternion, axis);
                a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>()
            })
            .sum::<f64>();
        ((trace - 1.) * 0.5).clamp(-1., 1.).acos().to_degrees()
    }
}
pub struct Scan {
    pub origin: [f32; 3],
    pub points: Vec<[f32; 3]>,
    pub groups: Vec<usize>,
    pub directions: Vec<[f32; 3]>,
    pub timestamp: f64,
    pub timestamp_ns: u64,
    pub pose: Pose,
    pub frame: usize,
}

struct RawScan {
    pose: Pose,
    timestamp_ns: u64,
    body: Vec<[f32; 3]>,
}
impl RawScan {
    fn read(reader: &mut BufReader<File>) -> Option<Self> {
        let mut header = [0u8; 68];
        if reader.read(&mut header[..1]).unwrap() == 0 {
            return None;
        }
        reader.read_exact(&mut header[1..]).expect("实录帧头截断");
        let timestamp_ns = u64::from_le_bytes(header[..8].try_into().unwrap());
        let p: [f64; 7] = std::array::from_fn(|k| {
            f64::from_le_bytes(header[8 + k * 8..16 + k * 8].try_into().unwrap())
        });
        assert!(p.iter().all(|v| v.is_finite()), "实录位姿含非有限数");
        let pose = Pose {
            translation: [p[0], p[1], p[2]],
            quaternion: [p[6], p[3], p[4], p[5]],
        };
        let count = u32::from_le_bytes(header[64..68].try_into().unwrap()) as usize;
        assert!(count < 1_000_000, "单帧点数不合理");
        let mut bytes = vec![0u8; count * 12];
        reader.read_exact(&mut bytes).expect("实录点数据截断");
        let body: Vec<[f32; 3]> = bytes
            .chunks_exact(12)
            .map(|p| {
                std::array::from_fn(|k| f32::from_le_bytes(p[k * 4..k * 4 + 4].try_into().unwrap()))
            })
            .collect();
        assert!(
            body.iter().flatten().all(|v| v.is_finite()),
            "实录帧含非有限点"
        );
        Some(Self {
            pose,
            timestamp_ns,
            body,
        })
    }
    fn directions(&self) -> Vec<[f32; 3]> {
        self.body
            .iter()
            .filter_map(|p| {
                let d: [f32; 3] = std::array::from_fn(|k| p[k] - SENSOR[k] as f32);
                let norm = d.iter().map(|x| x * x).sum::<f32>().sqrt();
                (norm.is_finite() && norm > 1e-6).then(|| d.map(|x| x / norm))
            })
            .collect()
    }
}

pub struct Input {
    scene: String,
    reader: Option<BufReader<File>>,
    frame: usize,
    limit: usize,
    first_timestamp: Option<u64>,
}
impl Input {
    pub fn new(scene: &str, path: Option<&str>, limit: usize, direction_start: usize) -> Self {
        let mut reader = matches!(scene, "recorded" | "l2")
            .then(|| BufReader::new(File::open(path.expect("此场景需要实录路径")).unwrap()));
        if scene == "l2" {
            for _ in 0..direction_start {
                RawScan::read(reader.as_mut().unwrap()).expect("方向片段起点超过文件末尾");
            }
        }
        assert!(
            matches!(scene, "sparse" | "dense" | "l2" | "recorded"),
            "未知场景: {scene}"
        );
        let limit = if scene == "recorded" {
            limit
        } else {
            if limit == 0 {
                90
            } else {
                limit.min(90)
            }
        };
        Self {
            scene: scene.into(),
            reader,
            frame: 0,
            limit,
            first_timestamp: None,
        }
    }
    pub fn next(&mut self) -> Option<Scan> {
        if self.limit != 0 && self.frame >= self.limit {
            return None;
        }
        let frame = self.frame;
        let scan = if self.scene == "recorded" {
            let raw = RawScan::read(self.reader.as_mut().unwrap())?;
            let directions = raw.directions();
            let points: Vec<_> = raw
                .body
                .iter()
                .map(|p| raw.pose.world(p.map(f64::from)))
                .collect();
            Scan {
                origin: raw.pose.world(SENSOR),
                groups: vec![0; points.len()],
                points,
                directions,
                timestamp: raw.timestamp_ns as f64 * 1e-9,
                timestamp_ns: raw.timestamp_ns,
                pose: raw.pose,
                frame,
            }
        } else {
            let (directions, timestamp_ns) = if self.scene == "l2" {
                let raw =
                    RawScan::read(self.reader.as_mut().unwrap()).expect("方向片段不足合成帧数");
                let first = *self.first_timestamp.get_or_insert(raw.timestamp_ns);
                (raw.directions(), raw.timestamp_ns.saturating_sub(first))
            } else {
                (
                    fixed_directions(self.scene == "dense"),
                    frame as u64 * 1_000_000_000 / 15,
                )
            };
            synthetic(frame, directions, timestamp_ns)
        };
        self.frame += 1;
        Some(scan)
    }
}
fn fixed_directions(dense: bool) -> Vec<[f32; 3]> {
    let (naz, nel) = if dense { (121, 61) } else { (64, 32) };
    let mut out = Vec::with_capacity(naz * nel);
    for az in 0..naz {
        for el in 0..nel {
            let a = (-45. + 90. * az as f32 / (naz - 1) as f32).to_radians();
            let e = (-30. + 60. * el as f32 / (nel - 1) as f32).to_radians();
            out.push([a.cos() * e.cos(), a.sin() * e.cos(), e.sin()]);
        }
    }
    out
}
fn synthetic(frame: usize, directions: Vec<[f32; 3]>, timestamp_ns: u64) -> Scan {
    let origin = [0.53 + 3. * frame as f32 / 89., 0.013, 0.53];
    let scene: Vec<Box3> = boxes(frame);
    let mut points = Vec::with_capacity(directions.len());
    let mut groups = Vec::with_capacity(directions.len());
    for &dir in &directions {
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
    let pose = Pose {
        translation: std::array::from_fn(|k| origin[k] as f64 - SENSOR[k]),
        quaternion: [1., 0., 0., 0.],
    };
    Scan {
        origin,
        points,
        groups,
        directions,
        timestamp: timestamp_ns as f64 * 1e-9,
        timestamp_ns,
        pose,
        frame,
    }
}

pub struct Selector {
    mode: String,
    previous: Option<(Pose, u64)>,
}
impl Selector {
    pub fn new(mode: &str) -> Self {
        assert!(
            matches!(mode, "all" | "v2" | "keyframe"),
            "未知抽帧方式: {mode}"
        );
        Self {
            mode: mode.into(),
            previous: None,
        }
    }
    pub fn selects(&mut self, scan: &Scan) -> bool {
        let selected = match self.previous {
            None => true,
            Some((previous, time)) => {
                let distance = previous
                    .translation
                    .iter()
                    .zip(scan.pose.translation)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt();
                self.mode == "all"
                    || distance >= if self.mode == "v2" { 0.1 } else { 0.5 }
                    || previous.angle(&scan.pose) >= if self.mode == "v2" { 3. } else { 10. }
                    || (self.mode == "v2"
                        && scan.timestamp_ns.saturating_sub(time) >= 1_000_000_000)
            }
        };
        if selected {
            self.previous = Some((scan.pose, scan.timestamp_ns));
        }
        selected
    }
}

pub struct Density {
    frames: Vec<String>,
    counts: Vec<f64>,
    rays010: Vec<f64>,
    rays015: Vec<f64>,
    az: [u64; 36],
    el: [u64; 18],
    pitches: Vec<f64>,
    directions_seen: u64,
    sample_state: u64,
    pitch_min: f64,
    pitch_max: f64,
}
impl Density {
    pub fn new() -> Self {
        Self {
            frames: vec![],
            counts: vec![],
            rays010: vec![],
            rays015: vec![],
            az: [0; 36],
            el: [0; 18],
            pitches: vec![],
            directions_seen: 0,
            sample_state: 0x1a2b3c4d5e6f7788,
            pitch_min: 90.,
            pitch_max: -90.,
        }
    }
    pub fn observe(&mut self, scan: &Scan) {
        let mut az = [0u64; 36];
        let mut el = [0u64; 18];
        let mut bins = HashSet::new();
        let mut pitch_min = 90f64;
        let mut pitch_max = -90f64;
        for &d in &scan.directions {
            let a = (d[1] as f64).atan2(d[0] as f64).to_degrees();
            let e = (d[2] as f64).clamp(-1., 1.).asin().to_degrees();
            let ai = (((a + 180.) / 10.).floor() as usize).min(35);
            let ei = (((e + 90.) / 10.).floor() as usize).min(17);
            az[ai] += 1;
            el[ei] += 1;
            bins.insert((ai, ei));
            pitch_min = pitch_min.min(e);
            pitch_max = pitch_max.max(e);
            // 固定种子的有界水塘抽样，避免固定步长与扫描图案混叠；直方图与范围统计全部方向。
            self.directions_seen += 1;
            self.sample_state ^= self.sample_state << 13;
            self.sample_state ^= self.sample_state >> 7;
            self.sample_state ^= self.sample_state << 17;
            if self.pitches.len() < 100_000 {
                self.pitches.push(e);
            } else {
                let index = (self.sample_state % self.directions_seen) as usize;
                if index < 100_000 {
                    self.pitches[index] = e;
                }
            }
        }
        self.pitch_min = self.pitch_min.min(pitch_min);
        self.pitch_max = self.pitch_max.max(pitch_max);
        let rays = |resolution: f64| {
            scan.points
                .iter()
                .filter(|p| {
                    p.iter()
                        .zip(scan.origin)
                        .map(|(a, b)| (a - b).powi(2))
                        .sum::<f32>()
                        >= 0.2f32 * 0.2f32
                })
                .map(|p| p.map(|v| (v as f64 / resolution).floor() as i64))
                .collect::<HashSet<_>>()
                .len()
        };
        let (r010, r015) = (rays(0.1), rays(0.15));
        self.counts.push(scan.points.len() as f64);
        self.rays010.push(r010 as f64);
        self.rays015.push(r015 as f64);
        for k in 0..36 {
            self.az[k] += az[k];
        }
        for k in 0..18 {
            self.el[k] += el[k];
        }
        self.frames.push(object(&[
            ("frame", scan.frame.to_string()),
            ("points", scan.points.len().to_string()),
            ("directions", scan.directions.len().to_string()),
            ("center010_rays", r010.to_string()),
            ("center015_rays", r015.to_string()),
            ("azimuth_10deg_hist", array(&az)),
            ("elevation_10deg_hist", array(&el)),
            ("occupied_10deg_bins", bins.len().to_string()),
            ("elevation_min_deg", pitch_min.to_string()),
            ("elevation_max_deg", pitch_max.to_string()),
        ]));
    }
    pub fn json(&self) -> String {
        object(&[
            ("point_count", distribution(&self.counts)),
            ("center010_rays", distribution(&self.rays010)),
            ("center015_rays", distribution(&self.rays015)),
            ("azimuth_10deg_hist", array(&self.az)),
            ("elevation_10deg_hist", array(&self.el)),
            ("elevation_sample_deg", distribution(&self.pitches)),
            ("elevation_sample_count", self.pitches.len().to_string()),
            ("direction_count", self.directions_seen.to_string()),
            ("elevation_min_deg", self.pitch_min.to_string()),
            ("elevation_max_deg", self.pitch_max.to_string()),
            ("frames", format!("[{}]", self.frames.join(","))),
        ])
    }
}
