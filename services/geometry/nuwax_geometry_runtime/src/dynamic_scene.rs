//! Pointwise dynamic evidence from real measured rays. No shape-only deletion.
use crate::human_point_labels::{FrameLabels, PointLabel, StaticReferences};
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::Point3;
use std::collections::VecDeque;
pub(crate) type Cell = (i32, i32, i32);
pub(crate) const CELL_SIZE: f64 = 0.1;
const RANGE: f64 = 30.;
const SURFACE_MARGIN: f64 = 0.2;
const FREE_OBSERVATIONS: u8 = 2;
const EVIDENCE_GAP_NS: u64 = 2_000_000_000;
const RECOVERY_NS: u64 = 2_000_000_000;
const RECOVERY_OBSERVATIONS: u16 = 20;
const MAX_CELLS: usize = 500_000;
#[derive(Clone, Default)]
struct Evidence {
    free_count: u8,
    last_free: u64,
    occupied_count: u16,
    occupied_since: u64,
    last_occupied: u64,
    known_free: bool,
    first_hit: u64,
    total_hits: u16,
    first_clear: u64,
    last_hit_frame: u64,
    void_since: u64,
}
#[derive(Clone)]
pub(crate) struct DynamicScene {
    pub(crate) offline: bool,
    minimum_range_m: f64,
    evidence: HashMap<Cell, Evidence>,
    last_timestamp_ns: u64,
    clear_cells: HashSet<Cell>,
    dynamic_cells: HashSet<Cell>,
    frame: u64,
    window: VecDeque<(u64, HashSet<Cell>, HashSet<Cell>)>,
    references: StaticReferences,
    labels: FrameLabels,
    point_labels: HashMap<[u64; 3], PointLabel>,
    tracks: crate::dynamic_tracks::Tracks,
}
/// One immutable decision for one admitted scan, in frozen odometry coordinates.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub(crate) struct FrameDecision {
    pub labels: FrameLabels,
    clear_cells: HashSet<Cell>,
    references: StaticReferences,
}
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MappedFrameDecision {
    pub decision: std::sync::Arc<FrameDecision>,
    odom_from_map: nalgebra::Isometry3<f64>,
    point_labels: HashMap<[u64; 3], PointLabel>,
}
impl MappedFrameDecision {
    pub(crate) fn new(
        decision: std::sync::Arc<FrameDecision>,
        map_from_odom: nalgebra::Isometry3<f64>,
        points: &[Point3<f64>],
        first: usize,
    ) -> Result<Self, String> {
        let labels = decision
            .labels
            .labels
            .get(first..first + points.len())
            .ok_or("共享人体标签与原扫描点序不一致")?;
        Ok(Self {
            decision: decision.clone(),
            odom_from_map: map_from_odom.inverse(),
            point_labels: points
                .iter()
                .zip(labels)
                .map(|(p, &l)| (bits([p.x, p.y, p.z]), l))
                .collect(),
        })
    }
    pub(crate) fn encode(&self) -> Result<Vec<u8>, String> {
        let pose: [f64; 16] = self
            .odom_from_map
            .to_homogeneous()
            .as_slice()
            .try_into()
            .unwrap();
        let labels = self
            .point_labels
            .iter()
            .map(|(&k, &v)| (k, v))
            .collect::<Vec<_>>();
        serde_json::to_vec(&(&*self.decision, pose, labels)).map_err(|e| e.to_string())
    }
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, String> {
        let (decision, m, labels): (FrameDecision, [f64; 16], Vec<([u64; 3], PointLabel)>) =
            serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
        let rotation = nalgebra::Matrix3::from_column_slice(&[
            m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10],
        ]);
        Ok(Self {
            decision: std::sync::Arc::new(decision),
            odom_from_map: nalgebra::Isometry3::from_parts(
                nalgebra::Translation3::new(m[12], m[13], m[14]),
                nalgebra::UnitQuaternion::from_matrix(&rotation),
            ),
            point_labels: labels.into_iter().collect(),
        })
    }
    pub(crate) fn map_from_odom(&self) -> nalgebra::Isometry3<f64> {
        self.odom_from_map.inverse()
    }
    pub(crate) fn is_dynamic(&self, p: [f64; 3]) -> bool {
        self.point_labels.get(&bits(p)).is_some_and(|l| l.dynamic())
    }
    pub(crate) fn label(&self, p: [f64; 3]) -> PointLabel {
        self.point_labels
            .get(&bits(p))
            .copied()
            .unwrap_or(PointLabel::PendingNew)
    }
    pub(crate) fn clears(&self, p: [f64; 3]) -> bool {
        let p = self.odom_from_map * Point3::from(p);
        let p = [p.x, p.y, p.z];
        cell(p).is_some_and(|k| self.decision.clear_cells.contains(&k))
            && self.decision.references.protect(p, None).is_none()
    }
}
/// 与已选原生参数及已有观察标定取同一有效近距；设备未声明时保留原保守范围。
fn observation_minimum_range(
    backend: &str,
    options: &str,
    observation_min: Option<f64>,
) -> Result<f64, String> {
    use nuwax_lio_backends::{
        native::{NativeBackend, NativeLioConfig},
        pointlio_options::PointLioOptions,
    };
    let native = match backend.trim() {
        "pointlio" => {
            let mut config = NativeLioConfig::for_backend(NativeBackend::PointLio);
            config.pointlio_options = PointLioOptions::from_json(options)?;
            Some(config.minimum_observation_range_m())
        }
        "fastlio2" => Some(
            NativeLioConfig::for_backend(NativeBackend::FastLio2).minimum_observation_range_m(),
        ),
        "" | "device" => None,
        _ => return Err("动态点云观察范围对应的LIO后端无效".into()),
    };
    Ok(match (native, observation_min) {
        (Some(a), Some(b)) => a.max(b),
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0.5,
    })
}

impl Default for DynamicScene {
    fn default() -> Self {
        let minimum = (|| {
            let capabilities = nuwax_nav_types_messages::NavigationCapabilities::from_env()?;
            observation_minimum_range(
                &std::env::var("NUWAX_GEOMETRY_LIO_BACKEND").unwrap_or_default(),
                &std::env::var(nuwax_lio_backends::pointlio_options::POINTLIO_OPTIONS_ENV)
                    .unwrap_or_default(),
                capabilities.and_then(|value| value.observation.map(|model| model.min_range_m)),
            )
        })();
        // 原拥有方预检负责报告错误；无效配置不能偷偷放宽地图删除范围。
        Self::with_minimum_range(minimum.unwrap_or(f64::INFINITY))
    }
}

pub(crate) fn cell(point: [f64; 3]) -> Option<Cell> {
    let v = point.map(|x| (x / CELL_SIZE).floor());
    v.iter()
        .all(|x| x.is_finite() && *x > i32::MIN as f64 + 2. && *x < i32::MAX as f64 - 2.)
        .then(|| (v[0] as i32, v[1] as i32, v[2] as i32))
}
fn neighbors(k: Cell) -> impl Iterator<Item = Cell> {
    (-1..=1).flat_map(move |x| {
        (-1..=1).flat_map(move |y| (-1..=1).map(move |z| (k.0 + x, k.1 + y, k.2 + z)))
    })
}
fn bits(p: [f64; 3]) -> [u64; 3] {
    p.map(f64::to_bits)
}
impl DynamicScene {
    pub(crate) fn with_minimum_range(minimum_range_m: f64) -> Self {
        Self {
            offline: false,
            minimum_range_m,
            evidence: HashMap::new(),
            last_timestamp_ns: 0,
            clear_cells: HashSet::new(),
            dynamic_cells: HashSet::new(),
            frame: 0,
            window: VecDeque::new(),
            references: StaticReferences::default(),
            labels: FrameLabels {
                source_timestamp_ns: 0,
                labels: Vec::new(),
            },
            point_labels: HashMap::new(),
            tracks: Default::default(),
        }
    }
    pub(crate) fn decision(&self) -> std::sync::Arc<FrameDecision> {
        std::sync::Arc::new(FrameDecision {
            labels: self.labels.clone(),
            clear_cells: self.clear_cells.clone(),
            references: self.references.near(self.clear_cells.iter().map(|k| {
                [
                    (k.0 as f64 + 0.5) * CELL_SIZE,
                    (k.1 as f64 + 0.5) * CELL_SIZE,
                    (k.2 as f64 + 0.5) * CELL_SIZE,
                ]
            })),
        })
    }
    pub(crate) fn set_minimum_range_m(&mut self, value: f64) {
        self.minimum_range_m = value;
    }
    pub(crate) fn minimum_range_m(&self) -> f64 {
        self.minimum_range_m
    }
    pub(crate) fn clear_frame(&mut self) {
        self.clear_cells.clear();
        self.dynamic_cells.clear();
        self.labels.labels.clear();
        self.point_labels.clear();
    }
    pub(crate) fn labels(&self) -> &FrameLabels {
        &self.labels
    }
    pub(crate) fn known_void(&self, p: [f64; 3]) -> bool {
        cell(p)
            .and_then(|k| self.evidence.get(&k))
            .is_some_and(|e| e.known_free)
    }
    pub(crate) fn observe(&mut self, points: &[Point3<f64>], origin: [f64; 3], stamp: u64) {
        self.observe_with_ground(points, origin, stamp, &[]);
    }
    pub(crate) fn observe_with_ground(
        &mut self,
        points: &[Point3<f64>],
        origin: [f64; 3],
        stamp: u64,
        ground: &[u32],
    ) {
        self.observe_with_known_ground(points, origin, stamp, ground, &[]);
    }
    pub(crate) fn observe_with_known_ground(
        &mut self,
        points: &[Point3<f64>],
        origin: [f64; 3],
        stamp: u64,
        ground: &[u32],
        known: &[u32],
    ) {
        self.clear_frame();
        if stamp == 0
            || stamp <= self.last_timestamp_ns
            || points.is_empty()
            || cell(origin).is_none()
            || points.iter().any(|p| cell([p.x, p.y, p.z]).is_none())
        {
            return;
        }
        self.frame += 1;
        let mut hits = HashSet::new();
        let mut rays = Vec::new();
        for p in points {
            let xyz = [p.x, p.y, p.z];
            hits.insert(cell(xyz).unwrap());
            let range = (p.coords - nalgebra::Vector3::from(origin)).norm();
            if range > self.minimum_range_m + SURFACE_MARGIN {
                // Real hit extension towards the sensor, never isotropic dilation.
                let direction = (p.coords - nalgebra::Vector3::from(origin)) / range;
                for step in 0..=(SURFACE_MARGIN / CELL_SIZE * 2.).ceil() as usize {
                    let d = SURFACE_MARGIN * step as f64 / (SURFACE_MARGIN / CELL_SIZE * 2.).ceil();
                    let q = p.coords - direction * d;
                    if let Some(k) = cell(q.into()) {
                        hits.insert(k);
                    }
                }
                rays.push((p, range));
            }
        }
        let mut free = HashSet::new();
        for (p, r) in rays {
            trace(origin, p, r, self.minimum_range_m, &hits, &mut free);
        }
        self.last_timestamp_ns = stamp;
        let observed: HashSet<_> = hits.iter().chain(free.iter()).copied().collect();
        self.window.push_back((stamp, observed, hits.clone()));
        while self.window.len() > 5
            || self
                .window
                .front()
                .is_some_and(|f| stamp.saturating_sub(f.0) > 500_000_000)
        {
            self.window.pop_front();
        }
        let observed: HashSet<_> = self
            .window
            .iter()
            .flat_map(|f| f.1.iter().copied())
            .collect();
        let occupied: HashSet<_> = self
            .window
            .iter()
            .flat_map(|f| f.2.iter().copied())
            .collect();
        let raw: Vec<_> = points.iter().map(|p| [p.x, p.y, p.z]).collect();
        let mut ground_set: HashSet<usize> = ground
            .iter()
            .map(|&i| i as usize)
            .filter(|&i| i < points.len())
            .collect();
        // Reuse supplied ground; fallback only fills XYs without real ground.
        let ground_xy: HashSet<_> = ground_set
            .iter()
            .map(|&i| {
                let k = cell(raw[i]).unwrap();
                (k.0, k.1)
            })
            .collect();
        let known: HashSet<_> = known.iter().map(|&i| i as usize).collect();
        ground_set.extend(
            crate::offline_residual_cleanup::local_ground_indices(&raw)
                .into_iter()
                .filter(|&i| {
                    let k = cell(raw[i]).unwrap();
                    !known.contains(&i) && !ground_xy.contains(&(k.0, k.1))
                }),
        );
        let eligible: HashSet<_> = raw
            .iter()
            .enumerate()
            .filter_map(|(i, &p)| (!ground_set.contains(&i)).then_some(i))
            .collect();
        let before = self.tracks.prepare(&raw, &eligible, origin, stamp);
        let held: HashSet<_> = before.keys().filter_map(|&i| cell(raw[i])).collect();
        let mut recovered = Vec::new();
        for k in &hits {
            let e = self.evidence.entry(*k).or_default();
            if e.occupied_count == 0 || stamp.saturating_sub(e.last_occupied) > EVIDENCE_GAP_NS {
                e.occupied_since = stamp;
                e.occupied_count = 0;
            }
            e.occupied_count = e.occupied_count.saturating_add(1);
            e.last_occupied = stamp;
            e.last_hit_frame = self.frame;
            if e.total_hits == 0 {
                e.first_hit = stamp;
            }
            e.total_hits = e.total_hits.saturating_add(1);
            e.free_count = 0;
            e.first_clear = 0;
            if e.occupied_count >= RECOVERY_OBSERVATIONS
                && stamp.saturating_sub(e.occupied_since) >= RECOVERY_NS
                && !held.contains(k)
            {
                recovered.push(*k);
            }
        }
        for k in recovered {
            for n in neighbors(k) {
                if let Some(e) = self.evidence.get_mut(&n) {
                    e.known_free = false;
                }
            }
        }
        for k in &free {
            let e = self.evidence.entry(*k).or_default();
            if e.free_count == 0 || stamp.saturating_sub(e.last_free) > EVIDENCE_GAP_NS {
                e.free_count = 0;
                e.first_clear = stamp;
            }
            e.free_count = e.free_count.saturating_add(1);
            e.last_free = stamp;
            e.occupied_count = 0;
            e.occupied_since = 0;
            let static_confirmed =
                e.total_hits >= 5 && e.last_occupied.saturating_sub(e.first_hit) >= 2_000_000_000;
            let enough = if static_confirmed {
                e.free_count >= 10 && stamp.saturating_sub(e.first_clear) >= 1_000_000_000
            } else {
                e.free_count >= FREE_OBSERVATIONS
            };
            if enough
                && self.frame.saturating_sub(e.last_hit_frame) > 2
                && (e.known_free
                    || (!occupied.contains(k) && neighbors(*k).all(|n| observed.contains(&n))))
            {
                if !e.known_free {
                    e.void_since = stamp;
                }
                e.known_free = true;
                self.clear_cells.insert(*k);
            }
        }
        let clears = &self.clear_cells;
        self.references
            .remove_where(|p| cell(p).is_some_and(|k| clears.contains(&k)));
        let mut labels: Vec<_> = raw
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                if ground_set.contains(&i) {
                    PointLabel::Ground
                } else if let Some(label) = self.references.protect(p, before.get(&i).copied()) {
                    label
                } else if self.known_void(p) {
                    PointLabel::Seed
                } else {
                    PointLabel::PendingNew
                }
            })
            .collect();
        crate::human_point_labels::grow(points, &mut labels);
        let evidence = &self.evidence;
        self.tracks.update(&raw, &mut labels, stamp, &|k| {
            evidence
                .get(&k)
                .filter(|e| e.known_free)
                .map(|e| e.void_since)
        });
        for (&p, &label) in raw.iter().zip(&labels) {
            self.point_labels.insert(bits(p), label);
            if label.dynamic() {
                self.dynamic_cells.insert(cell(p).unwrap());
            }
        }
        self.references.observe(points, &labels, stamp);
        self.labels = FrameLabels {
            source_timestamp_ns: stamp,
            labels,
        };
        // Capacity eviction forgets only evidence, never geometry. Retain near
        // cells first rather than resetting the whole map when one frame is big.
        if !self.offline && self.evidence.len() > MAX_CELLS {
            let o = cell(origin).unwrap();
            let mut keys: Vec<_> = self.evidence.keys().copied().collect();
            keys.sort_unstable_by_key(|k| {
                std::cmp::Reverse(
                    (i64::from(k.0) - i64::from(o.0))
                        .abs()
                        .max((i64::from(k.1) - i64::from(o.1)).abs())
                        .max((i64::from(k.2) - i64::from(o.2)).abs()),
                )
            });
            for k in keys.into_iter().take(self.evidence.len() - MAX_CELLS) {
                self.evidence.remove(&k);
            }
        }
    }
    pub(crate) fn new_void_cells(&self) -> impl Iterator<Item = Cell> + '_ {
        self.clear_cells.iter().copied()
    }
    pub(crate) fn has_clears(&self) -> bool {
        !self.clear_cells.is_empty()
    }
    pub(crate) fn clears(&self, p: [f64; 3]) -> bool {
        cell(p).is_some_and(|k| self.clear_cells.contains(&k))
            && self.references.protect(p, None).is_none()
    }
    pub(crate) fn is_dynamic(&self, p: [f64; 3]) -> bool {
        self.point_labels.get(&bits(p)).is_some_and(|l| l.dynamic())
    }
}
fn trace(
    origin: [f64; 3],
    point: &Point3<f64>,
    range: f64,
    minimum_range_m: f64,
    hits: &HashSet<Cell>,
    free: &mut HashSet<Cell>,
) {
    let end = (range - SURFACE_MARGIN).min(RANGE);
    if end <= minimum_range_m {
        return;
    }
    let d = [
        point.x - origin[0],
        point.y - origin[1],
        point.z - origin[2],
    ]
    .map(|v| v / range);
    let s = cell(origin).unwrap();
    let mut k = [s.0, s.1, s.2];
    let mut step = [0; 3];
    let mut next = [f64::INFINITY; 3];
    let mut interval = [f64::INFINITY; 3];
    for a in 0..3 {
        if d[a] != 0. {
            step[a] = if d[a] > 0. { 1 } else { -1 };
            let b = (k[a] as f64 + if step[a] > 0 { 1. } else { 0. }) * CELL_SIZE;
            next[a] = ((b - origin[a]) / d[a]).max(0.);
            interval[a] = CELL_SIZE / d[a].abs();
        }
    }
    let mut distance = 0.;
    // DDA's finite range is the termination bound, not an arbitrary ray quota.
    while distance < end {
        let c = (k[0], k[1], k[2]);
        if hits.contains(&c) {
            break;
        }
        let a = if next[0] <= next[1] && next[0] <= next[2] {
            0
        } else if next[1] <= next[2] {
            1
        } else {
            2
        };
        let exit = next[a].min(end);
        if distance >= minimum_range_m && exit - distance > CELL_SIZE * 0.05 {
            free.insert(c);
        }
        if next[a] >= end {
            break;
        }
        distance = next[a];
        k[a] += step[a];
        next[a] += interval[a];
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const ORIGIN: [f64; 3] = [0.03, 0.03, 0.65];

    #[test]
    fn known_free_person_trail_retires_after_local_visibility_returns() {
        let mut scene = DynamicScene::with_minimum_range(0.05);
        let target = [2.7, 0.03, 0.65];
        for frame in 1..=4 {
            let (points, _) = corridor(None, (frame % 3) as f64);
            scene.observe(&points, ORIGIN, frame * 100_000_000);
        }
        assert!(
            scene
                .evidence
                .get(&cell(target).unwrap())
                .is_some_and(|e| e.known_free),
            "空走廊完整观察必须先证明这个区域可用"
        );
        let (person, truth) = corridor(Some(0.0), 0.0);
        let actual = person
            .iter()
            .zip(&truth)
            .find(|(p, h)| **h && cell([p.x, p.y, p.z]) == cell(target))
            .unwrap()
            .0;
        scene.observe(&person, ORIGIN, 500_000_000);
        assert!(
            scene.is_dynamic([actual.x, actual.y, actual.z]),
            "人体进入原空闲区域仍作为当前动态障碍"
        );
        let visible = vec![Point3::new(8.0, 0.03, 0.65)];
        for stamp in [600_000_000, 600_000_000, 700_000_000] {
            scene.observe(&visible, ORIGIN, stamp);
            assert!(!scene.clears(target), "重复帧或不足三帧不能确认人体离开");
        }
        scene.observe(&visible, ORIGIN, 800_000_000);
        assert!(
            scene.clears(target),
            "局部真实穿越恢复后不再要求整个27格同时可见"
        );
        let unseen = [2.7, 0.63, 0.65];
        assert!(!scene.clears(unseen), "未重新观察的旁格不能跟随清除");
    }

    /// 独立解析射线与地面、墙面、直立圆柱求交，返回每个回波的真实动态标签。
    pub(crate) fn corridor(person_y: Option<f64>, phase: f64) -> (Vec<Point3<f64>>, Vec<bool>) {
        let mut points = Vec::new();
        let mut labels = Vec::new();
        for row in 0..61 {
            let elevation = -0.55 + row as f64 * 0.017 + phase * 0.002;
            for col in 0..141 {
                let azimuth = -0.7 + col as f64 * 0.01 + phase * 0.002;
                let direction = [
                    elevation.cos() * azimuth.cos(),
                    elevation.cos() * azimuth.sin(),
                    elevation.sin(),
                ];
                let mut distance = (8.0 - ORIGIN[0]) / direction[0];
                if direction[2] < 0.0 {
                    distance = distance.min(-ORIGIN[2] / direction[2]);
                }
                if direction[2] > 0.0 {
                    distance = distance.min((3.0 - ORIGIN[2]) / direction[2]);
                }
                if direction[1] != 0.0 {
                    let side = if direction[1] > 0.0 { 3.0 } else { -3.0 };
                    distance = distance.min((side - ORIGIN[1]) / direction[1]);
                }
                let mut dynamic = false;
                if let Some(y) = person_y {
                    let dx = ORIGIN[0] - 3.0;
                    let dy = ORIGIN[1] - y;
                    let a = direction[0].powi(2) + direction[1].powi(2);
                    let b = 2.0 * (dx * direction[0] + dy * direction[1]);
                    let c = dx * dx + dy * dy - 0.25_f64.powi(2);
                    let discriminant = b * b - 4.0 * a * c;
                    if discriminant >= 0.0 {
                        let hit = (-b - discriminant.sqrt()) / (2.0 * a);
                        let z = ORIGIN[2] + hit * direction[2];
                        if hit > 0.0 && hit < distance && (0.12..=1.8).contains(&z) {
                            distance = hit;
                            dynamic = true;
                        }
                    }
                }
                points.push(Point3::from(std::array::from_fn::<_, 3, _>(|i| {
                    ORIGIN[i] + distance * direction[i]
                })));
                labels.push(dynamic);
            }
        }
        (points, labels)
    }

    #[test]
    fn calibrated_range_follows_selected_native_blind_and_observation_model() {
        assert_eq!(
            observation_minimum_range("pointlio", r#"{"blind":0.05}"#, None).unwrap(),
            f64::from(0.05_f32)
        );
        assert_eq!(
            observation_minimum_range("pointlio", "{}", None).unwrap(),
            0.5
        );
        assert_eq!(
            observation_minimum_range("fastlio2", "ignored", None).unwrap(),
            0.5
        );
        assert_eq!(
            observation_minimum_range("pointlio", r#"{"blind":0.05}"#, Some(0.12)).unwrap(),
            0.12
        );
        assert_eq!(
            observation_minimum_range("device", "ignored", Some(0.1)).unwrap(),
            0.1
        );
        assert_eq!(
            observation_minimum_range("device", "ignored", None).unwrap(),
            0.5
        );
        assert!(observation_minimum_range("pointlio", r#"{"blind":-1}"#, None).is_err());
    }

    #[test]
    fn known_free_space_detects_passing_person_without_labeling_static_room() {
        let mut scene = DynamicScene::default();
        for frame in 1..=6 {
            scene.observe(
                &corridor(None, frame as f64 % 3.0).0,
                ORIGIN,
                frame * 100_000_000,
            );
        }
        let (points, labels) = corridor(Some(0.0), 0.0);
        scene.observe(&points, ORIGIN, 700_000_000);
        let true_positive = points
            .iter()
            .zip(&labels)
            .filter(|(p, label)| **label && scene.is_dynamic([p.x, p.y, p.z]))
            .count();
        let false_positive = points
            .iter()
            .zip(&labels)
            .filter(|(p, label)| !**label && scene.is_dynamic([p.x, p.y, p.z]))
            .count();
        let people = labels.iter().filter(|label| **label).count();
        eprintln!("人体回波={people} 识别动态={true_positive} 静态误删={false_positive}");
        assert!(true_positive as f64 / people as f64 > 0.8);
        assert_eq!(false_positive, 0);
        scene.observe(&points, ORIGIN, 700_000_000);
        assert!(!scene.has_clears(), "重复源时间不能重复判定");
        scene.observe(&points, ORIGIN, 600_000_000);
        assert!(!scene.has_clears(), "倒序源时间不能沿用删除结果");
    }

    #[test]
    fn unseen_person_and_occluded_space_are_not_assumed_free() {
        let mut scene = DynamicScene::default();
        let (points, _) = corridor(Some(0.0), 0.0);
        for frame in 1..=6 {
            scene.observe(&points, ORIGIN, frame * 100_000_000);
        }
        assert!(!scene.clears([3.1, 0.03, 1.0]));
        assert!(!scene.is_dynamic([2.8, 0.03, 1.0]));
        let (background, _) = corridor(None, 0.0);
        for frame in 7..=12 {
            scene.observe(&background, ORIGIN, frame * 100_000_000);
        }
        assert!(scene.clears([3.1, 0.03, 1.0]));
        assert!(!scene.clears([8.0, 0.03, 1.0]));
        assert!(!scene.clears([3.1, 0.03, 0.0]));
    }

    #[test]
    fn continuously_occupied_space_recovers_but_invalid_frames_do_not_vote() {
        let mut scene = DynamicScene::default();
        for frame in 1..=6 {
            scene.observe(&corridor(None, 0.0).0, ORIGIN, frame * 100_000_000);
        }
        let (points, labels) = corridor(Some(0.0), 0.0);
        let p = points
            .iter()
            .zip(&labels)
            .find(|(p, label)| **label && p.z > 0.9 && p.z < 1.1)
            .unwrap()
            .0;
        scene.observe(&points, ORIGIN, 700_000_000);
        assert!(scene.is_dynamic([p.x, p.y, p.z]));
        scene.observe(&[Point3::new(f64::NAN, 0., 0.)], ORIGIN, 800_000_000);
        assert!(!scene.has_clears());
        for frame in 8..=28 {
            scene.observe(&points, ORIGIN, frame * 100_000_000);
        }
        assert!(!scene.is_dynamic([p.x, p.y, p.z]));
    }
}
