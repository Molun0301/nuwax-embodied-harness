//! 多分辨率动态点证据：在粗网格累计保守自由空间，在原分辨率删除历史点。
//!
//! 参考 FreeDOM（https://arxiv.org/abs/2504.11073）与 Dynablox 的自由空间判据。
//! 只使用真实回波，不推测无回波方向；没有运动证据时不按人体外形删除结构。

use ahash::{AHashMap as HashMap, AHashSet as HashSet};
use nalgebra::Point3;

pub(crate) type Cell = (i32, i32, i32);
pub(crate) const CELL_SIZE: f64 = 0.2;
const RANGE: f64 = 30.0;
const MIN_RANGE: f64 = 0.5;
const SURFACE_MARGIN: f64 = 0.3;
const FREE_OBSERVATIONS: u8 = 3;
const EVIDENCE_GAP_NS: u64 = 2_000_000_000;
const RETENTION_NS: u64 = 60_000_000_000;
const RECOVERY_NS: u64 = 2_000_000_000;
const RECOVERY_OBSERVATIONS: u16 = 20;
const MAX_CELLS: usize = 500_000;
const MAX_ENDPOINTS: usize = 65_536;

#[derive(Clone, Default)]
struct Evidence {
    free_count: u8,
    last_free: u64,
    occupied_count: u16,
    occupied_since: u64,
    last_occupied: u64,
    known_free: bool,
}

#[derive(Clone, Default)]
pub(crate) struct DynamicScene {
    evidence: HashMap<Cell, Evidence>,
    last_timestamp_ns: u64,
    clear_cells: HashSet<Cell>,
    dynamic_cells: HashSet<Cell>,
}

pub(crate) fn cell(point: [f64; 3]) -> Option<Cell> {
    let values = point.map(|value| (value / CELL_SIZE).floor());
    values
        .iter()
        .all(|v| v.is_finite() && *v > i32::MIN as f64 + 2.0 && *v < i32::MAX as f64 - 2.0)
        .then(|| (values[0] as i32, values[1] as i32, values[2] as i32))
}

fn neighbors(key: Cell) -> impl Iterator<Item = Cell> {
    (-1..=1).flat_map(move |x| {
        (-1..=1).flat_map(move |y| (-1..=1).map(move |z| (key.0 + x, key.1 + y, key.2 + z)))
    })
}

impl DynamicScene {
    pub(crate) fn clear_frame(&mut self) {
        self.clear_cells.clear();
        self.dynamic_cells.clear();
    }
    /// 本帧只生成一次证据；缺原点、非法/倒序扫描不沿用上一帧删除结果。
    pub(crate) fn observe(&mut self, points: &[Point3<f64>], origin: [f64; 3], timestamp_ns: u64) {
        self.clear_cells.clear();
        self.dynamic_cells.clear();
        if timestamp_ns == 0
            || timestamp_ns <= self.last_timestamp_ns
            || points.is_empty()
            || cell(origin).is_none()
        {
            return;
        }

        let mut endpoints: HashMap<Cell, (Point3<f64>, f64)> = HashMap::new();
        let mut hits = HashSet::new();
        for point in points {
            let Some(key) = cell([point.x, point.y, point.z]) else {
                return;
            };
            hits.insert(key);
            let range = (point.coords - nalgebra::Vector3::from(origin)).norm();
            if range <= MIN_RANGE + SURFACE_MARGIN {
                continue;
            }
            // 同一个回波格仅用最近的真实端点发射射线，全部命中格仍参与遮挡。
            let endpoint = endpoints.entry(key).or_insert((*point, range));
            if range < endpoint.1 {
                *endpoint = (*point, range);
            }
        }
        if endpoints.len() > MAX_ENDPOINTS {
            return;
        }
        let mut free = HashSet::new();
        for (point, range) in endpoints.values() {
            trace(origin, point, *range, &hits, &mut free);
            // 容量不足时整帧不提供新自由证据，不能让任意散列顺序决定删除区域。
            if free.len() > MAX_CELLS {
                return;
            }
        }
        if free.len().saturating_add(hits.len()) > MAX_CELLS {
            return;
        }
        self.last_timestamp_ns = timestamp_ns;
        let origin_cell = cell(origin).unwrap();
        let radius = (RANGE / CELL_SIZE) as i64 + 2;
        self.evidence.retain(|key, evidence| {
            timestamp_ns.saturating_sub(evidence.last_free.max(evidence.last_occupied))
                <= RETENTION_NS
                && (i64::from(key.0) - i64::from(origin_cell.0)).abs() <= radius
                && (i64::from(key.1) - i64::from(origin_cell.1)).abs() <= radius
                && (i64::from(key.2) - i64::from(origin_cell.2)).abs() <= radius
        });
        // 未知自由证据宁可重新积累，也不能无限增长或把淘汰误认为地图删除。
        if self
            .evidence
            .len()
            .saturating_add(free.len())
            .saturating_add(hits.len())
            > MAX_CELLS
        {
            let additions = free
                .iter()
                .chain(hits.iter())
                .filter(|key| !self.evidence.contains_key(key))
                .count();
            if self.evidence.len().saturating_add(additions) > MAX_CELLS {
                self.evidence.clear();
            }
        }
        let mut recovered = Vec::new();
        for key in &hits {
            let e = self.evidence.entry(*key).or_default();
            if e.occupied_count == 0
                || timestamp_ns.saturating_sub(e.last_occupied) > EVIDENCE_GAP_NS
            {
                e.occupied_since = timestamp_ns;
                e.occupied_count = 0;
            }
            e.occupied_count = e.occupied_count.saturating_add(1);
            e.last_occupied = timestamp_ns;
            e.free_count = 0;
            if e.occupied_count >= RECOVERY_OBSERVATIONS
                && timestamp_ns.saturating_sub(e.occupied_since) >= RECOVERY_NS
            {
                recovered.push(*key);
            }
        }
        // 持续静态命中能恢复自由判定，避免轻微位姿漂移永久挖空墙面。
        for key in recovered {
            for neighbor in neighbors(key) {
                if let Some(e) = self.evidence.get_mut(&neighbor) {
                    e.known_free = false;
                }
            }
        }
        for key in &hits {
            if self.evidence.get(key).is_some_and(|e| e.known_free) {
                self.dynamic_cells.insert(*key);
            }
        }
        for key in &free {
            let e = self.evidence.entry(*key).or_default();
            if timestamp_ns.saturating_sub(e.last_free) > EVIDENCE_GAP_NS {
                e.free_count = 0;
            }
            e.free_count = e.free_count.saturating_add(1);
            e.last_free = timestamp_ns;
            e.occupied_count = 0;
            e.occupied_since = 0;
        }
        // 同帧27邻域都被真实射线穿过，并各自至少有三次有效观测才确认空闲。
        // 当前回波格从来不在free中；遮挡、薄墙和地面擦射不能靠单束扩大清空。
        for key in &free {
            if neighbors(*key).all(|neighbor| {
                free.contains(&neighbor)
                    && self
                        .evidence
                        .get(&neighbor)
                        .is_some_and(|e| e.free_count >= FREE_OBSERVATIONS)
            }) {
                self.clear_cells.insert(*key);
            }
        }
        for key in &self.clear_cells {
            self.evidence.get_mut(key).unwrap().known_free = true;
        }
        // 动态区历史先撤销，调用者随后重新加入本帧真实障碍供导航使用。
        self.clear_cells.extend(self.dynamic_cells.iter().copied());
    }

    pub(crate) fn has_clears(&self) -> bool {
        !self.clear_cells.is_empty()
    }

    pub(crate) fn clears(&self, point: [f64; 3]) -> bool {
        cell(point).is_some_and(|key| self.clear_cells.contains(&key))
    }

    pub(crate) fn is_dynamic(&self, point: [f64; 3]) -> bool {
        cell(point).is_some_and(|key| self.dynamic_cells.contains(&key))
    }
}

fn trace(
    origin: [f64; 3],
    point: &Point3<f64>,
    range: f64,
    hits: &HashSet<Cell>,
    free: &mut HashSet<Cell>,
) {
    let end = (range - SURFACE_MARGIN).min(RANGE);
    if end <= MIN_RANGE {
        return;
    }
    let direction = [
        point.x - origin[0],
        point.y - origin[1],
        point.z - origin[2],
    ]
    .map(|v| v / range);
    let start = cell(origin).unwrap();
    let mut key = [start.0, start.1, start.2];
    let mut step = [0; 3];
    let mut next = [f64::INFINITY; 3];
    let mut interval = [f64::INFINITY; 3];
    for axis in 0..3 {
        if direction[axis] != 0.0 {
            step[axis] = if direction[axis] > 0.0 { 1 } else { -1 };
            let boundary = (key[axis] as f64 + if step[axis] > 0 { 1.0 } else { 0.0 }) * CELL_SIZE;
            next[axis] = ((boundary - origin[axis]) / direction[axis]).max(0.0);
            interval[axis] = CELL_SIZE / direction[axis].abs();
        }
    }
    let mut distance = 0.0;
    for _ in 0..800 {
        let current = (key[0], key[1], key[2]);
        if hits.contains(&current) {
            break;
        }
        let axis = if next[0] <= next[1] && next[0] <= next[2] {
            0
        } else if next[1] <= next[2] {
            1
        } else {
            2
        };
        let exit = next[axis].min(end);
        // 不记录只擦过角点的零长度格；近场与回波端点留保护带。
        if distance >= MIN_RANGE && exit - distance > CELL_SIZE * 0.05 {
            free.insert(current);
        }
        if next[axis] >= end {
            break;
        }
        distance = next[axis];
        key[axis] += step[axis];
        next[axis] += interval[axis];
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) const ORIGIN: [f64; 3] = [0.03, 0.03, 0.65];

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
