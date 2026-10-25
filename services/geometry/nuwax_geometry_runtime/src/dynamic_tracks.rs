//! Tracks never classify geometry as dynamic. They only delay void recovery for
//! measured points belonging to a shape-continuous, one-to-one matched object.
use crate::dynamic_scene::{cell, Cell};
use crate::human_point_labels::PointLabel;
use ahash::{AHashMap as HashMap, AHashSet as HashSet};
#[derive(Clone)]
struct Cluster {
    ids: Vec<usize>,
    center: [f64; 3],
    height: f64,
    area: f64,
    radius: f64,
}
#[derive(Clone)]
struct Track {
    first: u64,
    last: u64,
    center: [f64; 3],
    previous: [f64; 3],
    height: f64,
    area: f64,
    radius: f64,
    points: Vec<[f64; 3]>,
    seed_frames: u16,
    proven: bool,
    stop_since: u64,
    stop_anchor: [f64; 3],
}
#[derive(Clone, Default)]
pub(crate) struct Tracks {
    tracks: Vec<Track>,
    matches: Vec<(usize, Cluster, Vec<usize>)>,
}
fn d2(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum()
}
fn grid(p: [f64; 3]) -> [i32; 3] {
    p.map(|v| (v / 0.12).floor() as i32)
}
fn clusters(points: &[[f64; 3]], eligible: &HashSet<usize>) -> Vec<Cluster> {
    let mut bins: HashMap<[i32; 3], Vec<usize>> = HashMap::new();
    for &i in eligible {
        bins.entry(grid(points[i])).or_default().push(i);
    }
    let mut unseen = eligible.clone();
    let mut out = Vec::new();
    while let Some(&first) = unseen.iter().next() {
        unseen.remove(&first);
        let mut ids = vec![first];
        let mut cursor = 0;
        while cursor < ids.len() {
            let i = ids[cursor];
            cursor += 1;
            let k = grid(points[i]);
            for x in -1..=1 {
                for y in -1..=1 {
                    for z in -1..=1 {
                        if let Some(neighbors) = bins.get(&[k[0] + x, k[1] + y, k[2] + z]) {
                            for &j in neighbors {
                                if unseen.contains(&j)
                                    && d2(points[i], points[j]) <= 0.12f64.powi(2)
                                {
                                    unseen.remove(&j);
                                    ids.push(j);
                                }
                            }
                        }
                    }
                }
            }
        }
        if ids.len() < 8 {
            continue;
        }
        let cells: HashSet<_> = ids.iter().filter_map(|&i| cell(points[i])).collect();
        if cells.len() < 3 {
            continue;
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut center = [0.; 3];
        for &i in &ids {
            for a in 0..3 {
                lo[a] = lo[a].min(points[i][a]);
                hi[a] = hi[a].max(points[i][a]);
                center[a] += points[i][a];
            }
        }
        let extent = std::array::from_fn::<_, 3, _>(|a| hi[a] - lo[a]);
        if extent[0] > 3. || extent[1] > 3. || extent[2] > 2.5 {
            continue;
        }
        for v in &mut center {
            *v /= ids.len() as f64;
        }
        out.push(Cluster {
            ids,
            center,
            height: extent[2].max(0.03),
            area: extent[0].max(0.03) * extent[1].max(0.03),
            radius: extent.iter().map(|v| v * v).sum::<f64>().sqrt() / 2.,
        });
    }
    out.sort_by(|a, b| {
        a.center[0]
            .total_cmp(&b.center[0])
            .then(a.center[1].total_cmp(&b.center[1]))
            .then(a.center[2].total_cmp(&b.center[2]))
    });
    out
}
impl Tracks {
    /// Membership is evaluated before the recovery counter changes. Returns a
    /// per-point pre-track reference cutoff only for proven matched objects.
    pub(crate) fn prepare(
        &mut self,
        points: &[[f64; 3]],
        eligible: &HashSet<usize>,
        origin: [f64; 3],
        stamp: u64,
    ) -> HashMap<usize, u64> {
        self.tracks
            .retain(|t| stamp.saturating_sub(t.last) <= 1_000_000_000);
        self.matches.clear();
        let groups = clusters(points, eligible);
        let mut costs = Vec::new();
        for (ti, t) in self.tracks.iter().enumerate() {
            let predicted = std::array::from_fn(|a| t.center[a] + t.center[a] - t.previous[a]);
            for (ci, c) in groups.iter().enumerate() {
                let ratio = c.height / t.height;
                let area = c.area / t.area;
                if !(0.7..=1.3).contains(&ratio) || !(0.5..=2.).contains(&area) {
                    continue;
                }
                let dist = d2(c.center, predicted);
                if dist <= (c.radius + t.radius + 0.12).powi(2) {
                    costs.push((dist, ti, ci));
                }
            }
        }
        // All candidate pairs compete together; a track or cluster can win only
        // once. Splits start fresh tracks and cannot inherit a motion label.
        costs.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let mut used_tracks = HashSet::new();
        let mut used_clusters = HashSet::new();
        let mut before = HashMap::new();
        for (_, ti, ci) in costs {
            if used_tracks.contains(&ti) || used_clusters.contains(&ci) {
                continue;
            }
            let t = &self.tracks[ti];
            let c = &groups[ci];
            let offset = std::array::from_fn::<_, 3, _>(|a| c.center[a] - t.center[a]);
            let mut members = Vec::new();
            for &i in &c.ids {
                let p = points[i];
                let current = std::array::from_fn::<_, 3, _>(|a| p[a] - origin[a]);
                let range = d2(p, origin).sqrt();
                // Nearest actual surface after object translation, plus a real
                // depth-reveal veto along the sensor direction.
                let old = t.points.iter().min_by(|a, b| {
                    d2(p, std::array::from_fn(|k| a[k] + offset[k]))
                        .total_cmp(&d2(p, std::array::from_fn(|k| b[k] + offset[k])))
                });
                if let Some(old) = old {
                    let translated = std::array::from_fn(|a| old[a] + offset[a]);
                    let old_range = d2(*old, origin).sqrt();
                    let parallel = if range > 0. {
                        (0..3)
                            .map(|a| (old[a] - origin[a]) * current[a] / range)
                            .sum::<f64>()
                    } else {
                        old_range
                    };
                    let lateral = (old_range.powi(2) - parallel.powi(2)).max(0.).sqrt();
                    if d2(p, translated) <= 0.12f64.powi(2)
                        && !(lateral <= 0.12 && range - parallel > 0.15)
                    {
                        members.push(i);
                    }
                }
            }
            if members.is_empty() {
                continue;
            }
            used_tracks.insert(ti);
            used_clusters.insert(ci);
            if t.proven && stamp.saturating_sub(t.stop_since) <= 30_000_000_000 {
                for &i in &members {
                    before.insert(i, t.first);
                }
            }
            self.matches.push((ti, c.clone(), members));
        }
        for (ci, c) in groups.into_iter().enumerate() {
            if used_clusters.contains(&ci) {
                continue;
            }
            let ti = self.tracks.len();
            self.tracks.push(Track {
                first: stamp,
                last: stamp,
                center: c.center,
                previous: c.center,
                height: c.height,
                area: c.area,
                radius: c.radius,
                points: c.ids.iter().map(|&i| points[i]).collect(),
                seed_frames: 0,
                proven: false,
                stop_since: stamp,
                stop_anchor: c.center,
            });
            self.matches.push((ti, c.clone(), c.ids.clone()));
        }
        before
    }
    pub(crate) fn update(
        &mut self,
        points: &[[f64; 3]],
        labels: &mut [PointLabel],
        stamp: u64,
        void_since: &impl Fn(Cell) -> Option<u64>,
    ) {
        for (ti, c, members) in &self.matches {
            let t = &mut self.tracks[*ti];
            let preexisting_void = members.iter().any(|&i| {
                labels[i] == PointLabel::Seed
                    && cell(points[i])
                        .and_then(void_since)
                        .is_some_and(|v| v < t.first)
            });
            if preexisting_void {
                t.seed_frames = t.seed_frames.saturating_add(1);
            }
            let vacated = t
                .points
                .iter()
                .any(|&p| cell(p).and_then(void_since).is_some_and(|v| v > t.first));
            t.proven |= t.seed_frames >= 2 || vacated;
            if d2(c.center, t.stop_anchor) > 0.03f64.powi(2) {
                t.stop_anchor = c.center;
                t.stop_since = stamp;
            }
            for &i in members {
                if labels[i] == PointLabel::PendingNew && !t.proven {
                    labels[i] = PointLabel::PendingTrack;
                }
            }
            t.previous = t.center;
            t.center = c.center;
            t.height = c.height;
            t.area = c.area;
            t.radius = c.radius;
            t.last = stamp;
            t.points = members.iter().map(|&i| points[i]).collect();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn body(x: f64) -> Vec<[f64; 3]> {
        (0..=20)
            .flat_map(|z| (-1..=1).map(move |y| [x, y as f64 * 0.04, 0.2 + z as f64 * 0.05]))
            .collect()
    }
    #[test]
    fn tracking_only_holds_actual_matched_points_and_expires() {
        let mut tracks = Tracks::default();
        let points = body(3.);
        let eligible = (0..points.len()).collect();
        for frame in 1..=2 {
            tracks.prepare(&points, &eligible, [0., 0., 0.65], frame * 100_000_000);
            let mut labels = vec![PointLabel::Seed; points.len()];
            tracks.update(&points, &mut labels, frame * 100_000_000, &|_| Some(1));
        }
        assert_eq!(
            tracks
                .prepare(&points, &eligible, [0., 0., 0.65], 300_000_000)
                .len(),
            points.len()
        );
        let mut labels = vec![PointLabel::PendingNew; points.len()];
        tracks.update(&points, &mut labels, 300_000_000, &|_| None);
        assert!(
            labels.iter().all(|l| !l.dynamic()),
            "track identity must not create motion evidence"
        );
        // Keep observing the same stopped surface. The hold ends at 30 seconds.
        for frame in 4..=305 {
            let held = tracks.prepare(&points, &eligible, [0., 0., 0.65], frame * 100_000_000);
            if frame == 305 {
                assert!(held.is_empty());
            }
            let mut labels = vec![PointLabel::Seed; points.len()];
            tracks.update(&points, &mut labels, frame * 100_000_000, &|_| Some(1));
        }
        assert!(
            tracks
                .prepare(&points, &eligible, [0., 0., 0.65], 32_000_000_000)
                .is_empty(),
            "unseen for one second ends previous motion qualification"
        );
    }
    #[test]
    fn exposed_background_cannot_inherit_motion_hold() {
        let mut tracks = Tracks::default();
        let points = body(3.);
        let eligible = (0..points.len()).collect();
        for frame in 1..=2 {
            tracks.prepare(&points, &eligible, [0., 0., 0.65], frame * 100_000_000);
            let mut labels = vec![PointLabel::Seed; points.len()];
            tracks.update(&points, &mut labels, frame * 100_000_000, &|_| Some(1));
        }
        let behind = body(3.3);
        assert!(tracks
            .prepare(&behind, &eligible, [0., 0., 0.65], 300_000_000)
            .is_empty());
        let mut labels = vec![PointLabel::SurfaceLine; behind.len()];
        tracks.update(&behind, &mut labels, 300_000_000, &|_| None);
        assert!(labels.iter().all(|l| *l == PointLabel::SurfaceLine));
    }
}
