//! Request-local recent obstacle hints for PCT guidance, never execution authority.
//! Static terrain roots and graph connectivity remain untouched. Radius projection
//! is deliberately only a guide approximation; a blocked guide falls back to Scan.
use std::collections::{HashMap, HashSet};

type Bounds = ([f64; 3], [f64; 3]);
pub(super) struct RecentGuideOverlay {
    size: f64,
    boxes: Vec<Bounds>,
    bins: HashMap<[i32; 2], Vec<usize>>,
}
impl RecentGuideOverlay {
    pub(super) fn new(
        hints: Vec<Bounds>, resolution: f64, body: &nuwax_nav_mls_planner::SurfaceBodyEnvelope,
    ) -> Result<Self, String> {
        let radius = body.radius();
        let size = resolution.max(radius * 2.0);
        let mut result = Self { size, boxes: Vec::with_capacity(hints.len()), bins: HashMap::new() };
        for (low, high) in hints {
            if nuwax_nav_pct::planning_cancelled() { return Err("recent guide planning cancelled".into()); }
            // Transform obstacle bounds into forbidden support-center bounds.
            // XY uses the body's circumscribed radius, Z its declared support offset.
            let low = [low[0]-radius, low[1]-radius, low[2]-body.body_to_support_z_m-body.z_bounds_m[1]];
            let high = [high[0]+radius, high[1]+radius, high[2]-body.body_to_support_z_m-body.z_bounds_m[0]];
            let id = result.boxes.len();
            result.boxes.push((low, high));
            let a = result.bin(low); let b = result.bin(high);
            for x in a[0]..=b[0] {
                if nuwax_nav_pct::planning_cancelled() { return Err("recent guide planning cancelled".into()); }
                for y in a[1]..=b[1] { result.bins.entry([x,y]).or_default().push(id); }
            }
        }
        Ok(result)
    }
    pub(super) fn is_empty(&self) -> bool { self.boxes.is_empty() }
    fn bin(&self, point: [f64;3]) -> [i32;2] {
        [(point[0]/self.size).floor() as i32, (point[1]/self.size).floor() as i32]
    }
    pub(super) fn accepts(&self, a: [f64;3], b: [f64;3]) -> bool {
        let lo = self.bin(std::array::from_fn(|i|a[i].min(b[i])));
        let hi = self.bin(std::array::from_fn(|i|a[i].max(b[i])));
        let mut seen = HashSet::new();
        // Neighbor checks touch only adjacent bins. Reduction checks at most its
        // existing 3m shortcut window, rather than scanning the complete cloud.
        for x in lo[0]..=hi[0] {
            if nuwax_nav_pct::planning_cancelled() { return false; }
            for y in lo[1]..=hi[1] {
                if let Some(ids) = self.bins.get(&[x,y]) {
                    for &id in ids {
                        if seen.insert(id) && intersects(a,b,self.boxes[id]) { return false; }
                    }
                }
            }
        }
        true
    }
}
fn intersects(a: [f64;3], b: [f64;3], (low, high): Bounds) -> bool {
    let (mut enter, mut leave) = (0.0_f64, 1.0_f64);
    for axis in 0..3 {
        let delta = b[axis]-a[axis];
        if delta == 0.0 {
            if a[axis]<low[axis] || a[axis]>high[axis] { return false; }
        } else {
            let t0 = (low[axis]-a[axis])/delta; let t1 = (high[axis]-a[axis])/delta;
            enter = enter.max(t0.min(t1)); leave = leave.min(t0.max(t1));
            if enter>leave { return false; }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn segment_box_filter_covers_crossings_endpoints_and_other_heights() {
        let bounds = ([0.6,0.7,-0.1],[0.9,1.3,0.15]);
        assert!(intersects([0.5,1.,0.1],[1.5,1.,0.1],bounds));
        assert!(intersects([0.5,1.,0.1],[0.6,1.,0.1],bounds));
        assert!(intersects([0.75,1.,0.1],[0.75,1.,0.1],bounds));
        assert!(!intersects([0.5,1.,0.2],[1.5,1.,0.2],bounds));
        assert!(!intersects([0.5,1.5,0.1],[1.5,1.5,0.1],bounds));
    }
}
