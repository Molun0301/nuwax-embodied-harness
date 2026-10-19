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
        hints: Vec<Bounds>, resolution: f64, source_quantum: f64,
        body: &nuwax_nav_mls_planner::SurfaceBodyEnvelope,
    ) -> Result<Self, String> {
        if !source_quantum.is_finite() || source_quantum<=0.0 {
            return Err("近期障碍引导缺少有效的源分辨率".into());
        }
        let radius = body.radius();
        let size = resolution.max(radius * 2.0);
        let mut result = Self { size, boxes: Vec::with_capacity(hints.len()), bins: HashMap::new() };
        for (low, high) in hints {
            if nuwax_nav_pct::planning_cancelled() { return Err("recent guide planning cancelled".into()); }
            let surface = (low == high).then_some(low[2]);
            // Transform obstacle bounds into forbidden support-center bounds.
            // XY uses the body's circumscribed radius, Z its declared support offset.
            let low = [low[0]-radius, low[1]-radius, low[2]-body.body_to_support_z_m-body.z_bounds_m[1]];
            let mut high = [high[0]+radius, high[1]+radius, high[2]-body.body_to_support_z_m-body.z_bounds_m[0]];
            if let Some(z) = surface {
                // 零体积提示来自已分类的真实面点。接触关联沿用机体查询的
                // 源量化加标定不确定度，不用更粗的层析格距扩大误差范围。
                // 面点仍阻挡其下方其它楼层的机身；未分类占据体完全保留。
                let contact = z - source_quantum - body.support_uncertainty_m;
                high[2] = high[2].min(contact.next_down());
            }
            if low[2]>high[2] {continue;}
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
    fn measured_contact_on_a_higher_floor_does_not_hide_an_obstacle_on_the_lower_floor() {
        let body=nuwax_nav_mls_planner::SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m:[-0.45,0.2],body_to_support_z_m:0.4,
            max_step_up_m:0.2,max_step_down_m:0.2,max_cross_slope_deg:18.0,support_uncertainty_m:0.0,
        };
        let point=[1.0,0.0,1.075];
        let surface=RecentGuideOverlay::new(vec![(point,point)],0.05,0.05,&body).unwrap();
        assert!(surface.accepts([0.5,0.0,1.05],[1.5,0.0,1.05]),
            "未来楼层的实测地面应按该段接触高度解释，不能被起点高度变成障碍");
        assert!(!surface.accepts([0.5,0.0,0.65],[1.5,0.0,0.65]),
            "相同回波对下层路线仍是机身高度内的障碍");
        let precise=RecentGuideOverlay::new(vec![(point,point)],0.5,0.01,&body).unwrap();
        assert!(!precise.accepts([0.5,0.0,1.05],[1.5,0.0,1.05]),
            "规划格距不能冒充源噪声或扩大接触高度容差");
        let volume=RecentGuideOverlay::new(vec![([1.0,0.0,1.05],[1.05,0.05,1.1])],0.05,0.05,&body).unwrap();
        assert!(!volume.accepts([0.5,0.0,1.05],[1.5,0.0,1.05]),
            "未分类占据体不能套用测量面接触解释");
    }
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
