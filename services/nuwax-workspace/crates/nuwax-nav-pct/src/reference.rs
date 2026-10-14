//! Request-local reference geometry. Unknown heights are carried coordinates,
//! never GroundLayers, observed evidence, or mutations of the shared tomogram.
use crate::{Tomogram, distance};
use std::sync::atomic::AtomicBool;
pub(crate) const UNKNOWN_COST: f64 = 4.0;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub(crate) enum Reference {
    Known(usize),
    Unknown { column: [i32; 2], height: u64 },
}
impl Tomogram {
    pub(crate) fn reference_inside_window(&self, xy: [f64; 2]) -> bool {
        self.window.is_none_or(|(center, radius)| (0..2).all(|axis|
            (xy[axis] - center[axis]).abs() + self.half_width + self.resolution * 0.5 <= radius + 1e-9))
    }
    pub(crate) fn reference_column(&self, reference: Reference) -> [i32; 2] {
        match reference {
            Reference::Known(id) => self.cells[id].column,
            Reference::Unknown { column, .. } => column,
        }
    }
    pub(crate) fn reference_position(&self, reference: Reference) -> [f64; 3] {
        match reference {
            Reference::Known(id) => self.position(id),
            Reference::Unknown { column, height } => [
                (f64::from(column[0]) + 0.5) * self.resolution,
                (f64::from(column[1]) + 0.5) * self.resolution,
                f64::from_bits(height),
            ],
        }
    }
    pub(crate) fn reference_cost(&self, reference: Reference) -> f64 {
        // Match the existing maximum missing-support-edge surcharge (1 + 3),
        // so absence is traversable without rewarding it over observed terrain.
        match reference { Reference::Known(id) => self.cells[id].cost, Reference::Unknown { .. } => UNKNOWN_COST }
    }
    fn reference_ceiling(&self, reference: Reference) -> f64 {
        match reference {
            Reference::Known(id) => self.cells[id].ceiling,
            Reference::Unknown { column, height } => self.occupancy.get(&column)
                .into_iter().flatten().filter(|band| band[0] > f64::from_bits(height))
                .map(|band| band[0]).fold(f64::INFINITY, f64::min),
        }
    }
    fn column_allowed(&self, column: [i32; 2]) -> bool {
        #[cfg(not(feature = "mls-adapter"))]
        let _ = column;
        #[cfg(feature = "mls-adapter")]
        if self.source.as_ref().is_some_and(|source|
            !source.navigation_column_allowed(column, self.resolution)) { return false; }
        // An empty occupancy vector alone is not forbidden/known-void evidence.
        true
    }
    fn unknown_at(&self, column: [i32; 2], ground: f64) -> Option<Reference> {
        if !self.column_allowed(column) { return None; }
        if let Some(bands) = self.occupancy.get(&column) {
            // Physical overlap is obstacle evidence independently of ground
            // classification. Below-body occupancy alone does not prove a cliff.
            if bands.iter().any(|band| band[1] > ground + 1e-6
                && band[0] < ground + self.body_height - 1e-6) { return None; }
        }
        // A lower *ground candidate* owns known-drop rejection. This includes
        // the explicit legacy geometry contract but never promotes raw, rejected
        // or unclassified occupancy to measured ground just to forbid unknown.
        if self.columns.get(&column).into_iter().flatten().any(|&id|
            self.cells[id].ground < ground - self.step_down.max(self.resolution * self.slope) - 1e-6) {
            return None;
        }
        let xy = column.map(|v| (f64::from(v) + 0.5) * self.resolution);
        if !self.unknown_footprint_clear(xy, ground) { return None; }
        Some(Reference::Unknown { column, height: if ground == 0.0 { 0.0_f64.to_bits() } else { ground.to_bits() } })
    }
    /// The existing yaw-free reference uses body half-width, not the full
    /// circumscribed footprint radius. Only actual occupancy/masks are dilated;
    /// unobserved neighboring ground never becomes a wall.
    fn unknown_footprint_clear(&self, xy: [f64; 2], ground: f64) -> bool {
        self.reference_footprint_clear(xy, ground, ground)
    }
    fn same_band_reference(&self, cell: &crate::Cell, ground: f64) -> bool {
        ground >= cell.ground_interval[0] - 1e-6
            && ground <= cell.ground_interval[1] + 1e-6
            && (cell.ground - ground).abs() <= self.source_quantum + 1e-6
    }
    fn reference_footprint_clear(&self, xy: [f64; 2], ground: f64, carried: f64) -> bool {
        let width = self.half_width;
        let lo = xy.map(|v| ((v - width) / self.resolution).floor() as i32);
        let hi = xy.map(|v| ((v + width) / self.resolution).floor() as i32);
        for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] {
            if crate::planning_cancelled() { return false; }
            let column = [x, y];
            let nearest: [f64; 2] = std::array::from_fn(|axis| xy[axis].clamp(
                f64::from(column[axis]) * self.resolution,
                (f64::from(column[axis]) + 1.0) * self.resolution));
            if (nearest[0] - xy[0]).hypot(nearest[1] - xy[1]) > width + 1e-9 { continue; }
            if !self.column_allowed(column) { return false; }
            for band in self.occupancy.get(&column).into_iter().flatten() {
                if band[1] <= carried + 1e-6 || band[0] >= carried + self.body_height - 1e-6 { continue; }
                // Same-layer contact bands are geometric floor evidence, not a
                // body collision just because the representative is inside them.
                let contact = self.columns.get(&column).into_iter().flatten().any(|&id| {
                    let cell = &self.cells[id];
                    ((cell.ground - ground).abs() <= self.step_up.max(self.step_down) + 1e-6
                        || self.same_band_reference(cell, ground))
                        && band[0] >= cell.ground_interval[0] - 1e-6
                        && band[1] <= cell.ground_interval[1] + 1e-6
                });
                if !contact { return false; }
            }
        }}
        true
    }
    /// Establish a coordinate reference once, without changing the factual layer.
    /// A missing center may associate only to the same physical source band in
    /// the existing footprint; it cannot snap to a nearby floor or invent a step.
    /// This is only coordinate alignment, not observed ground or motion clearance.
    pub fn calibrated_reference_height(&self, point: [f64; 3]) -> Option<f64> {
        let initial = self.reference_start(point)?;
        if let Reference::Known(id) = initial { return Some(self.cells[id].ground); }
        let xy = [point[0], point[1]];
        let lo = xy.map(|v| ((v - self.half_width) / self.resolution).floor() as i32);
        let hi = xy.map(|v| ((v + self.half_width) / self.resolution).floor() as i32);
        let mut best: Option<f64> = None;
        for x in lo[0]..=hi[0] { for y in lo[1]..=hi[1] {
            let column = [x,y];
            let nearest: [f64;2] = std::array::from_fn(|axis| xy[axis].clamp(
                f64::from(column[axis])*self.resolution,
                (f64::from(column[axis])+1.0)*self.resolution));
            if (nearest[0]-xy[0]).hypot(nearest[1]-xy[1]) > self.half_width + 1e-9 { continue; }
            for &id in self.columns.get(&column).into_iter().flatten() {
                let cell = &self.cells[id];
                if self.same_band_reference(cell,point[2])
                    && best.is_none_or(|old|(cell.ground-point[2]).abs()<(old-point[2]).abs()) {
                    best = Some(cell.ground);
                }
            }
        }}
        Some(best.unwrap_or(point[2]))
    }
    /// Planar execution geometry for legacy callers without a body envelope.
    /// Carries the current calibrated height bias through unknown space and adds
    /// only factual terrain deltas. Every sample checks the existing circular
    /// footprint and vertical body interval, including samples on known ground.
    /// Recent occupancy must still be checked against the caller's current source.
    pub fn project_reference_motion(&self, from: [f64;3], to_xy: [f64;2],
        boundary: &[[f64;2]]) -> Result<[f64;3],String> {
        crate::cancellation::checkpoint()?;
        if !from.into_iter().chain(to_xy).all(f64::is_finite)
            || boundary.iter().flatten().any(|v|!v.is_finite()) {
            return Err("平面运动参考坐标无效".into());
        }
        if !self.reference_inside_window([from[0],from[1]]) || !self.reference_inside_window(to_xy) {
            return Err("SearchDomainLimited: 平面运动超出本次地图物化范围".into());
        }
        if !crate::segment_inside_boundary([from[0],from[1]],to_xy,boundary) {
            return Err("平面运动超出任务边界".into());
        }
        let base = self.calibrated_reference_height(from);
        crate::cancellation::checkpoint()?;
        let base = base.ok_or("平面运动起点存在已知障碍或不兼容地形")?;
        let bias = from[2] - base;
        let mut reference = self.reference_start([from[0],from[1],base])
            .ok_or("平面运动起点参考不可用")?;
        let steps = ((to_xy[0]-from[0]).hypot(to_xy[1]-from[1]) / (self.resolution*0.4)).ceil().max(1.0) as usize;
        let mut endpoint = from;
        for i in 0..=steps {
            crate::cancellation::checkpoint()?;
            let t = i as f64 / steps as f64;
            let xy = [from[0]+t*(to_xy[0]-from[0]),from[1]+t*(to_xy[1]-from[1])];
            let next = self.reference_neighbor(reference,self.point_column([xy[0],xy[1],base]));
            crate::cancellation::checkpoint()?;
            reference = next.ok_or("平面运动穿过已知障碍或不兼容地形")?;
            let ground = self.reference_position(reference)[2];
            #[cfg(feature = "mls-adapter")]
            let previous = endpoint;
            endpoint = [xy[0],xy[1],ground+bias];
            if !self.reference_footprint_clear(xy,ground,endpoint[2]) {
                crate::cancellation::checkpoint()?;
                return Err("平面运动外形与实际占据或禁行区域相交".into());
            }
            #[cfg(feature = "mls-adapter")]
            if i != 0 {
                if let Some(source) = self.source.as_ref().filter(|source|source.body_envelope().is_none()) {
                    source.validate_known_reference_motion(previous,endpoint).map_err(|error|error.to_string())?;
                }
            }
        }
        Ok(endpoint)
    }
    pub(crate) fn reference_start(&self, point: [f64; 3]) -> Option<Reference> {
        let column = self.point_column(point);
        if !self.column_allowed(column) { return None; }
        self.cell_at(point).map(Reference::Known).or_else(|| self.unknown_at(column, point[2]))
    }
    pub(crate) fn point_column(&self, point: [f64; 3]) -> [i32; 2] {
        [(point[0] / self.resolution).floor() as i32, (point[1] / self.resolution).floor() as i32]
    }
    fn references_compatible(&self, a: Reference, b: Reference, span: f64) -> bool {
        let a_ground = self.reference_position(a)[2];
        let b_ground = self.reference_position(b)[2];
        let rise = b_ground - a_ground;
        let step = if rise >= 0.0 { self.step_up } else { self.step_down };
        rise.abs() <= step.max(span * self.slope) + 1e-6
            && self.reference_ceiling(a).min(self.reference_ceiling(b))
                - a_ground.max(b_ground) + 1e-6 >= self.body_height
    }
    fn reference_in(&self, from: Reference, column: [i32; 2], span: f64) -> Option<Reference> {
        if !self.column_allowed(column) { return None; }
        let ground = self.reference_position(from)[2];
        self.columns.get(&column).into_iter().flatten().copied()
            .map(Reference::Known)
            .filter(|&to| self.references_compatible(from, to, span))
            .min_by(|&a, &b| (self.reference_position(a)[2] - ground).abs()
                .total_cmp(&(self.reference_position(b)[2] - ground).abs()))
            .or_else(|| self.unknown_at(column, ground)
                .filter(|&to| self.references_compatible(from, to, span)))
    }
    pub(crate) fn reference_neighbor(&self, from: Reference, column: [i32; 2]) -> Option<Reference> {
        let old = self.reference_column(from);
        let dx = i64::from(column[0]) - i64::from(old[0]);
        let dy = i64::from(column[1]) - i64::from(old[1]);
        if dx.abs() > 1 || dy.abs() > 1 { return None; }
        if dx == 0 && dy == 0 { return Some(from); }
        let span = self.resolution * ((dx * dx + dy * dy) as f64).sqrt();
        let next = self.reference_in(from, column, span)?;
        if dx != 0 && dy != 0 {
            let x = self.reference_in(from, [column[0], old[1]], self.resolution)?;
            let y = self.reference_in(from, [old[0], column[1]], self.resolution)?;
            if !self.references_compatible(x, next, self.resolution)
                || !self.references_compatible(y, next, self.resolution) { return None; }
        }
        Some(next)
    }
    /// Every segment uses the same height-specific admission and corner rules as
    /// expansion. Unknown does not interpolate towards the requested endpoint z.
    pub(crate) fn reference_segment(&self, a: [f64; 3], b: [f64; 3], boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>) -> Option<f64> {
        self.reference_segment_evidence(a, b, boundary, cancelled).map(|(cost, _)| cost)
    }
    pub(crate) fn reference_segment_evidence(&self, a: [f64; 3], b: [f64; 3], boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>) -> Option<(f64, bool)> {
        if !a.into_iter().chain(b).all(f64::is_finite)
            || !self.reference_inside_window([a[0],a[1]])
            || !self.reference_inside_window([b[0],b[1]])
            || !crate::segment_inside_boundary([a[0], a[1]], [b[0], b[1]], boundary) { return None; }
        let mut reference = self.reference_start(a)?;
        let mut observed = matches!(reference, Reference::Known(_));
        let length = distance(a, b);
        let steps = ((a[0] - b[0]).hypot(a[1] - b[1]) / (self.resolution * 0.4)).ceil().max(1.0) as usize;
        let mut cost = 0.0;
        for i in 1..=steps {
            crate::search::checkpoint(cancelled).ok()?;
            let t = i as f64 / steps as f64;
            let p = std::array::from_fn(|axis| a[axis] + t * (b[axis] - a[axis]));
            reference = self.reference_neighbor(reference, self.point_column(p))?;
            observed |= matches!(reference, Reference::Known(_));
            let z = self.reference_position(reference)[2];
            if matches!(reference, Reference::Unknown { .. })
                && !self.unknown_footprint_clear([p[0], p[1]], z) { return None; }
            // A shortcut may approximate a known step/ramp, but it cannot invent
            // a rising/falling plane while passing through absent observations.
            let tolerance = match reference {
                Reference::Known(_) => self.step_up.max(self.step_down).max(self.resolution) * 1.5 + 1e-6,
                Reference::Unknown { .. } => {
                    let col = self.reference_column(reference);
                    let end = self.point_column(b);
                    if (i64::from(col[0]) - i64::from(end[0])).abs() <= 1
                        && (i64::from(col[1]) - i64::from(end[1])).abs() <= 1
                        && self.cell_at(b).is_some() {
                        self.step_up.max(self.step_down).max(self.resolution) + 1e-6
                    } else { 1e-6 }
                },
            };
            if (z - p[2]).abs() > tolerance { return None; }
            cost += self.reference_cost(reference) * length / steps as f64;
        }
        Some((cost, observed))
    }
}
