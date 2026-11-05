//! 在线去动态后端：单次积分结果只随本帧交付，不进入 PGO 历史。
use ahash::{AHashMap, AHashSet};
use nalgebra::{Isometry3, Point3};
use nuwax_dufomap_native::{DufoMap, Params};

#[path = "pgo_free_evidence.rs"]
pub(crate) mod free_evidence;

pub(crate) const BACKEND_VAR: &str = "NUWAX_GEOMETRY_DYNAMIC_BACKEND";
pub(crate) fn parse_backend(value: Option<&str>) -> Result<(), String> {
    match value.map(str::trim).filter(|v| !v.is_empty()) {
        None | Some("dufomap") => Ok(()),
        Some(value) => Err(format!("{BACKEND_VAR} 取值非法：{value:?}，应为 dufomap")),
    }
}
pub(crate) fn online_params() -> Params {
    Params {
        resolution: 0.15,
        inflate_unknown: 2,
        inflate_unknown_compensation: false,
        inflate_hits_dist: 0.30,
        sliding_window_size: 30,
        ..Params::default()
    }
}

/// 仅测试程序可启用旧包围盒路径，生产构建不读取这个比较开关。
pub(crate) fn reference_revoke() -> bool {
    #[cfg(test)]
    {
        std::env::var_os("NUWAX_G3_REVOKE_REFERENCE").is_some()
    }
    #[cfg(not(test))]
    {
        false
    }
}

#[cfg(test)]
pub(crate) fn test_free_cells(
    resolution: f64,
    cells: impl IntoIterator<Item = ([i32; 3], u32)>,
    map_from_evidence: Isometry3<f64>,
) -> MappedFrame {
    MappedFrame::Dufo(DufoFrame {
        resolution,
        evidence_from_map: map_from_evidence.inverse(),
        free: cells.into_iter().map(|cell| (cell, ())).collect(),
        dynamic_points: AHashSet::new(),
    })
}

#[derive(Clone, Debug, Default)]
pub(crate) struct OnlineDufo {
    map: Option<DufoMap>,
    free_revision: u64,
    free_changes: free_evidence::FreeChangeJournal,
}
impl OnlineDufo {
    pub(crate) fn free_revision(&self) -> u64 {
        self.free_revision
    }

    /// 调用者持地图拥有方锁时，树与完整变化增量来自同一个不可变状态。
    pub(crate) fn snapshot_since(
        &self,
        revision: u64,
    ) -> (Self, Option<Vec<nuwax_dufomap_native::FreeCell>>) {
        (
            self.clone(),
            self.free_changes.since(revision, self.free_revision),
        )
    }

    #[cfg(test)]
    pub(crate) fn free_change_payload_stats(&self) -> (usize, usize) {
        self.free_changes.payload_stats()
    }

    /// 只查询冻结终态，不再以抽帧扫描重新积分另一套时域证据。
    pub(crate) fn terminal_frame(
        &self,
        evidence_points: &[Point3<f64>],
        corrected_points: &[Point3<f64>],
    ) -> Result<MappedFrame, String> {
        let xyz: Vec<_> = evidence_points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        let flags = match &self.map {
            Some(map) => map.seen_free(&xyz)?,
            None => vec![false; xyz.len()],
        };
        let mut frame = MappedFrame::unfiltered();
        let MappedFrame::Dufo(dufo) = &mut frame;
        dufo.dynamic_points = corrected_points
            .iter()
            .zip(flags)
            .filter_map(|(p, flag)| flag.then_some([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]))
            .collect();
        Ok(frame)
    }
    #[cfg(test)]
    pub(crate) fn memory_bytes(&self) -> usize {
        self.map.as_ref().map_or(0, DufoMap::memory_bytes)
    }
    pub(crate) fn observe(
        &mut self,
        points: &[Point3<f64>],
        origin: [f64; 3],
        stamp: u64,
        map_from_evidence: Isometry3<f64>,
    ) -> Result<(MappedFrame, Vec<u32>), String> {
        let mut free = Vec::new();
        let params = online_params();
        let map = match &mut self.map {
            Some(map) => map,
            slot @ None => slot.insert(DufoMap::new(&params)?),
        };
        let xyz: Vec<_> = points
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        if stamp > 0 {
            free = map.insert_with_free_cells(&xyz, origin.map(|v| v as f32))?;
        }
        if !free.is_empty() {
            self.free_revision = self
                .free_revision
                .checked_add(1)
                .ok_or("DUFOMap自由证据修订溢出")?;
            self.free_changes.record(self.free_revision, &free);
        }
        let flags = map.seen_free(&xyz)?;
        let indices = flags
            .iter()
            .enumerate()
            .filter_map(|(i, dynamic)| (!dynamic).then_some(i as u32))
            .collect();
        let dynamic_points = points
            .iter()
            .zip(flags)
            .filter_map(|(p, flag)| {
                flag.then(|| {
                    let p = map_from_evidence * p;
                    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
                })
            })
            .collect();
        Ok((
            MappedFrame::Dufo(DufoFrame {
                resolution: params.resolution,
                evidence_from_map: map_from_evidence.inverse(),
                free: free.into_iter().map(|c| ((c.key, c.depth), ())).collect(),
                dynamic_points,
            }),
            indices,
        ))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum MappedFrame {
    Dufo(DufoFrame),
}
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DufoFrame {
    resolution: f64,
    evidence_from_map: Isometry3<f64>,
    free: AHashMap<([i32; 3], u32), ()>,
    dynamic_points: AHashSet<[u64; 3]>,
}
impl MappedFrame {
    pub(crate) fn clear_bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let Self::Dufo(frame) = self;
        if frame.free.is_empty() {
            return None;
        }
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        let transform = frame.evidence_from_map.inverse();
        for &(key, depth) in frame.free.keys() {
            for corner in 0..8 {
                let p = transform
                    * Point3::from(std::array::from_fn::<_, 3, _>(|axis| {
                        (f64::from(key[axis])
                            + f64::from((corner >> axis) & 1) * (1u32 << depth) as f64)
                            * frame.resolution
                    }));
                for axis in 0..3 {
                    low[axis] = low[axis].min(p[axis] - 1e-6);
                    high[axis] = high[axis].max(p[axis] + 1e-6);
                }
            }
        }
        Some((low, high))
    }

    /// 每个新增空单元各自产生候选范围，不合并跨越无关地图的空隙。
    pub(crate) fn clear_regions(&self) -> Option<impl Iterator<Item = ([f64; 3], [f64; 3])> + '_> {
        let Self::Dufo(frame) = self;
        let transform = frame.evidence_from_map.inverse();
        Some(frame.free.keys().map(move |&(key, depth)| {
            let step = f64::from(1u32 << depth) * frame.resolution;
            let evidence_low = key.map(|v| f64::from(v) * frame.resolution);
            let evidence_high = evidence_low.map(|v| v + step);
            // clears先把证据坐标降为f32；范围覆盖该舍入的原像，避免细分范围遗漏边界原点。
            let pad: [f64; 3] = std::array::from_fn(|axis| {
                evidence_low[axis].abs().max(evidence_high[axis].abs()) * f64::from(f32::EPSILON)
                    + 1e-6
            });
            let mut low = [f64::INFINITY; 3];
            let mut high = [f64::NEG_INFINITY; 3];
            for corner in 0..8 {
                let point = transform
                    * Point3::from(std::array::from_fn::<_, 3, _>(|axis| {
                        if corner & (1 << axis) == 0 {
                            evidence_low[axis] - pad[axis]
                        } else {
                            evidence_high[axis] + pad[axis]
                        }
                    }));
                for axis in 0..3 {
                    low[axis] = low[axis].min(point[axis]);
                    high[axis] = high[axis].max(point[axis]);
                }
            }
            (low, high)
        }))
    }

    /// 有规则格键的表示可直接枚举相交键，查询次数只取决于新增空单元。
    pub(crate) fn candidate_grid_keys(
        &self,
        size: f64,
        mut present: impl FnMut((i32, i32, i32)) -> bool,
    ) -> Option<AHashSet<(i32, i32, i32)>> {
        let regions = self.clear_regions()?;
        let mut candidates = AHashSet::new();
        for (low, high) in regions {
            let low = low.map(|v| (v / size).floor());
            let high = high.map(|v| (v / size).floor());
            if (0..3)
                .any(|axis| high[axis] < f64::from(i32::MIN) || low[axis] > f64::from(i32::MAX))
            {
                continue;
            }
            let low = low.map(|v| v.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32);
            let high = high.map(|v| v.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32);
            for x in low[0]..=high[0] {
                for y in low[1]..=high[1] {
                    for z in low[2]..=high[2] {
                        let key = (x, y, z);
                        if !candidates.contains(&key) && present(key) {
                            candidates.insert(key);
                        }
                    }
                }
            }
        }
        Some(candidates)
    }

    #[cfg(test)]
    pub(crate) fn free_cell_count(&self) -> usize {
        match self {
            Self::Dufo(frame) => frame.free.len(),
        }
    }
    pub(crate) fn unfiltered() -> Self {
        Self::Dufo(DufoFrame {
            resolution: online_params().resolution,
            evidence_from_map: Isometry3::identity(),
            free: AHashMap::new(),
            dynamic_points: AHashSet::new(),
        })
    }
    pub(crate) fn bind_points(&mut self, points: &[Point3<f64>], static_indices: &[u32]) {
        let Self::Dufo(frame) = self;
        let keep: AHashSet<_> = static_indices.iter().copied().collect();
        frame.dynamic_points = points
            .iter()
            .enumerate()
            .filter_map(|(i, p)| {
                (!keep.contains(&(i as u32))).then_some([
                    p.x.to_bits(),
                    p.y.to_bits(),
                    p.z.to_bits(),
                ])
            })
            .collect();
    }

    pub(crate) fn is_dynamic(&self, p: [f64; 3]) -> bool {
        match self {
            Self::Dufo(f) => f.dynamic_points.contains(&p.map(f64::to_bits)),
        }
    }
    pub(crate) fn has_clears(&self) -> bool {
        match self {
            Self::Dufo(f) => !f.free.is_empty(),
        }
    }
    pub(crate) fn clears(&self, p: [f64; 3]) -> bool {
        match self {
            Self::Dufo(f) => {
                let p = f.evidence_from_map * Point3::from(p);
                let key =
                    [p.x, p.y, p.z].map(|v| ((v as f32 as f64) / f.resolution).floor() as i32);
                // 上游自由证据可能把八个叶子合并到父单元；深度也属于身份。
                (0..=1).any(|depth| {
                    let step = 1 << depth;
                    f.free
                        .contains_key(&(key.map(|v| v.div_euclid(step) * step), depth))
                })
            }
        }
    }
    pub(crate) fn potential_window_clear_cells(
        &self,
        cells: &AHashMap<crate::dynamic_scene::Cell, u32>,
    ) -> Option<AHashSet<crate::dynamic_scene::Cell>> {
        #[cfg(test)]
        if reference_revoke() {
            return self.potential_window_clear_cells_reference(cells);
        }
        match self {
            Self::Dufo(_) => self.candidate_grid_keys(crate::dynamic_scene::CELL_SIZE, |key| {
                cells.contains_key(&key)
            }),
        }
    }
    #[cfg(test)]
    fn potential_window_clear_cells_reference(
        &self,
        cells: &AHashMap<crate::dynamic_scene::Cell, u32>,
    ) -> Option<AHashSet<crate::dynamic_scene::Cell>> {
        match self {
            // 先按现有有界窗口格筛候选，具体点仍由 clears 判断。
            Self::Dufo(f) => {
                let mut candidates = AHashSet::new();
                let transform = f.evidence_from_map.inverse();
                for &(key, depth) in f.free.keys() {
                    let mut low = [f64::INFINITY; 3];
                    let mut high = [f64::NEG_INFINITY; 3];
                    for corner in 0..8 {
                        let p = Point3::from(std::array::from_fn::<_, 3, _>(|axis| {
                            (f64::from(key[axis])
                                + f64::from((corner >> axis) & 1) * (1u32 << depth) as f64)
                                * f.resolution
                        }));
                        let p = transform * p;
                        for axis in 0..3 {
                            low[axis] = low[axis].min(p[axis] - 1e-6);
                            high[axis] = high[axis].max(p[axis] + 1e-6);
                        }
                    }
                    let (Some(a), Some(b)) = (
                        crate::dynamic_scene::cell(low),
                        crate::dynamic_scene::cell(high),
                    ) else {
                        return None;
                    };
                    for x in a.0..=b.0 {
                        for y in a.1..=b.1 {
                            for z in a.2..=b.2 {
                                if cells.contains_key(&(x, y, z)) {
                                    candidates.insert((x, y, z));
                                }
                            }
                        }
                    }
                }
                Some(candidates)
            }
        }
    }
    pub(crate) fn encode(&self) -> Result<Vec<u8>, String> {
        match self {
            Self::Dufo(f) => {
                let pose: [f64; 16] = f
                    .evidence_from_map
                    .to_homogeneous()
                    .as_slice()
                    .try_into()
                    .unwrap();
                let mut bytes = vec![1];
                bytes.extend(
                    serde_json::to_vec(&(
                        f.resolution,
                        pose,
                        f.free.keys().copied().collect::<Vec<_>>(),
                        f.dynamic_points.iter().copied().collect::<Vec<_>>(),
                    ))
                    .map_err(|e| e.to_string())?,
                );
                Ok(bytes)
            }
        }
    }
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, String> {
        match bytes.split_first() {
            Some((1, body)) => {
                let (resolution, m, free, points): (
                    f64,
                    [f64; 16],
                    Vec<([i32; 3], u32)>,
                    Vec<[u64; 3]>,
                ) = serde_json::from_slice(body).map_err(|e| e.to_string())?;
                if !resolution.is_finite()
                    || resolution <= 0.0
                    || free.iter().any(|(_, depth)| *depth > 1)
                    || m.iter().any(|v| !v.is_finite())
                {
                    return Err("DUFOMap 帧参数非法".into());
                }
                let rotation = nalgebra::Matrix3::from_column_slice(&[
                    m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10],
                ]);
                Ok(Self::Dufo(DufoFrame {
                    resolution,
                    evidence_from_map: Isometry3::from_parts(
                        nalgebra::Translation3::new(m[12], m[13], m[14]),
                        nalgebra::UnitQuaternion::from_matrix(&rotation),
                    ),
                    free: free.into_iter().map(|c| (c, ())).collect(),
                    dynamic_points: points.into_iter().collect(),
                }))
            }
            _ => Err("去动态帧编码未知".into()),
        }
    }
}

#[cfg(test)]
#[path = "dynamic_backend_tests.rs"]
mod tests;
