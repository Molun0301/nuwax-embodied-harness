//! 在线去动态后端：单次积分结果只随本帧交付，不进入 PGO 历史。
use ahash::{AHashMap, AHashSet};
use nalgebra::{Isometry3, Point3};
use nuwax_dufomap_native::{DufoMap, Params};
use std::sync::OnceLock;

pub(crate) const BACKEND_VAR: &str = "NUWAX_GEOMETRY_DYNAMIC_BACKEND";
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Backend {
    DufoMap,
    Legacy,
}
static CONFIGURED: OnceLock<Backend> = OnceLock::new();
pub(crate) fn parse_backend(value: Option<&str>) -> Result<Backend, String> {
    match value.map(str::trim).filter(|v| !v.is_empty()) {
        None | Some("dufomap") => Ok(Backend::DufoMap),
        Some("legacy") => Ok(Backend::Legacy),
        Some(value) => Err(format!(
            "{BACKEND_VAR} 取值非法：{value:?}，应为 dufomap 或 legacy"
        )),
    }
}
pub(crate) fn configure(backend: Backend) -> Result<(), String> {
    CONFIGURED
        .set(backend)
        .map_err(|_| "在线去动态后端已经配置".into())
}
pub(crate) fn backend() -> Backend {
    #[cfg(test)]
    if let Some(value) = TEST_BACKEND.with(|slot| slot.get()) {
        return value;
    }
    // 测试以进程环境选择整套后端，生产启动只解析一次。
    CONFIGURED.get().copied().unwrap_or_else(|| {
        parse_backend(std::env::var(BACKEND_VAR).ok().as_deref()).expect("在线去动态后端非法")
    })
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
    { std::env::var_os("NUWAX_G3_REVOKE_REFERENCE").is_some() }
    #[cfg(not(test))]
    { false }
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
}
impl OnlineDufo {
    #[cfg(test)]
    pub(crate) fn memory_bytes(&self) -> usize { self.map.as_ref().map_or(0, DufoMap::memory_bytes) }
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
    Legacy(crate::dynamic_scene::MappedFrameDecision),
    Dufo(DufoFrame),
}
impl From<crate::dynamic_scene::MappedFrameDecision> for MappedFrame {
    fn from(value: crate::dynamic_scene::MappedFrameDecision) -> Self {
        Self::Legacy(value)
    }
}
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DufoFrame {
    resolution: f64,
    evidence_from_map: Isometry3<f64>,
    free: AHashMap<([i32; 3], u32), ()>,
    dynamic_points: AHashSet<[u64; 3]>,
}
impl MappedFrame {
    pub(crate) fn is_dufo(&self) -> bool {
        matches!(self, Self::Dufo(_))
    }
    pub(crate) fn clear_bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let Self::Dufo(frame) = self else {
            return None;
        };
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
    pub(crate) fn clear_regions(
        &self,
    ) -> Option<impl Iterator<Item = ([f64; 3], [f64; 3])> + '_> {
        let Self::Dufo(frame) = self else { return None; };
        let transform = frame.evidence_from_map.inverse();
        Some(frame.free.keys().map(move |&(key, depth)| {
            let step = f64::from(1u32 << depth) * frame.resolution;
            let evidence_low = key.map(|v| f64::from(v) * frame.resolution);
            let evidence_high = evidence_low.map(|v| v + step);
            // clears先把证据坐标降为f32；范围覆盖该舍入的原像，避免细分范围遗漏边界原点。
            let pad: [f64; 3] = std::array::from_fn(|axis| {
                evidence_low[axis].abs().max(evidence_high[axis].abs())
                    * f64::from(f32::EPSILON) + 1e-6
            });
            let mut low = [f64::INFINITY; 3];
            let mut high = [f64::NEG_INFINITY; 3];
            for corner in 0..8 {
                let point = transform * Point3::from(std::array::from_fn::<_, 3, _>(|axis| {
                    if corner & (1 << axis) == 0 { evidence_low[axis] - pad[axis] }
                    else { evidence_high[axis] + pad[axis] }
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
            if (0..3).any(|axis| high[axis] < f64::from(i32::MIN)
                || low[axis] > f64::from(i32::MAX)) { continue; }
            let low = low.map(|v| v.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32);
            let high = high.map(|v| v.clamp(f64::from(i32::MIN), f64::from(i32::MAX)) as i32);
            for x in low[0]..=high[0] {
                for y in low[1]..=high[1] {
                    for z in low[2]..=high[2] {
                        let key = (x, y, z);
                        if !candidates.contains(&key) && present(key) { candidates.insert(key); }
                    }
                }
            }
        }
        Some(candidates)
    }

    #[cfg(test)]
    pub(crate) fn free_cell_count(&self) -> usize {
        match self { Self::Dufo(frame) => frame.free.len(), Self::Legacy(_) => 0 }
    }
    pub(crate) fn unfiltered() -> Self {
        Self::Dufo(DufoFrame {
            resolution: online_params().resolution,
            evidence_from_map: Isometry3::identity(),
            free: AHashMap::new(),
            dynamic_points: AHashSet::new(),
        })
    }
    pub(crate) fn new(
        decision: std::sync::Arc<crate::dynamic_scene::FrameDecision>,
        transform: Isometry3<f64>,
        points: &[Point3<f64>],
        first: usize,
    ) -> Result<Self, String> {
        crate::dynamic_scene::MappedFrameDecision::new(decision, transform, points, first)
            .map(Self::Legacy)
    }
    pub(crate) fn bind_points(&mut self, points: &[Point3<f64>], static_indices: &[u32]) {
        if let Self::Dufo(frame) = self {
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
    }
    pub(crate) fn is_dynamic(&self, p: [f64; 3]) -> bool {
        match self {
            Self::Legacy(f) => f.is_dynamic(p),
            Self::Dufo(f) => f.dynamic_points.contains(&p.map(f64::to_bits)),
        }
    }
    pub(crate) fn has_clears(&self) -> bool {
        match self {
            Self::Legacy(f) => f.has_clears(),
            Self::Dufo(f) => !f.free.is_empty(),
        }
    }
    pub(crate) fn clears(&self, p: [f64; 3]) -> bool {
        match self {
            Self::Legacy(f) => f.clears(p),
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
        if reference_revoke() { return self.potential_window_clear_cells_reference(cells); }
        match self {
            Self::Legacy(frame) => frame.potential_window_clear_cells(cells),
            Self::Dufo(_) => self.candidate_grid_keys(crate::dynamic_scene::CELL_SIZE,
                |key| cells.contains_key(&key)),
        }
    }
    #[cfg(test)]
    fn potential_window_clear_cells_reference(
        &self,
        cells: &AHashMap<crate::dynamic_scene::Cell, u32>,
    ) -> Option<AHashSet<crate::dynamic_scene::Cell>> {
        match self {
            Self::Legacy(f) => f.potential_window_clear_cells(cells),
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
            Self::Legacy(f) => {
                let mut bytes = vec![0];
                bytes.extend(f.encode()?);
                Ok(bytes)
            }
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
            Some((0, body)) => {
                crate::dynamic_scene::MappedFrameDecision::decode(body).map(Self::Legacy)
            }
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
thread_local! { static TEST_BACKEND: std::cell::Cell<Option<Backend>> = const { std::cell::Cell::new(None) }; }
#[cfg(test)]
pub(crate) struct TestBackend(Option<Backend>);
#[cfg(test)]
pub(crate) fn test_backend(value: Backend) -> TestBackend {
    TestBackend(TEST_BACKEND.with(|slot| slot.replace(Some(value))))
}
#[cfg(test)]
impl Drop for TestBackend {
    fn drop(&mut self) {
        TEST_BACKEND.with(|slot| slot.set(self.0));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sparse_clear_regions_probe_only_local_grid_keys_on_large_maps() {
        let frame = test_free_cells(0.15, [([0, 0, 0], 0), ([6000, 0, 0], 0)], Isometry3::identity());
        let expected: AHashSet<_> = [(1, 1, 1), (18001, 1, 1)].into_iter().collect();
        let mut small_queries = 0;
        let small = frame.candidate_grid_keys(0.05, |key| {
            small_queries += 1;
            expected.contains(&key)
        }).unwrap();
        let mut large = expected.clone();
        large.extend((0..100_000).map(|i| (1_000_000 + i, 17, 19)));
        let mut large_queries = 0;
        let actual = frame.candidate_grid_keys(0.05, |key| {
            large_queries += 1;
            large.contains(&key)
        }).unwrap();
        assert_eq!(small, expected);
        assert_eq!(actual, expected);
        assert_eq!(small_queries, large_queries, "远处历史体素不能增加候选格查询次数");
        assert!(small_queries < 400, "两端空单元之间的900米不应被枚举");
        let empty = test_free_cells(0.15, [], Isometry3::identity());
        assert!(empty.candidate_grid_keys(0.05, |_| panic!("无新增空单元不应查询地图")).unwrap().is_empty());
    }

    #[test]
    fn transformed_regions_cover_the_f32_rounding_preimage_at_negative_coordinates() {
        let transform = Isometry3::from_parts(nalgebra::Translation3::new(-1.7, 21.3, 0.4),
            nalgebra::UnitQuaternion::from_euler_angles(0.04, -0.07, 0.7));
        let frame = test_free_cells(0.15, [([-500_000, 100_000, -20], 0)], transform);
        // 这个双精度证据坐标略低于单元面，但f32舍入后实际落在本次新增空单元内。
        let point = transform * Point3::new(-75_000.001, 15_000.075, -2.925);
        let xyz = [point.x, point.y, point.z];
        assert!(frame.clears(xyz));
        let key = crate::dynamic_scene::cell(xyz).unwrap();
        let candidates = frame.candidate_grid_keys(crate::dynamic_scene::CELL_SIZE, |candidate| candidate == key).unwrap();
        assert!(candidates.contains(&key), "空间定位必须覆盖精确clears接受的舍入原像");
        assert!(frame.clear_regions().unwrap().any(|(low, high)|
            (0..3).all(|axis| xyz[axis] >= low[axis] && xyz[axis] <= high[axis])));
    }

    #[test]
    fn corrected_frame_and_sidecar_keep_exact_static_points_and_free_cell_centers() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        let transform = Isometry3::from_parts(nalgebra::Translation3::new(-12.31, 2.13, 0.17),
            nalgebra::UnitQuaternion::from_euler_angles(0.07, -0.03, 0.41));
        let mut map = OnlineDufo::default();
        let mut checked = 0;
        for index in 1..=12 {
            let (points, _) = corridor((index < 5).then_some(0.0), (index % 3) as f64);
            let (mut frame, indices) = map.observe(&points, ORIGIN, index, transform).unwrap();
            let corrected: Vec<_> = points.iter().map(|p| transform * p).collect();
            frame.bind_points(&corrected, &indices);
            let decoded = MappedFrame::decode(&frame.encode().unwrap()).unwrap();
            let keep: AHashSet<_> = indices.iter().copied().collect();
            for (i, p) in corrected.iter().enumerate() {
                assert_eq!(decoded.is_dynamic([p.x,p.y,p.z]), !keep.contains(&(i as u32)));
            }
            let MappedFrame::Dufo(dufo) = &frame else { unreachable!() };
            let mut cells = AHashMap::new();
            for &(key, depth) in dufo.free.keys() {
                let center = transform * Point3::from(key.map(|v| (v as f64 + (1u32 << depth) as f64 * 0.5) * dufo.resolution));
                let p = [center.x, center.y, center.z];
                assert!(frame.clears(p) && decoded.clears(p), "校正及跨进程编码必须保留历史空单元身份");
                let cell = crate::dynamic_scene::cell(p).unwrap();
                cells.insert(cell, 1);
                checked += 1;
            }
            // 按生产入口每帧查询一次，仍逐一检查全部原空单元，避免把空间查询重复为平方级。
            let candidates = decoded.potential_window_clear_cells(&cells).unwrap();
            for cell in cells.keys() { assert!(candidates.contains(cell)); }
        }
        assert!(checked > 0, "用真实新增空体素检验校正坐标，不能空集合通过");
    }
}
