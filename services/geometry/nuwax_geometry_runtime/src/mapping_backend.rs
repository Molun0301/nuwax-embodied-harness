//! 当前地图的体素与射线数值内核；输入与检查点由 Mapper 串行拥有。
use nalgebra::Point3;
use nuwax_mapping::cost_mapper::{CostMapper, OccupancyConfig, PointCloud2Like};
use nuwax_mapping::OccupancyGrid;
#[cfg(test)]
use nuwax_mapping_raytracing::voxel_ray_tracer::iter_global_points;
use nuwax_mapping_raytracing::voxel_ray_tracer::{
    append_voxel_map_checkpoint, decode_voxel_map_checkpoint_for_config, try_update_map,
    voxel_map_checkpoint_encoded_len, Config, PreparedRayUpdate, VoxelMap,
    VOXEL_MAP_CHECKPOINT_ENTRY_BYTES, VOXEL_MAP_CHECKPOINT_HEADER_BYTES,
};
#[cfg(test)]
use std::time::Instant;
use std::{collections::VecDeque, env, fmt::Display, mem::size_of, str::FromStr};
pub(crate) const MAPPING_CANDIDATE_INTERVAL_NS: u64 = 33_333_333;
pub(crate) const MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION: u32 = 1;
pub(crate) const MAX_MAPPING_BACKEND_CHECKPOINT_BYTES: usize = 64 * 1024 * 1024;
const MAPPING_BACKEND_CHECKPOINT_MAGIC: [u8; 8] = *b"NWMBCKP\0";
const MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES: usize = 16;
const MAPPING_BACKEND_VOXEL_METADATA_BYTES: usize = 4 * size_of::<u64>();
const MAPPING_BACKEND_VOXEL_HEADER_BYTES: usize =
    MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + MAPPING_BACKEND_VOXEL_METADATA_BYTES;
const MAPPING_BACKEND_VOXEL_POINT_BYTES: usize = 3 * size_of::<f64>();
const MAX_RETAINED_VOXEL_POINTS: usize = (MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
    - MAPPING_BACKEND_VOXEL_HEADER_BYTES)
    / MAPPING_BACKEND_VOXEL_POINT_BYTES;
const MAPPING_BACKEND_RAY_CONFIG_BYTES: usize = 12 * size_of::<u32>();
const MAPPING_BACKEND_RAY_METADATA_BYTES: usize = 4 * size_of::<u64>();
const MAPPING_BACKEND_RAY_HEADER_BYTES: usize = MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES
    + MAPPING_BACKEND_RAY_CONFIG_BYTES
    + MAPPING_BACKEND_RAY_METADATA_BYTES;
const MAPPING_BACKEND_OCCUPANCY_FIXED_BYTES: usize = 2 * size_of::<u32>()
    + size_of::<f64>()
    + 3 * size_of::<f64>()
    + 4 * size_of::<f64>()
    + size_of::<f64>()
    + size_of::<u32>()
    + size_of::<u64>();
const MAPPING_BACKEND_KIND_VOXEL: u8 = 0;
const MAPPING_BACKEND_KIND_RAYTRACING: u8 = 1;

#[derive(Clone, Copy, PartialEq)]
struct RayConfigCheckpoint {
    voxel_size: f32,
    max_range: f32,
    ray_subsample: u32,
    shadow_depth: f32,
    grace_depth: f32,
    min_health: i32,
    max_health: i32,
    graze_cos: f32,
    recency_window: u32,
    emit_every: u32,
    global_emit_every: u32,
    region_percentile: f32,
}

pub(crate) const RAY_VOXEL_SIZE_ENV: &str = "NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE";
pub(crate) const RAY_MAX_RANGE_ENV: &str = "NUWAX_MAPPING_RAYTRACING_MAX_RANGE";
pub(crate) const RAY_SUBSAMPLE_ENV: &str = "NUWAX_MAPPING_RAYTRACING_RAY_SUBSAMPLE";
pub(crate) const RAY_SHADOW_DEPTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_SHADOW_DEPTH";
pub(crate) const RAY_GRACE_DEPTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_GRACE_DEPTH";
pub(crate) const RAY_MIN_HEALTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_MIN_HEALTH";
pub(crate) const RAY_MAX_HEALTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_MAX_HEALTH";
pub(crate) const RAY_GRAZE_COS_ENV: &str = "NUWAX_MAPPING_RAYTRACING_GRAZE_COS";
pub(crate) const RAY_RECENCY_WINDOW_ENV: &str = "NUWAX_MAPPING_RAYTRACING_RECENCY_WINDOW";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RaytracingBackendSpec {
    voxel_size: f32,
    max_range: f32,
    ray_subsample: u32,
    shadow_depth: f32,
    grace_depth: f32,
    min_health: i32,
    max_health: i32,
    graze_cos: f32,
    recency_window: u32,
}

impl Default for RaytracingBackendSpec {
    fn default() -> Self {
        Self {
            voxel_size: 0.1,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: -2,
            max_health: 1,
            graze_cos: 0.7,
            recency_window: 15,
        }
    }
}

impl RaytracingBackendSpec {
    fn from_env() -> Result<Self, String> {
        Self::from_fallible_lookup(|key| match env::var(key) {
            Ok(value) => Ok(Some(value)),
            Err(env::VarError::NotPresent) => Ok(None),
            Err(env::VarError::NotUnicode(_)) => Err(format!(
                "RAYTRACING_CONFIG_INVALID: {key} must contain valid Unicode"
            )),
        })
    }

    #[cfg(test)]
    fn from_lookup(mut lookup: impl FnMut(&str) -> Option<String>) -> Result<Self, String> {
        Self::from_fallible_lookup(|key| Ok(lookup(key)))
    }

    fn from_fallible_lookup(
        mut lookup: impl FnMut(&str) -> Result<Option<String>, String>,
    ) -> Result<Self, String> {
        let defaults = Self::default();
        let spec = Self {
            voxel_size: parse_value(&mut lookup, RAY_VOXEL_SIZE_ENV, defaults.voxel_size)?,
            max_range: parse_value(&mut lookup, RAY_MAX_RANGE_ENV, defaults.max_range)?,
            ray_subsample: parse_value(&mut lookup, RAY_SUBSAMPLE_ENV, defaults.ray_subsample)?,
            shadow_depth: parse_value(&mut lookup, RAY_SHADOW_DEPTH_ENV, defaults.shadow_depth)?,
            grace_depth: parse_value(&mut lookup, RAY_GRACE_DEPTH_ENV, defaults.grace_depth)?,
            min_health: parse_value(&mut lookup, RAY_MIN_HEALTH_ENV, defaults.min_health)?,
            max_health: parse_value(&mut lookup, RAY_MAX_HEALTH_ENV, defaults.max_health)?,
            graze_cos: parse_value(&mut lookup, RAY_GRAZE_COS_ENV, defaults.graze_cos)?,
            recency_window: parse_value(
                &mut lookup,
                RAY_RECENCY_WINDOW_ENV,
                defaults.recency_window,
            )?,
        };
        spec.validate()?;
        Ok(spec)
    }

    fn validate(self) -> Result<(), String> {
        let mut empty = VoxelMap::default();
        try_update_map(&mut empty, (0.0, 0.0, 0.0), &[], &self.config())
            .map(|_| ())
            .map_err(|error| format!("RAYTRACING_CONFIG_INVALID: {error}"))
    }

    fn config(self) -> Config {
        Config {
            voxel_size: self.voxel_size,
            max_range: self.max_range,
            ray_subsample: self.ray_subsample,
            shadow_depth: self.shadow_depth,
            grace_depth: self.grace_depth,
            min_health: self.min_health,
            max_health: self.max_health,
            graze_cos: self.graze_cos,
            recency_window: self.recency_window,
            emit_every: 0,
            global_emit_every: 0,
            region_percentile: 95.0,
        }
    }
}

fn parse_value<T>(
    lookup: &mut impl FnMut(&str) -> Result<Option<String>, String>,
    key: &str,
    default: T,
) -> Result<T, String>
where
    T: Copy + Display + FromStr,
    T::Err: Display,
{
    let Some(raw) = lookup(key)? else {
        return Ok(default);
    };
    let value = raw.trim();
    if value.is_empty() {
        return Err(format!(
            "RAYTRACING_CONFIG_INVALID: {key} must not be empty"
        ));
    }
    value.parse::<T>().map_err(|error| {
        format!("RAYTRACING_CONFIG_INVALID: {key}={raw:?} cannot be parsed: {error}")
    })
}

impl From<&Config> for RayConfigCheckpoint {
    fn from(config: &Config) -> Self {
        Self {
            voxel_size: config.voxel_size,
            max_range: config.max_range,
            ray_subsample: config.ray_subsample,
            shadow_depth: config.shadow_depth,
            grace_depth: config.grace_depth,
            min_health: config.min_health,
            max_health: config.max_health,
            graze_cos: config.graze_cos,
            recency_window: config.recency_window,
            emit_every: config.emit_every,
            global_emit_every: config.global_emit_every,
            region_percentile: config.region_percentile,
        }
    }
}

fn ray_config_checkpoint_matches(config: &Config, checkpoint: RayConfigCheckpoint) -> bool {
    config.voxel_size.to_bits() == checkpoint.voxel_size.to_bits()
        && config.max_range.to_bits() == checkpoint.max_range.to_bits()
        && config.ray_subsample == checkpoint.ray_subsample
        && config.shadow_depth.to_bits() == checkpoint.shadow_depth.to_bits()
        && config.grace_depth.to_bits() == checkpoint.grace_depth.to_bits()
        && config.min_health == checkpoint.min_health
        && config.max_health == checkpoint.max_health
        && config.graze_cos.to_bits() == checkpoint.graze_cos.to_bits()
        && config.recency_window == checkpoint.recency_window
        && config.emit_every == checkpoint.emit_every
        && config.global_emit_every == checkpoint.global_emit_every
        && config.region_percentile.to_bits() == checkpoint.region_percentile.to_bits()
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[doc(hidden)]
pub enum MappingBackendSpec {
    SlamToolbox,
    Voxel,
    Raytracing(RaytracingBackendSpec),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[doc(hidden)]
pub enum MappingBackendKind {
    SlamToolbox,
    Voxel,
    Raytracing,
}

#[cfg(test)]
impl MappingBackendKind {
    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::SlamToolbox => "slam_toolbox",
            Self::Voxel => "voxel",
            Self::Raytracing => "raytracing",
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct MappingBackendInput {
    pub source_timestamp_ns: u64,
    pub received_at_ns: u64,
    pub registered_frame_id: String,
    pub sensor_origin_map: [f64; 3],
    pub dynamic_frame: Option<crate::dynamic_backend::MappedFrame>,
    pub registered_points_map: Vec<Point3<f64>>,
}
#[derive(Debug)]
pub(crate) struct MappingBackendCandidate {
    pub latest_frame_occupancy: OccupancyGrid,
    #[cfg(test)]
    pub source_timestamp_ns: u64,
    #[cfg(test)]
    pub received_at_ns: u64,
    #[cfg(test)]
    pub backend_kind: MappingBackendKind,
    #[cfg(test)]
    pub live_window_occupancy: OccupancyGrid,
    #[cfg(test)]
    pub accepted_points_map: Vec<Point3<f64>>,
}
#[derive(Clone, Copy, Debug)]
pub struct MappingBackendComputeConfig {
    pub resolution: f64,
    pub initial_safe_radius_meters: f32,
    pub window_capacity: usize,
}
impl MappingBackendSpec {
    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub fn raytracing(
        voxel_size: f32,
        max_range: f32,
        ray_subsample: u32,
        shadow_depth: f32,
        grace_depth: f32,
        min_health: i32,
        max_health: i32,
        graze_cos: f32,
        recency_window: u32,
    ) -> Self {
        Self::Raytracing(RaytracingBackendSpec {
            voxel_size,
            max_range,
            ray_subsample,
            shadow_depth,
            grace_depth,
            min_health,
            max_health,
            graze_cos,
            recency_window,
        })
    }

    #[doc(hidden)]
    pub fn from_env(selected: MappingBackendKind) -> Result<Self, String> {
        match selected {
            MappingBackendKind::SlamToolbox => Ok(Self::SlamToolbox),
            MappingBackendKind::Voxel => Ok(Self::Voxel),
            MappingBackendKind::Raytracing => {
                Ok(Self::Raytracing(RaytracingBackendSpec::from_env()?))
            }
        }
    }

    #[cfg(test)]
    fn from_lookup(
        selected: MappingBackendKind,
        lookup: impl FnMut(&str) -> Option<String>,
    ) -> Result<Self, String> {
        match selected {
            MappingBackendKind::SlamToolbox => Ok(Self::SlamToolbox),
            MappingBackendKind::Voxel => Ok(Self::Voxel),
            MappingBackendKind::Raytracing => Ok(Self::Raytracing(
                RaytracingBackendSpec::from_lookup(lookup)?,
            )),
        }
    }

    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::SlamToolbox => "slam_toolbox",
            Self::Voxel => "voxel",
            Self::Raytracing(_) => "raytracing",
        }
    }

    pub(crate) fn into_kernel(self, window_capacity: usize) -> MappingKernel {
        match self {
            Self::SlamToolbox => unreachable!("二维进程不能进入点云内核"),
            Self::Voxel => MappingKernel::Voxel(VoxelKernel {
                current_window: VoxelWindow::default(),
                window_cells: Default::default(),
                window_point_indices: Default::default(),
                capacity: window_capacity,
                last_candidate_point_count: 0,
                last_candidate_received_at_ns: 0,
                dirty_since_candidate: false,
                dufo: Default::default(),
            }),
            Self::Raytracing(spec) => MappingKernel::Raytracing(RaytracingKernel {
                config: spec.config(),
                map: VoxelMap::default(),
                prepared_update: None,
                input: Vec::new(),
                last_occupancy: None,
                last_candidate_point_count: 0,
                last_candidate_received_at_ns: 0,
                dirty_since_candidate: false,
                dufo: Default::default(),
            }),
        }
    }

    #[cfg(test)]
    pub(crate) fn raytracing_defaults() -> Self {
        Self::Raytracing(RaytracingBackendSpec::default())
    }
}

#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
pub(crate) enum MappingKernel {
    Voxel(VoxelKernel),
    Raytracing(RaytracingKernel),
}

#[cfg(test)]
impl MappingKernel {
    pub(crate) const fn stable_name(&self) -> &'static str {
        self.kind().stable_name()
    }

    const fn kind(&self) -> MappingBackendKind {
        match self {
            Self::Voxel(_) => MappingBackendKind::Voxel,
            Self::Raytracing(_) => MappingBackendKind::Raytracing,
        }
    }

    pub(crate) fn ready_to_build(&self) -> bool {
        match self {
            Self::Voxel(_) => true,
            Self::Raytracing(kernel) => kernel.map.healthy_count() != 0,
        }
    }

    pub(crate) fn prepare_map_frame(
        &mut self,
        sensor_origin: [f64; 3],
        points: &[Point3<f64>],
    ) -> Result<(), String> {
        let Self::Raytracing(kernel) = self else {
            return Ok(());
        };
        kernel.prepare_map_frame(sensor_origin, points)
    }

    pub(crate) fn commit_prepared_map_frame(&mut self) -> Result<(), String> {
        let Self::Raytracing(kernel) = self else {
            return Ok(());
        };
        kernel.commit_prepared_map_frame()
    }

    // 旧诊断测试显式读取时才物化；生产构图只读取健康体素计数。
    pub(crate) fn accumulated_points(&self, voxel_points: &[Point3<f64>]) -> Vec<Point3<f64>> {
        match self {
            Self::Voxel(_) => voxel_points.to_vec(),
            Self::Raytracing(kernel) => {
                let mut points = iter_global_points(&kernel.map, kernel.config.voxel_size)
                    .map(|(x, y, z)| Point3::new(f64::from(x), f64::from(y), f64::from(z)))
                    .collect::<Vec<_>>();
                points.sort_unstable_by(|left, right| {
                    left.x
                        .total_cmp(&right.x)
                        .then_with(|| left.y.total_cmp(&right.y))
                        .then_with(|| left.z.total_cmp(&right.z))
                });
                points
            }
        }
    }

    pub(crate) fn reset(&mut self) {
        if let Self::Raytracing(kernel) = self {
            kernel.map = VoxelMap::default();
            kernel.prepared_update = None;
            kernel.input.clear();
        }
    }
}

const NO_WINDOW_SLOT: u32 = u32::MAX;

#[derive(Clone, Debug)]
struct VoxelWindowNode {
    point: Point3<f64>,
    previous: u32,
    next: u32,
}

/// 有界稳定槽位；顺序链只记录有效点，空槽复用，不把派生槽号写入检查点。
#[derive(Clone, Debug)]
struct VoxelWindow {
    slots: Vec<VoxelWindowNode>,
    free: Vec<u32>,
    first: u32,
    last: u32,
    len: usize,
}
impl Default for VoxelWindow {
    fn default() -> Self {
        Self {
            slots: Vec::new(),
            free: Vec::new(),
            first: NO_WINDOW_SLOT,
            last: NO_WINDOW_SLOT,
            len: 0,
        }
    }
}
impl VoxelWindow {
    fn len(&self) -> usize {
        self.len
    }
    fn is_empty(&self) -> bool {
        self.len == 0
    }
    fn try_reserve_exact(
        &mut self,
        additional: usize,
    ) -> Result<(), std::collections::TryReserveError> {
        self.slots.try_reserve_exact(additional)
    }
    fn point(&self, slot: u32) -> &Point3<f64> {
        &self.slots[slot as usize].point
    }
    fn push_back(&mut self, point: Point3<f64>) -> u32 {
        let node = VoxelWindowNode {
            point,
            previous: self.last,
            next: NO_WINDOW_SLOT,
        };
        let slot = if let Some(slot) = self.free.pop() {
            self.slots[slot as usize] = node;
            slot
        } else {
            let slot = self.slots.len() as u32;
            self.slots.push(node);
            slot
        };
        if self.last == NO_WINDOW_SLOT {
            self.first = slot;
        } else {
            self.slots[self.last as usize].next = slot;
        }
        self.last = slot;
        self.len += 1;
        slot
    }
    fn remove(&mut self, slot: u32) -> Point3<f64> {
        let VoxelWindowNode {
            point,
            previous,
            next,
        } = self.slots[slot as usize];
        if previous == NO_WINDOW_SLOT {
            self.first = next;
        } else {
            self.slots[previous as usize].next = next;
        }
        if next == NO_WINDOW_SLOT {
            self.last = previous;
        } else {
            self.slots[next as usize].previous = previous;
        }
        self.len -= 1;
        self.free.push(slot);
        point
    }
    fn indexed_iter(&self) -> VoxelWindowIter<'_> {
        VoxelWindowIter {
            window: self,
            next: self.first,
            remaining: self.len,
        }
    }
    fn iter(&self) -> impl ExactSizeIterator<Item = &Point3<f64>> {
        self.indexed_iter().map(|(_, point)| point)
    }
    fn retain(&mut self, mut keep: impl FnMut(&Point3<f64>) -> bool) {
        let mut slot = self.first;
        while slot != NO_WINDOW_SLOT {
            let next = self.slots[slot as usize].next;
            if !keep(self.point(slot)) {
                self.remove(slot);
            }
            slot = next;
        }
    }
}
struct VoxelWindowIter<'a> {
    window: &'a VoxelWindow,
    next: u32,
    remaining: usize,
}
impl<'a> Iterator for VoxelWindowIter<'a> {
    type Item = (u32, &'a Point3<f64>);
    fn next(&mut self) -> Option<Self::Item> {
        if self.next == NO_WINDOW_SLOT {
            return None;
        }
        let slot = self.next;
        let node = &self.window.slots[slot as usize];
        self.next = node.next;
        self.remaining -= 1;
        Some((slot, &node.point))
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}
impl ExactSizeIterator for VoxelWindowIter<'_> {}
impl FromIterator<Point3<f64>> for VoxelWindow {
    fn from_iter<T: IntoIterator<Item = Point3<f64>>>(points: T) -> Self {
        let mut window = Self::default();
        for point in points {
            window.push_back(point);
        }
        window
    }
}
impl PartialEq<VecDeque<Point3<f64>>> for VoxelWindow {
    fn eq(&self, other: &VecDeque<Point3<f64>>) -> bool {
        self.iter().eq(other.iter())
    }
}

#[derive(Clone)]
pub(crate) struct VoxelKernel {
    current_window: VoxelWindow,
    // 与窗口同步计数；不能量化的点放入保留键，带坐标变换的清除必须回退逐点。
    window_cells: ahash::AHashMap<crate::dynamic_scene::Cell, u32>,
    // 单格内按到达顺序保存稳定槽号，只在实际删除或容量淘汰时移除对应槽。
    window_point_indices: ahash::AHashMap<crate::dynamic_scene::Cell, VecDeque<u32>>,
    capacity: usize,
    last_candidate_point_count: usize,
    last_candidate_received_at_ns: u64,
    dirty_since_candidate: bool,
    dufo: crate::dynamic_backend::OnlineDufo,
}

impl VoxelKernel {
    fn append_frame(&mut self, points: &[Point3<f64>]) {
        let retention_capacity = self.capacity.min(MAX_RETAINED_VOXEL_POINTS);
        let accepted = points.len().min(retention_capacity);
        let retained_start = points.len().saturating_sub(accepted);
        let existing_budget = retention_capacity.saturating_sub(accepted);
        let evicted = self.current_window.len().saturating_sub(existing_budget);
        for _ in 0..evicted {
            let point = self.current_window.remove(self.current_window.first);
            uncount_window_cell(&mut self.window_cells, &point);
            let key = window_cell(&point);
            let indices = self.window_point_indices.get_mut(&key).unwrap();
            indices.pop_front();
            if indices.is_empty() {
                self.window_point_indices.remove(&key);
            }
        }
        for &point in &points[retained_start..] {
            let slot = self.current_window.push_back(point);
            count_window_cell(&mut self.window_cells, &point);
            self.window_point_indices
                .entry(window_cell(&point))
                .or_default()
                .push_back(slot);
        }
    }

    fn rebuild_window_index(&mut self) {
        self.window_cells = window_cells(&self.current_window);
        self.window_point_indices = window_point_indices(&self.current_window);
    }

    /// 先核对相交格里的具体点；无实际删除则不遍历整窗，保持原点序与参考保护。
    fn retain_uncleared(&mut self, frame: Option<&crate::dynamic_backend::MappedFrame>) {
        #[cfg(test)]
        let force_full = std::env::var_os("NUWAX_PGO_FULL_WINDOW_FILTER").is_some();
        #[cfg(not(test))]
        let force_full = false;
        let candidates = if force_full {
            None
        } else if let Some(frame) = frame {
            if frame.has_clears() && self.window_cells.contains_key(&UNINDEXED_WINDOW_CELL) {
                None
            } else {
                frame.potential_window_clear_cells(&self.window_cells)
            }
        } else {
            Some(ahash::AHashSet::new())
        };
        if let Some(candidates) = candidates {
            #[cfg(test)]
            let indexed_started = Instant::now();
            #[cfg(test)]
            let (candidate_count, mut checked) = (candidates.len(), 0usize);
            let mut removed = ahash::AHashSet::new();
            for key in &candidates {
                for &index in &self.window_point_indices[key] {
                    #[cfg(test)]
                    {
                        checked += 1;
                    }
                    let point = self.current_window.point(index);
                    let p = [point.x, point.y, point.z];
                    if frame.is_some_and(|f| f.clears(p)) {
                        removed.insert(index);
                    }
                }
            }
            #[cfg(test)]
            if std::env::var_os("NUWAX_PGO_WINDOW_SHADOW").is_some() {
                let expected: ahash::AHashSet<_> = self
                    .current_window
                    .indexed_iter()
                    .filter_map(|(i, p)| {
                        frame
                            .is_some_and(|f| f.clears([p.x, p.y, p.z]))
                            .then_some(i)
                    })
                    .collect();
                assert_eq!(removed, expected, "窗口索引不得漏掉或误报任何待删点");
            }
            #[cfg(test)]
            if std::env::var_os("NUWAX_PGO_WINDOW_DIAGNOSTIC").is_some() {
                eprintln!("WINDOW_INDEX points={} candidate_cells={candidate_count} checked={checked} removed={} lookup_ms={:.3}",
                    self.current_window.len(), removed.len(), indexed_started.elapsed().as_secs_f64() * 1e3);
            }
            if removed.is_empty() {
                return;
            }
            #[cfg(test)]
            let rebuilding = Instant::now();
            for key in candidates {
                let indices = self.window_point_indices.get_mut(&key).unwrap();
                indices.retain(|slot| !removed.contains(slot));
                if indices.is_empty() {
                    self.window_point_indices.remove(&key);
                }
            }
            for slot in removed {
                let point = self.current_window.remove(slot);
                uncount_window_cell(&mut self.window_cells, &point);
            }
            #[cfg(test)]
            if std::env::var_os("NUWAX_PGO_WINDOW_DIAGNOSTIC").is_some() {
                eprintln!(
                    "WINDOW_REMOVE points={} ms={:.3}",
                    self.current_window.len(),
                    rebuilding.elapsed().as_secs_f64() * 1e3
                );
            }
        } else {
            let before = self.current_window.len();
            self.current_window.retain(|point| {
                let p = [point.x, point.y, point.z];
                !frame.is_some_and(|f| f.clears(p))
            });
            if self.current_window.len() != before {
                self.rebuild_window_index();
            }
        }
    }
}
// cell()排除i32边缘，保留键不与任何真实格冲突。
const UNINDEXED_WINDOW_CELL: crate::dynamic_scene::Cell = (i32::MIN, i32::MIN, i32::MIN);
fn window_cell(point: &Point3<f64>) -> crate::dynamic_scene::Cell {
    crate::dynamic_scene::cell([point.x, point.y, point.z]).unwrap_or(UNINDEXED_WINDOW_CELL)
}
fn window_point_indices(
    window: &VoxelWindow,
) -> ahash::AHashMap<crate::dynamic_scene::Cell, VecDeque<u32>> {
    let mut indices: ahash::AHashMap<_, VecDeque<u32>> = Default::default();
    for (index, point) in window.indexed_iter() {
        indices
            .entry(window_cell(point))
            .or_default()
            .push_back(index);
    }
    indices
}
fn count_window_cell(
    cells: &mut ahash::AHashMap<crate::dynamic_scene::Cell, u32>,
    point: &Point3<f64>,
) {
    let key =
        crate::dynamic_scene::cell([point.x, point.y, point.z]).unwrap_or(UNINDEXED_WINDOW_CELL);
    *cells.entry(key).or_default() += 1;
}
fn uncount_window_cell(
    cells: &mut ahash::AHashMap<crate::dynamic_scene::Cell, u32>,
    point: &Point3<f64>,
) {
    let key =
        crate::dynamic_scene::cell([point.x, point.y, point.z]).unwrap_or(UNINDEXED_WINDOW_CELL);
    if let std::collections::hash_map::Entry::Occupied(mut entry) = cells.entry(key) {
        *entry.get_mut() -= 1;
        if *entry.get() == 0 {
            entry.remove();
        }
    }
}
fn window_cells(window: &VoxelWindow) -> ahash::AHashMap<crate::dynamic_scene::Cell, u32> {
    let mut cells = ahash::AHashMap::new();
    for point in window.iter() {
        count_window_cell(&mut cells, point);
    }
    cells
}

#[derive(Clone)]
pub(crate) struct RaytracingKernel {
    config: Config,
    map: VoxelMap,
    prepared_update: Option<PreparedRayUpdate>,
    input: Vec<(f32, f32, f32)>,
    last_occupancy: Option<OccupancyGrid>,
    last_candidate_point_count: usize,
    last_candidate_received_at_ns: u64,
    dirty_since_candidate: bool,
    dufo: crate::dynamic_backend::OnlineDufo,
}

impl RaytracingKernel {
    fn prepare_map_frame(
        &mut self,
        sensor_origin: [f64; 3],
        points: &[Point3<f64>],
    ) -> Result<(), String> {
        self.prepared_update = None;
        let origin = (
            sensor_origin[0] as f32,
            sensor_origin[1] as f32,
            sensor_origin[2] as f32,
        );
        self.input.clear();
        self.input.reserve(points.len());
        self.input.extend(
            points
                .iter()
                .map(|point| (point.x as f32, point.y as f32, point.z as f32)),
        );
        let input = std::mem::take(&mut self.input);
        self.prepared_update = Some(
            PreparedRayUpdate::new(origin, input, self.config.clone())
                .map_err(|error| format!("RAYTRACING_UPDATE_REJECTED: {error}"))?,
        );
        Ok(())
    }

    #[cfg(test)]
    fn commit_prepared_map_frame(&mut self) -> Result<(), String> {
        self.commit_prepared_map_frame_with_parallelism(true)
    }

    fn commit_prepared_map_frame_with_parallelism(&mut self, parallel: bool) -> Result<(), String> {
        let prepared = self
            .prepared_update
            .take()
            .ok_or_else(|| "RAYTRACING_UPDATE_REJECTED: no prepared frame".to_owned())?;
        self.input = if parallel {
            prepared.apply(&mut self.map)
        } else {
            prepared.apply_serial(&mut self.map)
        };
        Ok(())
    }

    fn validate_checkpoint_budget(&self) -> Result<(), String> {
        // 已拥有的栅格由构图或检查点解码验证；预算只需精确长度，不再逐格检查旧内容。
        let occupancy_len = self
            .last_occupancy
            .as_ref()
            .map(|grid| {
                MAPPING_BACKEND_OCCUPANCY_FIXED_BYTES + grid.frame_id.len() + grid.data.len()
            })
            .unwrap_or(0);
        let encoded_len = MAPPING_BACKEND_RAY_HEADER_BYTES
            .checked_add(voxel_map_checkpoint_encoded_len(&self.map)?)
            .and_then(|length| length.checked_add(occupancy_len))
            .ok_or_else(|| "RAYTRACING_UPDATE_REJECTED: checkpoint length overflow".to_owned())?;
        if encoded_len > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
            return Err(format!(
                "RAYTRACING_UPDATE_REJECTED: retained ray state requires {encoded_len} checkpoint bytes, limit is {MAX_MAPPING_BACKEND_CHECKPOINT_BYTES}"
            ));
        }
        Ok(())
    }
}

fn mapping_backend_voxel_checkpoint_encoded_len(voxel: &VoxelKernel) -> Result<usize, String> {
    if voxel.current_window.len() > MAX_RETAINED_VOXEL_POINTS {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window contains {} points, limit is {MAX_RETAINED_VOXEL_POINTS}",
            voxel.current_window.len()
        ));
    }
    MAPPING_BACKEND_VOXEL_POINT_BYTES
        .checked_mul(voxel.current_window.len())
        .and_then(|points| MAPPING_BACKEND_VOXEL_HEADER_BYTES.checked_add(points))
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel checkpoint length overflow".to_owned()
        })
}

fn mapping_backend_occupancy_checkpoint_encoded_len(
    occupancy: &OccupancyGrid,
) -> Result<usize, String> {
    validate_kernel_checkpoint_occupancy(occupancy)?;
    u32::try_from(occupancy.frame_id.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id is too long".to_owned()
    })?;
    u64::try_from(occupancy.data.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy data length overflow".to_owned()
    })?;
    MAPPING_BACKEND_OCCUPANCY_FIXED_BYTES
        .checked_add(occupancy.frame_id.len())
        .and_then(|length| length.checked_add(occupancy.data.len()))
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy checkpoint length overflow".to_owned()
        })
}

fn mapping_backend_ray_checkpoint_encoded_len(ray: &RaytracingKernel) -> Result<usize, String> {
    let voxel_map_len = voxel_map_checkpoint_encoded_len(&ray.map).map_err(|error| {
        format!("MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length failed: {error}")
    })?;
    let occupancy_len = ray
        .last_occupancy
        .as_ref()
        .map(mapping_backend_occupancy_checkpoint_encoded_len)
        .transpose()?
        .unwrap_or(0);
    MAPPING_BACKEND_RAY_HEADER_BYTES
        .checked_add(voxel_map_len)
        .and_then(|length| length.checked_add(occupancy_len))
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: ray checkpoint length overflow".to_owned()
        })
}

fn allocate_mapping_checkpoint(encoded_len: usize) -> Result<Vec<u8>, String> {
    if encoded_len == 0 || encoded_len > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: payload size {encoded_len} is outside 1..={MAX_MAPPING_BACKEND_CHECKPOINT_BYTES} bytes"
        ));
    }
    let mut payload = Vec::new();
    payload.try_reserve_exact(encoded_len).map_err(|error| {
        format!("MAPPING_BACKEND_CHECKPOINT_INVALID: allocation failed: {error}")
    })?;
    Ok(payload)
}

fn encode_mapping_checkpoint_header(payload: &mut Vec<u8>, backend_kind: u8) {
    payload.extend_from_slice(&MAPPING_BACKEND_CHECKPOINT_MAGIC);
    payload.extend_from_slice(&MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION.to_le_bytes());
    payload.push(backend_kind);
    payload.push(0);
    payload.extend_from_slice(&0_u16.to_le_bytes());
}

fn encode_ray_config(payload: &mut Vec<u8>, config: &Config) {
    payload.extend_from_slice(&config.voxel_size.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.max_range.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.ray_subsample.to_le_bytes());
    payload.extend_from_slice(&config.shadow_depth.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.grace_depth.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.min_health.to_le_bytes());
    payload.extend_from_slice(&config.max_health.to_le_bytes());
    payload.extend_from_slice(&config.graze_cos.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.recency_window.to_le_bytes());
    payload.extend_from_slice(&config.emit_every.to_le_bytes());
    payload.extend_from_slice(&config.global_emit_every.to_le_bytes());
    payload.extend_from_slice(&config.region_percentile.to_bits().to_le_bytes());
}

fn encode_checkpoint_occupancy(
    payload: &mut Vec<u8>,
    occupancy: &OccupancyGrid,
) -> Result<(), String> {
    let frame_id_len = u32::try_from(occupancy.frame_id.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id is too long".to_owned()
    })?;
    let data_len = u64::try_from(occupancy.data.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy data length overflow".to_owned()
    })?;
    payload.extend_from_slice(&occupancy.width.to_le_bytes());
    payload.extend_from_slice(&occupancy.height.to_le_bytes());
    payload.extend_from_slice(&occupancy.resolution.to_bits().to_le_bytes());
    for value in occupancy.origin {
        payload.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    for value in occupancy.origin_orientation {
        payload.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    payload.extend_from_slice(&occupancy.ts.to_bits().to_le_bytes());
    payload.extend_from_slice(&frame_id_len.to_le_bytes());
    payload.extend_from_slice(&data_len.to_le_bytes());
    payload.extend_from_slice(occupancy.frame_id.as_bytes());
    payload.extend(occupancy.data.iter().map(|value| *value as u8));
    Ok(())
}

pub(crate) fn export_mapping_kernel_checkpoint(kernel: &MappingKernel) -> Result<Vec<u8>, String> {
    let dirty = match kernel {
        MappingKernel::Voxel(voxel) => voxel.dirty_since_candidate,
        MappingKernel::Raytracing(ray) => ray.dirty_since_candidate,
    };
    if dirty {
        return Err(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: backend state must be flushed before export"
                .to_owned(),
        );
    }
    let encoded_len = match kernel {
        MappingKernel::Voxel(voxel) => mapping_backend_voxel_checkpoint_encoded_len(voxel)?,
        MappingKernel::Raytracing(ray) => mapping_backend_ray_checkpoint_encoded_len(ray)?,
    };
    let mut payload = match kernel {
        MappingKernel::Voxel(voxel) => {
            let mut payload = allocate_mapping_checkpoint(encoded_len)?;
            encode_mapping_checkpoint_header(&mut payload, MAPPING_BACKEND_KIND_VOXEL);
            payload.extend_from_slice(
                &u64::try_from(voxel.capacity)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel capacity overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(
                &u64::try_from(voxel.current_window.len())
                    .map_err(|_| {
                        "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window length overflow"
                    })?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(
                &u64::try_from(voxel.last_candidate_point_count)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel point count overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(&voxel.last_candidate_received_at_ns.to_le_bytes());
            for point in voxel.current_window.iter() {
                payload.extend_from_slice(&point.x.to_bits().to_le_bytes());
                payload.extend_from_slice(&point.y.to_bits().to_le_bytes());
                payload.extend_from_slice(&point.z.to_bits().to_le_bytes());
            }
            payload
        }
        MappingKernel::Raytracing(ray) => {
            let mut payload = allocate_mapping_checkpoint(encoded_len)?;
            let voxel_map_len = voxel_map_checkpoint_encoded_len(&ray.map).map_err(|error| {
                format!("MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length failed: {error}")
            })?;
            let occupancy_len = ray
                .last_occupancy
                .as_ref()
                .map(mapping_backend_occupancy_checkpoint_encoded_len)
                .transpose()?
                .unwrap_or(0);
            encode_mapping_checkpoint_header(&mut payload, MAPPING_BACKEND_KIND_RAYTRACING);
            encode_ray_config(&mut payload, &ray.config);
            payload.extend_from_slice(
                &u64::try_from(ray.last_candidate_point_count)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: ray point count overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(&ray.last_candidate_received_at_ns.to_le_bytes());
            payload.extend_from_slice(
                &u64::try_from(voxel_map_len)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(
                &u64::try_from(occupancy_len)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy length overflow")?
                    .to_le_bytes(),
            );
            append_voxel_map_checkpoint(&ray.map, &mut payload).map_err(|error| {
                format!("MAPPING_BACKEND_CHECKPOINT_INVALID: ray map encode failed: {error}")
            })?;
            if let Some(occupancy) = ray.last_occupancy.as_ref() {
                encode_checkpoint_occupancy(&mut payload, occupancy)?;
            }
            payload
        }
    };
    if payload.len() != encoded_len {
        let actual_len = payload.len();
        payload.clear();
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: encoded length mismatch: expected={encoded_len} actual={actual_len}"
        ));
    }
    Ok(payload)
}

struct MappingCheckpointReader<'a> {
    payload: &'a [u8],
    offset: usize,
}

impl<'a> MappingCheckpointReader<'a> {
    fn new(payload: &'a [u8]) -> Self {
        Self { payload, offset: 0 }
    }

    fn remaining(&self) -> usize {
        self.payload.len() - self.offset
    }

    fn read_bytes(&mut self, length: usize) -> Result<&'a [u8], String> {
        let end = self.offset.checked_add(length).ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: decode offset overflow".to_owned()
        })?;
        let bytes = self.payload.get(self.offset..end).ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: checkpoint is truncated".to_owned()
        })?;
        self.offset = end;
        Ok(bytes)
    }

    fn read_array<const N: usize>(&mut self) -> Result<[u8; N], String> {
        self.read_bytes(N)?
            .try_into()
            .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: checkpoint is truncated".to_owned())
    }

    fn read_u8(&mut self) -> Result<u8, String> {
        Ok(self.read_array::<1>()?[0])
    }

    fn read_u16(&mut self) -> Result<u16, String> {
        Ok(u16::from_le_bytes(self.read_array()?))
    }

    fn read_u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.read_array()?))
    }

    fn read_i32(&mut self) -> Result<i32, String> {
        Ok(i32::from_le_bytes(self.read_array()?))
    }

    fn read_u64(&mut self) -> Result<u64, String> {
        Ok(u64::from_le_bytes(self.read_array()?))
    }

    fn read_f32(&mut self) -> Result<f32, String> {
        Ok(f32::from_bits(self.read_u32()?))
    }

    fn read_f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_bits(self.read_u64()?))
    }

    fn finish(self) -> Result<(), String> {
        if self.remaining() == 0 {
            Ok(())
        } else {
            Err(format!(
                "MAPPING_BACKEND_CHECKPOINT_INVALID: {} trailing bytes",
                self.remaining()
            ))
        }
    }
}

fn decode_mapping_checkpoint_header(
    reader: &mut MappingCheckpointReader<'_>,
    spec: MappingBackendSpec,
) -> Result<(), String> {
    if reader.read_array::<8>()? != MAPPING_BACKEND_CHECKPOINT_MAGIC {
        return Err("MAPPING_BACKEND_CHECKPOINT_INVALID: invalid checkpoint magic".to_owned());
    }
    let schema_version = reader.read_u32()?;
    if schema_version != MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: unsupported schema version {schema_version}"
        ));
    }
    let backend_kind = reader.read_u8()?;
    let expected_kind = match spec {
        MappingBackendSpec::SlamToolbox => 2,
        MappingBackendSpec::Voxel => MAPPING_BACKEND_KIND_VOXEL,
        MappingBackendSpec::Raytracing(_) => MAPPING_BACKEND_KIND_RAYTRACING,
    };
    if backend_kind != expected_kind {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: backend kind mismatch: checkpoint={backend_kind} selected={}",
            spec.stable_name()
        ));
    }
    let flags = reader.read_u8()?;
    let reserved = reader.read_u16()?;
    if flags != 0 || reserved != 0 {
        return Err(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: checkpoint flags/reserved bytes must be zero"
                .to_owned(),
        );
    }
    Ok(())
}

fn decode_ray_config(
    reader: &mut MappingCheckpointReader<'_>,
) -> Result<RayConfigCheckpoint, String> {
    Ok(RayConfigCheckpoint {
        voxel_size: reader.read_f32()?,
        max_range: reader.read_f32()?,
        ray_subsample: reader.read_u32()?,
        shadow_depth: reader.read_f32()?,
        grace_depth: reader.read_f32()?,
        min_health: reader.read_i32()?,
        max_health: reader.read_i32()?,
        graze_cos: reader.read_f32()?,
        recency_window: reader.read_u32()?,
        emit_every: reader.read_u32()?,
        global_emit_every: reader.read_u32()?,
        region_percentile: reader.read_f32()?,
    })
}

fn decode_checkpoint_occupancy(payload: &[u8]) -> Result<OccupancyGrid, String> {
    let mut reader = MappingCheckpointReader::new(payload);
    let width = reader.read_u32()?;
    let height = reader.read_u32()?;
    let resolution = reader.read_f64()?;
    let mut origin = [0.0; 3];
    for value in &mut origin {
        *value = reader.read_f64()?;
    }
    let mut origin_orientation = [0.0; 4];
    for value in &mut origin_orientation {
        *value = reader.read_f64()?;
    }
    let ts = reader.read_f64()?;
    let frame_id_len = usize::try_from(reader.read_u32()?).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id length exceeds usize".to_owned()
    })?;
    let data_len = usize::try_from(reader.read_u64()?).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy data length exceeds usize".to_owned()
    })?;
    let expected_remaining = frame_id_len.checked_add(data_len).ok_or_else(|| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy variable length overflow".to_owned()
    })?;
    if reader.remaining() != expected_remaining {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy payload length mismatch: declared={expected_remaining} actual={}",
            reader.remaining()
        ));
    }
    let frame_id =
        String::from_utf8(reader.read_bytes(frame_id_len)?.to_vec()).map_err(|error| {
            format!("MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id is not UTF-8: {error}")
        })?;
    let data = reader
        .read_bytes(data_len)?
        .iter()
        .map(|value| *value as i8)
        .collect();
    reader.finish()?;
    let occupancy = OccupancyGrid {
        data,
        width,
        height,
        resolution,
        origin,
        origin_orientation,
        frame_id,
        ts,
    };
    validate_kernel_checkpoint_occupancy(&occupancy)?;
    Ok(occupancy)
}

pub(crate) fn restore_mapping_kernel_checkpoint(
    spec: MappingBackendSpec,
    compute_config: MappingBackendComputeConfig,
    payload: &[u8],
) -> Result<MappingKernel, String> {
    if payload.is_empty() || payload.len() > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: payload size {} is outside 1..={} bytes",
            payload.len(),
            MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
        ));
    }
    let mut reader = MappingCheckpointReader::new(payload);
    decode_mapping_checkpoint_header(&mut reader, spec)?;

    match spec {
        MappingBackendSpec::SlamToolbox => Err("二维图检查点须由二维工作进程恢复".into()),
        MappingBackendSpec::Voxel => {
            let capacity = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel capacity exceeds usize".to_owned()
            })?;
            let point_count = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel point count exceeds usize".to_owned()
            })?;
            let last_candidate_point_count = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel candidate count exceeds usize".to_owned()
            })?;
            let last_candidate_received_at_ns = reader.read_u64()?;
            if capacity != compute_config.window_capacity {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel capacity mismatch: checkpoint={capacity} selected={}",
                    compute_config.window_capacity
                ));
            }
            let retained_capacity = capacity.min(MAX_RETAINED_VOXEL_POINTS);
            if point_count > retained_capacity {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window contains {point_count} points for retained capacity {retained_capacity}"
                ));
            }
            if last_candidate_point_count > retained_capacity {
                return Err(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel candidate count exceeds retained capacity"
                        .to_owned(),
                );
            }
            let encoded_points = point_count
                .checked_mul(MAPPING_BACKEND_VOXEL_POINT_BYTES)
                .ok_or_else(|| {
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel point bytes overflow".to_owned()
                })?;
            if reader.remaining() != encoded_points {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel payload length mismatch: declared={encoded_points} actual={}",
                    reader.remaining()
                ));
            }
            let mut current_window = VoxelWindow::default();
            current_window
                .try_reserve_exact(point_count)
                .map_err(|error| {
                    format!("MAPPING_BACKEND_CHECKPOINT_INVALID: voxel allocation failed: {error}")
                })?;
            for _ in 0..point_count {
                let point = Point3::new(reader.read_f64()?, reader.read_f64()?, reader.read_f64()?);
                if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                    return Err(
                        "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window contains non-finite coordinates"
                            .to_owned(),
                    );
                }
                current_window.push_back(point);
            }
            reader.finish()?;
            Ok(MappingKernel::Voxel(VoxelKernel {
                window_cells: window_cells(&current_window),
                window_point_indices: window_point_indices(&current_window),
                current_window,
                capacity,
                last_candidate_point_count,
                last_candidate_received_at_ns,
                dirty_since_candidate: false,
                dufo: Default::default(),
            }))
        }
        MappingBackendSpec::Raytracing(ray_spec) => {
            let checkpoint_config = decode_ray_config(&mut reader)?;
            let last_candidate_point_count = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: ray candidate count exceeds usize".to_owned()
            })?;
            let last_candidate_received_at_ns = reader.read_u64()?;
            let voxel_map_len = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length exceeds usize".to_owned()
            })?;
            let occupancy_len = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy length exceeds usize".to_owned()
            })?;
            let encoded_body_len = voxel_map_len.checked_add(occupancy_len).ok_or_else(|| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: ray body length overflow".to_owned()
            })?;
            if reader.remaining() != encoded_body_len {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: ray payload length mismatch: declared={encoded_body_len} actual={}",
                    reader.remaining()
                ));
            }
            let config = ray_spec.config();
            if !ray_config_checkpoint_matches(&config, checkpoint_config) {
                return Err(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: ray configuration mismatch".to_owned(),
                );
            }
            let max_voxels = (MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
                - MAPPING_BACKEND_RAY_HEADER_BYTES
                - VOXEL_MAP_CHECKPOINT_HEADER_BYTES)
                / VOXEL_MAP_CHECKPOINT_ENTRY_BYTES;
            let map = decode_voxel_map_checkpoint_for_config(
                reader.read_bytes(voxel_map_len)?,
                max_voxels,
                &config,
            )?;
            let last_occupancy = if occupancy_len == 0 {
                None
            } else {
                Some(decode_checkpoint_occupancy(
                    reader.read_bytes(occupancy_len)?,
                )?)
            };
            reader.finish()?;
            if last_candidate_point_count > max_voxels {
                return Err(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: ray candidate count exceeds state limit"
                        .to_owned(),
                );
            }
            let ray = RaytracingKernel {
                config,
                map,
                prepared_update: None,
                input: Vec::new(),
                last_occupancy,
                last_candidate_point_count,
                last_candidate_received_at_ns,
                dirty_since_candidate: false,
                dufo: Default::default(),
            };
            ray.validate_checkpoint_budget().map_err(|error| {
                format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: restored ray state is oversized: {error}"
                )
            })?;
            Ok(MappingKernel::Raytracing(ray))
        }
    }
}

fn validate_kernel_checkpoint_occupancy(occupancy: &OccupancyGrid) -> Result<(), String> {
    let expected = usize::try_from(occupancy.width)
        .ok()
        .and_then(|width| {
            usize::try_from(occupancy.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy dimensions overflow".to_owned()
        })?;
    if occupancy.width == 0
        || occupancy.height == 0
        || occupancy.data.len() != expected
        || !occupancy.resolution.is_finite()
        || occupancy.resolution <= 0.0
        || occupancy.origin.iter().any(|value| !value.is_finite())
        || occupancy
            .origin_orientation
            .iter()
            .any(|value| !value.is_finite())
        || occupancy.frame_id.trim().is_empty()
        || !occupancy.ts.is_finite()
        || occupancy
            .data
            .iter()
            .any(|value| !(-1..=100).contains(value))
    {
        return Err("MAPPING_BACKEND_CHECKPOINT_INVALID: retained occupancy is invalid".to_owned());
    }
    Ok(())
}

fn occupancy_config(config: MappingBackendComputeConfig) -> OccupancyConfig {
    OccupancyConfig {
        initial_safe_radius_meters: config.initial_safe_radius_meters,
    }
}

struct BorrowedBackendCloud<'a> {
    points: &'a [Point3<f64>],
    sensor_origin: Point3<f64>,
    frame_id: &'a str,
    timestamp: f64,
}

impl PointCloud2Like for BorrowedBackendCloud<'_> {
    fn points(&self) -> &[Point3<f64>] {
        self.points
    }

    fn sensor_origin(&self) -> Point3<f64> {
        self.sensor_origin
    }

    fn frame_id(&self) -> &str {
        self.frame_id
    }

    fn timestamp(&self) -> f64 {
        self.timestamp
    }
}

fn build_occupancy(
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
) -> Result<OccupancyGrid, String> {
    // 水平扫描必须与本帧雷达原点配对，不能从当前原点向历史点云发射射线。
    let retained: Vec<_> = input
        .registered_points_map
        .iter()
        .filter(|p| {
            !input
                .dynamic_frame
                .as_ref()
                .is_some_and(|f| f.is_dynamic([p.x, p.y, p.z]))
        })
        .copied()
        .collect();
    let cloud = BorrowedBackendCloud {
        points: &retained,
        sensor_origin: nalgebra::point![
            input.sensor_origin_map[0],
            input.sensor_origin_map[1],
            input.sensor_origin_map[2]
        ],
        frame_id: &input.registered_frame_id,
        timestamp: input.source_timestamp_ns as f64 / 1_000_000_000.0,
    };
    CostMapper::occupancy_from_pointcloud_auto(&cloud, &occupancy_config(config), config.resolution)
        .map_err(|error| format!("MAPPING_BACKEND_COMPUTE_REJECTED: {error}"))
}

fn candidate_due(last_received_at_ns: u64, received_at_ns: u64, interval_ns: u64) -> bool {
    last_received_at_ns == 0 || received_at_ns.saturating_sub(last_received_at_ns) >= interval_ns
}

pub(crate) fn compute_candidate_in_place(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
) -> Result<Option<MappingBackendCandidate>, String> {
    compute_candidate_with_parallelism(kernel, input, config, MAPPING_CANDIDATE_INTERVAL_NS, true)
}

/// 历史回放自行选择构图节拍；点云接纳与累积仍逐帧执行。
pub(crate) fn compute_candidate_in_place_with_interval(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
    candidate_interval_ns: u64,
) -> Result<Option<MappingBackendCandidate>, String> {
    compute_candidate_with_parallelism(kernel, input, config, candidate_interval_ns, false)
}

fn compute_candidate_with_parallelism(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
    candidate_interval_ns: u64,
    parallel: bool,
) -> Result<Option<MappingBackendCandidate>, String> {
    if input.registered_points_map.is_empty() {
        return Ok(None);
    }

    if input.dynamic_frame.is_none() {
        let dufo = match kernel {
            MappingKernel::Voxel(k) => &mut k.dufo,
            MappingKernel::Raytracing(k) => &mut k.dufo,
        };
        let frame = dufo
            .observe(
                &input.registered_points_map,
                input.sensor_origin_map,
                input.source_timestamp_ns,
                nalgebra::Isometry3::identity(),
            )?
            .0;
        let mut classified = input.clone();
        classified.dynamic_frame = Some(frame);
        return compute_candidate_with_parallelism(
            kernel,
            &classified,
            config,
            candidate_interval_ns,
            parallel,
        );
    }
    #[cfg(test)]
    let backend_kind = kernel.kind();
    #[cfg(test)]
    let diagnostic = std::env::var_os("NEH_PGO_PHASE_DIAGNOSTIC").is_some();
    match kernel {
        MappingKernel::Voxel(voxel) => {
            voxel.retain_uncleared(input.dynamic_frame.as_ref());
            let retained = input
                .registered_points_map
                .iter()
                .filter(|p| {
                    !input
                        .dynamic_frame
                        .as_ref()
                        .is_some_and(|f| f.is_dynamic([p.x, p.y, p.z]))
                })
                .copied()
                .collect::<Vec<_>>();
            voxel.append_frame(&retained);
            voxel.dirty_since_candidate = true;
        }
        MappingKernel::Raytracing(ray) => {
            #[cfg(test)]
            let phase = Instant::now();
            let retained = input
                .registered_points_map
                .iter()
                .filter(|p| {
                    !input
                        .dynamic_frame
                        .as_ref()
                        .is_some_and(|f| f.is_dynamic([p.x, p.y, p.z]))
                })
                .copied()
                .collect::<Vec<_>>();
            // 本地 9e719718 已删除 validate_retained_state_bound（重复的保留体素上限校验），这里不再调用。
            ray.prepare_map_frame(input.sensor_origin_map, &retained)?;
            #[cfg(test)]
            let prepare_us = phase.elapsed().as_micros();
            #[cfg(test)]
            let phase = Instant::now();
            ray.commit_prepared_map_frame_with_parallelism(parallel)?;
            #[cfg(test)]
            let update_us = phase.elapsed().as_micros();
            #[cfg(test)]
            let phase = Instant::now();
            if input.dynamic_frame.is_some() {
                // 当前回波仍作为导航障碍，清理只撤销本帧未占据的历史体素。
                let size = f64::from(ray.config.voxel_size);
                // 命中键必须与射线库的单精度乘逆量化完全相同，边界点不可漏保护。
                let inv = 1.0_f32 / ray.config.voxel_size;
                let hits: ahash::AHashSet<_> = ray
                    .input
                    .iter()
                    .map(|point| {
                        (
                            (point.0 * inv).floor() as i32,
                            (point.1 * inv).floor() as i32,
                            (point.2 * inv).floor() as i32,
                        )
                    })
                    .collect();
                let candidates = (!crate::dynamic_backend::reference_revoke())
                    .then(|| {
                        input.dynamic_frame.as_ref().and_then(|frame| {
                            frame.candidate_grid_keys(size, |key| {
                                !hits.contains(&key) && ray.map.voxels.contains_key(&key)
                            })
                        })
                    })
                    .flatten();
                if let Some(candidates) = candidates {
                    let frame = input.dynamic_frame.as_ref().unwrap();
                    for key in candidates {
                        let point = [
                            (key.0 as f64 + 0.5) * size,
                            (key.1 as f64 + 0.5) * size,
                            (key.2 as f64 + 0.5) * size,
                        ];
                        if frame.clears(point) {
                            ray.map.voxels.remove(&key);
                        }
                    }
                } else {
                    // 测试参考路径保留原来的整表复核和本帧命中保护。
                    ray.map.voxels.retain(|key, _| {
                        hits.contains(key)
                            || !{
                                let p = [
                                    (key.0 as f64 + 0.5) * size,
                                    (key.1 as f64 + 0.5) * size,
                                    (key.2 as f64 + 0.5) * size,
                                ];
                                input.dynamic_frame.as_ref().is_some_and(|f| f.clears(p))
                            }
                    });
                }
            }
            #[cfg(test)]
            if diagnostic {
                eprintln!("PGO_RAY_PHASE frame={} points={} voxels={} prepare_us={prepare_us} update_us={update_us} dynamic_us={}", input.source_timestamp_ns, input.registered_points_map.len(), ray.map.voxels.len(), phase.elapsed().as_micros());
            }
            ray.dirty_since_candidate = true;
            // 每帧更新体素；构图只读本帧扫描，历史体素无需物化成点云。
            if !candidate_due(
                ray.last_candidate_received_at_ns,
                input.received_at_ns,
                candidate_interval_ns,
            ) {
                return Ok(None);
            }
        }
    }

    let latest_frame_occupancy = match kernel {
        MappingKernel::Voxel(voxel) => {
            let point_count = voxel.current_window.len();
            if !candidate_due(
                voxel.last_candidate_received_at_ns,
                input.received_at_ns,
                candidate_interval_ns,
            ) {
                return Ok(None);
            }
            let latest = build_occupancy(input, config)?;
            voxel.last_candidate_received_at_ns = input.received_at_ns;
            voxel.last_candidate_point_count = point_count;
            voxel.dirty_since_candidate = false;
            latest
        }
        MappingKernel::Raytracing(ray) => {
            let point_count = ray.map.healthy_count();
            if !candidate_due(
                ray.last_candidate_received_at_ns,
                input.received_at_ns,
                candidate_interval_ns,
            ) {
                return Ok(None);
            }
            let live = build_occupancy(input, config)?;
            ray.last_occupancy = Some(live.clone());
            ray.last_candidate_received_at_ns = input.received_at_ns;
            ray.last_candidate_point_count = point_count;
            ray.dirty_since_candidate = false;
            live
        }
    };

    Ok(Some(MappingBackendCandidate {
        #[cfg(test)]
        source_timestamp_ns: input.source_timestamp_ns,
        #[cfg(test)]
        received_at_ns: input.received_at_ns,
        #[cfg(test)]
        backend_kind,
        #[cfg(test)]
        live_window_occupancy: latest_frame_occupancy.clone(),
        latest_frame_occupancy,
        #[cfg(test)]
        accepted_points_map: input.registered_points_map.clone(),
    }))
}

pub(crate) fn cleanup_completed_checkpoint(
    payload: &[u8],
    q: f32,
    removed: &ahash::AHashSet<[i32; 3]>,
    retained: &[[i32; 3]],
    occupancy: OccupancyGrid,
) -> Result<Vec<u8>, String> {
    use nuwax_cloud_cleanup::offline_residual_cleanup::asset_key;
    if payload.len() < MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES {
        return Err("建图后端存档不完整".into());
    }
    let mut reader = MappingCheckpointReader::new(payload);
    let spec = match payload[12] {
        0 => MappingBackendSpec::Voxel,
        1 => MappingBackendSpec::Raytracing(Default::default()),
        _ => return Err("该地图没有点云建图后端".into()),
    };
    decode_mapping_checkpoint_header(&mut reader, spec)?;
    let kernel = match spec {
        MappingBackendSpec::Voxel => {
            let capacity = reader.read_u64()? as usize;
            let count = reader.read_u64()? as usize;
            let _old_count = reader.read_u64()?;
            let stamp = reader.read_u64()?;
            let mut points = VecDeque::new();
            for _ in 0..count {
                let p = Point3::new(reader.read_f64()?, reader.read_f64()?, reader.read_f64()?);
                if !removed.contains(&asset_key([p.x, p.y, p.z], q)) {
                    points.push_back(p);
                }
            }
            reader.finish()?;
            // 窗口格计数与恢复检查点时相同，按清理后保留的窗口点重建。
            let points: VoxelWindow = points.into_iter().collect();
            let window_cells = window_cells(&points);
            MappingKernel::Voxel(VoxelKernel {
                last_candidate_point_count: points.len(),
                window_cells,
                window_point_indices: window_point_indices(&points),
                current_window: points,
                capacity,
                last_candidate_received_at_ns: stamp,
                dirty_since_candidate: false,
                dufo: Default::default(),
            })
        }
        MappingBackendSpec::Raytracing(_) => {
            let c = decode_ray_config(&mut reader)?;
            let config = Config {
                voxel_size: c.voxel_size,
                max_range: c.max_range,
                ray_subsample: c.ray_subsample,
                shadow_depth: c.shadow_depth,
                grace_depth: c.grace_depth,
                min_health: c.min_health,
                max_health: c.max_health,
                graze_cos: c.graze_cos,
                recency_window: c.recency_window,
                emit_every: c.emit_every,
                global_emit_every: c.global_emit_every,
                region_percentile: c.region_percentile,
            };
            let _old_count = reader.read_u64()?;
            let stamp = reader.read_u64()?;
            let map_len = reader.read_u64()? as usize;
            let occ_len = reader.read_u64()? as usize;
            let mut map = decode_voxel_map_checkpoint_for_config(
                reader.read_bytes(map_len)?,
                MAX_MAPPING_BACKEND_CHECKPOINT_BYTES / VOXEL_MAP_CHECKPOINT_ENTRY_BYTES,
                &config,
            )?;
            reader.read_bytes(occ_len)?;
            reader.finish()?;
            let inv = 1.0f32 / config.voxel_size;
            let ray_key = |k: &[i32; 3]| {
                let p = k.map(|v| ((v as f64 + 0.5) * f64::from(q)) as f32);
                (
                    (p[0] * inv).floor() as i32,
                    (p[1] * inv).floor() as i32,
                    (p[2] * inv).floor() as i32,
                )
            };
            let keep: ahash::AHashSet<_> = retained.iter().map(ray_key).collect();
            let cut: ahash::AHashSet<_> = removed
                .iter()
                .map(ray_key)
                .filter(|k| !keep.contains(k))
                .collect();
            map.voxels.retain(|k, _| !cut.contains(k));
            MappingKernel::Raytracing(RaytracingKernel {
                last_candidate_point_count: map.healthy_count(),
                map,
                config,
                prepared_update: None,
                input: Vec::new(),
                last_occupancy: Some(occupancy),
                last_candidate_received_at_ns: stamp,
                dirty_since_candidate: false,
                dufo: Default::default(),
            })
        }
        _ => unreachable!(),
    };
    export_mapping_kernel_checkpoint(&kernel)
}

#[cfg(test)]
fn compute_candidate(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
) -> Result<Option<MappingBackendCandidate>, String> {
    let mut staged = kernel.clone();
    let result = compute_candidate_in_place(&mut staged, input, config)?;
    *kernel = staged;
    Ok(result)
}
#[cfg(test)]
#[path = "mapping_backend_tests.rs"]
mod tests;

pub(crate) fn flush_candidate(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
) -> Result<Option<MappingBackendCandidate>, String> {
    #[cfg(test)]
    let backend_kind = kernel.kind();
    let latest_frame_occupancy = match kernel {
        MappingKernel::Voxel(voxel) => {
            if !voxel.dirty_since_candidate {
                return Ok(None);
            }
            if voxel.current_window.is_empty() {
                voxel.dirty_since_candidate = false;
                return Ok(None);
            }
            let latest = build_occupancy(input, config)?;
            voxel.last_candidate_received_at_ns = input.received_at_ns;
            voxel.last_candidate_point_count = voxel.current_window.len();
            voxel.dirty_since_candidate = false;
            latest
        }
        MappingKernel::Raytracing(ray) => {
            if !ray.dirty_since_candidate {
                return Ok(None);
            }
            let live = build_occupancy(input, config)?;
            ray.last_occupancy = Some(live.clone());
            ray.last_candidate_received_at_ns = input.received_at_ns;
            ray.last_candidate_point_count = ray.map.healthy_count();
            ray.dirty_since_candidate = false;
            live
        }
    };
    Ok(Some(MappingBackendCandidate {
        #[cfg(test)]
        source_timestamp_ns: input.source_timestamp_ns,
        #[cfg(test)]
        received_at_ns: input.received_at_ns,
        #[cfg(test)]
        backend_kind,
        #[cfg(test)]
        live_window_occupancy: latest_frame_occupancy.clone(),
        latest_frame_occupancy,
        #[cfg(test)]
        accepted_points_map: Vec::new(),
    }))
}

#[cfg(test)]
#[path = "mapping_backend_ray_count_equivalence_tests.rs"]
pub(crate) mod ray_count_equivalence_tests;

/// 目录读取边界用资产内的原配置校验结构；运行后端选择仍由启动配置决定。
pub(crate) fn validate_saved_checkpoint(payload: &[u8]) -> Result<(), String> {
    if payload.len() < MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES {
        return Err("建图后端检查点截断".into());
    }
    let mut reader = MappingCheckpointReader::new(payload);
    let (spec, capacity) = match payload[12] {
        MAPPING_BACKEND_KIND_VOXEL => {
            decode_mapping_checkpoint_header(&mut reader, MappingBackendSpec::Voxel)?;
            (
                MappingBackendSpec::Voxel,
                usize::try_from(reader.read_u64()?).map_err(|_| "后端窗口大小溢出")?,
            )
        }
        MAPPING_BACKEND_KIND_RAYTRACING => {
            decode_mapping_checkpoint_header(
                &mut reader,
                MappingBackendSpec::Raytracing(Default::default()),
            )?;
            let c = decode_ray_config(&mut reader)?;
            (
                MappingBackendSpec::raytracing(
                    c.voxel_size,
                    c.max_range,
                    c.ray_subsample,
                    c.shadow_depth,
                    c.grace_depth,
                    c.min_health,
                    c.max_health,
                    c.graze_cos,
                    c.recency_window,
                ),
                0,
            )
        }
        _ => return Err("未知建图后端检查点类型".into()),
    };
    restore_mapping_kernel_checkpoint(
        spec,
        MappingBackendComputeConfig {
            resolution: 0.05,
            initial_safe_radius_meters: 0.,
            window_capacity: capacity,
        },
        payload,
    )
    .map(|_| ())
}
