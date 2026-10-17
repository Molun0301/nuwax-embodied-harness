//! Bounded observations of one associated physical surface, never occupancy samples.
use crate::{surface_statistics::ObservationStatistics, Block};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Arc, OnceLock};

pub const MAX_LAYER_SAMPLES: usize = 32;
// 每条层记录至少拥有一个互不重叠的真实成员，结构上限取32³个块内体素。
// 层按细列保存，256条甚至不足以覆盖32×32的完整平地；实际资源仍由
// 原有4MiB块编码、样本数和读写缓存字节预算共同约束。
pub const MAX_LAYERS_PER_BLOCK: usize = 32 * 32 * 32;
pub const MAX_LAYER_MEMBERS: usize = 32;
pub type LayerObservations = BTreeMap<u16, LayerObservation>;

/// The original producer namespace. Legacy samples do not claim a known source.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LayerSource {
    #[default]
    Legacy,
    Lidar {
        backend: i32,
        runtime_generation: i64,
        sensor_frame_id: String,
    },
    Auxiliary {
        sensor_id: String,
        clock_id: String,
        runtime_generation: u64,
        session_generation: u64,
    },
}
impl LayerSource {
    pub fn is_legacy(&self) -> bool {
        matches!(self, Self::Legacy)
    }
    pub fn validate(&self) -> Result<(), String> {
        let valid = match self {
            Self::Legacy => true,
            Self::Lidar {
                backend,
                runtime_generation,
                sensor_frame_id,
            } => matches!(backend, 1..=3) && *runtime_generation > 0 && !sensor_frame_id.is_empty(),
            Self::Auxiliary {
                sensor_id,
                clock_id,
                runtime_generation,
                session_generation,
            } => {
                !sensor_id.trim().is_empty()
                    && !clock_id.trim().is_empty()
                    && *runtime_generation > 0
                    && *session_generation > 0
            }
        };
        if valid {
            Ok(())
        } else {
            Err("invalid admitted layer source".into())
        }
    }
    pub(crate) fn validate_sample(&self, timestamp: u64, sequence: u64) -> Result<(), String> {
        self.validate()?;
        if timestamp == 0 || (sequence == 0 && !matches!(self, Self::Auxiliary { .. })) {
            return Err("invalid admitted source timestamp or sequence".into());
        }
        Ok(())
    }
    pub(crate) fn resident_bytes(&self) -> usize {
        std::mem::size_of::<Self>()
            + 2 * std::mem::size_of::<usize>()
            + match self {
                Self::Legacy => 0,
                Self::Lidar {
                    sensor_frame_id, ..
                } => sensor_frame_id.capacity(),
                Self::Auxiliary {
                    sensor_id,
                    clock_id,
                    ..
                } => sensor_id.capacity() + clock_id.capacity(),
            }
    }
}
fn legacy_source() -> Arc<LayerSource> {
    static LEGACY: OnceLock<Arc<LayerSource>> = OnceLock::new();
    LEGACY.get_or_init(|| Arc::new(LayerSource::Legacy)).clone()
}

/// Last successfully captured original frame in exactly one producer namespace.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceWatermark {
    pub source: Arc<LayerSource>,
    pub source_timestamp_ns: u64,
    pub source_sequence: u64,
}
impl SourceWatermark {
    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.source.is_legacy() {
            return Err("legacy source cannot claim a watermark".into());
        }
        self.source
            .validate_sample(self.source_timestamp_ns, self.source_sequence)
    }
}

/// Identity is the admitted source sequence/time and original emitted-point index.
/// Coordinates are that admitted return, not a new sample made by resampling.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerSample {
    #[serde(default = "legacy_source")]
    pub source: Arc<LayerSource>,
    pub point: [f64; 3],
    pub source_sequence: u64,
    pub source_timestamp_ns: u64,
    pub point_index: u32,
}

/// A local surface patch at one fine XY column. Normal/association are supplied
/// by the capture owner; consumers never refit or merge neighbouring layers.
/// `samples` contains only retained inliers. Its statistics describe this bounded
/// population, not every historical return or calibrated sensor uncertainty.
#[derive(Clone, Debug, PartialEq)]
pub struct LayerObservation {
    /// Sorted exact occupied members in this block and XY column.
    pub members: Vec<u16>,
    /// Stable fitted plane at the fixed XY cell centre; not the observed mean.
    pub plane_origin: [f64; 3],
    /// Unit upward normal of the associated local plane.
    pub normal: [f64; 3],
    pub samples: Vec<LayerSample>,
}
/// Wire samples keep their historical numeric fields. Source descriptors are
/// grouped once per layer and shared by all decoded samples in that group.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct NumericSample {
    point: [f64; 3],
    source_sequence: u64,
    source_timestamp_ns: u64,
    point_index: u32,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SampleSource {
    source: Arc<LayerSource>,
    sample_positions: Vec<u32>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WireLayer {
    members: Vec<u16>,
    plane_origin: [f64; 3],
    normal: [f64; 3],
    samples: Vec<NumericSample>,
    #[serde(default)]
    sample_sources: Vec<SampleSource>,
}
impl Serialize for LayerObservation {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::{SerializeSeq, SerializeStruct};
        struct Samples<'a>(&'a [LayerSample]);
        impl Serialize for Samples<'_> {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                let mut seq = serializer.serialize_seq(Some(self.0.len()))?;
                for sample in self.0 {
                    seq.serialize_element(&NumericSample {
                        point: sample.point,
                        source_sequence: sample.source_sequence,
                        source_timestamp_ns: sample.source_timestamp_ns,
                        point_index: sample.point_index,
                    })?;
                }
                seq.end()
            }
        }
        let mut groups: BTreeMap<&Arc<LayerSource>, Vec<u32>> = BTreeMap::new();
        for (position, sample) in self.samples.iter().enumerate() {
            if !sample.source.is_legacy() {
                groups
                    .entry(&sample.source)
                    .or_default()
                    .push(position as u32);
            }
        }
        #[derive(Serialize)]
        struct Group<'a> {
            source: &'a LayerSource,
            sample_positions: &'a [u32],
        }
        let groups: Vec<_> = groups
            .iter()
            .map(|(source, positions)| Group {
                source: source.as_ref(),
                sample_positions: positions,
            })
            .collect();
        let mut layer =
            serializer.serialize_struct("LayerObservation", 4 + usize::from(!groups.is_empty()))?;
        layer.serialize_field("members", &self.members)?;
        layer.serialize_field("plane_origin", &self.plane_origin)?;
        layer.serialize_field("normal", &self.normal)?;
        layer.serialize_field("samples", &Samples(&self.samples))?;
        if !groups.is_empty() {
            layer.serialize_field("sample_sources", &groups)?;
        }
        layer.end()
    }
}
impl<'de> Deserialize<'de> for LayerObservation {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        use serde::de::Error;
        let wire = WireLayer::deserialize(deserializer)?;
        let mut samples: Vec<_> = wire
            .samples
            .into_iter()
            .map(|sample| LayerSample {
                source: legacy_source(),
                point: sample.point,
                source_sequence: sample.source_sequence,
                source_timestamp_ns: sample.source_timestamp_ns,
                point_index: sample.point_index,
            })
            .collect();
        let mut sources = BTreeSet::new();
        let mut positions = BTreeSet::new();
        for group in wire.sample_sources {
            group.source.validate().map_err(D::Error::custom)?;
            if group.source.is_legacy()
                || group.sample_positions.is_empty()
                || !sources.insert(group.source.clone())
            {
                return Err(D::Error::custom("invalid or duplicate layer source group"));
            }
            for position in group.sample_positions {
                if !positions.insert(position) {
                    return Err(D::Error::custom("overlapping layer source positions"));
                }
                let sample = samples
                    .get_mut(position as usize)
                    .ok_or_else(|| D::Error::custom("layer source position out of range"))?;
                sample.source = group.source.clone();
            }
        }
        Ok(Self {
            members: wire.members,
            plane_origin: wire.plane_origin,
            normal: wire.normal,
            samples,
        })
    }
}
pub(crate) fn layer_bytes(layer: &LayerObservation) -> usize {
    // Charge each immutable descriptor once per layer, never once per sample.
    // Across layers this intentionally overestimates shared Arc allocations.
    let mut seen = BTreeSet::new();
    let source_bytes = layer
        .samples
        .iter()
        .map(|sample| &sample.source)
        .filter(|source| !source.is_legacy() && seen.insert(Arc::as_ptr(source)))
        .map(|source| source.resident_bytes())
        .sum::<usize>();
    128 + layer.members.len() * 2 + layer.samples.len() * 64 + source_bytes
}

impl LayerObservation {
    pub fn statistics(&self) -> Result<ObservationStatistics, String> {
        let mut points = self.samples.iter();
        let first = points.next().ok_or("empty layer samples")?;
        let mut statistics =
            ObservationStatistics::from_point(first.point).map_err(|_| "invalid layer point")?;
        for point in points {
            statistics
                .push(point.point)
                .map_err(|_| "invalid layer statistics")?;
        }
        Ok(statistics)
    }
    pub fn representative(&self) -> Result<[f64; 3], String> {
        self.plane_origin
            .iter()
            .all(|v| v.is_finite())
            .then_some(self.plane_origin)
            .ok_or_else(|| "invalid fitted plane origin".into())
    }
    pub fn latest_timestamp_ns(&self) -> u64 {
        self.samples
            .iter()
            .map(|p| p.source_timestamp_ns)
            .max()
            .unwrap_or(0)
    }
    /// Only geometry used by planning invalidates the terrain. Sample scatter,
    /// extent, count and source metadata remain observation-only diagnostics.
    pub fn geometry_eq(&self, other: &Self) -> bool {
        self.members == other.members
            && self.normal == other.normal
            && self.plane_origin == other.plane_origin
    }
    pub fn height_at(&self, xy: [f64; 2]) -> Option<f64> {
        let p = self.representative().ok()?;
        let z = p[2]
            - (self.normal[0] * (xy[0] - p[0]) + self.normal[1] * (xy[1] - p[1])) / self.normal[2];
        z.is_finite().then_some(z)
    }
}
pub fn layer_geometry_eq(a: &LayerObservations, b: &LayerObservations) -> bool {
    a.len() == b.len()
        && a.iter()
            .all(|(key, value)| b.get(key).is_some_and(|other| value.geometry_eq(other)))
}
pub(crate) fn validate_layers(layers: &LayerObservations, block: &Block) -> Result<(), String> {
    if layers.len() > MAX_LAYERS_PER_BLOCK {
        return Err("layer count exceeds block budget".into());
    }
    let mut claimed = std::collections::BTreeSet::new();
    for (&anchor, layer) in layers {
        if layer.members.is_empty()
            || layer.members.len() > MAX_LAYER_MEMBERS
            || layer.samples.is_empty()
            || layer.samples.len() > MAX_LAYER_SAMPLES
            || layer.members[0] != anchor
            || anchor >= 32768
            || layer.members.windows(2).any(|w| w[0] >= w[1])
            || layer.members.iter().any(|member| {
                !block.contains_key(member)
                    || member % 1024 != anchor % 1024
                    || !claimed.insert(*member)
            })
        {
            return Err("invalid or overlapping layer members".into());
        }
        if !layer.normal.iter().all(|v| v.is_finite()) {
            return Err("invalid layer normal".into());
        }
        let mut identities = std::collections::BTreeSet::new();
        for sample in &layer.samples {
            sample
                .source
                .validate_sample(sample.source_timestamp_ns, sample.source_sequence)?;
            if !sample.point.iter().all(|v| v.is_finite())
                || !identities.insert((
                    sample.source.as_ref(),
                    sample.source_sequence,
                    sample.source_timestamp_ns,
                    sample.point_index,
                ))
            {
                return Err("invalid or duplicate admitted layer sample".into());
            }
        }
        layer.representative()?;
        if layer.latest_timestamp_ns()
            > layer
                .members
                .iter()
                .map(|k| block[k].observed_ns)
                .max()
                .unwrap_or(0)
        {
            return Err("layer observation exceeds occupancy evidence".into());
        }
    }
    Ok(())
}
