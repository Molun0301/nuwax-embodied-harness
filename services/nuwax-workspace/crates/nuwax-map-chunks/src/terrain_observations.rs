//! Bounded observations of one associated physical surface, never occupancy samples.
use crate::{surface_statistics::ObservationStatistics, Block};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const MAX_LAYER_SAMPLES: usize = 32;
pub const MAX_LAYERS_PER_BLOCK: usize = 256;
pub const MAX_LAYER_MEMBERS: usize = 32;
pub type LayerObservations = BTreeMap<u16, LayerObservation>;

/// Identity is the admitted source sequence/time and original emitted-point index.
/// Coordinates are that admitted return, not a new sample made by resampling.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerSample {
    pub point: [f64; 3],
    pub source_sequence: u64,
    pub source_timestamp_ns: u64,
    pub point_index: u32,
}

/// A local surface patch at one fine XY column. Normal/association are supplied
/// by the capture owner; consumers never refit or merge neighbouring layers.
/// `samples` contains only retained inliers. Its statistics describe this bounded
/// population, not every historical return or calibrated sensor uncertainty.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerObservation {
    /// Sorted exact occupied members in this block and XY column.
    pub members: Vec<u16>,
    /// Stable fitted plane at the fixed XY cell centre; not the observed mean.
    pub plane_origin: [f64; 3],
    /// Unit upward normal of the associated local plane.
    pub normal: [f64; 3],
    pub samples: Vec<LayerSample>,
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
            if sample.source_sequence == 0
                || sample.source_timestamp_ns == 0
                || !sample.point.iter().all(|v| v.is_finite())
                || !identities.insert((
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
