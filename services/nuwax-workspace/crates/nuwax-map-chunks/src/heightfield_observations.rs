//! Heightfield-v1 statistics for returns already associated with one surface.
//!
//! This module does not classify returns, grant contact exemptions, or admit
//! source frames. The capture owner must use its existing source watermark and
//! supply the whole associated XY column before splitting it across blocks.
//! Eight sub-bins per occupied member encode statistics, not finer measurements.
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const HEIGHT_SUB_BINS: usize = 8;

/// One column distribution, normalized across all members, including z-blocks.
/// Bounds describe exact member voxels; holes between members stay holes.
#[derive(Clone, Debug, PartialEq)]
pub struct HeightfieldColumn {
    xy: [i32; 2],
    voxel_size_m: f32,
    members: BTreeMap<i32, [f64; HEIGHT_SUB_BINS]>,
}

impl HeightfieldColumn {
    /// Build one frame from the capture owner's already-keyed, already-associated
    /// member/sub-bin counts. All z-block fragments of this XY must be assembled
    /// before this call; no point-to-voxel mapping or source admission is repeated.
    /// Counts use the original u32 emitted-point-index domain.
    pub fn from_counts(
        xy: [i32; 2],
        voxel_size_m: f32,
        counts: BTreeMap<i32, [u32; HEIGHT_SUB_BINS]>,
    ) -> Result<Option<Self>, String> {
        check_resolution(voxel_size_m)?;
        let mut members = counts
            .into_iter()
            .filter(|(_, counts)| counts.iter().any(|n| *n != 0))
            .map(|(z, counts)| (z, counts.map(f64::from)))
            .collect();
        if !normalize(&mut members) {
            return Ok(None);
        }
        Ok(Some(Self {
            xy,
            voxel_size_m,
            members,
        }))
    }

    // Complete native fragment assembly is a wire/source boundary, not a query.
    pub(crate) fn from_normalized_members(
        xy: [i32; 2], voxel_size_m: f32,
        members: BTreeMap<i32, [f64; HEIGHT_SUB_BINS]>,
    ) -> Result<Self, String> {
        check_resolution(voxel_size_m)?;
        let total: f64 = members.values().flatten().sum();
        let rounding = (members.len() * HEIGHT_SUB_BINS + 1) as f64 * f64::EPSILON;
        if members.is_empty() || members.values().flatten().any(|v| !v.is_finite() || *v < 0.0)
            || !total.is_finite() || total <= 0.0 || (total - 1.0).abs() > rounding {
            return Err("heightfield mass is not column-normalized".into());
        }
        Ok(Self { xy, voxel_size_m, members })
    }

    pub fn xy(&self) -> [i32; 2] {
        self.xy
    }
    pub fn voxel_size_m(&self) -> f32 {
        self.voxel_size_m
    }
    pub fn members(&self) -> impl ExactSizeIterator<Item = (i32, &[f64; HEIGHT_SUB_BINS])> {
        self.members.iter().map(|(&z, mass)| (z, mass))
    }

    /// Initial temporal policy: equal old/new frame weight, never per-member votes.
    /// The caller has already established compatible surface association and
    /// admitted a genuinely new source frame. This is not an idempotence wrapper.
    pub fn merge_frame(&self, frame: &Self) -> Result<Self, String> {
        if self.xy != frame.xy || self.voxel_size_m != frame.voxel_size_m {
            return Err("heightfield frame changes column or resolution".into());
        }
        let mut members = self.members.clone();
        for mass in members.values_mut() {
            for value in mass {
                *value *= 0.5;
            }
        }
        for (&z, incoming) in &frame.members {
            let mass = members.entry(z).or_default();
            for (old, new) in mass.iter_mut().zip(incoming) {
                *old += 0.5 * new;
            }
        }
        // A valid incoming frame always supplies positive mass. Zero-mass old
        // members remain exact ownership records until occupancy removes them.
        normalize(&mut members);
        Ok(Self {
            xy: self.xy,
            voxel_size_m: self.voxel_size_m,
            members,
        })
    }

    /// Withdraw only members removed from the owner's actual occupancy.
    /// If no statistical mass survives, there is no height record (None), not
    /// height zero. This never deletes the caller's underlying occupancy.
    pub fn retain_members(&self, occupied: &BTreeSet<i32>) -> Option<Self> {
        let mut members = self
            .members
            .iter()
            .filter(|(z, _)| occupied.contains(z))
            .map(|(&z, mass)| (z, *mass))
            .collect();
        if !normalize(&mut members) {
            return None;
        }
        Some(Self {
            xy: self.xy,
            voxel_size_m: self.voxel_size_m,
            members,
        })
    }

    /// 10% mass trimmed from each tail, including partial bins. Only the
    /// representative is trimmed; no member or occupied bound is removed.
    /// Quantization is to q/8 bin centres and does not improve source accuracy.
    pub fn height(&self) -> f64 {
        let total: f64 = self.members.values().flat_map(|mass| mass.iter()).sum();
        let lower = total * 0.1;
        let upper = total * 0.9;
        let width = f64::from(self.voxel_size_m) / HEIGHT_SUB_BINS as f64;
        let (mut cumulative, mut retained, mut moment) = (0.0_f64, 0.0_f64, 0.0_f64);
        for (&z, mass) in &self.members {
            for (bin, &weight) in mass.iter().enumerate() {
                let next = cumulative + weight;
                let keep = (next.min(upper) - cumulative.max(lower)).max(0.0);
                let centre = (f64::from(z) * HEIGHT_SUB_BINS as f64 + bin as f64 + 0.5) * width;
                moment += keep * centre;
                retained += keep;
                cumulative = next;
            }
        }
        let mean = moment / retained;
        // Quantize in world coordinates, without a previous-output deadband that
        // could permanently hide accumulating real changes.
        // Half-open world bins [k*w, (k+1)*w): exact boundaries, including
        // +/-zero, always choose the upper bin's centre. No zero-specific tie.
        ((mean / width).floor() + 0.5) * width
    }

    pub fn occupied_bounds(&self) -> [f64; 2] {
        let q = f64::from(self.voxel_size_m);
        [
            f64::from(*self.members.first_key_value().unwrap().0) * q,
            (f64::from(*self.members.last_key_value().unwrap().0) + 1.0) * q,
        ]
    }

    /// Future consumers depend on height, exact members and their bounds, not
    /// every statistical-weight update. This does not claim source freshness.
    pub fn geometry_eq(&self, other: &Self) -> bool {
        self.xy == other.xy
            && self.voxel_size_m == other.voxel_size_m
            && self.members.keys().eq(other.members.keys())
            && self.height() == other.height()
            && self.occupied_bounds() == other.occupied_bounds()
    }
}

fn check_resolution(q: f32) -> Result<(), String> {
    if !q.is_finite() || q <= 0.0 || q > 1_048_576.0 {
        return Err("invalid heightfield resolution".into());
    }
    Ok(())
}

fn normalize(members: &mut BTreeMap<i32, [f64; HEIGHT_SUB_BINS]>) -> bool {
    let total: f64 = members.values().flat_map(|mass| mass.iter()).sum();
    if total == 0.0 {
        return false;
    }
    for mass in members.values_mut() {
        for value in mass {
            *value /= total;
        }
    }
    true
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Representation {
    HeightfieldV1,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct WireColumn {
    representation: Representation,
    xy: [i32; 2],
    voxel_size_m: f32,
    // Sequence preserves duplicate detection; a JSON object map would overwrite.
    members: Vec<(i32, [f64; HEIGHT_SUB_BINS])>,
}

/// Standalone v1 payload. Not BL4, a root/archive version, or a wire activation.
/// The next integration must place it in the original versioned block protocol.
pub fn encode_heightfield_column(column: &HeightfieldColumn) -> Result<Vec<u8>, String> {
    let wire = WireColumn {
        representation: Representation::HeightfieldV1,
        xy: column.xy,
        voxel_size_m: column.voxel_size_m,
        members: column.members.iter().map(|(&z, mass)| (z, *mass)).collect(),
    };
    let bytes = serde_json::to_vec(&wire).map_err(|error| error.to_string())?;
    if bytes.len() > crate::MAX_BLOCK_BYTES {
        return Err("heightfield payload exceeds block budget".into());
    }
    Ok(bytes)
}

pub fn decode_heightfield_column(bytes: &[u8]) -> Result<HeightfieldColumn, String> {
    if bytes.len() > crate::MAX_BLOCK_BYTES {
        return Err("heightfield payload exceeds block budget".into());
    }
    let wire: WireColumn = serde_json::from_slice(bytes).map_err(|error| error.to_string())?;
    check_resolution(wire.voxel_size_m)?;
    if wire.members.is_empty()
        || wire.members.windows(2).any(|w| w[0].0 >= w[1].0)
        || wire
            .members
            .iter()
            .flat_map(|(_, mass)| mass.iter())
            .any(|v| !v.is_finite() || *v < 0.0)
    {
        return Err("invalid heightfield member distribution".into());
    }
    let total: f64 = wire.members.iter().flat_map(|(_, mass)| mass.iter()).sum();
    // A rounding bound for summing normalized f64 fractions, not a terrain gate.
    let rounding = (wire.members.len() * HEIGHT_SUB_BINS + 1) as f64 * f64::EPSILON;
    if !total.is_finite() || total <= 0.0 || (total - 1.0).abs() > rounding {
        return Err("heightfield mass is not column-normalized".into());
    }
    Ok(HeightfieldColumn {
        xy: wire.xy,
        voxel_size_m: wire.voxel_size_m,
        members: wire.members.into_iter().collect(),
    })
}

#[cfg(test)]
#[path = "heightfield_observations_tests.rs"]
mod tests;
