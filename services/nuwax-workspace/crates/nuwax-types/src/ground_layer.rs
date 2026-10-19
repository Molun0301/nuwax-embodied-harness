//! Shared interpretation of an already associated legacy height band.
//!
//! This is geometric compatibility, not point statistics or surface segmentation.
//! Callers must retain separate bands/layers and all original collision geometry.

/// What occupied samples mean; this is independent of robot step capability.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum GroundEvidence {
    /// Samples of a measured surface, after association to one physical layer.
    #[default]
    MeasuredSurface,
    /// A solid volume whose contact candidate is its exposed top.
    OccupiedVolume,
}

/// Explicit provenance: legacy bands cannot supply a point mean or sample count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroundApproximation {
    /// Representative plane from bounded, associated actual return samples.
    ObservedLayer,
    /// 源端同一 XY 列的固定地表高度；成员和可选方向仍由原始源记录拥有。
    ObservedHeightfield,
    /// Midpoint of the quantized contact-height interval, not an observed mean.
    LegacyContactInterval,
    /// Exposed top of an occupied volume, not a measured surface statistic.
    ExposedVolumeTop,
}

/// One layer candidate. Its identity and geometry do not depend on body pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GroundLayer {
    /// Complete source occupancy; never replace collision geometry with the midpoint.
    pub occupied_interval: [f64; 2],
    /// Possible contact tops: lower occupied bound plus one source quantum to top.
    pub contact_interval: [f64; 2],
    pub representative_z: f64,
    pub evidence: GroundEvidence,
    pub approximation: GroundApproximation,
}

impl GroundLayer {
    /// Interpret a band already associated by the source to a single layer.
    /// Continuity alone does not establish that a wall or several surfaces are ground.
    /// No averaging across columns, disconnected bands, steps or floors occurs here.
    pub fn from_legacy_band(
        occupied_interval: [f64; 2],
        source_quantum: f64,
        evidence: GroundEvidence,
    ) -> Option<Self> {
        let [bottom, top] = occupied_interval;
        if !bottom.is_finite() || !top.is_finite() || bottom >= top
            || !source_quantum.is_finite() || source_quantum <= 0.0 {
            return None;
        }
        let (contact_interval, approximation) = match evidence {
            GroundEvidence::MeasuredSurface => (
                [(bottom + source_quantum).min(top), top],
                GroundApproximation::LegacyContactInterval,
            ),
            GroundEvidence::OccupiedVolume => ([top, top], GroundApproximation::ExposedVolumeTop),
        };
        // Difference form avoids overflow for same-sign finite coordinates.
        let representative_z = contact_interval[0]
            + (contact_interval[1] - contact_interval[0]) * 0.5;
        Some(Self { occupied_interval, contact_interval, representative_z, evidence, approximation })
    }

    /// Exact occupied voxel run. Adding one is done in f64 to avoid i32 overflow.
    pub fn from_voxel_run(run: [i32; 2], quantum: f64, evidence: GroundEvidence) -> Option<Self> {
        if run[0] > run[1] { return None; }
        Self::from_legacy_band(
            [f64::from(run[0]) * quantum, (f64::from(run[1]) + 1.0) * quantum],
            quantum, evidence,
        )
    }

    /// Rank against a fixed terrain reference, never a trial body-height offset.
    pub fn distance_to_reference(&self, reference_z: f64) -> f64 {
        (self.contact_interval[0] - reference_z)
            .max(reference_z - self.contact_interval[1]).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_contact_midpoint_corrects_half_voxel_without_claiming_point_mean() {
        let layer=GroundLayer::from_voxel_run([0,2],0.05,GroundEvidence::MeasuredSurface).unwrap();
        assert!((layer.representative_z-0.1).abs()<1e-12);
        assert!((layer.representative_z-(0.0+0.15)*0.5-0.025).abs()<1e-12);
        assert_eq!(layer.approximation,GroundApproximation::LegacyContactInterval);
        assert_eq!(layer.contact_interval[0],0.05);
        assert_eq!(layer.occupied_interval[0],0.0);
    }

    #[test]
    fn volume_keeps_exposed_top_and_layers_are_never_averaged() {
        let lower=GroundLayer::from_voxel_run([-3,-1],0.05,GroundEvidence::MeasuredSurface).unwrap();
        let upper=GroundLayer::from_voxel_run([17,19],0.05,GroundEvidence::MeasuredSurface).unwrap();
        assert!((lower.representative_z+0.05).abs()<1e-12);
        assert!((upper.representative_z-0.95).abs()<1e-12);
        let volume=GroundLayer::from_voxel_run([-3,-1],0.05,GroundEvidence::OccupiedVolume).unwrap();
        assert_eq!(volume.representative_z,0.0);
        assert_eq!(volume.contact_interval,[0.0,0.0]);
        assert_eq!(volume.occupied_interval,lower.occupied_interval);
    }

    #[test]
    fn invalid_bands_are_unknown_and_extreme_voxel_indices_do_not_overflow() {
        for band in [[0.0,0.0],[1.0,0.0],[f64::NAN,1.0],[0.0,f64::INFINITY]] {
            assert!(GroundLayer::from_legacy_band(band,0.05,GroundEvidence::MeasuredSurface).is_none());
        }
        for z in [i32::MIN,i32::MAX] {
            let layer=GroundLayer::from_voxel_run([z,z],0.05,GroundEvidence::MeasuredSurface).unwrap();
            assert!(layer.representative_z.is_finite());
        }
    }
}
