//! Canonical LiDAR/IMU measurement admission built on the NWS1 sensor frames.
//!
//! This module performs scan-window and identity admission only. It does not
//! decode device data, load external payloads, or call a LIO backend.

use super::sensors::{CanonicalCloudFrame, CanonicalImuFrame, SensorFrame};
use crate::{CodecError, CodecResult};

/// A LiDAR scan and the IMU samples that cover its complete time window.
#[derive(Debug, Clone, PartialEq)]
pub struct CanonicalLioMeasurement {
    /// The scan-start timestamp in the shared sensor clock.
    pub scan_begin_timestamp_ns: u64,
    /// The scan-end timestamp in the shared sensor clock.
    pub scan_end_timestamp_ns: u64,
    /// The canonical LiDAR cloud.
    pub cloud: CanonicalCloudFrame,
    /// Ordered IMU samples covering the complete scan window.
    pub imu_samples: Vec<CanonicalImuFrame>,
    /// Shared clock domain copied from the cloud and every IMU sample.
    pub clock_domain: String,
    /// NWS1 carries no clock ID; this field remains explicitly unavailable.
    pub clock_id: Option<String>,
    /// Shared registration target frame copied from all input identities.
    pub registration_frame_id: String,
    /// Shared registration state copied from all input identities.
    pub registration_state: String,
    /// Shared calibration reference copied from all input identities.
    pub calibration_ref: String,
    /// Provenance for the applied frame/calibration transform.
    pub transform_provenance: String,
}

/// Assemble one fail-closed canonical LiDAR/IMU measurement.
#[allow(clippy::too_many_arguments)]
pub fn assemble(
    cloud: CanonicalCloudFrame,
    imu_samples: Vec<CanonicalImuFrame>,
    scan_begin_timestamp_ns: u64,
    scan_end_timestamp_ns: u64,
    transform_provenance: impl Into<String>,
) -> CodecResult<CanonicalLioMeasurement> {
    if scan_end_timestamp_ns < scan_begin_timestamp_ns {
        return Err(error("scan end precedes scan begin"));
    }
    // Reuse the NWS1 validators without loading external payloads.
    super::sensors::encode(&SensorFrame::Cloud(cloud.clone()))?;
    for imu in &imu_samples {
        super::sensors::encode(&SensorFrame::Imu(imu.clone()))?;
    }
    if cloud.identity.timestamp_ns != scan_begin_timestamp_ns {
        return Err(error("cloud timestamp must equal scan begin"));
    }
    let cloud_end = cloud
        .identity
        .timestamp_ns
        .checked_add(cloud.scan_duration_ns)
        .ok_or_else(|| error("cloud scan end overflows timestamp"))?;
    if cloud_end != scan_end_timestamp_ns {
        return Err(error("cloud scan duration does not match scan end"));
    }
    if imu_samples.is_empty() {
        return Err(error("at least one IMU sample is required"));
    }
    let provenance = transform_provenance.into();
    let calibration_ref = cloud
        .identity
        .calibration_ref
        .as_deref()
        .ok_or_else(|| error("cloud calibration_ref is required"))?;
    let expected_provenance = format!("calibration_applied:{calibration_ref}");
    if provenance != expected_provenance {
        return Err(error("transform provenance is not an applied calibration"));
    }

    let first = &imu_samples[0];
    let last = imu_samples.last().expect("non-empty IMU samples");
    if first.identity.timestamp_ns > scan_begin_timestamp_ns {
        return Err(error("IMU samples do not cover scan begin"));
    }
    if last.identity.timestamp_ns < scan_end_timestamp_ns {
        return Err(error("IMU samples do not cover scan end"));
    }
    for pair in imu_samples.windows(2) {
        let previous = &pair[0].identity;
        let current = &pair[1].identity;
        if current.source_sequence <= previous.source_sequence {
            return Err(error("IMU source sequence regressed"));
        }
        if current.timestamp_ns <= previous.timestamp_ns {
            return Err(error("IMU timestamp regressed"));
        }
    }

    for imu in &imu_samples {
        if imu.identity.clock_domain != cloud.identity.clock_domain
            || imu.identity.frame_id != cloud.identity.frame_id
            || imu.identity.registration_state != cloud.identity.registration_state
            || imu.identity.registration_frame_id != cloud.identity.registration_frame_id
            || imu.identity.calibration_ref != cloud.identity.calibration_ref
        {
            return Err(error("LiDAR and IMU identity fields do not match"));
        }
        if imu.identity.payload_ownership != super::sensors::PayloadOwnership::Inline {
            return Err(error("IMU payload ownership must be inline"));
        }
    }

    Ok(CanonicalLioMeasurement {
        scan_begin_timestamp_ns,
        scan_end_timestamp_ns,
        clock_domain: cloud.identity.clock_domain.clone(),
        clock_id: None,
        registration_frame_id: cloud.identity.registration_frame_id.clone(),
        registration_state: cloud.identity.registration_state.clone(),
        calibration_ref: calibration_ref.to_owned(),
        cloud,
        imu_samples,
        transform_provenance: provenance,
    })
}

fn error(message: &str) -> CodecError {
    CodecError::LcmEncode(message.to_owned())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lcm::sensors::PayloadOwnership;

    fn identity(sequence: u64, timestamp_ns: u64) -> super::super::sensors::SensorIdentity {
        super::super::sensors::SensorIdentity {
            schema_version: 1,
            sensor_id: "go2_l2".into(),
            source_sequence: sequence,
            timestamp_ns,
            clock_domain: "unix_ns".into(),
            frame_id: "go2_base_link".into(),
            registration_state: "body".into(),
            registration_frame_id: "go2_base_link".into(),
            payload_ownership: PayloadOwnership::Inline,
            calibration_ref: Some("go2-urdf-base-v1".into()),
        }
    }

    fn cloud() -> CanonicalCloudFrame {
        CanonicalCloudFrame {
            identity: identity(10, 100),
            point_count: 1,
            scan_duration_ns: 100,
            payload: super::super::sensors::CloudPayload::Inline {
                xyz_f32_le: vec![0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64],
                intensity_f32_le: None,
                ring_u16_le: None,
                time_offset_ns_le: Some(50_u64.to_le_bytes().to_vec()),
            },
        }
    }

    fn imu(sequence: u64, timestamp_ns: u64) -> CanonicalImuFrame {
        CanonicalImuFrame {
            identity: identity(sequence, timestamp_ns),
            accel_mps2: [0.0; 3],
            gyro_rps: [0.0; 3],
        }
    }

    #[test]
    fn assembles_when_imu_covers_scan_end() {
        let measurement = assemble(
            cloud(),
            vec![imu(1, 90), imu(2, 200)],
            100,
            200,
            "calibration_applied:go2-urdf-base-v1",
        )
        .unwrap();
        assert_eq!(measurement.imu_samples.len(), 2);
        assert_eq!(measurement.scan_end_timestamp_ns, 200);
        assert_eq!(measurement.clock_domain, "unix_ns");
        assert_eq!(measurement.clock_id, None);
        assert_eq!(measurement.registration_frame_id, "go2_base_link");
        assert_eq!(measurement.registration_state, "body");
        assert_eq!(measurement.calibration_ref, "go2-urdf-base-v1");
    }

    #[test]
    fn rejects_missing_scan_end_imu() {
        assert!(assemble(
            cloud(),
            vec![imu(1, 90), imu(2, 199)],
            100,
            200,
            "calibration_applied:go2-urdf-base-v1",
        )
        .is_err());
    }

    #[test]
    fn rejects_timestamp_regression() {
        assert!(assemble(
            cloud(),
            vec![imu(2, 150), imu(1, 200)],
            100,
            200,
            "calibration_applied:go2-urdf-base-v1",
        )
        .is_err());
    }

    #[test]
    fn rejects_frame_and_calibration_mismatch() {
        let mut bad = imu(2, 200);
        bad.identity.frame_id = "sensor".into();
        assert!(assemble(
            cloud(),
            vec![imu(1, 90), bad],
            100,
            200,
            "calibration_applied:go2-urdf-base-v1",
        )
        .is_err());
    }

    #[test]
    fn rejects_calibration_mismatch() {
        let mut bad = imu(2, 200);
        bad.identity.calibration_ref = Some("other-calibration".into());
        assert!(assemble(
            cloud(),
            vec![imu(1, 90), bad],
            100,
            200,
            "calibration_applied:go2-urdf-base-v1",
        )
        .is_err());
    }

    #[test]
    fn rejects_raw_transform_provenance() {
        assert!(assemble(
            cloud(),
            vec![imu(1, 90), imu(2, 200)],
            100,
            200,
            "raw_sensor_frame"
        )
        .is_err());
    }
}
