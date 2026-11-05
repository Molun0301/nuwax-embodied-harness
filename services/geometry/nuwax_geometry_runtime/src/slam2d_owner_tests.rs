use super::*;
use nuwax_codec::lcm::planar::{self, PlanarScan};

#[test]
fn public_planar_input_accepts_frozen_wire_and_rejects_old_generation_wire() {
    let scan = PlanarScan {
        runtime_generation: 1,
        sensor_id: "laser".into(),
        source_sequence: 0,
        timestamp_ns: 1,
        clock_domain: "unix_ns".into(),
        clock_id: String::new(),
        frame_id: "laser".into(),
        calibration_ref: "calibrated-laser".into(),
        beam_count: 1,
        angular_layout: 1,
        angle_min_rad: 0.0,
        angle_increment_rad: 1.0,
        range_min_m: 0.1,
        range_max_m: 10.0,
        scan_duration_ns: 0,
        timestamp_quality: 1,
        time_offsets_quality: 0,
        timing_uncertainty_ns: -1,
        motion_compensated: false,
        ranges_f32_le: 1.0_f32.to_le_bytes().to_vec(),
        angles_f32_le: Vec::new(),
        beam_states_u8: vec![1],
        intensities_f32_le: Vec::new(),
        time_offsets_ns_le: Vec::new(),
    };
    let old = planar::encode(&scan).unwrap();
    let mut frozen = 0xdec4_0103_4b18_dbb5u64.to_be_bytes().to_vec();
    frozen.extend_from_slice(&old[8..12]);
    frozen.extend_from_slice(&old[20..]);
    assert_eq!(decode_input(&frozen).unwrap(), scan);
    assert!(decode_input(&old).is_err());
    frozen.push(0);
    assert!(decode_input(&frozen).is_err());
}
