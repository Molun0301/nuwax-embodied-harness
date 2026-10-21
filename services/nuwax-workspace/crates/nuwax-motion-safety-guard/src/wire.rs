//! Generated protobuf types for the authoritative motion-guard UDS contract.

#![allow(missing_docs)]

/// Version carried by every accepted request and response.
pub const API_VERSION: u32 = 2;

include!(concat!(env!("OUT_DIR"), "/nuwax.motion_guard.v2.rs"));

#[cfg(test)]
mod tests {
    use prost::Message;

    use super::*;

    #[test]
    fn typed_twist_request_roundtrips_with_monotonic_deadline() {
        let request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: "request-1".to_string(),
            operation: Some(motion_guard_request::Operation::SubmitMotionCommand(
                SubmitMotionCommandRequest {
                    envelope: Some(RequestEnvelope {
                        tuple: Some(LeaseTuple {
                            generation: 9,
                            boot_nonce: vec![7; 16],
                            lease_id: 42,
                        }),
                        source: GuardSource::Nav as i32,
                        action: GuardAction::Twist as i32,
                        operation_id: "operation-1".to_string(),
                        payload_sha256: vec![3; 32],
                        sequence: 8,
                        issued_monotonic_ns: 1_000_000,
                        deadline_monotonic_ns: 1_300_000,
                        motion_stop_generation: 0,
                    }),
                    command: Some(submit_motion_command_request::Command::Twist(
                        TwistCommand {
                            linear_x_mps: 0.4,
                            linear_y_mps: -0.1,
                            angular_z_rps: 0.2,
                        },
                    )),
                },
            )),
        };

        let encoded = request.encode_to_vec();
        let decoded = MotionGuardRequest::decode(encoded.as_slice()).unwrap();
        let motion_guard_request::Operation::SubmitMotionCommand(command) =
            decoded.operation.unwrap()
        else {
            panic!("wrong request oneof variant");
        };
        let envelope = command.envelope.unwrap();
        assert_eq!(envelope.issued_monotonic_ns, 1_000_000);
        assert_eq!(envelope.deadline_monotonic_ns, 1_300_000);
        assert!(matches!(
            command.command,
            Some(submit_motion_command_request::Command::Twist(_))
        ));
    }

    #[test]
    fn typed_fence_response_carries_zero_ack() {
        let response = MotionGuardResponse {
            measured_motion_feedback: None,
            api_version: API_VERSION,
            request_id: "request-2".to_string(),
            outcome: Some(motion_guard_response::Outcome::RevokeAndZero(
                RevokeAndZeroResponse {
                    result: Some(GuardResult {
                        ack: AckCode::ResourceFenced as i32,
                        lease: LeaseState::Idle as i32,
                        error: None,
                    }),
                    zero_ack: Some(ZeroAck {
                        zero_ack_id: "zero-17".to_string(),
                        applied_monotonic_ns: 44,
                        ..Default::default()
                    }),
                },
            )),
        };

        let decoded = MotionGuardResponse::decode(response.encode_to_vec().as_slice()).unwrap();
        let motion_guard_response::Outcome::RevokeAndZero(fence) = decoded.outcome.unwrap() else {
            panic!("wrong response oneof variant");
        };
        assert_eq!(fence.zero_ack.unwrap().zero_ack_id, "zero-17");
        assert_eq!(fence.result.unwrap().ack, AckCode::ResourceFenced as i32);
    }

    #[test]
    fn typed_global_stop_roundtrips_without_a_lease_tuple() {
        let request = MotionGuardRequest {
            api_version: API_VERSION,
            request_id: "request-stop".to_string(),
            operation: Some(motion_guard_request::Operation::StopAllMotion(
                StopAllMotionRequest {
                    envelope: Some(GlobalStopEnvelope {
                        source: GuardSource::GatewayManual as i32,
                        operation_id: "operation-stop".to_string(),
                        payload_sha256: vec![5; 32],
                        issued_monotonic_ns: 10,
                        deadline_monotonic_ns: 20,
                    }),
                    reason: "operator requested stop".to_string(),
                    mode: GlobalStopMode::Ordinary as i32,
                    recovery_correlation: String::new(),
                    recovery_fence_token: Vec::new(),
                },
            )),
        };

        let decoded = MotionGuardRequest::decode(request.encode_to_vec().as_slice()).unwrap();
        let motion_guard_request::Operation::StopAllMotion(stop) = decoded.operation.unwrap()
        else {
            panic!("wrong request oneof variant");
        };
        let envelope = stop.envelope.unwrap();
        assert_eq!(envelope.source, GuardSource::GatewayManual as i32);
        assert_eq!(envelope.operation_id, "operation-stop");
    }

    #[test]
    fn schema_contains_no_opaque_actuator_payloads() {
        let schema = include_str!("../../../../navigation/proto/motion_guard.proto");
        assert!(!schema.contains("bytes command"));
        assert!(!schema.contains("operation_payload"));
        for message in [
            "TwistCommand",
            "StopCommand",
            "BodyPoseCommand",
            "SpeedLevelCommand",
            "ObstaclePolicyCommand",
            "SportActionCommand",
            "ModeSwitchCommand",
        ] {
            assert!(schema.contains(&format!("message {message}")));
        }
    }

    #[test]
    fn rust_wire_contract_is_v2_without_retired_gate_operations() {
        let schema = include_str!("../../../../navigation/proto/motion_guard.proto");
        assert_eq!(API_VERSION, 2);
        assert!(schema.contains("package nuwax.motion_guard.v2;"));
        for retired_declaration in [
            "enum SafetyGate",
            "message LatchSafetyStopRequest",
            "message LatchSafetyStopResponse",
            "message ResetSafeStopRequest",
            "message ResetSafeStopResponse",
        ] {
            assert!(!schema.contains(retired_declaration));
        }
        assert!(!schema.contains("GUARD_SOURCE_ADMIN ="));
        assert!(schema.contains("StopAllMotionRequest stop_all_motion = 21;"));
        assert!(schema.contains("GetGuardStateRequest get_guard_state = 22;"));
    }
}
