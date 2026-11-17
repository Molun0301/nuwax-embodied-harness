    use super::*;

    #[test]
    fn module_internal_native_transport_timeout_and_all_tick_errors_are_fatal() {
        use nuwax_nav_base::{NativeProcessError, NavError};

        for error in [
            NavError::Internal("planner invariant".into()),
            NavError::Transport("planner transport closed".into()),
            NavError::Timeout {
                operation: "planner-input",
                millis: 10,
            },
            NavError::NativeProcess(NativeProcessError::ExecutableMissing {
                path: std::path::PathBuf::from("/missing/planner"),
            }),
        ] {
            assert!(module_input_error::<()>("planner-input", error).is_err());
        }
        assert!(matches!(
            module_input_error::<()>(
                "planner-input",
                NavError::InvalidInput {
                    field: "pose",
                    reason: "non-finite".into(),
                },
            )
            .expect("external module input is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let tick_error = module_tick_error(
            "planner.tick",
            NavError::InvalidInput {
                field: "module",
                reason: "unexpected tick state".into(),
            },
        );
        assert!(matches!(tick_error, NavStackError::Internal(_)));
    }

    #[test]
    fn planner_sensor_validation_rejects_frame_and_nonfinite() {
        let mut pose = schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 40,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        };
        let foreign_frame = schema::encode_pose_stamped_body(&pose).expect("encode foreign pose");
        assert!(matches!(
            decode_planner_pose_input(&foreign_frame, "planner.map_pose")
                .expect("foreign frame is external"),
            OwnerInput::Rejected(_)
        ));

        pose.header.frame_id = "map".into();
        pose.pose.position[1] = f64::INFINITY;
        let non_finite = schema::encode_pose_stamped_body(&pose).expect("encode non-finite pose");
        assert!(matches!(
            decode_planner_pose_input(&non_finite, "planner.map_pose")
                .expect("non-finite pose is external"),
            OwnerInput::Rejected(_)
        ));

    }
