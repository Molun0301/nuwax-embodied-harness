// Actual NativeLioIngress::process_pair facts, not an optimization.
// Included inside lio_ingress::tests::native_self_filter_facts.
use super::*;
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Site {
    Pair,
    Common,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct FilterCall {
    site: Site,
    input: usize,
    output: usize,
    pose: [u64; 7],
    filter_owner: usize,
}

tokio::task_local! {
    static FILTER_CALLS: Rc<RefCell<Vec<FilterCall>>>;
}

fn pose_bits(pose: &Isometry3<f64>) -> [u64; 7] {
    let t = &pose.translation.vector;
    let q = &pose.rotation.coords;
    [t[0].to_bits(), t[1].to_bits(), t[2].to_bits(),
        q[0].to_bits(), q[1].to_bits(), q[2].to_bits(), q[3].to_bits()]
}

pub(crate) fn record(
    site: Site,
    input: usize,
    output: usize,
    pose: &Isometry3<f64>,
    filter: &crate::self_cloud::RobotSelfFilter,
) {
    // No scope means no record, point scan, pose extraction or allocation.
    let _ = FILTER_CALLS.try_with(|calls| calls.borrow_mut().push(FilterCall {
        site, input, output, pose: pose_bits(pose),
        filter_owner: filter as *const crate::self_cloud::RobotSelfFilter as usize,
    }));
}

struct CloseIngress(NativeLioIngress);
impl Drop for CloseIngress {
    fn drop(&mut self) {
        LioIngressSink::close(&self.0);
    }
}

#[derive(Clone, Copy, Debug)]
enum Case {
    MixedWithLaterOdometry,
    FullPoseExternal,
    DifferentPublicPose,
    EvictedPose,
    SingleExternal,
    AllSelf,
    SignedZeroPublicPose,
    QuaternionSignPublicPose,
    OneUlpPublicPose,
}

fn encoded_public_accept(ingress: &NativeLioIngress, envelope: &LioOdometryEnvelope) {
    let wire = encode_lio_odometry_envelope(envelope).expect("encode actual public odometry");
    let decoded = nuwax_codec::lcm::lio::decode_lio_odometry_envelope(&wire)
        .expect("decode actual public odometry");
    // Exercise the real public admission core, not accept_odometry_for_test.
    ingress.common.accept_public_odometry(decoded, envelope.source_timestamp_ns)
        .expect("admit encoded public odometry");
}

fn source_odometry(pose: &Isometry3<f64>) -> Odometry {
    let mut odometry = valid_odometry();
    odometry.pose.position = pose.translation.vector.into();
    let q = pose.rotation.quaternion();
    odometry.pose.orientation_xyzw = [q.i, q.j, q.k, q.w];
    odometry
}

fn point_bits(points: &[Point3<f64>]) -> Vec<[u64; 3]> {
    points.iter().map(|point| [point.x.to_bits(), point.y.to_bits(), point.z.to_bits()]).collect()
}

async fn run_case(backend_id: i32, case: Case) {
    let backend = RecordingBackend::default();
    let (handle, tasks) = crate::lcm_runtime::build_handle_for_tests_with_unit_transport(
        false,
        Some(crate::mapping_backend::MappingBackendSpec::Voxel),
        LcmTransport::new(Arc::new(backend.clone())),
    ).await.expect("owned in-process geometry fixture");

    // Include setup in the unwind boundary. Every clone, pin and lock created
    // here is dropped before cleanup; CloseIngress also closes during unwind.
    let business = std::panic::AssertUnwindSafe(async {
        assert_eq!(handle.transport_backend_name, "in-process-unit");
        assert!(!handle.lcm_backend_available);
        {
            let _lifecycle = handle.mapping_lifecycle_lock.lock().await;
            let mut mapping = handle.mapping_adapter.as_ref().expect("original mapping owner").lock();
            mapping.start_session("native-filter-facts", "native-filter-facts-map")
                .expect("start original mapping owner");
            mapping.feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
                .expect("establish identity map-from-odom anchor");
        }
        let extrinsic = BodyFromSensorTransform::from_json(if matches!(case, Case::FullPoseExternal) {
            r#"{"translation_m":[0.125,-0.25,0.0625],"rotation_row_major":[0,-1,0,1,0,0,0,0,1]}"#
        } else {
            r#"{"translation_m":[0,0,0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]}"#
        }).expect("explicit valid generic sensor extrinsic");
        let public_cloud = binding("/test/native-filter-facts/cloud", "nuwax_msgs.LioCloudEnvelope");
        let mut native = NativeLioIngress::with_runtime(
            DeviceLioIngressSpec::new_for_backend(backend_id, 7, "lidar", "base_link", "odom", "map")
                .expect("native source spec"),
            handle.clone(),
            binding("/test/native-filter-facts/odom", "nuwax_msgs.LioOdometryEnvelope"),
            public_cloud.clone(),
            binding("/test/native-filter-facts/tf", "tf2_msgs.TFMessage"),
            extrinsic.clone(), Duration::from_millis(1),
        ).expect("runtime-bound Native ingress");
        native.common.self_cloud_filter = Ok(Some(crate::self_cloud::tests::filter()));
        let native = CloseIngress(native);
        let ingress = &native.0;
        assert!(matches!(ingress.pair_state.lock().probe_phase, NativeProbePhase::Runtime));
        let paired_pose = if matches!(case, Case::FullPoseExternal) {
            Isometry3::from_parts(Translation3::new(1.25, -0.5, 0.375),
                UnitQuaternion::from_euler_angles(0.2, -0.3, 0.7))
        } else if matches!(case, Case::OneUlpPublicPose) {
            Isometry3::translation(1.0, 0.0, 0.0)
        } else { Isometry3::identity() };
        let source = ingress.common.prepare_legacy_odometry(
            1_000_000_000, source_odometry(&paired_pose),
        ).expect("prepare source envelope");
        let source_identity = SourceIdentity::from_odometry(&source);
        let pair_pose = odometry_pose_isometry(&source.odometry).expect("actual paired pose");
        let mut public_source = source.clone();
        match case {
            Case::DifferentPublicPose => public_source.odometry.pose.position = [2.0, 0.0, 0.0],
            Case::SignedZeroPublicPose => public_source.odometry.pose.position[0] = -0.0,
            Case::QuaternionSignPublicPose => {
                for component in &mut public_source.odometry.pose.orientation_xyzw {
                    *component = -*component;
                }
            }
            Case::OneUlpPublicPose => {
                public_source.odometry.pose.position[0] = f64::from_bits(1.0_f64.to_bits() + 1);
            }
            _ => {}
        }
        encoded_public_accept(ingress, &public_source);
        let public_pose = odometry_pose_isometry(&public_source.odometry).expect("actual public pose");
        if matches!(case, Case::SignedZeroPublicPose | Case::QuaternionSignPublicPose) {
            assert_eq!(pair_pose, public_pose, "nalgebra PartialEq is deliberately wider than reuse eligibility");
            assert_ne!(pose_bits(&pair_pose), pose_bits(&public_pose));
        }
        let correlated = CorrelatedOdometry {
            source_sequence: source_identity.source_sequence,
            source_timestamp_ns: source_identity.source_timestamp_ns,
        };
        let sensor_points = match case {
            Case::DifferentPublicPose => vec![Point3::new(0.0, 0.0, -0.2),
                Point3::new(2.0, 0.0, -0.2), Point3::new(4.0, 0.0, -0.2)],
            Case::FullPoseExternal => vec![Point3::new(2.0, 0.25, 0.125), Point3::new(3.0, -0.5, 0.25)],
            Case::SingleExternal => vec![Point3::new(2.0, 0.0, -0.2)],
            Case::AllSelf => vec![Point3::new(0.0, 0.0, -0.2)],
            _ => vec![Point3::new(2.0, 0.0, -0.2), Point3::new(0.0, 0.0, -0.2),
                Point3::new(4.0, 0.25, -0.2)],
        };
        let source_count = sensor_points.len();
        assert!(ingress.insert_odometry(NativeOdometrySample {
            identity: source_identity, odometry: source.odometry.clone(),
        }).is_none());
        let cloud_identity = ingress.common.reserve_legacy_cloud_identity(source_identity.source_timestamp_ns)
            .expect("reserve real cloud identity");
        let pair = ingress.insert_cloud(NativeCloudSample {
            identity: cloud_identity, source_point_count: source_count, sensor_points,
            registration_mode_token: ingress.common.capture_registration_mode_token(),
        }).expect("exact native pair from original pairing function");
        let transformed = transform_native_pair(&pair, &extrinsic).expect("frozen transform oracle");
        let survivor_indices: &[usize] = match case {
            Case::DifferentPublicPose => &[2],
            Case::FullPoseExternal => &[0, 1],
            Case::SingleExternal => &[0],
            Case::AllSelf => &[],
            _ => &[0, 2],
        };
        let expected_odom: Vec<_> = survivor_indices.iter().map(|&index| transformed[index]).collect();
        let expected_map = handle.mapping_adapter.as_ref().unwrap().lock()
            .register_odom_cloud_to_map(&expected_odom, "odom").expect("original map registration oracle");
        if matches!(case, Case::MixedWithLaterOdometry) {
            let later = ingress.common.prepare_legacy_odometry(
                source_identity.source_timestamp_ns + 1, source_odometry(&Isometry3::translation(30.0, 40.0, 0.0)),
            ).expect("prepare later actual odometry");
            encoded_public_accept(ingress, &later);
            assert_eq!(ingress.common.snapshot().latest_odometry.unwrap().source_sequence,
                source_identity.source_sequence + 1);
        }

        let calls = Rc::new(RefCell::new(Vec::new()));
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        FILTER_CALLS.scope(Rc::clone(&calls), async {
            if matches!(case, Case::AllSelf) {
                let error = ingress.process_pair(pair, &mut shutdown_rx).await
                    .expect_err("first-pass empty input keeps its original early error");
                assert_eq!(error, "本帧自体过滤后没有外部点云");
                assert!(!*ingress.source_pair_ready_rx.borrow(), "empty first pass remains before READY");
                let observed = calls.borrow();
                assert_eq!(observed.len(), 1);
                assert_eq!((observed[0].site, observed[0].input, observed[0].output), (Site::Pair, 1, 0));
                drop(observed);
                assert!(ingress.common.state.lock().pending_registrations.is_empty());
                assert!(backend.published.lock().unwrap().iter().all(|frame| frame.topic != public_cloud.0));
                return;
            }
            if matches!(case, Case::EvictedPose) {
                let lifecycle = handle.mapping_lifecycle_lock.lock().await;
                assert!(ingress.common.state.lock().accepted_odometry_poses
                    .contains_key(&(correlated.source_sequence, correlated.source_timestamp_ns)));
                let mut readiness_rx = shutdown_rx.clone();
                assert_eq!(ingress.common.wait_for_public_odometry(correlated, &mut readiness_rx)
                    .now_or_never(), Some(true), "real public wait is already Ready before polling process_pair");
                drop(readiness_rx);
                let mut process = Box::pin(ingress.process_pair(pair, &mut shutdown_rx));
                assert!(futures::poll!(process.as_mut()).is_pending(), "original lifecycle lock must suspend actual process_pair");
                assert!(*ingress.source_pair_ready_rx.borrow());
                assert_eq!(calls.borrow().len(), 1, "pair filtering ran before original lifecycle wait");
                assert_eq!(calls.borrow()[0].site, Site::Pair);
                // Synthetic lock interleaving with the real public admission core.
                // This does not claim 32 outer LCM callbacks can overtake Tokio's queued lock waiter.
                for offset in 1..=MAX_ACCEPTED_ODOMETRY_IDENTITIES {
                    let next = ingress.common.prepare_legacy_odometry(
                        source_identity.source_timestamp_ns + offset as i64,
                        source_odometry(&Isometry3::translation(offset as f64, 0.0, 0.0)),
                    ).expect("prepare newer real source identity");
                    encoded_public_accept(ingress, &next);
                }
                assert!(!ingress.common.state.lock().accepted_odometry_poses
                    .contains_key(&(correlated.source_sequence, correlated.source_timestamp_ns)));
                drop(lifecycle);
                let error = process.await.expect_err("exact frozen public pose was really evicted");
                assert_eq!(error, LioIngressError::MissingFrozenOdometry.to_string());
                assert_eq!(calls.borrow().len(), 1, "failure precedes Common filter");
                assert!(ingress.common.state.lock().pending_registrations.is_empty());
                assert!(backend.published.lock().unwrap().iter().all(|frame| frame.topic != public_cloud.0));
                return;
            }
            ingress.process_pair(pair, &mut shutdown_rx).await.expect("actual Runtime pair processing");
            let observed = calls.borrow();
            let fallback = matches!(case, Case::DifferentPublicPose | Case::SignedZeroPublicPose
                | Case::QuaternionSignPublicPose | Case::OneUlpPublicPose);
            let first_output = if matches!(case, Case::SingleExternal) { 1 } else { 2 };
            assert_eq!(observed.len(), if fallback { 2 } else { 1 });
            assert_eq!(observed[0].site, Site::Pair);
            assert_eq!((observed[0].input, observed[0].output), (source_count, first_output));
            assert_eq!(observed[0].pose, pose_bits(&pair_pose));
            if fallback {
                assert_eq!(observed[1].site, Site::Common);
                assert_eq!((observed[1].input, observed[1].output), (first_output, survivor_indices.len()));
                assert_eq!(observed[1].pose, pose_bits(&public_pose));
                assert_eq!(observed[0].filter_owner, observed[1].filter_owner);
                assert_ne!(observed[0].pose, observed[1].pose);
            } else {
                assert_eq!(pose_bits(&pair_pose), pose_bits(&public_pose));
                assert_eq!(observed[0].output, survivor_indices.len());
            }
            drop(observed);
            let frames: Vec<_> = backend.published.lock().unwrap().iter()
                .filter(|frame| frame.topic == public_cloud.0 && frame.msg_type == public_cloud.1)
                .cloned().collect();
            assert_eq!(frames.len(), 1, "one actual canonical cloud publication");
            assert_eq!(i64::from(frames[0].header.seqno), cloud_identity.source_sequence);
            let mut wire = nuwax_codec::lcm::lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1.to_vec();
            wire.extend_from_slice(&frames[0].payload);
            let accepted = ingress.common.accept_public_cloud(&wire, source_identity.source_timestamp_ns + 100)
                .expect("real canonical publication matches original pending permit");
            assert_eq!(accepted.envelope().source_point_count, source_count as i64);
            assert_eq!(accepted.envelope().emitted_point_count, survivor_indices.len() as i64);
            assert_eq!(accepted.source_identity(), cloud_identity);
            assert_eq!(accepted.binding().correlated_odometry, correlated);
            assert_eq!(pose_bits(&accepted.binding().frozen_odom_from_body), pose_bits(&public_pose));
            assert_eq!(point_bits(accepted.map_points()), point_bits(&expected_map.points),
                "all surviving coordinates and original order are unchanged");
            let emitted = parse_xyz_cloud_with_limit(&accepted.envelope().cloud, MAX_LIO_EMITTED_POINT_COUNT)
                .expect("actual wire coordinates");
            let expected_wire: Vec<_> = expected_map.points.iter().map(|p|
                Point3::new(p.x as f32 as f64, p.y as f32 as f64, p.z as f32 as f64)).collect();
            assert_eq!(point_bits(&emitted), point_bits(&expected_wire), "canonical f32 wire retains full survivor order");
            assert!(ingress.common.state.lock().pending_registrations.is_empty());
        }).await;
        eprintln!("native_filter_facts backend={backend_id} case={case:?} calls={:?}", calls.borrow());
    }).catch_unwind().await;

    // Business locals (including pinned process and original lock) have gone.
    // Stop the original adapter under the original lifecycle fence before
    // dropping the last caller handle and asking its task owner to join fanout.
    let cleanup = std::panic::AssertUnwindSafe(async {
        let _lifecycle = handle.mapping_lifecycle_lock.lock().await;
        let mut mapping = handle.mapping_adapter.as_ref().ok_or("missing original mapping owner")?.lock();
        let current = mapping.snapshot();
        if current.active {
            mapping.stop_session(&current.session_id).map_err(|error| error.to_string())?;
        }
        Ok::<(), String>(())
    }).catch_unwind().await;
    drop(handle);
    let task_cleanup = tasks.finish().await;
    if let Ok(Err(error)) = &cleanup { eprintln!("native filter fixture cleanup error: {error}"); }
    if cleanup.is_err() { eprintln!("native filter fixture cleanup panicked"); }
    if let Err(error) = &task_cleanup { eprintln!("native filter task cleanup error: {error}"); }
    if let Err(panic) = business { std::panic::resume_unwind(panic); }
    match cleanup {
        Ok(result) => result.expect("original mapping backend cleanup"),
        Err(panic) => std::panic::resume_unwind(panic),
    }
    task_cleanup.expect("owned fanout joined and fixture files retired");
}

#[tokio::test(flavor = "current_thread")]
async fn native_runtime_same_pose_reuses_filtered_survivors() {
    for backend in [LIO_BACKEND_FASTLIO2, LIO_BACKEND_POINTLIO] {
        run_case(backend, Case::MixedWithLaterOdometry).await;
        run_case(backend, Case::FullPoseExternal).await;
    }
}

#[tokio::test(flavor = "current_thread")]
async fn native_runtime_distinct_public_pose_preserves_both_classifications() {
    run_case(LIO_BACKEND_POINTLIO, Case::DifferentPublicPose).await;
}

#[tokio::test(flavor = "current_thread")]
async fn native_runtime_evicted_public_pose_rejects_before_second_filter() {
    run_case(LIO_BACKEND_POINTLIO, Case::EvictedPose).await;
}

#[tokio::test(flavor = "current_thread")]
async fn native_runtime_reuse_preserves_small_and_exact_pose_boundaries() {
    for case in [Case::SingleExternal, Case::AllSelf, Case::SignedZeroPublicPose,
        Case::QuaternionSignPublicPose, Case::OneUlpPublicPose] {
        run_case(LIO_BACKEND_POINTLIO, case).await;
    }
}
