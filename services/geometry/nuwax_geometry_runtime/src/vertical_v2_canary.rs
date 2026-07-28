use super::*;

use futures::StreamExt;
use nalgebra::Point3;
use nuwax_exploration::ExplorationBoundary;
use nuwax_nav_stack::{
    register_compiled_in_process_owners, ModuleGraph, NavStackBuilder, NavStackPreset,
    NavStackProfile, NavTopicConfig, StackRunner,
};
use nuwax_protocol::RawLcmFrame;
use std::ffi::OsString;
use std::sync::atomic::{AtomicBool, AtomicU64};
use std::sync::Mutex;

static ENV_LOCK: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

struct EnvRestore(Vec<(&'static str, Option<OsString>)>);

impl EnvRestore {
    fn set(values: &[(&'static str, &str)]) -> Self {
        let saved = values
            .iter()
            .map(|(key, _)| (*key, std::env::var_os(key)))
            .collect();
        for (key, value) in values {
            std::env::set_var(key, value);
        }
        Self(saved)
    }
}

impl Drop for EnvRestore {
    fn drop(&mut self) {
        for (key, value) in self.0.drain(..) {
            match value {
                Some(value) => std::env::set_var(key, value),
                None => std::env::remove_var(key),
            }
        }
    }
}

#[derive(Default)]
struct FailClosedNavSink {
    submissions: Mutex<Vec<crate::explore_runtime::ChildNavigationRequest>>,
}

#[async_trait]
impl crate::explore_runtime::ExploreNavigationSink for FailClosedNavSink {
    async fn submit(
        &self,
        request: crate::explore_runtime::ChildNavigationRequest,
    ) -> Result<
        crate::explore_runtime::ChildNavigationTicket,
        crate::explore_runtime::ChildNavigationSubmitError,
    > {
        self.submissions.lock().expect("submissions").push(request);
        panic!("out-of-boundary frontier must not reach navigation")
    }

    async fn cancel(
        &self,
        _lease: &crate::explore_runtime::ChildNavigationLease,
    ) -> Result<crate::explore_runtime::ChildNavigationCancelTicket, String> {
        panic!("fail-closed case must not own child navigation")
    }
}

fn typed_frame(topic: &str, message_type: &str, payload: Vec<u8>, seqno: u64) -> RawLcmFrame {
    crate::typed_codec::build_typed_frame(
        LcmTopic::new(topic).expect("canary topic"),
        LcmMessageType::new(message_type).expect("canary message type"),
        payload,
        seqno,
    )
}

fn canary_mapping() -> MappingAdapter {
    let mut adapter = MappingAdapter::new_with_frames(
        MappingAdapterConfig {
            resolution: 0.5,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-5.0, -5.0),
            voxel_size: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::General,
            initial_safe_radius_meters: 2.0,
        },
        "map",
        "odom",
    );
    adapter
        .start_session("v2-canary-mapping", "v2-canary-map")
        .expect("start V2 canary mapping session");
    adapter
}

fn simple_planner_runner(topics: &NavTopicConfig) -> StackRunner {
    let full_graph = NavStackBuilder::with_topics(NavStackProfile::Simple2D, topics.clone())
        .preset(NavStackPreset::None)
        .build()
        .expect("build production Simple2D graph");
    let mut planner = full_graph
        .find("simple_planner")
        .expect("Simple2D simple_planner node")
        .clone();
    planner.depends_on.clear();
    let mut graph = ModuleGraph::new(NavStackProfile::Simple2D.as_str());
    graph.push_node(planner);
    graph.validate().expect("reduced planner graph");
    let mut runner = StackRunner::from_graph(graph);
    register_compiled_in_process_owners(&mut runner);
    runner
}

async fn next_typed(
    stream: &mut nuwax_transport::LcmFrameStream,
    timeout: Duration,
) -> RawLcmFrame {
    tokio::time::timeout(timeout, stream.next())
        .await
        .expect("timed out waiting for V2 canary frame")
        .expect("V2 canary subscription closed")
        .expect("V2 canary transport error")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn vertical_v2_mapping_exploration_simple_planner_canary() {
    let _env_lock = ENV_LOCK.lock().await;
    let nonce = format!("{:x}", std::process::id());
    let provider = test_lcm_url();
    let occupancy_channel = format!("/v2/{nonce}/occ");
    let goal_channel = "/nav/goal".to_string();
    let path_channel = "/nav/path".to_string();
    let pose_channel = format!("/v2/{nonce}/pose");
    let stop_channel = format!("/v2/{nonce}/stop");
    let _env = EnvRestore::set(&[("NUWAX_LCM_URL", &provider)]);

    let publisher = LcmTransport::new_real(&provider).expect("real LCM publisher context");
    let observer = LcmTransport::new_real(&provider).expect("independent real LCM observer");
    assert_eq!(publisher.backend_name(), "real-lcm");
    assert_eq!(observer.backend_name(), "real-lcm");

    let occupancy_topic = LcmTopic::new(&occupancy_channel).expect("occupancy topic");
    let occupancy_type = LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy type");
    let goal_topic = LcmTopic::new(&goal_channel).expect("goal topic");
    let goal_type = LcmMessageType::new("nuwax_msgs.GoalEnvelope").expect("goal type");
    let path_topic = LcmTopic::new(&path_channel).expect("path topic");
    let path_type = LcmMessageType::new("nuwax_msgs.PathEnvelope").expect("path type");
    let mut occupancy_frames = observer
        .subscribe(occupancy_topic.clone(), Some(occupancy_type.clone()))
        .await;
    let mut goal_frames = observer
        .subscribe(goal_topic.clone(), Some(goal_type.clone()))
        .await;
    let mut path_frames = observer
        .subscribe(path_topic.clone(), Some(path_type.clone()))
        .await;

    let mut topics = NavTopicConfig::geometry_runtime_defaults();
    topics.set("occupancy", &occupancy_channel);
    topics.set("map_pose", &pose_channel);
    topics.set("goal_pose", &goal_channel);
    topics.set("stop_movement", &stop_channel);
    topics.set("goal_path", &path_channel);
    let mut runner = simple_planner_runner(&topics);
    assert_eq!(runner.unowned_nodes(), Vec::<String>::new());
    runner
        .spawn_all()
        .await
        .expect("start real simple_planner owner");
    assert_eq!(runner.running_nodes(), vec!["simple_planner"]);

    tokio::time::sleep(Duration::from_millis(75)).await;
    let state = Arc::new(RwLock::new(SharedState::default()));
    let mapping = Arc::new(parking_lot::Mutex::new(canary_mapping()));
    let (map_revision_tx, map_revision_rx) = broadcast::channel(4);
    let (map_revision_latest_tx, mut map_revision_latest_rx) = watch::channel(None);
    let bridge_map_revision_latest_tx = map_revision_latest_tx.clone();
    let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
    spawn_map_revision_fanout(
        map_revision_rx,
        map_revision_latest_tx,
        publisher.clone(),
        occupancy_topic.clone(),
        occupancy_type.clone(),
        None,
        None,
        Arc::new(RwLock::new(None)),
        explore.clone(),
        mapping.clone(),
        lifecycle_lock,
    );

    let source_timestamp_ns = crate::explore_runtime::now_unix_nanos();
    let event = {
        let mut adapter = mapping.lock();
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("production mapping pose API");
        let points = [
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.5, 0.5, 0.0),
            Point3::new(1.5, -0.5, 0.0),
        ];
        let outcome = ingest_mapping_points(
            &mut adapter,
            &points,
            12,
            source_timestamp_ns,
            "odom",
            source_timestamp_ns,
            &state,
            &map_revision_tx,
        )
        .expect("production pointcloud ingest API");
        assert!(outcome.occupancy_committed);
        assert_eq!(adapter.current_revision(), 1);
        MapRevisionEvent::from_map(
            &state.try_read().expect("committed map").map,
            &adapter.snapshot(),
        )
    };
    assert_eq!(event.revision, 1);
    assert_eq!(event.frame_id, "map");

    map_revision_latest_rx
        .changed()
        .await
        .expect("map fanout revision");
    let fanned_out = map_revision_latest_rx
        .borrow_and_update()
        .clone()
        .expect("committed map fanout event");
    assert_eq!(fanned_out.revision, event.revision);
    assert_eq!(fanned_out.frame_id, "map");
    let occupancy_frame = next_typed(&mut occupancy_frames, Duration::from_secs(3)).await;
    let occupancy = schema::decode_occupancy_grid_body(&occupancy_frame.payload)
        .expect("decode fanout occupancy");
    assert_eq!(occupancy.header.seq, event.revision as i32);
    assert_eq!(occupancy.header.frame_id, "map");

    let pose = schema::PoseStamped {
        header: schema::Header {
            seq: 1,
            stamp_sec: (source_timestamp_ns / 1_000_000_000) as i32,
            stamp_nsec: (source_timestamp_ns % 1_000_000_000) as i32,
            frame_id: "map".to_string(),
        },
        pose: schema::Pose {
            position: [0.0, 0.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
    };
    publisher
        .publish(typed_frame(
            &pose_channel,
            "geometry_msgs.PoseStamped",
            schema::encode_pose_stamped_body(&pose).expect("encode map pose"),
            1,
        ))
        .await
        .expect("publish planner map pose");

    {
        let mut shared = state.write().await;
        shared.pose = PoseState {
            x: 0.0,
            y: 0.0,
            qw: 1.0,
            frame_id: "map".to_string(),
            timestamp_ns: source_timestamp_ns,
            ..PoseState::default()
        };
    }
    let nav = crate::nav_runtime::NavRuntimeState::from_env();
    nav.observe_stack_state(NavState {
        status: "IDLE".to_string(),
        message: "V2 canary Simple2D owner active".to_string(),
        timestamp_ns: crate::explore_runtime::now_unix_nanos(),
        ..NavState::default()
    })
    .await;
    nav.observe_tf_chain(crate::nav_runtime::TfChainObservation {
        generation: 1,
        map_frame: "map".to_string(),
        odom_frame: "odom".to_string(),
        base_frame: "base_link".to_string(),
        map_to_odom: true,
        odom_to_base: true,
    })
    .expect("navigation TF readiness");
    nav.observe_planner_input(crate::nav_runtime::PlannerInputObservation {
        generation: 1,
        map_revision: event.revision,
        map_frame: "map".to_string(),
        pose_frame: "map".to_string(),
        ready: true,
    })
    .expect("navigation planner readiness");
    nav.observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
        generation: 1,
        ready: true,
    })
    .expect("navigation motion-sink readiness");

    let (cloud_map_tx, _) = broadcast::channel(4);
    let (nav_state_tx, _) = broadcast::channel(8);
    let (frontiers_tx, _) = broadcast::channel(4);
    let bridge = ExploreNavigationBridge {
        state: state.clone(),
        expected_map_frame: Arc::from("map"),
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx: map_revision_tx.clone(),
        map_revision_latest_tx: bridge_map_revision_latest_tx,
        transport: publisher.clone(),
        transport_backend_name: "real-lcm",
        lcm_backend_available: true,
        cmd_vel_sink_ready: Arc::new(AtomicBool::new(true)),
        nav: nav.clone(),
        runtime_observation_generation: Arc::new(AtomicU64::new(1)),
    };
    explore.set_navigation_sink(Arc::new(bridge));
    tokio::time::sleep(Duration::from_millis(75)).await;
    let parent_request = format!("v2-explore-{nonce}");
    let boundary = ExplorationBoundary::from_vertices(vec![
        Point2 { x: -2.0, y: -2.0 },
        Point2 { x: 2.0, y: -2.0 },
        Point2 { x: 2.0, y: 2.0 },
        Point2 { x: -2.0, y: 2.0 },
    ])
    .expect("V2 canary boundary");
    let started = explore
        .start_with_config(
            &parent_request,
            crate::explore_runtime::ExploreStartConfig {
                motion_admission: crate::nav_runtime::test_motion_admission(),
                boundary: Some(boundary),
                ..crate::explore_runtime::ExploreStartConfig::default()
            },
        )
        .await;
    assert_eq!(started.status, "ACCEPTED", "{}", started.message);
    let active = explore.tick((0.0, 0.0)).await;
    assert_eq!(active.status, "ACTIVE", "{}", active.message);
    assert!(active.child_lease.is_some());

    let goal_frame = next_typed(&mut goal_frames, Duration::from_secs(3)).await;
    let goal = schema::decode_goal_envelope_body(&goal_frame.payload).expect("decode child goal");
    let lease = nav.active_goal_lease().await.expect("active child lease");
    assert_eq!(
        goal.owner_kind,
        schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
    );
    assert_eq!(goal.parent_request_id, parent_request);
    assert_eq!(goal.session_id, lease.session_id);
    assert_eq!(goal.goal_generation, lease.goal_generation as i64);
    assert_eq!(
        goal.producer_runtime_generation,
        lease.producer_runtime_generation as i64
    );
    assert_eq!(goal.payload_sha256, lease.goal_payload_sha256);
    assert_eq!(goal.boundary_frame_id, "map");
    assert_eq!(goal.boundary_points.len(), 4);
    assert_eq!(goal.goal.header.frame_id, "map");

    let path_frame = next_typed(&mut path_frames, Duration::from_secs(5)).await;
    let path = schema::decode_path_envelope_body(&path_frame.payload).expect("decode planner path");
    assert!(!path.path.poses.is_empty());
    assert_eq!(path.path.header.frame_id, "map");
    assert_eq!(path.session_id, lease.session_id);
    assert_eq!(path.owner_kind, lease.owner_kind);
    assert_eq!(path.parent_request_id, lease.parent_request_id);
    assert_eq!(path.goal_generation, lease.goal_generation as i64);
    assert_eq!(path.goal_payload_sha256, lease.goal_payload_sha256);
    assert!(path.planner_runtime_generation > 0);
    assert!(path.path_revision > 0);
    assert_eq!(
        path.path_payload_sha256,
        schema::path_envelope_payload_sha256(&path).expect("path hash")
    );
    let accepted_path_lease = nav
        .observe_path_envelope(&path)
        .await
        .expect("correlated PathEnvelope accepted");
    assert_eq!(accepted_path_lease.path_revision, path.path_revision as u64);

    let mut wrong_identity = path.clone();
    wrong_identity.session_id.push_str("-stale");
    schema::seal_path_envelope(&mut wrong_identity).expect("reseal wrong-identity path");
    assert!(nav.observe_path_envelope(&wrong_identity).await.is_err());
    assert_eq!(explore.snapshot().status, "ACTIVE");
    runner
        .ensure_healthy()
        .expect("simple_planner owner healthy");

    let first_child_id = active.child_task_id.clone();
    let first_nav_lease = nav
        .active_goal_lease()
        .await
        .expect("active terminal lease");
    let terminal_update = NavigationUpdate::from_local(
        NavState {
            status: "SUCCEEDED".to_string(),
            message: "V2 canary reached".to_string(),
            goal_id: first_child_id.clone(),
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            ..NavState::default()
        },
        &nav,
        Some(first_nav_lease),
    )
    .await;
    let applied_terminal = apply_typed_navigation_feedback(&nav, terminal_update)
        .await
        .expect("apply correlated terminal feedback");
    report_exploration_navigation_terminal(&explore, &applied_terminal).await;
    let child_terminal = explore.snapshot();
    assert_eq!(child_terminal.status, "ACTIVE");
    assert!(child_terminal.child_task_id.is_empty());
    assert!(child_terminal.child_lease.is_none());
    assert!(nav.active_goal_lease().await.is_none());

    let next_active = explore.tick((0.0, 0.0)).await;
    assert_eq!(next_active.status, "ACTIVE", "{}", next_active.message);
    assert!(!next_active.child_task_id.is_empty());
    assert_ne!(next_active.child_task_id, first_child_id);
    assert!(next_active.child_lease.is_some());

    let second_goal_frame = next_typed(&mut goal_frames, Duration::from_secs(3)).await;
    let second_goal = schema::decode_goal_envelope_body(&second_goal_frame.payload)
        .expect("decode post-terminal child goal");
    assert_ne!(second_goal.goal_generation, goal.goal_generation);
    assert_eq!(second_goal.parent_request_id, parent_request);

    let second_path_frame = next_typed(&mut path_frames, Duration::from_secs(5)).await;
    let second_path = schema::decode_path_envelope_body(&second_path_frame.payload)
        .expect("decode post-terminal planner path");
    assert_eq!(second_path.session_id, second_goal.session_id);
    assert_eq!(second_path.goal_generation, second_goal.goal_generation);
    assert_eq!(second_path.goal_payload_sha256, second_goal.payload_sha256);

    let second_child_lease = next_active
        .child_lease
        .expect("post-terminal child cleanup lease");
    let second_terminal = explore
        .report_navigation_result(
            &next_active.child_task_id,
            crate::explore_runtime::ChildNavigationResult::Failed {
                proof: crate::explore_runtime::ChildNavigationTerminalProof {
                    lease: second_child_lease,
                },
                reason: "V2 canary cleanup".to_string(),
            },
        )
        .await
        .expect("release post-terminal child ownership");
    assert!(second_terminal.child_task_id.is_empty());
    let stopped_explore = explore.stop(&parent_request).await;
    assert_eq!(stopped_explore.status, "CANCELLED");
    assert!(stopped_explore.child_lease.is_none());
    explore.clear_navigation_sink();

    runner
        .shutdown()
        .await
        .expect("shutdown simple_planner owner");
    assert!(runner.running_nodes().is_empty());
    let stopped_mapping = mapping
        .lock()
        .stop_session("v2-canary-mapping")
        .expect("stop mapping session");
    assert!(!stopped_mapping.active);
}

#[tokio::test]
async fn vertical_v2_rejects_wrong_frame_and_out_of_boundary_goal() {
    let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let wrong_frame = crate::explore_runtime::ExploreCostmapSnapshot {
        grid: OccupancyGrid2D::filled(Point2 { x: 0.0, y: 0.0 }, 0.5, 2, 2, OccupancyCell::Free),
        frame_id: "odom".to_string(),
    };
    assert!(explore.ingest_costmap_snapshot(wrong_frame).await.is_err());
    assert!(!explore.cached_readiness().costmap_input_ready);

    let sink = Arc::new(FailClosedNavSink::default());
    explore.set_navigation_sink(sink.clone());

    let mut grid = OccupancyGrid2D::filled(
        Point2 { x: 0.0, y: 0.0 },
        0.5,
        20,
        20,
        OccupancyCell::Unknown,
    );
    for y in 0..20 {
        for x in 0..10 {
            grid.set(x, y, OccupancyCell::Free);
        }
    }
    explore
        .ingest_costmap_snapshot(crate::explore_runtime::ExploreCostmapSnapshot {
            grid,
            frame_id: "map".to_string(),
        })
        .await
        .expect("map-frame costmap");
    let boundary = ExplorationBoundary::from_vertices(vec![
        Point2 { x: 0.0, y: 0.0 },
        Point2 { x: 0.25, y: 0.0 },
        Point2 { x: 0.25, y: 0.25 },
        Point2 { x: 0.0, y: 0.25 },
    ])
    .expect("small valid boundary");
    let accepted = explore
        .start_with_config(
            "v2-boundary-reject",
            crate::explore_runtime::ExploreStartConfig {
                motion_admission: crate::nav_runtime::test_motion_admission(),
                boundary: Some(boundary),
                ..crate::explore_runtime::ExploreStartConfig::default()
            },
        )
        .await;
    assert_eq!(accepted.status, "ACCEPTED");

    let pending = explore.tick((0.0, 0.0)).await;
    assert_eq!(pending.status, "PENDING", "{}", pending.message);
    assert!(pending.child_task_id.is_empty());
    assert!(pending.child_lease.is_none());
    assert!(sink.submissions.lock().expect("submissions").is_empty());
    let diagnostics = explore.diagnostics().await;
    assert!(diagnostics
        .sources
        .iter()
        .any(|source| { source.last_error.contains("outside exploration boundary") }));

    let stopped = explore.stop("v2-boundary-reject").await;
    assert_eq!(stopped.status, "CANCELLED");
    explore.clear_navigation_sink();
}
