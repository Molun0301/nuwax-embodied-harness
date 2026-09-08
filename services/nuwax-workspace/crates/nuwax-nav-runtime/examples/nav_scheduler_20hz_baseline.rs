//! NAV-20HZ 最小本地基线。
//!
//! 该示例只使用 nuwax-nav-runtime 的公开 scheduler/trait API 和内存 fake
//! 端口，不连接真实 Motion Guard UDS，也不执行真实机器人动作。
//!
//! 运行：
//! `cargo run --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-nav-runtime --example nav_scheduler_20hz_baseline`

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use nuwax_nav_astar::navigation_map::CostMap2D;
use nuwax_nav_runtime::{
    spawn_scheduler, AppliedProof, CostmapObservation, ExecutionMap, ExecutionPath,
    FenceLeaseState, FenceProof, LeaseProof, MotionPortError, NavMotionAdmission, NavMotionPort,
    NavPlanner, NavScheduler, NavSession, PathExecutionIdentity, PlannerStep, RuntimeError,
    SchedulerConfig, SchedulerStatus, VersionedPath,
};
use nuwax_nav_types_messages::{NavFrameId, Path2D, Point2, Pose3, PoseStamped, Timestamp, Twist2};

#[derive(Debug, Default)]
struct FakeMotionState {
    acquire_count: u64,
    renew_count: u64,
    applied_count: u64,
    fence_count: u64,
}

#[derive(Clone, Default)]
struct FakeMotion {
    state: Arc<Mutex<FakeMotionState>>,
}

#[async_trait]
impl NavMotionPort for FakeMotion {
    async fn acquire(
        &mut self,
        _operation_id: &str,
        admission: &NavMotionAdmission,
    ) -> Result<LeaseProof, MotionPortError> {
        self.state.lock().expect("fake motion state").acquire_count += 1;
        Ok(LeaseProof {
            lease_id: 1,
            guard_generation: admission.guard_generation,
        })
    }

    async fn renew(&mut self, _operation_id: &str) -> Result<(), MotionPortError> {
        self.state.lock().expect("fake motion state").renew_count += 1;
        Ok(())
    }

    async fn submit_twist(
        &mut self,
        operation_id: &str,
        _twist: Twist2,
    ) -> Result<AppliedProof, MotionPortError> {
        self.state.lock().expect("fake motion state").applied_count += 1;
        Ok(AppliedProof {
            operation_id: operation_id.to_string(),
            applied_monotonic_ns: 1,
        })
    }

    async fn revoke_and_zero(
        &mut self,
        _operation_id: &str,
    ) -> Result<FenceProof, MotionPortError> {
        self.state.lock().expect("fake motion state").fence_count += 1;
        Ok(FenceProof {
            zero_ack_id: "nav_scheduler_20hz_baseline-zero".to_string(),
            applied_monotonic_ns: 2,
            lease: FenceLeaseState::Idle,
        })
    }
}

struct FakePlanner {
    ticks: u32,
    succeed_after: u32,
}

impl FakePlanner {
    fn new(succeed_after: u32) -> Self {
        Self {
            ticks: 0,
            succeed_after,
        }
    }
}

impl NavPlanner for FakePlanner {
    fn replace_path(&mut self, _path: Path2D) {}

    fn update_odometry(&mut self, _odometry: PoseStamped) {}

    fn update_costmap(&mut self, _costmap: CostMap2D) {}

    fn step(&mut self, _robot_width_m: f64) -> PlannerStep {
        self.ticks = self.ticks.saturating_add(1);
        if self.ticks >= self.succeed_after {
            PlannerStep::Succeeded
        } else {
            PlannerStep::Command(Twist2 {
                linear: Point2::new(0.05, 0.0),
                angular_z: 0.0,
            })
        }
    }

    fn stop(&mut self) {}
}

fn identity(session_id: &str, path_revision: i64) -> PathExecutionIdentity {
    PathExecutionIdentity {
        session_id: session_id.to_string(),
        owner_kind: 1,
        parent_request_id: format!("nav_scheduler_20hz_baseline-request-{session_id}"),
        goal_generation: 1,
        producer: "nav_scheduler_20hz_baseline".to_string(),
        producer_epoch: 1,
        producer_runtime_generation: 1,
        committed_map_session_id: "nav_scheduler_20hz_baseline-map".to_string(),
        committed_map_session_generation: 1,
        committed_map_revision: 1,
        committed_map_payload_sha256: [3; 32],
        committed_map_frame_id: "map".to_string(),
        boundary_id: String::new(),
        boundary_sha256: [0; 32],
        motion_guard_generation: 7,
        motion_guard_boot_nonce: [7; 16],
        motion_stop_generation: 1,
        planner_runtime_generation: if path_revision == 0 { 0 } else { 1 },
        path_revision: path_revision as u64,
        goal_payload_sha256: [4; 32],
        path_payload_sha256: if path_revision == 0 { [0; 32] } else { [5; 32] },
    }
}

fn session() -> NavSession {
    NavSession {
        goal_pose: nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, 0.0),
        session_id: "nav_scheduler_20hz_baseline-nav".to_string(),
        execution_identity: identity("nav_scheduler_20hz_baseline-nav", 0),
        local_owner_runtime_generation: 1,
        motion_admission: NavMotionAdmission {
            guard_generation: 7,
            guard_boot_nonce: [7; 16],
            motion_stop_generation: 1,
        },
    }
}

fn path() -> VersionedPath {
    VersionedPath {
        path_yaws: None,
        session_id: "nav_scheduler_20hz_baseline-nav".to_string(),
        execution_identity: identity("nav_scheduler_20hz_baseline-nav", 1),
        path_revision: 1,
        payload_hash: [5; 32],
        path: ExecutionPath::Planar(Path2D::stamped(
            NavFrameId::Map,
            Timestamp::from_unix(1, 0),
            vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
        )),
    }
}

fn pose() -> PoseStamped {
    let mut value = PoseStamped::from_pose(Pose3::default(), NavFrameId::Map);
    value.pose.orientation = nuwax_nav_types_messages::Quaternion::IDENTITY;
    value.stamp = Timestamp::from_unix(1, 0);
    value
}

fn costmap(source_timestamp_ns: i64) -> CostmapObservation {
    CostmapObservation {
        costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 4, 4, 0.1)),
        source_timestamp_ns,
    }
}

fn summary(status: &SchedulerStatus, elapsed: Duration, motion: &FakeMotion) -> String {
    let state = motion.state.lock().expect("fake motion state");
    // FakePlanner emits two commands and one terminal step, so the terminal
    // control tick is included explicitly alongside APPLIED command count.
    let control_ticks = status.applied_commands.saturating_add(1);
    format!(
        "{{\"scenario\":\"NAV-20HZ\",\"phase\":\"{:?}\",\"terminal\":\"{:?}\",\"control_ticks\":{},\"applied_commands\":{},\"elapsed_ms\":{:.3},\"acquire\":{},\"renew\":{},\"fence\":{},\"real_motion_guard\":\"NOT_EVALUATED\"}}",
        status.phase,
        status.last_terminal,
        control_ticks,
        status.applied_commands,
        elapsed.as_secs_f64() * 1000.0,
        state.acquire_count,
        state.renew_count,
        state.fence_count,
    )
}

async fn run() -> Result<(), RuntimeError> {
    let motion = FakeMotion::default();
    let motion_for_summary = motion.clone();
    let scheduler = NavScheduler::new(
        motion,
        FakePlanner::new(3),
        SchedulerConfig {
            tick_interval: Duration::from_millis(50),
            renew_interval: Duration::from_millis(200),
            path_wait_timeout: Duration::from_secs(1),
            robot_width_m: 0.3,
            pose_ttl: Duration::from_millis(500),
            costmap_ttl: Duration::from_millis(500),
        },
    )?;
    let (handle, scheduler_worker) = spawn_scheduler(scheduler);
    let started = Instant::now();
    handle.start_session(session()).await?;
    handle.update_odometry(pose()).await?;
    handle.update_costmap(costmap(2_000_000_000)).await?;
    handle.submit_path(path()).await?;

    let terminal = tokio::time::timeout(Duration::from_secs(2), async {
        loop {
            let status = handle.status();
            if status.last_terminal.is_some() {
                break status;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .map_err(|_| RuntimeError::Planner("NAV-20HZ terminal timeout".to_string()))?;
    let elapsed = started.elapsed();
    handle.shutdown().await?;
    scheduler_worker
        .await
        .map_err(|_| RuntimeError::SchedulerStopped)??;
    println!("{}", summary(&terminal, elapsed, &motion_for_summary));
    Ok(())
}

#[tokio::main]
async fn main() {
    if let Err(error) = run().await {
        eprintln!("NAV-20HZ baseline failed: {error}");
        std::process::exit(1);
    }
}
