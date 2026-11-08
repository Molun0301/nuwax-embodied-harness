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
    spawn_scheduler, CostmapObservation, ExecutionGoal, ExecutionMap, ExecutionPath,
    MotionPortError, NavMotionPort,
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
    async fn acquire(&mut self, _generation:u64) -> Result<u64,MotionPortError> {
        self.state.lock().unwrap().acquire_count+=1; Ok(1)
    }
    async fn velocity(&mut self, _twist:Twist2) -> Result<(),MotionPortError> {
        self.state.lock().unwrap().applied_count+=1; Ok(())
    }
    async fn release(&mut self) -> Result<(),MotionPortError> {
        self.state.lock().unwrap().fence_count+=1; Ok(())
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

    fn step(&mut self) -> PlannerStep {
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
        goal_key: nuwax_nav_types_messages::GoalKey { producer_runtime_generation: (1) as u64, goal_generation: (1) as u64 },
        session_id: session_id.to_string(),

        planner_runtime_generation: if path_revision == 0 { 0 } else { 1 },
        path_revision: path_revision as u64,
    }
}

fn session() -> NavSession {
    NavSession {
        boundary: Vec::new(),
            options: Default::default(),
        goal: ExecutionGoal::Body(nuwax_nav_astar::helpers::make_pose_stamped("map", 2.0, 0.0, 0.0)),
        session_id: "nav_scheduler_20hz_baseline-nav".to_string(),
        execution_identity: identity("nav_scheduler_20hz_baseline-nav", 0),

        motion_generation: 1,
    }
}

fn path() -> VersionedPath {
    VersionedPath {
        route_kind: nuwax_nav_runtime::RouteKind::Complete,
        path_yaws: None,
        session_id: "nav_scheduler_20hz_baseline-nav".to_string(),
        execution_identity: identity("nav_scheduler_20hz_baseline-nav", 1),
        path_revision: 1,
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

            path_wait_timeout: Duration::from_secs(1),


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
