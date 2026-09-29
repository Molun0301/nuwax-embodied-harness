//! 局部几何计算拥有独立线程；续租、取消和输入接收只交换有界快照。
use super::*;
use std::sync::{Condvar, Mutex};

type Route = (ExecutionPath, Option<Vec<f64>>, PoseStamped);
#[derive(Default)]
struct Inputs {
    goal_id:String,
    epoch: u64,
    goal_version: u64,
    boundary: Vec<[f64;2]>,
    path_version: u64,
    goal: Option<(PoseStamped, nuwax_nav_types_messages::GoalOptions)>,
    path: Option<Arc<Route>>,
    map: Option<ExecutionMap>,
    map_version: u64,
    pose: Option<PoseStamped>,
    pose_version: u64,
    requested: Option<f64>,
    control_width: Option<f64>,
    applied: Twist2,
    applied_version: u64,
    result: Option<(std::time::Instant, PlannerStep)>,
    result_pose_version: u64,
    result_wait_reason: Option<String>,
    display_path: Vec<[f64; 3]>,
    closed: bool,
}

/// 计算线程没有运动端，结果须经原调度器核对目标身份再执行。
pub(crate) struct PlannerWorker {
    shared: Arc<(Mutex<Inputs>, Condvar)>,
    thread: Option<std::thread::JoinHandle<()>>,
    wait_reason: Option<String>,
}
impl PlannerWorker {
    pub(crate) fn new(mut planner: impl NavPlanner) -> Result<Self, RuntimeError> {
        let shared = Arc::new((Mutex::new(Inputs::default()), Condvar::new()));
        let worker = Arc::clone(&shared);
        let thread = std::thread::Builder::new()
            .name("nav-local-compute".into())
            .spawn(move || {
                let (lock, wake) = &*worker;
                let mut goal_version = u64::MAX;
                let mut path_version = u64::MAX;
                let mut map_version = u64::MAX;
                let mut slow_log_at: Option<std::time::Instant> = None;
                loop {
                    let (
                        version,
                        goal_id,
                        goal_seq,
                        path_seq,
                        goal,
                        boundary,
                        path,
                        map,
                        map_seq,
                        pose,
                        pose_seq,
                        width,
                        applied,
                        applied_seq,
                    ) = {
                        let mut input = lock.lock().unwrap();
                        while input.requested.is_none() && !input.closed {
                            input = wake.wait(input).unwrap();
                        }
                        if input.closed {
                            break;
                        }
                        (
                            input.epoch,
                            input.goal_id.clone(),
                            input.goal_version,
                            input.path_version,
                            input.goal.clone(),
                            input.boundary.clone(),
                            input.path.clone(),
                            input.map.clone(),
                            input.map_version,
                            input.pose.clone(),
                            input.pose_version,
                            input.requested.take().unwrap(),
                            input.applied,
                            input.applied_version,
                        )
                    };
                    let started = std::time::Instant::now();
                    let span=tracing::info_span!("navigation.local",goal_id=%goal_id);
                    let _entered=span.enter();
                    let computed = (|| -> Result<PlannerStep, RuntimeError> {
                        if goal_seq != goal_version {
                            planner.stop();
                            if let Some((goal, options)) = &goal {
                                planner.configure_goal(goal, *options)?;
                                planner.configure_boundary(&boundary);
                            }
                            goal_version = goal_seq;
                            path_version = u64::MAX;
                        }
                        if map_seq != map_version {
                            if let Some(map) = map {
                                planner.update_execution_map(map)?;
                            } else {
                                planner.invalidate_execution_map();
                            }
                            map_version = map_seq;
                        }
                        if let Some(pose) = pose {
                            planner.update_odometry(pose);
                        }
                        // 同一目标的新路径只替换路线，保留速度、制动及跟随状态。
                        if path_seq != path_version {
                            if let Some(path) = &path {
                                planner.replace_execution_path(
                                    path.0.clone(),
                                    path.1.clone(),
                                    path.2.clone(),
                                )?;
                            }
                            path_version = path_seq;
                        }
                        planner.observe_applied_command(applied);
                        Ok(planner.step(width))
                    })()
                    .unwrap_or_else(|error| PlannerStep::ReplanRequired(error.to_string()));
                    let wait_reason = if matches!(computed, PlannerStep::Command(command) if command == Twist2::ZERO) {
                        planner.execution_wait_reason()
                    } else { None };
                    let display_path = if matches!(computed, PlannerStep::Command(_)) {
                        planner.local_path_for_display()
                    } else { Vec::new() };
                    let elapsed = started.elapsed();
                    let slow_result = (elapsed > DEFAULT_CONTROL_HORIZON
                        && slow_log_at.is_none_or(|last| last.elapsed() >= Duration::from_secs(5)))
                        .then(|| format!("{computed:?}"));
                    let mut input = lock.lock().unwrap();
                    let current = input.epoch == version
                        && input.applied_version == applied_seq && !input.closed;
                    // 换目标、换路径、取消或来源撤销后的迟到结果一律退休。
                    if current {
                        input.result = Some((started, computed));
                        input.result_pose_version = pose_seq;
                        input.result_wait_reason = wait_reason;
                        input.display_path = display_path;
                    }
                    drop(input);
                    // 日志不持输入锁，慢磁盘不能阻塞调度器的取消与续租。
                    if let Some(result) = slow_result {
                        tracing::warn!(target: "navigation.local", event = "navigation.local.slow",
                            elapsed_ms = elapsed.as_secs_f64() * 1000.0, current, result,
                            "局部计算超过速度预测窗口，速度候选不延长许可，状态结论继续交付");
                        slow_log_at = Some(std::time::Instant::now());
                    }
                }
                planner.stop();
            })
            .map_err(|error| RuntimeError::Planner(error.to_string()))?;
        Ok(Self {
            shared,
            thread: Some(thread),
            wait_reason: None,
        })
    }
}
impl NavPlanner for PlannerWorker {
    fn set_goal_log_context(&mut self,goal_id:&str) {
        self.shared.0.lock().unwrap().goal_id=goal_id.to_owned();
    }
    fn configure_boundary(&mut self, boundary: &[[f64;2]]) {
        let mut input=self.shared.0.lock().unwrap();
        input.boundary=boundary.to_vec();
    }
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: nuwax_nav_types_messages::GoalOptions,
    ) -> Result<(), RuntimeError> {
        options.validate().map_err(RuntimeError::Planner)?;
        // 自主导航授权按前向策略收窄；旧目标允许倒退不能导致误拒或恢复倒退。
        let options=nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options};
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
        input.display_path.clear();
        input.path = None;
        input.goal_version += 1;
        input.path_version += 1;
        input.goal = Some((goal.clone(), options));
        Ok(())
    }
    fn replace_path(&mut self, _path: Path2D) {
        self.stop();
    }
    fn replace_execution_path(
        &mut self,
        path: ExecutionPath,
        yaws: Option<Vec<f64>>,
        goal: PoseStamped,
    ) -> Result<(), RuntimeError> {
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
        input.display_path.clear();
        input.path_version += 1;
        input.path = Some(Arc::new((path, yaws, goal)));
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        let mut input = self.shared.0.lock().unwrap();
        if input.pose.as_ref().is_none_or(|previous|
            previous.pose != pose.pose || previous.frame_id != pose.frame_id)
        {
            input.pose_version += 1;
        }
        input.pose = Some(pose);
    }
    fn update_costmap(&mut self, map: CostMap2D) {
        let _ = self.update_execution_map(ExecutionMap::Planar(map));
    }
    fn update_execution_map(&mut self, map: ExecutionMap) -> Result<(), RuntimeError> {
        let mut input = self.shared.0.lock().unwrap();
        input.map = Some(map);
        input.map_version += 1;
        Ok(())
    }
    fn invalidate_execution_map(&mut self) {
        self.stop();
        let mut input = self.shared.0.lock().unwrap();
        input.map = None;
        input.map_version += 1;
    }
    fn observe_applied_command(&mut self, command: Twist2) {
        let mut input = self.shared.0.lock().unwrap();
        if input.applied != command {
            input.applied_version += 1;
            input.applied = command;
        }
        // 执行回执到达后计算下一拍，使用实际已下发速度；不在本拍
        // 下发前抢算下一拍，避免候选被误作真实执行状态。
        input.requested = input.control_width;
        self.shared.1.notify_one();
    }
    fn step(&mut self, width: f64) -> PlannerStep {
        let mut input = self.shared.0.lock().unwrap();
        if self
            .thread
            .as_ref()
            .is_none_or(|thread| thread.is_finished())
        {
            return PlannerStep::Failed("局部计算线程已退出".into());
        }
        input.control_width = Some(width);
        let expired = input.result.as_ref().is_some_and(|(started, step)|
            matches!(step, PlannerStep::Command(_)) && started.elapsed() > DEFAULT_CONTROL_HORIZON);
        let arrival_changed = input.result.as_ref().is_some_and(|(_, step)|
            matches!(step, PlannerStep::Succeeded) && input.result_pose_version != input.pose_version);
        if input.result.is_none() || expired || arrival_changed {
            input.requested = Some(width);
            self.shared.1.notify_one();
        }
        // 只在已核验控制段的预测窗口内持续发送同一意图；新结果未及时
        // 完成时归零等待，不能将独立续租变成无限重放旧速度。
        let ready = input
            .result
            .as_ref()
            .filter(|(started, step)| {
                // 时效窗约束速度许可；失败、重规划及到达只触发原调度器归零，
                // 不能因计算慢而把这些结论吞成永远运行中的零速度。
                match step {
                    PlannerStep::Command(_) => started.elapsed() <= DEFAULT_CONTROL_HORIZON,
                    PlannerStep::Succeeded => !arrival_changed,
                    _ => true,
                }
            })
            .map(|(_, step)| step.clone());
        self.wait_reason = if ready.is_some() { input.result_wait_reason.clone() } else if arrival_changed {
            Some("位姿已更新，保持零速并重新核对到达状态".into())
        } else if expired {
            Some("局部速度计算结果已过期，保持零速并重新计算".into())
        } else {
            Some("局部轨迹正在计算，保持零速等待".into())
        };
        ready.unwrap_or(PlannerStep::Command(Twist2::ZERO))
    }
    fn local_path_for_display(&self) -> Vec<[f64; 3]> {
        self.shared.0.lock().unwrap().display_path.clone()
    }
    fn execution_wait_reason(&self) -> Option<String> {
        self.wait_reason.clone()
    }
    fn stop(&mut self) {
        self.wait_reason = None;
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
        input.display_path.clear();
        input.result_wait_reason = None;
        input.path = None;
        input.goal = None;
        input.applied = Twist2::ZERO;
        input.applied_version += 1;
        input.goal_version += 1;
        input.path_version += 1;
        input.requested = None;
        input.control_width = None;
    }
}
impl Drop for PlannerWorker {
    fn drop(&mut self) {
        {
            let mut input = self.shared.0.lock().unwrap();
            input.closed = true;
            input.result = None;
            input.display_path.clear();
            self.shared.1.notify_one();
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn local_display_drops_late_worker_results_after_stop() {
        struct DisplayPlanner { release: std::sync::mpsc::Receiver<()>, started: std::sync::mpsc::Sender<()> }
        impl NavPlanner for DisplayPlanner {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn step(&mut self, _: f64) -> PlannerStep {
                self.started.send(()).unwrap();
                self.release.recv().unwrap();
                PlannerStep::Command(Twist2::ZERO)
            }
            fn local_path_for_display(&self) -> Vec<[f64; 3]> { vec![[0.0,0.0,0.4],[1.0,0.5,0.4]] }
            fn stop(&mut self) {}
        }
        let (release, receive) = std::sync::mpsc::channel();
        let (started, start) = std::sync::mpsc::channel();
        let mut worker = PlannerWorker::new(DisplayPlanner {release:receive, started}).unwrap();
        worker.step(0.3);
        start.recv_timeout(Duration::from_secs(1)).unwrap();
        assert!(worker.local_path_for_display().is_empty());
        release.send(()).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        while worker.local_path_for_display().is_empty() {
            assert!(std::time::Instant::now() < deadline);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(worker.local_path_for_display().len(), 2);
        worker.observe_applied_command(Twist2::ZERO);
        start.recv_timeout(Duration::from_secs(1)).unwrap();
        worker.stop();
        release.send(()).unwrap();
        std::thread::sleep(Duration::from_millis(10));
        assert!(worker.local_path_for_display().is_empty(), "取消后的旧局部轨迹不能复活");
    }

    #[test]
    fn stationary_geometry_wait_reason_crosses_the_compute_thread() {
        struct WaitingPlanner;
        impl NavPlanner for WaitingPlanner {
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn step(&mut self, _: f64) -> PlannerStep { PlannerStep::Command(Twist2::ZERO) }
            fn execution_wait_reason(&self) -> Option<String> { Some("等待当前足下支撑恢复".into()) }
            fn stop(&mut self) {}
        }
        let mut worker = PlannerWorker::new(WaitingPlanner).unwrap();
        let deadline = std::time::Instant::now() + Duration::from_secs(1);
        loop {
            assert_eq!(worker.step(0.3), PlannerStep::Command(Twist2::ZERO));
            if worker.execution_wait_reason().as_deref() == Some("等待当前足下支撑恢复") { break; }
            assert!(std::time::Instant::now() < deadline, "计算线程必须交付实际等待原因");
            std::thread::sleep(Duration::from_millis(1));
        }
        worker.stop();
        assert!(worker.execution_wait_reason().is_none());
        assert!(worker.shared.0.lock().unwrap().result_wait_reason.is_none());
    }
    use std::sync::atomic::{AtomicBool, Ordering};
    #[test]
    fn slow_planner_outcomes_are_not_lost_to_velocity_expiry() {
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered: Arc::new(AtomicBool::new(false)),
            release: Arc::new(AtomicBool::new(true)),
        }).unwrap();
        // 重规划和终态本身不授权运动；慢查询的结论仍必须交给原调度器收尾。
        for outcome in [
            PlannerStep::ReplanRequired("现场近场轨迹受阻".into()),
            PlannerStep::Failed("局部输入无效".into()),
            PlannerStep::Succeeded,
        ] {
            worker.shared.0.lock().unwrap().result = Some((
                std::time::Instant::now() - DEFAULT_CONTROL_HORIZON - Duration::from_millis(1),
                outcome.clone(),
            ));
            assert_eq!(worker.step(0.3), outcome);
        }
        worker.shared.0.lock().unwrap().result = Some((
            std::time::Instant::now() - DEFAULT_CONTROL_HORIZON - Duration::from_millis(1),
            PlannerStep::Command(Twist2 { linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0), angular_z: 0.0 }),
        ));
        assert_eq!(worker.step(0.3), PlannerStep::Command(Twist2::ZERO),
            "传播慢查询结论不能延长速度许可");
        assert!(worker.execution_wait_reason().unwrap().contains("过期"));
    }

    #[test]
    fn arrival_is_rechecked_when_pose_changes_after_computation() {
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered: Arc::new(AtomicBool::new(false)),
            release: Arc::new(AtomicBool::new(true)),
        }).unwrap();
        let mut pose = nuwax_nav_astar::helpers::make_pose_stamped("map", 1.0, 0.0, 0.0);
        worker.update_odometry(pose.clone());
        {
            let mut input = worker.shared.0.lock().unwrap();
            input.result = Some((std::time::Instant::now(), PlannerStep::Succeeded));
            input.result_pose_version = input.pose_version;
        }
        pose.stamp.nanos += 50_000_000;
        worker.update_odometry(pose.clone());
        assert_eq!(worker.step(0.3), PlannerStep::Succeeded,
            "同位姿新时间戳不应阻断已确认的位置到达");
        pose.pose.position.x += 0.5;
        worker.update_odometry(pose);
        assert_eq!(worker.step(0.3), PlannerStep::Command(Twist2::ZERO),
            "移动后不能沿旧位置报告成功");
        assert!(worker.execution_wait_reason().unwrap().contains("到达状态"));
    }
    struct SlowPlanner {
        entered: Arc<AtomicBool>,
        release: Arc<AtomicBool>,
    }
    impl NavPlanner for SlowPlanner {
        fn replace_path(&mut self, _: Path2D) {}
        fn update_odometry(&mut self, _: PoseStamped) {}
        fn update_costmap(&mut self, _: CostMap2D) {}
        fn step(&mut self, _: f64) -> PlannerStep {
            self.entered.store(true, Ordering::SeqCst);
            while !self.release.load(Ordering::SeqCst) {
                std::thread::sleep(Duration::from_millis(1));
            }
            PlannerStep::Command(Twist2 {
                linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0),
                angular_z: 0.0,
            })
        }
        fn stop(&mut self) {}
    }
    #[test]
    fn autonomous_worker_delivers_forward_only_goal_options() {
        let entered=Arc::new(AtomicBool::new(false));
        let mut worker=PlannerWorker::new(SlowPlanner {
            entered:Arc::clone(&entered),release:Arc::new(AtomicBool::new(true)),
        }).unwrap();
        let goal=nuwax_nav_astar::helpers::make_pose_stamped("map",1.0,0.0,0.0);
        let options=nuwax_nav_types_messages::GoalOptions {
            allow_reverse:true,position_tolerance_m:0.23,yaw_tolerance_rad:0.7,
        };
        worker.configure_goal(&goal,options).unwrap();
        worker.step(0.3);
        let deadline=std::time::Instant::now()+Duration::from_secs(1);
        while worker.shared.0.lock().unwrap().result.is_none() && std::time::Instant::now()<deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        let input=worker.shared.0.lock().unwrap();
        assert_eq!(input.goal.as_ref().unwrap().1,nuwax_nav_types_messages::GoalOptions {allow_reverse:false,..options});
        assert!(matches!(input.result.as_ref().map(|r|&r.1),Some(PlannerStep::Command(v))if v.linear.x>0.0),
            "默认不支持倒退的后端必须收到收窄后的目标，不能误拒旧授权");
        assert!(entered.load(Ordering::SeqCst));
    }
    #[test]
    fn replacement_path_preserves_goal_and_applied_control_state() {
        use std::sync::atomic::AtomicU64;
        struct Probe {
            goals: Arc<AtomicU64>,
            stops: Arc<AtomicU64>,
            applied: Twist2,
        }
        impl NavPlanner for Probe {
            fn configure_goal(
                &mut self,
                _: &PoseStamped,
                _: nuwax_nav_types_messages::GoalOptions,
            ) -> Result<(), RuntimeError> {
                self.goals.fetch_add(1, Ordering::SeqCst);
                Ok(())
            }
            fn replace_path(&mut self, _: Path2D) {}
            fn update_odometry(&mut self, _: PoseStamped) {}
            fn update_costmap(&mut self, _: CostMap2D) {}
            fn observe_applied_command(&mut self, command: Twist2) {
                self.applied = command;
            }
            fn step(&mut self, _: f64) -> PlannerStep {
                PlannerStep::Command(self.applied)
            }
            fn stop(&mut self) {
                self.stops.fetch_add(1, Ordering::SeqCst);
                self.applied = Twist2::ZERO;
            }
        }
        let goals = Arc::new(AtomicU64::new(0));
        let stops = Arc::new(AtomicU64::new(0));
        let mut worker = PlannerWorker::new(Probe {
            goals: goals.clone(),
            stops: stops.clone(),
            applied: Twist2::ZERO,
        })
        .unwrap();
        let goal = nuwax_nav_astar::helpers::make_pose_stamped("map", 1.0, 0.0, 0.0);
        worker.configure_goal(&goal, Default::default()).unwrap();
        worker.update_costmap(CostMap2D::new_all_free("map", 10, 10, 0.1));
        let command = Twist2 {
            linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0),
            angular_z: 0.1,
        };
        worker.observe_applied_command(command);
        let route = ExecutionPath::Planar(Path2D::stamped(
            nuwax_nav_types_messages::NavFrameId::Map,
            Default::default(),
            vec![nuwax_nav_types_messages::Point2::new(1.0, 0.0)],
        ));
        for revision in 0..2 {
            worker
                .replace_execution_path(route.clone(), None, goal.clone())
                .unwrap();
            worker.step(0.3);
            let deadline = std::time::Instant::now() + Duration::from_secs(1);
            while worker.shared.0.lock().unwrap().result.is_none()
                && std::time::Instant::now() < deadline
            {
                std::thread::sleep(Duration::from_millis(1));
            }
            assert_eq!(
                worker.shared.0.lock().unwrap().result.as_ref().unwrap().1,
                PlannerStep::Command(command)
            );
            assert_eq!(
                goals.load(Ordering::SeqCst),
                1,
                "第{revision}次换路径不能重新配置目标"
            );
            assert_eq!(
                stops.load(Ordering::SeqCst),
                1,
                "换路径不能额外重置运动历史"
            );
        }
    }

    #[test]
    fn blocked_geometry_does_not_block_stop_and_late_velocity_is_retired() {
        let entered = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered: entered.clone(),
            release: release.clone(),
        })
        .unwrap();
        assert_eq!(worker.step(0.3), PlannerStep::Command(Twist2::ZERO));
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while !entered.load(Ordering::SeqCst) && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        let started = std::time::Instant::now();
        worker.stop();
        let elapsed = started.elapsed();
        release.store(true, Ordering::SeqCst);
        assert!(elapsed < Duration::from_millis(20), "取消等待了几何计算");
        std::thread::sleep(Duration::from_millis(20));
        assert!(
            worker.shared.0.lock().unwrap().result.is_none(),
            "取消后的计算不能恢复旧速度"
        );
    }
    #[test]
    fn pending_next_computation_preserves_continuous_velocity_only_inside_verified_horizon() {
        let entered = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(true));
        let mut worker = PlannerWorker::new(SlowPlanner {
            entered,
            release: release.clone(),
        })
        .unwrap();
        worker.step(0.3);
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        while worker.shared.0.lock().unwrap().result.is_none()
            && std::time::Instant::now() < deadline
        {
            std::thread::sleep(Duration::from_millis(1));
        }
        release.store(false, Ordering::SeqCst);
        let result = worker.step(0.3);
        assert!(matches!(result,PlannerStep::Command(t) if t.linear.x>0.0));
        if let PlannerStep::Command(command) = result {
            worker.observe_applied_command(command);
        }
        std::thread::sleep(DEFAULT_CONTROL_HORIZON + Duration::from_millis(20));
        let stale = worker.step(0.3);
        release.store(true, Ordering::SeqCst);
        assert_eq!(
            stale,
            PlannerStep::Command(Twist2::ZERO),
            "不能无限重放旧速度"
        );
    }
}
