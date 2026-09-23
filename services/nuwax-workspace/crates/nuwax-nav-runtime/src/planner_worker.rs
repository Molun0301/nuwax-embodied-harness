//! 局部几何计算拥有独立线程；续租、取消和输入接收只交换有界快照。
use super::*;
use std::sync::{Condvar, Mutex};

type Route = (ExecutionPath, Option<Vec<f64>>, PoseStamped);
#[derive(Default)]
struct Inputs {
    epoch: u64,
    goal_version: u64,
    path_version: u64,
    goal: Option<(PoseStamped, nuwax_nav_types_messages::GoalOptions)>,
    path: Option<Arc<Route>>,
    map: Option<ExecutionMap>,
    map_version: u64,
    pose: Option<PoseStamped>,
    requested: Option<f64>,
    control_width: Option<f64>,
    applied: Twist2,
    applied_version: u64,
    result: Option<(std::time::Instant, PlannerStep)>,
    closed: bool,
}

/// 计算线程没有运动端，结果须经原调度器核对目标身份再执行。
pub(crate) struct PlannerWorker {
    shared: Arc<(Mutex<Inputs>, Condvar)>,
    thread: Option<std::thread::JoinHandle<()>>,
    reverse_supported: bool,
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
                loop {
                    let (
                        version,
                        goal_seq,
                        path_seq,
                        goal,
                        path,
                        map,
                        map_seq,
                        pose,
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
                            input.goal_version,
                            input.path_version,
                            input.goal.clone(),
                            input.path.clone(),
                            input.map.clone(),
                            input.map_version,
                            input.pose.clone(),
                            input.requested.take().unwrap(),
                            input.applied,
                            input.applied_version,
                        )
                    };
                    let started = std::time::Instant::now();
                    let computed = (|| -> Result<PlannerStep, RuntimeError> {
                        if goal_seq != goal_version {
                            planner.stop();
                            if let Some((goal, options)) = &goal {
                                planner.configure_goal(goal, *options)?;
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
                    let mut input = lock.lock().unwrap();
                    // 换目标、换路径、取消或来源撤销后的迟到结果一律退休。
                    if input.epoch == version
                        && input.applied_version == applied_seq
                        && !input.closed
                    {
                        input.result = Some((started, computed));
                    }
                }
                planner.stop();
            })
            .map_err(|error| RuntimeError::Planner(error.to_string()))?;
        Ok(Self {
            shared,
            thread: Some(thread),
            reverse_supported: false,
        })
    }
}
impl PlannerWorker {
    pub(crate) fn with_reverse_support(mut self, enabled: bool) -> Self {
        self.reverse_supported = enabled;
        self
    }
}
impl NavPlanner for PlannerWorker {
    fn configure_goal(
        &mut self,
        goal: &PoseStamped,
        options: nuwax_nav_types_messages::GoalOptions,
    ) -> Result<(), RuntimeError> {
        options.validate().map_err(RuntimeError::Planner)?;
        if options.allow_reverse && !self.reverse_supported {
            return Err(RuntimeError::InvalidInput {
                field: "allow_reverse",
                reason: "三维执行未提供倒车标定".into(),
            });
        }
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
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
        input.path_version += 1;
        input.path = Some(Arc::new((path, yaws, goal)));
        Ok(())
    }
    fn update_odometry(&mut self, pose: PoseStamped) {
        self.shared.0.lock().unwrap().pose = Some(pose);
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
        if input.result.is_none() {
            input.requested = Some(width);
            self.shared.1.notify_one();
        }
        // 只在已核验控制段的预测窗口内持续发送同一意图；新结果未及时
        // 完成时归零等待，不能将独立续租变成无限重放旧速度。
        input
            .result
            .as_ref()
            .filter(|(started, _)| started.elapsed() <= DEFAULT_CONTROL_HORIZON)
            .map(|(_, step)| step.clone())
            .unwrap_or(PlannerStep::Command(Twist2::ZERO))
    }
    fn stop(&mut self) {
        let mut input = self.shared.0.lock().unwrap();
        input.epoch += 1;
        input.result = None;
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
    use std::sync::atomic::{AtomicBool, Ordering};
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
