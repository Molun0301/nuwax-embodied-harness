//! 全局层析参考到既有三维路径契约的转换；实际运动姿态由近场轨迹产生。
use crate::{PctPlanner,PlanningFailure};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_types_messages::GoalOptions;
use std::sync::atomic::AtomicBool;
pub type BodyPose = ([f64; 3], [f64; 4]);
impl PctPlanner {
    pub fn body_route(
        &self,
        map: &SurfaceSnapshot,
        start: BodyPose,
        goal: BodyPose,
        options: GoalOptions,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
    ) -> Result<Vec<BodyPose>, String> {
        self.body_route_with_limits(map, start, goal, options, boundary, cancelled, self.search_limits)
    }
    /// Preserve an owner's existing per-attempt resource budget all the way to
    /// the PCT query; budget exhaustion remains a distinct, inconclusive error.
    pub fn body_route_with_limits(
        &self,
        map: &SurfaceSnapshot,
        start: BodyPose,
        goal: BodyPose,
        options: GoalOptions,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
        limits: crate::SearchLimits,
    ) -> Result<Vec<BodyPose>, String> {
        self.body_route_with_limits_typed(map,start,goal,options,boundary,cancelled,limits)
            .map_err(|error|error.to_string())
    }
    /// Preserve producer resource outcomes for an owner without text parsing.
    pub fn body_route_with_limits_typed(
        &self,
        map: &SurfaceSnapshot,
        start: BodyPose,
        goal: BodyPose,
        options: GoalOptions,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
        limits: crate::SearchLimits,
    ) -> Result<Vec<BodyPose>, PlanningFailure> {
        crate::search::checkpoint(cancelled)?;
        let penalties = self.local_costs();
        self.body_route_with_limits_and_penalties_typed(
            map, start, goal, options, boundary, cancelled, limits, &penalties,
        )
    }
    /// 使用调用方已捕获的不可变代价快照；实际搜索不会重新读取 local_costs。
    /// 调用方可将同一份快照保存在本次规划输入中，不必跨搜索持有代价锁。
    pub fn body_route_with_limits_and_penalties_typed(
        &self,
        map: &SurfaceSnapshot,
        start: BodyPose,
        goal: BodyPose,
        options: GoalOptions,
        boundary: &[[f64; 2]],
        cancelled: Option<&AtomicBool>,
        limits: crate::SearchLimits,
        penalties: &[crate::RoutePenalty],
    ) -> Result<Vec<BodyPose>, PlanningFailure> {
        // The owner's flag must reject obsolete work before deriving a tomogram,
        // including the near-goal return. Reuse the search's existing semantics.
        crate::search::checkpoint(cancelled)?;
        let model = map
            .body_envelope()
            .ok_or("层析规划需要机器人外形和支撑标定")?;
        self.reference_body_route_with_offset_and_penalties_typed(
            map, start, goal, options, boundary, cancelled, limits,
            model.body_to_support_z_m, penalties,
        )
    }
    /// Share the yaw-only reference contract with legacy callers that have an
    /// explicit vertical calibration but no complete body envelope. Their same
    /// configured circular footprint/height is checked by every route segment.
    pub fn reference_body_route_with_offset(
        &self, map: &SurfaceSnapshot, start: BodyPose, goal: BodyPose,
        options: GoalOptions, boundary: &[[f64;2]], cancelled: Option<&AtomicBool>,
        limits: crate::SearchLimits, offset: f64,
    ) -> Result<Vec<BodyPose>,String> {
        self.reference_body_route_with_offset_typed(map,start,goal,options,boundary,cancelled,limits,offset)
            .map_err(|error|error.to_string())
    }
    /// Typed explicit-calibration counterpart for legacy request owners.
    pub fn reference_body_route_with_offset_typed(
        &self, map: &SurfaceSnapshot, start: BodyPose, goal: BodyPose,
        options: GoalOptions, boundary: &[[f64;2]], cancelled: Option<&AtomicBool>,
        limits: crate::SearchLimits, offset: f64,
    ) -> Result<Vec<BodyPose>,PlanningFailure> {
        crate::search::checkpoint(cancelled)?;
        let penalties = self.local_costs();
        self.reference_body_route_with_offset_and_penalties_typed(
            map, start, goal, options, boundary, cancelled, limits, offset, &penalties,
        )
    }
    /// 显式竖直标定路径同样消费调用方快照；只在本次搜索副本上转换代价高度。
    pub fn reference_body_route_with_offset_and_penalties_typed(
        &self, map: &SurfaceSnapshot, start: BodyPose, goal: BodyPose,
        options: GoalOptions, boundary: &[[f64;2]], cancelled: Option<&AtomicBool>,
        limits: crate::SearchLimits, offset: f64, penalties: &[crate::RoutePenalty],
    ) -> Result<Vec<BodyPose>,PlanningFailure> {
        crate::search::checkpoint(cancelled)?;
        // Validate the complete input poses and calibration, but use one
        // consistent yaw-only guide transform at both ends. A rotated XY support
        // anchor cannot be inverted later with a vertical-only body translation.
        map.support_position_from_body(start.0,start.1,offset).map_err(|e|e.to_string())?;
        map.support_position_from_body(goal.0,goal.1,offset).map_err(|e|e.to_string())?;
        if crate::distance(start.0, goal.0) < options.position_tolerance_m {
            return Ok(vec![start]);
        }
        let start_support = [start.0[0],start.0[1],start.0[2]-offset];
        let goal_support = [goal.0[0],goal.0[1],goal.0[2]-offset];
        let tomogram = self.map(map)?;
        // The current calibration is a coordinate bias, not a terrain step.
        // Shift both endpoints by the same amount so the original 3D arrival
        // sphere is unchanged when references are translated back to body poses.
        let reference_z = tomogram.calibrated_reference_height(start_support);
        crate::search::checkpoint(cancelled)?;
        let reference_z = reference_z.ok_or("层析起点存在已知障碍或不兼容地形")?;
        let bias = start_support[2] - reference_z;
        let reference_start = [start_support[0],start_support[1],reference_z];
        let reference_goal = [goal_support[0],goal_support[1],goal_support[2]-bias];
        let mut penalties = penalties.to_vec();
        for penalty in &mut penalties { penalty.center[2] -= bias; }
        let planar_accepts = |a: [f64;3], b: [f64;3]| {
            let carried_a = [a[0],a[1],a[2]+bias];
            let carried_b = [b[0],b[1],b[2]+bias];
            tomogram.project_reference_motion(carried_a,[b[0],b[1]],boundary)
                .is_ok_and(|end|crate::distance(end,carried_b)<=1e-6)
        };
        let route = if map.body_envelope().is_some() {
            // Calibrated bodies retain a global reference; Scan owns the full
            // oriented body sweep rather than repeating it at every guide node.
            tomogram.plan_bounded_with_limits_typed(reference_start,reference_goal,
                options.position_tolerance_m,boundary,cancelled,None,&penalties,limits)
        } else {
            tomogram.plan_bounded_with_segment_filter_and_limits_typed(reference_start,reference_goal,
                options.position_tolerance_m,boundary,cancelled,None,&penalties,&planar_accepts,limits)
        }?;
        let mut poses = vec![start];
        for (i, &point) in route.points.iter().enumerate().skip(1) {
            crate::search::checkpoint(cancelled)?;
            let previous = route.points[i - 1];
            let next = route.points.get(i + 1).copied().unwrap_or(point);
            let dx = next[0] - previous[0];
            let dy = next[1] - previous[1];
            let yaw = dy.atan2(dx);
            let q = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
            // 此处交付层析参考而非精确站姿；不能再次用旧MLS离散表面拒绝新高程。
            // 参考只有偏航，沿竖直标定平移即可，真实横滚/俯仰在Scan中从地形生成。
            let body = [point[0], point[1], point[2] + bias + offset];
            poses.push((body, q));
        }
        tracing::info!(target:"navigation.pct",event="navigation.pct.route",expanded=route.stats.expanded,
            elapsed_ms=route.stats.elapsed_ms,points=poses.len(),"层析全局路线计算完成");
        crate::search::checkpoint(cancelled)?;
        Ok(poses)
    }
}

// 未运行的单测草案：追加到 body_route.rs；不为生产代码增加探针或预算门。
#[cfg(test)]
mod penalty_snapshot_tests {
    use super::*;
    use crate::{RoutePenalty, SearchLimits};
    use nuwax_nav_mls_planner::SurfaceBodyEnvelope;
    use std::sync::{mpsc, Arc};
    use std::time::Duration;

    fn source(calibrated: bool) -> SurfaceSnapshot {
        let source = crate::sampling_tests::sampled_room(true, false);
        if !calibrated { return source; }
        source.with_body_envelope(SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
            z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.4,
            max_step_up_m: 0.15, max_step_down_m: 0.15,
            max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
        }).unwrap()
    }

    fn route(
        planner: &PctPlanner, source: &SurfaceSnapshot, calibrated: bool,
        snapshot: Option<&[RoutePenalty]>, cancelled: Option<&AtomicBool>,
    ) -> Result<Vec<BodyPose>, PlanningFailure> {
        let start = ([-0.5,0.0,0.462],[0.0,0.0,0.0,1.0]);
        let goal = ([0.5,0.0,0.462],start.1);
        let options = GoalOptions { position_tolerance_m: 0.001, ..GoalOptions::default() };
        // 复用已有零状态预算作确定性观测：空代价可验证直连；非空代价保留
        // 原 BudgetExhausted。此处不测试绕行形状，也不改变生产预算行为。
        let limits = SearchLimits { max_states: 0 };
        match (calibrated, snapshot) {
            (true, Some(penalties)) => planner.body_route_with_limits_and_penalties_typed(
                source, start, goal, options, &[], cancelled, limits, penalties),
            (false, Some(penalties)) => planner.reference_body_route_with_offset_and_penalties_typed(
                source, start, goal, options, &[], cancelled, limits, 0.4, penalties),
            (true, None) => planner.body_route_with_limits_typed(
                source, start, goal, options, &[], cancelled, limits),
            (false, None) => planner.reference_body_route_with_offset_typed(
                source, start, goal, options, &[], cancelled, limits, 0.4),
        }
    }

    fn penalty_bits(penalties: &[RoutePenalty]) -> Vec<[u64; 5]> {
        penalties.iter().map(|p| [p.center[0].to_bits(), p.center[1].to_bits(),
            p.center[2].to_bits(), p.radius.to_bits(), p.weight.to_bits()]).collect()
    }

    fn assert_outcome(result: Result<Vec<BodyPose>, PlanningFailure>, nonempty: bool) {
        if nonempty {
            assert!(matches!(&result, Err(PlanningFailure::BudgetExhausted(_))),
                "非空快照须到达原搜索预算分支，不能被重新读取的空代价覆盖：{result:?}");
        } else {
            let route = result.expect("空快照须保持已有直连能力，不能被新代价覆盖");
            assert!(route.len() >= 2);
        }
    }

    fn interleaved_cost_change(explicit_snapshot: bool) {
        for calibrated in [true, false] {
            for initially_nonempty in [false, true] {
                let planner = Arc::new(PctPlanner::default());
                if initially_nonempty { planner.report_local_blockage([0.0,0.0,0.062],0.3); }
                let snapshot = planner.local_costs();
                let before = penalty_bits(&snapshot);
                let worker_source = source(calibrated);
                let worker_planner = planner.clone();
                let (ready, entered) = mpsc::channel();
                let (release, resume) = mpsc::channel();
                let worker = std::thread::spawn(move || {
                    // 显式入口的首个 checkpoint 已在调用方捕获 A 之后；旧
                    // wrapper 先做取消 preflight，再捕获 A，因此跳过该 preflight。
                    // 暂停时快照均已固定，另一线程改为 B 后才继续真实搜索。
                    let after_snapshot = if explicit_snapshot { 0 } else { 1 };
                    let result = crate::cancellation::after_checkpoint_action(after_snapshot, move || {
                        ready.send(()).unwrap();
                        resume.recv_timeout(Duration::from_secs(5)).unwrap();
                    }, || route(&worker_planner, &worker_source, calibrated,
                        explicit_snapshot.then_some(snapshot.as_slice()), None));
                    (result, snapshot)
                });
                entered.recv_timeout(Duration::from_secs(5)).unwrap();
                if initially_nonempty {
                    planner.clear_local_costs();
                } else {
                    planner.report_local_blockage([0.0,0.0,0.062],0.3);
                }
                assert_eq!(planner.local_costs().is_empty(), initially_nonempty);
                release.send(()).unwrap();
                let (result, retained_snapshot) = worker.join().unwrap();
                assert_outcome(result, initially_nonempty);
                assert_eq!(penalty_bits(&retained_snapshot), before,
                    "竖直 bias 只能修改搜索局部副本，不能改变 memo 捕获的 A");
                // 下一次原公开入口必须看见 B；没有把代价永久缓存为 A。
                assert_outcome(route(&planner, &source(calibrated), calibrated, None, None),
                    !initially_nonempty);
            }
        }
    }

    #[test]
    fn explicit_penalty_snapshot_survives_concurrent_live_cost_change() {
        interleaved_cost_change(true);
    }

    #[test]
    fn legacy_wrappers_capture_once_before_search_and_release_cost_lock() {
        interleaved_cost_change(false);
    }

    #[test]
    fn legacy_wrapper_preflight_precedes_cost_snapshot() {
        for calibrated in [true, false] {
            let planner = Arc::new(PctPlanner::default());
            planner.report_local_blockage([0.0,0.0,0.062],0.3);
            let changed = planner.clone();
            // 在首个 preflight 中改变 live costs：wrapper 随后必须捕获空 B。
            // 若先读取 costs 再做 checkpoint，将错误地保留非空 A 并耗尽预算。
            let result = crate::cancellation::after_checkpoint_action(0,
                move || changed.clear_local_costs(),
                || route(&planner, &source(calibrated), calibrated, None, None));
            assert_outcome(result, false);
        }
    }

    #[test]
    fn legacy_and_snapshot_entries_keep_existing_cancellation_precedence() {
        let planner = PctPlanner::default();
        let cancelled = AtomicBool::new(true);
        let penalties = [RoutePenalty { center: [0.0,0.0,0.062], radius: 0.3, weight: 16.0 }];
        for calibrated in [true, false] {
            for snapshot in [None, Some(penalties.as_slice())] {
                let error = route(&planner, &source(calibrated), calibrated,
                    snapshot, Some(&cancelled)).unwrap_err();
                assert_eq!(error.to_string(), "层析规划已取消");
            }
        }
    }
}
