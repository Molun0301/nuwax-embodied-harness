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
        // The owner's flag must reject obsolete work before deriving a tomogram,
        // including the near-goal return. Reuse the search's existing semantics.
        crate::search::checkpoint(cancelled)?;
        let model = map
            .body_envelope()
            .ok_or("层析规划需要机器人外形和支撑标定")?;
        self.reference_body_route_with_offset_typed(map,start,goal,options,boundary,cancelled,limits,
            model.body_to_support_z_m)
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
        let mut penalties = self.local_costs();
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
