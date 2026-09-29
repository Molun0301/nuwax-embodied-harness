//! 全局层析参考到既有三维路径契约的转换；实际运动姿态由近场轨迹产生。
use crate::PctPlanner;
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
        let model = map
            .body_envelope()
            .ok_or("层析规划需要机器人外形和支撑标定")?;
        if crate::distance(start.0, goal.0) < options.position_tolerance_m {
            return Ok(vec![start]);
        }
        let offset = model.body_to_support_z_m;
        let start_support = map
            .support_position_from_body(start.0, start.1, offset)
            .map_err(|e| e.to_string())?;
        let goal_support = map
            .support_position_from_body(goal.0, goal.1, offset)
            .map_err(|e| e.to_string())?;
        let tomogram = self.map(map)?;
        let route = tomogram.plan_bounded(
            start_support,
            goal_support,
            options.position_tolerance_m,
            boundary,
            cancelled,
            None,
            &self.local_costs(),
        )?;
        let mut poses = vec![start];
        for (i, &point) in route.points.iter().enumerate().skip(1) {
            let previous = route.points[i - 1];
            let next = route.points.get(i + 1).copied().unwrap_or(point);
            let dx = next[0] - previous[0];
            let dy = next[1] - previous[1];
            let yaw = dy.atan2(dx);
            let q = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
            // 此处交付层析参考而非精确站姿；不能再次用旧MLS离散表面拒绝新高程。
            // 参考只有偏航，沿竖直标定平移即可，真实横滚/俯仰在Scan中从地形生成。
            let body = [point[0], point[1], point[2] + offset];
            poses.push((body, q));
        }
        tracing::info!(target:"navigation.pct",event="navigation.pct.route",expanded=route.stats.expanded,
            elapsed_ms=route.stats.elapsed_ms,points=poses.len(),"层析全局路线计算完成");
        Ok(poses)
    }
}
