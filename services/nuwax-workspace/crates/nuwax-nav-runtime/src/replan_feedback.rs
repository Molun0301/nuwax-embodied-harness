//! 实际执行反馈与已发布全局结果的轻量比较。
use crate::{ExecutionGoal, PathExecutionIdentity, SurfaceSnapshot};
use nuwax_nav_types_messages::{GoalOptions, PoseStamped, Twist2};
use std::sync::Arc;
/// 已消费的真实输入，预测轨迹不得修改它。
#[derive(Clone, Debug)]
pub struct ActualExecutionContext {
    /// 当前感知来源。
    pub source: Arc<SurfaceSnapshot>,
    /// 实际里程计。
    pub odometry: PoseStamped,
    /// 运动端确认应用的命令值；生产来源为已应用回执，不代表测速传感器读数。
    pub applied: Twist2,
}
/// 对应原路径身份的执行失败与实际上下文。
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceReplanFeedback {
    /// 原路径身份。
    pub path_identity: Arc<PathExecutionIdentity>,
    /// 失败计算消费的输入。
    pub rejected_context: Option<Arc<ActualExecutionContext>>,
    /// 最新已消费的输入。
    pub current_context: Option<Arc<ActualExecutionContext>>,
    /// 原归零链已经确认停车。
    pub zero_confirmed_rejection: bool,
}
/// 全局规划实际使用的输入。
#[derive(Clone)]
pub struct SurfaceReplanInput {
    /// 原地图。
    pub source: Arc<SurfaceSnapshot>,
    /// 实际起点。
    pub start: PoseStamped,
    /// 原目标。
    pub goal: ExecutionGoal,
    /// 目标容差。
    pub options: GoalOptions,
    /// 原边界。
    pub boundary: Arc<Vec<[f64; 2]>>,
    /// 已消费的局部执行输入。
    pub actual: Option<Arc<ActualExecutionContext>>,
    /// 实际机体到地面距离。
    pub offset: f64,
}
/// 排除传输时刻，保留全部位置及朝向。
pub fn same_pose(a: &PoseStamped, b: &PoseStamped) -> bool {
    let bits = |p: &PoseStamped| {
        let p = p.pose;
        [
            p.position.x,
            p.position.y,
            p.position.z,
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        ]
        .map(f64::to_bits)
    };
    a.frame_id == b.frame_id && bits(a) == bits(b)
}
/// 地图、位姿、已应用速度全部相同才复用实际上下文。
pub fn same_actual(a: &ActualExecutionContext, b: &ActualExecutionContext) -> bool {
    a.source.same_body_query_view(&b.source)
        && same_pose(&a.odometry, &b.odometry)
        && [a.applied.linear.x, a.applied.linear.y, a.applied.angular_z].map(f64::to_bits)
            == [b.applied.linear.x, b.applied.linear.y, b.applied.angular_z].map(f64::to_bits)
}
impl SurfaceReplanInput {
    /// 缺少实际上下文时不复用失败后的旧结果。
    pub fn same_complete_input(&self, other: &Self) -> bool {
        self.actual
            .as_deref()
            .zip(other.actual.as_deref())
            .is_some_and(|(a, b)| same_actual(a, b))
            && crate::native_map::same_source_geometry(&self.source, &other.source)
            && same_pose(&self.start, &other.start)
            && self.goal.same_geometry(&other.goal)
            && self.options == other.options
            && self.boundary == other.boundary
            && self.offset.to_bits() == other.offset.to_bits()
    }
}
/// One published success, not an unbounded history or a blocked-state database.
/// K is the original path identity (production) or existing plan counter (sim).
pub struct SuccessfulReplanMemo<K> {
    published: Option<(K, SurfaceReplanInput)>,
}
impl<K> Default for SuccessfulReplanMemo<K> {
    fn default() -> Self {
        Self { published: None }
    }
}
impl<K: PartialEq> SuccessfulReplanMemo<K> {
    /// Call only after the original publish/admission succeeds, never on Err or
    /// an obsolete task's completion. Existing tickets still retire late results.
    pub fn published(&mut self, path: K, input: SurfaceReplanInput) {
        self.published = Some((path, input));
    }
    /// Reuse only a correlated, actual execution refusal after its original stop
    /// confirmation. The caller supplies the unmodified rejected context, or
    /// None for an initial wait, missing proof, stale owner or pending planner.
    /// A success at A followed by refusal at B cannot suppress a retry back at A.
    pub fn should_reuse(
        &self,
        rejection: Option<(&K, &ActualExecutionContext)>,
        input: &SurfaceReplanInput,
    ) -> bool {
        self.published.as_ref().zip(rejection).is_some_and(
            |((path, previous), (rejected_path, rejected_context))| {
                path == rejected_path
                    && input
                        .actual
                        .as_deref()
                        .is_some_and(|current| same_actual(rejected_context, current))
                    && previous.same_complete_input(input)
            },
        )
    }
    /// Original owner/goal/domain retirement, not a new synthetic generation.
    pub fn clear(&mut self) {
        self.published = None;
    }
}

impl PartialEq for ActualExecutionContext {
    fn eq(&self, other: &Self) -> bool {
        same_actual(self, other)
    }
}
