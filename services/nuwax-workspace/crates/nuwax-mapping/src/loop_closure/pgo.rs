//! 位姿图数据结构
//! PoseGraph + PoseNode + PoseEdge 三层抽象。

use nalgebra::{Isometry3, Matrix6, Translation3, UnitQuaternion, Vector3, Vector6};
use serde::{Deserialize, Serialize};

/// 单节点（位姿 + 时间戳）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseNode {
    /// 节点 ID（单调递增）
    pub id: u64,
    /// 时间戳（epoch 秒）
    pub stamp_secs: i64,
    /// 当前位姿估计
    pub pose: Isometry3<f64>,
    /// 是否固定（anchor）
    pub fixed: bool,
}

impl PoseNode {
    /// 构造新节点
    #[must_use]
    pub fn new(id: u64, stamp_secs: i64, pose: Isometry3<f64>) -> Self {
        Self {
            id,
            stamp_secs,
            pose,
            fixed: false,
        }
    }
}

/// 单条边（两个节点间的相对位姿约束 + 6x6 信息矩阵）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseEdge {
    /// 起始节点 ID
    pub from: u64,
    /// 终止节点 ID
    pub to: u64,
    /// 测量相对位姿（from → to）
    pub measurement: Isometry3<f64>,
    /// 信息矩阵（6x6 = inverse covariance）
    pub information: Matrix6<f64>,
}

impl PoseEdge {
    /// 构造新边（默认信息矩阵 = 6x6 identity）
    #[must_use]
    pub fn new(from: u64, to: u64, measurement: Isometry3<f64>) -> Self {
        Self {
            from,
            to,
            measurement,
            information: Matrix6::identity(),
        }
    }

    /// 构造带噪声权重的边
    #[must_use]
    pub fn with_information(mut self, information: Matrix6<f64>) -> Self {
        self.information = information;
        self
    }
}

/// 位姿图
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct PoseGraph {
    /// 节点（id → PoseNode）
    pub nodes: std::collections::HashMap<u64, PoseNode>,
    /// 边列表
    pub edges: Vec<PoseEdge>,
    /// 下个节点 ID
    next_id: u64,
}

impl PoseGraph {
    /// 构造空图
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// 添加节点
    pub fn add_node(&mut self, stamp_secs: i64, pose: Isometry3<f64>) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        self.nodes.insert(id, PoseNode::new(id, stamp_secs, pose));
        id
    }

    /// 添加边
    pub fn add_edge(&mut self, edge: PoseEdge) {
        self.edges.push(edge);
    }

    /// 固定第一个节点（避免 GAUGE 自由度问题）
    pub fn fix_first_node(&mut self) {
        if let Some((_, node)) = self.nodes.iter_mut().next() {
            node.fixed = true;
        }
    }

    /// 节点数量
    #[must_use]
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// 边数量
    #[must_use]
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }
}

/// 位姿向量 ↔ Isometry3 转换工具
pub mod transform_utils {
    use super::*;

    /// Isometry3 → 6D twist 向量（[tx, ty, tz, rx, ry, rz]）
    ///
    /// 平移直接取出；旋转转欧拉角（roll/pitch/yaw）。
    #[must_use]
    pub fn isometry_to_twist(pose: &Isometry3<f64>) -> Vector6<f64> {
        let t = pose.translation;
        let (roll, pitch, yaw) = pose.rotation.euler_angles();
        Vector6::new(t.x, t.y, t.z, roll, pitch, yaw)
    }

    /// 6D twist 向量 → Isometry3
    #[must_use]
    pub fn twist_to_isometry(twist: &Vector6<f64>) -> Isometry3<f64> {
        let t = Translation3::new(twist[0], twist[1], twist[2]);
        let r = UnitQuaternion::from_euler_angles(twist[3], twist[4], twist[5]);
        Isometry3::from_parts(t, r)
    }

    /// 增量更新：tangent-space 加法（SE3 局部参数化）
    #[must_use]
    pub fn exp_update(pose: &Isometry3<f64>, delta: &Vector6<f64>) -> Isometry3<f64> {
        // 简化：translation 直接加，rotation 用 quaternion 乘
        let t = Translation3::new(
            pose.translation.x + delta[0],
            pose.translation.y + delta[1],
            pose.translation.z + delta[2],
        );
        let r = pose.rotation
            * UnitQuaternion::from_euler_angles(delta[3], delta[4], delta[5]);
        Isometry3::from_parts(t, r)
    }

    /// 计算两 pose 间相对位姿
    #[must_use]
    pub fn relative_pose(from: &Isometry3<f64>, to: &Isometry3<f64>) -> Isometry3<f64> {
        from.inverse() * to
    }

    /// 位姿误差向量（用于 PGO 残差）
    #[must_use]
    pub fn pose_error(measured: &Isometry3<f64>, predicted: &Isometry3<f64>) -> Vector6<f64> {
        let t = measured.translation.vector - predicted.translation.vector;
        // 旋转误差：measured * predicted.inverse() → log
        let r_diff = measured.rotation * predicted.rotation.inverse();
        let (r_err, p_err, y_err) = r_diff.euler_angles();
        Vector6::new(t.x, t.y, t.z, r_err, p_err, y_err)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn add_node_assigns_unique_ids() {
        let mut g = PoseGraph::new();
        let id1 = g.add_node(0, Isometry3::identity());
        let id2 = g.add_node(1, Isometry3::identity());
        assert_ne!(id1, id2);
        assert_eq!(g.num_nodes(), 2);
    }

    #[test]
    fn add_edge_basic() {
        let mut g = PoseGraph::new();
        let id1 = g.add_node(0, Isometry3::identity());
        let id2 = g.add_node(1, Isometry3::identity());
        let edge = PoseEdge::new(id1, id2, Isometry3::identity());
        g.add_edge(edge);
        assert_eq!(g.num_edges(), 1);
    }

    #[test]
    fn fix_first_node() {
        let mut g = PoseGraph::new();
        let id = g.add_node(0, Isometry3::identity());
        g.fix_first_node();
        assert!(g.nodes[&id].fixed);
    }

    #[test]
    fn twist_roundtrip() {
        let original = Isometry3::translation(1.0, 2.0, 3.0);
        let twist = transform_utils::isometry_to_twist(&original);
        let back = transform_utils::twist_to_isometry(&twist);
        assert_relative_eq!(back.translation.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(back.translation.y, 2.0, epsilon = 1e-9);
        assert_relative_eq!(back.translation.z, 3.0, epsilon = 1e-9);
    }

    #[test]
    fn relative_pose_identity() {
        let p = Isometry3::identity();
        let r = transform_utils::relative_pose(&p, &p);
        assert_relative_eq!(r.translation.x, 0.0, epsilon = 1e-9);
    }

    #[test]
    fn pose_error_zero() {
        let p = Isometry3::translation(1.0, 0.0, 0.0);
        let e = transform_utils::pose_error(&p, &p);
        assert_relative_eq!(e.norm(), 0.0, epsilon = 1e-9);
    }
}