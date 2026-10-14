//! 自动回环检测
//! 基于距离阈值的自动回环检测：
//! 1. 维护当前位姿的 kiddo KD-tree
//! 2. 每加入新节点，查询最近历史节点
//! 3. 若距离 < min_distance 且时间间隔 > min_time_diff，添加回环边

use super::pgo::{PoseEdge, PoseGraph, PoseNode};
use kiddo::KdTree;
use nalgebra::{Isometry3, Matrix6, Point3};

/// 由 ICP、marker 或其他独立观测估计器产生的回环约束。
#[derive(Debug, Clone)]
pub struct LoopConstraint {
    /// `from -> to` 的观测相对位姿，不能由当前 graph estimates 自行生成。
    pub measurement: Isometry3<f64>,
    /// 观测信息矩阵。
    pub information: Matrix6<f64>,
}

impl LoopConstraint {
    /// 使用单位信息矩阵构造约束。
    #[must_use]
    pub fn new(measurement: Isometry3<f64>) -> Self {
        Self {
            measurement,
            information: Matrix6::identity(),
        }
    }
}

/// 自动回环检测器
pub struct AutoLoopDetector {
    /// 最小距离阈值（m）—— 距离小于此值的两个位姿视为候选回环
    pub min_distance: f64,
    /// 最小时间差（秒）—— 间隔小于此值的位姿不视为回环（避免相邻帧误判）
    pub min_time_diff_secs: i64,
}

impl Default for AutoLoopDetector {
    fn default() -> Self {
        Self {
            min_distance: 1.0,
            min_time_diff_secs: 30,
        }
    }
}

impl AutoLoopDetector {
    /// 构造
    #[must_use]
    pub fn new(min_distance: f64, min_time_diff_secs: i64) -> Self {
        Self {
            min_distance,
            min_time_diff_secs,
        }
    }

    /// 检测候选，并仅在 estimator 返回独立观测时添加回环边。
    ///
    /// estimator 的输入节点顺序与最终 edge 的 `from/to` 一致。返回 `None`
    /// 表示 ICP/marker fitness gate 拒绝该候选。
    pub fn detect_and_add<F>(&self, graph: &mut PoseGraph, mut estimator: F) -> usize
    where
        F: FnMut(&PoseNode, &PoseNode) -> Option<LoopConstraint>,
    {
        let candidates = self.detect_candidates(graph);
        let mut added = 0;
        for (from, to) in candidates {
            let constraint = {
                let from_node = &graph.nodes[&from];
                let to_node = &graph.nodes[&to];
                estimator(from_node, to_node)
            };
            let Some(constraint) = constraint else {
                continue;
            };
            graph.add_edge(
                PoseEdge::new(from, to, constraint.measurement)
                    .with_information(constraint.information),
            );
            added += 1;
        }
        added
    }

    /// 仅执行空间/时间候选搜索，不生成测量。
    #[must_use]
    pub fn detect_candidates(&self, graph: &PoseGraph) -> Vec<(u64, u64)> {
        if graph.num_nodes() < 2 || !self.min_distance.is_finite() || self.min_distance <= 0.0 {
            return Vec::new();
        }

        // 收集所有节点（id, stamp_secs, position）
        let mut nodes: Vec<(u64, i64, Point3<f64>)> = graph
            .nodes
            .iter()
            .map(|(id, n)| (*id, n.stamp_secs, n.pose.translation.vector.into()))
            .collect();
        nodes.sort_by_key(|(id, _, _)| *id);

        // 构建 KD-tree（key = position, value = node index；kiddo 4.x 默认 K=const usize）
        let mut tree: KdTree<f64, 3> = KdTree::new();
        for (idx, &(_, _, p)) in nodes.iter().enumerate() {
            tree.add(&[p.x, p.y, p.z], idx as u64);
        }

        let mut candidates = Vec::new();

        // 对每个新节点，查询最近邻
        for (idx, &(id_i, stamp_i, pos_i)) in nodes.iter().enumerate() {
            // 在半径 min_distance 内的所有近邻（kiddo 4.x API:metric 类型参数）
            let neighbors: Vec<(u64, f64)> = tree
                .within::<kiddo::SquaredEuclidean>(
                    &[pos_i.x, pos_i.y, pos_i.z],
                    self.min_distance * self.min_distance,
                )
                .iter()
                .map(|n| (n.item, n.distance))
                .collect();

            for (jdx_u64, dist_sq) in neighbors {
                let jdx = jdx_u64 as usize;
                if jdx <= idx {
                    continue; // 仅向后查询（j > i）
                }
                let (id_j, stamp_j, _) = nodes[jdx];
                if (stamp_i - stamp_j).abs() < self.min_time_diff_secs {
                    continue;
                }

                candidates.push((id_i, id_j));

                let _ = dist_sq; // 抑制未使用警告
            }
        }

        candidates
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_simple_loop() {
        let mut g = PoseGraph::new();
        // 创建回环：node id0 和 node id3 在同一位置
        let id0 = g.add_node(0, Isometry3::identity());
        let _id1 = g.add_node(10, Isometry3::translation(1.0, 0.0, 0.0));
        let _id2 = g.add_node(20, Isometry3::translation(2.0, 0.0, 0.0));
        let id3 = g.add_node(60, Isometry3::translation(0.2, 0.0, 0.0));
        assert_eq!(g.num_nodes(), 4);

        let detector = AutoLoopDetector::new(0.5, 30);
        let added = detector.detect_and_add(&mut g, |from, to| {
            assert!((from.id == id0 && to.id == id3) || (from.id == id3 && to.id == id0));
            Some(LoopConstraint::new(Isometry3::identity()))
        });
        assert_eq!(added, 1, "should detect 1 loop closure");
        assert_eq!(g.num_edges(), 1);
        // 回环边应在 id0 和 id3 之间(顺序由 HashMap iteration 决定,不指定 from/to 顺序)
        let edge = &g.edges[0];
        assert!(
            (edge.from == id0 && edge.to == id3) || (edge.from == id3 && edge.to == id0),
            "loop edge must connect id0={} and id3={} (got from={} to={})",
            id0,
            id3,
            edge.from,
            edge.to
        );
        assert_eq!(edge.measurement, Isometry3::identity());
        let predicted = g.nodes[&edge.from].pose.inverse() * g.nodes[&edge.to].pose;
        assert_ne!(
            edge.measurement, predicted,
            "measurement must be independent"
        );
    }

    #[test]
    fn no_loop_when_distance_too_large() {
        let mut g = PoseGraph::new();
        let _ = g.add_node(0, Isometry3::identity());
        let _ = g.add_node(60, Isometry3::translation(10.0, 0.0, 0.0));
        let detector = AutoLoopDetector::new(1.0, 30);
        let added = detector.detect_and_add(&mut g, |_, _| {
            panic!("distance-rejected candidate must not call estimator")
        });
        assert_eq!(added, 0);
    }

    #[test]
    fn candidate_radius_is_linear_meters_not_squared_metric_value() {
        let mut outside = PoseGraph::new();
        outside.add_node(0, Isometry3::identity());
        outside.add_node(60, Isometry3::translation(0.6, 0.0, 0.0));
        let detector = AutoLoopDetector::new(0.5, 30);
        assert!(detector.detect_candidates(&outside).is_empty());

        let mut inside = PoseGraph::new();
        inside.add_node(0, Isometry3::identity());
        inside.add_node(60, Isometry3::translation(0.49, 0.0, 0.0));
        assert_eq!(detector.detect_candidates(&inside).len(), 1);
    }

    #[test]
    fn no_loop_when_time_diff_too_small() {
        let mut g = PoseGraph::new();
        let _ = g.add_node(0, Isometry3::identity());
        let _ = g.add_node(5, Isometry3::identity()); // 5 秒 < 30 秒阈值
        let detector = AutoLoopDetector::new(1.0, 30);
        let added = detector.detect_and_add(&mut g, |_, _| {
            panic!("time-rejected candidate must not call estimator")
        });
        assert_eq!(added, 0);
    }

    #[test]
    fn estimator_can_reject_spatial_candidate() {
        let mut g = PoseGraph::new();
        let _ = g.add_node(0, Isometry3::identity());
        let _ = g.add_node(60, Isometry3::translation(0.1, 0.0, 0.0));
        let detector = AutoLoopDetector::new(0.5, 30);
        let added = detector.detect_and_add(&mut g, |_, _| None);
        assert_eq!(added, 0);
        assert!(g.edges.is_empty());
    }
}
