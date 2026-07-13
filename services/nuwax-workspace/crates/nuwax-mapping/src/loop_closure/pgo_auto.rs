//! 自动回环检测
//! 基于距离阈值的自动回环检测：
//! 1. 维护当前位姿的 kiddo KD-tree
//! 2. 每加入新节点，查询最近历史节点
//! 3. 若距离 < min_distance 且时间间隔 > min_time_diff，添加回环边

use super::pgo::{transform_utils, PoseEdge, PoseGraph};
use kiddo::KdTree;
use nalgebra::{Isometry3, Point3};

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

    /// 检测并添加回环边到 graph
    ///
    /// Returns 添加的回环边数量。
    pub fn detect_and_add(&self, graph: &mut PoseGraph) -> usize {
        if graph.num_nodes() < 2 {
            return 0;
        }

        // 收集所有节点（id, stamp_secs, position）
        let nodes: Vec<(u64, i64, Point3<f64>)> = graph
            .nodes
            .iter()
            .map(|(id, n)| (*id, n.stamp_secs, n.pose.translation.vector.into()))
            .collect();

        // 构建 KD-tree（key = position, value = node index；kiddo 4.x 默认 K=const usize）
        let mut tree: KdTree<f64, 3> = KdTree::new();
        for (idx, &(_, _, p)) in nodes.iter().enumerate() {
            tree.add(&[p.x, p.y, p.z], idx as u64);
        }

        let mut added = 0;

        // 对每个新节点，查询最近邻
        for (idx, &(id_i, stamp_i, pos_i)) in nodes.iter().enumerate() {
            // 在半径 min_distance 内的所有近邻（kiddo 4.x API:metric 类型参数）
            let neighbors: Vec<(u64, f64)> = tree
                .within::<kiddo::SquaredEuclidean>(&[pos_i.x, pos_i.y, pos_i.z], self.min_distance)
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

                // 添加回环边：测量 = from.inverse() * to
                let from_pose = graph.nodes[&id_i].pose;
                let to_pose = graph.nodes[&id_j].pose;
                let measurement = transform_utils::relative_pose(&from_pose, &to_pose);
                graph.add_edge(PoseEdge::new(id_i, id_j, measurement));
                added += 1;

                let _ = dist_sq; // 抑制未使用警告
            }
        }

        added
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
        let id3 = g.add_node(60, Isometry3::identity()); // 回到起点
        assert_eq!(g.num_nodes(), 4);

        let detector = AutoLoopDetector::new(0.5, 30);
        let added = detector.detect_and_add(&mut g);
        assert_eq!(added, 1, "should detect 1 loop closure");
        assert_eq!(g.num_edges(), 1);
        // 回环边应在 id0 和 id3 之间(顺序由 HashMap iteration 决定,不指定 from/to 顺序)
        let edge = &g.edges[0];
        assert!(
            (edge.from == id0 && edge.to == id3) || (edge.from == id3 && edge.to == id0),
            "loop edge must connect id0={} and id3={} (got from={} to={})",
            id0, id3, edge.from, edge.to
        );
    }

    #[test]
    fn no_loop_when_distance_too_large() {
        let mut g = PoseGraph::new();
        let _ = g.add_node(0, Isometry3::identity());
        let _ = g.add_node(60, Isometry3::translation(10.0, 0.0, 0.0));
        let detector = AutoLoopDetector::new(1.0, 30);
        let added = detector.detect_and_add(&mut g);
        assert_eq!(added, 0);
    }

    #[test]
    fn no_loop_when_time_diff_too_small() {
        let mut g = PoseGraph::new();
        let _ = g.add_node(0, Isometry3::identity());
        let _ = g.add_node(5, Isometry3::identity()); // 5 秒 < 30 秒阈值
        let detector = AutoLoopDetector::new(1.0, 30);
        let added = detector.detect_and_add(&mut g);
        assert_eq!(added, 0);
    }
}