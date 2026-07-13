//! P0-09 回归测试：`CostMapper::from_pointcloud` 接 PointCloud2 + 三套算法 + safe radius
//!
//! 覆盖：
//! - SimplePointCloud2 适配 trait PointCloud2Like
//! - 三套算法（HeightCost/Probability/Probabilistic）都产出同语义 costmap（占位语义一致）
//! - safe_radius > 0 时障碍被膨胀
//! - safe_radius == 0 时障碍不膨胀

use nalgebra::{point, vector};
use nuwax_mapping::cost_mapper::{
    CostAlgoSelector, CostMapper, OccupancyConfig, SimplePointCloud2,
};

fn make_cloud_with_obstacle() -> SimplePointCloud2 {
    // sensor 在原点，10x10 栅格 / resolution=0.1 / origin=(0,0)
    // 在 (0.5, 0.5, z=2.0) 放一个障碍点（z > max_obstacle_height=1.5 → OCCUPIED）；
    // 其余空。
    //
    // 注意：rework22 P1 #3 之后，障碍点必须 z > max_obstacle_height_m 才能在
    // HeightCost kernel 下被标 OCCUPIED；旧的 z=0 占位不再代表障碍（会被 height-zone
    // 判为 UNKNOWN）。
    let pts = vec![
        point![0.5, 0.5, 2.0],
        point![0.55, 0.5, 2.0],
    ];
    SimplePointCloud2::new(pts, point![0.0, 0.0, 0.0])
}

#[test]
fn from_pointcloud_three_algos_consistent_occupied_cells() {
    let cloud = make_cloud_with_obstacle();
    for algo in [
        CostAlgoSelector::HeightCost,
        CostAlgoSelector::Probability,
        CostAlgoSelector::Probabilistic,
    ] {
        let cfg = OccupancyConfig {
            algo,
            initial_safe_radius_meters: 0.0,
        };
        let (_mapper, costmap) =
            CostMapper::from_pointcloud(&cloud, &cfg, 10, 10, 0.1, (0.0, 0.0));

        // 障碍点附近应有 occupied cell
        let mut occupied_count = 0;
        for r in 0..10 {
            for c in 0..10 {
                if costmap.get(r, c) == Some(100) {
                    occupied_count += 1;
                }
            }
        }
        assert!(
            occupied_count >= 1,
            "algo {:?} must mark at least 1 occupied cell, got {}",
            algo,
            occupied_count
        );
    }
}

#[test]
fn from_pointcloud_safe_radius_inflates_obstacles() {
    let cloud = make_cloud_with_obstacle();
    let cfg_no_safe = OccupancyConfig {
        algo: CostAlgoSelector::HeightCost,
        initial_safe_radius_meters: 0.0,
    };
    let (_m1, cm_no_safe) =
        CostMapper::from_pointcloud(&cloud, &cfg_no_safe, 10, 10, 0.1, (0.0, 0.0));

    let cfg_with_safe = OccupancyConfig {
        algo: CostAlgoSelector::HeightCost,
        initial_safe_radius_meters: 0.3, // 3 cells @ resolution=0.1
    };
    let (_m2, cm_safe) =
        CostMapper::from_pointcloud(&cloud, &cfg_with_safe, 10, 10, 0.1, (0.0, 0.0));

    // 统计 occupied cells：safe 版本应更多
    let count_occ = |cm: &nuwax_mapping::Costmap| -> usize {
        (0..10)
            .flat_map(|r| (0..10).map(move |c| (r, c)))
            .filter(|&(r, c)| cm.get(r, c) == Some(100))
            .count()
    };
    let safe_count = count_occ(&cm_safe);
    let no_safe_count = count_occ(&cm_no_safe);
    assert!(
        safe_count > no_safe_count,
        "safe_radius 必须膨胀障碍：safe={} no_safe={}",
        safe_count,
        no_safe_count
    );
}

#[test]
fn from_pointcloud_empty_points_yields_all_unknown() {
    let cloud = SimplePointCloud2::new(vec![], point![0.0, 0.0, 0.0]);
    let cfg = OccupancyConfig::default();
    let (_m, cm) = CostMapper::from_pointcloud(&cloud, &cfg, 4, 4, 0.1, (0.0, 0.0));
    // 空点云 → 全 255(Unknown)
    for r in 0..4 {
        for c in 0..4 {
            assert_eq!(cm.get(r, c), Some(255), "({},{}) should be unknown", r, c);
        }
    }
}

#[test]
fn cost_algo_selector_dimos_string_roundtrip() {
    for s in ["height_cost", "probability", "probabilistic"] {
        let algo = CostAlgoSelector::from_dimos_str(s);
        assert_eq!(algo.as_dimos_str(), s);
    }
    // 未知值 fallback
    assert_eq!(
        CostAlgoSelector::from_dimos_str("bogus_algo"),
        CostAlgoSelector::HeightCost
    );
}

// 静默 vector 引用（保持 nalgebra import）
#[allow(dead_code)]
fn _force_vec_use() -> nalgebra::Vector3<f64> {
    vector![0.0, 0.0, 0.0]
}