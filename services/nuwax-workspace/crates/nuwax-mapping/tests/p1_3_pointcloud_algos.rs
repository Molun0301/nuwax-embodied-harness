//! P1 #3 回归测试：pointcloud occupancy 三套算法（HeightCost / General / Simple）
//! + Bresenham 不应再出现在 `pointclouds/occupancy.rs`
//!
//! 覆盖：
//! - `p1_3_height_cost_marks_above_ground_as_free`：默认 HeightCost kernel
//!   对高于 ground + voxel/2 的点给出 FREE
//! - `p1_3_height_cost_marks_below_ground_as_unknown`：低于 ground 的点为 UNKNOWN
//! - `p1_3_height_cost_marks_high_as_occupied`：高过 max_obstacle_height 为 OCCUPIED
//! - `p1_3_general_and_simple_also_classify_three_zones`：General / Simple 两个
//!   kernel 同样能给出 FREE/UNKNOWN/OCCUPIED 三档
//! - `p1_3_no_bresenham_in_source`：源文件 `occupancy.rs` 中无
//!   `Bresenham` / `bresenham` 字符串

use nuwax_mapping::pointclouds::occupancy::{
    make_costmap2d, project_to_occupancy, PointCloudOccAlgo, PointCloudOccConfig,
    PointCloudOccupancyBuilder,
};
use nuwax_mapping::cost_mapper::{PointCloud2Like, SimplePointCloud2};

fn cost_label(v: u8) -> &'static str {
    match v {
        0 => "FREE",
        100 => "OCCUPIED",
        255 => "UNKNOWN",
        _ => "OTHER",
    }
}

#[test]
fn p1_3_height_cost_marks_above_ground_as_free() {
    let cfg = PointCloudOccConfig {
        algo: PointCloudOccAlgo::HeightCost,
        ground_height_m: 0.0,
        max_obstacle_height_m: 1.5,
        robot_height_m: 0.5,
        voxel_size_m: 0.05,
    };
    // z = 0.5m, ground + voxel = 0.05 → 0.5 in [0.05, 1.5] → FREE
    let mut g = make_costmap2d(3, 3, 0.1);
    let pts = vec![nalgebra::point![0.1, 0.1, 0.5]];
    let cloud = SimplePointCloud2::new(pts, nalgebra::point![0.0, 0.0, 0.0]);
    project_to_occupancy(&cloud, &mut g, &cfg).expect("ok");
    // 0.1/0.1 = col=1, row=1 (origin (0,0))
    assert_eq!(
        cost_label(g.get(1, 1).unwrap()),
        "FREE",
        "0.5m should be FREE under HeightCost"
    );
}

#[test]
fn p1_3_height_cost_marks_below_ground_as_unknown() {
    let cfg = PointCloudOccConfig {
        algo: PointCloudOccAlgo::HeightCost,
        ground_height_m: 0.0,
        max_obstacle_height_m: 1.5,
        robot_height_m: 0.5,
        voxel_size_m: 0.05,
    };
    // z = -0.5m < 0.05 → UNKNOWN
    let mut g = make_costmap2d(3, 3, 0.1);
    let pts = vec![nalgebra::point![0.1, 0.1, -0.5]];
    let cloud = SimplePointCloud2::new(pts, nalgebra::point![0.0, 0.0, 0.0]);
    project_to_occupancy(&cloud, &mut g, &cfg).expect("ok");
    assert_eq!(
        cost_label(g.get(1, 1).unwrap()),
        "UNKNOWN",
        "-0.5m should be UNKNOWN under HeightCost"
    );
}

#[test]
fn p1_3_height_cost_marks_high_as_occupied() {
    let cfg = PointCloudOccConfig {
        algo: PointCloudOccAlgo::HeightCost,
        ground_height_m: 0.0,
        max_obstacle_height_m: 1.5,
        robot_height_m: 0.5,
        voxel_size_m: 0.05,
    };
    // z = 2.0m > 1.5 → OCCUPIED
    let mut g = make_costmap2d(3, 3, 0.1);
    let pts = vec![nalgebra::point![0.1, 0.1, 2.0]];
    let cloud = SimplePointCloud2::new(pts, nalgebra::point![0.0, 0.0, 0.0]);
    project_to_occupancy(&cloud, &mut g, &cfg).expect("ok");
    assert_eq!(
        cost_label(g.get(1, 1).unwrap()),
        "OCCUPIED",
        "2.0m should be OCCUPIED under HeightCost"
    );
}

#[test]
fn p1_3_general_and_simple_also_classify_three_zones() {
    // 三个 kernel 都至少给出 FREE/UNKNOWN/OCCUPIED 三档
    for algo in [PointCloudOccAlgo::HeightCost, PointCloudOccAlgo::General, PointCloudOccAlgo::Simple] {
        let cfg = PointCloudOccConfig {
            algo,
            ground_height_m: 0.0,
            max_obstacle_height_m: 1.5,
            robot_height_m: 0.5,
            voxel_size_m: 0.05,
        };
        // 三档：低、中、高
        let mut g_low = make_costmap2d(3, 3, 0.1);
        let pts_low = vec![nalgebra::point![0.1, 0.1, -0.5]];
        project_to_occupancy(
            &SimplePointCloud2::new(pts_low, nalgebra::point![0.0, 0.0, 0.0]),
            &mut g_low,
            &cfg,
        )
        .expect("ok");
        let low = cost_label(g_low.get(1, 1).unwrap());

        let mut g_mid = make_costmap2d(3, 3, 0.1);
        let pts_mid = vec![nalgebra::point![0.1, 0.1, 0.3]];
        project_to_occupancy(
            &SimplePointCloud2::new(pts_mid, nalgebra::point![0.0, 0.0, 0.0]),
            &mut g_mid,
            &cfg,
        )
        .expect("ok");
        let mid = cost_label(g_mid.get(1, 1).unwrap());

        let mut g_high = make_costmap2d(3, 3, 0.1);
        let pts_high = vec![nalgebra::point![0.1, 0.1, 2.0]];
        project_to_occupancy(
            &SimplePointCloud2::new(pts_high, nalgebra::point![0.0, 0.0, 0.0]),
            &mut g_high,
            &cfg,
        )
        .expect("ok");
        let high = cost_label(g_high.get(1, 1).unwrap());

        // 三个 kernel 至少覆盖三档中的每一档至少一次
        assert_ne!(low, mid, "algo {algo:?} low should differ from mid");
        assert_ne!(mid, high, "algo {algo:?} mid should differ from high");
        assert_ne!(low, high, "algo {algo:?} low should differ from high");
    }
}

#[test]
fn p1_3_no_bresenham_in_source() {
    // 源文件不应再有 Bresenham 字样
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("src/pointclouds/occupancy.rs");
    let content = std::fs::read_to_string(&path).expect("read occupancy.rs");
    assert!(
        !content.contains("Bresenham"),
        "occupancy.rs must not contain 'Bresenham'; rework22 P1 #3 replaced it with three-algorithm kernel"
    );
    assert!(
        !content.contains("bresenham"),
        "occupancy.rs must not contain 'bresenham'; case-insensitive check"
    );
}

// 兜底：保证 PointCloudOccupancyBuilder 旧 API 仍可用（cost_mapper.rs 依赖）
#[test]
fn p1_3_old_builder_still_compiles_and_runs() {
    let b = PointCloudOccupancyBuilder::new(
        nalgebra::point![0.0, 0.0, 0.0],
        0.1,
        10,
        10,
        (0.0, 0.0),
    );
    let pts = vec![nalgebra::point![0.0, 0.0, 0.5]]; // 0.5m → HeightCost default 中间 → FREE
    let g = b.build(&pts);
    // 0/0/0.5 → (row=0, col=0)
    assert!(g.get(0, 0).is_some());
}

// 触发 PointCloud2Like 使用（防止 dead_code 警告）
#[allow(dead_code)]
fn _use_trait<P: PointCloud2Like>(_p: &P) {}
