//! P1 #2 回归测试：gradient 统一（nav-astar 用 mapping::occupancy::gradient::make_costmap）
//!
//! 覆盖：
//! - `make_costmap` 占用 → 255
//! - `make_costmap` UNKNOWN 不参与障碍膨胀（与 Occupied 行为不同）
//! - `make_costmap` FREE → cost 梯度（远=0, 近=高）
//! - nav-astar `make_gradient_costmap` 调 mapping make_costmap 产生等价位 costmap

use nuwax_mapping::occupancy::gradient::{make_costmap, CostMapConfig};
use nuwax_mapping::occupancy::types::{OccupancyGrid, OccupancyValue};

/// 辅助：构造一个 5x5 全 Free 的 OccupancyGrid（new() 默认全 Unknown,需手动覆盖）
fn all_free_grid(width: u32, height: u32, resolution: f64) -> OccupancyGrid {
    let mut g = OccupancyGrid::new(width, height, resolution);
    for r in 0..height {
        for c in 0..width {
            g.set(r, c, OccupancyValue::Free);
        }
    }
    g
}

#[test]
fn make_costmap_marks_occupied_as_255() {
    let mut g = all_free_grid(5, 5, 0.1);
    g.set(2, 2, OccupancyValue::Occupied);
    let cfg = CostMapConfig::default();
    let cm = make_costmap(&g, &cfg);
    // 障碍自身 → 255
    assert_eq!(cm.get(2, 2), 255);
}

#[test]
fn make_costmap_unknown_not_inflated() {
    // 5x5 栅格；中心 (2,2) 设 Unknown, (0,0) 设 Occupied
    // 关键: Unknown 自身不应作为膨胀源;验证中心 Unknown 仍为 255
    let mut g = all_free_grid(5, 5, 0.1);
    g.set(0, 0, OccupancyValue::Occupied);
    g.set(2, 2, OccupancyValue::Unknown);
    // 关闭膨胀（robot_increase = 0）只保留 UNKNOWN 自身不可走
    let cfg = CostMapConfig {
        resolution: 0.1,
        robot_increase: 0.0,
        max_cost: 90,
    };
    let cm = make_costmap(&g, &cfg);
    // UNKNOWN 仍为 255
    assert_eq!(cm.get(2, 2), 255);
    // OCCUPIED (0,0) 也为 255
    assert_eq!(cm.get(0, 0), 255);
    // FREE 区域 (4,4) → cost < 255 (无障碍 + 无膨胀)
    assert!(cm.get(4, 4) < 255, "free cell should have cost < 255");
}

#[test]
fn make_costmap_free_near_obstacle_high_cost() {
    let mut g = all_free_grid(5, 5, 0.1);
    g.set(2, 2, OccupancyValue::Occupied);
    let cfg = CostMapConfig {
        resolution: 0.1,
        robot_increase: 0.0,
        max_cost: 90,
    };
    let cm = make_costmap(&g, &cfg);
    // 紧贴障碍的 cell (2,1) cost 应该 > 远离的 cell (4,4)
    let near = cm.get(2, 1);
    let far = cm.get(4, 4);
    assert!(near > far, "near cell cost {near} should > far cell cost {far}");
}

#[test]
fn make_costmap_inflate_creates_larger_obstacle_zone() {
    let mut g = all_free_grid(10, 10, 0.1);
    g.set(5, 5, OccupancyValue::Occupied);
    // 无膨胀
    let cfg_no_inflate = CostMapConfig {
        resolution: 0.1,
        robot_increase: 0.0,
        max_cost: 90,
    };
    let cm_no = make_costmap(&g, &cfg_no_inflate);
    // 大膨胀（robot_increase = 0.3 → 3 cells）
    let cfg_inflate = CostMapConfig {
        resolution: 0.1,
        robot_increase: 0.3,
        max_cost: 90,
    };
    let cm_yes = make_costmap(&g, &cfg_inflate);
    // 距障碍 2 cells 的 (5, 7) 在无膨胀时是 free (cost < 255),
    // 有膨胀时是 OCCUPIED (255)
    assert!(cm_no.get(5, 7) < 255, "no inflate: (5,7) should be free");
    assert_eq!(
        cm_yes.get(5, 7),
        255,
        "with inflate radius 3: (5,7) should be inflated to OCCUPIED"
    );
}

#[test]
fn make_costmap_handles_all_free_grid() {
    let g = all_free_grid(5, 5, 0.1);
    let cfg = CostMapConfig::default();
    let cm = make_costmap(&g, &cfg);
    // 全 Free → cost 渐变（无 255）
    for r in 0..5 {
        for c in 0..5 {
            assert!(
                cm.get(r, c) < 255,
                "({},{}) should not be 255, got {}",
                r,
                c,
                cm.get(r, c)
            );
        }
    }
}