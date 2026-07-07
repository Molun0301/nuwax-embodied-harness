// Copyright 2026 Nuwax.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! MLS Planner 基础 sanity 测试集。
//!
//! 覆盖 5 个层级的 sanity 校验:
//! 1. voxel 坐标 roundtrip(voxel 模块)
//! 2. SurfaceCells 插入/查找/移除(adjacency 模块)
//! 3. PlannerGraph 默认构造(edges 模块)
//! 4. Planner surface extraction 从平面点云(planner + mls_planner 模块)
//! 5. Planner 端到端小图(mls_planner 模块)

use nuwax_nav_mls_planner::adjacency::SurfaceCells;
use nuwax_nav_mls_planner::edges::PlannerGraph;
use nuwax_nav_mls_planner::mls_planner::{Config, Planner, RegionBounds};
use nuwax_nav_mls_planner::voxel::{surface_point_xyz, voxelize};

// ----------------------------------------------------------------------------
// Test 1: voxel 坐标 roundtrip
// ----------------------------------------------------------------------------
#[test]
fn voxel_roundtrip_is_consistent() {
    let vs = 0.25_f32;
    let p = (1.75_f32, -0.125_f32, 0.5_f32);
    let key = voxelize(p, vs);

    // 索引应该 <= 向下取整值
    assert!(key.0 <= (p.0 / vs).floor() as i32 + 1);
    assert!(key.1 <= (p.1 / vs).floor() as i32 + 1);
    assert!(key.2 <= (p.2 / vs).floor() as i32 + 1);

    let center = surface_point_xyz(key.0, key.1, key.2, vs);
    // x/y 中心偏差 < voxel_size
    let dx = (center.0 - p.0).abs();
    let dy = (center.1 - p.1).abs();
    assert!(dx < vs, "x deviation {dx} exceeds voxel_size {vs}");
    assert!(dy < vs, "y deviation {dy} exceeds voxel_size {vs}");
    // z 严格等于 (iz + 1) * voxel_size
    let expected_z = (key.2 as f32 + 1.0) * vs;
    assert!(
        (center.2 - expected_z).abs() < 1e-6,
        "z surface expected {expected_z}, got {}",
        center.2
    );
}

// ----------------------------------------------------------------------------
// Test 2: SurfaceCells 插入/查找/移除
// ----------------------------------------------------------------------------
#[test]
fn surface_cells_insert_and_remove() {
    let mut cells = SurfaceCells::default();
    let k1 = (0_i32, 0_i32, 0_i32);
    let k2 = (1_i32, 0_i32, 0_i32);
    let k3 = (0_i32, 1_i32, 0_i32);

    let id1 = cells.insert(k1);
    let id2 = cells.insert(k2);
    let id3 = cells.insert(k3);

    assert!(cells.is_live(id1));
    assert!(cells.is_live(id2));
    assert!(cells.is_live(id3));
    assert_eq!(cells.coord(id1), k1);
    assert_eq!(cells.coord(id2), k2);
    assert_eq!(cells.coord(id3), k3);

    // 重复插入返回相同的 id
    assert_eq!(cells.insert(k1), id1);

    // 移除
    let removed = cells.remove(k2);
    assert_eq!(removed, Some(id2));
    assert!(!cells.is_live(id2));
    assert!(cells.is_live(id1));
    assert!(cells.is_live(id3));
}

// ----------------------------------------------------------------------------
// Test 3: PlannerGraph 默认构造 + simple sanity
// ----------------------------------------------------------------------------
#[test]
fn planner_graph_default_is_empty() {
    let graph = PlannerGraph::default();
    assert!(graph.cells.is_empty());
}

// ----------------------------------------------------------------------------
// Test 4: Planner surface extraction 从平面点云
// ----------------------------------------------------------------------------
#[test]
fn planner_extracts_surface_from_flat_floor() {
    let vs = 0.1_f32;
    let mut points = Vec::new();
    let n = 20_i32;
    for ix in 0..n {
        for iy in 0..n {
            let x = (ix as f32 + 0.5) * vs;
            let y = (iy as f32 + 0.5) * vs;
            points.push((x, y, 0.05_f32));
        }
    }

    let config = Config {
        world_frame: String::new(),
        voxel_size: vs,
        robot_height: 0.5,
        surface_closing_radius: 0.3,
        node_spacing_m: 1.0,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.3,
        wall_buffer_weight: 1.0,
        step_threshold_m: 0.25,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
    };

    let mut planner = Planner::default();
    planner.update_global_map(&points, &config);

    assert!(planner.voxel_count() > 0);
    let surface_count = planner.surface().count();
    assert!(
        surface_count > 0,
        "expected at least some surface cells extracted, got 0"
    );
}

// ----------------------------------------------------------------------------
// Test 5: Planner 端到端小图(带墙的 floor slab)
// ----------------------------------------------------------------------------
#[test]
fn planner_end_to_end_does_not_panic_with_wall() {
    let config = Config {
        world_frame: String::new(),
        voxel_size: 0.1,
        robot_height: 0.5,
        surface_closing_radius: 0.3,
        node_spacing_m: 1.0,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.3,
        wall_buffer_weight: 1.0,
        step_threshold_m: 0.25,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.3,
        viz_publish_hz: 0.0,
    };

    let mut points = Vec::new();
    let vs = 0.1_f32;
    // 5x5 米 floor
    for ix in 0..50 {
        for iy in 0..50 {
            let x = (ix as f32 + 0.5) * vs;
            let y = (iy as f32 + 0.5) * vs;
            points.push((x, y, 0.05));
        }
    }
    // 中央 y=2.5 横向墙
    for ix in 0..50 {
        for iz in 0..15 {
            let x = (ix as f32 + 0.5) * vs;
            let y = 2.5_f32;
            let z = (iz as f32 + 0.5) * vs;
            points.push((x, y, z));
        }
    }

    let mut planner = Planner::default();
    planner.update_global_map(&points, &config);

    let start = (0.5_f32, 0.5_f32, 0.1_f32);
    let goal = (0.5_f32, 4.5_f32, 0.1_f32);
    // 不 panic 即可,允许 None(墙挡住)
    let _path = planner.plan(start, goal, &config);
}

// ----------------------------------------------------------------------------
// Test 6: RegionBounds 接受局部更新
// ----------------------------------------------------------------------------
#[test]
fn region_bounds_is_constructible() {
    let bounds = RegionBounds {
        origin_x: 1.0,
        origin_y: 2.0,
        radius: 3.0,
        z_min: 0.0,
        z_max: 1.0,
    };
    assert_eq!(bounds.origin_x, 1.0);
    assert_eq!(bounds.origin_y, 2.0);
}
