//! 集成测试:rework22 P1.9 scoring information_gain 分母统一为 cell count。
//!
//! 覆盖:
//! - `p1_9_information_gain_uses_cell_count`:分子分母都按 cell count 归一化;
//!   增加 cell_count 应当严格提升 information_gain(不受 grid resolution 影响)。
//! - `p1_9_information_gain_clamps_to_one`:极大 cell_count 仍 clamp 在 1.0。

#![cfg(feature = "runtime")]

use nuwax_explore_wavefront::config::WavefrontConfig;
use nuwax_explore_wavefront::test_helpers::{CandidateGeometry, FrontierScorer};
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

fn empty_grid(w: u32, h: u32, resolution: f64) -> OccupancyGrid2D {
    OccupancyGrid2D {
        origin: Point2 { x: 0.0, y: 0.0 },
        resolution,
        width: w,
        height: h,
        cells: vec![OccupancyCell::Unknown; (w * h) as usize],
    }
}

fn cand_at(x: f64, y: f64, cell_count: usize) -> CandidateGeometry {
    let perimeter_m = (cell_count as f64).sqrt() * 4.0 * 0.1;
    let area_m2 = (cell_count as f64) * 0.1 * 0.1;
    CandidateGeometry {
        centroid: Point2 { x, y },
        cell_count,
        perimeter_m,
        area_m2,
    }
}

/// P1.9 主测试:cell_count 翻倍 → information_gain 翻倍;
/// 同时验证分子用 cell_count(不被 area_m2 影响)。
#[test]
fn p1_9_information_gain_uses_cell_count() {
    let grid = empty_grid(20, 20, 0.1);
    let cfg = WavefrontConfig::default();
    let scorer = FrontierScorer::new();
    let robot = Point2 { x: 0.0, y: 0.0 };

    let small = cand_at(1.0, 1.0, 10);
    let large = cand_at(1.0, 1.0, 20);
    // 故意把 area_m2 设大,小 candidate 反设小;若 scorer 用了 area_m2,small
    // 反而分数更高——这是历史 bug,这里确保 scorer 不再走 area_m2。
    let mut small_geom = small;
    small_geom.area_m2 = 1_000_000.0;
    let mut large_geom = large;
    large_geom.area_m2 = 0.0001;

    let b_small = scorer.score(&small_geom, robot, &grid, &[], None, &cfg);
    let b_large = scorer.score(&large_geom, robot, &grid, &[], None, &cfg);

    // large cell_count = 2 * small cell_count → information_gain 翻倍
    let info_small = b_small.information_gain as f64;
    let info_large = b_large.information_gain as f64;
    assert!(
        (info_large - 2.0 * info_small).abs() < 1e-5,
        "cell_count 翻倍应使 information_gain 翻倍: small={}, large={}",
        info_small,
        info_large
    );
    // 防止 scorer 退化到 area_m2 分母(此处 small_area = 1e6 / 50 = 20000,clamp 1.0)
    // 确保两个值都没用 area_m2 作为分母:
    // max_expected_cells = (0.5 / 0.1) * 10 = 50
    // small: 10 / 50 = 0.2
    // large: 20 / 50 = 0.4
    assert!((info_small - 0.2).abs() < 1e-5, "small info_gain={}", info_small);
    assert!((info_large - 0.4).abs() < 1e-5, "large info_gain={}", info_large);
}

/// 极大 cell_count 仍 clamp 到 1.0(分母保护)
#[test]
fn p1_9_information_gain_clamps_to_one() {
    let grid = empty_grid(20, 20, 0.1);
    let cfg = WavefrontConfig::default();
    let scorer = FrontierScorer::new();
    let robot = Point2 { x: 0.0, y: 0.0 };

    let huge = cand_at(1.0, 1.0, 10_000);
    let b = scorer.score(&huge, robot, &grid, &[], None, &cfg);
    assert!(
        (b.information_gain - 1.0).abs() < 1e-6,
        "information_gain 应当 clamp 到 1.0;got {}",
        b.information_gain
    );
}
