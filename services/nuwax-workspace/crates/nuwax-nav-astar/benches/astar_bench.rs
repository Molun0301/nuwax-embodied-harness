//! A* benchmark
//!
//! 运行：
//! `cargo bench --features bench -p nuwax-nav-astar`
//!
//! 度量 64x64 / 128x128 / 256x256 costmap 的 min_cost_astar 规划时间。
//! 输出 p50 / p95 与 Python baseline 的对比（baseline 数据记录在 `feadback.md` 中）。

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nuwax_nav_astar::astar::{min_cost_astar, AStarOptions};
use nuwax_nav_astar::helpers::Point2;
use nuwax_nav_astar::navigation_map::{CostMap2D, OCCUPIED};

fn build_random_costmap(size: usize, occupancy_ratio: f64) -> CostMap2D {
    // 使用固定 seed 保证可重复
    let mut cm = CostMap2D::new_all_free("map", size, size, 1.0);
    let mut state: u64 = 0xdeadbeefcafebabe;
    for y in 0..size {
        for x in 0..size {
            // 简单 LCG 产生伪随机
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            let r = ((state >> 33) as f64) / (u32::MAX as f64);
            if r < occupancy_ratio {
                cm.set(x, y, OCCUPIED);
            }
        }
    }
    cm
}

fn bench_astar(c: &mut Criterion) {
    for (size, occupancy) in [(64usize, 0.20f64), (128, 0.20), (256, 0.20)] {
        let cm = build_random_costmap(size, occupancy);
        let opts = AStarOptions::default();
        let start = Point2::new(0.5, 0.5);
        let goal = Point2::new((size as f64) - 0.5, (size as f64) - 0.5);
        c.bench_function(&format!("astar_{size}x{size}_occ{occupancy:0.2}"), |b| {
            b.iter(|| {
                let r = min_cost_astar(
                    black_box(&cm),
                    black_box(start),
                    black_box(goal),
                    black_box(&opts),
                );
                black_box(r)
            })
        });
    }
}

criterion_group!(benches, bench_astar);
criterion_main!(benches);
