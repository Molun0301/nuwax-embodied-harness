//! MAP-500 基线：直接测量三种公开占据栅格 kernel。
//!
//! 运行：
//! `cargo bench --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-mapping --bench occupancy_algorithms`

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nalgebra::Point3;
use nuwax_mapping::pointclouds::occupancy::{
    PointCloudOccAlgo, PointCloudOccConfig, PointCloudOccupancyBuilder,
};

const GRID_SIZE: u32 = 500;
const RESOLUTION: f64 = 0.05;

fn fixture_points() -> Vec<Point3<f64>> {
    // 固定、有限的稀疏地形输入，保证每轮 benchmark 使用同一份点集。
    let mut points = Vec::with_capacity(20_000);
    for index in 0..20_000_u32 {
        let x = (index % 400) as f64 * RESOLUTION - 10.0;
        let y = ((index / 400) % 400) as f64 * RESOLUTION - 10.0;
        let z = ((index.wrapping_mul(17) % 41) as f64) * 0.01;
        points.push(Point3::new(x, y, z));
    }
    points
}

fn bench_map_500(c: &mut Criterion) {
    let points = fixture_points();
    let builder = PointCloudOccupancyBuilder::new(
        Point3::new(0.0, 0.0, 0.0),
        RESOLUTION,
        GRID_SIZE,
        GRID_SIZE,
        (-12.5, -12.5),
    );

    for (name, algo) in [
        ("general", PointCloudOccAlgo::General),
        ("simple", PointCloudOccAlgo::Simple),
        ("height_cost", PointCloudOccAlgo::HeightCost),
    ] {
        let config = PointCloudOccConfig {
            algo,
            ..PointCloudOccConfig::default()
        };
        c.bench_function(&format!("map_500x500_{name}"), |bencher| {
            bencher.iter(|| {
                let grid = builder.build_with_config(black_box(&points), black_box(&config));
                black_box((grid.width, grid.height, grid.data.len()));
                grid
            });
        });
    }
}

criterion_group!(benches, bench_map_500);
criterion_main!(benches);
