//! 源码树断言不存在独立算法模块(方案 §11.2 `no_algorithm_fork`)

#[test]
fn no_independent_algorithm_modules() {
    // 这些文件名若被错误引入,build 会失败;CI 通过即视为约束达成。
    let forbidden = ["voxel_grid.rs", "crop.rs", "ground.rs", "merge.rs"];
    let src = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    for f in forbidden {
        assert!(
            !src.join(f).exists(),
            "algorithm module `src/{f}` must not exist in nuwax-explore-terrain-map-bridge",
        );
    }
}
