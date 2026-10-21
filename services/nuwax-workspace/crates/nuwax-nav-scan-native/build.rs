fn main() {
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-env-changed=EIGEN3_INCLUDE_DIR");
    let eigen =
        std::env::var("EIGEN3_INCLUDE_DIR").unwrap_or_else(|_| "/usr/include/eigen3".into());
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++17")
        .opt_level(2)
        .warnings(false)
        .include(eigen)
        .include("native/include");
    for module in ["path_searching", "bspline_opt", "traj_utils", "plan_manage"] {
        build.include(format!("native/upstream/{module}/include"));
    }
    for file in [
        "native/src/scan_native.cpp",
        "native/src/planner_manager.cpp",
        "native/upstream/path_searching/src/dyn_a_star.cpp",
        "native/upstream/bspline_opt/src/uniform_bspline.cpp",
        "native/upstream/bspline_opt/src/bspline_optimizer.cpp",
        "native/upstream/traj_utils/src/polynomial_traj.cpp",
    ] {
        build.file(file);
    }
    build.compile("nuwax_scan_native");
}
