use std::{env, path::PathBuf, process::Command};
fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("缺少OUT_DIR"));
    let build = out.join("native");
    let dependency_prefix = env::var_os("NUWAX_SLAM2D_DEPENDENCY_PREFIX")
        .map(PathBuf::from)
        .unwrap_or_default();
    let mut configure = Command::new("cmake");
    configure
        .args(["-S", "native", "-B"])
        .arg(&build)
        .arg("-DCMAKE_BUILD_TYPE=Release")
        .arg(format!(
            "-DNUWAX_SLAM2D_DEPENDENCY_PREFIX={}",
            dependency_prefix.display()
        ));
    if let Some(path) = env::var_os("CMAKE_TOOLCHAIN_FILE") {
        configure.arg(format!(
            "-DCMAKE_TOOLCHAIN_FILE={}",
            PathBuf::from(path).display()
        ));
    } else if env::var("HOST").ok() != env::var("TARGET").ok() {
        panic!("二维算法跨架构构建需要CMAKE_TOOLCHAIN_FILE，禁止链接宿主机库");
    }
    assert!(
        configure.status().expect("启动二维原生配置").success(),
        "二维原生配置失败；安装Ceres 2.2、Eigen、Boost、TBB，或设置CMAKE_PREFIX_PATH"
    );
    assert!(
        Command::new("cmake")
            .arg("--build")
            .arg(&build)
            .args(["--parallel"])
            .arg(env::var("CMAKE_BUILD_PARALLEL_LEVEL").unwrap_or_else(|_| "2".into()))
            .status()
            .expect("启动二维原生编译")
            .success(),
        "二维原生编译失败"
    );
    println!("cargo:rustc-link-search=native={}", build.display());
    println!("cargo:rustc-link-lib=dylib=nuwax_slam_toolbox");
    // 开发构建可直接运行；发布安装器另以相对库目录打包，并携带所有动态依赖。
    println!("cargo:libdir={}", build.display());
    println!("cargo:rerun-if-changed=native");
    for key in [
        "CMAKE_PREFIX_PATH",
        "CMAKE_TOOLCHAIN_FILE",
        "NUWAX_SLAM2D_DEPENDENCY_PREFIX",
    ] {
        println!("cargo:rerun-if-env-changed={key}");
    }
}
