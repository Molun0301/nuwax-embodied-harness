use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

fn main() {
    let output = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR"));
    let build = output.join("native");
    let mut configure = Command::new("cmake");
    configure
        .arg("-S")
        .arg("native")
        .arg("-B")
        .arg(&build)
        .arg("-DCMAKE_BUILD_TYPE=Release");
    if let Some(toolchain) = env::var_os("CMAKE_TOOLCHAIN_FILE") {
        configure.arg(format!(
            "-DCMAKE_TOOLCHAIN_FILE={}",
            PathBuf::from(toolchain).display()
        ));
    } else if env::var("HOST").ok() != env::var("TARGET").ok() {
        panic!("GTSAM跨架构构建需要显式CMAKE_TOOLCHAIN_FILE，不能嵌入宿主机程序");
    }
    assert!(
        configure.status().expect("启动GTSAM配置").success(),
        "GTSAM配置失败"
    );
    assert!(
        Command::new("cmake")
            .arg("--build")
            .arg(&build)
            .args(["--target", "nuwax_gtsam", "--parallel"])
            .arg(env::var("CMAKE_BUILD_PARALLEL_LEVEL").unwrap_or_else(|_| "2".into()))
            .status()
            .expect("启动GTSAM构建")
            .success(),
        "GTSAM构建失败"
    );
    println!("cargo:rustc-link-search=native={}", build.display());
    println!("cargo:rustc-link-lib=static=nuwax_gtsam");
    for library in fs::read_to_string(build.join("gtsam-library.txt"))
        .expect("CMake库清单")
        .lines()
    {
        let path = Path::new(library);
        println!(
            "cargo:rustc-link-search=native={}",
            path.parent().unwrap().display()
        );
        let name = path.file_name().unwrap().to_str().unwrap();
        if let Some(name) = name.strip_prefix("lib").and_then(|n| n.strip_suffix(".a")) {
            println!("cargo:rustc-link-lib=static={name}");
        } else {
            panic!("GTSAM必须静态构建，避免未封装的运行时动态依赖：{library}");
        }
    }
    for lib in [
        "boost_serialization",
        "boost_graph",
        "boost_program_options",
        "boost_random",
        "boost_timer",
        "boost_chrono",
    ] {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }
    for lib in [
        "pcl_common",
        "pcl_kdtree",
        "pcl_search",
        "pcl_registration",
        "pcl_features",
        "pcl_filters",
    ] {
        println!("cargo:rustc-link-lib=dylib={lib}");
    }
    let cpp = if env::var("TARGET").unwrap_or_default().contains("apple") {
        "c++"
    } else {
        "stdc++"
    };
    println!("cargo:rustc-link-lib=dylib={cpp}");
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-env-changed=CMAKE_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_TOOLCHAIN_FILE");
    println!("cargo:rerun-if-env-changed=CMAKE_BUILD_PARALLEL_LEVEL");
}
