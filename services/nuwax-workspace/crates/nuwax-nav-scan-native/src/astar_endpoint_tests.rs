#[test]
fn astar_endpoint_adjustment_returns_failure_after_leaving_allowed_region() {
    use std::{path::PathBuf, process::Command};
    struct Temporary(PathBuf);
    impl Drop for Temporary {
        fn drop(&mut self) { std::fs::remove_dir_all(&self.0).unwrap(); }
    }
    let temp = Temporary(std::env::temp_dir().join(format!("nuwax-astar-endpoint-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let binary = temp.0.join("astar-endpoint");
    let mut compiler = Command::new(std::env::var("CXX").unwrap_or_else(|_| "c++".into()));
    compiler.args(["-std=c++17", "-O2"]);
    compiler.arg(format!("-I{}", std::env::var("EIGEN3_INCLUDE_DIR").unwrap_or_else(|_| "/usr/include/eigen3".into())));
    for include in ["native/author_a/include", "native/include", "native/upstream/path_searching/include"] {
        compiler.arg(format!("-I{}", root.join(include).display()));
    }
    let output = compiler.arg(root.join("src/astar_endpoint_fixture.cpp"))
        .arg(PathBuf::from(env!("OUT_DIR")).join("libnuwax_scan_native.a"))
        .arg("-pthread").arg("-o").arg(&binary).output().unwrap();
    assert!(output.status.success(), "端点夹具编译失败：{}", String::from_utf8_lossy(&output.stderr));
    // 外层进程期限能让旧无界循环可靠失败，避免挂住整个测试进程。
    let output = Command::new("timeout").arg("5s").arg(&binary).output().unwrap();
    assert!(output.status.success(), "端点夹具失败或超过5秒：{}\n{}",
        String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr));
    eprintln!("{}", String::from_utf8_lossy(&output.stdout));
}
