//! 以独立暴力距离和精确多边形距离验证实际原生净空场。
#[test]
fn clearance_field_matches_brute_force_and_exact_rectangles() {
    use std::{path::PathBuf, process::Command};
    struct Temporary(PathBuf);
    impl Drop for Temporary {
        fn drop(&mut self) {
            std::fs::remove_dir_all(&self.0).unwrap();
        }
    }
    let temp = Temporary(std::env::temp_dir().join(format!("nuwax-clearance-field-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let binary = temp.0.join("clearance-field");
    let mut compiler = Command::new("c++");
    compiler.args(["-std=c++17", "-O2", "-I/usr/include/eigen3"]);
    for include in ["native/author_a/include", "native/include"] {
        compiler.arg(format!("-I{}", root.join(include).display()));
    }
    let output = compiler
        .arg(root.join("native/author_a/clearance_field_fixture.cpp"))
        .arg(PathBuf::from(env!("OUT_DIR")).join("libnuwax_scan_native.a"))
        .arg("-pthread").arg("-o").arg(&binary).output().unwrap();
    assert!(output.status.success(), "净空场夹具编译失败：{}", String::from_utf8_lossy(&output.stderr));
    let output = Command::new(&binary).output().unwrap();
    assert!(output.status.success(), "净空场几何对照失败：{}\n{}",
        String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr));
    println!("{}", String::from_utf8_lossy(&output.stdout));
}
