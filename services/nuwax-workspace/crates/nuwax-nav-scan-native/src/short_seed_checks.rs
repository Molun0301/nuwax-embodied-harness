// 从真实规划调用读取优化前初值，再调用原动力学检查。


#[test]
fn stationary_short_seed_passes_original_checks_before_optimization() {
    use std::{path::PathBuf, process::Command};
    struct Temporary(PathBuf);
    impl Drop for Temporary {
        fn drop(&mut self) { std::fs::remove_dir_all(&self.0).unwrap(); }
    }
    let temp = Temporary(std::env::temp_dir().join(format!("nuwax-short-seed-checks-{}", std::process::id())));
    std::fs::create_dir(&temp.0).unwrap();
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let binary = temp.0.join("native-regressions");
    let mut compiler = Command::new(std::env::var("CXX").unwrap_or_else(|_| "c++".into()));
    compiler.args(["-std=c++17", "-O2", "-g"]);
    compiler.arg(format!("-I{}", std::env::var("EIGEN3_INCLUDE_DIR").unwrap_or_else(|_| "/usr/include/eigen3".into())));
    for include in ["native/author_a/include", "native/include", "native/upstream/path_searching/include",
        "native/upstream/bspline_opt/include", "native/upstream/traj_utils/include"] {
        compiler.arg(format!("-I{}", root.join(include).display()));
    }
    let output = compiler.arg(root.join("src/short_seed_checks.cpp"))
        .arg(PathBuf::from(env!("OUT_DIR")).join("libnuwax_scan_native.a"))
        .arg("-pthread").arg("-o").arg(&binary).output().unwrap();
    assert!(output.status.success(), "原生回归编译失败：{}", String::from_utf8_lossy(&output.stderr));
    let trace = temp.0.join("短程初值.jsonl");
    let output = Command::new(&binary).env("NEH_SCAN_A_TRACE", &trace).output().unwrap();
    assert!(output.status.success(), "原生回归失败：{}\n{}",
        String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr));
    // 从真实规划调用的诊断中取得优化前控制点，再交回原生原检查；不只验证优化后的输出。
    let mut short_distance = None;
    let mut checked = 0;
    for line in std::fs::read_to_string(trace).unwrap().lines() {
        let event: serde_json::Value = serde_json::from_str(line).unwrap();
        if event["event"] == "rebound_input" {
            let start = &event["start_p"][0];
            let target = &event["target_p"][0];
            let distance = target[0].as_f64().unwrap() - start[0].as_f64().unwrap();
            short_distance = (start[0] == 0. && start[1] == 0. && start[2] == 0.5
                && (distance == 0.03 || distance == 0.4)).then_some(distance);
        }
        if event["event"] != "initial_control_points" { continue; }
        let Some(distance) = short_distance else { continue; };
        let points = event["points"].as_array().unwrap();
        let dt = event["ts"].as_f64().unwrap();
        let before = (5. * distance / 0.5).max((20. * distance / 0.5).sqrt());
        let after = (points.len() - 3) as f64 * dt;
        assert!(after < before * 0.75, "短程初值应缩短：{distance}米，{before}秒→{after}秒");
        let input = temp.0.join("短程原检查.txt");
        let mut text = format!("{} {dt}\n", points.len());
        for point in points {
            text.push_str(&format!("{} {} {}\n", point[0], point[1], point[2]));
        }
        std::fs::write(&input, text).unwrap();
        let validation = Command::new(&binary).arg(input).output().unwrap();
        assert!(validation.status.success(), "优化前初值未通过原检查：{}",
            String::from_utf8_lossy(&validation.stderr));
        eprintln!("原生短程初值：{distance}米，{before:.9}秒→{after:.9}秒，原采样/逐轴/三维控制点检查通过");
        checked += 1;
    }
    assert_eq!(checked, 2, "两种短程都须取得真实优化前初值");
}
