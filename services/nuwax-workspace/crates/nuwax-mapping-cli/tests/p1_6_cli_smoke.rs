//! P1 #6 — mapping-cli 6 个子命令实装 smoke 测试
//!
//! 每个 cmd 都用最小 fixture 调一遍 happy path：
//! - map:          写一个 PGM 文件，验证 costmap 维度非零
//! - pose_fill:    读 fixture CSV → 输出 CSV，验证行数 >= 输入
//! - rename:       准备 fake.mcap 临时目录 → dry-run 模式不真改文件
//! - replay:       fixture PointCloud2 → PGM，验证文件存在
//! - replay_marker: fixture PoseGraph → .rrd JSON，验证有 nodes 字段
//! - summary:      fixture 目录 → stdout summary

use nuwax_mapping_cli::cmd::{
    map::{run as map_run, MapArgs},
    pose_fill::{run as pose_fill_run, PoseFillArgs},
    rename::{run as rename_run, RenameArgs},
    replay::{run as replay_run, ReplayArgs},
    replay_marker::{run as replay_marker_run, ReplayMarkerArgs},
    summary::{run as summary_run, SummaryArgs},
};
use std::path::PathBuf;

fn temp_dir(name: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!(
        "nuwax_cli_smoke_{}_{}",
        name,
        std::process::id()
    ));
    let _ = std::fs::remove_dir_all(&p);
    std::fs::create_dir_all(&p).unwrap();
    p
}

#[tokio::test]
async fn smoke_map_writes_pgm() {
    let dir = temp_dir("map");
    let bag = dir.join("dummy.bag");
    std::fs::write(&bag, b"dummy").unwrap();
    let output = dir.join("map.pgm");

    map_run(MapArgs {
        bag,
        output: Some(output.clone()),
        theme: "default".to_string(),
    })
    .await
    .expect("map run");

    let meta = std::fs::metadata(&output).expect("pgm file exists");
    assert!(meta.len() > 0, "pgm should have non-zero size");
}

#[tokio::test]
async fn smoke_pose_fill_writes_csv() {
    let dir = temp_dir("pose_fill");
    let input = dir.join("traj.csv");
    std::fs::write(
        &input,
        "stamp_sec,x,y,z,qx,qy,qz,qw\n0.0,0,0,0,0,0,0,1\n1.0,1,0,0,0,0,0,1\n",
    )
    .unwrap();
    let output = dir.join("traj_filled.csv");

    pose_fill_run(PoseFillArgs {
        input,
        output: output.clone(),
        rate: 10.0,
    })
    .await
    .expect("pose_fill run");

    let content = std::fs::read_to_string(&output).expect("output csv");
    let lines: Vec<&str> = content.lines().collect();
    // 1 header + 2 endpoints + 10 interpolated between 0s and 1s @ 10Hz
    assert!(
        lines.len() >= 12,
        "expected >= 12 lines, got {}",
        lines.len()
    );
    assert!(lines[0].starts_with("stamp_sec"));
}

#[tokio::test]
async fn smoke_rename_dry_run_does_not_mutate() {
    let dir = temp_dir("rename");
    let src = dir.join("data.mcap");
    std::fs::write(&src, b"x").unwrap();

    rename_run(RenameArgs {
        bag: src.clone(),
        prefix: "renamed".to_string(),
        dry_run: true,
    })
    .await
    .expect("rename dry-run");

    assert!(src.exists(), "dry-run must not delete the source file");
}

#[tokio::test]
async fn smoke_replay_writes_pgm() {
    let dir = temp_dir("replay");
    let bag = dir.join("dummy.bag");
    std::fs::write(&bag, b"dummy").unwrap();

    replay_run(ReplayArgs {
        bag: bag.clone(),
        start_offset: 0.0,
        rate: 1.0,
        dry_run: false,
    })
    .await
    .expect("replay run");

    let expected = {
        let mut p = bag.clone();
        p.set_extension("pgm");
        p
    };
    let meta = std::fs::metadata(&expected).expect("replay pgm exists");
    assert!(meta.len() > 0);
}

#[tokio::test]
async fn smoke_replay_marker_writes_rrd_json() {
    let dir = temp_dir("replay_marker");
    let bag = dir.join("dummy.bag");
    std::fs::write(&bag, b"dummy").unwrap();
    let output = dir.join("markers.rrd");

    replay_marker_run(ReplayMarkerArgs {
        bag,
        output: output.clone(),
        family: "tag36h11".to_string(),
    })
    .await
    .expect("replay-marker run");

    let content = std::fs::read_to_string(&output).expect("rrd file");
    let json: serde_json::Value = serde_json::from_str(&content).expect("valid JSON");
    assert!(json.get("num_nodes").is_some());
    assert!(json.get("loops_added").is_some());
}

#[tokio::test]
async fn smoke_summary_scans_dir() {
    let dir = temp_dir("summary");
    // 放几个 fixture 文件
    std::fs::write(dir.join("a.mcap"), b"x".repeat(2048)).unwrap();
    std::fs::write(dir.join("b.db3"), b"y".repeat(1024)).unwrap();

    summary_run(SummaryArgs { bag: dir.clone() })
        .await
        .expect("summary run");
}