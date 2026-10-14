//! 按规范等角扫描记录运行真实二维核心；该工具不连接机器人或运动执行器。
use nuwax_slam_toolbox::{Config, Pose, Slam};
use serde::Deserialize;
use serde_json::json;
use std::{
    fs::{self, File},
    io::{BufRead, BufReader, Read, Write},
    path::PathBuf,
};
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Frame {
    timestamp_ns: u64,
    sequence: u64,
    ranges: Vec<Option<f64>>,
    odometry: Option<[f64; 3]>,
}
fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() != 3 {
        return Err("用法：nuwax-slam-2d-replay 配置.json 扫描.jsonl 新输出目录".into());
    }
    let config: Config = serde_json::from_reader(File::open(&args[0])?)?;
    let mut slam = Slam::new(config)?;
    let output = PathBuf::from(&args[2]);
    fs::create_dir(&output)?;
    let mut trajectory = File::create(output.join("轨迹.jsonl"))?;
    let mut count = 0usize;
    let mut accepted = 0usize;
    let mut input = BufReader::new(File::open(&args[1])?);
    loop {
        let mut line = String::new();
        let read_bytes = (&mut input)
            .take(2 * 1024 * 1024 + 1)
            .read_line(&mut line)?;
        if read_bytes == 0 {
            break;
        }
        if line.trim().is_empty() {
            continue;
        }
        if line.len() > 2 * 1024 * 1024 {
            return Err("单帧记录超过读取预算".into());
        }
        let frame: Frame = serde_json::from_str(&line)?;
        let ranges: Vec<_> = frame
            .ranges
            .into_iter()
            .map(|v| v.unwrap_or(f64::NAN))
            .collect();
        let odom = frame.odometry.map(|p| Pose {
            x: p[0],
            y: p[1],
            yaw: p[2],
        });
        let started = std::time::Instant::now();
        let result = slam.process(frame.timestamp_ns, frame.sequence, &ranges, odom)?;
        if result.accepted() {
            accepted += 1;
        }
        let p = result.output.map_from_base;
        let o = result.output.odom_from_base;
        writeln!(
            trajectory,
            "{}",
            json!({"sequence":frame.sequence,"timestamp_ns":frame.timestamp_ns,
            "disposition":format!("{:?}",result.disposition),"pose":[p.x,p.y,p.yaw],"odometry":[o.x,o.y,o.yaw],
            "graph_revision":result.output.graph_revision,"nodes":result.output.node_count,"edges":result.output.edge_count,
            "optimizations":result.output.optimizations,"matching_response":result.output.matching_response,
            "local_response":result.output.local_response,"elapsed_us":started.elapsed().as_micros()})
        )?;
        count += 1;
    }
    if accepted == 0 {
        return Err("没有真实接纳的扫描".into());
    }
    let grid = slam.occupancy_grid()?;
    let mut pgm = File::create(output.join("地图.pgm"))?;
    write!(
        pgm,
        "P5\n{} {}\n255\n",
        grid.metadata.width, grid.metadata.height
    )?;
    for row in grid.cells.chunks(grid.metadata.width as usize).rev() {
        let pixels: Vec<u8> = row
            .iter()
            .map(|v| match *v {
                100 => 0,
                0 => 254,
                _ => 205,
            })
            .collect();
        pgm.write_all(&pixels)?;
    }
    let m = grid.metadata;
    fs::write(output.join("地图.yaml"),format!("image: 地图.pgm\nresolution: {}\norigin: [{}, {}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",m.resolution,m.origin_x,m.origin_y))?;
    fs::write(
        output.join("算法配置.json"),
        serde_json::to_vec_pretty(slam.config())?,
    )?;
    slam.save(&output.join("图数据"))?;
    fs::write(
        output.join("回放结果.json"),
        serde_json::to_vec_pretty(&json!({"input_frames":count,"accepted_frames":accepted,
        "graph_revision":m.graph_revision,"width":m.width,"height":m.height,"resolution":m.resolution,
        "upstream_commit":"6953058b8994c7e750277fd8f39b984453abf63c","validation":"离线输入，非实机验收"}))?,
    )?;
    eprintln!(
        "回放完成：输入{count}帧，接纳{accepted}帧，输出{}",
        output.display()
    );
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("二维回放失败：{error}");
        std::process::exit(1);
    }
}
