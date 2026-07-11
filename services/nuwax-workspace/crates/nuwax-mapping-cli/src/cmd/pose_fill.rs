//! `pose-fill` 子命令
//!
//! 真做行为：读 trajectory CSV (stamp, x, y, z, qx, qy, qz, qw)，按目标采样率插值，
//! 输出新 CSV。每两行间用线性插值（位置）+ SLERP（旋转四元数）生成中间帧。
//!
//! CSV 格式：`stamp_sec,x,y,z,qx,qy,qz,qw` （一行一条轨迹）

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct PoseFillArgs {
    /// 输入轨迹 CSV 路径
    #[arg(long)]
    pub input: PathBuf,
    /// 输出路径
    #[arg(long)]
    pub output: PathBuf,
    /// 采样率（Hz）
    #[arg(long, default_value_t = 100.0)]
    pub rate: f64,
}

pub async fn run(args: PoseFillArgs) -> anyhow::Result<()> {
    if args.rate <= 0.0 {
        anyhow::bail!("rate must be positive, got {}", args.rate);
    }

    // 阶段 1：读输入 CSV
    let content = std::fs::read_to_string(&args.input)
        .map_err(|e| anyhow::anyhow!("read input {} failed: {}", args.input.display(), e))?;
    let poses = parse_trajectory_csv(&content)?;
    if poses.is_empty() {
        anyhow::bail!("input CSV has no valid rows");
    }

    // 阶段 2：按 target rate 插值
    let dt = 1.0 / args.rate;
    let mut out_lines = Vec::new();
    out_lines.push("stamp_sec,x,y,z,qx,qy,qz,qw".to_string());
    let mut out_count = 0usize;

    for window in poses.windows(2) {
        let (a, b) = (&window[0], &window[1]);
        if b.stamp <= a.stamp {
            continue;
        }
        let span = b.stamp - a.stamp;
        let n_steps = ((span / dt).ceil() as usize).max(1);

        for i in 0..n_steps {
            let t_frac = (i as f64 * dt) / span;
            let t_frac = t_frac.clamp(0.0, 1.0);
            let interp = slerp_pose(a, b, t_frac);
            let line = format!(
                "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                a.stamp + i as f64 * dt, interp.x, interp.y, interp.z, interp.qx, interp.qy, interp.qz, interp.qw
            );
            out_lines.push(line);
            out_count += 1;
        }
    }
    // 加上最后一个 pose
    if let Some(last) = poses.last() {
        out_lines.push(format!(
            "{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
            last.stamp, last.x, last.y, last.z, last.qx, last.qy, last.qz, last.qw
        ));
        out_count += 1;
    }

    // 阶段 3：写输出
    std::fs::write(&args.output, out_lines.join("\n") + "\n")?;
    println!(
        "[pose-fill] input={} ({} poses) rate={}Hz → output={} ({} poses)",
        args.input.display(),
        poses.len(),
        args.rate,
        args.output.display(),
        out_count
    );
    tracing::info!("pose-fill done");
    Ok(())
}

#[derive(Debug, Clone)]
struct Pose {
    stamp: f64,
    x: f64,
    y: f64,
    z: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    qw: f64,
}

fn parse_trajectory_csv(content: &str) -> anyhow::Result<Vec<Pose>> {
    let mut out = Vec::new();
    for (i, line) in content.lines().enumerate() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        // 跳过表头
        if i == 0 && trimmed.starts_with("stamp") {
            continue;
        }
        let parts: Vec<&str> = trimmed.split(',').map(|s| s.trim()).collect();
        if parts.len() < 8 {
            continue;
        }
        let parse_f = |s: &str| -> anyhow::Result<f64> {
            s.parse::<f64>()
                .map_err(|e| anyhow::anyhow!("parse '{s}' failed: {e}"))
        };
        let pose = Pose {
            stamp: parse_f(parts[0])?,
            x: parse_f(parts[1])?,
            y: parse_f(parts[2])?,
            z: parse_f(parts[3])?,
            qx: parse_f(parts[4])?,
            qy: parse_f(parts[5])?,
            qz: parse_f(parts[6])?,
            qw: parse_f(parts[7])?,
        };
        out.push(pose);
    }
    Ok(out)
}

/// SLERP 插值（位置 lerp + 四元数 slerp）
fn slerp_pose(a: &Pose, b: &Pose, t: f64) -> Pose {
    // 位置线性插值
    let x = a.x + (b.x - a.x) * t;
    let y = a.y + (b.y - a.y) * t;
    let z = a.z + (b.z - a.z) * t;

    // 四元数 SLERP（简化：nlerp，对小角度足够；本期不上完整 SLERP）
    let mut qx = a.qx + (b.qx - a.qx) * t;
    let mut qy = a.qy + (b.qy - a.qy) * t;
    let mut qz = a.qz + (b.qz - a.qz) * t;
    let mut qw = a.qw + (b.qw - a.qw) * t;
    let norm = (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
    if norm > 1e-9 {
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;
    }

    Pose {
        stamp: 0.0, // caller 重写
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
    }
}