//! 原始小端 f32 XYZ（每点12字节）输入，输出可选的稀疏几何用于独立数值比较。
use nuwax_pct_tomography::{build, Tomogram, TomographyConfig};
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::time::Instant;

fn write_geometry(path: &str, map: &Tomogram) -> Result<(), Box<dyn std::error::Error>> {
    let mut output = BufWriter::new(File::create(path)?);
    output.write_all(b"PCTG4V01")?;
    output.write_all(&(map.cells.len() as u64).to_le_bytes())?;
    output.write_all(&(map.bands.len() as u64).to_le_bytes())?;
    for cell in &map.cells {
        output.write_all(&cell.x.to_le_bytes())?;
        output.write_all(&cell.y.to_le_bytes())?;
        output.write_all(&cell.ground.to_le_bytes())?;
        output.write_all(&cell.ceiling.to_le_bytes())?;
    }
    for band in &map.bands {
        output.write_all(&band.x.to_le_bytes())?;
        output.write_all(&band.y.to_le_bytes())?;
        output.write_all(&band.low.to_le_bytes())?;
        output.write_all(&band.high.to_le_bytes())?;
    }
    output.flush()?;
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() < 9 || args.len() > 10 {
        return Err("用法：tomography_benchmark 点云.bin 格边长 首层高度 切片间隔 采样精度 线程数 重复次数 预热次数 [几何输出.bin]".into());
    }
    let cfg = TomographyConfig {
        resolution: args[2].parse()?,
        slice_h0: args[3].parse()?,
        slice_dh: args[4].parse()?,
        point_resolution: args[5].parse()?,
    };
    let threads: usize = args[6].parse()?;
    let repeats: usize = args[7].parse()?;
    // 第8位置保存预热次数，确保性能记录显式说明预热策略。
    let warmups: usize = args[8].parse()?;
    if threads == 0 || repeats == 0 {
        return Err("线程数与重复次数必须为正值".into());
    }
    let mut bytes = Vec::new();
    File::open(&args[1])?.read_to_end(&mut bytes)?;
    if bytes.len() % 12 != 0 {
        return Err("点云必须由完整的12字节小端 f32 XYZ 记录组成".into());
    }
    let points: Vec<[f32; 3]> = bytes
        .chunks_exact(12)
        .map(|record| {
            std::array::from_fn(|axis| {
                f32::from_le_bytes(record[axis * 4..axis * 4 + 4].try_into().unwrap())
            })
        })
        .collect();
    drop(bytes);
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()?;
    let mut elapsed = Vec::new();
    let mut last = None;
    for run in 0..warmups + repeats {
        let start = Instant::now();
        let map = pool.install(|| build(&points, &cfg))?;
        if run >= warmups {
            elapsed.push(start.elapsed().as_secs_f64() * 1000.0);
        }
        last = Some(map);
    }
    elapsed.sort_by(f64::total_cmp);
    let map = last.unwrap();
    if let Some(path) = args.get(9) {
        write_geometry(path, &map)?;
    }
    println!(
        "{{\"points\":{},\"threads\":{},\"warmups\":{},\"repeats\":{},\"cells\":{},\"bands\":{},\"min_ms\":{:.6},\"median_ms\":{:.6},\"max_ms\":{:.6},\"samples_ms\":{:?}}}",
        points.len(), threads, warmups, repeats, map.cells.len(), map.bands.len(),
        elapsed[0], elapsed[elapsed.len() / 2], elapsed[elapsed.len() - 1], elapsed
    );
    Ok(())
}
