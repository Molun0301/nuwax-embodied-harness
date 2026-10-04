//! 生成或检查确定的精细资产样本，供跨语言对照。
use nuwax_map_chunks::*;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let path = if args.first().is_some_and(|v| v == "--create") {
        let path = args.get(1).ok_or("缺少输出路径")?;
        let repository = Repository::temporary()?;
        let mut writer = Writer::new(repository.clone(), 0.025, 4 * 1024 * 1024)?;
        writer.observe([[-1, -1, -1], [0, 0, 6], [4000, 4000, 6]], 1_000_008)?;
        let root = writer.snapshot(Domain { session: "mapping-session-a".into(), generation: 4,
            frame_generation: 2, producer_generation: 19 }, 70, 1_000_008)?;
        let (mut source, _) = archive_file(&root, &repository)?;
        let mut output = std::fs::File::create(path)?;
        std::io::copy(&mut source, &mut output)?;
        output.sync_all()?;
        path
    } else { args.first().ok_or("缺少精细资产路径")? };
    let mut reader = ArchiveReader::open(std::fs::File::open(path)?)?;
    let root = reader.root().clone();
    let near = reader.region([-10; 3], [10; 3], 100)?;
    println!("{}", serde_json::json!({"voxel_size_m": root.voxel_size_m(), "points": root.points(),
        "near": near.iter().map(|(key, e)| serde_json::json!({"key":key,"observed_ns":e.observed_ns,"hits":e.hits})).collect::<Vec<_>>() }));
    Ok(())
}
