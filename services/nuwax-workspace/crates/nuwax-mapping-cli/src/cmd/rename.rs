//! `rename` 子命令
//!
//! 真做行为：把 bag 目录中的 .mcap / .db3 / .bag 文件按 prefix 重命名。
//! dry-run 模式只打印不执行；正式模式真改文件名。

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct RenameArgs {
    /// 输入 bag 路径（文件或目录）
    #[arg(long)]
    pub bag: PathBuf,
    /// 新名称前缀
    #[arg(long)]
    pub prefix: String,
    /// dry-run（只打印不改名）
    #[arg(long)]
    pub dry_run: bool,
}

pub async fn run(args: RenameArgs) -> anyhow::Result<()> {
    if !args.bag.exists() {
        anyhow::bail!("bag path does not exist: {}", args.bag.display());
    }

    let files = if args.bag.is_file() {
        vec![args.bag.clone()]
    } else {
        collect_bag_files(&args.bag)?
    };

    let mut renamed = 0usize;
    for (i, f) in files.iter().enumerate() {
        // 新名字 = {prefix}_{idx:04}.{ext}
        let ext = f
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("mcap");
        let new_name = format!("{}_{:04}.{}", args.prefix, i, ext);
        let new_path = f.with_file_name(new_name);

        if args.dry_run {
            println!("[rename] (dry-run) {} → {}", f.display(), new_path.display());
        } else {
            std::fs::rename(f, &new_path)?;
            println!("[rename] {} → {}", f.display(), new_path.display());
            renamed += 1;
        }
    }

    if !args.dry_run {
        tracing::info!("rename: {} files renamed under {}", renamed, args.bag.display());
    }
    Ok(())
}

fn collect_bag_files(root: &PathBuf) -> anyhow::Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    visit(root, &mut out)?;
    // 按文件名排序以保证确定性
    out.sort();
    Ok(out)
}

fn visit(p: &PathBuf, out: &mut Vec<PathBuf>) -> anyhow::Result<()> {
    if p.is_file() {
        if let Some(ext) = p.extension().and_then(|e| e.to_str()) {
            if matches!(ext, "mcap" | "db3" | "bag") {
                out.push(p.clone());
            }
        }
    } else if p.is_dir() {
        for entry in std::fs::read_dir(p)? {
            visit(&entry?.path(), out)?;
        }
    }
    Ok(())
}