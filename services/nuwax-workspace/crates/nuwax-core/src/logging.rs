//! 各独立进程共用的结构日志：组件文件、容量轮转及标准错误双写。

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use tracing_subscriber::fmt::MakeWriter;

struct ComponentFile {
    path: PathBuf,
    lock: File,
    max_bytes: u64,
    backups: usize,
}

impl ComponentFile {
    fn write_record(&mut self, bytes: &[u8]) -> io::Result<()> {
        // 命令行任务可能并行运行，同组件进程必须在同一写入/轮转临界区内。
        #[cfg(unix)]
        let _lock =
            nix::fcntl::Flock::lock(self.lock.try_clone()?, nix::fcntl::FlockArg::LockExclusive)
                .map_err(|(_, error)| io::Error::from(error))?;
        let result = (|| {
            let length = fs::metadata(&self.path).map(|meta| meta.len()).unwrap_or(0);
            if self.max_bytes > 0
                && self.backups > 0
                && length > 0
                && length + bytes.len() as u64 > self.max_bytes
            {
                for index in (1..=self.backups).rev() {
                    let source = if index == 1 {
                        self.path.clone()
                    } else {
                        rotated(&self.path, index - 1)
                    };
                    let destination = rotated(&self.path, index);
                    match fs::rename(source, destination) {
                        Ok(()) => {}
                        Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                        Err(error) => return Err(error),
                    }
                }
            }
            // 每次打开当前文件，避免另一个进程轮转后继续写到已淘汰的句柄。
            OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.path)?
                .write_all(bytes)
        })();
        result
    }
}

fn rotated(path: &Path, index: usize) -> PathBuf {
    PathBuf::from(format!("{}.{}", path.display(), index))
}

#[derive(Clone)]
struct ComponentWriter(Arc<Mutex<Option<ComponentFile>>>);

struct RecordWriter {
    output: ComponentWriter,
    bytes: Vec<u8>,
}

impl Write for RecordWriter {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        self.bytes.extend_from_slice(bytes);
        Ok(bytes.len())
    }
    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Drop for RecordWriter {
    fn drop(&mut self) {
        let _ = io::stderr().lock().write_all(&self.bytes);
        if let Ok(mut output) = self.output.0.lock() {
            if let Some(file) = output.as_mut() {
                if let Err(error) = file.write_record(&self.bytes) {
                    eprintln!("组件日志文件写入失败，后续日志仍保留在标准错误：{error}");
                    *output = None;
                }
            }
        }
    }
}

impl<'a> MakeWriter<'a> for ComponentWriter {
    type Writer = RecordWriter;
    fn make_writer(&'a self) -> Self::Writer {
        RecordWriter {
            output: self.clone(),
            bytes: Vec::with_capacity(512),
        }
    }
}

/// 初始化独立组件日志，复用平台已有日志目录、容量、轮转数和等级设置。
///
/// 文件不可写时保留标准错误，不以日志路径问题阻止机器人功能运行。
pub fn init_component_logging(component: &str, default_filter: &str) {
    let config = nuwax_config::logging().unwrap_or_else(|error| {
        eprintln!("{error}");
        std::process::exit(2)
    });
    let mut directory = nuwax_config::resolve_path(&config.dir);
    let mut path = directory.join(format!("{component}.log"));
    let open_file = |directory: &Path, path: &Path| -> io::Result<ComponentFile> {
        fs::create_dir_all(&directory)?;
        OpenOptions::new().create(true).append(true).open(path)?;
        let lock = OpenOptions::new()
            .create(true)
            .append(true)
            .open(format!("{}.lock", path.display()))?;
        Ok(ComponentFile {
            path: path.to_path_buf(),
            lock,
            max_bytes: config.max_bytes as u64,
            backups: config.backup_count as usize,
        })
    };
    let mut file = open_file(&directory, &path).ok();
    if file.is_none() {
        let mut alternatives = Vec::new();
        if let Ok(system_path) = std::env::var("LOGS_DIRECTORY") {
            if let Some(first) = system_path
                .split(':')
                .next()
                .filter(|value| !value.is_empty())
            {
                alternatives.push(PathBuf::from(first));
            }
        }
        alternatives.push(PathBuf::from("runtime_data/logs"));
        for candidate in alternatives {
            let candidate_file = candidate.join(format!("{component}.log"));
            if let Ok(opened) = open_file(&candidate, &candidate_file) {
                eprintln!(
                    "日志目录不可写，组件日志已改用 {}",
                    candidate_file.display()
                );
                directory = candidate;
                path = candidate_file;
                file = Some(opened);
                break;
            }
        }
        if file.is_none() {
            eprintln!(
                "无法创建组件日志文件，继续使用标准错误：{}",
                directory.display()
            );
        }
    }
    let filter = std::env::var("RUST_LOG")
        .ok()
        .filter(|value| !value.trim().is_empty())
        .unwrap_or_else(|| match config.level.to_ascii_lowercase().as_str() {
            "warning" => "warn".into(),
            "critical" => "error".into(),
            other => other.into(),
        });
    let filter = tracing_subscriber::EnvFilter::try_new(filter)
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(default_filter));
    let writer = ComponentWriter(Arc::new(Mutex::new(file)));
    if let Err(error) = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .json()
        .with_ansi(false)
        .with_target(true)
        .with_writer(writer)
        .try_init()
    {
        eprintln!("组件日志初始化失败：{error}");
        return;
    }
    tracing::info!(event = "process.logging.ready", component, log_file = %path.display(), "组件日志已初始化");
}

/// 将独立进程退出原因记入组件日志，供没有直接依赖tracing的入口复用。
pub fn process_error(component: &str, error: impl std::fmt::Display) {
    tracing::error!(event = "process.failed", component, error = %error, "组件执行失败");
}
