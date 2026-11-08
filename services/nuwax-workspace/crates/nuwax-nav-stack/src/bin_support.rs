//! 固定导航任务的命令行入口与进程监督。
use crate::{NavStackError, NavStackResult, ProductionPipeline};

/// 启动器使用的就绪协议标识。
pub const LAUNCHER_CONTRACT: &str = "nuwax-nav-stack-internal-ready-v1";

/// 无副作用的启动器协议查询。
pub fn launcher_contract_requested(args: &[String]) -> bool { args == ["--launcher-contract"] }

/// 命令行选择的固定组合。
#[derive(Debug, Clone)]
pub struct CliOptions {
    /// 实际执行的组合。
    pub pipeline: ProductionPipeline,
}

impl CliOptions {
    /// 解析固定组合选项；不再提供通用图检查或特性预设。
    pub fn parse(args: &[String]) -> NavStackResult<Self> {
        let mut pipeline = None;
        let mut index = 0;
        while index < args.len() {
            let argument = &args[index];
            match argument.as_str() {
                "--help" | "-h" => {
                    println!("nuwax-nav-stack {}\n\n--profile <pct-scan-3d|simple-2d> 选择固定导航组合\n--launcher-contract 查询启动协议\n--version 显示版本", crate::VERSION);
                    std::process::exit(0);
                }
                "--version" | "-V" => { println!("{} {}", crate::CRATE_NAME, crate::VERSION); std::process::exit(0); }
                "--profile" | "-p" => {
                    index += 1;
                    let value = args.get(index).ok_or_else(|| NavStackError::invalid("--profile requires a value"))?;
                    pipeline = Some(ProductionPipeline::from_profile(value)?);
                }
                value if value.starts_with("--profile=") => {
                    pipeline = Some(ProductionPipeline::from_profile(&value[10..])?);
                }
                value => return Err(NavStackError::invalid(format!("unknown argument: {value}"))),
            }
            index += 1;
        }
        Ok(Self { pipeline: match pipeline { Some(value) => value, None => ProductionPipeline::from_runtime()? } })
    }
}

/// 启动固定组合，直到收到关闭信号或任务退出。
#[cfg(all(feature = "mls", feature = "simple-2d"))]
pub fn run(options: CliOptions) -> NavStackResult<()> {
    tokio::runtime::Builder::new_current_thread().enable_all().build()?.block_on(
        run_until_shutdown(options, async { tokio::signal::ctrl_c().await.map_err(Into::into) })
    )
}

/// 以参数提供关闭信号，生产与测试共用实际组装路径。
#[cfg(all(feature = "mls", feature = "simple-2d"))]
pub async fn run_until_shutdown<F>(options: CliOptions, shutdown: F) -> NavStackResult<()>
where F: std::future::Future<Output = NavStackResult<()>> + Send {
    let tasks = crate::runtime_owners::start_tasks(options.pipeline)?;
    supervise_tasks_until(tasks, shutdown, nuwax_config::resolve_path(nuwax_config::NAV_READY)).await
}

#[cfg(all(feature = "mls", feature = "simple-2d"))]
async fn supervise_tasks_until<F>(mut tasks: Vec<crate::task::TaskThread>, shutdown: F, ready_path: std::path::PathBuf) -> NavStackResult<()>
where F: std::future::Future<Output = NavStackResult<()>> + Send {
    let ready_marker = RuntimeReadyMarker::publish(ready_path)?;
    let mut health = tokio::time::interval(std::time::Duration::from_millis(100));
    health.tick().await;
    tokio::pin!(shutdown);
    let supervision = loop {
        tokio::select! {
            result = &mut shutdown => break result,
            _ = health.tick() => {
                let result = tasks.iter_mut().try_for_each(|task| {
                    if task.is_running()? { Ok(()) }
                    else { Err(NavStackError::RuntimeNodeExited { node: task.label().into() }) }
                });
                if let Err(error) = result { break Err(error); }
            }
        }
    };
    let mut cleanup = Ok(());
    for task in &mut tasks {
        let result = task.shutdown();
        if cleanup.is_ok() { cleanup = result; }
    }
    drop(ready_marker);
    supervision.and(cleanup)
}

#[cfg(all(feature = "mls", feature = "simple-2d"))]
struct RuntimeReadyMarker { path: std::path::PathBuf }

#[cfg(all(feature = "mls", feature = "simple-2d"))]
impl RuntimeReadyMarker {
    fn publish(path: std::path::PathBuf) -> NavStackResult<Self> {
        std::fs::create_dir_all(path.parent().unwrap())?;
        let mut temporary = path.as_os_str().to_os_string();
        temporary.push(format!(".tmp.{}", std::process::id()));
        let temporary = std::path::PathBuf::from(temporary);
        std::fs::write(&temporary, format!("pid={}\nready=1\n", std::process::id()))?;
        std::fs::rename(&temporary, &path)?;
        Ok(Self { path })
    }
}

#[cfg(all(feature = "mls", feature = "simple-2d"))]
impl Drop for RuntimeReadyMarker {
    fn drop(&mut self) { let _ = std::fs::remove_file(&self.path); }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn launcher_contract_probe_is_exact_and_side_effect_free() {
        assert!(launcher_contract_requested(&["--launcher-contract".into()]));
        assert!(!launcher_contract_requested(&["--launcher-contract".into(), "--profile=simple-2d".into()]));
        assert_eq!(LAUNCHER_CONTRACT, "nuwax-nav-stack-internal-ready-v1");
    }
    #[test]
    fn parse_default_profile_follows_complete_pipeline() {
        assert_eq!(CliOptions::parse(&[]).unwrap().pipeline, ProductionPipeline::from_runtime().unwrap());
    }
    #[test]
    fn parse_profile_with_equals() {
        assert_eq!(CliOptions::parse(&["--profile=simple-2d".into()]).unwrap().pipeline, ProductionPipeline::Slam2D);
        assert_eq!(CliOptions::parse(&["--profile".into(),"pct-scan-3d".into()]).unwrap().pipeline, ProductionPipeline::Lio3D);
    }
    #[test]
    fn parse_unknown_flag_errors() { assert!(CliOptions::parse(&["--nonsense".into()]).unwrap_err().to_string().contains("unknown argument")); }
    #[test]
    fn parse_missing_value_errors() { assert!(CliOptions::parse(&["--profile".into()]).unwrap_err().to_string().contains("requires a value")); }

    #[tokio::test]
    #[cfg(all(feature = "mls", feature = "simple-2d"))]
    async fn supervision_returns_task_failure_and_removes_readiness() {
        use crate::task::spawn_task;
        let root = std::env::temp_dir().join(format!("nuwax-s9-supervision-{}",std::process::id()));
        let ready_path = root.join("navigation.ready");
        let (release, waiting) = tokio::sync::oneshot::channel();
        let task = spawn_task("supervised", Box::new(|_, ready| Box::pin(async move {
            ready.send(Ok(())).unwrap();
            let _ = waiting.await;
            Err(NavStackError::Internal("已就绪任务的实际错误".into()))
        }))).unwrap();
        let observed = ready_path.clone();
        let error = supervise_tasks_until(vec![task], async move {
            assert!(observed.is_file());
            release.send(()).unwrap();
            std::future::pending().await
        }, ready_path.clone()).await.unwrap_err();
        assert!(error.to_string().contains("已就绪任务的实际错误"));
        assert!(!ready_path.exists());
        std::fs::remove_dir_all(root).unwrap();
    }
}
