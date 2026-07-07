//! Mock native binary(供 bridge crate 单元 / 集成测试使用)
//!
//! 行为:
//! - 将全部 argv 逐行打印到 stdout(每行 `--idx value`,便于测试断言)
//! - 读取环境变量:
//!   - `MOCK_NATIVE_SLEEP_MS`:进程启动后 sleep 的毫秒数(默认 0)
//!   - `MOCK_NATIVE_EXIT_CODE`:退出码(默认 0)
//!   - `MOCK_NATIVE_STDERR`:额外输出到 stderr 的内容
//!   - `MOCK_NATIVE_IGNORE_SIGTERM`:`"1"` 时忽略 SIGTERM(只响应 SIGKILL),
//!     用于验证 graceful stop() 的 timeout 路径
//! - 如果 `argv` 中出现 `--should-fail`,打印到 stderr 并以非零退出

use std::env;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::Duration;

/// 在 `MOCK_NATIVE_IGNORE_SIGTERM=1` 时让进程忽略 SIGTERM,只对 SIGKILL 响应。
/// 用 unsafe `libc::signal` 直接设置 handler,避免引入额外 crate。
fn maybe_ignore_sigterm() {
    if env::var("MOCK_NATIVE_IGNORE_SIGTERM").as_deref() == Ok("1") {
        // SAFETY: SIG_IGN 永远安全;不需要保存旧 handler。
        unsafe {
            libc::signal(libc::SIGTERM, libc::SIG_IGN);
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    // 跳过 argv[0](binary 名),逐行打印
    for (i, a) in args.iter().enumerate().skip(1) {
        println!("argv[{i}]={a}");
    }
    // 标记
    println!("mock-native-ok");
    // 先设置信号处理(必须在 sleep 之前注册,否则竞态)
    maybe_ignore_sigterm();
    // sleep
    let sleep_ms: u64 = env::var("MOCK_NATIVE_SLEEP_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    if sleep_ms > 0 {
        sleep(Duration::from_millis(sleep_ms));
    }
    // stderr
    if let Ok(extra) = env::var("MOCK_NATIVE_STDERR") {
        if !extra.is_empty() {
            eprintln!("{extra}");
        }
    }
    // 失败标记
    let should_fail = args.iter().any(|a| a == "--should-fail");
    let code: i32 = if should_fail {
        2
    } else {
        env::var("MOCK_NATIVE_EXIT_CODE")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0)
    };
    // 让 stdout 真正 flush
    use std::io::Write;
    let _ = std::io::stdout().flush();
    let _ = std::io::stderr().flush();
    if code != 0 {
        // 打印到 stderr 方便 diagnostics 检查
        eprintln!("mock-native-exit-code={code}");
    }
    // tokio 需要 child stdio flush;给 5ms
    sleep(Duration::from_millis(5));
    ExitCode::from(code as u8)
}
