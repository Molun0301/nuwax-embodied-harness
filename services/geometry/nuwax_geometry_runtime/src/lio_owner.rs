//! 原生子进程出错只报告会话故障，由根拥有方决定何时重建。
use crate::native_lio_owner::NativeProcessOwner;
use std::time::Duration;
use tokio::sync::watch;

pub(crate) async fn supervise_native(
    mut owner: NativeProcessOwner,
    mut shutdown: watch::Receiver<bool>,
    fault: impl Fn(String),
) {
    tokio::select! {
        _ = shutdown.wait_for(|closed|*closed) => {},
        result = owner.wait_unexpected_exit() => {
            let reason = match result { Ok(status) => format!("原生LIO子进程退出：{status}；{}", String::from_utf8_lossy(&owner.stderr_tail())), Err(error) => error };
            fault(reason);
        }
    }
    if let Err(error) = owner
        .shutdown(Duration::from_millis(1500), Duration::from_millis(1500))
        .await
    {
        fault(error);
    }
}

#[cfg(test)]
#[path = "lio_owner_tests.rs"]
mod tests;
