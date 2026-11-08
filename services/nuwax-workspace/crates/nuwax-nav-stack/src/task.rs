//! 固定导航任务的线程、启动结果与关闭回收。

use std::future::Future;
use std::pin::Pin;
use std::sync::mpsc;
use std::thread::JoinHandle;
use tokio::sync::oneshot;
use crate::{NavStackError, NavStackResult};

pub(crate) type ReadySender = mpsc::SyncSender<Result<(), String>>;
pub(crate) type OwnerFuture = Pin<Box<dyn Future<Output = NavStackResult<()>> + Send>>;
pub(crate) type OwnerRun = Box<dyn FnOnce(oneshot::Receiver<()>, ReadySender) -> OwnerFuture + Send>;

pub(crate) struct TaskThread {
    label: &'static str,
    shutdown: Option<oneshot::Sender<()>>,
    thread: Option<JoinHandle<NavStackResult<()>>>,
}

pub(crate) fn spawn_task(label: &'static str, run: OwnerRun) -> NavStackResult<TaskThread> {
    let (shutdown, receiver) = oneshot::channel();
    let (ready, result) = mpsc::sync_channel(1);
    let startup_error = ready.clone();
    let thread = std::thread::Builder::new().name(format!("nav-stack-{label}")).spawn(move || {
        let result = tokio::runtime::Builder::new_current_thread().enable_all().build()
            .map_err(NavStackError::from)
            .and_then(|runtime| runtime.block_on(run(receiver, ready)));
        if let Err(error) = &result {
            let _ = startup_error.send(Err(error.to_string()));
        }
        result
    })?;
    let mut task = TaskThread { label, shutdown: Some(shutdown), thread: Some(thread) };
    match result.recv() {
        Ok(Ok(())) => Ok(task),
        Ok(Err(reason)) => Err(NavStackError::Internal(reason)),
        Err(_) => {
            task.shutdown()?;
            Err(NavStackError::Internal(format!("{label} 在启动完成前退出")))
        }
    }
}

impl TaskThread {
    pub(crate) fn is_running(&mut self) -> NavStackResult<bool> {
        if self.thread.as_ref().is_some_and(|thread| thread.is_finished()) {
            self.join()?;
        }
        Ok(self.thread.is_some())
    }

    pub(crate) fn label(&self) -> &'static str { self.label }

    fn join(&mut self) -> NavStackResult<()> {
        if let Some(thread) = self.thread.take() {
            thread.join().map_err(|_| NavStackError::Internal(format!("{} 任务线程异常退出", self.label)))??;
        }
        Ok(())
    }

    pub(crate) fn shutdown(&mut self) -> NavStackResult<()> {
        if let Some(shutdown) = self.shutdown.take() { let _ = shutdown.send(()); }
        self.join()
    }
}

impl Drop for TaskThread {
    fn drop(&mut self) { let _ = self.shutdown(); }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

    #[test]
    fn startup_failure_returns_the_actual_reason_and_reaps_started_tasks() {
        let stopped = Arc::new(AtomicBool::new(false));
        let observed = stopped.clone();
        let first = spawn_task("first", Box::new(move |shutdown, ready| Box::pin(async move {
            ready.send(Ok(())).unwrap();
            let _ = shutdown.await;
            observed.store(true, Ordering::Release);
            Ok(())
        }))).unwrap();
        let error = spawn_task("failed", Box::new(|_, _| Box::pin(async {
            Err(NavStackError::Internal("订阅失败：测试原因".into()))
        }))).err().unwrap();
        assert!(error.to_string().contains("订阅失败：测试原因"));
        drop(first);
        assert!(stopped.load(Ordering::Acquire));
    }

    #[test]
    fn errors_after_startup_are_reported_by_the_same_thread_result() {
        let (release, waiting) = oneshot::channel();
        let mut task = spawn_task("later-error", Box::new(|_, ready| Box::pin(async move {
            ready.send(Ok(())).unwrap();
            let _ = waiting.await;
            Err(NavStackError::Internal("任务执行失败：测试原因".into()))
        }))).unwrap();
        release.send(()).unwrap();
        let error = task.shutdown().unwrap_err();
        assert!(error.to_string().contains("任务执行失败：测试原因"));
    }
}
