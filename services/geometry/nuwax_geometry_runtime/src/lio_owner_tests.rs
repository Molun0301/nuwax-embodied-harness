use super::*;
use nuwax_lio_backends::native::{NativeBackend, NativeRuntimeSpec};
#[tokio::test]
async fn native_child_kill_reports_fault_and_keeps_parent_alive() {
    let runtime = NativeRuntimeSpec {
        backend: NativeBackend::FastLio2,
        cwd: "/".into(),
        executable: "/bin/sleep".into(),
        args: vec!["60".into()],
        env: Default::default(),
    };
    let owner = NativeProcessOwner::spawn(&runtime).unwrap();
    let pid = owner.pid();
    let (_stop, receiver) = watch::channel(false);
    let (fault, mut faults) = tokio::sync::mpsc::unbounded_channel();
    let task = tokio::spawn(supervise_native(owner, receiver, move |reason| {
        let _ = fault.send(reason);
    }));
    nix::sys::signal::kill(
        nix::unistd::Pid::from_raw(pid as i32),
        nix::sys::signal::Signal::SIGKILL,
    )
    .unwrap();
    let reason = tokio::time::timeout(Duration::from_secs(3), faults.recv())
        .await
        .unwrap()
        .unwrap();
    assert!(reason.contains("原生LIO子进程退出"), "{reason}");
    task.await.unwrap();
    assert!(
        unsafe { libc::getpid() } > 0,
        "原生子进程退出后父进程仍可处理请求"
    );
}
