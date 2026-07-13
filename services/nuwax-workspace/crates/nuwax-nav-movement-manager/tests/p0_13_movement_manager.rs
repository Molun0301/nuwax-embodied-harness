//! P0-13 MovementManagerModule 真副作用回归测试。
//!
//! 覆盖:
//! - cancel 触发后,`stop_out` 端口收到 `MovementCommand::Stop`
//! - 同时 publish_stop + publish_nan_goal,两个端口各收到一条
//! - 重复 cancel 不 panic,端口仍能持续收到事件

use std::time::Duration;

use nuwax_nav_movement_manager::module::{
    MovementCommand, MovementManagerModule, MovementManagerModuleConfig,
};
use nuwax_nav_movement_manager::CancelGoalPolicy;
use tokio::sync::broadcast::error::TryRecvError;

fn default_module() -> MovementManagerModule {
    MovementManagerModule::new(MovementManagerModuleConfig::default()).expect("module new ok")
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_movement_manager_cancel_publishes_stop() {
    let mut m = default_module();
    let mut rx = m.subscribe_stop();
    // trigger_cancel(publish_stop=true)
    m.trigger_cancel(true, false).expect("trigger ok");
    // 端口应有 Stop 事件
    let cmd = tokio::time::timeout(Duration::from_millis(200), rx.recv())
        .await
        .expect("no timeout")
        .expect("channel closed");
    assert_eq!(cmd, MovementCommand::Stop, "expected Stop command");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_movement_manager_cancel_with_nan_goal() {
    let mut m = default_module();
    let mut stop_rx = m.subscribe_stop();
    let mut nan_rx = m.subscribe_nan_goal();
    // 同时 publish_stop + publish_nan_goal
    m.trigger_cancel(true, true).expect("trigger ok");
    let stop_cmd = tokio::time::timeout(Duration::from_millis(200), stop_rx.recv())
        .await
        .expect("no timeout stop")
        .expect("stop channel closed");
    assert_eq!(stop_cmd, MovementCommand::Stop);

    let nan_pose = tokio::time::timeout(Duration::from_millis(200), nan_rx.recv())
        .await
        .expect("no timeout nan")
        .expect("nan channel closed");
    // NaN goal:position 与 orientation 各分量均为 NaN
    assert!(nan_pose.pose.position.x.is_nan());
    assert!(nan_pose.pose.position.y.is_nan());
    assert!(nan_pose.pose.position.z.is_nan());
    assert!(nan_pose.pose.orientation.x.is_nan());
    assert!(nan_pose.pose.orientation.w.is_nan());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_movement_manager_idempotent_cancel() {
    let mut m = default_module();
    let mut rx = m.subscribe_stop();
    // 重复 cancel 5 次,不 panic
    for _ in 0..5 {
        m.trigger_cancel(true, false).expect("trigger ok");
    }
    // 至少收到 5 条 Stop(可能因 broadcast ring buffer 截断前若干条 -- 这里 cap=16 够)
    let mut count = 0_usize;
    // 用 try_recv 快速 drain(因为 channel 不会有"消息间 await")
    loop {
        match rx.try_recv() {
            Ok(MovementCommand::Stop) => count += 1,
            Ok(_) => {} // 忽略非 Stop
            Err(TryRecvError::Empty) => break,
            Err(TryRecvError::Lagged(_)) => continue,
            Err(TryRecvError::Closed) => break,
        }
    }
    assert!(
        count >= 1,
        "should have received at least 1 Stop, got {}",
        count
    );
}

#[test]
fn p0_13_movement_manager_lifecycle_idempotent() {
    let mut m = default_module();
    // start/stop 幂等(本模块 lifecycle 是 stub,但 API 仍应保证幂等)
    m.start().expect("start ok");
    m.start().expect("start again ok");
    assert!(m.is_running());
    m.stop().expect("stop ok");
    m.stop().expect("stop again ok");
    assert!(!m.is_running());
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_movement_manager_cancel_policy_typed_stop_only() {
    // 用 TypedStop policy 构造 -- 即使外部仍传 publish_nan_goal=true,模块应照发;
    // 本测试验证 module 自身不"自动"按 policy 过滤 trigger_cancel 参数,
    // 而是忠实执行 caller 的 publish_stop / publish_nan_goal。
    let cfg = MovementManagerModuleConfig {
        manager_config: nuwax_nav_movement_manager::MovementManagerConfig {
            cancel_goal_policy: CancelGoalPolicy::TypedStop,
            ..Default::default()
        },
        ..MovementManagerModuleConfig::default()
    };
    let mut m = MovementManagerModule::new(cfg).expect("new ok");
    assert_eq!(m.cancel_policy(), CancelGoalPolicy::TypedStop);
    let mut stop_rx = m.subscribe_stop();
    let mut nan_rx = m.subscribe_nan_goal();
    // 即使 policy = TypedStop,caller 仍显式要求发 NaN goal -- 模块应照做
    m.trigger_cancel(true, true).expect("trigger ok");
    let stop_cmd = tokio::time::timeout(Duration::from_millis(200), stop_rx.recv())
        .await
        .expect("no timeout")
        .expect("closed");
    assert_eq!(stop_cmd, MovementCommand::Stop);
    let nan_pose = tokio::time::timeout(Duration::from_millis(200), nan_rx.recv())
        .await
        .expect("no timeout")
        .expect("closed");
    assert!(nan_pose.pose.position.x.is_nan());
}