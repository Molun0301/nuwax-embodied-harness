//! P0-13 BasicPathFollowerModule 真副作用回归测试。
//!
//! 覆盖:
//! - lifecycle 幂等
//! - on_path 后输出 cmd_vel
//! - 终点 = 起点 -> goal_reached
//! - 端口 mock(通过 `tokio::sync::mpsc`)收到 cmd_vel

use std::time::Duration;

use nuwax_nav_basic_path_follower::module::{
    BasicPathFollowerModule, BasicPathFollowerModuleConfig, Module,
};
use nuwax_nav_types_messages::{
    NavFrameId, Path2D, Point2, Point3, Pose3, PoseStamped, Quaternion, Timestamp,
};

/// 构造一个 yaw=0 的位姿(单位 quaternion)
fn make_pose(x: f64, y: f64, yaw: f64) -> PoseStamped {
    let half = yaw * 0.5;
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::default(),
        pose: Pose3 {
            position: Point3 { x, y, z: 0.0 },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: half.sin(),
                w: half.cos(),
            },
        },
    }
}

fn msg_path(x: f64, y: f64) -> Path2D {
    Path2D::from_points(vec![Point2::new(x, y)])
}

fn msg_path_multi(pts: &[(f64, f64)]) -> Path2D {
    Path2D::from_points(pts.iter().map(|p| Point2::new(p.0, p.1)).collect())
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_follower_lifecycle_is_idempotent() {
    let mut m = BasicPathFollowerModule::new(BasicPathFollowerModuleConfig::default())
        .expect("module new ok");
    // start 两次不报错
    m.start().expect("first start ok");
    m.start().expect("second start ok (idempotent)");
    // stop 两次不报错
    m.stop().expect("first stop ok");
    m.stop().expect("second stop ok (idempotent)");
    // stop 后再 start 也可(允许重新启动)
    m.start().expect("start after stop ok");
    m.stop().expect("cleanup stop ok");
}

#[test]
fn p0_13_follower_emits_cmd_vel_when_path_set() {
    let mut m = BasicPathFollowerModule::new(BasicPathFollowerModuleConfig::default())
        .expect("module new ok");
    m.on_path(msg_path_multi(&[(1.0, 0.0), (2.0, 0.0)]));
    m.on_odom(make_pose(0.0, 0.0, 0.0));
    let out = m.tick_and_publish().expect("cmd_vel expected");
    // 朝向 x 正方向,目标 (1,0)/(2,0) -> 应输出正 linear.x
    assert!(
        out.linear.x > 0.0,
        "linear.x should be positive, got {:?}",
        out
    );
    // angular_z 接近 0(目标在 x+)
    assert!(out.angular_z.abs() < 0.1, "angular_z={}", out.angular_z);
}

#[test]
fn p0_13_follower_reports_goal_reached() {
    let mut m = BasicPathFollowerModule::new(BasicPathFollowerModuleConfig::default())
        .expect("module new ok");
    // 终点 (0.1, 0) 距 (0,0) = 0.1 < tolerance 0.3
    m.on_path(msg_path(0.1, 0.0));
    m.on_odom(make_pose(0.0, 0.0, 0.0));
    let _ = m.tick_and_publish();
    assert!(m.goal_reached(), "goal_reached should be true at endpoint");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_follower_publishes_to_out_port() {
    let mut m = BasicPathFollowerModule::new(BasicPathFollowerModuleConfig::default())
        .expect("module new ok");
    // 先订阅端口,再注入 path + odom,再手动 tick_and_publish
    // 注意:subscribe 内部用 watch 转发,需要值变化才能发送。
    // 这里在 on_path/on_odom 之前先订阅,然后 tick 触发 watch 写入。
    let mut rx = m.subscribe_cmd_vel();
    m.on_path(msg_path_multi(&[(1.0, 0.0), (2.0, 0.0)]));
    m.on_odom(make_pose(0.0, 0.0, 0.0));
    let out = m.tick_and_publish().expect("tick produced twist");
    // 用 timeout 防止 watch 转发链路 hang
    let received = tokio::time::timeout(Duration::from_secs(1), rx.recv())
        .await
        .expect("no timeout")
        .expect("channel closed");
    assert_eq!(
        received.linear.x, out.linear.x,
        "subscribe should observe latest twist"
    );
    assert!(received.linear.x > 0.0);
    // 显式停掉 lifecycle(如果 start 过)
    let _ = m.stop();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn p0_13_follower_lifecycle_publishes_periodically() {
    // 验证真副作用:lifecycle 启动后,即使 caller 不调 tick,
    // 端口仍会被周期发布(因为 tokio::spawn task 在 tick)
    let cfg = BasicPathFollowerModuleConfig {
        control_rate_hz: 50, // 50Hz 便于快速验证
        ..BasicPathFollowerModuleConfig::default()
    };
    let mut m = BasicPathFollowerModule::new(cfg).expect("module new ok");
    m.on_path(msg_path(1.0, 0.0));
    m.on_odom(make_pose(0.0, 0.0, 0.0));
    m.start().expect("start ok");
    // 给 lifecycle ~100ms 跑几次 tick
    tokio::time::sleep(Duration::from_millis(150)).await;
    // 此时 cmd_vel() 应已被 fill
    let out = m.cmd_vel().expect("cmd_vel should be populated by lifecycle");
    assert!(out.linear.x > 0.0);
    m.stop().expect("stop ok");
}