//! 回归测试:rework22 P0-05 Transform::inverse + BFS sibling 组合
//!
//! 验证:
//! 1. 旋转 90° 后与 inverse 复合应得 identity
//! 2. chain("lidar", "world") 与 chain("world", "lidar") 互为 inverse
//! 3. sibling 场景:world → lidar_left 与 world → lidar_right,lidar_left ↔ lidar_right
//!    经过公共祖先 world 走完整链路,互为 inverse

use nuwax_tf::{make_transform, TfTree};
use nalgebra::Isometry3;

/// 用平移 + 四元数 (XYZW) 构造 TransformStamped 的便捷包装
fn stamped(parent: &str, child: &str, translation: [f64; 3], xyzw: [f64; 4]) -> nuwax_tf::TransformStamped {
    make_transform(parent, child, translation, xyzw)
}

/// 判定 isometry 接近 identity:平移 < 1e-9,旋转角 < 1e-9
fn assert_near_identity(label: &str, t: &Isometry3<f64>) {
    let tr_norm = t.translation.vector.norm();
    // 旋转到 identity 的角距离:angle = |axis * sin(theta/2)| * 2
    // 简化:四元数到 identity 的角度 = 2 * acos(|w|)
    let q = t.rotation;
    let w = q.w.abs().clamp(0.0, 1.0);
    let angle = 2.0 * w.acos();
    assert!(
        tr_norm < 1e-9,
        "{label}: 平移范数 {tr_norm} 应 < 1e-9"
    );
    assert!(
        angle < 1e-9,
        "{label}: 旋转到 identity 的角距离 {angle} 应 < 1e-9"
    );
}

#[test]
fn p0_05_z90_rotation_inverse_yields_identity() {
    // 绕 Z 轴 90°:xyzw = [0, 0, sin(45°), cos(45°)] = [0, 0, √2/2, √2/2]
    let s = 0.5_f64.sqrt();
    let t = stamped("a", "b", [0.0, 0.0, 0.0], [0.0, 0.0, s, s]);
    let inv = t.transform.inverse();
    let product = t.transform * inv;
    assert_near_identity("z90° * inverse", &product);

    // 反向也要:inverse * t = identity
    let product2 = inv * t.transform;
    assert_near_identity("inverse * z90°", &product2);
}

#[test]
fn p0_05_z90_with_translation_inverse_yields_identity() {
    let s = 0.5_f64.sqrt();
    let t = stamped("a", "b", [1.0, 2.0, 3.0], [0.0, 0.0, s, s]);
    let inv = t.transform.inverse();
    let product = t.transform * inv;
    assert_near_identity("z90°+平移 * inverse", &product);
}

#[test]
fn p0_05_chain_forward_inverse_of_reverse() {
    let tree = TfTree::new();
    // world → lidar: 平移 (1, 0, 0)
    tree.set(stamped("world", "lidar", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));

    let forward = tree.chain("lidar", "world").expect("chain lidar->world ok");
    let reverse = tree.chain("world", "lidar").expect("chain world->lidar ok");

    // 语义检查:forward 应把 lidar-点映到 world-点
    // lidar 点 (1, 0, 0) 在 world 应是 (2, 0, 0)(因为 lidar 相对 world 平移 (1,0,0),
    // 所以 lidar 原点 = world 原点 + (1,0,0); lidar 点 (1,0,0) = lidar 原点 + (1,0,0) = world (2,0,0))
    let p_in_lidar = nalgebra::Point3::new(1.0, 0.0, 0.0);
    let p_in_world = forward.transform_point(&p_in_lidar);
    assert!(
        (p_in_world.x - 2.0).abs() < 1e-9,
        "lidar->world: (1,0,0) 应映到 (2,0,0), got {:?}",
        p_in_world
    );

    // 互逆: forward * reverse 应是 identity(但注意 forward: lidar→world, reverse: world→lidar;
    // 这里测的是"它们的语义互逆",即 reverse 应该 = forward.inverse())
    let combined = forward * reverse;
    assert_near_identity("forward * reverse", &combined);

    let combined2 = reverse * forward;
    assert_near_identity("reverse * forward", &combined2);
}

#[test]
fn p0_05_sibling_via_common_ancestor() {
    let tree = TfTree::new();
    // world → lidar_left: 平移 (1, 0, 0)
    tree.set(stamped(
        "world",
        "lidar_left",
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ));
    // world → lidar_right: 平移 (-1, 0, 0)
    tree.set(stamped(
        "world",
        "lidar_right",
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ));

    let left_to_right = tree
        .chain("lidar_left", "lidar_right")
        .expect("chain left->right ok");
    let right_to_left = tree
        .chain("lidar_right", "lidar_left")
        .expect("chain right->left ok");

    // 语义:左 lidar 原点在 world (1,0,0);右 lidar 原点在 world (-1,0,0)。
    // 所以左 lidar 原点映到右 lidar 坐标系 = (1 - (-1), 0, 0) = (2, 0, 0)。
    let left_origin_in_right = left_to_right.translation.vector;
    assert!(
        (left_origin_in_right.x - 2.0).abs() < 1e-9,
        "lidar_left→lidar_right 平移 x 应为 2.0, got {}",
        left_origin_in_right.x
    );
    assert!(
        left_origin_in_right.y.abs() < 1e-9,
        "lidar_left→lidar_right 平移 y 应为 0, got {}",
        left_origin_in_right.y
    );
    assert!(
        left_origin_in_right.z.abs() < 1e-9,
        "lidar_left→lidar_right 平移 z 应为 0, got {}",
        left_origin_in_right.z
    );

    // 反向也对称
    let right_origin_in_left = right_to_left.translation.vector;
    assert!(
        (right_origin_in_left.x + 2.0).abs() < 1e-9,
        "lidar_right→lidar_left 平移 x 应为 -2.0, got {}",
        right_origin_in_left.x
    );

    // 互逆
    let combined = left_to_right * right_to_left;
    assert_near_identity("left_to_right * right_to_left", &combined);
}

#[test]
fn p0_05_multi_level_chain_sibling() {
    // 三层链 + sibling:world → a → b,world → c → d
    let tree = TfTree::new();
    tree.set(stamped("world", "a", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    tree.set(stamped("a", "b", [0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    tree.set(stamped("world", "c", [-1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]));
    tree.set(stamped("c", "d", [0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]));

    // b → d:b 原点在 world = (1, 1, 0);d 原点在 world = (-1, 0, 1)
    // b→d 平移 = b_in_world - d_in_world = (1-(-1), 1-0, 0-1) = (2, 1, -1)
    let b_to_d = tree.chain("b", "d").expect("chain b->d ok");
    let v = b_to_d.translation.vector;
    assert!(
        (v.x - 2.0).abs() < 1e-9,
        "b→d 平移 x 应为 2.0, got {}",
        v.x
    );
    assert!(
        (v.y - 1.0).abs() < 1e-9,
        "b→d 平移 y 应为 1.0, got {}",
        v.y
    );
    assert!(
        (v.z + 1.0).abs() < 1e-9,
        "b→d 平移 z 应为 -1.0, got {}",
        v.z
    );

    // 互逆
    let d_to_b = tree.chain("d", "b").expect("chain d->b ok");
    let combined = b_to_d * d_to_b;
    assert_near_identity("b_to_d * d_to_b", &combined);
}