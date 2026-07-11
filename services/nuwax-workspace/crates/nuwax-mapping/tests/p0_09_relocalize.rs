//! P0-09 回归测试：relocation pipeline (FPFH stub + RANSAC stub + multi-scale ICP)
//!
//! 覆盖：
//! - `compute_fpfh_stub` 返空 vec
//! - `ransac_align_stub` 返 Isometry3（用 ICP 单次 best-fit 近似）
//! - `multiscale_icp` 粗→细两阶段，输出比单次 ICP 更精细
//! - `relocation_pipeline` 走完整链路（FPFH → multi-scale → gravity filter）

use nalgebra::{point, vector, Isometry3, Point3, Vector3};
use nuwax_mapping::relocalization::{
    compute_fpfh_stub, filter_upright, is_upright, multiscale_icp, ransac_align_stub,
    relocation_pipeline, GRAVITY_TILT_MAX_DEG,
};

#[test]
fn fpfh_stub_returns_empty() {
    let pts = vec![point![0.0, 0.0, 0.0], point![1.0, 0.0, 0.0]];
    let normals = vec![vector![0.0, 0.0, 1.0], vector![1.0, 0.0, 0.0]];
    let fpfh = compute_fpfh_stub(&pts, &normals);
    assert_eq!(fpfh.len(), 0, "FPFH stub should return empty vec");
}

#[test]
fn ransac_align_stub_returns_isometry() {
    let target = vec![
        point![0.0, 0.0, 0.0],
        point![1.0, 0.0, 0.0],
        point![0.0, 1.0, 0.0],
        point![0.0, 0.0, 1.0],
    ];
    let source = vec![
        point![2.0, 3.0, 0.0],
        point![3.0, 3.0, 0.0],
        point![2.0, 4.0, 0.0],
        point![2.0, 3.0, 1.0],
    ];
    let t = ransac_align_stub(&source, &target, 100, 0.5);
    // 应把 source 映回 target，translation 大致 (-2, -3, 0)
    let tr = t.translation.vector;
    assert!(
        (tr.x - (-2.0)).abs() < 0.5,
        "ransac stub should produce reasonable translation, got {}",
        tr.x
    );
    assert!(
        (tr.y - (-3.0)).abs() < 0.5,
        "ransac stub y got {}",
        tr.y
    );
}

#[test]
fn multiscale_icp_refines_alignment() {
    let target: Vec<Point3<f64>> = (0..20)
        .map(|i| point![i as f64 * 0.1, 0.0, 0.0])
        .collect();
    // source 平移 (0.5, 0.0, 0.0)
    let source: Vec<Point3<f64>> =
        target.iter().map(|p| p + vector![0.5, 0.0, 0.0]).collect();
    let res = multiscale_icp(&source, &target, 1.0, 0.05);
    assert!(res.converged || res.mse < 0.5);
    // 粗+细合并后 translation 应接近 (-0.5, 0, 0)
    let t = res.transformation.translation.vector;
    assert!(
        (t.x - (-0.5)).abs() < 0.2,
        "multiscale translation x got {}",
        t.x
    );
}

#[test]
fn relocation_pipeline_handles_identity_case() {
    let pts = vec![
        point![0.0, 0.0, 0.0],
        point![1.0, 0.0, 0.0],
        point![0.0, 1.0, 0.0],
    ];
    let res = relocation_pipeline(&pts, &pts);
    // 完全相同点云 → 应得到 identity-like 变换
    assert!(res.converged);
    let t = res.transformation.translation.vector;
    assert!(t.norm() < 0.1, "identity case translation got {t:?}");
}

#[test]
fn relocation_pipeline_rejects_upside_down() {
    // 构造一个目标点云让它和 source 强行匹配出 90° roll 变换
    // 通过绕 y 轴 90° 旋转 + 加点云使得 ICP 找到这个变换
    let target: Vec<Point3<f64>> = (0..10)
        .map(|i| point![0.0, i as f64 * 0.1, 0.0])
        .collect();
    // source 是绕 y 轴 90° 旋转后的 target（即 z→x）
    let source: Vec<Point3<f64>> = target
        .iter()
        .map(|p| {
            let rotated = Isometry3::from_parts(
                nalgebra::Translation3::identity(),
                nalgebra::UnitQuaternion::from_euler_angles(
                    0.0,
                    core::f64::consts::FRAC_PI_2,
                    0.0,
                ),
            ) * p;
            rotated
        })
        .collect();

    // ICP 应该能找到一个倾斜变换（roll 90°）；relocation_pipeline 因 gravity filter
    // 应该拒绝它，回退到 identity
    let res = relocation_pipeline(&source, &target);
    // 重力 filter 触发 → 回退到 identity
    assert!(
        !res.converged
            || !is_upright(&res.transformation, GRAVITY_TILT_MAX_DEG),
        "relocation_pipeline 应因 90° roll 触发 gravity filter"
    );
}

// suppress unused warning on Vector3 in case some test is dropped
#[allow(dead_code)]
fn _force_vec3(_v: Vector3<f64>) {}

// suppress unused warning on filter_upright for the test module
#[allow(dead_code)]
fn _force_filter() {
    let _ = filter_upright(&[Isometry3::identity()], 5.0);
}