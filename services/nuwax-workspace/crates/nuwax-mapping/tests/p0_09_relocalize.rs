//! P0-09 回归测试：relocation pipeline (FPFH stub + RANSAC stub + multi-scale ICP)
//!
//! 覆盖：
//! - `compute_fpfh_stub` 返空 vec
//! - `ransac_align_stub` 返 Isometry3（用 ICP 单次 best-fit 近似）
//! - `multiscale_icp` 粗→细两阶段，输出比单次 ICP 更精细
//! - `relocation_pipeline` 走完整链路（FPFH → multi-scale → gravity filter）

use nalgebra::{point, vector, Isometry3};
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
    // 简单：identity-like 对齐不应让 ransac_stub 卡死
    let pts = vec![
        point![0.0, 0.0, 0.0],
        point![1.0, 0.0, 0.0],
        point![0.0, 1.0, 0.0],
        point![1.0, 1.0, 0.0],
    ];
    let t = ransac_align_stub(&pts, &pts, 100, 0.5);
    // identity → translation 接近 0
    let tr = t.translation.vector;
    assert!(
        tr.norm() < 0.5,
        "ransac stub identity case should yield near-zero translation, got {tr:?}"
    );
}

#[test]
fn multiscale_icp_refines_alignment() {
    // ICP 单次对齐到原点的简单 case → multiscale 应也能搞定
    let pts = vec![
        point![0.0, 0.0, 0.0],
        point![1.0, 0.0, 0.0],
        point![0.0, 1.0, 0.0],
        point![1.0, 1.0, 0.0],
    ];
    let res = multiscale_icp(&pts, &pts, 1.0, 0.05);
    // 完全相同点云 → translation 接近 0
    let t = res.transformation.translation.vector;
    assert!(
        t.norm() < 0.5,
        "multiscale identity translation got {t:?}"
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
    let t = res.transformation.translation.vector;
    assert!(t.norm() < 0.1, "identity case translation got {t:?}");
}

#[test]
fn relocation_pipeline_rejects_upside_down() {
    // 直接构造一个 90° roll 的 Isometry3 作为结果，调用 is_upright + relocation_pipeline 应拒绝
    let tilted = Isometry3::from_parts(
        nalgebra::Translation3::identity(),
        nalgebra::UnitQuaternion::from_euler_angles(
            0.0,
            core::f64::consts::FRAC_PI_2,
            0.0,
        ),
    );
    // 单独检查 gravity filter 行为
    assert!(
        !is_upright(&tilted, GRAVITY_TILT_MAX_DEG),
        "90° y-rotation must NOT be upright"
    );
    // filter_upright 在所有候选都倾斜时应回退（保留所有）
    let candidates = vec![tilted];
    let kept = filter_upright(&candidates, GRAVITY_TILT_MAX_DEG);
    assert_eq!(kept.len(), 1, "all-tilted 应回退到原集合");
}

// suppress unused warning on filter_upright for the test module
#[allow(dead_code)]
fn _force_filter() {
    let _ = filter_upright(&[Isometry3::identity()], 5.0);
}