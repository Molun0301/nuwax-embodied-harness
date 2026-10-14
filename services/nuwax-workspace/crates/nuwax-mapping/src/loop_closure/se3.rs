//! SE(3) 李群数学工具
//!
//! 提供：
//! - `se3_log_to_vector6`：Isometry3 → 6D twist `[tx, ty, tz, rx, ry, rz]`
//! - `se3_exp_from_vector6`：6D twist → Isometry3
//! - `se3_adjoint`：SE(3) Adjoint 6x6 矩阵
//! - `linearize_edge`：对单边 `e_ij = log(measured^{-1} * from^{-1} * to)`
//!   给出残差 `e` 与关于 `from`、`to` 节点 tangent 坐标的雅可比
//!   给出包含 `Log` 微分的 right-perturbation 雅可比
//!
//! 约定：
//! - twist 顺序 = `[tx, ty, tz, rx, ry, rz]`（平移在前，旋转在后）
//! - rotation 用 axis-angle 向量表示（与 nalgebra UnitQuaternion 一致）
//! - 用 right perturbation 约定：`T_new = T * exp(δ)`

use nalgebra::{Isometry3, Matrix3, Matrix6, Translation3, UnitQuaternion, Vector3, Vector6};

/// SE(3) log map：Isometry3 → 6D twist `[v_x, v_y, v_z, ω_x, ω_y, ω_z]`
///
/// 标准 SE(3) log 约定：`exp(log(T)) = T`。其中：
/// - 平移：`v = V^{-1} · t`（V 是 SE(3) left Jacobian）
/// - 旋转：转轴-角（Rodrigues）→ `ω = axis * theta`
///
/// 小角度时用一阶泰勒近似避免奇异。
#[must_use]
pub fn se3_log_to_vector6(pose: &Isometry3<f64>) -> Vector6<f64> {
    let t = pose.translation.vector;

    // nalgebra canonicalizes the quaternion hemisphere here, so q and -q
    // produce the same principal log with rotation norm <= pi.
    let rotation = pose.rotation.scaled_axis();

    // 平移部分：v = V^{-1} · t
    // V^{-1} = I - 0.5 [ω]× + (1/θ² - (1 + cos θ)/(2 θ sin θ)) [ω]×²
    // 小角度近似：V^{-1} ≈ I - 0.5 [ω]× + (1/12) [ω]×²
    let theta_sq = rotation.norm_squared();
    let translation = if theta_sq < 1e-12 {
        // 小角度也保留旋转和平移的耦合，不能直接把平移当作李代数分量。
        let omega_skew = skew3(&rotation);
        t - 0.5 * omega_skew * t + (1.0 / 12.0) * omega_skew * (omega_skew * t)
    } else {
        let theta = theta_sq.sqrt();
        let a = 0.5;
        let b = (1.0 / theta_sq) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());
        let omega_skew = skew3(&rotation);
        t - a * omega_skew * t + b * omega_skew * (omega_skew * t)
    };

    Vector6::new(
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
    )
}

/// SE(3) exp map：6D twist `[tx, ty, tz, rx, ry, rz]` → Isometry3
///
/// 旋转部分用 Rodrigues 公式从 axis-angle 构造四元数；
/// 平移部分用 left-Jacobian `V^{-1}` 乘以 `v`（`exp(ξ)` 的 translation 列）：
///
///   `exp(ξ) = [ exp(ω) ,   V · v ;  0 , 1 ]`
///
/// 其中 `V = I + (1 - cos θ)/θ² [ω]× + (θ - sin θ)/θ³ [ω]×²`。
///
/// 反算 `v = V^{-1} · t_exp`，确保 `exp(log(T)) = T`（数值容差内）。
#[must_use]
pub fn se3_exp_from_vector6(twist: &Vector6<f64>) -> Isometry3<f64> {
    let v = Vector3::new(twist[0], twist[1], twist[2]);
    let omega = Vector3::new(twist[3], twist[4], twist[5]);

    let theta_sq = omega.norm_squared();
    let theta = theta_sq.sqrt();

    // 旋转 = exp(ω̂)，Rodrigues
    let rotation = if theta_sq < 1e-12 {
        // 小角度：q ≈ (1 - |ω|²/8, ω/2)，再 normalize
        let qv = 0.5 * omega;
        let qw = (1.0 - 0.25 * omega.norm_squared()).max(0.0).sqrt();
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(qw, qv.x, qv.y, qv.z))
    } else {
        let k = omega / theta;
        // 等价于 R = exp([k_skew] * theta)
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(k), theta)
    };

    // 平移 = V · v
    let translation = if theta_sq < 1e-12 {
        // V ≈ I + 0.5 [ω]× + (1/6) [ω]×²
        let skew = skew3(&omega);
        v + 0.5 * skew * v + (1.0 / 6.0) * skew * (skew * v)
    } else {
        let theta2 = theta_sq;
        let theta3 = theta_sq * theta;
        let a = (1.0 - theta.cos()) / theta2;
        let b = (theta - theta.sin()) / theta3;
        let omega_skew = skew3(&omega);
        v + a * omega_skew * v + b * omega_skew * (omega_skew * v)
    };

    Isometry3::from_parts(Translation3::from(translation), rotation)
}

/// 反对称矩阵（so(3) 元素 → so(3) 生成元）
fn skew3(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// SE(3) Adjoint 6×6 矩阵
///
/// 对 `T = [R t; 0 1]`：
///   `Ad_T = [ R   [t]× R ;  0   R ]`
///
/// 用于把一个 tangent 坐标 `ξ` 在不同坐标系间转换：
///   `Ad_T · ξ_local = ξ_global`
#[must_use]
pub fn se3_adjoint(pose: &Isometry3<f64>) -> Matrix6<f64> {
    let r = pose.rotation.to_rotation_matrix();
    let t = pose.translation.vector;
    let t_skew = skew3(&t);

    // 按块拼接 6x6：
    //   [ R   t_skew * R ]
    //   [ 0   R          ]
    let mut out = Matrix6::zeros();
    // 左上 R + 右下 R
    for i in 0..3 {
        for j in 0..3 {
            out[(i, j)] = r[(i, j)];
            out[(i + 3, j + 3)] = r[(i, j)];
        }
    }
    // 右上 [t]× R
    let tr = t_skew * r;
    for i in 0..3 {
        for j in 0..3 {
            out[(i, j + 3)] = tr[(i, j)];
        }
    }
    // 左下保持 0（已初始化为零）
    out
}

/// se(3) 的小 adjoint：`ad_[v,w] = [[w^, v^], [0, w^]]`。
fn se3_little_adjoint(twist: &Vector6<f64>) -> Matrix6<f64> {
    let v = Vector3::new(twist[0], twist[1], twist[2]);
    let omega = Vector3::new(twist[3], twist[4], twist[5]);
    let omega_skew = skew3(&omega);
    let v_skew = skew3(&v);
    let mut out = Matrix6::zeros();
    for row in 0..3 {
        for col in 0..3 {
            out[(row, col)] = omega_skew[(row, col)];
            out[(row, col + 3)] = v_skew[(row, col)];
            out[(row + 3, col + 3)] = omega_skew[(row, col)];
        }
    }
    out
}

/// `Log(T * Exp(delta))` 对 `delta` 的微分，即 SE(3) 右雅可比的逆。
///
/// 使用 Bernoulli 级数
/// `J_r^{-1}(xi) = I + 1/2 ad_xi + sum B_2n/(2n)! ad_xi^(2n)`。
/// `Log` 返回主值（旋转角不超过 pi），因此该级数处于收敛域内；保留到 20 次项，
/// 并由接近 pi 的数值微分回归约束其误差。
fn se3_right_jacobian_inverse(twist: &Vector6<f64>) -> Matrix6<f64> {
    let ad = se3_little_adjoint(twist);
    let ad_squared = ad * ad;
    let mut result = Matrix6::identity() + 0.5 * ad;
    let mut power = ad_squared;
    let coefficients = [
        1.0 / 12.0,
        -1.0 / 720.0,
        1.0 / 30_240.0,
        -1.0 / 1_209_600.0,
        1.0 / 47_900_160.0,
        -691.0 / 1_307_674_368_000.0,
        7.0 / 523_069_747_200.0,
        -3_617.0 / 10_670_622_842_880_000.0,
        43_867.0 / 5_109_094_217_170_944_000.0,
        -174_611.0 / 802_857_662_698_291_200_000.0,
    ];
    for coefficient in coefficients {
        result += coefficient * power;
        power *= ad_squared;
    }
    result
}

/// 对单边做线性化，返回残差与关于 from / to 节点 tangent 坐标的雅可比。
///
/// 残差约定：
///   `predicted = from^{-1} * to`
///   `err_pose  = measurement^{-1} * predicted`
///   `e         = log(err_pose) ∈ ℝ⁶`
///
/// 雅可比约定（right perturbation，`T <- T * exp(delta)`）：
/// - `from` 扰动使 `err_pose` 右乘 `Exp(-Ad(predicted^-1) * delta_from)`；
/// - `to` 扰动使 `err_pose` 右乘 `Exp(delta_to)`；
/// - `Log(err_pose * Exp(eta))` 的微分为 `J_r^-1(e) * eta`。
///
/// 因而 `J_from = -J_r^-1(e) * Ad(predicted^-1)`，`J_to = J_r^-1(e)`。
#[must_use]
pub fn linearize_edge(
    measurement: &Isometry3<f64>,
    from_pose: &Isometry3<f64>,
    to_pose: &Isometry3<f64>,
) -> (Vector6<f64>, Matrix6<f64>, Matrix6<f64>) {
    let predicted = from_pose.inverse() * to_pose;
    let err_pose = measurement.inverse() * predicted;
    let e = se3_log_to_vector6(&err_pose);

    let log_jacobian = se3_right_jacobian_inverse(&e);
    let adj_pred_inv = se3_adjoint(&predicted.inverse());

    let j_from = -(log_jacobian * adj_pred_inv);
    let j_to = log_jacobian;
    (e, j_from, j_to)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// 纯旋转 roundtrip：exp(log(T)) ≈ T
    #[test]
    fn se3_exp_log_roundtrip_rotation() {
        let pose = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_euler_angles(0.3, 0.5, -0.7),
        );
        let v = se3_log_to_vector6(&pose);
        let back = se3_exp_from_vector6(&v);
        let diff = (back.translation.vector - pose.translation.vector).norm();
        assert!(diff < 1e-9, "translation diff = {diff}");
        // rotation: 比较 quaternion
        let dot = (back.rotation.coords.dot(&pose.rotation.coords)).abs();
        assert!(
            (1.0 - dot).abs() < 1e-9,
            "rotation diff (quat dot) = {}",
            (1.0 - dot).abs()
        );
    }

    /// 纯平移 roundtrip：exp(log(T)) ≈ T
    #[test]
    fn se3_exp_log_roundtrip_translation() {
        let pose = Isometry3::translation(1.5, -2.0, 3.7);
        let v = se3_log_to_vector6(&pose);
        let back = se3_exp_from_vector6(&v);
        let diff = (back.translation.vector - pose.translation.vector).norm();
        assert!(diff < 1e-9, "translation diff = {diff}");
    }

    /// 旋转+平移 roundtrip
    #[test]
    fn se3_exp_log_roundtrip_full() {
        let pose = Isometry3::from_parts(
            Translation3::new(0.5, -0.3, 1.2),
            UnitQuaternion::from_euler_angles(0.2, -0.4, 1.1),
        );
        let v = se3_log_to_vector6(&pose);
        let back = se3_exp_from_vector6(&v);
        let diff = (back.translation.vector - pose.translation.vector).norm();
        assert!(diff < 1e-9, "translation diff = {diff}");
        let dot = (back.rotation.coords.dot(&pose.rotation.coords)).abs();
        assert!((1.0 - dot).abs() < 1e-9);
    }

    /// log(identity) = 0
    #[test]
    fn se3_log_identity_is_zero() {
        let v = se3_log_to_vector6(&Isometry3::identity());
        for k in 0..6 {
            assert_relative_eq!(v[k], 0.0, epsilon = 1e-12);
        }
    }

    #[test]
    fn se3_log_uses_principal_rotation_for_negative_quaternion_w() {
        let pose = Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 3.5),
        );
        assert!(pose.rotation.w < 0.0, "fixture must exercise w < 0");
        let log = se3_log_to_vector6(&pose);
        let rotation = Vector3::new(log[3], log[4], log[5]);
        assert!(rotation.norm() <= core::f64::consts::PI);
        assert_relative_eq!(rotation.z, 3.5 - core::f64::consts::TAU, epsilon = 1e-12);
    }

    /// exp(0) = identity
    #[test]
    fn se3_exp_zero_is_identity() {
        let t = se3_exp_from_vector6(&Vector6::zeros());
        assert!(t.translation.vector.norm() < 1e-12);
        let dot = (t
            .rotation
            .coords
            .dot(&Isometry3::identity().rotation.coords))
        .abs();
        assert!((1.0 - dot).abs() < 1e-12);
    }

    /// Adjoint 在 identity 处 ≈ I
    #[test]
    fn se3_adjoint_identity_is_identity() {
        let adj = se3_adjoint(&Isometry3::identity());
        for i in 0..6 {
            for j in 0..6 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert_relative_eq!(adj[(i, j)], expected, epsilon = 1e-12);
            }
        }
    }

    /// Adjoint 群性质：Ad(T1·T2) = Ad(T1)·Ad(T2)
    #[test]
    fn se3_adjoint_homomorphism() {
        let t1 = Isometry3::from_parts(
            Translation3::new(0.3, -0.1, 0.2),
            UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
        );
        let t2 = Isometry3::from_parts(
            Translation3::new(-0.4, 0.5, 0.1),
            UnitQuaternion::from_euler_angles(-0.2, 0.15, 0.25),
        );
        let lhs = se3_adjoint(&(t1 * t2));
        let rhs = se3_adjoint(&t1) * se3_adjoint(&t2);
        let diff = (lhs - rhs).norm();
        assert!(diff < 1e-9, "Ad homomorphism diff = {diff}");
    }

    /// 对零残差：linearize_edge 雅可比 ≈ ±I（旧占位行为）
    #[test]
    fn linearize_edge_zero_residual_jacobian_is_identity() {
        // 当 from = to = measurement = I 时,err_pose = I 且 predicted = I,
        // J_from = -I, J_to = +I(零残差情况下退化)
        let m = Isometry3::identity();
        let from = Isometry3::identity();
        let to = Isometry3::identity();
        let (e, j_from, j_to) = linearize_edge(&m, &from, &to);
        assert!(e.norm() < 1e-12, "residual should be zero, got {e}");
        for i in 0..6 {
            for j in 0..6 {
                let exp_from = if i == j { -1.0 } else { 0.0 };
                let exp_to = if i == j { 1.0 } else { 0.0 };
                assert_relative_eq!(j_from[(i, j)], exp_from, epsilon = 1e-9);
                assert_relative_eq!(j_to[(i, j)], exp_to, epsilon = 1e-9);
            }
        }
    }

    /// 非零残差的 Log 微分包含平移/旋转耦合。
    #[test]
    fn linearize_edge_jacobian_includes_adjoint_coupling() {
        // from = to = I, measurement = m: predicted = I, err_pose = m^-1。
        let m = Isometry3::from_parts(
            Translation3::new(1.0, 0.0, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, 0.5),
        );
        let (e, j_from, _j_to) = linearize_edge(&m, &Isometry3::identity(), &Isometry3::identity());
        // 残差 log(m^{-1}) 非零 = [tx, ty, tz, rx, ry, rz] 全非零(平移+旋转)
        assert!(e.norm() > 0.1, "nontrivial residual expected, got {e:?}");
        // J_r^-1(log(m^-1)) 应区别于零残差处的 -I。
        assert!(
            (j_from + Matrix6::identity()).norm() > 1e-3,
            "非零残差雅可比不应退化成 -I"
        );
    }

    /// 对非零残差：linearize_edge 雅可比非平凡（区别于 ±I 占位）
    #[test]
    fn linearize_edge_nontrivial_residual_has_offdiag() {
        // from = I, to = I, measurement = 旋转 + 平移 → err_pose 非平凡
        let m = Isometry3::from_parts(
            Translation3::new(0.3, -0.2, 0.4),
            UnitQuaternion::from_euler_angles(0.0, 0.0, core::f64::consts::FRAC_PI_4),
        );
        let (e, j_from, j_to) = linearize_edge(&m, &Isometry3::identity(), &Isometry3::identity());
        // 旋转部分应非零
        let rot_norm = (e[3] * e[3] + e[4] * e[4] + e[5] * e[5]).sqrt();
        assert!(
            rot_norm > 0.5,
            "expected rotation residual ~ π/4, got {rot_norm}"
        );
        // Log 微分的右上块非零，区别于 ±I 占位。
        let mut has_offdiag = false;
        for i in 0..3 {
            for j in 3..6 {
                if j_from[(i, j)].abs() > 1e-6 || j_to[(i, j)].abs() > 1e-6 {
                    has_offdiag = true;
                }
            }
        }
        assert!(
            has_offdiag,
            "non-trivial translation must produce off-diagonal J blocks (got ±I)"
        );
    }

    /// 雅可比一致性：扰动数值微分 vs 解析右雅可比（right perturbation 约定）
    #[test]
    fn linearize_edge_matches_numerical_derivative() {
        // from/to/measurement 全都非平凡，以覆盖 J_from 中的 Ad(predicted^-1)。
        let m = Isometry3::from_parts(
            Translation3::new(0.4, -0.3, 0.2),
            UnitQuaternion::from_euler_angles(0.3, -0.2, 0.5),
        );
        let from = Isometry3::from_parts(
            Translation3::new(-0.2, 0.1, 0.35),
            UnitQuaternion::from_euler_angles(0.1, -0.25, 0.2),
        );
        let to = Isometry3::from_parts(
            Translation3::new(0.8, 0.45, -0.15),
            UnitQuaternion::from_euler_angles(-0.15, 0.2, 0.55),
        );
        let (e0, j_from, j_to) = linearize_edge(&m, &from, &to);

        // 数值验证：right perturbation  T_new = T · exp(δξ)
        let eps = 1e-7;
        for axis in 0..6 {
            let mut delta = Vector6::zeros();
            delta[axis] = eps;
            // 扰动 from
            let pert_from = from * se3_exp_from_vector6(&delta);
            let predicted_new = pert_from.inverse() * to;
            let err_new = m.inverse() * predicted_new;
            let e_new_from = se3_log_to_vector6(&err_new);
            let mut col_from = Vector6::zeros();
            for k in 0..6 {
                col_from[k] = j_from[(k, axis)];
            }
            let expected_from = (e_new_from - e0) / eps;
            let diff_from = (col_from - expected_from).norm();
            assert!(
                diff_from < 1e-4,
                "J_from[{axis}] numerical mismatch: analytic={col_from:?}, numeric={expected_from:?}, diff={diff_from}"
            );

            // 扰动 to
            let pert_to = to * se3_exp_from_vector6(&delta);
            let predicted_new_to = from.inverse() * pert_to;
            let err_new_to = m.inverse() * predicted_new_to;
            let e_new_to = se3_log_to_vector6(&err_new_to);
            let mut col_to = Vector6::zeros();
            for k in 0..6 {
                col_to[k] = j_to[(k, axis)];
            }
            let expected_to = (e_new_to - e0) / eps;
            let diff_to = (col_to - expected_to).norm();
            assert!(
                diff_to < 1e-4,
                "J_to[{axis}] numerical mismatch: analytic={col_to:?}, numeric={expected_to:?}, diff={diff_to}"
            );
        }
    }

    /// Bernoulli 级数在主值 Log 的边界附近仍需匹配数值微分。
    #[test]
    fn linearize_edge_matches_numerical_derivative_near_pi() {
        let axis = nalgebra::Unit::new_normalize(Vector3::new(0.3, -0.4, 0.5));
        let measurement = Isometry3::from_parts(
            Translation3::new(0.7, -0.4, 0.2),
            UnitQuaternion::from_axis_angle(&axis, 3.0),
        );
        let from = Isometry3::identity();
        let to = Isometry3::identity();
        let (e0, j_from, j_to) = linearize_edge(&measurement, &from, &to);

        let epsilon = 1e-7;
        for axis_index in 0..6 {
            let mut delta = Vector6::zeros();
            delta[axis_index] = epsilon;

            let perturbed_from = from * se3_exp_from_vector6(&delta);
            let from_error = measurement.inverse() * perturbed_from.inverse() * to;
            let numerical_from = (se3_log_to_vector6(&from_error) - e0) / epsilon;
            let analytic_from = j_from.column(axis_index).into_owned();
            assert!(
                (analytic_from - numerical_from).norm() < 1e-4,
                "near-pi J_from[{axis_index}] mismatch"
            );

            let perturbed_to = to * se3_exp_from_vector6(&delta);
            let to_error = measurement.inverse() * from.inverse() * perturbed_to;
            let numerical_to = (se3_log_to_vector6(&to_error) - e0) / epsilon;
            let analytic_to = j_to.column(axis_index).into_owned();
            assert!(
                (analytic_to - numerical_to).norm() < 1e-4,
                "near-pi J_to[{axis_index}] mismatch"
            );
        }
    }
}
