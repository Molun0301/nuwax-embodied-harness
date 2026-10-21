//! 实测运动与地图姿态分开保留；这里只做轴系表达，不由命令制造观测。
use crate::MeasuredMotion;
/// 与生产反馈同宿主的原单调时钟，供既有物理模拟快照标注接收时刻。
pub use nuwax_motion_safety_guard::measured_motion::monotonic_ns;
use nuwax_nav_types_messages::PoseStamped;

/// SCAN 可选边界；缺失交给原 SCAN 历史轨迹机制，不伪造测速。
#[derive(Clone, Debug, PartialEq)]
pub struct MotionBoundary {
    /// 地图轴系中沿用设备既有受控参考点约定的速度；使用当前地图姿态重新表达异步 body 测量。
    pub velocity: Option<[f64; 3]>,
    /// 同测量姿态对应的 Euler yaw 导数，不直接使用 gyro z。
    pub yaw_rate: Option<f64>,
    /// 速度来源，缺失时为 None。
    pub source_id: Option<String>,
    /// 实测原采样时间，不假定等于位姿采样时间。
    pub motion_timestamp_ns: Option<u64>,
    /// 本次坐标表达使用的真实地图位姿时间。
    pub pose_timestamp_ns: i64,
    /// 位姿原时钟种类，不能把不同钟域当同步样本。
    pub pose_clock: nuwax_nav_types_messages::ClockKind,
    /// 同为Unix时钟语义时的测速减位姿采样时间差。
    pub sample_time_delta_ns: Option<i128>,
}

/// quaternion 是 body→map；沿用适配器参考点，未声称 SDK 原点已独立标定，不新增杆臂补偿。
pub fn boundary_for_pose(pose: &PoseStamped, measured: Option<&MeasuredMotion>) -> MotionBoundary {
    let measured = measured.filter(|m| m.is_current());
    let velocity = measured.map(|m| {
        let q = pose.pose.orientation;
        let [vx, vy, vz] = m.body_linear_xyz;
        [
            (1. - 2. * (q.y * q.y + q.z * q.z)) * vx
                + 2. * (q.x * q.y - q.w * q.z) * vy
                + 2. * (q.x * q.z + q.w * q.y) * vz,
            2. * (q.x * q.y + q.w * q.z) * vx
                + (1. - 2. * (q.x * q.x + q.z * q.z)) * vy
                + 2. * (q.y * q.z - q.w * q.x) * vz,
            2. * (q.x * q.z - q.w * q.y) * vx
                + 2. * (q.y * q.z + q.w * q.x) * vy
                + (1. - 2. * (q.x * q.x + q.y * q.y)) * vz,
        ]
    });
    let yaw_rate = measured
        .and_then(|m| m.orientation_wxyz.map(|q| (m, q)))
        .and_then(|(m, q)| {
            let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            let [w, x, y, z] = q.map(|v| v / norm);
            let roll = (2. * (w * x + y * z)).atan2(1. - 2. * (x * x + y * y));
            let sin_pitch = (2. * (w * y - z * x)).clamp(-1., 1.);
            let cos_pitch = (1. - sin_pitch * sin_pitch).sqrt();
            // Euler yaw 在万向锁处无定义；保持未知，不把奇异值当实际角速度。
            (cos_pitch > 1e-8).then(|| {
                (roll.sin() * m.body_angular_xyz[1] + roll.cos() * m.body_angular_xyz[2])
                    / cos_pitch
            })
        });
    MotionBoundary {
        velocity,
        yaw_rate,
        source_id: measured.map(|m| m.source_id.clone()),
        motion_timestamp_ns: measured.map(|m| m.source_timestamp_ns),
        pose_timestamp_ns: pose.stamp.nanos,
        pose_clock: pose.stamp.clock,
        sample_time_delta_ns: measured
            .filter(|_| pose.stamp.clock == nuwax_nav_types_messages::ClockKind::Unix)
            .map(|m| i128::from(m.source_timestamp_ns) - i128::from(pose.stamp.nanos)),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use nuwax_nav_types_messages::{NavFrameId, Point3, Pose3, Quaternion, Timestamp};
    pub(crate) fn measured(v: [f64; 3], w: [f64; 3]) -> MeasuredMotion {
        let now = nuwax_motion_safety_guard::measured_motion::monotonic_ns();
        MeasuredMotion {
            source_id: "test:actual-state".into(),
            source_timestamp_ns: 123,
            received_monotonic_ns: now,
            valid_until_monotonic_ns: now + 1_000_000_000,
            frame_id: "base_link".into(),
            body_linear_xyz: v,
            body_angular_xyz: w,
            orientation_wxyz: Some([1., 0., 0., 0.]),
        }
    }
    fn pose(yaw: f64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(0, 456),
            pose: Pose3 {
                position: Point3::new(0., 0., 0.),
                orientation: Quaternion::from_yaw(yaw),
            },
        }
    }
    #[test]
    fn body_velocity_rotates_once_and_keeps_two_distinct_sample_times() {
        let m = measured([1., 0., 0.], [0., 0., 0.3]);
        let b = boundary_for_pose(&pose(std::f64::consts::FRAC_PI_2), Some(&m));
        let v = b.velocity.unwrap();
        assert!(v[0].abs() < 1e-12);
        assert!((v[1] - 1.).abs() < 1e-12);
        assert_eq!(b.motion_timestamp_ns, Some(123));
        assert_eq!(b.pose_timestamp_ns, 456);
        assert_eq!(b.yaw_rate, Some(0.3));
    }
    #[test]
    fn body_gyro_z_is_not_yaw_derivative_on_a_tilt() {
        let mut m = measured([0.; 3], [0., 1., 0.]);
        let roll = std::f64::consts::FRAC_PI_4;
        m.orientation_wxyz = Some([(roll / 2.).cos(), (roll / 2.).sin(), 0., 0.]);
        assert!(
            (boundary_for_pose(&pose(0.), Some(&m)).yaw_rate.unwrap() - roll.sin()).abs() < 1e-12
        );
        m.orientation_wxyz = None;
        assert_eq!(boundary_for_pose(&pose(0.), Some(&m)).yaw_rate, None);
    }
    #[test]
    fn euler_singularity_is_unknown_not_a_fabricated_zero() {
        let mut m = measured([0.; 3], [0., 0., 1.]);
        let a = std::f64::consts::FRAC_PI_4;
        m.orientation_wxyz = Some([a.cos(), 0., a.sin(), 0.]);
        assert_eq!(boundary_for_pose(&pose(0.), Some(&m)).yaw_rate, None);
        let near = (std::f64::consts::FRAC_PI_2 - 1e-4) / 2.;
        m.orientation_wxyz = Some([near.cos(), 0., near.sin(), 0.]);
        assert!(boundary_for_pose(&pose(0.), Some(&m))
            .yaw_rate
            .unwrap()
            .is_finite());
    }
    #[test]
    fn missing_or_expired_measurement_stays_unknown() {
        let p = pose(0.);
        assert_eq!(boundary_for_pose(&p, None).velocity, None);
        let mut m = measured([0.; 3], [0.; 3]);
        assert_eq!(boundary_for_pose(&p, Some(&m)).velocity, Some([0.; 3]));
        m.valid_until_monotonic_ns = 1;
        assert_eq!(boundary_for_pose(&p, Some(&m)).velocity, None);
    }
}
