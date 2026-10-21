//! 独立于命令回执的实测运动快照；只沿用来源截止，不在读取时续期。
use crate::wire::MeasuredMotionFeedback;

/// 沿用设备既有受控参考点约定、机体轴系的真实测速，位姿是另一条独立采样流。
#[derive(Clone, Debug, PartialEq)]
pub struct MeasuredMotion {
    /// 设备/估计器来源标识，不能使用 command_ack。
    pub source_id: String,
    /// 原采样时间；不改成 APPLIED 时间。
    pub source_timestamp_ns: u64,
    /// 同宿主接收样本的单调时间。
    pub received_monotonic_ns: u64,
    /// 来源拥有方既有有效性策略推导的绝对截止。
    pub valid_until_monotonic_ns: u64,
    /// 适配器声明的受控参考点和轴系标识。
    pub frame_id: String,
    /// 同一样本的机体线速度，米/秒。
    pub body_linear_xyz: [f64; 3],
    /// 同一样本的机体角速度，弧度/秒；z 不等于一般姿态下的 yaw 导数。
    pub body_angular_xyz: [f64; 3],
    /// 同样本 body→设备世界姿态 wxyz；缺失时不能推导一般姿态 yaw 导数。
    pub orientation_wxyz: Option<[f64; 4]>,
}
impl MeasuredMotion {
    /// 检查原单调期限；不会延长来源有效时间。
    pub fn valid_at(&self, now: u64) -> bool {
        self.source_timestamp_ns > 0
            && self.received_monotonic_ns > 0
            && self.received_monotonic_ns <= now
            && now <= self.valid_until_monotonic_ns
            && !self.source_id.trim().is_empty()
            && !self.frame_id.trim().is_empty()
            && self
                .body_linear_xyz
                .iter()
                .chain(self.body_angular_xyz.iter())
                .all(|v| v.is_finite())
            && self.orientation_wxyz.is_none_or(|q| {
                q.iter().all(|v| v.is_finite())
                    && (0.9..=1.1).contains(&q.iter().map(|v| v * v).sum::<f64>().sqrt())
            })
    }
    /// 使用同宿主 CLOCK_MONOTONIC 检查；与设备 Unix 采样时间分开。
    pub fn is_current(&self) -> bool {
        self.valid_at(monotonic_ns())
    }
    pub(crate) fn from_wire(value: MeasuredMotionFeedback, now: u64) -> Option<Self> {
        let orientation_wxyz = if value.orientation_wxyz.is_empty() {
            None
        } else {
            Some(value.orientation_wxyz.try_into().ok()?)
        };
        let value = Self {
            source_id: value.source_id,
            source_timestamp_ns: value.source_timestamp_ns,
            received_monotonic_ns: value.received_monotonic_ns,
            valid_until_monotonic_ns: value.valid_until_monotonic_ns,
            frame_id: value.frame_id,
            body_linear_xyz: value.body_linear_xyz.try_into().ok()?,
            body_angular_xyz: value.body_angular_xyz.try_into().ok()?,
            orientation_wxyz,
        };
        value.valid_at(now).then_some(value)
    }
}

/// 同宿主真实单调时钟；异常值为0，不能产生有效测量。
pub fn monotonic_ns() -> u64 {
    let now = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    u64::try_from(now.tv_sec)
        .ok()
        .and_then(|s| s.checked_mul(1_000_000_000))
        .and_then(|s| {
            u64::try_from(now.tv_nsec)
                .ok()
                .and_then(|n| s.checked_add(n))
        })
        .unwrap_or(0)
}

#[derive(Default)]
pub(crate) struct MeasurementCache {
    value: Option<MeasuredMotion>,
    latest: Option<MeasuredMotion>,
}
impl MeasurementCache {
    pub(crate) fn update(&mut self, wire: Option<MeasuredMotionFeedback>, now: u64) {
        let value = wire.and_then(|v| MeasuredMotion::from_wire(v, now));
        self.value = value.filter(|v| {
            self.latest.as_ref().is_none_or(|old| {
                old.source_id != v.source_id
                    || v.source_timestamp_ns > old.source_timestamp_ns
                    || (v.source_timestamp_ns == old.source_timestamp_ns && v == old)
            })
        });
        if let Some(value) = &self.value {
            self.latest = Some(value.clone());
        }
    }
    pub(crate) fn current(&self) -> Option<MeasuredMotion> {
        self.value.as_ref().filter(|v| v.is_current()).cloned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn wire() -> MeasuredMotionFeedback {
        MeasuredMotionFeedback {
            source_id: "device:sport".into(),
            source_timestamp_ns: 10,
            received_monotonic_ns: 100,
            valid_until_monotonic_ns: 200,
            frame_id: "base_link".into(),
            body_linear_xyz: vec![0., 0., 0.],
            body_angular_xyz: vec![0., 0., 0.],
            orientation_wxyz: vec![1., 0., 0., 0.],
        }
    }
    #[test]
    fn measured_zero_is_present_but_placeholder_missing_axes_is_unknown() {
        assert!(MeasuredMotion::from_wire(wire(), 150).is_some());
        let mut missing = wire();
        missing.body_linear_xyz.clear();
        assert!(MeasuredMotion::from_wire(missing, 150).is_none());
    }
    #[test]
    fn stale_future_repeat_and_regression_never_renew_measurement() {
        assert!(MeasuredMotion::from_wire(wire(), 99).is_none());
        assert!(MeasuredMotion::from_wire(wire(), 201).is_none());
        let mut cache = MeasurementCache::default();
        cache.update(Some(wire()), 150);
        cache.update(Some(wire()), 180);
        assert_eq!(cache.value.as_ref().unwrap().valid_until_monotonic_ns, 200);
        let mut renewed = wire();
        renewed.valid_until_monotonic_ns = 300;
        cache.update(Some(renewed), 190);
        assert!(cache.value.is_none());
        cache.update(None, 190);
        let mut older = wire();
        older.source_timestamp_ns = 9;
        cache.update(Some(older), 190);
        assert!(cache.value.is_none());
        let mut fresh = wire();
        fresh.source_timestamp_ns = 11;
        fresh.received_monotonic_ns = 190;
        fresh.valid_until_monotonic_ns = 290;
        cache.update(Some(fresh), 195);
        assert!(cache.value.is_some());
    }
    #[test]
    fn optional_snapshot_roundtrips_independently_on_error_response() {
        use prost::Message;
        use crate::wire::{MotionGuardResponse,motion_guard_response,GetReadinessResponse,GuardResult,ErrorDetail};
        let response=MotionGuardResponse {api_version:crate::wire::API_VERSION,request_id:"rejected-command".into(),
            measured_motion_feedback:Some(wire()),
            outcome:Some(motion_guard_response::Outcome::GetReadiness(GetReadinessResponse {
                result:Some(GuardResult{error:Some(ErrorDetail{code:"REJECTED".into(),message:"not applied".into(),retryable:false}),..Default::default()}),
                ..Default::default()}))};
        let decoded=MotionGuardResponse::decode(response.encode_to_vec().as_slice()).unwrap();
        let mut cache=MeasurementCache::default();cache.update(decoded.measured_motion_feedback,150);
        assert_eq!(cache.value.unwrap().body_linear_xyz,[0.;3]);
        let old=MotionGuardResponse{measured_motion_feedback:None,..response};
        assert!(MotionGuardResponse::decode(old.encode_to_vec().as_slice()).unwrap().measured_motion_feedback.is_none());
    }

}
