//! A single admitted world-frame scan and its frozen, paired registration pose.
use crate::CommandDispatchBound;
use std::sync::Arc;

/// Source domain. A change must retire the previous native occupancy history.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegisteredFrameDomain {
    /// Geometry 当前地图会话。
    pub session_id: String,
}

/// One source observation, not the current pose or a fused map revision.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegisteredFrameIdentity {
    /// Frozen source domain.
    pub domain: RegisteredFrameDomain,
    /// Source cloud sequence.
    pub sequence: i64,
    /// Original cloud acquisition time.
    pub timestamp_ns: i64,
}

/// Immutable native-map input. World points are never transformed again.
#[derive(Clone, Debug)]
pub struct RegisteredFrame {
    /// Exact source observation identity.
    pub identity: RegisteredFrameIdentity,
    /// Complete registered returns, including ground returns.
    pub points_world: Arc<[[f64; 3]]>,
    /// 原始发布者认定的同帧地面成员下标，不按当前高度重新推测。
    pub ground_point_indices: Arc<[u32]>,
    /// Same-frame calibrated sensor origin, never filled from latest odometry.
    pub sensor_origin_world: [f64; 3],
    /// Same-frame frozen body position.
    pub body_position_world: [f64; 3],
    /// Same-frame body quaternion for provenance; not sensor orientation.
    pub body_orientation_xyzw: [f64; 4],
    /// The odometry sample actually paired by the registration owner.
    /// Paired odometry acquisition time, distinct from cloud time.
    pub paired_odometry_timestamp_ns: i64,
    /// Existing source-owner deadline, not renewed by forwarding or replay.
    pub dispatch_bound: Option<CommandDispatchBound>,
}
impl RegisteredFrame {
    /// Reuse the existing source qualification without introducing a new TTL.
    pub fn is_current(&self) -> bool {
        self.dispatch_bound.is_none_or(|bound| !bound.expired_now())
    }
}

#[cfg(test)]
pub(crate) fn test_frame(sequence:i64) -> Arc<RegisteredFrame> {
    Arc::new(RegisteredFrame {
        identity:RegisteredFrameIdentity {
            domain:RegisteredFrameDomain {session_id:"registered-test".into()},
            sequence,timestamp_ns:1_000_000_000+sequence,
        },
        points_world:vec![[0.0,0.0,0.0],[1.0,0.0,0.0]].into(),
        ground_point_indices:Arc::from([]),
        sensor_origin_world:[0.5,1.0,0.5],body_position_world:[0.5,1.0,0.5],
        body_orientation_xyzw:[0.0,0.0,0.0,1.0],
        paired_odometry_timestamp_ns:900_000_000+sequence,dispatch_bound:None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn original_registered_source_deadline_is_not_receipt_freshness() {
        let mut frame=(*test_frame(1)).clone();
        frame.dispatch_bound=Some(CommandDispatchBound::from_source_timestamp(1,std::time::Duration::ZERO));
        assert!(!frame.is_current());
        assert!(!frame.clone().is_current());
        assert_ne!(frame.identity.timestamp_ns,frame.paired_odometry_timestamp_ns);
    }
}
