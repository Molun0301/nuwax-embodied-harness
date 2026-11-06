//! 二维扫描在物理图上形成局部导航观测；真实采样时间和原生匹配依据分别保留。
mod local_observation;
mod rays;
use crate::{
    mapper::{proto_grid_to_kernel, Mapper},
    slam2d_owner::Settings,
};
use nuwax_codec::lcm::{
    planar::PlanarScan,
    planar_navigation::{self, PlanarNavigationMap},
    schema,
};
use nuwax_slam_2d_runtime::{
    input::{navigation_rays, PoseHistory, TimedPose},
    protocol::{Estimate, OdometrySource, Session},
};
use std::{
    collections::VecDeque,
    sync::atomic::{AtomicI64, Ordering},
};

#[derive(Default)]
pub(crate) struct Owner {
    scans: parking_lot::Mutex<VecDeque<PlanarScan>>,
    history: parking_lot::Mutex<PoseHistory>,
    local: parking_lot::Mutex<local_observation::LocalObservation>,
    revision: AtomicI64,
}
impl Owner {
    pub(crate) fn clear(&self) {
        self.scans.lock().clear();
        self.history.lock().clear();
        *self.local.lock() = Default::default();
    }
    pub(crate) fn capture(&self, scan: PlanarScan) {
        let mut scans = self.scans.lock();
        scans.push_back(scan);
        while scans.len() > 16 {
            scans.pop_front();
        }
    }
    pub(crate) fn odometry(&self, sample: TimedPose) -> Result<(), String> {
        self.history.lock().push(sample)
    }
    fn source(
        &self,
        session: &Session,
        basis: &Estimate,
    ) -> Result<Option<(PlanarScan, [f64; 3])>, String> {
        if !matches!(
            basis.disposition.as_str(),
            "initialized" | "accepted" | "motion_skipped"
        ) {
            return Ok(None);
        }
        let scans = self.scans.lock();
        if session.odometry_source == OdometrySource::Laser {
            return Ok(scans
                .iter()
                .find(|scan| {
                    scan.source_sequence as u64 == basis.source_sequence
                        && scan.timestamp_ns as u64 == basis.source_timestamp_ns
                })
                .map(|scan| (scan.clone(), basis.odom_from_base)));
        }
        let history = self.history.lock();
        for scan in scans.iter().rev() {
            if (scan.timestamp_ns as u64) < basis.source_timestamp_ns
                || scan.timestamp_ns as u64 - basis.source_timestamp_ns > 500_000_000
            {
                continue;
            }
            if let Some(pose) =
                history.at(scan.timestamp_ns as u64, &scan.clock_domain, &scan.clock_id)?
            {
                return Ok(Some((scan.clone(), pose)));
            }
        }
        Ok(None)
    }
    pub(crate) fn compose(
        &self,
        mapper: &Mapper,
        settings: &Settings,
        session: &Session,
        basis: &Estimate,
    ) -> Result<Option<PlanarNavigationMap>, String> {
        let Some((scan, odom_from_base)) = self.source(session, basis)? else {
            return Ok(None);
        };
        let stamp = settings.unix_stamp(scan.timestamp_ns as u64)? as i64;
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos() as i64;
        if stamp > now + 50_000_000
            || now.saturating_sub(stamp) > planar_navigation::MAX_SOURCE_AGE_NS
        {
            return Ok(None);
        }
        if mapper.forbidden_status().is_err() {
            return Ok(None);
        }
        let Some(raw) = mapper.occupancy(false) else {
            return Ok(None);
        };
        let grid = proto_grid_to_kernel(&raw)?;
        let history = self.history.lock();
        let Some(rays) = navigation_rays(
            &scan,
            if session.odometry_source == OdometrySource::Device {
                Some(&history)
            } else {
                None
            },
            settings.base_from_laser,
        )?
        else {
            return Ok(None);
        };
        drop(history);
        let odom = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(odom_from_base[0], odom_from_base[1], 0.),
            nalgebra::UnitQuaternion::from_euler_angles(0., 0., odom_from_base[2]),
        );
        let pose = mapper.map_from_odom() * odom;
        let mut local = self.local.lock().compose(
            &grid,
            &rays,
            [
                pose.translation.x,
                pose.translation.y,
                pose.rotation.euler_angles().2,
            ],
        )?;
        if let Some(mask) = mapper.forbidden_mask() {
            for (cell, forbidden) in local.data.iter_mut().zip(mask.data) {
                if forbidden == 100 {
                    *cell = 100;
                }
            }
        }
        // 旧线协议字段只在编码边界填必需值，不建立另一套代际或提交收据。
        let source = raw.meta.as_ref().unwrap().timestamp_ns as i64;
        Ok(Some(PlanarNavigationMap {
            runtime_generation: 1,
            session_id: session.session_id.clone(),
            session_generation: 1,
            map_frame_generation: 0,
            map_revision: source,
            map_boot_nonce: [1; 16],
            map_commit_sequence: source,
            map_source_timestamp_ns: source,
            observation_revision: self.revision.fetch_add(1, Ordering::Relaxed) + 1,
            source_sequence: scan.source_sequence,
            source_timestamp_ns: stamp,
            cleared: false,
            grid: schema::OccupancyGrid {
                header: schema::Header {
                    stamp_sec: (stamp / 1_000_000_000) as i32,
                    stamp_nsec: (stamp % 1_000_000_000) as i32,
                    frame_id: local.frame_id,
                    ..Default::default()
                },
                info: schema::MapMetaData {
                    resolution: local.resolution as f32,
                    width: local.width as i32,
                    height: local.height as i32,
                    origin: schema::Pose {
                        position: local.origin,
                        orientation_xyzw: [0., 0., 0., 1.],
                    },
                    ..Default::default()
                },
                data: local.data,
            },
        }))
    }
    pub(crate) fn revoke(&self, session: &Session) -> PlanarNavigationMap {
        PlanarNavigationMap {
            runtime_generation: 1,
            session_id: session.session_id.clone(),
            session_generation: 1,
            observation_revision: self.revision.fetch_add(1, Ordering::Relaxed) + 1,
            cleared: true,
            ..Default::default()
        }
    }
}
pub(crate) async fn publish(
    transport: &nuwax_transport::LcmTransport,
    value: &PlanarNavigationMap,
) -> Result<(), String> {
    use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
    let channel = std::env::var("NUWAX_LCM_CHANNEL_PLANAR_NAVIGATION").unwrap_or_else(|_| {
        format!(
            "{}#{}",
            planar_navigation::DEFAULT_TOPIC,
            planar_navigation::TYPE_NAME
        )
    });
    let topic = channel.split('#').next().unwrap();
    let bytes = planar_navigation::encode_body(value).map_err(|e| e.to_string())?;
    transport
        .publish(
            RawLcmFrameBuilder::new(
                LcmTopic::new(topic).map_err(|e| e.to_string())?,
                LcmMessageType::new(planar_navigation::TYPE_NAME).map_err(|e| e.to_string())?,
            )
            .payload(bytes)
            .build(),
        )
        .await
        .map_err(|e| e.to_string())
}
#[cfg(test)]
#[path = "planar_navigation_tests.rs"]
mod tests;
