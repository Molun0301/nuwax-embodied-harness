//! 已加载三维地图的逐秒跟踪；会话停止并等待线程后交还变换写入权。
use super::FrameRouter;
use crate::mapper::{pose_from_isometry, pose_isometry, Mapper, MappingFrame};
use nalgebra::{
    Isometry3, Matrix3, Point3, SMatrix, SVector, Translation3, UnitQuaternion, Vector3,
};
use nuwax_mapping::relocalization::{
    score_relocalization, NearestPointIndex, RelocalizationScore, SourceCloud, TargetCache,
};
use std::{
    sync::{mpsc as sync_channel, Arc},
    thread,
    time::{Duration, Instant},
};
use tokio::sync::mpsc;

pub(super) struct TrackingUpdate {
    pub score: RelocalizationScore,
    pub bad_rounds: u32,
    pub timestamp_ns: u64,
}

pub(super) struct Localizer {
    stop: sync_channel::Sender<()>,
    worker: thread::JoinHandle<()>,
    updates: mpsc::UnboundedReceiver<TrackingUpdate>,
}
impl Localizer {
    pub fn start(router: FrameRouter, mapper: Arc<Mapper>, target: Arc<TargetCache>) -> Self {
        let (stop, stopped) = sync_channel::channel();
        let (sender, updates) = mpsc::unbounded_channel();
        let runtime = tokio::runtime::Handle::current();
        let worker = thread::spawn(move || {
            let mut bad_rounds = 0;
            let mut next = Instant::now() + Duration::from_secs(1);
            loop {
                match stopped.recv_timeout(next.saturating_duration_since(Instant::now())) {
                    Ok(()) | Err(sync_channel::RecvTimeoutError::Disconnected) => break,
                    Err(sync_channel::RecvTimeoutError::Timeout) => {}
                }
                // 每轮最多使用最近两秒的已有帧；没有帧不产生质量采样。
                next = Instant::now() + Duration::from_secs(1);
                let Some(round) =
                    compute_round(&router.recent_frames(), mapper.map_from_odom(), &target)
                else {
                    continue;
                };
                if let Some(transform) = round.correction {
                    runtime.block_on(router.publish_map_from_odom(
                        mapper.session_id(),
                        pose_from_isometry(transform, mapper.map_frame_id(), super::now_ns()),
                    ));
                    bad_rounds = 0;
                } else {
                    bad_rounds += 1;
                }
                let _ = sender.send(TrackingUpdate {
                    score: round.score,
                    bad_rounds,
                    timestamp_ns: super::now_ns(),
                });
            }
        });
        Self {
            stop,
            worker,
            updates,
        }
    }
    pub async fn next(&mut self) -> Option<TrackingUpdate> {
        self.updates.recv().await
    }
    pub async fn stop(self) {
        let _ = self.stop.send(());
        tokio::task::spawn_blocking(move || self.worker.join().expect("跟踪线程应正常退出"))
            .await
            .expect("跟踪线程回收任务应正常完成");
    }
}

pub(super) struct TrackingRound {
    pub correction: Option<Isometry3<f64>>,
    pub score: RelocalizationScore,
    pub source_points: usize,
    pub target_points: usize,
    pub registration_ms: f64,
    pub scoring_ms: f64,
}

pub(super) fn compute_round(
    frames: &[MappingFrame],
    initial: Isometry3<f64>,
    target: &TargetCache,
) -> Option<TrackingRound> {
    let last = frames.last()?;
    let source = SourceCloud::from_rays(frames.iter().flat_map(|frame| {
        let odom = pose_isometry(&frame.odom_pose);
        let origin = odom * Point3::from(frame.ray_origin_body);
        frame
            .points_body
            .iter()
            .map(move |p| (odom * Point3::from(*p), origin))
    }));
    let odom = pose_isometry(&last.odom_pose);
    let p0 = initial * odom;
    let started = Instant::now();
    let local: Vec<_> = target
        .points
        .iter()
        .filter(|p| {
            (p.x - p0.translation.x).powi(2) + (p.y - p0.translation.y).powi(2) <= 30.0_f64.powi(2)
        })
        .copied()
        .collect();
    let index = NearestPointIndex::new(&local);
    let candidate = point_to_plane(&source, &local, &index, initial, p0.translation.vector);
    let registration_ms = started.elapsed().as_secs_f64() * 1000.0;
    let started = Instant::now();
    let score = score_relocalization(&source, candidate, target);
    let scoring_ms = started.elapsed().as_secs_f64() * 1000.0;
    let correction = score
        .accepts(&candidate)
        .then(|| constrained_step(&source, target, initial, candidate, odom));
    Some(TrackingRound {
        correction,
        score,
        source_points: source.points.len(),
        target_points: local.len(),
        registration_ms,
        scoring_ms,
    })
}

// 点到平面线性最小二乘使用奇异值分解；零奇异方向解为零，不把退化当作拒绝条件。
fn point_to_plane(
    source: &SourceCloud,
    target: &[Point3<f64>],
    index: &NearestPointIndex,
    initial: Isometry3<f64>,
    pivot: Vector3<f64>,
) -> Isometry3<f64> {
    let mut transform = initial;
    for _ in 0..20 {
        let mut information = SMatrix::<f64, 6, 6>::zeros();
        let mut rhs = SVector::<f64, 6>::zeros();
        for (point, normal) in source.points.iter().zip(&source.normals) {
            let point = transform * point;
            let Some((i, distance)) = index.nearest(&point) else {
                continue;
            };
            if distance > 0.3_f64.powi(2) {
                continue;
            }
            let normal = transform.rotation * normal;
            let rotation_row = (point.coords - pivot).cross(&normal);
            let row = SVector::<f64, 6>::new(
                normal.x,
                normal.y,
                normal.z,
                rotation_row.x,
                rotation_row.y,
                rotation_row.z,
            );
            information += row * row.transpose();
            rhs += row * normal.dot(&(target[i] - point));
        }
        let svd = information.svd(true, true);
        let tolerance = f64::EPSILON * 6.0 * svd.singular_values.max();
        let step = svd
            .solve(&rhs, tolerance)
            .expect("奇异值分解已请求左右奇异向量");
        let translation = Vector3::new(step[0], step[1], step[2]);
        let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(step[3], step[4], step[5]));
        let delta = Isometry3::from_parts(
            Translation3::from(pivot + translation - rotation * pivot),
            rotation,
        );
        transform = delta * transform;
    }
    transform
}

fn constrained_step(
    source: &SourceCloud,
    target: &TargetCache,
    initial: Isometry3<f64>,
    candidate: Isometry3<f64>,
    odom: Isometry3<f64>,
) -> Isometry3<f64> {
    let mut information = Matrix3::zeros();
    let mut inliers = 0;
    for (point, normal) in source.points.iter().zip(&source.normals) {
        if target
            .index
            .nearest(&(candidate * point))
            .is_some_and(|(_, d)| d <= 0.15_f64.powi(2))
        {
            let normal = candidate.rotation * normal;
            information += normal * normal.transpose();
            inliers += 1;
        }
    }
    let p0 = initial * odom;
    let p1 = candidate * odom;
    let delta = p1.translation.vector - p0.translation.vector;
    let eigen = information.symmetric_eigen();
    let mut constrained = Vector3::zeros();
    for i in 0..3 {
        if eigen.eigenvalues[i] >= inliers as f64 * 0.05 {
            let direction = eigen.eigenvectors.column(i);
            constrained += direction * direction.dot(&delta);
        }
    }
    let translation = p0.translation.vector + constrained * (0.02 / constrained.norm()).min(1.0);
    let angle = (p0.rotation.inverse() * p1.rotation).angle();
    let rotation = p0
        .rotation
        .slerp(&p1.rotation, (0.2_f64.to_radians() / angle).min(1.0));
    Isometry3::from_parts(Translation3::from(translation), rotation) * odom.inverse()
}

#[cfg(test)]
#[path = "localizer_tests.rs"]
mod tests;
