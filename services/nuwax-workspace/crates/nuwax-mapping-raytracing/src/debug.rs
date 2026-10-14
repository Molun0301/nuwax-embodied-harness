//! Dependency-free debug-event runtime for ray-tracing comparisons.
//!
//! Dimos visualizes four mapper configurations through Rerun. This module
//! preserves the executable comparison and all visualization data as stable,
//! serializable events. A Rerun or other UI adapter can consume these events
//! without becoming the mapping owner.

use nalgebra::Vector3;
use serde::Serialize;
use std::collections::VecDeque;

use crate::transformer::{Point3, RayTraceError, RayTraceFrame, RayTraceMap};
use crate::voxel_ray_tracer::Config;

const RECENCY_OFF: u32 = 1_000_000_000;
const MAX_DEBUG_TRAJECTORY_POINTS: usize = 4_096;

/// One fitted surface normal at a healthy voxel center.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DebugNormal {
    /// Voxel center in the world frame.
    pub origin: Point3,
    /// Unit normal, oriented toward the robot when non-zero.
    pub vector: [f32; 3],
}

/// Global-map state for one comparison variant.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RayTraceVariantEvent {
    /// Stable variant identifier.
    pub name: &'static str,
    /// Stable visualization base color.
    pub color_rgb: [u8; 3],
    /// Healthy world-frame voxel centers.
    pub world_points: Vec<Point3>,
    /// Fitted normals for variants whose normal gate is meaningful.
    pub normals: Vec<DebugNormal>,
}

/// One cadence-aligned comparison event.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RayTraceDebugEvent {
    /// Source timestamp.
    pub timestamp: f64,
    /// Number of posed frames processed.
    pub frame_count: u64,
    /// Robot translation in the world frame.
    pub robot_position: Point3,
    /// Robot quaternion in XYZW order.
    pub robot_rotation_xyzw: (f32, f32, f32, f32),
    /// Original sensor-frame points for this cadence event.
    pub sensor_points: Vec<Point3>,
    /// Robot trajectory through this event.
    pub trajectory: Vec<Point3>,
    /// Mapper variants in stable display order.
    pub variants: Vec<RayTraceVariantEvent>,
}

struct Variant {
    name: &'static str,
    color_rgb: [u8; 3],
    emit_normals: bool,
    runtime: RayTraceMap,
}

/// Runs the Dimos naive/no-normal/default/no-recency comparison in one owner.
pub struct RayTraceComparison {
    variants: Vec<Variant>,
    trajectory: VecDeque<Point3>,
    last_frame: Option<RayTraceFrame>,
}

impl RayTraceComparison {
    /// Construct all four variants from a common base configuration.
    pub fn new(base: &Config) -> Result<Self, RayTraceError> {
        let mut naive = copy_config(base);
        naive.shadow_depth = 0.0;
        naive.grace_depth = base.max_range;
        naive.min_health = 0;

        let mut no_normal_gate = copy_config(base);
        no_normal_gate.graze_cos = 0.0;

        let defaults = copy_config(base);

        let mut no_recency = copy_config(base);
        no_recency.recency_window = RECENCY_OFF;

        Ok(Self {
            variants: vec![
                Variant {
                    name: "naive",
                    color_rgb: [90, 200, 90],
                    emit_normals: false,
                    runtime: RayTraceMap::new(naive)?,
                },
                Variant {
                    name: "no_normal_gate",
                    color_rgb: [235, 120, 60],
                    emit_normals: false,
                    runtime: RayTraceMap::new(no_normal_gate)?,
                },
                Variant {
                    name: "defaults",
                    color_rgb: [70, 170, 235],
                    emit_normals: true,
                    runtime: RayTraceMap::new(defaults)?,
                },
                Variant {
                    name: "no_recency",
                    color_rgb: [200, 80, 200],
                    emit_normals: true,
                    runtime: RayTraceMap::new(no_recency)?,
                },
            ],
            trajectory: VecDeque::new(),
            last_frame: None,
        })
    }

    /// Process one sensor frame and emit only on the configured cadence.
    pub fn process(
        &mut self,
        frame: RayTraceFrame,
    ) -> Result<Option<RayTraceDebugEvent>, RayTraceError> {
        let prepared = self
            .variants
            .iter()
            .map(|variant| variant.runtime.prepare_frame(frame.clone()))
            .collect::<Result<Vec<_>, _>>()?;
        if frame.pose.is_none() {
            debug_assert!(prepared.iter().all(Option::is_none));
            return Ok(None);
        }

        let mut cadence = None;
        for (variant, prepared) in self.variants.iter_mut().zip(prepared) {
            let output = variant
                .runtime
                .apply_prepared(prepared.expect("posed comparison frame was prepared"));
            let emitted = output.is_some();
            if let Some(expected) = cadence {
                debug_assert_eq!(emitted, expected, "comparison cadence diverged");
            } else {
                cadence = Some(emitted);
            }
        }

        let pose = frame.pose.expect("posed frame was checked above");
        if self.trajectory.len() == MAX_DEBUG_TRAJECTORY_POINTS {
            self.trajectory.pop_front();
        }
        self.trajectory.push_back(pose.translation);
        self.last_frame = Some(frame);
        if cadence == Some(true) {
            return Ok(Some(self.build_event()));
        }
        Ok(None)
    }

    /// Flush a final partial batch. Repeated calls without new input are empty.
    pub fn finish(&mut self) -> Option<RayTraceDebugEvent> {
        let mut emitted = false;
        for variant in &mut self.variants {
            emitted |= variant.runtime.finish().is_some();
        }
        emitted.then(|| self.build_event())
    }

    fn build_event(&self) -> RayTraceDebugEvent {
        let frame = self
            .last_frame
            .as_ref()
            .expect("an emitted comparison always has a posed frame");
        let pose = frame.pose.expect("last comparison frame is posed");
        let variants = self
            .variants
            .iter()
            .map(|variant| {
                let snapshot = variant.runtime.global_snapshot();
                let world_points = snapshot.iter().map(|(point, _)| *point).collect();
                let normals = if variant.emit_normals {
                    orient_normals(snapshot, pose.translation)
                } else {
                    Vec::new()
                };
                RayTraceVariantEvent {
                    name: variant.name,
                    color_rgb: variant.color_rgb,
                    world_points,
                    normals,
                }
            })
            .collect();
        RayTraceDebugEvent {
            timestamp: frame.timestamp,
            frame_count: self.variants[0].runtime.frame_count(),
            robot_position: pose.translation,
            robot_rotation_xyzw: pose.rotation_xyzw,
            sensor_points: frame.points.clone(),
            trajectory: self.trajectory.iter().copied().collect(),
            variants,
        }
    }
}

fn copy_config(config: &Config) -> Config {
    Config {
        voxel_size: config.voxel_size,
        max_range: config.max_range,
        ray_subsample: config.ray_subsample,
        shadow_depth: config.shadow_depth,
        grace_depth: config.grace_depth,
        min_health: config.min_health,
        max_health: config.max_health,
        graze_cos: config.graze_cos,
        recency_window: config.recency_window,
        emit_every: config.emit_every,
        global_emit_every: config.global_emit_every,
        region_percentile: config.region_percentile,
    }
}

fn orient_normals(snapshot: Vec<(Point3, [f32; 3])>, robot: Point3) -> Vec<DebugNormal> {
    let robot = Vector3::new(robot.0, robot.1, robot.2);
    snapshot
        .into_iter()
        .map(|(origin, normal)| {
            let origin_vector = Vector3::new(origin.0, origin.1, origin.2);
            let mut vector = Vector3::from(normal);
            if vector.dot(&(robot - origin_vector)) < 0.0 {
                vector = -vector;
            }
            DebugNormal {
                origin,
                vector: [vector.x, vector.y, vector.z],
            }
        })
        .collect()
}
