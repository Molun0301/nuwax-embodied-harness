// Copyright 2026 Nuwax Inc.
// SPDX-License-Identifier: Apache-2.0

//! Dependency-free debug-event adapter for planner visualization artifacts.

use crate::transformer::{DebugEdge, PlanObservation};

/// Visualization-neutral planner event.
#[derive(Clone, Debug, PartialEq)]
pub enum DebugEvent {
    /// Planner start point.
    Start((f32, f32, f32)),
    /// Occupied voxel centers.
    VoxelMap(Vec<(f32, f32, f32)>),
    /// Standable cells and clearance.
    SurfaceClearance(Vec<((f32, f32, f32), f32)>),
    /// Sparse graph nodes.
    Nodes(Vec<(f32, f32, f32)>),
    /// Expanded graph edges.
    NodeEdges(Vec<DebugEdge>),
    /// Planned path, empty on failure.
    Path(Vec<(f32, f32, f32)>),
    /// Stable numeric metrics for logging or visualization adapters.
    Metrics {
        /// Region update duration in milliseconds.
        update_ms: f64,
        /// Plan duration in milliseconds.
        plan_ms: f64,
        /// Total duration in milliseconds.
        total_ms: f64,
        /// Occupied voxel count.
        voxels: usize,
        /// Whether a route was found.
        planned: bool,
    },
}

/// Convert a transformer output into a complete ordered event batch.
#[must_use]
pub fn debug_events(output: &PlanObservation) -> Vec<DebugEvent> {
    vec![
        DebugEvent::Start(output.start),
        DebugEvent::VoxelMap(output.artifacts.voxel_map.clone()),
        DebugEvent::SurfaceClearance(output.artifacts.surface_clearance.clone()),
        DebugEvent::Nodes(output.artifacts.nodes.clone()),
        DebugEvent::NodeEdges(output.artifacts.node_edges.clone()),
        DebugEvent::Path(output.path.clone()),
        DebugEvent::Metrics {
            update_ms: output.timings.update_ms,
            plan_ms: output.timings.plan_ms,
            total_ms: output.timings.total_ms,
            voxels: output.voxels,
            planned: output.planned,
        },
    ]
}

/// Consumer boundary for CLI, JSON, Rerun or other debug backends.
pub trait DebugEventSink {
    /// Consume one event.
    fn emit(&mut self, event: DebugEvent);
}

/// Emit all events for one output in stable order.
pub fn emit_debug_events(output: &PlanObservation, sink: &mut impl DebugEventSink) {
    for event in debug_events(output) {
        sink.emit(event);
    }
}
