//! Shared bounded geometry-delta proofs for global and local planning owners.
use crate::SurfaceSnapshot;
use nuwax_nav_types_messages::Path3D;
use std::sync::Arc;
#[path = "surface_reference_scope.rs"]
mod reference_scope;
pub use reference_scope::ReferenceCorridor;

/// Maximum aggregate path, model and changed-column work for an advisory.
pub const MAX_REFERENCE_SCOPE_WORK: usize = 4096;

/// Raw changes relative to one complete source, including deleted columns.
#[derive(Debug)]
pub enum SurfaceGeometryChanges {
    /// Proven unchanged geometry (timestamps and other metadata may change).
    Empty,
    /// Complete fine and coarse column differences; heights are deliberately unbounded.
    Columns {
        /// Actual source resolution in metres.
        fine_size: f32,
        /// Changed source XY columns, including deletions.
        fine: Vec<[i32; 2]>,
        /// Existing planner resolution in metres.
        planner_size: f32,
        /// Changed planner XY columns, including deletions.
        coarse: Vec<[i32; 2]>,
    },
    /// Missing differences, semantic changes or exceeded work bounds.
    Unknown,
}

/// 将既有完整变更证据借给 PCT；不复制列，不从当前占据反推删除。
/// 基准与查询参数是否匹配由持有旧层析图的 PCT 拥有方再次核验。
pub fn pct_surface_delta<'a>(basis:Option<&'a SurfaceSnapshot>,changes:&'a SurfaceGeometryChanges)
    ->nuwax_nav_pct::SurfaceDelta<'a> {
    let Some(basis)=basis else {return nuwax_nav_pct::SurfaceDelta::Rebuild;};
    match changes {
        SurfaceGeometryChanges::Empty=>nuwax_nav_pct::SurfaceDelta::Unchanged {basis},
        SurfaceGeometryChanges::Columns {fine_size,fine,..}=>nuwax_nav_pct::SurfaceDelta::Columns {
            basis,resolution:f64::from(*fine_size),columns:fine.as_slice(),
        },
        SurfaceGeometryChanges::Unknown=>nuwax_nav_pct::SurfaceDelta::Rebuild,
    }
}

// Source owners classify map/relocation/mask changes as Unknown. Independently
// enforce model, frame and query-configuration semantics when coalescing too, so
// a transient change followed by a reversal cannot be lost before delivery.
pub(crate) fn same_query_semantics(a:&SurfaceSnapshot,b:&SurfaceSnapshot)->bool {
    fn parameters(s:&SurfaceSnapshot)->[u32;13] {
        let c=s.config();
        [c.voxel_size,c.robot_height,c.surface_closing_radius,c.node_spacing_m,
            c.wall_clearance_m,c.wall_buffer_m,c.wall_buffer_weight,c.step_threshold_m,
            c.max_slope_deg,c.min_step_tread_depth_m,c.step_penalty_weight,c.goal_tolerance,
            s.source_voxel_size_m()].map(f32::to_bits)
    }
    a.body_envelope()==b.body_envelope() && a.config().world_frame==b.config().world_frame
        && a.ground_evidence()==b.ground_evidence() && parameters(a)==parameters(b)
}

/// A map publication carries the exact baseline of its geometry differences.
#[derive(Clone)]
pub struct SurfaceMapUpdate {
    /// Latest immutable source for the execution guard.
    pub snapshot: Arc<SurfaceSnapshot>,
    /// Static planar query artifact prepared off the control thread for this exact source.
    /// Recent obstacle frames are intentionally checked on the live snapshot separately.
    pub planar_reference: Option<Arc<nuwax_nav_pct::Tomogram>>,
    /// Successful source immediately preceding these differences.
    pub change_basis: Option<Arc<SurfaceSnapshot>>,
    /// Complete geometry differences, never inferred from a revision number alone.
    pub changes: Arc<SurfaceGeometryChanges>,
}
impl SurfaceMapUpdate {
    /// Prepare a raw immutable surface before handing it to an execution owner.
    /// This can derive a tomogram and must run on the existing background map or
    /// planning owner, never inside update_execution_map or a controller tick.
    /// Callers with complete deltas may replace the default Unknown change proof.
    pub fn prepare(snapshot:Arc<SurfaceSnapshot>,planner:&nuwax_nav_pct::PctPlanner)->Result<Self,String> {
        let planar_reference=if snapshot.body_envelope().is_none() {Some(planner.map(&snapshot)?)} else {None};
        Ok(Self {snapshot,planar_reference,change_basis:None,changes:Arc::new(SurfaceGeometryChanges::Unknown)})
    }

    /// Coalesce undelivered publications without dropping intermediate deletions or additions.
    /// This only joins the existing bounded column sets; a gap remains Unknown.
    pub fn append(&mut self, next: Self) {
        let linked = next.change_basis.as_ref().is_some_and(|basis|
            nuwax_nav_pct::same_source_geometry(&self.snapshot, basis))
            && same_query_semantics(&self.snapshot,&next.snapshot);
        let merged = if !linked { Arc::new(SurfaceGeometryChanges::Unknown) } else {
            use SurfaceGeometryChanges::*;
            match (self.changes.as_ref(), next.changes.as_ref()) {
                (Unknown, _) | (_, Unknown) => Arc::new(Unknown),
                (Empty, _) => Arc::clone(&next.changes),
                (_, Empty) => Arc::clone(&self.changes),
                (Columns {fine_size:a, fine:af, planner_size:b, coarse:ac},
                 Columns {fine_size:c, fine:bf, planner_size:d, coarse:bc}) => {
                    let count=af.len().checked_add(ac.len()).and_then(|n|n.checked_add(bf.len()))
                        .and_then(|n|n.checked_add(bc.len()));
                    if a.to_bits()!=c.to_bits() || b.to_bits()!=d.to_bits()
                        || count.is_none_or(|n|n>MAX_REFERENCE_SCOPE_WORK) { Arc::new(Unknown) }
                    else { Arc::new(Columns {fine_size:*a, fine:af.iter().chain(bf).copied().collect(),
                        planner_size:*b, coarse:ac.iter().chain(bc).copied().collect()}) }
                }
            }
        };
        self.snapshot=next.snapshot;
        self.planar_reference=next.planar_reference;
        self.changes=merged;
    }
}

/// Existing job-ticket proof, shared by ON and OFF without a second job lifecycle.
/// Empty publications advance its basis; changed batches retain map-owned Arcs.
#[derive(Clone)]
pub struct SurfaceJobTicket {
    /// Most recent completely observed source; None disables selective acceptance.
    pub basis: Option<Arc<SurfaceSnapshot>>,
    /// Every nonempty delta since submission, including changes later reversed.
    pub changes: Vec<Arc<SurfaceGeometryChanges>>,
    /// Total retained column work, counted conservatively with duplicates.
    pub columns: usize,
    /// An absorbing gap or semantic invalidation.
    pub unknown: bool,
}
impl SurfaceJobTicket {
    /// Start a proof at the job's actual source. None conservatively disables it.
    pub fn new(source: Option<Arc<SurfaceSnapshot>>) -> Self {
        Self {basis:source, changes:Vec::new(), columns:0, unknown:false}
    }
    /// Observe the complete next publication before testing any candidate.
    pub fn observe(&mut self, current: &Arc<SurfaceSnapshot>, previous: Option<&SurfaceSnapshot>,
        changes: &Arc<SurfaceGeometryChanges>) {
        if self.unknown { return; }
        let Some(basis)=self.basis.as_ref() else {return;};
        if previous.is_none_or(|old|!nuwax_nav_pct::same_source_geometry(basis,old))
            || !same_query_semantics(basis,current) {
            self.unknown=true;return;
        }
        match changes.as_ref() {
            SurfaceGeometryChanges::Empty => {},
            SurfaceGeometryChanges::Columns {fine,coarse,..} => {
                let count=fine.len().checked_add(coarse.len()).and_then(|n|n.checked_add(self.columns));
                if count.is_none_or(|n|n>MAX_REFERENCE_SCOPE_WORK) {self.unknown=true;return;}
                self.columns=count.expect("bounded column count");
                if !fine.is_empty() || !coarse.is_empty() {self.changes.push(Arc::clone(changes));}
            }
            SurfaceGeometryChanges::Unknown => {self.unknown=true;return;}
        }
        self.basis=Some(Arc::clone(current));
    }
    /// False includes Unknown. This proof never replaces the latest execution guard.
    pub fn permits_path(&self, latest:&SurfaceSnapshot, path:&Path3D)->bool {
        if self.unknown || self.basis.as_ref().is_none_or(|basis|
            !nuwax_nav_pct::same_source_geometry(basis,latest)) {return false;}
        // A continuous chain containing no spatial changes needs no corridor;
        // admission still validates the candidate against the latest guard.
        if self.changes.is_empty() {return true;}
        let Some(model)=latest.body_envelope() else {return false;};
        let work=model.footprint_xy_m.len().checked_add(path.poses.len())
            .and_then(|n|n.checked_add(model.fixed_collision.as_ref().map_or(0,|g|g.primitives.len())))
            .and_then(|n|n.checked_add(self.columns));
        if work.is_none_or(|n|n>MAX_REFERENCE_SCOPE_WORK) {return false;}
        let Some(corridor)=ReferenceCorridor::new(latest,path) else {return false;};
        self.changes.iter().all(|change|match change.as_ref() {
            SurfaceGeometryChanges::Empty=>true,
            SurfaceGeometryChanges::Unknown=>false,
            SurfaceGeometryChanges::Columns {fine_size,fine,planner_size,coarse}=>
                !corridor.intersects_columns(*fine_size,fine) && !corridor.intersects_columns(*planner_size,coarse),
        })
    }
}
