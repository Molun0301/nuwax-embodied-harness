//! Fixed-size evidence from the original body-check rejection, never a new permission.
use super::SurfaceError;

/// Progress of the body's obstacle-exclusion pass, not a free-space certificate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionCheck { ConfirmedIntersection }
/// The exact short-circuit height comparison which rejected the selected evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum HeightPredicate { ContactBelowGround, BodyPlaneBelowGround }
/// Origin of an actual obstacle intersection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OccupancySource { CoarseFallback, Fine, FineFringe, Recent }
/// Pose-independent evidence. Missing values are deliberately not fabricated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BodyRejection {
    Occupancy { cell:[i32;3], source:OccupancySource, z_bounds:[f64;2], ground_height:Option<f64> },
}
impl BodyRejection {
    /// Exact pre-existing error variant for legacy callers and oracles.
    pub fn error(self)->SurfaceError {
        SurfaceError::InsufficientClearance
    }
    /// Only an explicit Occupancy result proves an obstacle intersection.
    pub fn collision_check(self)->CollisionCheck {
        CollisionCheck::ConfirmedIntersection
    }
}
/// One rejected sample from one query; it never describes all candidate paths.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyFailureWitness {
    pub pose: ([f64;3],[f64;4]),
    pub rejection: BodyRejection,
}
impl std::fmt::Display for BodyFailureWitness {
    fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result {
        write!(f,"拒绝样本 pose={:?}, collision={:?}, evidence={:?}",self.pose,self.rejection.collision_check(),self.rejection)
    }
}

// A disjoint cache payload avoids retaining a redundant SurfaceError beside a witness.
#[derive(Clone)]
pub(super) enum CachedBodyResult<T,W> { Ok(T), Error(SurfaceError), Witness(W) }
pub(super) trait WitnessError: Copy { fn error(self)->SurfaceError; }
impl WitnessError for BodyRejection {fn error(self)->SurfaceError {self.error()}}
impl WitnessError for BodyFailureWitness {fn error(self)->SurfaceError {self.rejection.error()}}
impl<T,W:WitnessError> CachedBodyResult<T,W> {
    pub(super) fn new(result:Result<T,SurfaceError>,witness:Option<W>)->Self {
        match result {
            Ok(value)=>Self::Ok(value),
            Err(error)=>match witness {Some(w)=>{debug_assert_eq!(error,w.error());Self::Witness(w)},None=>Self::Error(error)},
        }
    }
    pub(super) fn result(self,slot:&mut Option<W>)->Result<T,SurfaceError> {
        *slot=None;
        match self {Self::Ok(value)=>Ok(value),Self::Error(error)=>Err(error),
            Self::Witness(w)=>{*slot=Some(w);Err(w.error())}}
    }
}

pub(super) fn reject(slot:&mut Option<BodyRejection>,witness:BodyRejection)->SurfaceError {
    *slot=Some(witness);witness.error()
}
pub(super) fn height_rejection(lower:f64,support:f64,plane:f64,contact:f64,height:f64,uncertainty:f64)
    ->Option<(HeightPredicate,[f64;2])> {
    let left=lower.max(support)+contact+1e-6;
    if left<height {Some((HeightPredicate::ContactBelowGround,[left,height]))}
    else {let right=height-uncertainty;
        (plane<right).then_some((HeightPredicate::BodyPlaneBelowGround,[plane,right]))}
}
