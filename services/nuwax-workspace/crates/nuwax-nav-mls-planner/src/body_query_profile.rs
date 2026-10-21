//! Opt-in wall-time attribution for body projection and contact queries.
//!
//! Available only with the default-disabled `body-query-profile` feature. All
//! storage is fixed-size and thread-local; only the thread calling [`begin`]
//! records work. Nested stages are inclusive and must not be summed as disjoint
//! cost. Durations are elapsed `Instant` time, including scheduling/preemption,
//! not thread CPU time. Stage entry/drop and counter updates perform no
//! formatting, allocation, hashing or locking. Format the returned [`Record`]
//! only after the measured interval has ended.

use std::cell::RefCell;
use std::marker::PhantomData;
use std::rc::Rc;
use std::time::Instant;

/// Fixed stages of the existing body-query implementation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(usize)]
pub enum Stage {
    /// Complete `body_pose_after_motion` call, including early returns.
    MotionTotal,
    /// Locate the current body's support.
    CurrentSupportLookup,
    /// Locate support near the initial body-centre projection.
    BaseNearSupportLookup,
    /// Locate support near the requested body-centre projection.
    NextNearSupportLookup,
    /// Complete contact-frame call, inclusive of cache/domain/fit work.
    ContactFrame,
    /// Original coarse segment-support query, excluding its fallback handling.
    SegmentSupport,
    /// Final body validation on normal, identity or fallback motion paths.
    FinalBodyValidation,
    /// Contact-plane cache lookup, including the existing lock scope.
    PlaneCacheLookup,
    /// Collect contact points, including domain setup and temporary cleanup.
    /// Dense substages are partial: table/queue setup, the seed-presence check,
    /// and retained-buffer cleanup remain here outside those substages.
    ContactDomainCollection,
    /// Sort the collected contact points in the original order.
    ContactPointSort,
    /// Fit the contact plane, including fit-local temporary cleanup.
    ContactPlaneFit,
    /// Construct the final contact frame from its plane and requested yaw.
    ContactFrameAlgebra,
    /// Dense batch band-cache read, including the existing lock and its release.
    DenseBandCacheRead,
    /// Dense band-selection and seed-ranking loop, including cache-miss queries
    /// and per-column Arc cleanup, but excluding selected-table allocation.
    DenseBandSelectionSeed,
    /// Dense BFS connectivity loop, excluding queue and visited-table setup.
    DenseConnectivityBfs,
    /// Dense ordered point emission, including output growth and consumed
    /// visited-table cleanup, but excluding other retained-buffer cleanup.
    DenseOrderedEmission,
    /// Acquire the existing dense batch-read cache lock, including poison recovery.
    DenseBandCacheAcquire,
    /// Complete known-terrain validation, including early errors.
    KnownTerrainTotal,
    /// Existing common-band helper, including early None; excludes caller Vec cleanup.
    KnownCommonBand,
    /// Existing tread helper, including early None; excludes caller Vec cleanup.
    KnownTreadGeometry,
    /// Existing known-only fallback after both geometry proofs miss.
    KnownOnlyFallback,
    /// Only actual known-terrain calls from normal or estimated projection.
    ProjectedKnownTerrain,
    /// Only the actual known-terrain call inside the checked sweep.
    SweptKnownTerrain,
    /// Uncached check_body_pose kernel, including early returns and local cleanup.
    /// Nested inside its caller; not exclusive of Scan project/sweep or final validation.
    PoseCoreTotal,
    /// Kernel entry through support/shape/patch/refine setup, inclusive on early errors.
    /// Normal-path retained locals are retired later inside PoseCoreTotal.
    PosePrelude,
    /// One complete coarse XY loop, including assigned selection and unassigned checks.
    /// Excludes preceding fringe and following slope/ground-relation checks.
    PoseCoarseColumns,
}

impl Stage {
    /// Number of fixed timing categories.
    pub const COUNT: usize = 26;
    /// Every timing category, in record-array order.
    pub const ALL: [Self; Self::COUNT] = [
        Self::MotionTotal,
        Self::CurrentSupportLookup,
        Self::BaseNearSupportLookup,
        Self::NextNearSupportLookup,
        Self::ContactFrame,
        Self::SegmentSupport,
        Self::FinalBodyValidation,
        Self::PlaneCacheLookup,
        Self::ContactDomainCollection,
        Self::ContactPointSort,
        Self::ContactPlaneFit,
        Self::ContactFrameAlgebra,
        Self::DenseBandCacheRead,
        Self::DenseBandSelectionSeed,
        Self::DenseConnectivityBfs,
        Self::DenseOrderedEmission,
        Self::DenseBandCacheAcquire,
        Self::KnownTerrainTotal,
        Self::KnownCommonBand,
        Self::KnownTreadGeometry,
        Self::KnownOnlyFallback,
        Self::ProjectedKnownTerrain,
        Self::SweptKnownTerrain,
        Self::PoseCoreTotal,
        Self::PosePrelude,
        Self::PoseCoarseColumns,
    ];

    /// Stable label for reporting after the measured interval.
    pub fn label(self) -> &'static str {
        match self {
            Self::MotionTotal => "motion_total",
            Self::CurrentSupportLookup => "current_support_lookup",
            Self::BaseNearSupportLookup => "base_near_support_lookup",
            Self::NextNearSupportLookup => "next_near_support_lookup",
            Self::ContactFrame => "contact_frame",
            Self::SegmentSupport => "segment_support",
            Self::FinalBodyValidation => "final_body_validation",
            Self::PlaneCacheLookup => "plane_cache_lookup",
            Self::ContactDomainCollection => "contact_domain_collection",
            Self::ContactPointSort => "contact_point_sort",
            Self::ContactPlaneFit => "contact_plane_fit",
            Self::ContactFrameAlgebra => "contact_frame_algebra",
            Self::DenseBandCacheRead => "dense_band_cache_read",
            Self::DenseBandSelectionSeed => "dense_band_selection_seed",
            Self::DenseConnectivityBfs => "dense_connectivity_bfs",
            Self::DenseOrderedEmission => "dense_ordered_emission",
            Self::DenseBandCacheAcquire => "dense_band_cache_acquire",
            Self::KnownTerrainTotal => "known_terrain_total",
            Self::KnownCommonBand => "known_common_band",
            Self::KnownTreadGeometry => "known_tread_geometry",
            Self::KnownOnlyFallback => "known_only_fallback",
            Self::ProjectedKnownTerrain => "projected_known_terrain",
            Self::SweptKnownTerrain => "swept_known_terrain",
            Self::PoseCoreTotal => "pose_core_total",
            Self::PosePrelude => "pose_prelude",
            Self::PoseCoarseColumns => "pose_coarse_columns",

        }
    }
}

/// Actual observed events; cache misses include unsuccessful plane collection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(usize)]
pub enum Counter {
    /// A plane was returned by the existing contact-plane cache lookup.
    PlaneCacheHits,
    /// The existing contact-plane cache lookup returned no plane.
    PlaneCacheMisses,
    /// Points collected for a cache miss, before sorting and fitting.
    ContactPoints,
    /// Fine-domain calls selecting the bounded dense implementation.
    DenseDomainCalls,
    /// Invocations of the sparse fine-domain implementation.
    SparseDomainCalls,
    /// Rectangular grid positions allocated per dense call, counted once rather
    /// than summed across temporary tables; this is not a byte count.
    DenseAllocatedSlots,
    /// Eligible dense columns with a band-cache entry in the batch read,
    /// including cached empty bands.
    DenseCachePresentColumns,
    /// Eligible dense columns without a band-cache entry in the batch read.
    DenseCacheAbsentColumns,
    /// Dense columns with a selected real band, before seed step-height checks.
    DenseSelectedColumns,
    /// Unique dense columns visited by BFS, including the seed and columns
    /// whose final point weight is zero.
    DenseVisitedColumns,
    /// Cached eligible columns whose immutable band vector has exactly one band.
    DenseCacheSingleBandColumns,
    /// Visited-table slots requested only after a valid dense seed exists.
    DenseVisitedTableSlots,
    /// Persistent observation block-root lookups by short-lived borrowed accessors.
    ObservationBlockRootResolutions,
    /// Original exact-pose lookup returned an entry, including cached failures.
    PoseCacheHits,
    /// Original exact-pose lookup returned no entry.
    PoseCacheMisses,
    /// Original pose-cache capacity branch cleared its table; not source replacement.
    PoseCacheCapacityClears,
    /// Original empty-boundary motion lookup returned an entry, including failures.
    MotionCacheHits,
    /// Original empty-boundary motion lookup returned no entry; excludes bypasses.
    MotionCacheMisses,
    /// Original motion-cache capacity branch cleared its table; not source replacement.
    MotionCacheCapacityClears,
    /// Nonempty boundary took the original uncached motion path; never a miss.
    MotionCacheBoundaryBypasses,
    /// Sum of original half-open integer Z spans in assigned fine XY slices.
    AssignedDenseZSpan,
    /// Actual assigned-loop Z entries before contact checks: dense baseline or indexed candidate.
    AssignedActualZCandidates,
    /// Actual non-exempt assigned-loop get calls returning None.
    AssignedFineGetAbsent,
    /// Polygon-intersecting coarse XY columns taking the patch Some branch,
    /// counted before virtual-mask rejection; not a count of completed fine selectors.
    PoseAssignedColumns,
    /// Polygon-intersecting coarse XY columns taking the patch None branch,
    /// counted before virtual-mask rejection; not a count of completed collision checks.
    PoseUnassignedColumns,
}

impl Counter {
    /// Number of fixed counter categories.
    pub const COUNT: usize = 25;
    /// Every counter category, in record-array order.
    pub const ALL: [Self; Self::COUNT] = [
        Self::PlaneCacheHits,
        Self::PlaneCacheMisses,
        Self::ContactPoints,
        Self::DenseDomainCalls,
        Self::SparseDomainCalls,
        Self::DenseAllocatedSlots,
        Self::DenseCachePresentColumns,
        Self::DenseCacheAbsentColumns,
        Self::DenseSelectedColumns,
        Self::DenseVisitedColumns,
        Self::DenseCacheSingleBandColumns,
        Self::DenseVisitedTableSlots,
        Self::ObservationBlockRootResolutions,
        Self::PoseCacheHits,
        Self::PoseCacheMisses,
        Self::PoseCacheCapacityClears,
        Self::MotionCacheHits,
        Self::MotionCacheMisses,
        Self::MotionCacheCapacityClears,
        Self::MotionCacheBoundaryBypasses,
        Self::AssignedDenseZSpan,
        Self::AssignedActualZCandidates,
        Self::AssignedFineGetAbsent,
        Self::PoseAssignedColumns,
        Self::PoseUnassignedColumns,
    ];

    /// Stable label for reporting after the measured interval.
    pub fn label(self) -> &'static str {
        match self {
            Self::PlaneCacheHits => "plane_cache_hits",
            Self::PlaneCacheMisses => "plane_cache_misses",
            Self::ContactPoints => "contact_points",
            Self::DenseDomainCalls => "dense_domain_calls",
            Self::SparseDomainCalls => "sparse_domain_calls",
            Self::DenseAllocatedSlots => "dense_allocated_slots",
            Self::DenseCachePresentColumns => "dense_cache_present_columns",
            Self::DenseCacheAbsentColumns => "dense_cache_absent_columns",
            Self::DenseSelectedColumns => "dense_selected_columns",
            Self::DenseVisitedColumns => "dense_visited_columns",
            Self::DenseCacheSingleBandColumns => "dense_cache_single_band_columns",
            Self::DenseVisitedTableSlots => "dense_visited_table_slots",
            Self::ObservationBlockRootResolutions => "observation_block_root_resolutions",
            Self::PoseCacheHits => "pose_cache_hits",
            Self::PoseCacheMisses => "pose_cache_misses",
            Self::PoseCacheCapacityClears => "pose_cache_capacity_clears",
            Self::MotionCacheHits => "motion_cache_hits",
            Self::MotionCacheMisses => "motion_cache_misses",
            Self::MotionCacheCapacityClears => "motion_cache_capacity_clears",
            Self::MotionCacheBoundaryBypasses => "motion_cache_boundary_bypasses",
            Self::AssignedDenseZSpan => "dense_z_span",
            Self::AssignedActualZCandidates => "actual_z_candidates",
            Self::AssignedFineGetAbsent => "fine_get_absent",
            Self::PoseAssignedColumns => "pose_assigned_columns",
            Self::PoseUnassignedColumns => "pose_unassigned_columns",

        }
    }
}

/// Fixed-size results for one thread's explicit profiling interval.
#[derive(Clone, Copy, Debug)]
pub struct Record {
    /// Inclusive per-stage elapsed nanoseconds, saturated at `u64::MAX`.
    pub wall_nanos: [u64; Stage::COUNT],
    /// Completed stage guards, including error paths, saturated at `u32::MAX`.
    pub calls: [u32; Stage::COUNT],
    /// Actual event counts, saturated at `u64::MAX`.
    pub counters: [u64; Counter::COUNT],
}

impl Default for Record {
    fn default() -> Self { Self::EMPTY }
}

impl Record {
    const EMPTY: Self = Self {
        wall_nanos: [0; Stage::COUNT],
        calls: [0; Stage::COUNT],
        counters: [0; Counter::COUNT],
    };
}

struct State {
    enabled: bool,
    epoch: u64,
    record: Record,
}

thread_local! {
    static PROFILE: RefCell<State> = const { RefCell::new(State {
        enabled: false,
        epoch: 0,
        record: Record::EMPTY,
    }) };
}

/// Begin a fresh record on this thread. Panics if an interval is already active.
pub fn begin() {
    PROFILE.with(|profile| {
        let mut state = profile.borrow_mut();
        assert!(!state.enabled, "body query profile already active on this thread");
        state.epoch = state.epoch.wrapping_add(1);
        state.record = Record::EMPTY;
        state.enabled = true;
    });
}

/// Stop and return this thread's record. Panics if no interval is active.
///
/// Call after all intended stage guards have dropped. A guard retained beyond
/// this call is ignored and cannot contribute to the next interval.
pub fn end() -> Record {
    PROFILE.with(|profile| {
        let mut state = profile.borrow_mut();
        assert!(state.enabled, "body query profile is not active on this thread");
        state.enabled = false;
        std::mem::take(&mut state.record)
    })
}

/// Read completed work without changing the active interval, epoch or record.
/// A live guard contributes only after drop; call at completed query boundaries.
/// Inactive threads return None. This fixed-size copy reads no clock.
pub fn snapshot() -> Option<Record> {
    PROFILE.with(|profile| {
        let state = profile.borrow();
        state.enabled.then_some(state.record)
    })
}

pub(crate) fn count(counter: Counter) {
    add(counter, 1);
}

pub(crate) fn add(counter: Counter, amount: u64) {
    PROFILE.with(|profile| {
        let mut state = profile.borrow_mut();
        if state.enabled {
            let index = counter as usize;
            state.record.counters[index] = state.record.counters[index].saturating_add(amount);
        }
    });
}

#[must_use = "hold the guard for the complete stage, including intended cleanup"]
pub(crate) struct Guard {
    stage: Stage,
    started: Option<(Instant, u64)>,
    // A zero-sized marker makes the guard !Send and !Sync; no Rc is allocated.
    _same_thread: PhantomData<Rc<()>>,
}

pub(crate) fn guard(stage: Stage) -> Guard {
    let epoch = PROFILE.with(|profile| {
        let state = profile.borrow();
        state.enabled.then_some(state.epoch)
    });
    Guard {
        stage,
        started: epoch.map(|epoch| (Instant::now(), epoch)),
        _same_thread: PhantomData,
    }
}

impl Drop for Guard {
    fn drop(&mut self) {
        let Some((started, epoch)) = self.started else { return; };
        let nanos = u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX);
        PROFILE.with(|profile| {
            let mut state = profile.borrow_mut();
            // Retaining a guard past end must not contaminate a later interval.
            if state.enabled && state.epoch == epoch {
                let index = self.stage as usize;
                state.record.wall_nanos[index] = state.record.wall_nanos[index].saturating_add(nanos);
                state.record.calls[index] = state.record.calls[index].saturating_add(1);
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snapshots_are_read_only_and_only_include_completed_guards() {
        assert!(snapshot().is_none());
        begin();
        let pending = guard(Stage::KnownTerrainTotal);
        count(Counter::PoseCacheMisses);
        let first = snapshot().expect("active profile");
        let second = snapshot().expect("still active");
        assert_eq!(first.calls, second.calls);
        assert_eq!(first.wall_nanos, second.wall_nanos);
        assert_eq!(first.counters, second.counters);
        assert_eq!(first.calls[Stage::KnownTerrainTotal as usize], 0);
        drop(pending);
        let completed = snapshot().expect("snapshot did not end the interval");
        assert_eq!(completed.calls[Stage::KnownTerrainTotal as usize], 1);
        assert_eq!(completed.counters[Counter::PoseCacheMisses as usize], 1);
        let final_record = end();
        assert_eq!(completed.calls, final_record.calls);
        assert_eq!(completed.wall_nanos, final_record.wall_nanos);
        assert_eq!(completed.counters, final_record.counters);
        assert!(snapshot().is_none());
    }

    #[test]
    fn disabled_and_stale_guards_do_not_leak() {
        let disabled = guard(Stage::MotionTotal);
        begin();
        drop(disabled);
        let record = end();
        assert_eq!(record.calls, [0; Stage::COUNT]);
        assert_eq!(record.wall_nanos, [0; Stage::COUNT]);

        begin();
        let stale = guard(Stage::MotionTotal);
        assert_eq!(end().calls, [0; Stage::COUNT]);
        begin();
        drop(stale);
        let record = end();
        assert_eq!(record.calls, [0; Stage::COUNT]);
        assert_eq!(record.wall_nanos, [0; Stage::COUNT]);
    }

    #[test]
    fn nested_stages_are_inclusive_and_repeated_calls_accumulate() {
        begin();
        {
            let _outer = guard(Stage::MotionTotal);
            for _ in 0..2 {
                let _inner = guard(Stage::ContactFrame);
            }
        }
        let record = end();
        assert_eq!(record.calls[Stage::MotionTotal as usize], 1);
        assert_eq!(record.calls[Stage::ContactFrame as usize], 2);
        assert!(record.wall_nanos[Stage::MotionTotal as usize]
            >= record.wall_nanos[Stage::ContactFrame as usize]);
    }

    #[test]
    fn early_returns_still_finish_guards() {
        fn failing_query() -> Result<(), ()> {
            let _guard = guard(Stage::ContactPlaneFit);
            Err(())?;
            Ok(())
        }
        begin();
        assert_eq!(failing_query(), Err(()));
        assert_eq!(end().calls[Stage::ContactPlaneFit as usize], 1);
    }

    #[test]
    fn counters_are_opt_in_and_reset_between_intervals() {
        count(Counter::PlaneCacheHits);
        begin();
        assert_eq!(end().counters, [0; Counter::COUNT]);

        begin();
        count(Counter::PlaneCacheHits);
        count(Counter::PlaneCacheHits);
        add(Counter::ContactPoints, 37);
        let record = end();
        assert_eq!(record.counters[Counter::PlaneCacheHits as usize], 2);
        assert_eq!(record.counters[Counter::ContactPoints as usize], 37);
        assert_eq!(record.counters.iter().sum::<u64>(), 39);

        count(Counter::PlaneCacheMisses);
        begin();
        assert_eq!(end().counters, [0; Counter::COUNT]);
    }

    #[test]
    fn records_are_thread_local() {
        begin();
        std::thread::spawn(|| {
            { let _guard = guard(Stage::MotionTotal); }
            count(Counter::PlaneCacheHits);
            begin();
            { let _guard = guard(Stage::ContactFrame); }
            count(Counter::PlaneCacheMisses);
            let record = end();
            assert_eq!(record.calls[Stage::ContactFrame as usize], 1);
            assert_eq!(record.calls.iter().sum::<u32>(), 1);
            assert_eq!(record.counters[Counter::PlaneCacheMisses as usize], 1);
            assert_eq!(record.counters.iter().sum::<u64>(), 1);
        }).join().unwrap();
        let record = end();
        assert_eq!(record.calls, [0; Stage::COUNT]);
        assert_eq!(record.wall_nanos, [0; Stage::COUNT]);
        assert_eq!(record.counters, [0; Counter::COUNT]);
    }

    #[test]
    fn totals_saturate_instead_of_wrapping() {
        begin();
        PROFILE.with(|profile| {
            let mut state = profile.borrow_mut();
            state.record.wall_nanos[Stage::MotionTotal as usize] = u64::MAX;
            state.record.calls[Stage::MotionTotal as usize] = u32::MAX;
        });
        { let _guard = guard(Stage::MotionTotal); }
        add(Counter::ContactPoints, u64::MAX);
        count(Counter::ContactPoints);
        let record = end();
        assert_eq!(record.wall_nanos[Stage::MotionTotal as usize], u64::MAX);
        assert_eq!(record.calls[Stage::MotionTotal as usize], u32::MAX);
        assert_eq!(record.counters[Counter::ContactPoints as usize], u64::MAX);
    }

    #[test]
    fn category_arrays_match_indices_and_labels_are_unique() {
        for (index, stage) in Stage::ALL.into_iter().enumerate() {
            assert_eq!(stage as usize, index);
            assert!(!stage.label().is_empty());
            assert!(Stage::ALL[..index].iter().all(|other| other.label() != stage.label()));
        }
        for (index, counter) in Counter::ALL.into_iter().enumerate() {
            assert_eq!(counter as usize, index);
            assert!(!counter.label().is_empty());
            assert!(Counter::ALL[..index].iter().all(|other| other.label() != counter.label()));
        }
    }
}
