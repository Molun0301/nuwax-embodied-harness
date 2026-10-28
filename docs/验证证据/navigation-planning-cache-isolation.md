# Private cache owners for planning snapshots

`SurfaceSnapshot::fork_for_planning()` is an explicit shallow fork for a future
planner worker. Ordinary `Clone` behavior is unchanged. This prerequisite does
not introduce an executor, new authority metadata, cancellation, or query rules.
The caller must retain the original request source for result qualification:
`same_geometry` and `same_body_query_view` intentionally remain false between
source and fork, because their strict owner-identity rules are unchanged.

The fork creates fresh body-query, body-patch, frontier, routing/centering, and
observation-obstacle cache owners. These are disposable memoization, not evidence.
It neither locks nor copies any old cache. Geometry graphs, physical graph,
regions, occupied columns, height bounds, virtual masks/blocked sets and body
model remain shared immutable Arc payloads. Graph and region structures contain
no interior mutation; updates produce new owners. No original point array is
stored in a snapshot, and none is copied by this operation.

Observation wrappers are rebuilt with persistent `im::HashMap` root clones;
block maps and column vectors remain shared. All observation records, timestamps,
block size and completeness flags are retained exactly. Mutation of a separate
observation value requires `&mut self`, uses persistent copy-on-write updates,
and replaces its memoization cache. No freshness or permission is manufactured.

Current-body support is semantic evidence, not a disposable cache. Its polygon,
inferred cells and cleared voxels are immutable Arc payloads. A private support
wrapper retains those exact payloads and body pose, but forks its source snapshot
as well. This is necessary because body rebasing and centering follow that source.
Existing construction flattens current-support lineage to an unaugmented source,
so the fork visits at most the current view and its source. It performs fixed
wrapper/root copies plus configuration metadata (including the frame string),
not work proportional to geometry, observations, cache entries or proof-set size.

Validation includes shared-root pointer assertions, distinct private cache-owner
checks, obstacle-cache clear isolation, and deterministic queries while source
locks are held, including current-support rebase and source centering. Equivalence
checks cover body/support geometry, real and virtual obstacles, boundaries, and
missing/fresh/expired observations. Existing MLS tests remain authoritative for
kinematics, guards, evidence and routing behavior.

The three support-payload Arc wrappers add three small allocations when a new
current-body support is constructed, including control-side construction. This
change claims cache-owner isolation, not zero hot-path overhead or an established
latency improvement. A later matched control/planning measurement should assess
that tradeoff; grouping the proof payload could reduce wrapper allocations if
measurement warrants it.
