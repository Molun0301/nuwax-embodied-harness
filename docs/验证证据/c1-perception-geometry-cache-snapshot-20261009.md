# C1: retain complete geometry cache groups across perception waits

Baseline: `f0f16c1847004fdd104e601ff296de41e3287a3e`.
Qualified in the cloud on 2026-10-08 UTC (2026-10-09 Beijing time).

## Change

The full perception entry previously looked up point cloud, frame tree and pose
only after detector and segmentation work. A newer cache publication during a
model or lock wait could therefore change the geometry inputs for an older image.

`PerceptionService._run_full_pipeline` now retains the three existing cache
payload references before waiting for the full pipeline lock, only when all
three are available and the existing point-cloud usability check succeeds.
The existing StreamBus slot-snapshot method supplies this group; no new lock,
provider call, array copy, coordinate calculation or rejection condition is added.

If any member is absent, the entire original late-resolution path is retained.
An early cloud is never combined with a later provider TF by this change.
No-detection/no-calibration early returns still avoid provider calls; original
fallback exceptions remain visible. Fast detection is unchanged.

## Qualification

New script: `scripts/check_perception_geometry_snapshot.py`.

- Python 3.8.20: 6 tests passed
- Python 3.12.14: the same 6 tests passed
- The unchanged script against the original service reports 4 failure events
  across 3 test methods (detector and segmentation are separate subtests).
  The model-update cases directly observe the wrong newer reference group;
  the lock-wait case times out waiting for the missing early cache capture.
- Python 3.8.20 existing regressions: perception dispatch 11 passed, scene
  dispatch 9 passed, artifact producer lifetimes 30 passed
- Total distinct test methods qualified: 56; two Python runs of the new six
  are not counted as twelve distinct tests
- `git diff --check` passed

The new tests use the real StreamBus and full service entry points, replacing
models and the final projection with deterministic stubs. They cover reference
identity before lock waits, detector/segmentation publication, each missing
member, no-detection and no-calibration behavior, preservation of a provider
exception, the current-scene entry, and isolation of the fast path.
No model download, robot operation or full project rebuild was performed.
Python 3.8.10 itself was not available and is not claimed as tested.

## Limits

This is a snapshot of current bus slots, not measurement-time synchronization.
The three topics can already have different source times before capture. Missing
cache groups still use the old late path. The change does not add TF history,
interpolation, raw/registered point pairing, or image-time registration binding.
It cannot undo source mismatch before the request or image artifact saving.

References rely on existing publishers replacing frame objects. They are not a
promise to protect against an arbitrary provider mutating an old payload in place.
No inference latency or real-robot projection accuracy improvement is claimed.
