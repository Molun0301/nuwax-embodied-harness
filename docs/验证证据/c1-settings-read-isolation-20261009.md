# C1: profile settings reads do not wait for hot-apply consumers

Baseline: `d1e83d32a1d0fb5f2b28573f6bd0f4b05652f4aa`.
Qualified in the cloud on 2026-10-08 UTC / 2026-10-09 Beijing time.

## Problem and change

The settings application previously held the same write-sequence lock for schema
reads and the whole save/activate transaction. Detector threshold application
waits for a running detection to finish. Thus a settings read could wait for the
model even though its saved configuration was already available.

Save and activate retain their original write-sequence lock and ordering. They
are not turned into background tasks, and later saves do not overtake an earlier
hot-apply operation. The single-slot inference worker lock is unchanged.

Profile schema reads now capture file bytes/revision and a small runtime-state
snapshot within the existing profile file-transaction boundary. A short state
lock protects adopted values, application errors and pending keys; it is not
held for a consumer callback, schema rendering, logging, stream publication or
service-manager status lookup. One schema traversal uses one captured state.
A waiting group reports its saved value, previous adopted value and
`application.hot_apply_pending`. Success publishes the entire group of adopted
values and clears its errors/pending; failure retains old adopted values.

The initial profile/legacy resolution selects a lock domain only; selection is
resolved again within that domain. A mode change releases that domain before
entering the other one. No file-transaction-to-write-lock inversion is introduced.
Legacy no-profile reads retain their prior write-lock serialization and do not
create a new runtime lock file. Consequently this stage does not promise
nonblocking legacy reads or eliminate all waits during mode transitions.

Fork restart reconstructs both local locks and discards inherited pending keys.
A schema retains its original revision owner; concurrent close cannot silently
revive it under a replacement owner. Error formatting and status subprocess
waits remain outside the state lock. A formatting exception still propagates,
but cannot leave a permanent pending marker.

## Qualification

- 14 new `check_settings_read_isolation.py` tests passed on Python 3.8.20 and the
  same 14 passed on Python 3.12.14
- Against the original two source files, the gated-reader regression failed:
  schema waited for the still-paused consumer. Candidate passes without releasing
  that consumer; this is an ordering proof, not a wall-clock speed assertion
- Existing Python 3.8.20: configuration revisions 28 passed; local-planning
  settings 10 passed; point-cloud configuration truth 8 passed; hash-free config
  migration 10 passed with 2 historical-comparison checks skipped because no
  historical baseline argument was supplied
- Total distinct Python results: 70 passed, 2 skipped. The second interpreter
  does not count as another 14 distinct tests
- Production-JS synthetic-DOM checks: local-planning settings 16 passed;
  point-cloud configuration renderer 4 passed
- `git diff --check` passed

New checks use actual SettingsApplication, profile capture/CAS and file writes in
temporary directories, with a synthetic detector gate. They cover saved/adopted
separation, all three threshold fields, two writers on the same revision,
failed-then-successful application, both activate/save orders, read-state capture
through rendering, slow/broken error formatting, no new legacy write permission,
profile write-lock exclusion, both lock-domain transitions, service-manager wait
isolation, close and fork behavior.

## Boundaries

This is consistency for hot-apply operations owned by one SettingsApplication;
it is not a new global lock framework for arbitrary external calls to
`record_value` or runtime config reload. Physical consumer operations are not
made transactional by a schema snapshot. Re-saving identical file bytes still
has the original no-change semantics and does not automatically retry a previous
failed hot application.

No real model, robot, systemd service or browser was exercised. Python 3.8.10
itself was unavailable; no exact-version qualification is claimed.
