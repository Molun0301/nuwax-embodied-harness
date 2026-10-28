# Assigned-map catalog publication

Baseline: `65c841b67566003f7892e2f2743b610218c07dbb`.

`_save_selected_asset` previously reconciled active flags before saving the
complete selected record. For an initially inactive target this persisted its
`active=True` flag with its old status, then immediately replaced that same
metadata with the complete `LOCALIZING` record.

The change saves the complete selected record first, then runs the unchanged
catalog reconciliation. Its existing active-flag comparison naturally skips
the already selected target and still deactivates other maps. The selected
write remains unconditional on every retry, including after a visible rename
whose directory fsync failed. No journal barrier, fsync implementation,
validation, lock, callback, receipt or session-commit operation was removed.

Both writes remain inside the same catalog transaction. Until reconciliation
succeeds, the retained journal and runtime fence still prevent exposing a
completed workspace. Receipt persistence, session adoption and journal
retirement retain their original ordering.

## Evidence

A diagnostic wrapper used the existing public workspace fixture on CPython
3.8.20, with real file writes and fake native RPC. One fresh activation at the
baseline performed two selected metadata saves and 18 fsync calls. The candidate
performed one complete selected save and 16 fsync calls. All other durable
phase writes remained. This is a tiny, warm, synthetic Python-only fixture;
cloud filesystem fsync timings were microseconds and are not a latency estimate
for the user's disk, a native map restore or Go2.

The initial three regressions produced two expected baseline failures: duplicate
target save count and old-target reconciliation preceding the complete new
target save. The target-write recovery check already passed. With the candidate,
these three and all 32 existing assigned-workspace activation tests passed on
Python 3.8.20. Two additional checks brought the new suite to five, all passing
on Python 3.8.20 and 3.12.14:

- Fresh activation writes complete selected metadata once
- A-to-B activation writes B once and clears A's flag
- Failure before the target metadata write preserves the original request and
  recovers without another native installation
- Failure after the target rename, at the real directory-fsync boundary, is
  retried on two explicit attempts; successful recovery reuses native evidence
- Failure clearing the old flag after the new selected metadata was persisted
  retains recovery state, then completes without another StopMapping/ActivateMap

There are 37 distinct passing tests, not 45: repeated interpreter executions
are additional compatibility evidence, not additional test cases. Exact
Python 3.8.10 and power-loss testing were not available. Existing tests also
cover receipt failures, journal retirement, successor callbacks and replay.

## Existing boundary not changed

If an old map's inactive metadata rename succeeds but its directory fsync
fails, a later reconciliation can read `active=False` and skip that old record.
This pre-existing behavior is not repaired here. Public active-map projection
uses the active pointer and pending journal; this small optimization does not
claim a new crash-durability protocol for every historical raw metadata flag.
Native restore, callback-drain latency and full deployed activation remain
separate qualification work.
