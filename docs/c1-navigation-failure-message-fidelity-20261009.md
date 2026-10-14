# Preserve existing navigation failure messages

Baseline: `799b150544058ee69b03d1350304c7a7a3270a35`.

This phase preserves evidence already present in planner failures. It does not
change geometric acceptance, search budgets, motion commands, stopping policy,
wire fields, or add a new collision/support classifier.

The Rust scheduler previously replaced `Pending(failures)` with a planner wait
description when one existed, then replaced that text again with a generic
path timeout. Pending status now combines the current wait description and
current nonempty failure list. It reconstructs text each tick rather than
appending prior status repeatedly. Timeout prefixes its explanation to the
prior diagnostic. The existing terminal handler retains that cause through
the verified fence; a real stopping failure still takes priority.

The gateway's ordinary backend `FAILED` result now uses the neutral public
code `navigation_failed`, preserving the backend message. A finite search
budget, expired observation or waiting timeout is not proof that the target
is unreachable. This is an intentional public error-code change for that
branch. The separate target-resolution error, stale-input retry, arrival
confirmation, cancellation and parking cleanup branches remain unchanged.
Both old/new ordinary failure codes retain the existing capability-error
classification and retry behavior.

## Qualification

- Local Rust release: three new scheduler tests and seven related regressions,
  all ten passed. Cases cover wait reason plus failures, empty combinations,
  repeated-tick stability, unchanged intent/deadline/zero command, timeout
  cause in the terminal fence result, and an unconfirmed stop retaining its
  primary error and active recovery ownership. The first test run's two
  failures came from fixtures leaving timeout disabled; the corrected fixture
  explicitly sets one second. Both logs are preserved.
  The local checkout retained its five pre-existing diagnostic differences;
  its nav-runtime difference adds test-only diagnostic modules, and its other
  tracked change belongs to the separate simulator, which was not rebuilt.
  This is targeted local scheduler qualification, not a clean root-tree full
  Rust suite rerun.
- Parent Python 3.8.20 and 3.12.14: three tests each run the real task manager
  and navigation task runner with isolated navigation/parking endpoints. They
  verify budget/timeout/geometry text, generic failure classification, the
  existing arrival error and stale-input resubmission to success. The actual
  task receipt, event feedback and direct navigation-state handler preserve
  the cause. The shared Web/MCP serializer returns matching JSON text and
  structured content, `isError=True`, and the original error message/code.
- Twelve production `navigation.js` synthetic-DOM checks passed, including a
  new terminal-failure rendering check. No browser or HTTP transport was
  exercised for this phase. Motion admission and parking proofs in the
  Python fixture are doubles, not hardware qualification.

Initial Python fixture mistakes (missing mock arguments and reading stage
outside the event payload) are retained separately from the expected baseline
`target_not_reachable` assertion failures. Corrected tests pass without any
production changes to accommodate those fixture errors. Exact Python 3.8.10
was not available. Native tests were run locally, not repeated in the cloud.

## What this does not recover

Current fieldless `SurfaceError` variants still lose exact rejected cells,
layer heights and support/occupancy comparisons at their source. Some candidate
errors are deliberately discarded during search. Keeping the final message
cannot reconstruct that missing information or prove all alternative actions
are impossible. `MissingSupport` before collision checks also does not mean
there is no collision. Adding source witnesses remains a separate, bounded
diagnostic task; no per-candidate replay, extra map copy or logging was added.
