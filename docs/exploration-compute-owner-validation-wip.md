# Unqualified exploration runtime owner integration

This validation branch proposes activation of the reusable exploration compute owner. It is **not qualified for the primary recovery branch**. The reviewed primary checkpoint is `95e797aed4618f2bac6ba8ee9030deb81aafb3d8`; its scheduler is still synchronous. The two component checkpoints in `exploration-compute-owner.md` do not qualify the wiring in this branch.

The proposed wiring moves controller ownership, selection, mission route preparation/queries and coverage work to the reusable owner. Outer state keeps committed projections, immutable inputs, retained transactions/feedback/lifecycle acknowledgments, and the existing navigation lease/cancellation protocol. Test-only attribution follows the worker scopes and the same 500×500 fixture.

Initial integration evidence:

- `runtime-parse-format-v1.log` records successful Rust parsing with formatting differences; the pre-existing runtime file is not globally rustfmt-clean. New service and test files are formatted separately.
- `runtime-compile-v1.log` fails on one attribution-fixture `Option<&Arc<_>>` versus `Option<&_>` comparison. It was corrected to `as_deref()`. No actor runtime test has run from that failed build.
- Independent review identified the driver cache-publication ordering race, retained fault/child-cancellation action, source requalification after internal mission feedback, accepted Stop surviving waiter cancellation, and compute failure visibility to root supervision. These are being repaired and must be covered by deterministic tests before activation.
- Additional gate tests are source-only and unqualified. Their fake navigation and injected ZEROED proofs establish software ownership only.

Pending: final source review, successful final-source compilation, deterministic actor/lifecycle/cancellation/fault tests, unchanged runtime regressions, original optimized attribution matrix, and source-change/unchanged-input liveness. No asynchronous responsiveness, hardware, real-network, deployment, or whole-platform completion claim is made.

This WIP checkpoint exists to preserve work across C1’s prioritized original-path qualification window. C1 must validate from the reviewed primary base and restore the exact WIP branch/HEAD afterward. Do not publish or merge this WIP as a qualified runtime change.
