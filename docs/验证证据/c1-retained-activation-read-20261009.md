# Retain the admitted checkpoint during explicit activation

Baseline: `99cdc120a4edb89578fee03d98042407939c645e`.

Explicit `activate_map` previously validated large checkpoint bodies through
borrowed directory views, closed those views, then reopened, froze and validated
the bodies again during runtime preparation. The second owner was necessary;
an admission marker or matching ID alone could not safely reuse closed views.

The activation-only read now performs the existing `Store.load` inside the
original stable directory lifetime instead of the borrowed body pass. It keeps
the original final directory checks and returns a private process-local result
binding the actual repository, bundle, root and owned checkpoint. Runtime
preparation consumes that same checkpoint rather than loading it again.
Provider-origin, predecessor and installation checks remain in place. No new
cache, lock, hash, persistent schema or native/RPC format is introduced.

Ordinary bundle getters and finalization retain borrowed validation without
freezing. Direct runtime preparation and `restore_active_workspace` retain
their previous loading path. Thus cold recovery is compatible but is not part
of this scan-reduction optimization. The internal `_for_activation` path may
return the private retained result; ordinary callers still receive a bundle.

The loader resolves the root before opening the checkpoint, matching the old
runtime's semantics for a catalog with a symlink ancestor or `..` component.
The original open tree still verifies named entries afterward. Two added alias
regressions failed before that correction and passed after it. File no-follow
rules and directory-race rejection were not relaxed.

## Functional qualification

The retained-read checks cover one owned validation, unchanged three freezes,
closed directory descriptors with usable immutable payloads, ordinary getters,
owner/root/bundle mismatches, source/member replacement before and after load,
exception cleanup, direct preparation compatibility and both path aliases.

Local Python 3.12.3 logs record separate suites of 126 and 114 passing tests;
their dot-only logs do not establish a de-duplicated 240-test count. After the
alias fix, the relevant 44-test selection passed. The parent additionally ran
80 targeted tests on Python 3.8.20 before the alias fix, then all 12 retained-read
tests on that interpreter after the fix. Exact Python 3.8.10 was not available.
Baseline/initial failure evidence is retained rather than overwritten.

Real native activation uses the unchanged geometry release binary built from
`65c841b`, with Python at this baseline plus the candidate. Native source did
not change. Generated gRPC, native map restoration, checkpoint/catalog stores,
runtime state and localization publication were real; memory/navigation were
isolated doubles. Gateway readiness, live perception and hardware were excluded.

## Measured work reduction; end-to-end benefit not established

For the same saved larger map, three alternating read-plus-prepare pairs were
30.150/17.349, 29.682/17.684 and 29.744/17.744 ms (ordinary/retained).
Four body validators changed from two calls each to one. The removed borrowed
pass performed 79 managed-file slices returning 8,661,062 bytes. The existing
five full loader reads and three FrozenPayload constructions remained. Owned
validation still slices frozen memory; this is not a claim that all I/O or all
memory slicing is halved. The former scale probe's loader-read count did not
include this borrowed traversal.

An earlier non-interleaved native comparison used different setup and is not
an end-to-end A/B result. Subsequent two ABBA groups used the same saved map,
same setup, same binary and fresh processes, changing only the activation read
route through an experiment wrapper:

- Group 1 outer means: ordinary 262.11 ms, retained 275.64 ms
- Group 2 outer means: ordinary 322.09 ms, retained 317.95 ms
- Combined: ordinary 292.10 ms, retained 296.80 ms

Validator exclusive time decreased by 12.32 ms, but archive RPC (including
seal), native RPC, fsync, owned reads and unattributed Python time increased
in these samples. They are observations, not established causes or assumed
noise. The sample neither demonstrates end-to-end acceleration nor excludes
a small regression. This change is accepted for eliminating measured redundant
body traversal and validation, with that latency limitation explicitly open.
It does not fix or explain the user's long real-map switch time.

Nested spans must not be added: freeze is inside loader reads, archive RPC
inside provider activation, and fsync inside atomic catalog writes. Independent
medians cannot form one activation's total. No native rebuild, robot command,
cache deletion or additional SDK build was needed for these comparisons.
