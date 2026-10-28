# PointLIO patch preparation and publication

Repeated CMake configuration used to copy five upstream files directly into
`pointlio-initialized/src`, then patch them in place. Even when final bytes were
identical, all five consumed files received new modification times. A failure
in the last patch also left three consumed files different from the preceding
successful output in the measured baseline.

The candidate prepares those same five files under
`pointlio-initialized/patch-prepare/src`. The existing eight patches, forward and
reverse dry-runs, and failure handling run unchanged there. Only after the full
sequence succeeds does `configure_file(COPYONLY)` publish each file to its
existing consumption path. Equal bytes preserve consumption timestamps.
Preparation files remain because CMake registers them as configuration inputs.
This protects the previous publication from patch failure; it does not promise
atomic publication of all five files if copying itself fails.

## Focused regression

Run with an existing clean checkout of locked upstream commit
`288e357e5457723c1cce4d4060f76ed7f85b10d4` and a new experiment directory:

```sh
python3 scripts/check_pointlio_patch_publication.py \
  --source /path/to/locked/pointlio \
  --baseline-dir /path/to/existing/baseline/pointlio-initialized/src \
  --work-dir /path/to/new/pointlio-publication-check
```

The Python 3.8-compatible script copies only the five source files and eight
patches into the experiment. It extracts the actual production CMake preparation
and publication block verbatim, wraps it with `project(... NONE)`, and uses a
`DEPENDS` consumer that records executions. It does not reproduce the patching
implementation. Input changes and deliberate patch failures affect only the
experiment copies; upstream and production input preservation is checked.

Observed with CMake 3.28.3, GNU patch 2.7.6, and Python 3.12.3 (Python 3.8 syntax
also parsed; a Python 3.8 interpreter was not exercised):

- Baseline: second configuration produced identical bytes but changed all five
  mtimes; consumer executions increased from one to two. A plain idle build did
  not run the consumer. Final-patch failure changed three published files.
- Candidate: first output matched the existing baseline bytes. Repeat configuration preserved all five bytes and mtimes, with no
  extra consumer execution. Each of the five upstream inputs was independently changed; ordinary build
  automatically reconfigured and changed only that file’s published bytes and
  mtime. Changing a real added line of the last patch also propagated and ran
  the consumer once.
- Final-patch failure preserved all five published bytes and mtimes. Restoring
  the patch and retrying succeeded without changing the previous publication.
  Ordinary build also failed while the patch was broken, without consuming old
  results; a fresh failing build published none of the five files.
- Immediate idle builds, including two after recovery, neither reconfigured nor
  ran the consumer. Preparation files remained available. An unrelated generated
  header sentinel retained its bytes and mtime.

The candidate run contains 23 bounded commands: nine successful configurations,
three intended failing commands, and seven total consumer executions.
Complete command logs and the retained failure record accompany the review
artifact. No PointLIO native C++ build, SDK build, algorithm change, or native
compilation performance qualification was performed.

The parent subsequently compiled the regression script and ran its `--help`
entry point with Python 3.8.20. This confirms that narrow interpreter boundary;
the CMake regression itself was not rerun there, and exact Python 3.8.10 was not
available.
