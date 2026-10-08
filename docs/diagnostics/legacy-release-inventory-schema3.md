# Legacy installer local release inventory: schema 3

Base: `728db9bd635e5a892692235cae39e906df6af465`. Reviewed implementation; no deployment performed.

This slice changes only the legacy `deploy_installer` locally written `release-manifest.json` generation and completed-release read. The CLI, resume transaction fingerprint and release/recovery state machine remain available and unchanged.

## New and historical formats

New manifests use schema 3. Local `binary_sha256`, `source_tree_sha256`, `venv_inventory_sha256`, `vendor_tree_sha256`, `model_sha256`, `cargo_lock_sha256` and `python_input_sha256` are no longer generated. Their replacements are ordinary paths/layout records:

| Field | Meaning |
|---|---|
| `binary_paths` | Binary name to established relative release path; existing `binary_file` architecture description retained |
| `source_files` | Sorted source file names under the existing source exclusions |
| `venv_inventory` | Relative node names, kind, permission mode and symlink target; no content size, mtime or digest |
| `vendor_files` | Vendor directory to sorted source file names |
| `model_files` | Sorted model file names |
| `cargo_lock_files`, `python_input_files` | Existing source input paths, diagnostic provenance only |

Schema 3 verifies required files and layout, including source/vendor/model file sets and venv node/mode/link layout. It retains legitimate venv links and permits an internal ancestor symlink on the Geometry binary path. Binary/vendor paths must remain inside the release. Source-file symlinks and source/venv special nodes remain rejected; mutable source exclusions remain unchanged. Vendor symlinks remain rejected, while non-file/non-symlink nodes are skipped as before. Relative vendor spellings such as `./sdk` and `sdk/` remain valid; absolute paths and `..` remain rejected. Model enumeration preserves the historical rule: only non-symlink regular files are listed, unregistered symlinks/special nodes are ignored, and a valid models-root alias is accepted. Registered model paths still require a regular non-symlink file. Source enumeration retains the existing treatment of directory symlinks (no traversal); the explicit binary check covers the Geometry active link.

Schema 2 remains readable without modifying its file or manufacturing a historical stat inventory. Its binary/vendor/model names are used, but its recorded local digests are never recomputed or compared. Existing required binary, vendor-root, model-file-set and interpreter/layout checks still apply. Since schema 2 has no historical source/venv path inventory, this reader cannot reconstruct or claim their historical file sets; missing arbitrary source/package files must be caught by the remaining runtime/version/health checks. No missing-stat rollback gate is added. An older installer that accepts only schema 2 cannot read schema 3; backward reader compatibility is not claimed.

**Intentional semantic change:** if release bytes drift while required paths, node layout, versions and health still pass, the new reader accepts the release. It no longer proves equality with the original local release contents. The new test is named `test_byte_drift_accepted_without_claiming_local_integrity`; it does not assert old hash-tamper behavior.

## External trust and lifecycle boundaries

Approved wheel/source integrity, YOLO download source pins, public-key pins, actual RSA signature verification, signed file manifests and signed-profile matching are unchanged. Shared `_sha256` and `_tree_digest` remain for these uses. `third_party_sha256` still records already-verified external input provenance; it is not a new local release checksum. Binary architecture/linkage installation acceptance, interpreter/package checks and runtime health acceptance remain.

`rollback`, persisted recovery, latest/generation/current/candidate/previous relations, transaction phases, configuration backup and health-failure stop behavior are unchanged. No host installation, release switch, systemd invocation or robot operation occurred: lifecycle tests use only temporary fixture directories and recording/mocked runners.

## Qualification

The Geometry fixture tests an internal ancestor symlink at `target/release`; the real profile's active link is `target/release/nuwax-geometry-release`. This fixture does not qualify the complete real Geometry release layout. The new venv metadata inventory has linear space cost in the number of filesystem nodes (as well as metadata traversal); this slice contains no deployment performance measurements or overall speedup claim.

Fixture correction v1: schema 2 test fields are fixed, opaque synthetic historical-shape strings. They do not authenticate the current fixture content. Fixture construction no longer reads an old Git object, executes old code or computes any local release digest; read/digest/process sentinels cover construction as well as validation. Model and vendor enumeration are also corrected to the old reader behavior, with explicit schema 2 model alias/unlisted-node regressions and both-schema vendor spelling/special-node cases. The external RSA/source-pin tests and functions are unchanged; their earlier results below are retained, not rerun for this correction. The final 11 affected compatibility/fixture methods pass on both Python 3.12.3 and 3.8.20. The initial fixture-only five-method rerun also passed and is preserved, but is a subset of those 11, not five additional methods. The former test-fixture implementation and logs remain in Library v0 as superseded evidence.

- The initial v0's 12 new isolated tests passed on Python 3.12.3 and 3.8.20. They cover schema 2 read without writeback, schema 3 generation, no-content-read/digest sentinels, deliberate byte drift, missing files, invalid paths/schema, symlink/special-node rules, internal versus escaping ancestor links on the Geometry binary path, rollback config/health order, and real RSA/signed-file/key/profile and wheel/source/YOLO pin rejection. The final v1 script has 15 methods after the three compatibility additions; its 11 affected methods passed in both local runtimes as described above. The parent additionally ran all 15 final methods on Python 3.8.20, all passing.
- The 11 existing inline installer contracts extracted verbatim from `scripts/ci/check_deployment_installer.sh` pass on both interpreters. These include the 45-case recovery matrix, phase failure and activation-before-stop ordering. Only its isolated Python test block was run; no full shell installer CI or machine actions were invoked.
- Counts: 23 distinct methods at v0; 26 at final v1 including the unchanged 11 inline contracts, qualified through the separate runs above. Fixture subcases and repeated interpreter executions are not additional test methods. Exact Python 3.8.10 was unavailable; 3.8 syntax and 3.8.20 execution pass.
- Ruff checks with `--no-cache` and `git diff --check` pass. Initial Ruff cache write was rejected in the read-only command context; no approval escalation was needed to run the final read-only lint. New-script import-order notices were fixed with explicit annotations for its necessary repository path setup.
- AST/source audit confirms only `_validate_completed_release` and `install_release` changed among surviving functions. Removed helpers were solely the four local release/input digest helpers. External trust, rollback/recovery and health functions match the base byte-for-byte.

Remaining application hashes are separate work: `engine.py` path/source identities and transaction fingerprint, `operations.py` source-snapshot/Git identity, `model.py` Rust target directory key, and geometry release validation. This patch does not claim project-wide hash removal and does not weaken signed bundle creation/verification.

Evidence and exact patch are under `/home/ml/Documents/Codex/2026-10-08/task-4/release-inventory-qualification/`. The original five navigation diagnostic differences and simulator are untouched; no build, cache cleanup or new production cache/lock was introduced.
