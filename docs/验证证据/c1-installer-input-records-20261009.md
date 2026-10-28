# C1: hash-free input records for the module installer

Baseline: `5acc021e97b04e8da8075e99f4c127f63207bb6f`.
Qualified on 2026-10-08 UTC / 2026-10-09 Beijing time.

## Scope

The default module installer and Web update use `module_installer.manager.apply`.
This stage removes its source-content SHA and the second SHA combining source
identity with the interpreter. It replaces manifest `fingerprint` with one
versioned `installation_inputs` object, compared directly for the existing no-op
installation decision. This object is a local incremental-install hint, not a
cryptographic integrity/authenticity claim.

Source enumeration retains the prior Git tracked/untracked and non-Git export
scope. Regular files contribute relative path, size, mtime_ns, ctime_ns and mode;
source payloads are not opened. Deleted files disappear from the list. As before,
source symlinks are not treated as regular source payloads. Known installation
outputs are excluded when a custom install home is inside the source tree.

The record also contains selected modules, robot assembly, interpreter path and
resolved target metadata, plus the provided Go2 device wheel paths/metadata.
Input sampling takes place inside the existing installation/start locks. No new
lock, cache ledger, file-content copy or hash is introduced.

`repair`, parallel jobs and previous-generation retention are not source inputs.
A repair installs a new generation while saving the same input record; a following
ordinary install can be a no-op. Old hash-only manifests intentionally miss the
new comparison and migrate on the next successful installation. Failure keeps the
previous active generation. Native build/cache paths and cleanup rules are not
changed.

## Limits

Metadata equality is not proof of equal content. Metadata-only changes may cause
a conservative installation, while filesystems/tools that restore all recorded
metadata can evade this local change hint. No content-authentication guarantee is
made. Existing source enumeration scope is not expanded to unrelated hidden or
ignored inputs in this stage.

An effective wheel path change is an input change even if bytes are equal. In
particular, the CLI may switch from conventional wheels to its managed cache on
a later invocation, causing one conservative reinstall; no alias ledger is added.

The legacy `deploy/resume/verify` engine still has separate application hashes
in transaction/release identity. They are not removed here: its resume and seal
semantics need a coordinated migration. TLS, signed bundles, trusted public keys,
and third-party package verification are unchanged.

## Evidence

- New `check_installer_input_records.py`: 10 tests passed on Python 3.8.20 and the
  same 10 passed on Python 3.12.14
- Existing Python 3.8.20 regressions: lifecycle identity 25, terminal rollout 17,
  cache naming identity 51, installation lifecycle 13 passed
- Total distinct test methods: 116, not counting the second interpreter as new
  independent coverage
- Application hash constructors are forbidden in the fixtures; a source-read
  guard verifies metadata enumeration does not open source payloads
- Real temporary Git clean/dirty/add/delete and non-Git paths exercised; same-size
  writes with restored mtime are distinguished by ctime
- Interpreter target edit/symlink retargeting, external wheel replacement, old
  manifest migration, repair/no-op sequence,
  failure recovery, stable native cache path and an untouched cache marker checked
- `git diff --check` passed

Installation state transitions use actual apply/locks/manifest/link switching
with package/model/native build operations mocked. This is not a new full device
installation or a measured native incremental-build performance result. Python
3.8.10 itself was not available and is not claimed as tested.
