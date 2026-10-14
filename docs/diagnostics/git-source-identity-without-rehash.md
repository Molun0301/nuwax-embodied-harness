# In-memory Git source identity without an application digest

Baseline: `80ae58d88e7fea4e124a83485d8771903298d6df`.

The legacy installer's `_git_source_identity` already captures Git HEAD and
index output, plus a clean-status check. It now returns the two original byte
objects as a tuple rather than concatenating and hashing them. The matching
`install_release` check accepts that tuple. Commands, timeouts, dirty/unreadable
errors and preflight-to-install source-change rejection are unchanged.

This value stays in the current execution context. It is not a transaction
fingerprint, release manifest field, serialized recovery record or cache path.
The two callers that use the helper only to check a clean source still ignore
its return. Git's own object IDs and normal Git operations are unchanged.

Four isolated tests cover reference-preserving output, equality and changes of
both components, dirty/nonzero Git results, and rejection before any release
write when the retained pair changes. All four pass on Python 3.8.20 and
3.12.14; subprocess results are mocked, with no host installation or Git
mutation. The 15 release-inventory tests also pass on Python 3.8.20 after this
change. Exact Python 3.8.10 was not available.

The retained raw index output occupies memory until its execution context is
released, instead of retaining a short digest. No extra output copy or content
scan is introduced, but there is no measured speedup or RSS-reduction claim.
Persistent engine/source-snapshot fingerprints and Rust target-directory
names remain separate migration work; this does not claim complete project
hash removal or change any external signature verification.
