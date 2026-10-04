# Isolated NEH recovery

NOT CURRENTLY INTEGRATION-REVALIDATED. These are new local recovery commits from surviving archives, not the vanished original commits.

The v11 snapshot has 1470 inventoried files. Baseline-only files omitted from the archive are preserved and listed in RECOVERY-MANIFEST.json.

Branches are intentionally independent:
- recovery/v11-snapshot-20261004: known Git baseline plus inventoried v11 snapshot
- recovery/exploration-overlay-20261004: v11 plus nine exploration files
- recovery/python-checkpoint-overlay-20261004: v11 plus four checkpoint Python files; native protocol dependency unresolved
- recovery/c1-core-overlay-20261004: v11 plus nine C1 crate files; native consumers not migrated

No WIP native, activation fragments, reference-only relocalization material, or original dirty nav-control changes are incorporated. At the initial local-only recovery phase, no remotes were configured. The exact snapshot is now published to the independent GitHub recovery branch listed below. Archived tests document their own historical scope, not current validation. No native build or dependency installation was performed during initial snapshot recovery. This publication verifies repository durability and exact tree identity only; it does not claim full runtime or integration validation.

## Publication status (2026-10-04)

Published repository: https://github.com/Molun0301/nuwax-embodied-harness

- Branch: recovery/python-checkpoint-overlay-20261004
- Exact local recovery snapshot: 19f2c6a4f8fb9a94e7479cdd628887f7cf3b3b02
- Published exact snapshot: 9f9942699732ccb0569ea278bf328f3cec98909b
- Identical snapshot Git tree: f280c0723ee324d6401c5bd75420284b4b573a33

The original refactor/map-explore-nav remains at 3be3568bbc2d9ddcbec8f32d9a592d36f8633688. This follow-up changes only publication documentation. GitHub API publication creates new commit identities; tree equality preserves source bytes, paths, and modes. These are recovered derivatives, not recovered original lost commits. NOT fully runtime/integration-revalidated. Python/native checkpoint dependencies and C1 consumer migration remain unresolved as described in the manifest.
