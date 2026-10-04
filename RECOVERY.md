# Isolated NEH recovery

NOT CURRENTLY INTEGRATION-REVALIDATED. These are new local recovery commits from surviving archives, not the vanished original commits.

The v11 snapshot has 1470 inventoried files. Baseline-only files omitted from the archive are preserved and listed in RECOVERY-MANIFEST.json.

Branches are intentionally independent:
- recovery/v11-snapshot-20261004: known Git baseline plus inventoried v11 snapshot
- recovery/exploration-overlay-20261004: v11 plus nine exploration files
- recovery/python-checkpoint-overlay-20261004: v11 plus four checkpoint Python files; native protocol dependency unresolved
- recovery/c1-core-overlay-20261004: v11 plus nine C1 crate files; native consumers not migrated

No WIP native, activation fragments, reference-only relocalization material, or original dirty nav-control changes are incorporated. No remotes are configured. Archived tests document their own historical scope, not current validation. No native build or dependency installation was performed.
