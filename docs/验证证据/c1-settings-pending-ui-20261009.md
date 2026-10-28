# C1: show in-flight hot application separately from restart requirements

Baseline: `3f2a1f88e67a8aec689f049dd2c58e92481c1cf1`.

The settings read-isolation change already exposes
application.hot_apply_pending while a consumer waits. The row renderer did not
consume it and could tell users to restart while an ordinary hot update was
still being applied.

Only an active profile now uses these keys to show “已保存，正在应用” and its old
known adopted value. An inactive selected profile does not inherit another
profile's status. Normal and advanced rows use the same rendering path. When
pending clears, the existing pending_restart flag controls the display;
hot-and-restart fields retain their external restart scope. Sensitive adopted
values are still hidden, and unsaved input remains unchanged by a state refresh.
No timer, polling, save sequencing, backend behavior or layout is changed.

Verification:
- 21 production-JS synthetic-DOM/fetch checks passed (16 existing + 5 new)
- 4 existing point-cloud settings renderer checks passed
- New cases cover active normal/advanced rows, inactive profile attribution,
  completion with unsaved edits, hot-and-restart behavior, and sensitive values
- Older schemas without these optional fields keep the original behavior
- git diff --check passed; independent source review found no blocker

A cloud-browser attempt to view the isolated production-JS/CSS fixture at a
loopback address was blocked by the browser client (ERR_BLOCKED_BY_CLIENT).
No browser screenshot or visual qualification is claimed. The agent-created tab
and temporary HTTP server were closed. The supported offline renderer tests above
completed; the user's computer and navigation timing work were not disturbed.
