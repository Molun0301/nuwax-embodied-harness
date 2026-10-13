#!/usr/bin/env python3
"""Offline active-query scaling: real records/methods, no devices or model calls."""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.naming import build_task_id
from contracts.tasks import TaskSpec, TaskState, TaskStatus
from core.event_bus import EventBus
from core.task_manager import TaskManager, TaskRecord


class TimedLock:
    def __init__(self):
        self.lock = threading.RLock()
        self.held_ns = 0

    def __enter__(self):
        self.lock.acquire()
        self.started = time.perf_counter_ns()
        return self

    def __exit__(self, *args):
        self.held_ns += time.perf_counter_ns() - self.started
        self.lock.release()

    def _is_owned(self):
        return self.lock._is_owned()


class CountedRecords(dict):
    visits = 0

    def values(self):
        for value in super().values():
            self.visits += 1
            yield value


def measure(manager, call, repeats):
    records = manager._records
    counted = CountedRecords(records)
    manager._records = counted
    call()
    manager._records = records
    elapsed, held = [], []
    for _ in range(repeats):
        manager._lock.held_ns = 0
        start = time.perf_counter_ns()
        call()
        elapsed.append((time.perf_counter_ns() - start) / 1e6)
        held.append(manager._lock.held_ns / 1e6)
    return {"record_visits": counted.visits, "median_ms": statistics.median(elapsed),
            "median_lock_ms": statistics.median(held)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=11)
    args = parser.parse_args()
    for terminal in (200, 5000, 50000):
        bus = EventBus()
        manager = TaskManager(event_bus=bus)
        manager._lock = TimedLock()
        try:
            for index in range(terminal + 50):
                task_id = build_task_id("offline_task")
                record = TaskRecord(spec=TaskSpec(task_id=task_id, capability_name="offline_task"),
                                    status=TaskStatus(task_id=task_id, state=TaskState.SUCCEEDED))
                manager._records[task_id] = record
                if index < terminal:
                    manager._history_order.append(task_id)
            all_records = list(manager._records.items())
            for active in (0, 5, 50):
                manager._records = dict(all_records[:terminal + active])
                for _, record in all_records[terminal:terminal + active]:
                    record.status = record.status.model_copy(update={"state": TaskState.RUNNING})
                calls = {"list_active_tasks": manager.list_active_tasks,
                         "capture_task_list": lambda: manager.capture_task_list(history_limit=20),
                         "capture_status_only": lambda: manager.capture_task_list(history_limit=20, status_only=True)}
                for name in ("count_active_tasks", "has_active_tasks"):
                    if hasattr(manager, name):
                        calls[name] = getattr(manager, name)
                result = {"terminal": terminal, "active": active, "active_position": "after_terminal",
                          "methods": {name: measure(manager, call, args.repeats) for name, call in calls.items()}}
                print(json.dumps(result), flush=True)
        finally:
            manager.shutdown(wait=True)
            assert bus.shutdown(2)


if __name__ == "__main__":
    main()
