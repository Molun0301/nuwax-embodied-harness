"""有界终态回执拥有者；只恢复记录，绝不调用或重放领域动作。"""
from __future__ import annotations

import sqlite3
import threading
from collections import OrderedDict, deque
from time import monotonic

from core.errors import GatewayError
from gateways.agent.feedback import FULL_RESULT_MAX_BYTES, TERMINAL_STATES


class TerminalReceiptPersistence:
    def __init__(self, source, store, snapshot, eligible, *, capacity=256, id_bytes_limit=65536,
                 page_size=64, scan_interval=30.0, retry_base=0.1, retry_cap=5.0, clock=monotonic, autostart=True):
        self.source, self.store = source, store
        self._snapshot, self._eligible = snapshot, eligible
        self._capacity = max(1, capacity)
        self._id_bytes_limit = max(1, id_bytes_limit)
        self._page_size = max(1, min(64, page_size))
        self._scan_interval = max(0.01, scan_interval)
        self._retry_base, self._retry_cap, self._clock = retry_base, retry_cap, clock
        self._cv = threading.Condition()
        self._mailbox = OrderedDict()
        self._id_bytes = 0
        self._high_water = self._overflow = self._late_hints = 0
        self._scan_serial = 0
        self._scan_required = True
        self._next_scan = clock()
        self._cursor = None
        self._history_reference = None
        self._page = deque()
        self._page_versions = {}
        self._pass_serial = self._pass_blocked = self._pass_excluded = 0
        self._scan_offset = self._scan_end = 0
        self._covered_generation = None
        self._blocked = self._excluded = 0
        self._errors = OrderedDict()
        self._source_gap = False
        self._retry_at = 0.0
        self._retry_since = None
        self._in_flight_since = None
        self._retry_task = None
        self._retry_from_scan = False
        self._prepared = None
        self._attempts = self._failures = self._consecutive_failures = 0
        self._last_error = None
        self._in_flight = None
        self._close_requested = self._source_fenced = self._owner_closed = False
        self._restarts = 0
        self._thread = None
        self._supported = all(callable(getattr(source, name, None)) for name in (
            "task_receipt_header", "receipt_source_status", "begin_terminal_receipt_scan", "page_terminal_receipts"))
        self._supported = self._supported and all(callable(getattr(store, name, None)) for name in (
            "terminal_versions", "prepare_terminal", "save_prepared_terminal"))
        if self._supported:
            if autostart:
                self._start_locked()
        else:
            self._owner_closed = True
            self._last_error = "receipt_source_api_unavailable"

    def start(self):
        with self._cv:
            if self._supported and self._thread is None and not self._close_requested:
                self._start_locked()

    def _start_locked(self):
        self._owner_closed = False
        self._thread = threading.Thread(target=self._run, name="Agent-terminal-receipts", daemon=True)
        self._thread.start()

    def offer(self, task_id):
        """事件回调仅触及有界 ID 邮箱；不读任务源或存储。"""
        if not isinstance(task_id, str) or not task_id:
            return
        size = len(task_id.encode("utf-8")) if len(task_id) <= 512 else self._id_bytes_limit + 1
        with self._cv:
            if self._owner_closed:
                self._late_hints += 1
                return
            if task_id in self._mailbox:
                self._cv.notify_all()
                return
            if len(self._mailbox) >= self._capacity or self._id_bytes + size > self._id_bytes_limit:
                self._overflow += 1
                self._scan_required = True
                self._scan_serial += 1
                self._next_scan = min(self._next_scan, self._clock())
            else:
                self._mailbox[task_id] = (self._clock(), size)
                self._id_bytes += size
                self._high_water = max(self._high_water, len(self._mailbox))
            self._cv.notify_all()

    def request_reconcile(self):
        with self._cv:
            self._scan_required = True
            self._scan_serial += 1
            self._next_scan = self._clock()
            self._cv.notify_all()

    def wake(self):
        """唤醒时间门闩；不取消已有存储退避。"""
        with self._cv:
            self._cv.notify_all()

    def _blocked_result(self, task_id, code):
        sample_id, code = task_id[:512], str(code)[:128]
        with self._cv:
            self._last_error = code
            self._errors[sample_id] = code
            self._errors.move_to_end(sample_id)
            while len(self._errors) > 64:
                self._errors.popitem(last=False)
        return "blocked"

    def _storage_failure(self, task_id, *, from_scan):
        with self._cv:
            self._failures += 1
            self._consecutive_failures += 1
            delay = min(self._retry_cap, self._retry_base * (2 ** min(16, self._consecutive_failures - 1)))
            if self._retry_since is None:
                self._retry_since = self._clock()
            self._retry_at = self._clock() + delay
            self._retry_task = task_id
            self._retry_from_scan = from_scan
            self._last_error = "terminal_store_retry_pending"
            self._cv.notify_all()
        return "retry"

    def _process(self, task_id, versions=None, *, from_scan=False):
        with self._cv:
            self._in_flight = task_id
            self._in_flight_since = self._clock()
        try:
            header = self.source.task_receipt_header(task_id)
            if header["state"] not in TERMINAL_STATES:
                return "done"
            if not self._eligible(header["capability_name"]):
                return "excluded"
            if not header["ownership_supported"]:
                return self._blocked_result(task_id, "task_receipt_unsupported")
            prepared = self._prepared
            if prepared is not None and (prepared.task_id, prepared.source_id, prepared.revision) != (
                    task_id, header["source_id"], header["revision"]):
                self._prepared = prepared = None
            if prepared is None:
                saved = (self.store.terminal_versions((task_id,)) if versions is None else versions).get(task_id)
                if saved is not None:
                    if saved["error_code"]:
                        return self._blocked_result(task_id, saved["error_code"])
                    if saved["source_id"] != header["source_id"]:
                        return self._blocked_result(task_id, "agent_terminal_source_conflict")
                    if saved["revision"] >= header["revision"]:
                        return "done"
                snapshot = self._snapshot(task_id)
                prepared = self.store.prepare_terminal(task_id, snapshot, source_id=snapshot.receipt_source_id,
                                                       revision=snapshot.receipt_revision)
                if prepared.encoded_bytes > FULL_RESULT_MAX_BYTES:
                    return self._blocked_result(task_id, "agent_payload_too_large")
                self._prepared = prepared
            with self._cv:
                self._attempts += 1
            acknowledgement = self.store.save_prepared_terminal(prepared)
            self._prepared = None
            current = self.source.task_receipt_header(task_id)
            if current["source_id"] != prepared.source_id:
                return self._blocked_result(task_id, "agent_terminal_source_conflict")
            if current["revision"] > acknowledgement["revision"]:
                self.offer(task_id)
            with self._cv:
                self._errors.pop(task_id[:512], None)
            return "done"
        except KeyError:
            if not from_scan and self._prepared is None and self._retry_task != task_id:
                return "done"  # 外部任务通知不是本源记录丢失的证明。
            with self._cv:
                self._source_gap = True
            return self._blocked_result(task_id, "receipt_source_record_missing")
        except GatewayError as error:
            if error.error_code in {"access_denied", "forbidden", "capability_not_allowed"}:
                return "excluded"
            self._prepared = None
            return self._blocked_result(task_id, error.error_code)
        except sqlite3.IntegrityError:
            self._prepared = None
            return self._blocked_result(task_id, "receipt_storage_constraint")
        except (sqlite3.Error, OSError):
            return self._storage_failure(task_id, from_scan=from_scan)
        except Exception:  # noqa: BLE001 - 单项未知依赖错误不能阻塞其他任务的持久化。
            self._prepared = None
            return self._blocked_result(task_id, "receipt_dependency_error")
        finally:
            with self._cv:
                self._in_flight = None
                self._in_flight_since = None
                self._cv.notify_all()

    def _after_process(self, result, *, from_scan):
        if result == "retry":
            return
        self._prepared = None
        with self._cv:
            self._retry_at = 0.0
            self._retry_since = None
            self._retry_task = None
            self._consecutive_failures = 0
            if from_scan:
                self._pass_blocked += result == "blocked"
                self._pass_excluded += result == "excluded"
            elif result in {"blocked", "excluded"}:
                self._scan_required = True
            self._cv.notify_all()

    def _scan_step(self):
        try:
            if self._page:
                header = self._page.popleft()
                result = self._process(header["task_id"], self._page_versions, from_scan=True)
                self._after_process(result, from_scan=True)
                return
            if self._cursor is None:
                self._cursor = self.source.begin_terminal_receipt_scan()
                with self._cv:
                    if self._history_reference is not None and not self._cursor.same_index(self._history_reference):
                        self._source_gap = True
                        self._last_error = "source_index_replaced"
                    self._history_reference = self._cursor
                    self._pass_serial = self._scan_serial
                    self._pass_blocked = self._pass_excluded = 0
                    self._scan_offset, self._scan_end = 0, self._cursor.end
            headers, cursor, error = self.source.page_terminal_receipts(self._cursor, limit=self._page_size)
            if error:
                with self._cv:
                    self._source_gap = True
                    self._last_error = error
                    self._cursor = None
                    self._next_scan = self._clock() + self._scan_interval
                return
            versions = self.store.terminal_versions(tuple(header["task_id"] for header in headers))
            with self._cv:
                self._retry_at = 0.0
                self._retry_since = None
                self._consecutive_failures = 0
            self._cursor = cursor
            self._page_versions = versions
            self._page.extend(headers)
            with self._cv:
                self._scan_offset = cursor.offset
            if headers:
                return
            source_id, generation, _ = self.source.receipt_source_status()
            with self._cv:
                stable = (source_id == cursor.source_id and generation == cursor.generation
                          and self._scan_serial == self._pass_serial)
                if stable:
                    self._covered_generation = generation
                    self._blocked, self._excluded = self._pass_blocked, self._pass_excluded
                    self._scan_required = False
                else:
                    self._scan_required = True
                self._cursor = None
                self._next_scan = self._clock() + (self._scan_interval if stable else 0.0 if self._close_requested and self._source_fenced else 0.01)
                self._cv.notify_all()
        except (sqlite3.Error, OSError):
            self._storage_failure(None, from_scan=True)
        except Exception:  # noqa: BLE001 - 源扫描失败保持显式待恢复状态。
            self._storage_failure(None, from_scan=True)

    def _run(self):
        work_since_scan = 0
        while True:
            with self._cv:
                now = self._clock()
                if self._retry_at > now:
                    self._cv.wait(min(0.25, self._retry_at - now))
                    continue
                task_id, from_scan = self._retry_task, self._retry_from_scan
                scan_due = bool(self._cursor or self._page or self._scan_required and now >= self._next_scan)
                if task_id is None and self._mailbox and (work_since_scan < 8 or not scan_due):
                    task_id, (_, size) = self._mailbox.popitem(last=False)
                    self._id_bytes -= size
                    from_scan = False
                do_scan = task_id is None and scan_due
            if task_id is not None:
                self._after_process(self._process(task_id, from_scan=from_scan), from_scan=from_scan)
                work_since_scan += 1
                continue
            if do_scan:
                self._scan_step()
                work_since_scan = 0
                continue
            try:
                observed = self.source.begin_terminal_receipt_scan()
                generation = observed.generation
            except Exception:  # noqa: BLE001 - 无源时不能伪称完整。
                self._storage_failure(None, from_scan=True)
                continue
            with self._cv:
                now = self._clock()
                if self._history_reference is not None and not observed.same_index(self._history_reference):
                    self._source_gap = True
                    self._scan_required = True
                stable = generation == self._covered_generation and not self._scan_required
                if self._close_requested and stable and not self._mailbox and not self._retry_task:
                    self._owner_closed = True
                    self._cv.notify_all()
                    return
                if now >= self._next_scan:
                    if not stable or self._blocked or self._excluded:
                        self._scan_required = True
                    else:
                        self._next_scan = now + self._scan_interval
                    if self._scan_required:
                        continue
                self._cv.wait(min(0.25, max(0.001, self._next_scan - now)))

    def status(self):
        try:
            _, generation, retained = self.source.receipt_source_status() if self._supported else (None, None, None)
        except Exception:  # noqa: BLE001 - 健康读取不因源丢失抛错或猜测完整。
            generation, retained = None, None
        with self._cv:
            pending = len(self._mailbox) + bool(self._in_flight or self._retry_task) + len(self._page)
            coverage = (self._supported and generation == self._covered_generation and not self._scan_required
                        and not pending and not self._source_gap and not self._retry_at)
            persisted = coverage and self._blocked == 0
            prepared = self._prepared
            first = next(iter(self._mailbox.values()), None)
            pending_times = [value for value in (first[0] if first else None, self._retry_since, self._in_flight_since)
                             if value is not None]
            return {"state": "closed" if self._owner_closed else "draining" if self._close_requested else "running",
                    "owner_closed": self._owner_closed, "source_fenced": self._source_fenced,
                    "persistence_complete_at_observed_fence": persisted,
                    "complete": self._owner_closed and self._source_fenced and persisted,
                    "source_generation": generation, "covered_generation": self._covered_generation,
                    "retained_terminal_count": retained, "known_pending": pending + self._blocked,
                    "coverage_scope": "currently_agent_exposed_terminal_tasks",
                    "all_retained_terminal_receipts_complete": persisted and self._excluded == 0,
                    "pending_count_complete": coverage, "mailbox_count": len(self._mailbox),
                    "mailbox_capacity": self._capacity, "mailbox_id_bytes": self._id_bytes,
                    "mailbox_id_byte_limit": self._id_bytes_limit, "high_water": self._high_water,
                    "overflow_count": self._overflow, "late_hints": self._late_hints,
                    "scan_required": self._scan_required, "scan_offset": self._scan_offset, "scan_end": self._scan_end,
                    "source_gap": self._source_gap, "blocked_count": self._blocked,
                    "policy_excluded_count": self._excluded, "error_sample_count": len(self._errors),
                    "retry_at": self._retry_at or None, "attempts": self._attempts, "failures": self._failures,
                    "last_error_code": self._last_error, "in_flight": self._in_flight is not None,
                    "prepared_bytes": prepared.encoded_bytes if prepared is not None else 0,
                    "oldest_known_pending_age_sec": None if not pending_times else max(0.0, self._clock() - min(pending_times)),
                    "reconciliation_restarts": self._restarts}

    def close(self, timeout_sec=2.0, *, source_fenced=False):
        deadline = monotonic() + max(0.0, timeout_sec)
        before = self.status()
        with self._cv:
            self._source_fenced = self._source_fenced or source_fenced
            satisfied = before["owner_closed"] and before["persistence_complete_at_observed_fence"]
            if self._supported and not satisfied:
                self._close_requested = True
                if not self._owner_closed or self._restarts < 1:
                    self._scan_required = True
                    self._scan_serial += 1
                    self._next_scan = self._clock()
                    if self._thread is None:
                        self._start_locked()
                    elif self._owner_closed:
                        self._restarts += 1
                        self._start_locked()
                else:
                    self._last_error = "receipt_reconciliation_restart_exhausted"
                self._cv.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(max(0.0, deadline - monotonic()))
        return self.status()
