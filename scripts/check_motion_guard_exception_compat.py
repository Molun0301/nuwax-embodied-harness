"""Offline exception-chain regressions; never connect to a motion guard."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.navigation import motion_guard_client as candidate

baseline_path = os.environ.get("NEH_MOTION_GUARD_BASELINE")
if baseline_path:
    spec = importlib.util.spec_from_file_location("_motion_guard_compat_baseline", baseline_path)
    guard = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = guard
    spec.loader.exec_module(guard)
else:
    guard = candidate


class ExceptionCompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.socket_guard = patch("socket.socket", side_effect=AssertionError("real socket forbidden"))
        self.socket_guard.start()
        self.addCleanup(self.socket_guard.stop)
        with patch.object(guard.MotionGuardClient, "_connect", return_value=None):
            self.client = guard.MotionGuardClient(monotonic_ns=lambda: 1000000000)
        self.lease = guard.pb.LeaseTuple(generation=1, boot_nonce=b"b" * 16, lease_id=7)

    def session(self):
        return guard.MotionGuardSession(client=self.client,
            source=guard.pb.GUARD_SOURCE_GATEWAY_RELATIVE, lease_tuple=self.lease,
            renew_interval_seconds=0.1)

    def context_case(self, is_session, body_fails, close_fails):
        owner = self.session() if is_session else self.client
        body_error = ValueError("body failure") if body_fails else None
        close_error = guard.MotionGuardError("close fence failure") if close_fails else None
        with patch.object(owner, "close", side_effect=close_error) as close:
            if body_error is None and close_error is None:
                with owner as entered:
                    self.assertIs(entered, owner)
            else:
                expected = close_error if close_error is not None else body_error
                with self.assertRaises(type(expected)) as caught:
                    with owner:
                        if body_error is not None:
                            raise body_error
                self.assertIs(caught.exception, expected)
                self.assertIs(caught.exception.__cause__, body_error if close_fails else None)
            close.assert_called_once_with()

    def test_client_clean(self): self.context_case(False, False, False)
    def test_client_body_only(self): self.context_case(False, True, False)
    def test_client_close_only(self): self.context_case(False, False, True)
    def test_client_double_failure(self): self.context_case(False, True, True)
    def test_session_clean(self): self.context_case(True, False, False)
    def test_session_body_only(self): self.context_case(True, True, False)
    def test_session_close_only(self): self.context_case(True, False, True)
    def test_session_double_failure(self): self.context_case(True, True, True)

    def acquire_case(self, registration_fails, fence_fails):
        registration_error = guard.MotionGuardSessionClosedError("client closing")
        fence_error = guard.MotionGuardError("registration cleanup fence failure")
        outcome = guard.pb.AcquireLeaseResponse(lease_tuple=self.lease,
            owner_deadline_monotonic_ns=10000000000)
        proof = guard.MotionAdmissionProof(generation=1, boot_nonce=b"b" * 16)
        register_kwargs = ({"side_effect": registration_error} if registration_fails
                           else {"wraps": self.client._register_session})
        with patch.object(self.client, "_exchange", return_value=(outcome, None)) as exchange, \
             patch.object(self.client, "_register_session", **register_kwargs) as register, \
             patch.object(guard.MotionGuardSession, "_fence_without_registration",
                          side_effect=fence_error if fence_fails else None) as fence, \
             patch.object(guard.MotionGuardSession, "_start_renewal") as renewal:
            if registration_fails:
                expected = fence_error if fence_fails else registration_error
                with self.assertRaises(type(expected)) as caught:
                    self.client.acquire_session(guard.pb.GUARD_SOURCE_GATEWAY_RELATIVE, proof)
                self.assertIs(caught.exception, expected)
                self.assertIs(caught.exception.__cause__, registration_error if fence_fails else None)
                fence.assert_called_once_with()
                renewal.assert_not_called()
            else:
                session = self.client.acquire_session(guard.pb.GUARD_SOURCE_GATEWAY_RELATIVE, proof)
                self.assertIsInstance(session, guard.MotionGuardSession)
                self.assertTrue(session._registered)
                self.assertIn(session, self.client._sessions)
                self.assertEqual(session.lease_tuple, self.lease)
                fence.assert_not_called()
                renewal.assert_called_once_with()
            exchange.assert_called_once()
            register.assert_called_once()

    def test_registration_and_fence_failure(self): self.acquire_case(True, True)
    def test_registration_failure_fence_success(self): self.acquire_case(True, False)
    def test_registration_success(self): self.acquire_case(False, False)


if __name__ == "__main__":
    print("Python:", sys.version, "\nModule:", guard.__file__, flush=True)
    unittest.main(verbosity=2)
