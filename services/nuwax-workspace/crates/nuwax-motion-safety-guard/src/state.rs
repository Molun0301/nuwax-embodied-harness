//! Lease-only motion resource arbitration.
//!
//! The guard starts in `IDLE` after its automatic startup zero succeeds. This
//! reference state machine preserves exclusive lease ownership and fencing but
//! deliberately has no policy gate, latch, or manual-reset state.

use serde::{Deserialize, Serialize};

/// Lease lifecycle persisted by the guard.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum LeaseState {
    /// 无 active lease,可被新 acquire。
    Idle,
    /// 已 acquire,owner 可下发 motion command。
    Active,
    /// pause 路径过渡态。
    Pausing,
    /// revoke 路径过渡态。
    Revoking,
}

impl LeaseState {
    /// §P0B #20:IDLE 文字表达。
    pub const IDLE: Self = Self::Idle;
    /// §P0B #20:ACTIVE 文字表达。
    pub const ACTIVE: Self = Self::Active;
    /// §P0B #20:PAUSING 文字表达。
    pub const PAUSING: Self = Self::Pausing;
    /// §P0B #20:REVOKING 文字表达。
    pub const REVOKING: Self = Self::Revoking;
}

/// Single motion-resource arbitrator.
#[derive(Debug)]
pub struct MotionResourceArbitrator {
    active_lease_id: Option<u64>,
    state: LeaseState,
}

impl Default for MotionResourceArbitrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MotionResourceArbitrator {
    /// Construct an idle, leaseable arbitrator after startup zero confirmation.
    pub fn new() -> Self {
        Self {
            active_lease_id: None,
            state: LeaseState::Idle,
        }
    }

    /// 当前 LeaseState。
    pub fn lease_state(&self) -> LeaseState {
        self.state
    }

    /// 当前 active lease_id;无 active 时为 `None`。
    pub fn active_lease_id(&self) -> Option<u64> {
        self.active_lease_id
    }

    /// Whether a new lease may be acquired.
    pub fn is_leaseable(&self) -> bool {
        self.state == LeaseState::Idle && self.active_lease_id.is_none()
    }

    /// §P0B #21 普通 source 并发 Acquire:已被占则返 `false`。
    ///
    /// An active owner cannot be preempted by another acquire.
    pub fn try_acquire(&mut self, lease_id: u64) -> bool {
        if !self.is_leaseable() || lease_id == 0 {
            return false;
        }
        self.active_lease_id = Some(lease_id);
        self.state = LeaseState::Active;
        true
    }

    /// §P0B #23 续约:仅当 `lease_id` 匹配当前 active 时允许。
    pub fn renew(&self, lease_id: u64) -> bool {
        self.active_lease_id == Some(lease_id) && self.state == LeaseState::Active
    }

    /// Revoke the matching active lease after a correlated zero acknowledgement.
    pub fn revoke_and_zero(&mut self, lease_id: u64) -> (bool, &'static str) {
        if self.active_lease_id != Some(lease_id) {
            return (false, "LEASE_NOT_ACTIVE");
        }
        if self.state != LeaseState::Active {
            return (false, "LEASE_NOT_ACTIVE");
        }
        self.active_lease_id = None;
        self.state = LeaseState::Idle;
        (true, "RESOURCE_FENCED")
    }

    /// Apply a global stop after zero acknowledgement.
    ///
    /// The operation is valid while idle or active. It never creates a
    /// persistent blocker: the terminal state is always `IDLE`.
    pub fn stop_all_motion(&mut self) -> (bool, &'static str) {
        let revoked_active_lease = self.active_lease_id.is_some();
        self.state = LeaseState::Revoking;
        self.active_lease_id = None;
        self.state = LeaseState::Idle;
        if revoked_active_lease {
            (true, "RESOURCE_FENCED")
        } else {
            (false, "ZEROED")
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Lease ownership remains exclusive and fencing returns to idle.
    #[test]
    fn full_state_machine_self_test() {
        let mut arb = MotionResourceArbitrator::new();

        assert_eq!(arb.lease_state(), LeaseState::Idle);
        assert_eq!(arb.active_lease_id(), None);
        assert!(arb.is_leaseable());

        // try_acquire ok
        assert!(arb.try_acquire(42));
        assert_eq!(arb.active_lease_id(), Some(42));
        assert_eq!(arb.lease_state(), LeaseState::Active);
        assert!(!arb.is_leaseable(), "ACTIVE must not be leaseable");

        // 同 lease_id 二次 try_acquire 返 false
        assert!(!arb.try_acquire(42));
        assert!(!arb.try_acquire(99));
        assert_eq!(
            arb.active_lease_id(),
            Some(42),
            "并发 Acquire 不应替换 active"
        );

        // renew 当前 holder
        assert!(arb.renew(42));
        assert!(!arb.renew(99));

        // revoke_and_zero 由当前 holder 触发
        let (ok, code) = arb.revoke_and_zero(42);
        assert!(ok);
        assert_eq!(code, "RESOURCE_FENCED");
        assert_eq!(arb.active_lease_id(), None);
        assert_eq!(arb.lease_state(), LeaseState::Idle);
        assert!(arb.is_leaseable(), "revoke must return to leaseable IDLE");

        // 再次 acquire
        assert!(arb.try_acquire(77));
        assert_eq!(arb.active_lease_id(), Some(77));

        let (revoked, code) = arb.stop_all_motion();
        assert!(revoked);
        assert_eq!(code, "RESOURCE_FENCED");
        assert_eq!(arb.active_lease_id(), None);
        assert_eq!(arb.lease_state(), LeaseState::Idle);
        assert!(arb.is_leaseable());
        assert!(arb.try_acquire(100));
        arb.revoke_and_zero(100);

        // revoke 错 lease_id 拒绝
        assert!(arb.try_acquire(1));
        let (ok, code) = arb.revoke_and_zero(2);
        assert!(!ok);
        assert_eq!(code, "LEASE_NOT_ACTIVE");
    }

    /// Idle global stop is idempotent and does not block the next lease.
    #[test]
    fn idle_global_stop_allows_immediate_reacquire() {
        let mut arb = MotionResourceArbitrator::new();
        let (revoked, code) = arb.stop_all_motion();
        assert!(!revoked);
        assert_eq!(code, "ZEROED");
        assert!(arb.is_leaseable());
        assert!(arb.try_acquire(1));
    }
}
