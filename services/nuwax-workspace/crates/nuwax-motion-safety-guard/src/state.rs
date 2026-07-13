//! nuwax-motion-safety-guard 状态机 + 资源仲裁(§P0B #20/#21/#22)
//!
//! 对齐 W32 Python 端 `tools/motion_safety_guard/state_machine.py`:
//! - `SafetyGate` / `LeaseState` 正交枚举
//! - `LEASEABLE` 派生 = (gate == ARMED) ∧ (lease == IDLE)
//! - `MotionResourceArbitrator` 单一 motion resource 仲裁器:
//!   - `try_acquire` 普通 source 并发入口
//!   - `renew` 250ms 周期续约
//!   - `revoke_and_zero` 授权 fence,arm+idle 不变,不需 admin reset
//!   - `latch_safety_stop` SAFETY_LATCHED → 强制 gate → RESET_REQUIRED,需 admin reset
//!   - `admin_reset` 仅 admin CLI 物理确认可用

use serde::{Deserialize, Serialize};

/// §P0B #20 SafetyGate — 硬件/安全门控,与 lease 正交。
///
/// 决定外部能否对 actuator 进行写入。
#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum SafetyGate {
    /// 默认初始状态;需 admin CLI 物理确认才能进入 ARMED。
    ResetRequired,
    /// 门控 ARMED,与 LeaseState::Idle 同时满足才允许 LEASEABLE。
    Armed,
}

impl SafetyGate {
    /// §P0B #20:RESET_REQUIRED 文字表达。
    pub const RESET_REQUIRED: Self = Self::ResetRequired;
    /// §P0B #20:ARMED 文字表达。
    pub const ARMED: Self = Self::Armed;
}

/// §P0B #20 LeaseState — lease 生命周期,与 SafetyGate 正交。
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

/// §P0B #21 单一 motion resource 仲裁器。
///
/// 三条入边互斥:
/// - `try_acquire(lease_id)` 普通 source Acquire
/// - `revoke_and_zero(lease_id)` 授权 fence
/// - `latch_safety_stop(lease_id, reason)` SAFETY_LATCHED
///
/// 派生判断:`gate_leaseable() = (gate == ARMED) ∧ (lease == IDLE)`。
#[derive(Debug)]
pub struct MotionResourceArbitrator {
    active_lease_id: Option<u64>,
    state: LeaseState,
    gate: SafetyGate,
}

impl Default for MotionResourceArbitrator {
    fn default() -> Self {
        Self::new()
    }
}

impl MotionResourceArbitrator {
    /// §P0B #13 启动后初始状态:RESET_REQUIRED + IDLE,leaseable=False(因 gate)。
    pub fn new() -> Self {
        Self {
            active_lease_id: None,
            state: LeaseState::Idle,
            gate: SafetyGate::ResetRequired,
        }
    }

    /// 当前 SafetyGate。
    pub fn gate(&self) -> SafetyGate {
        self.gate
    }

    /// 当前 LeaseState。
    pub fn lease_state(&self) -> LeaseState {
        self.state
    }

    /// 当前 active lease_id;无 active 时为 `None`。
    pub fn active_lease_id(&self) -> Option<u64> {
        self.active_lease_id
    }

    /// §P0B #20 LEASEABLE 派生判断:ARMED ∧ IDLE。
    pub fn gate_leaseable(&self) -> bool {
        self.gate == SafetyGate::Armed && self.state == LeaseState::Idle
    }

    /// §P0B #21 普通 source 并发 Acquire:已被占则返 `false`。
    ///
    /// 不允许抢占;只有 `revoke_and_zero` 或 `latch_safety_stop` 才能回收。
    pub fn try_acquire(&mut self, lease_id: u64) -> bool {
        if !self.gate_leaseable() || lease_id == 0 {
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

    /// §P0B #21 授权 resource fence:ARMED + IDLE + RESOURCE_FENCED;不需 admin reset。
    ///
    /// 仅允许当前 lease holder 触发;其它 lease_id 返 `false`。
    pub fn revoke_and_zero(&mut self, lease_id: u64) -> (bool, &'static str) {
        if self.active_lease_id != Some(lease_id) {
            return (false, "LEASE_NOT_ACTIVE");
        }
        if self.state != LeaseState::Active {
            return (false, "LEASE_NOT_ACTIVE");
        }
        self.active_lease_id = None;
        self.state = LeaseState::Idle;
        // 注意:SafetyGate 不变,仍保持 ARMED;§#20 正交性
        (true, "RESOURCE_FENCED")
    }

    /// §P0B #21 SAFETY_LATCHED → SafetyGate 强制回 RESET_REQUIRED,需 admin_reset 解锁。
    ///
    /// - 若当前无 active lease,admin 仍可触发空 latch(常见于 watchdog timeout)
    /// - 若有 active lease,仅当 `lease_id` 匹配才允许(防误伤)
    pub fn latch_safety_stop(&mut self, lease_id: u64, reason: &str) -> (bool, &'static str) {
        let _ = reason; // 当前实现仅用于日志/审计,W14 不持久化
        if self.state == LeaseState::Idle {
            // 空 latch:仅落安全门
            self.gate = SafetyGate::ResetRequired;
            return (true, "SAFETY_LATCHED");
        }
        if self.active_lease_id != Some(lease_id) {
            return (false, "LEASE_NOT_ACTIVE");
        }
        self.active_lease_id = None;
        self.state = LeaseState::Idle;
        self.gate = SafetyGate::ResetRequired;
        (true, "SAFETY_LATCHED")
    }

    /// §P0B #20 仅 admin CLI 物理确认可用。
    ///
    /// RESET_REQUIRED → ARMED;已在 ARMED 时返 `false`(防止无意义重复 reset)。
    pub fn admin_reset(&mut self) -> bool {
        if self.gate == SafetyGate::ResetRequired {
            self.gate = SafetyGate::Armed;
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §P0B #20/21 self-test:覆盖全状态机 + 全入边。
    #[test]
    fn full_state_machine_self_test() {
        let mut arb = MotionResourceArbitrator::new();

        // 初始:RESET_REQUIRED + IDLE,leaseable=False(因 gate)
        assert_eq!(arb.gate(), SafetyGate::ResetRequired);
        assert_eq!(arb.lease_state(), LeaseState::Idle);
        assert_eq!(arb.active_lease_id(), None);
        assert!(
            !arb.gate_leaseable(),
            "初始 gate=RESET_REQUIRED,不可 LEASEABLE"
        );

        // admin_reset 解锁 → leaseable=True
        assert!(arb.admin_reset());
        assert_eq!(arb.gate(), SafetyGate::Armed);
        assert!(arb.gate_leaseable(), "ARMED+IDLE → LEASEABLE");

        // try_acquire ok
        assert!(arb.try_acquire(42));
        assert_eq!(arb.active_lease_id(), Some(42));
        assert_eq!(arb.lease_state(), LeaseState::Active);
        assert!(!arb.gate_leaseable(), "ACTIVE → not LEASEABLE");

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
        assert_eq!(arb.gate(), SafetyGate::Armed, "revoke 不改 gate");
        assert!(arb.gate_leaseable(), "revoke 后应回到 LEASEABLE");

        // 再次 acquire
        assert!(arb.try_acquire(77));
        assert_eq!(arb.active_lease_id(), Some(77));

        // latch_safety_stop 强制 gate → RESET_REQUIRED
        let (ok, code) = arb.latch_safety_stop(77, "watchdog timeout");
        assert!(ok);
        assert_eq!(code, "SAFETY_LATCHED");
        assert_eq!(arb.active_lease_id(), None);
        assert_eq!(arb.lease_state(), LeaseState::Idle);
        assert_eq!(arb.gate(), SafetyGate::ResetRequired);
        assert!(!arb.gate_leaseable(), "RESET_REQUIRED → not LEASEABLE");

        // RESET_REQUIRED 下不能 Acquire；admin_reset 后才恢复 leaseable。
        assert!(!arb.try_acquire(100));
        assert!(arb.admin_reset());
        assert!(arb.try_acquire(100));
        arb.revoke_and_zero(100);
        assert!(!arb.admin_reset(), "已在 ARMED,admin_reset 返 false");

        // latch_safety_stop 用错 lease_id 拒绝
        assert!(arb.try_acquire(55));
        let (ok, code) = arb.latch_safety_stop(999, "bad");
        assert!(!ok);
        assert_eq!(code, "LEASE_NOT_ACTIVE");

        // latch 空状态(IDLE 无 active)
        arb.revoke_and_zero(55);
        let (ok, code) = arb.latch_safety_stop(0, "watchdog");
        assert!(ok);
        assert_eq!(code, "SAFETY_LATCHED");
        assert_eq!(arb.gate(), SafetyGate::ResetRequired);

        // admin_reset 解锁
        assert!(arb.admin_reset());
        assert!(arb.gate_leaseable());

        // revoke 错 lease_id 拒绝
        assert!(arb.try_acquire(1));
        let (ok, code) = arb.revoke_and_zero(2);
        assert!(!ok);
        assert_eq!(code, "LEASE_NOT_ACTIVE");
    }

    /// §P0B #20 SafetyGate / LeaseState 正交性 + LEASEABLE 派生。
    #[test]
    fn gate_leaseable_is_orthogonal_derivation() {
        let mut arb = MotionResourceArbitrator::new();
        // 初始:IDLE + RESET_REQUIRED → not leaseable
        assert!(!arb.gate_leaseable());

        // ARMED + IDLE → leaseable
        arb.admin_reset();
        assert!(arb.gate_leaseable());

        // ARMED + ACTIVE → not leaseable
        arb.try_acquire(1);
        assert!(!arb.gate_leaseable());

        // 回 IDLE 后 → leaseable 恢复
        arb.revoke_and_zero(1);
        assert!(arb.gate_leaseable());

        // latch 后 → RESET_REQUIRED → not leaseable
        arb.latch_safety_stop(0, "watchdog");
        assert!(!arb.gate_leaseable());
    }
}
