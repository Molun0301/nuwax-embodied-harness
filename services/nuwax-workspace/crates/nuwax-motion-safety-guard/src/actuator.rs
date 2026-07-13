//! Exclusive actuator boundary owned by the motion-guard process.

use thiserror::Error;

use crate::wire::{
    submit_motion_command_request::Command as MotionCommand, ModeSwitchCommand, SportActionCommand,
};

/// A backend error is never converted into a successful acknowledgement.
#[derive(Debug, Clone, Error)]
#[error("{code}: {message}")]
pub struct ActuatorError {
    /// Stable machine-readable error code.
    pub code: &'static str,
    /// Operator-facing detail.
    pub message: String,
}

impl ActuatorError {
    /// Constructs an actuator failure.
    #[must_use]
    pub fn new(code: &'static str, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }
}

/// Sole actuator interface. Production implementations must live in the guard
/// process and report the locked actuator-symbol catalog hash.
pub trait ActuatorBackend: Send {
    /// Whether this backend represents a real configured actuator path.
    fn is_configured(&self) -> bool;

    /// SHA-256 of the actuator-symbol catalog used to build the backend.
    fn actuator_symbols_sha256(&self) -> Option<&str>;

    /// Applies a normal motion command.
    fn apply_motion(&mut self, command: &MotionCommand) -> Result<(), ActuatorError>;

    /// Applies a sport command.
    fn execute_sport(&mut self, command: &SportActionCommand) -> Result<(), ActuatorError>;

    /// Drains pending work and obtains a confirmed actuator zero/StopMove ack.
    fn zero(&mut self) -> Result<(), ActuatorError>;

    /// Switches mode only after a successful zero acknowledgement.
    fn switch_mode(&mut self, command: &ModeSwitchCommand) -> Result<(), ActuatorError>;
}

/// Production-safe boundary used until the official Go2 backend is wired.
/// It deliberately refuses arming, leases, and every actuator operation.
#[derive(Debug, Default)]
pub struct UnavailableActuatorBackend;

impl ActuatorBackend for UnavailableActuatorBackend {
    fn is_configured(&self) -> bool {
        false
    }

    fn actuator_symbols_sha256(&self) -> Option<&str> {
        None
    }

    fn apply_motion(&mut self, _command: &MotionCommand) -> Result<(), ActuatorError> {
        Err(unavailable())
    }

    fn execute_sport(&mut self, _command: &SportActionCommand) -> Result<(), ActuatorError> {
        Err(unavailable())
    }

    fn zero(&mut self) -> Result<(), ActuatorError> {
        Err(unavailable())
    }

    fn switch_mode(&mut self, _command: &ModeSwitchCommand) -> Result<(), ActuatorError> {
        Err(unavailable())
    }
}

fn unavailable() -> ActuatorError {
    ActuatorError::new(
        "ACTUATOR_BACKEND_UNAVAILABLE",
        "the official Go2 actuator backend is not configured in this process",
    )
}
