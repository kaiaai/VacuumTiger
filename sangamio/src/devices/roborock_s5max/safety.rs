//! Actuator target state, freshness gate, and drive deadman lease.

use super::lifecycle::{DriverState, Lifecycle};
use crate::error::{Error, Result};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub(crate) const DRIVE_LEASE_DURATION: Duration = Duration::from_millis(250);

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ActuatorTargets {
    pub linear: f32,
    pub angular: f32,
    pub fan: u8,
    pub main_brush: u8,
    pub side_brush: u8,
    pub water_pump: u8,
    pub lds_running: bool,
}

impl Default for ActuatorTargets {
    fn default() -> Self {
        Self {
            linear: 0.0,
            angular: 0.0,
            fan: 0,
            main_brush: 0,
            side_brush: 0,
            water_pump: 0,
            lds_running: false,
        }
    }
}

struct SafetyInner {
    targets: ActuatorTargets,
    drive_lease_deadline: Option<Instant>,
}

#[derive(Clone)]
pub(crate) struct SafetyController {
    lifecycle: Lifecycle,
    inner: Arc<Mutex<SafetyInner>>,
}

impl SafetyController {
    pub(crate) fn new(lifecycle: Lifecycle) -> Self {
        Self {
            lifecycle,
            inner: Arc::new(Mutex::new(SafetyInner {
                targets: ActuatorTargets::default(),
                drive_lease_deadline: None,
            })),
        }
    }

    pub(crate) fn reset_targets(&self) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.targets = ActuatorTargets::default();
            inner.drive_lease_deadline = None;
        } else {
            self.lifecycle
                .fault("safety mutex poisoned while resetting actuator targets");
        }
    }

    pub(crate) fn nonzero_allowed(&self, allow_actuation: bool, now: Instant) -> Result<()> {
        if !allow_actuation {
            return Err(Error::InvalidParameter(
                "S5 Max actuation is disabled by configuration".to_string(),
            ));
        }
        if self.lifecycle.state() != DriverState::Operational {
            return Err(Error::Other(
                "S5 Max is not in the operational lifecycle state".to_string(),
            ));
        }
        if !self.lifecycle.safety_reports_fresh(now) {
            return Err(Error::Other(
                "S5 Max safety telemetry is stale; nonzero target rejected".to_string(),
            ));
        }
        Ok(())
    }

    pub(crate) fn arm_drive_lease(&self, linear: f32, angular: f32, now: Instant) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| Error::MutexPoisoned("S5 Max safety".to_string()))?;
        inner.targets.linear = linear;
        inner.targets.angular = angular;
        inner.drive_lease_deadline = if linear == 0.0 && angular == 0.0 {
            None
        } else {
            Some(now + DRIVE_LEASE_DURATION)
        };
        Ok(())
    }

    pub(crate) fn clear_drive_target(&self) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| Error::MutexPoisoned("S5 Max safety".to_string()))?;
        inner.targets.linear = 0.0;
        inner.targets.angular = 0.0;
        inner.drive_lease_deadline = None;
        Ok(())
    }

    fn set_target(&self, update: impl FnOnce(&mut ActuatorTargets)) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| Error::MutexPoisoned("S5 Max safety".to_string()))?;
        update(&mut inner.targets);
        Ok(())
    }

    pub(crate) fn set_fan(&self, target: u8) -> Result<()> {
        self.set_target(|targets| targets.fan = target)
    }

    pub(crate) fn set_main_brush(&self, target: u8) -> Result<()> {
        self.set_target(|targets| targets.main_brush = target)
    }

    pub(crate) fn set_side_brush(&self, target: u8) -> Result<()> {
        self.set_target(|targets| targets.side_brush = target)
    }

    pub(crate) fn set_water_pump(&self, target: u8) -> Result<()> {
        self.set_target(|targets| targets.water_pump = target)
    }

    pub(crate) fn set_lds_running(&self, running: bool) -> Result<()> {
        self.set_target(|targets| targets.lds_running = running)
    }

    pub(crate) fn take_expired_drive_lease(&self, now: Instant) -> bool {
        let Ok(mut inner) = self.inner.lock() else {
            self.lifecycle
                .fault("safety mutex poisoned while checking drive lease");
            return true;
        };
        if inner
            .drive_lease_deadline
            .is_some_and(|deadline| now >= deadline)
        {
            inner.targets.linear = 0.0;
            inner.targets.angular = 0.0;
            inner.drive_lease_deadline = None;
            return true;
        }
        false
    }

    #[cfg(test)]
    fn targets(&self) -> ActuatorTargets {
        self.inner.lock().unwrap().targets
    }
}

#[cfg(test)]
mod tests {
    use super::{ActuatorTargets, DRIVE_LEASE_DURATION, SafetyController};
    use crate::devices::roborock_s5max::lifecycle::Lifecycle;
    use std::time::{Duration, Instant};

    #[test]
    fn targets_start_and_reset_to_zero() {
        let lifecycle = Lifecycle::new();
        let safety = SafetyController::new(lifecycle);
        assert_eq!(safety.targets(), ActuatorTargets::default());
        let now = Instant::now();
        safety.arm_drive_lease(0.1, -0.2, now).unwrap();
        assert!(
            !safety.take_expired_drive_lease(now + DRIVE_LEASE_DURATION - Duration::from_millis(1))
        );
        assert!(safety.take_expired_drive_lease(now + DRIVE_LEASE_DURATION));
        assert_eq!(safety.targets(), ActuatorTargets::default());
    }

    #[test]
    fn zero_drive_target_does_not_arm_deadman() {
        let lifecycle = Lifecycle::new();
        let safety = SafetyController::new(lifecycle);
        let now = Instant::now();

        safety.arm_drive_lease(0.0, 0.0, now).unwrap();

        assert!(!safety.take_expired_drive_lease(now + DRIVE_LEASE_DURATION));
        assert_eq!(safety.targets(), ActuatorTargets::default());
    }

    #[test]
    fn nonzero_gate_requires_config_state_and_fresh_telemetry() {
        let lifecycle = Lifecycle::new();
        let safety = SafetyController::new(lifecycle.clone());
        let now = Instant::now();
        assert!(safety.nonzero_allowed(false, now).is_err());
        assert!(safety.nonzero_allowed(true, now).is_err());
        lifecycle.begin_synchronizing().unwrap();
        lifecycle.mark_read_only_ready().unwrap();
        lifecycle.mark_operational().unwrap();
        assert!(safety.nonzero_allowed(true, now).is_err());
        lifecycle.mark_safety_report(now);
        assert!(safety.nonzero_allowed(true, now).is_ok());
    }
}
