//! Explicit S5 Max driver lifecycle and telemetry-freshness state.

use crate::core::types::{SensorGroupData, SensorValue};
use crate::error::{Error, Result};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub(crate) const SAFETY_REPORT_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DriverState {
    Created,
    Synchronizing,
    ReadOnlyReady,
    Operational,
    Stopping,
    Stopped,
    Fault,
}

impl DriverState {
    const fn name(self) -> &'static str {
        match self {
            Self::Created => "created",
            Self::Synchronizing => "synchronizing",
            Self::ReadOnlyReady => "read_only_ready",
            Self::Operational => "operational",
            Self::Stopping => "stopping",
            Self::Stopped => "stopped",
            Self::Fault => "fault",
        }
    }
}

struct LifecycleInner {
    state: DriverState,
    last_fault: Option<String>,
    last_safety_report: Option<Instant>,
}

#[derive(Clone)]
pub(crate) struct Lifecycle {
    inner: Arc<Mutex<LifecycleInner>>,
    sensor_data: Arc<Mutex<SensorGroupData>>,
}

impl Lifecycle {
    pub(crate) fn new() -> Self {
        let sensor_data = Arc::new(Mutex::new(SensorGroupData::new("driver_status")));
        let lifecycle = Self {
            inner: Arc::new(Mutex::new(LifecycleInner {
                state: DriverState::Created,
                last_fault: None,
                last_safety_report: None,
            })),
            sensor_data,
        };
        lifecycle.publish(DriverState::Created, None);
        lifecycle
    }

    pub(crate) fn sensor_data(&self) -> Arc<Mutex<SensorGroupData>> {
        Arc::clone(&self.sensor_data)
    }

    pub(crate) fn state(&self) -> DriverState {
        self.inner
            .lock()
            .map(|inner| inner.state)
            .unwrap_or(DriverState::Fault)
    }

    pub(crate) fn begin_synchronizing(&self) -> Result<()> {
        let current = self.state();
        if !matches!(current, DriverState::Created | DriverState::Stopped) {
            return Err(Error::Other(format!(
                "S5 Max cannot initialize from lifecycle state {}",
                current.name()
            )));
        }
        self.set_state(DriverState::Synchronizing)
    }

    pub(crate) fn mark_read_only_ready(&self) -> Result<()> {
        self.transition(DriverState::Synchronizing, DriverState::ReadOnlyReady)
    }

    pub(crate) fn mark_operational(&self) -> Result<()> {
        self.transition(DriverState::ReadOnlyReady, DriverState::Operational)
    }

    pub(crate) fn begin_stopping(&self) -> Result<()> {
        match self.state() {
            DriverState::Created | DriverState::Stopped => Ok(()),
            DriverState::Fault => Ok(()),
            DriverState::Synchronizing | DriverState::ReadOnlyReady | DriverState::Operational => {
                self.set_state(DriverState::Stopping)
            }
            DriverState::Stopping => Ok(()),
        }
    }

    pub(crate) fn mark_stopped(&self) -> Result<()> {
        match self.state() {
            DriverState::Created | DriverState::Stopped => self.set_state(DriverState::Stopped),
            DriverState::Stopping => self.set_state(DriverState::Stopped),
            DriverState::Fault => Ok(()),
            state => Err(Error::Other(format!(
                "S5 Max cannot enter stopped from {}",
                state.name()
            ))),
        }
    }

    pub(crate) fn fault(&self, reason: impl Into<String>) {
        let reason = reason.into();
        if let Ok(mut inner) = self.inner.lock() {
            inner.state = DriverState::Fault;
            inner.last_fault = Some(reason.clone());
        } else {
            log::error!("S5 Max lifecycle mutex poisoned while recording fault: {reason}");
        }
        self.publish(DriverState::Fault, Some(&reason));
    }

    pub(crate) fn mark_safety_report(&self, now: Instant) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.last_safety_report = Some(now);
        } else {
            self.fault("lifecycle mutex poisoned while recording safety telemetry");
        }
    }

    pub(crate) fn safety_reports_fresh(&self, now: Instant) -> bool {
        self.inner
            .lock()
            .ok()
            .and_then(|inner| inner.last_safety_report)
            .is_some_and(|last| now.saturating_duration_since(last) <= SAFETY_REPORT_TIMEOUT)
    }

    fn transition(&self, from: DriverState, to: DriverState) -> Result<()> {
        let current = self.state();
        if current != from {
            return Err(Error::Other(format!(
                "invalid S5 Max lifecycle transition {} -> {}; expected {}",
                current.name(),
                to.name(),
                from.name()
            )));
        }
        self.set_state(to)
    }

    fn set_state(&self, state: DriverState) -> Result<()> {
        let fault = {
            let mut inner = self
                .inner
                .lock()
                .map_err(|_| Error::MutexPoisoned("S5 Max lifecycle".to_string()))?;
            inner.state = state;
            if state == DriverState::Synchronizing {
                inner.last_fault = None;
                inner.last_safety_report = None;
            }
            inner.last_fault.clone()
        };
        self.publish(state, fault.as_deref());
        Ok(())
    }

    fn publish(&self, state: DriverState, fault: Option<&str>) {
        let Ok(mut data) = self.sensor_data.lock() else {
            log::error!("S5 Max driver-status sensor mutex poisoned");
            return;
        };
        data.set("state", SensorValue::String(state.name().to_string()));
        data.set("faulted", SensorValue::Bool(state == DriverState::Fault));
        if let Some(fault) = fault {
            data.set("last_fault", SensorValue::String(fault.to_string()));
        }
        data.touch();
    }
}

#[cfg(test)]
mod tests {
    use super::{DriverState, Lifecycle, SAFETY_REPORT_TIMEOUT};
    use std::time::{Duration, Instant};

    #[test]
    fn enforces_the_explicit_lifecycle() {
        let lifecycle = Lifecycle::new();
        assert_eq!(lifecycle.state(), DriverState::Created);
        assert!(lifecycle.mark_operational().is_err());
        lifecycle.begin_synchronizing().unwrap();
        lifecycle.mark_read_only_ready().unwrap();
        lifecycle.mark_operational().unwrap();
        lifecycle.begin_stopping().unwrap();
        lifecycle.mark_stopped().unwrap();
        assert_eq!(lifecycle.state(), DriverState::Stopped);
    }

    #[test]
    fn tracks_fresh_safety_reports_and_fault_reason() {
        let lifecycle = Lifecycle::new();
        let now = Instant::now();
        assert!(!lifecycle.safety_reports_fresh(now));
        lifecycle.mark_safety_report(now);
        assert!(lifecycle.safety_reports_fresh(now + SAFETY_REPORT_TIMEOUT));
        assert!(
            !lifecycle.safety_reports_fresh(now + SAFETY_REPORT_TIMEOUT + Duration::from_millis(1))
        );

        lifecycle.fault("test fault");
        assert_eq!(lifecycle.state(), DriverState::Fault);
        let data = lifecycle.sensor_data.lock().unwrap();
        assert!(matches!(
            data.values.get("last_fault"),
            Some(crate::core::types::SensorValue::String(value)) if value == "test fault"
        ));
    }
}
