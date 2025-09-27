/// PID Controller implementation for scalar signals.
///
/// The controller computes a control effort based on a proportional–integral–derivative
/// (PID) algorithm:
///
/// ```text
/// u(t) = K_p * e(t) + K_i * ∫ e(t) dt + K_d * de(t)/dt
/// ```
///
/// where
/// - `e(t)` is the instantaneous error (set-point – measurement),
/// - `K_p`, `K_i`, `K_d` are the proportional, integral, and derivative gains.
///
/// # Example
/// ```rust
/// use naominagata::Pid;
///
/// let mut pid = Pid::new(1.0, 0.1, 0.01);
/// let control = pid.update(0.05, 0.016); // error = 0.05 rad, dt = 16 ms
/// ```
///
/// The implementation is intentionally minimal: no anti-windup, filtering, or clamping
/// is performed. These can be added in a future refinement.
#[derive(Debug, Clone)]
pub struct Pid {
    /// Proportional gain.
    kp: f64,
    /// Integral gain.
    ki: f64,
    /// Derivative gain.
    kd: f64,

    /// Accumulated integral of the error.
    integral: f64,
    /// Error at the previous update; `None` until the first call.
    prev_error: Option<f64>,
}

impl Pid {
    /// Creates a new [`Pid`] controller with the provided gains.
    #[must_use]
    pub const fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: None,
        }
    }

    /// Resets the internal integral term and derivative memory.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = None;
    }

    /// Updates the controller with the current `error` and time step `dt` (in seconds).
    ///
    /// # Arguments
    /// * `error` - The current error signal.
    /// * `dt` - Time since the previous update in seconds. Must be positive.
    ///
    /// # Returns
    /// The control effort computed from the PID algorithm.
    ///
    /// # Panics
    /// Panics if `dt` is not strictly positive.
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        assert!(dt > 0.0, "dt ({}) must be > 0", dt);

        // Proportional term.
        let p = self.kp * error;

        // Integral term.
        let integral_increment = match self.prev_error {
            Some(prev) => 0.5 * (error + prev) * dt, // Trapezoidal integration
            None => error * dt, // First step: rectangular integration
        };
        self.integral += integral_increment;
        let i = self.ki * self.integral;

        // Derivative term.
        let derivative = match self.prev_error {
            Some(prev) => (error - prev) / dt,
            None => 0.0,
        };
        let d = self.kd * derivative;

        self.prev_error = Some(error);

        p + i + d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_error_gives_zero_output() {
        let mut pid = Pid::new(1.0, 0.5, 0.1);
        let u = pid.update(0.0, 0.01);
        assert_eq!(u, 0.0);
    }

    #[test]
    fn non_zero_error_produces_output() {
        let mut pid = Pid::new(1.0, 0.0, 0.0);
        let u = pid.update(2.0, 0.02);
        assert_eq!(u, 2.0); // purely proportional
    }
}
