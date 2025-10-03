//! Kalman filter
use maths_rs::{mat::Mat4, mat::MatInverse, mat::MatNew4, mat::MatTranspose, vec::Vec4};
use oort_api::prelude::*;

/// Kalman filter for 2D position and velocity tracking
pub struct KalmanPosVel2d {
    /// State vector [x, y, vx, vy]
    x: Vec4<f64>,
    /// State covariance matrix
    p: Mat4<f64>,
    /// State transition matrix
    f: Mat4<f64>,
    /// Process noise covariance matrix
    q: Mat4<f64>,
    /// Measurement matrix
    h: Mat4<f64>,
    /// Measurement noise covariance matrix
    r: Mat4<f64>,
}

impl KalmanPosVel2d {
    pub fn new(
        position_meas_std: f64,
        velocity_meas_std: f64,
        position_process_std: f64,
        velocity_process_std: f64,
    ) -> Self {
        let x = Vec4::new(0.0, 0.0, 0.0, 0.0);
        let p = mat4_from_diag(1e3, 1e3, 1e3, 1e3); // Large initial uncertainty

        let f = Mat4::<f64>::new(
            1.0,
            0.0,
            TICK_LENGTH,
            0.0,
            0.0,
            1.0,
            0.0,
            TICK_LENGTH,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        );

        let q = mat4_from_diag(
            position_process_std.powi(2),
            position_process_std.powi(2),
            velocity_process_std.powi(2),
            velocity_process_std.powi(2),
        );

        let h = Mat4::identity();

        let r = mat4_from_diag(
            position_meas_std.powi(2),
            position_meas_std.powi(2),
            velocity_meas_std.powi(2),
            velocity_meas_std.powi(2),
        );

        Self { x, p, f, q, h, r }
    }

    /// Predict the next state (without measurement update).
    /// Call this on ticks when we do not get a radar measurement of the target.
    pub fn predict(&mut self) {
        self.x = self.f * self.x;
        self.p = mat4_add(&(self.f * self.p * self.f.transpose()), &self.q);
    }

    /// Perform a full predict and update step with the given position and velocity measurements.
    /// Call this on ticks when we get a radar measurement of the target.
    pub fn predict_and_update(&mut self, position_meas: &Vec2, velocity_meas: &Vec2) {
        let z = Vec4::new(
            position_meas.x,
            position_meas.y,
            velocity_meas.x,
            velocity_meas.y,
        ); // Measurement vector

        // Prediction step
        self.predict();

        // Measurement update step
        let y = z - self.h * self.x; // Measurement residual
        let s = mat4_add(&(self.h * self.p * self.h.transpose()), &self.r); // Residual covariance
        let k = self.p * self.h.transpose() * s.inverse(); // Kalman gain

        self.x += k * y;
        let i = Mat4::<f64>::identity();
        let i_minus_kh = mat4_sub(&i, &(k * self.h));
        self.p = mat4_add(
            &(i_minus_kh * self.p * i_minus_kh.transpose()),
            &(k * self.r * k.transpose()),
        );
    }

    /// Get the latest position estimate
    pub fn position(&self) -> Vec2 {
        Vec2::new(self.x.x, self.x.y)
    }

    /// Get the latest velocity estimate
    pub fn velocity(&self) -> Vec2 {
        Vec2::new(self.x.z, self.x.w)
    }
}

/// addition for Mat4
pub fn mat4_add(left: &Mat4<f64>, right: &Mat4<f64>) -> Mat4<f64> {
    Mat4::<f64> {
        m: [
            left.m[0] + right.m[0],
            left.m[1] + right.m[1],
            left.m[2] + right.m[2],
            left.m[3] + right.m[3],
            left.m[4] + right.m[4],
            left.m[5] + right.m[5],
            left.m[6] + right.m[6],
            left.m[7] + right.m[7],
            left.m[8] + right.m[8],
            left.m[9] + right.m[9],
            left.m[10] + right.m[10],
            left.m[11] + right.m[11],
            left.m[12] + right.m[12],
            left.m[13] + right.m[13],
            left.m[14] + right.m[14],
            left.m[15] + right.m[15],
        ],
    }
}

/// subtraction for Mat4
pub fn mat4_sub(left: &Mat4<f64>, right: &Mat4<f64>) -> Mat4<f64> {
    Mat4::<f64> {
        m: [
            left.m[0] - right.m[0],
            left.m[1] - right.m[1],
            left.m[2] - right.m[2],
            left.m[3] - right.m[3],
            left.m[4] - right.m[4],
            left.m[5] - right.m[5],
            left.m[6] - right.m[6],
            left.m[7] - right.m[7],
            left.m[8] - right.m[8],
            left.m[9] - right.m[9],
            left.m[10] - right.m[10],
            left.m[11] - right.m[11],
            left.m[12] - right.m[12],
            left.m[13] - right.m[13],
            left.m[14] - right.m[14],
            left.m[15] - right.m[15],
        ],
    }
}

/// Create a diagonal Mat4
pub fn mat4_from_diag(a1: f64, a2: f64, a3: f64, a4: f64) -> Mat4<f64> {
    Mat4::<f64> {
        m: [
            a1, 0.0, 0.0, 0.0, 0.0, a2, 0.0, 0.0, 0.0, 0.0, a3, 0.0, 0.0, 0.0, 0.0, a4,
        ],
    }
}

//unit tests
#[cfg(test)]
mod tests {
    use super::*;
    use oort_api::prelude::vec2;

    #[test]
    fn test_converge_stationary_no_noise() {
        let mut kf = KalmanPosVel2d::new(1.0, 1.0, 0.1, 0.1);
        let x_true = 2.0;
        let y_true = 3.0;

        for _ in 0..10 {
            kf.predict_and_update(&vec2(x_true, y_true), &vec2(0.0, 0.0));
        }

        println!(
            "Final estimate: pos=({:.3}, {:.3}), vel=({:.3}, {:.3})",
            kf.position().x,
            kf.position().y,
            kf.velocity().x,
            kf.velocity().y
        );

        assert!((kf.position().x - x_true).abs() < 0.001);
        assert!((kf.position().y - y_true).abs() < 0.001);
        assert!((kf.velocity().x).abs() < 0.001);
        assert!((kf.velocity().y).abs() < 0.001);
    }

    #[test]
    fn test_convergence_constant_velocity_no_noise() {
        let mut kf = KalmanPosVel2d::new(1.0, 1.0, 0.1, 0.1);

        let x_init = 2.0;
        let y_init = 3.0;
        let vx_true = 10.0;
        let vy_true = 20.0;

        let mut x_true = x_init;
        let mut y_true = y_init;
        for _ in 0..10 {
            x_true += vx_true * TICK_LENGTH;
            y_true += vy_true * TICK_LENGTH;
            kf.predict_and_update(&vec2(x_true, y_true), &vec2(vx_true, vy_true));
        }

        println!(
            "Final estimate: pos=({:.3}, {:.3}), vel=({:.3}, {:.3})",
            kf.position().x,
            kf.position().y,
            kf.velocity().x,
            kf.velocity().y
        );

        assert!((kf.position().x - x_true).abs() < 0.001);
        assert!((kf.position().y - y_true).abs() < 0.001);
        assert!((kf.velocity().x - vx_true).abs() < 0.01);
        assert!((kf.velocity().y - vy_true).abs() < 0.01);
    }
}
