use crate::BULLET_SPEED;
use crate::mat2::Mat2;
use crate::fighter::Ship;
use oort_api::prelude::*;

const MAX_ITER: usize = 100;

pub struct TrackedTarget {
    pub r: Vec2,
    pub r_cov: Mat2,
    pub v: Vec2,
    pub v_cov: Mat2,
    pub a: Vec2,
    pub a_cov: Mat2,
    pub time_to_intercept: Option<f64>,
    pub intercept_point: Option<Vec2>,
    /// Priority for engaging this target with weapons (computed each tick).
    pub firing_priority: f64,
    /// Priority for allocating radar time to this target (computed each tick).
    pub radar_priority: f64,
}

impl TrackedTarget {
    pub fn new(r: Vec2, v: Vec2, a: Vec2, r_cov: Mat2, v_cov: Mat2, a_cov: Mat2) -> Self {
        Self {
            r,
            v,
            a,
            r_cov,
            v_cov,
            a_cov,
            time_to_intercept: None,
            intercept_point: None,
            firing_priority: 0.0,
            radar_priority: 0.0,
        }
    }

    pub fn tick(&mut self) {
        // State estimate updates
        self.r += self.v * TICK_LENGTH;
        self.v += self.a * TICK_LENGTH;
        self.r_cov = self.r_cov + self.v_cov * TICK_LENGTH;
        self.v_cov = self.v_cov + self.a_cov * TICK_LENGTH;

        draw_diamond(self.r, 10.0, 0x4f78ff);

        self.update_firing_solution();
        self.update_priorities();
    }

    pub fn update_firing_solution(&mut self) {
        let r_rel = self.r - position();
        let v_rel = self.v - velocity();

        let t_guess = match self.time_to_intercept {
            Some(t) => t,
            None => {
                // Initialize with constant velocity solution
                let maybe_guess = firing_solution_const_vel(r_rel, v_rel, BULLET_SPEED);
                if let Some((t, _)) = maybe_guess {
                    t
                } else {
                    // No solution found
                    self.time_to_intercept = None;
                    self.intercept_point = None;
                    return;
                }
            }
        };
        let t = firing_solution_const_accel(
            r_rel,
            v_rel,
            self.a,
            BULLET_SPEED,
            t_guess,
            1e-4,
        );
        if t <= 0.0 {
            // No valid intercept time
            self.time_to_intercept = None;
            self.intercept_point = None;
            return;
        }
        self.time_to_intercept = Some(t);
        self.intercept_point =
            Some(self.r + self.v * t + 0.5 * self.a * t * t);
    }

    pub fn update_priorities(&mut self) {
        // Firing priority: inverse of time‐to‐intercept if a solution exists.
        self.firing_priority = self
            .time_to_intercept
            .map(|t| 1.0 / t.max(1e-6))
            .unwrap_or(0.0);

        // Mahalanobis-based closest possible approach within 2σ uncertainty.
        let r_rel = self.r - position();
        let dist = r_rel.length();
        // Extract covariance entries.
        let a = self.r_cov.xx;
        let b = self.r_cov.xy;
        let d = self.r_cov.yy;
        // Unit vector in relative direction.
        let r_unit = if dist > 0.0 {
            r_rel / dist
        } else {
            vec2(1.0, 0.0)
        };
        let var_radial =
            r_unit.x * (a * r_unit.x + b * r_unit.y) + r_unit.y * (b * r_unit.x + d * r_unit.y);
        let min_possible = (dist - 2.0 * var_radial.sqrt()).max(0.0);

        // Radar priority: higher when firing priority is high AND uncertainty could bring
        // the target close. The +1 avoids division by zero.
        self.radar_priority = self.firing_priority / (min_possible + 1.0);
    }
}

/// Intercept a target moving with constant acceleration, using
/// a constant speed bullet in 2d. Position and velocity are
/// relative to the shooter.
//
/// r, v, a: initial position, velocity, acceleration of target
/// u: bullet velocity, |u| = bullet_speed
/// t: time to intercept
///
/// Governing equation:
///    r + v t + 0.5 a t^2 = u t
fn firing_solution_const_accel(
    r_rel: Vec2,
    v_rel: Vec2,
    a_rel: Vec2,
    bullet_speed: f64,
    t_guess: f64,
    tol: f64,
) -> f64 {
    let p4 = 0.5 * a_rel.dot(a_rel);
    let p3 = v_rel.dot(a_rel);
    let p2 = v_rel.dot(v_rel) + r_rel.dot(a_rel) - bullet_speed * bullet_speed;
    let p1 = 2.0 * r_rel.dot(v_rel);
    let p0 = r_rel.dot(r_rel);

    // Solve at^4 + bt^3 + ct^2 + dt + e = 0
    // This could be solved analytically, but we likely have a good guess from the previous
    // game tick, so finding the root with Newton's method should be faster.
    let mut t = t_guess;
    let mut t_next;
    for _ in 0..MAX_ITER {
        let f = (((p4 * t + p3) * t + p2) * t + p1) * t + p0;
        let df = (4.0 * p4 * t + 3.0 * p3) * t * t + 2.0 * p2 * t + p1;
        if df.abs() < 1e-6 {
            break; // Avoid division by zero
        }
        t_next = t - f / df;
        if (t_next - t).abs() < tol {
            break; // Converged
        }
        t = t_next;
    }
    t
}

/// Computes an intercept firing solution assuming constant velocities for both
/// the ship and the target.
///
/// The calculation solves the classic pursuit problem in 2-D by determining
/// the earliest positive time `t` at which a bullet—shot today at constant
/// speed `bullet_speed`—can meet the target.  If no positive‐time solution
/// exists (i.e. the discriminant is negative or both roots are non-positive),
/// `None` is returned.
///
/// Returns the time‐to‐impact `t` (seconds) together with the **relative** aim
/// point, expressed in the ship-centred coordinate frame (`r_rel + v_rel·t`).
fn firing_solution_const_vel(r_rel: Vec2, v_rel: Vec2, bullet_speed: f64) -> Option<(f64, Vec2)> {
    // Quadratic coefficients for |r_rel + v_rel·t| = bullet_speed·t.
    let a = v_rel.dot(v_rel) - bullet_speed * bullet_speed;
    let b = 2.0 * v_rel.dot(r_rel);
    let c = r_rel.dot(r_rel);

    // Discriminant of the quadratic.
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return None;
    }

    let sqrt_disc = disc.sqrt();
    let (t1, t2) = ((-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a));

    // Earliest positive interception time.
    let t = match (t1 > 0.0, t2 > 0.0) {
        (true, true) => t1.min(t2),
        (true, false) => t1,
        (false, true) => t2,
        _ => return None,
    };

    Some((t, r_rel + v_rel * t))
}
