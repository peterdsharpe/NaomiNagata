// Tutorial: Guns
// Destroy the asteroid.
use crate::pid::Pid;
use oort_api::prelude::*;

const BULLET_SPEED: f64 = 1000.0; // m/s

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
fn firing_solution(r_rel: Vec2, v_rel: Vec2, bullet_speed: f64) -> Option<(f64, Vec2)> {
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

pub struct Ship {
    pid: Pid,
}

impl Ship {
    pub fn new() -> Ship {
        // PID gains tuned empirically for stable heading control.
        let pid = Pid::new(8.0, 0.0, 5.0);
        Ship { pid }
    }

    pub fn tick(&mut self) {
        let r_rel = target() - position();
        let v_rel = target_velocity() - velocity();

        draw_diamond(target(), 50.0, 0xff0000);

        // Compute firing solution in the ship-centred frame.
        let (t, aim_point) = match firing_solution(r_rel, v_rel, BULLET_SPEED) {
            Some(sol) => sol,
            None => return,
        };
        let shot_distance = BULLET_SPEED * t;

        debug!("t: {}", t);
        draw_diamond(position() + aim_point, 10.0, 0x00ff00);
        draw_line(
            position(),
            position() + vec2(heading().cos(), heading().sin()) * BULLET_SPEED * t,
            0x00ff00,
        );

        let aim_angle = aim_point.angle();

        let bearing_error = angle_diff(heading(), aim_angle);

        // --- PID heading control ---
        let control = self.pid.update(bearing_error, TICK_LENGTH);
        torque(control);

        // Fire
        if bearing_error.abs() * shot_distance < 10.0 {
            fire(0);
        }

        accelerate(1000.0 * r_rel);
    }
}
