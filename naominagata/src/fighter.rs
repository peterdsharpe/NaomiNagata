// Tutorial: Guns
// Destroy the asteroid.
use crate::BULLET_SPEED;
use crate::pid::Pid;
use crate::target::Target;
use oort_api::prelude::*;

pub struct Ship {
    pid: Pid,
    target: Target,
}

impl Ship {
    pub fn new() -> Ship {
        // PID gains tuned empirically for stable heading control.
        let pid = Pid::new(8.0, 0.0, 5.0);
        let target = Target::new(vec2(0.0, 0.0), vec2(0.0, 0.0), vec2(0.0, 0.0));
        Ship { pid, target }
    }

    pub fn tick(&mut self) {
        self.target
            .update_state(target(), target_velocity(), vec2(0.0, 0.0));
        self.target.update_firing_solution();

        draw_diamond(target(), 50.0, 0xff0000);

        // Compute firing solution in the ship-centred frame.
        if self.target.time_to_intercept.is_none() {
            // No firing solution
            return;
        }
        let t = self.target.time_to_intercept.unwrap();
        let aim_point_rel = self.target.intercept_point.unwrap() - position();
        let shot_distance = BULLET_SPEED * t;

        debug!("t: {}", t);
        draw_diamond(position() + aim_point_rel, 10.0, 0x00ff00);
        draw_line(
            position(),
            position() + vec2(heading().cos(), heading().sin()) * BULLET_SPEED * t,
            0x00ff00,
        );

        let aim_angle = aim_point_rel.angle();

        let heading_rel_error = angle_diff(heading(), aim_angle);

        // --- PID heading control ---
        let control = self.pid.update(heading_rel_error, TICK_LENGTH);
        torque(control);

        // Fire
        if heading_rel_error.abs() * shot_distance < 10.0 {
            fire(0);
        }

        accelerate(1000.0 * aim_point_rel);
    }
}
