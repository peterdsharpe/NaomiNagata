// Tutorial: Guns
// Destroy the asteroid.
use crate::BULLET_SPEED;
use crate::pid::Pid;
use crate::tracked_target::TrackedTarget;
use oort_api::prelude::*;

pub struct Ship {
    pid: Pid,
    targets: Vec<TrackedTarget>,
}

impl Ship {
    pub fn new() -> Ship {
        // PID gains tuned empirically for stable heading control.
        let heading_pid = Pid::new(8.0, 0.0, 5.0);
        let targets = Vec::new();
        Ship {
            pid: heading_pid,
            targets,
        }
    }

    pub fn tick(&mut self) {
        // --- Update targets ---
        for tgt in &mut self.targets {
            tgt.tick();
        }

        // --- Radar scheduling (stub) ---
        if self.targets.is_empty() || current_tick() % 15 == 0 {
            debug!("Broad radar scan (90° arc)");
            // set_radar_heading(rand(0.0, 2.0 * PI));
            set_radar_heading((current_tick() as f64) / 10.0);
            set_radar_width(PI / 8.0);
            scan();
        } else if let Some(best_radar) = self
            .targets
            .iter()
            .max_by(|a, b| a.radar_priority.total_cmp(&b.radar_priority))
        {
            let dir = (best_radar.r - position()).angle();
            debug!("Narrow radar scan towards {} rad", dir);
            // TODO: call narrow_scan_api(dir);
        }

        // --- Choose target to engage ---
        let maybe_best_fire = self
            .targets
            .iter()
            .filter(|t| t.time_to_intercept.is_some() && t.intercept_point.is_some())
            .max_by(|a, b| a.firing_priority.total_cmp(&b.firing_priority));

        let Some(best) = maybe_best_fire else { return }; // nothing to shoot

        // Compute firing solution relative to ship.
        let (t_impact, intercept_point) = match (best.time_to_intercept, best.intercept_point) {
            (Some(t), Some(p)) => (t, p),
            _ => return, // Should be unreachable due to earlier filter, but be safe.
        };
        let aim_point_rel = intercept_point - position();
        let shot_distance = BULLET_SPEED * t_impact;

        draw_diamond(position() + aim_point_rel, 10.0, 0x00ff00);
        draw_line(
            position(),
            position() + vec2(heading().cos(), heading().sin()) * BULLET_SPEED * t_impact,
            0x00ff00,
        );

        let aim_angle = aim_point_rel.angle();
        let heading_rel_error = angle_diff(heading(), aim_angle);

        // --- PID heading control ---
        let control = self.pid.update(heading_rel_error, TICK_LENGTH);
        torque(control);

        if heading_rel_error.abs() * shot_distance < 10.0 {
            fire(0);
        }

        accelerate(1000.0 * aim_point_rel);
    }
}
