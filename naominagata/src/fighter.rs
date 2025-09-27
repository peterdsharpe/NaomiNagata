// Tutorial: Guns
// Destroy the asteroid.
use oort_api::prelude::*;

const BULLET_SPEED: f64 = 1000.0;  // m/s

pub struct Ship {}

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }

    // Uncomment me, then press Ctrl-Enter (Cmd-Enter on Mac) to upload the code.
    pub fn tick(&mut self) {
        
        let r_rel = target() - position();
        let v_rel = target_velocity() - velocity();

        draw_diamond(target(), 50.0, 0xff0000);
        
        // Set up and solve the quadratic equation for the time of impact
        let a = v_rel.dot(v_rel) - BULLET_SPEED * BULLET_SPEED;
        let b = 2.0 * v_rel.dot(r_rel);
        let c = r_rel.dot(r_rel);

        let discriminant = b * b - 4.0 * a * c;
        
        if discriminant < 0.0 {
            return;
        }
        let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
        let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
        let t = if t1 > 0.0 && t2 > 0.0 {
            t1.min(t2)
        } else if t1 > 0.0 {
            t1
        } else if t2 > 0.0 {
            t2
        } else {
            return;
        };

        debug!("t: {}" , t);

        let aim_point = r_rel + v_rel * t;
        draw_diamond(aim_point, 10.0, 0x00ff00);

        let aim_angle = aim_point.angle();

        let bearing = angle_diff(heading(), aim_angle);

        // // --- PID heading control ---
        // let control = self.pid.update(bearing, TICK_LENGTH);
        // turn(control);

        // if t < 1.0 {
        //     fire(0);
        // }
        
        

    }
}
