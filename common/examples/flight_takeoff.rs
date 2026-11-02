//! Why is the takeoff roll slow? Prints the force budget during the ground roll.
use flight_common::*;
use glam::Vec3;

fn main() {
    let t = Terrain::generate(SEED);
    let ac = AIRCRAFT;
    let y = t.surface_height(0.0, -400.0) + ac.gear_height;
    let mut s = PlaneState::spawn(Vec3::new(0.0, y, -400.0), 0.0, 0.0);
    s.grounded = true;
    let c = Controls {
        throttle: 1.0,
        elevator: 0.75,
        ..Default::default()
    };
    let dt = 1.0 / 30.0;
    let mut prev = 0.0f32;
    println!("weight={:.0} N  thrust={:.0} N", ac.mass * G, ac.thrust_max);
    for i in 0..(30 * 40) {
        let (r, out) = step_plane(&mut s, &c, &t, dt, &ac);
        let _a = (r.speed - prev) / dt;
        prev = r.speed;
        if i % 60 == 0 {
            println!(
                "t={} speed={} alpha_deg={} pitch_deg={} cl={} gload={} grounded={} pos={} body={}",
                (i as f32 * dt).round(),
                r.speed.round(),
                deg(r.alpha).round(),
                deg(r.pitch).round(),
                r.cl,
                r.g_load,
                s.grounded,
                s.pos,
                s.body
            );
        }
        if out.took_off {
            println!(
                "ROTATED t={} speed={}",
                (i as f32 * dt).round(),
                r.speed.round()
            );
            break;
        }
    }
}
