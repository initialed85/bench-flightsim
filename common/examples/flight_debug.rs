//! Fine-grained diagnostics: which mode diverges and when.
use flight_common::*;
use glam::{Quat, Vec3};

fn main() {
    let t = Terrain::generate(SEED);
    let dt = 1.0 / 30.0;
    let neutral = Controls { throttle: 0.5, ..Default::default() };

    println!("== bank decay from 60 deg, neutral stick (every 0.1 s) ==");
    let mut s = PlaneState::spawn(Vec3::new(0.0, 500.0, 0.0), 0.0, 55.0);
    s.rot = s.rot * Quat::from_rotation_z(-60f32.to_radians());
    for i in 25..80 {
        let (r, o) = step_plane(&mut s, &neutral, &t, dt, &AIRCRAFT);
        println!(
            "  t={:5.2} bank {:7.1} head {:7.1} pitch {:7.1} alpha {:8.1} beta {:7.1} spd {:5.1} vs {:6.1} body {:7.2}/{:7.2}/{:7.2} alt {:7.1} crash {:?}",
            i as f32 * dt,
            deg(r.bank),
            deg(r.heading),
            deg(r.pitch),
            deg(r.alpha),
            deg(r.beta),
            r.speed,
            r.vs,
            deg(s.body.x),
            deg(s.body.y),
            deg(s.body.z),
            s.pos.y,
            o.crash
        );
    }

    println!("== hands off trim, every 0.2 s from t=14 ==");
    let mut s = PlaneState::spawn(Vec3::new(0.0, 400.0, -600.0), 0.0, 45.7);
    let c = Controls { throttle: 0.31, ..Default::default() };
    for i in 420..700 {
        let (r, o) = step_plane(&mut s, &c, &t, dt, &AIRCRAFT);
        if i % 6 == 0 {
            println!(
                "  t={:5.2} bank {:7.1} head {:7.1} pitch {:7.1} alpha {:8.1} beta {:7.1} spd {:5.1} vs {:6.1} body {:7.2}/{:7.2}/{:7.2} alt {:7.1} g {:5.2} crash {:?}",
                i as f32 * dt,
                deg(r.bank),
                deg(r.heading),
                deg(r.pitch),
                deg(r.alpha),
                deg(r.beta),
                r.speed,
                r.vs,
                deg(s.body.x),
                deg(s.body.y),
                deg(s.body.z),
                s.pos.y,
                r.g_load,
                o.crash
            );
        }
    }
}
