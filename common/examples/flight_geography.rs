//! Geography checks: does "right" in the model actually mean right on the map?
//! Convention under test: +Z is north, +X is east, +Y is up, heading 0 = north.
use flight_common::*;
use glam::Vec3;

fn fly(label: &str, c: &Controls) {
    let t = Terrain::generate(SEED);
    let dt = 1.0 / 30.0;
    let mut s = PlaneState::spawn(Vec3::new(0.0, 600.0, -600.0), 0.0, 50.0);
    let mut turn_x = 0.0;
    for i in 0..90 {
        let (r, _) = step_plane(&mut s, c, &t, dt, &AIRCRAFT);
        if i % 15 == 0 {
            turn_x = s.vel.x;
            println!(
                "  {label} t={:4.1} vel=({:6.1},{:6.1},{:6.1}) heading {:7.1} bank {:7.1} yaw {:7.2} roll {:7.2}",
                i as f32 * dt,
                s.vel.x,
                s.vel.y,
                s.vel.z,
                deg(r.heading),
                deg(r.bank),
                deg(s.body.z),
                deg(s.body.x)
            );
        }
    }
    println!("  {label} final lateral velocity x = {turn_x:+.1} (right turn => +, left turn => -)");
}

fn main() {
    fly(
        "rudder +",
        &Controls {
            throttle: 0.5,
            rudder: 0.5,
            ..Default::default()
        },
    );
    fly(
        "rudder -",
        &Controls {
            throttle: 0.5,
            rudder: -0.5,
            ..Default::default()
        },
    );
    fly(
        "aileron +",
        &Controls {
            throttle: 0.5,
            aileron: 0.5,
            ..Default::default()
        },
    );
    fly(
        "aileron -",
        &Controls {
            throttle: 0.5,
            aileron: -0.5,
            ..Default::default()
        },
    );
    fly(
        "elevator +",
        &Controls {
            throttle: 0.8,
            elevator: 0.5,
            ..Default::default()
        },
    );
    println!("spawn heading 0 means north (+Z): initial velocity = (0,0,50)");
}
