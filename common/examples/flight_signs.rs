//! Empirical sign checks for the weathervane force and the yaw restoring torque.
use flight_common::*;
use glam::Vec3;

fn main() {
    let t = Terrain::generate(SEED);
    let dt = 1.0 / 30.0;
    let off = 20f32.to_radians();

    // Nose north, velocity 20 deg to the LEFT of the nose, wings level.
    let mut s = PlaneState::spawn(Vec3::new(0.0, 600.0, -600.0), 0.0, 50.0);
    s.vel = Vec3::new(-off.sin(), 0.0, off.cos()) * 50.0;
    let c = Controls {
        throttle: 0.4,
        ..Default::default()
    };
    println!("sideslip damping check (beta should shrink toward zero):");
    for i in 0..40 {
        let (r, _) = step_plane(&mut s, &c, &t, dt, &AIRCRAFT);
        if i % 4 == 0 {
            let vh = s.vel.x.atan2(s.vel.z);
            println!(
                "  t={:4.1} nose {:7.1} velocity {:7.1} beta {:7.1} yaw_rate {:7.2}",
                i as f32 * dt,
                deg(r.heading),
                deg(vh),
                deg(r.beta),
                deg(s.body.z)
            );
        }
    }

    // Same, but with the yaw restoring removed: only the weathervane force acts.
    let mut ac = AIRCRAFT;
    ac.yaw_stability = 0.0;
    ac.yaw_coordination = 0.0;
    let mut s = PlaneState::spawn(Vec3::new(0.0, 600.0, -600.0), 0.0, 50.0);
    s.vel = Vec3::new(-off.sin(), 0.0, off.cos()) * 50.0;
    println!("weathervane force only (velocity heading should move toward the nose):");
    for i in 0..40 {
        let (r, _) = step_plane(&mut s, &c, &t, dt, &ac);
        if i % 4 == 0 {
            let vh = s.vel.x.atan2(s.vel.z);
            println!(
                "  t={:4.1} nose {:7.1} velocity {:7.1} beta {:7.1}",
                i as f32 * dt,
                deg(r.heading),
                deg(vh),
                deg(r.beta)
            );
        }
    }

    // Force only, no weathervane at all: is the tilted wing stabilising or not?
    let mut ac2 = AIRCRAFT;
    ac2.yaw_stability = 0.0;
    ac2.yaw_coordination = 0.0;
    ac2.cy_beta = 0.0;
    let mut s = PlaneState::spawn(Vec3::new(0.0, 600.0, -600.0), 0.0, 50.0);
    s.vel = Vec3::new(-off.sin(), 0.0, off.cos()) * 50.0;
    println!("no weathervane at all (tells you what the tilted wing alone does):");
    for i in 0..40 {
        let (r, _) = step_plane(&mut s, &c, &t, dt, &ac2);
        if i % 4 == 0 {
            let vh = s.vel.x.atan2(s.vel.z);
            println!(
                "  t={:4.1} nose {:7.1} velocity {:7.1} beta {:7.1}",
                i as f32 * dt,
                deg(r.heading),
                deg(vh),
                deg(r.beta)
            );
        }
    }
}
