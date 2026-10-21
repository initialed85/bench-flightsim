//! What do the touchdown conditions actually look like? `cargo run -p flight-common --example flight_landing`.
use flight_common::*;
use glam::Vec3;

fn main() {
    let t = Terrain::generate(SEED);
    let dt = 1.0 / 30.0;
    for (i, (speed, sink, throttle, elevator)) in [
        (36.0f32, 1.0f32, 0.30f32, 0.0f32),
        (36.0, 1.0, 0.35, 0.05),
        (34.0, 0.8, 0.40, 0.10),
        (32.0, 0.6, 0.50, 0.15),
        (40.0, 1.5, 0.25, -0.05),
    ]
    .iter()
    .enumerate()
    {
        let y = RUNWAY_Y + AIRCRAFT.gear_height + 12.0;
        let mut s = PlaneState::spawn(Vec3::new(0.0, y, -300.0), 0.0, *speed);
        s.vel = Vec3::new(0.0, -*sink, *speed);
        let c = Controls { throttle: *throttle, elevator: *elevator, ..Default::default() };
        let mut result = "no contact";
        for step in 0..(30 * 30) {
            let (r, out) = step_plane(&mut s, &c, &t, dt, &AIRCRAFT);
            if out.landed || out.crash.is_some() {
                result = if out.landed { "LANDED" } else { "CRASH" };
                println!(
                    "case {i} spd_in {speed} sink {sink} thr {throttle} elev {elevator}: {result} at t={:.1}  spd {:.1} sink {:.2} pitch {:.1} bank {:.1} clearance {:.2} alpha {:.1}",
                    step as f32 * dt,
                    r.speed,
                    r.vs,
                    deg(r.pitch),
                    deg(r.bank),
                    r.clearance,
                    deg(r.alpha)
                );
                break;
            }
            if step % 90 == 0 {
                println!(
                    "  case {i} t={:4.1} alt {:6.1} spd {:5.1} vs {:+5.2} pitch {:5.1} alpha {:5.1} bank {:6.1}",
                    step as f32 * dt,
                    s.pos.y,
                    r.speed,
                    r.vs,
                    deg(r.pitch),
                    deg(r.alpha),
                    deg(r.bank)
                );
            }
        }
        println!("case {i}: {result}");
    }
}
