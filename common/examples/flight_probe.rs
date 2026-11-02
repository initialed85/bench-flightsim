//! One-shot behaviour report for the flight model. `cargo run -p flight-common --example flight_probe`.
use flight_common::*;
use glam::Vec3;

const DT: f32 = 1.0 / 30.0;

fn world() -> Terrain {
    Terrain::generate(SEED)
}

fn airborne(speed: f32) -> PlaneState {
    PlaneState::spawn(Vec3::new(0.0, 400.0, -600.0), 0.0, speed)
}

/// Pilot that holds a target bank with proportional aileron, like a player watching the horizon.
fn hold_bank(
    state: &mut PlaneState,
    target_bank: f32,
    elevator: f32,
    throttle: f32,
    seconds: f32,
    t: &Terrain,
) -> Readout {
    let mut last = Readout::default();
    for _ in 0..(seconds / DT) as usize {
        let bank = bank_of(state.rot);
        let aileron = ((target_bank - bank) * 1.6).clamp(-1.0, 1.0);
        let c = Controls {
            aileron,
            elevator,
            throttle,
            ..Default::default()
        };
        let (r, out) = step_plane(state, &c, t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?}");
        last = r;
    }
    last
}

fn find_trim(speed: f32, _t: &Terrain) -> f32 {
    let mut lo = 0.20f32;
    let mut hi = 1.0f32;
    for _ in 0..24 {
        let mid = (lo + hi) * 0.5;
        let mut s = airborne(speed);
        let r = simulate(&mut s, mid, 12.0);
        if r.vs > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    (lo + hi) * 0.5
}

fn simulate(state: &mut PlaneState, throttle: f32, seconds: f32) -> Readout {
    let t = world();
    let c = Controls {
        throttle,
        ..Default::default()
    };
    let mut last = Readout::default();
    for _ in 0..(seconds / DT) as usize {
        let (r, out) = step_plane(state, &c, &t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?} at {}", state.pos);
        last = r;
    }
    last
}

fn main() {
    let t = world();

    // Cruise: what does full power do at 400 m?
    let mut s = airborne(50.0);
    let c = Controls {
        throttle: 1.0,
        ..Default::default()
    };
    let mut r = Readout::default();
    for _ in 0..900 {
        let (rr, out) = step_plane(&mut s, &c, &t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?}");
        r = rr;
    }
    println!(
        "cruise full power   : speed {:.1} m/s  vs {:+.2}  alt {:.0}  alpha {:.1} deg",
        r.speed,
        r.vs,
        s.pos.y,
        deg(r.alpha)
    );

    let trim = find_trim(45.7, &t);
    let mut s = airborne(45.7);
    let mut band = (1e9f32, -1e9f32);
    for i in 0..1800 {
        let c = Controls {
            throttle: trim,
            ..Default::default()
        };
        let (rr, out) = step_plane(&mut s, &c, &t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?}");
        if i > 300 {
            band.0 = band.0.min(s.pos.y);
            band.1 = band.1.max(s.pos.y);
        }
        r = rr;
    }
    println!(
        "trim throttle {:.3}    : speed {:.1} m/s  vs {:+.2}  alt band {:.0}..{:.0}  alpha {:.1} deg",
        trim,
        r.speed,
        r.vs,
        band.0,
        band.1,
        deg(r.alpha)
    );

    // Stall mush: back stick at low speed, low power.
    let mut s = airborne(34.0);
    let c = Controls {
        elevator: 1.0,
        throttle: 0.35,
        ..Default::default()
    };
    let y0 = s.pos.y;
    let mut stalled_at = None;
    for i in 0..300 {
        let (rr, out) = step_plane(&mut s, &c, &t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?}");
        if stalled_at.is_none() && rr.stalled {
            stalled_at = Some(i as f32 * DT);
        }
        r = rr;
    }
    println!(
        "stall (34 m/s, back stick): stalled at {:?} s  speed {:.1}  pitch {:.1} deg  vs {:+.2}  alt change {:+.0}",
        stalled_at,
        r.speed,
        deg(r.pitch),
        r.vs,
        s.pos.y - y0
    );

    // Coordinated 45 deg turn.
    let mut s = airborne(55.0);
    let _r = hold_bank(&mut s, 45f32.to_radians(), 0.25, 0.6, 3.0, &t);
    let h0 = heading_of(s.rot);
    let _r = hold_bank(&mut s, 45f32.to_radians(), 0.25, 0.6, 5.0, &t);
    let mut dh = heading_of(s.rot) - h0;
    if dh < 0.0 {
        dh += core::f32::consts::TAU;
    }
    let rate = dh / 5.0;
    let ideal = G * r.bank.tan() / r.speed;
    println!(
        "45 deg hold (bank-hold)  : bank {:.1} deg  turn {:.1} deg/s (ideal {:.1})  beta {:.2} deg  vs {:+.2}  g {:.2}",
        deg(r.bank),
        deg(rate),
        deg(ideal),
        deg(r.beta),
        r.vs,
        r.g_load
    );

    // Self level from a steep bank with the stick neutral.
    let mut s = airborne(55.0);
    let r = hold_bank(&mut s, 60f32.to_radians(), 0.2, 0.5, 3.0, &t);
    println!("rolled to 60 deg         : bank {:.1} deg", deg(r.bank));
    let r1 = simulate(&mut s, 0.5, 3.0);
    let r2 = simulate(&mut s, 0.5, 5.0);
    println!(
        "stick neutral, self level: bank {:+.1} deg after 3 s, {:+.1} deg after 8 s, beta {:+.2} deg",
        deg(r1.bank),
        deg(r2.bank),
        deg(r2.beta)
    );

    // Coordinated turn from level flight using only the rudder (skid turn).
    let mut s = airborne(55.0);
    let c = Controls {
        rudder: 0.6,
        throttle: 0.5,
        ..Default::default()
    };
    let h0 = heading_of(s.rot);
    let mut r = Readout::default();
    for _ in 0..150 {
        let (rr, out) = step_plane(&mut s, &c, &t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?}");
        r = rr;
    }
    let mut dh = heading_of(s.rot) - h0;
    if dh < 0.0 {
        dh += core::f32::consts::TAU;
    }
    println!(
        "rudder 0.6 skid turn     : heading {:+.1} deg in 5 s  bank {:+.1} deg  beta {:+.1} deg",
        deg(dh),
        deg(r.bank),
        deg(r.beta)
    );

    // Takeoff roll.
    let y = t.surface_height(0.0, -400.0) + AIRCRAFT.gear_height;
    let mut s = PlaneState::spawn(Vec3::new(0.0, y, -400.0), 0.0, 0.0);
    s.grounded = true;
    let c = Controls {
        throttle: 1.0,
        elevator: 0.75,
        ..Default::default()
    };
    let mut rotated = None;
    for i in 0..(30 * 60) {
        let (r, out) = step_plane(&mut s, &c, &t, DT, &AIRCRAFT);
        if out.took_off {
            rotated = Some((i as f32 * DT, r.speed));
            break;
        }
    }
    let (tr, vrot) = rotated.unwrap_or((99.0, 0.0));
    println!(
        "takeoff roll             : rotated in {tr:.1} s at {vrot:.1} m/s ({:.0} km/h)",
        vrot * 3.6
    );

    // Climb away after rotation.
    let c = Controls {
        throttle: 1.0,
        elevator: 0.3,
        ..Default::default()
    };
    let mut r = Readout::default();
    for _ in 0..300 {
        let (rr, out) = step_plane(&mut s, &c, &t, DT, &AIRCRAFT);
        assert!(out.crash.is_none(), "crash {out:?} at {}", s.pos);
        r = rr;
    }
    println!(
        "climb after rotation     : vs {:+.2} m/s  speed {:.1}  pitch {:.1} deg  alt {:.0}",
        r.vs,
        r.speed,
        deg(r.pitch),
        s.pos.y
    );
}
