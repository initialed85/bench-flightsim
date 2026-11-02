//! Fixed-wing flight model, shared verbatim by the server (authoritative) and the
//! client (local prediction), so both integrate the same ODE.
//!
//! Convention, pinned by the tests below:
//! - nose is local `-Z`, right wing is local `+X`, up is local `+Y` (Bevy's basis);
//! - world map: `+Z` is north and `-X` is east, which is what makes that body basis a
//!   right-handed *aircraft* frame rather than a mirrored one;
//! - heading is a compass bearing in radians, 0 = `+Z` (north), pi/2 = `-X` (east),
//!   and turning right increases it;
//! - positive elevator = nose up, positive aileron = roll right, positive rudder = nose right;
//! - positive bank = right wing down, positive sideslip = sliding toward the right wing.

use glam::{Quat, Vec3};

use crate::terrain::{is_runway, Terrain, SEA_LEVEL};

pub const G: f32 = 9.81;

#[derive(Clone, Copy, Debug)]
pub struct Aircraft {
    pub mass: f32,
    pub wing_area: f32,
    pub rho: f32,
    /// Lift curve slope, per radian of angle of attack.
    pub cl_alpha: f32,
    pub alpha_stall: f32,
    pub cl_max: f32,
    pub cd0: f32,
    pub k_induced: f32,
    /// Side force per radian of sideslip: the fuselage/fin weathervane. Without it the
    /// tilted wing turns the flight path *away* from the nose and the plane slides.
    pub cy_beta: f32,
    pub thrust_max: f32,
    pub boost_thrust: f32,
    pub roll_max: f32,
    pub pitch_max: f32,
    pub yaw_max: f32,
    /// Natural stability: the airplane trims itself back to `alpha_trim`.
    pub alpha_trim: f32,
    pub pitch_stability: f32,
    pub roll_stability: f32,
    pub yaw_stability: f32,
    /// Adverse yaw per unit of aileron.
    pub adverse_yaw: f32,
    /// Extra yaw tracking that keeps the nose on the flight path (the model
    /// auto-coordinates a turn instead of making the player foot the rudder).
    pub yaw_coordination: f32,
    /// Control authority ramps up with airspeed and fades in the stall.
    pub authority_speed: f32,
    pub stall_authority_loss: f32,
    /// How fast the control surfaces actually reach the commanded rate.
    pub rate_response: f32,
    pub rate_limit_roll: f32,
    pub rate_limit_pitch: f32,
    pub rate_limit_yaw: f32,
    pub stall_pitch_drop: f32,
    pub gear_height: f32,
    pub touchdown_sink: f32,
    pub touchdown_bank: f32,
    pub touchdown_pitch_min: f32,
    pub touchdown_pitch_max: f32,
    pub touchdown_speed: f32,
    pub ground_drag: f32,
    pub brake_drag: f32,
    pub taxi_roll: f32,
}

pub const AIRCRAFT: Aircraft = Aircraft {
    mass: 700.0,
    wing_area: 11.0,
    rho: 1.15,
    cl_alpha: 5.2,
    alpha_stall: 0.22,
    cl_max: 1.144,
    cd0: 0.032,
    k_induced: 0.048,
    cy_beta: 1.7,
    thrust_max: 3400.0,
    boost_thrust: 1500.0,
    roll_max: 2.1,
    pitch_max: 0.95,
    yaw_max: 0.5,
    alpha_trim: 0.10,
    pitch_stability: 0.9,
    roll_stability: 0.30,
    yaw_stability: 0.5,
    adverse_yaw: 0.08,
    yaw_coordination: 0.7,
    authority_speed: 38.0,
    stall_authority_loss: 0.70,
    rate_response: 6.0,
    rate_limit_roll: 3.5,
    rate_limit_pitch: 2.2,
    rate_limit_yaw: 1.4,
    stall_pitch_drop: 4.5,
    gear_height: 2.4,
    touchdown_sink: 4.0,
    touchdown_bank: 0.22,
    touchdown_pitch_min: -0.06,
    touchdown_pitch_max: 0.28,
    touchdown_speed: 46.0,
    ground_drag: 0.05,
    brake_drag: 2.2,
    taxi_roll: 0.5,
};

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Controls {
    /// 0..1 power lever.
    pub throttle: f32,
    /// -1..1. +1 rolls right.
    pub aileron: f32,
    /// -1..1. +1 raises the nose.
    pub elevator: f32,
    /// -1..1. +1 swings the nose right.
    pub rudder: f32,
    pub boost: bool,
    pub trigger: bool,
    pub brake: bool,
}

impl Controls {
    pub fn clamped(&self) -> Controls {
        Controls {
            throttle: self.throttle.clamp(0.0, 1.0),
            aileron: self.aileron.clamp(-1.0, 1.0),
            elevator: self.elevator.clamp(-1.0, 1.0),
            rudder: self.rudder.clamp(-1.0, 1.0),
            boost: self.boost,
            trigger: self.trigger,
            brake: self.brake,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PlaneState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    /// Body rates: roll, pitch, yaw in rad/s.
    pub body: Vec3,
    pub grounded: bool,
}

impl PlaneState {
    /// Airborne spawn: `heading` is a compass bearing in radians (0 = north, growing right).
    pub fn spawn(pos: Vec3, heading: f32, speed: f32) -> PlaneState {
        let f = forward_from_heading(heading, 0.0);
        PlaneState {
            pos,
            vel: f * speed,
            rot: quat_from_forward(f),
            body: Vec3::ZERO,
            grounded: false,
        }
    }
}

/// Rotation with local `-Z` along `f` and no built-in roll.
pub fn quat_from_forward(f: Vec3) -> Quat {
    let f = f.normalize();
    let pitch = f.y.clamp(-1.0, 1.0).asin();
    let yaw = (-f.x).atan2(-f.z);
    Quat::from_rotation_y(yaw) * Quat::from_rotation_x(pitch)
}

/// Rotation with local `+Y` along `dir`: Bevy's `Torus` hole axis is local Y, so
/// this is what makes a gate face the traffic.
pub fn quat_align_y(dir: Vec3) -> Quat {
    let d = dir.normalize();
    let pitch = d.y.clamp(-1.0, 1.0).acos();
    let yaw = d.x.atan2(d.z);
    Quat::from_rotation_y(yaw) * Quat::from_rotation_x(pitch)
}

pub fn forward_of(q: Quat) -> Vec3 {
    q * Vec3::NEG_Z
}
pub fn right_of(q: Quat) -> Vec3 {
    q * Vec3::X
}
pub fn up_of(q: Quat) -> Vec3 {
    q * Vec3::Y
}

/// Radians, 0 = north (`+Z`), pi/2 = east (`-X`); increasing as you turn right.
pub fn heading_of(q: Quat) -> f32 {
    let f = forward_of(q);
    // Compass convention: north is `+Z`, east is `-X`, and heading grows as you turn
    // right (see `docs` note at the top of this module).
    let h = (-f.x).atan2(f.z);
    if h < 0.0 {
        h + core::f32::consts::TAU
    } else {
        h
    }
}

/// Radians, +ve = nose up.
pub fn pitch_of(q: Quat) -> f32 {
    forward_of(q).y.clamp(-1.0, 1.0).asin()
}

/// Radians, +ve = right wing down.
pub fn bank_of(q: Quat) -> f32 {
    let r = right_of(q);
    let u = up_of(q);
    (-r.y).atan2(u.y)
}

pub fn forward_from_heading(heading: f32, pitch: f32) -> Vec3 {
    let cp = pitch.cos();
    Vec3::new(-heading.sin() * cp, pitch.sin(), heading.cos() * cp)
}

pub fn deg(rad: f32) -> f32 {
    rad * 180.0 / core::f32::consts::PI
}

/// Flat-plate style stall: linear up to `alpha_stall`, then lift collapses.
pub fn lift_coefficient(alpha: f32, ac: &Aircraft) -> f32 {
    let a = alpha.abs();
    if a <= ac.alpha_stall {
        ac.cl_alpha * alpha
    } else {
        // Past the stall the flow separates and lift falls away, but a flat plate still
        // makes something: floor it so a deep stall mushs rather than dropping like a rock.
        let over = a - ac.alpha_stall;
        (ac.cl_max * (1.0 - 2.0 * over)).max(ac.cl_max * 0.30) * alpha.signum()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Readout {
    pub speed: f32,
    pub alpha: f32,
    pub beta: f32,
    pub cl: f32,
    pub bank: f32,
    pub pitch: f32,
    pub heading: f32,
    pub g_load: f32,
    pub stalled: bool,
    pub clearance: f32,
    /// Vertical speed, m/s.
    pub vs: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Crash {
    Terrain,
    Water,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Outcome {
    pub landed: bool,
    pub took_off: bool,
    pub crash: Option<Crash>,
}

/// One integration step. Semi-implicit Euler with `dt` clamped for stability.
pub fn step_plane(
    state: &mut PlaneState,
    controls: &Controls,
    terrain: &Terrain,
    dt: f32,
    ac: &Aircraft,
) -> (Readout, Outcome) {
    let dt = dt.clamp(0.0, 0.05);
    let c = controls.clamped();
    let was_grounded = state.grounded;

    let q = state.rot.normalize();
    let f = forward_of(q);
    let r = right_of(q);
    let u = up_of(q);
    let speed = state.vel.length();
    let vhat = if speed > 0.5 { state.vel / speed } else { f };

    // Full-quadrant angle of attack: `atan2` keeps its meaning even when the
    // airplane is flying backwards out of a spin.
    let alpha = (-vhat.dot(u)).atan2(vhat.dot(f));
    let alpha_stick = alpha.clamp(-1.2, 1.2);
    let beta = vhat.dot(r).clamp(-1.0, 1.0).asin();
    let bank = bank_of(q);
    let pitch = pitch_of(q);

    let cl = lift_coefficient(alpha, ac);
    let q_dyn = 0.5 * ac.rho * speed * speed;
    let lift_mag = q_dyn * ac.wing_area * cl;
    // Lift is perpendicular to the airflow, in the plane containing the wing's up axis.
    let lift_dir = {
        let d = u - vhat * vhat.dot(u);
        if d.length_squared() < 1e-6 {
            u
        } else {
            d.normalize()
        }
    };
    let surface_drag_bonus = 0.012 * (c.aileron.abs() + c.elevator.abs() + c.rudder.abs());
    let cd = ac.cd0 + ac.k_induced * cl * cl + surface_drag_bonus;
    let drag_mag = q_dyn * ac.wing_area * cd;
    let thrust = c.throttle * ac.thrust_max + if c.boost { ac.boost_thrust } else { 0.0 };

    let side_dir = {
        let d = r - vhat * vhat.dot(r);
        if d.length_squared() < 1e-6 {
            None
        } else {
            Some(d.normalize())
        }
    };
    let mut force = lift_dir * lift_mag - vhat * drag_mag + f * thrust;
    // Weathervane side force: pushes the flight path back behind the nose.
    if let Some(dir) = side_dir {
        force -= dir * (q_dyn * ac.wing_area * ac.cy_beta * beta);
    }
    force.y -= ac.mass * G;
    state.vel += force * (dt / ac.mass);

    // Body-rate targets: pilot command + natural stability + stall behaviour.
    let eff = (speed / ac.authority_speed).clamp(0.12, 1.0);
    // The wing is stalled: the surfaces sit in separated flow and lose bite.
    let stall_depth = ((alpha_stick.abs() - ac.alpha_stall) / 0.40).clamp(0.0, 1.0);
    let eff = eff * (1.0 - ac.stall_authority_loss * stall_depth);

    // Dihedral effect, taken from the vectors rather than an Euler angle: it is
    // `-sin(bank)` in normal flight (bank +ve = right wing down), and it fades out by
    // itself when the nose points at the sky, which is where wings-level means nothing.
    let roll_target =
        c.aileron * ac.roll_max * eff + ac.roll_stability * u.cross(Vec3::Y).dot(f) * 2.0;
    let mut pitch_target =
        c.elevator * ac.pitch_max * eff - ac.pitch_stability * (alpha_stick - ac.alpha_trim);
    if alpha_stick > ac.alpha_stall {
        pitch_target -= ac.stall_pitch_drop * (alpha_stick - ac.alpha_stall);
    } else if alpha_stick < -ac.alpha_stall {
        pitch_target += ac.stall_pitch_drop * (-alpha_stick - ac.alpha_stall);
    }
    let mut yaw_target = c.rudder * ac.yaw_max * eff
        + (ac.yaw_stability + ac.yaw_coordination) * beta
        - ac.adverse_yaw * c.aileron * eff;

    if state.grounded {
        // Nose-wheel steering: aileron acts as a taxi tiller at low speed.
        let taxi = (speed / 18.0).clamp(0.0, 1.0);
        yaw_target += c.aileron * ac.taxi_roll * taxi;
    }

    // Rates are rate-limited physically, then smoothed by the surface response.
    let roll_target = roll_target.clamp(-ac.rate_limit_roll, ac.rate_limit_roll);
    let pitch_target = pitch_target.clamp(-ac.rate_limit_pitch, ac.rate_limit_pitch);
    let yaw_target = yaw_target.clamp(-ac.rate_limit_yaw, ac.rate_limit_yaw);

    let k = (1.0 - (-ac.rate_response * dt).exp()).clamp(0.0, 1.0);
    state.body.x += (roll_target - state.body.x) * k;
    state.body.y += (pitch_target - state.body.y) * k;
    state.body.z += (yaw_target - state.body.z) * k;
    state.body = Vec3::new(
        state.body.x.clamp(-ac.rate_limit_roll, ac.rate_limit_roll),
        state
            .body
            .y
            .clamp(-ac.rate_limit_pitch, ac.rate_limit_pitch),
        state.body.z.clamp(-ac.rate_limit_yaw, ac.rate_limit_yaw),
    );

    state.rot = (state.rot
        * Quat::from_rotation_z(-state.body.x * dt)
        * Quat::from_rotation_x(state.body.y * dt)
        * Quat::from_rotation_y(-state.body.z * dt))
    .normalize();

    state.pos += state.vel * dt;

    if state.grounded {
        let k = ac.ground_drag + if c.brake { ac.brake_drag } else { 0.0 };
        let hv = Vec3::new(state.vel.x, 0.0, state.vel.z);
        state.vel -= hv * (k * dt);
    }

    resolve_ground(
        state,
        terrain,
        ac,
        was_grounded,
        lift_mag,
        speed,
        bank,
        pitch,
    )
    .with_readout(Readout {
        speed,
        alpha,
        beta,
        cl,
        bank,
        pitch,
        heading: heading_of(q),
        g_load: lift_mag / (ac.mass * G),
        stalled: alpha.abs() > ac.alpha_stall,
        clearance: state.pos.y - ac.gear_height - terrain.surface_height(state.pos.x, state.pos.z),
        vs: state.vel.y,
    })
}

impl Outcome {
    fn with_readout(self, r: Readout) -> (Readout, Outcome) {
        (r, self)
    }
}

fn resolve_ground(
    state: &mut PlaneState,
    terrain: &Terrain,
    ac: &Aircraft,
    was_grounded: bool,
    lift_mag: f32,
    speed: f32,
    bank: f32,
    pitch: f32,
) -> Outcome {
    let mut out = Outcome::default();
    let ground = terrain.surface_height(state.pos.x, state.pos.z);
    let gear = state.pos.y - ac.gear_height;

    if state.grounded {
        let lifted = lift_mag >= ac.mass * G * 1.02;
        if lifted || gear > ground + 0.5 {
            state.grounded = false;
            out.took_off = true;
            return out;
        }
        state.pos.y = ground + ac.gear_height;
        // On the surface the flight path is flat: keeping a vertical velocity here would
        // tilt it and steal the angle of attack the wings need to rotate.
        state.vel.y = 0.0;
        // On the surface the wings are level and the pitch is within the gear geometry.
        // Rotation is limited to the stall angle: pulling further just mushes.
        let h = heading_of(state.rot);
        let p = pitch_of(state.rot).clamp(-0.04, ac.alpha_stall);
        state.rot = quat_from_forward(forward_from_heading(h, p));
        state.body = Vec3::new(0.0, state.body.y, state.body.z);
        return out;
    }

    if gear > ground + 0.02 {
        return out;
    }

    let dry = ground > SEA_LEVEL + 0.4;
    if !dry {
        out.crash = Some(Crash::Water);
        return out;
    }

    let gentle = state.vel.y > -ac.touchdown_sink
        && bank.abs() < ac.touchdown_bank
        && pitch > ac.touchdown_pitch_min
        && pitch < ac.touchdown_pitch_max
        && speed < ac.touchdown_speed;
    let on_field = is_runway(state.pos.x, state.pos.z);

    if gentle && (on_field || (ground > 8.0 && slope_ok(terrain, state.pos.x, state.pos.z))) {
        state.grounded = true;
        state.pos.y = ground + ac.gear_height;
        state.vel.y = state.vel.y.max(0.0);
        let h = heading_of(state.rot);
        let p = pitch_of(state.rot).clamp(-0.04, ac.alpha_stall);
        state.rot = quat_from_forward(forward_from_heading(h, p));
        state.body = Vec3::ZERO;
        out.landed = !was_grounded;
    } else {
        out.crash = Some(Crash::Terrain);
    }
    out
}

/// Off-field landings need a gentle slope, otherwise it is a crash.
fn slope_ok(terrain: &Terrain, x: f32, z: f32) -> bool {
    terrain.normal(x, z).y > 0.90
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::SEED;

    fn world() -> Terrain {
        Terrain::generate(SEED)
    }

    fn airborne(speed: f32) -> PlaneState {
        PlaneState::spawn(Vec3::new(0.0, 400.0, -600.0), 0.0, speed)
    }

    fn simulate(
        state: &mut PlaneState,
        c: &Controls,
        terrain: &Terrain,
        seconds: f32,
        dt: f32,
    ) -> Readout {
        let mut last = Readout::default();
        let steps = (seconds / dt) as usize;
        for _ in 0..steps {
            let (r, out) = step_plane(state, c, terrain, dt, &AIRCRAFT);
            assert!(
                out.crash.is_none(),
                "unexpected crash {:?} at {:?}",
                out.crash,
                state.pos
            );
            last = r;
        }
        last
    }

    #[test]
    fn attitude_helpers_round_trip() {
        for dir in [
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(-0.4, 0.3, 0.8),
            Vec3::new(0.6, -0.5, -0.6),
        ] {
            let dir = dir.normalize();
            let q = quat_from_forward(dir);
            let f = forward_of(q);
            assert!(
                (f - dir.normalize()).length() < 1e-5,
                "round trip failed for {dir}: got {f}"
            );
            assert!(
                (pitch_of(q) - dir.y.asin()).abs() < 1e-3,
                "pitch {:?} vs dir {:?} gave {:?} (yaw/pitch are ill-conditioned near vertical)",
                pitch_of(q),
                dir,
                dir.y.asin()
            );
        }
    }

    #[test]
    fn heading_zero_is_north_and_clockwise() {
        let q = quat_from_forward(Vec3::new(0.0, 0.0, 1.0));
        assert!(heading_of(q).abs() < 1e-5);
        // East is `-X`: turning right from north goes toward `-X`.
        let q = quat_from_forward(Vec3::new(-1.0, 0.0, 0.0));
        assert!((deg(heading_of(q)) - 90.0).abs() < 1e-4);
        let q = quat_from_forward(Vec3::new(0.0, 0.0, -1.0));
        assert!((deg(heading_of(q)) - 180.0).abs() < 1e-4);
        // Spawning on a heading and reading it back must agree.
        let s = PlaneState::spawn(Vec3::ZERO, 1.0, 40.0);
        assert!((deg(heading_of(s.rot)) - deg(1.0)).abs() < 1e-3);
        assert!(s.vel.x < 0.0, "a right-turned heading flies toward -X");
    }

    #[test]
    fn gate_ring_faces_the_traffic() {
        for dir in [
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.7, 0.3, -0.6),
            Vec3::new(0.0, -1.0, 0.0),
        ] {
            let q = quat_align_y(dir);
            let axis = up_of(q);
            assert!(
                (axis - dir.normalize()).length() < 1e-5,
                "gate axis {axis} vs {dir}"
            );
        }
    }

    #[test]
    fn positive_elevator_raises_the_nose() {
        let t = world();
        let mut s = airborne(55.0);
        let before = pitch_of(s.rot);
        let c = Controls {
            elevator: 1.0,
            throttle: 0.5,
            ..Default::default()
        };
        simulate(&mut s, &c, &t, 1.5, 1.0 / 60.0);
        assert!(pitch_of(s.rot) > before + 0.1, "nose did not come up");
    }

    #[test]
    fn positive_aileron_banks_right() {
        let t = world();
        let mut s = airborne(55.0);
        let c = Controls {
            aileron: 1.0,
            throttle: 0.5,
            ..Default::default()
        };
        simulate(&mut s, &c, &t, 2.0, 1.0 / 60.0);
        assert!(deg(bank_of(s.rot)) > 30.0, "bank {}", deg(bank_of(s.rot)));
        assert!(right_of(s.rot).y < 0.0, "right wing should be down");
    }

    #[test]
    fn positive_rudder_swings_the_nose_right() {
        let t = world();
        let mut s = airborne(55.0);
        let c = Controls {
            rudder: 1.0,
            throttle: 0.4,
            ..Default::default()
        };
        simulate(&mut s, &c, &t, 3.0, 1.0 / 60.0);
        assert!(
            deg(heading_of(s.rot)) > 5.0,
            "heading {}",
            deg(heading_of(s.rot))
        );
    }

    /// A pilot holding a bank angle with proportional aileron, the way a player flies.
    fn hold_bank(
        state: &mut PlaneState,
        target_bank: f32,
        elevator: f32,
        throttle: f32,
        seconds: f32,
        t: &Terrain,
    ) -> Readout {
        let mut last = Readout::default();
        for _ in 0..(seconds * 30.0) as usize {
            let bank = bank_of(state.rot);
            let aileron = ((target_bank - bank) * 1.6).clamp(-1.0, 1.0);
            let c = Controls {
                aileron,
                elevator,
                throttle,
                ..Default::default()
            };
            let (r, out) = step_plane(state, &c, t, 1.0 / 30.0, &AIRCRAFT);
            assert!(out.crash.is_none(), "crash {out:?} at {}", state.pos);
            last = r;
        }
        last
    }

    /// Throttle that holds height, by bisection: the trim depends on every other number.
    fn trim_throttle(speed: f32, t: &Terrain) -> f32 {
        let mut lo = 0.05f32;
        let mut hi = 1.0f32;
        for _ in 0..20 {
            let mid = (lo + hi) * 0.5;
            let mut s = airborne(speed);
            let c = Controls {
                throttle: mid,
                ..Default::default()
            };
            let r = simulate(&mut s, &c, t, 12.0, 1.0 / 30.0);
            if r.vs > 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        (lo + hi) * 0.5
    }

    #[test]
    fn trimmed_flight_holds_altitude() {
        let t = world();
        let throttle = trim_throttle(45.0, &t);
        let mut s = airborne(45.0);
        let start = s.pos.y;
        let c = Controls {
            throttle,
            ..Default::default()
        };
        let r = simulate(&mut s, &c, &t, 30.0, 1.0 / 30.0);
        assert!(
            (s.pos.y - start).abs() < 80.0,
            "altitude drifted {} m at throttle {throttle}",
            s.pos.y - start
        );
        assert!(!r.stalled, "trimmed cruise should not stall");
        assert!(r.g_load > 0.7 && r.g_load < 1.4, "g load {}", r.g_load);
        assert!(r.speed > 35.0 && r.speed < 80.0, "cruise speed {}", r.speed);
    }

    #[test]
    fn too_little_power_comes_down() {
        let t = world();
        let mut s = airborne(45.0);
        let c = Controls {
            throttle: 0.05,
            ..Default::default()
        };
        let start = s.pos.y;
        simulate(&mut s, &c, &t, 12.0, 1.0 / 30.0);
        assert!(
            s.pos.y < start - 20.0,
            "idle power should descend: {} -> {}",
            start,
            s.pos.y
        );
    }

    #[test]
    fn pulling_too_hard_stalls_the_airplane() {
        let t = world();
        // Below the stall speed with back stick: the wing cannot hold the airplane up.
        let mut s = airborne(28.0);
        let start = s.pos.y;
        let c = Controls {
            elevator: 0.6,
            throttle: 0.25,
            ..Default::default()
        };
        let mut stalled = false;
        for _ in 0..150 {
            let (r, out) = step_plane(&mut s, &c, &t, 1.0 / 30.0, &AIRCRAFT);
            assert!(out.crash.is_none());
            if r.stalled {
                stalled = true;
                break;
            }
        }
        assert!(stalled, "no stall at 34 m/s with full back stick");
        // The nose drops and the airplane comes down.
        for _ in 0..90 {
            let (_, out) = step_plane(&mut s, &c, &t, 1.0 / 30.0, &AIRCRAFT);
            assert!(out.crash.is_none(), "stall test crashed: {:?}", out.crash);
        }
        assert!(
            s.pos.y < start - 10.0,
            "stall did not dump altitude: {}",
            s.pos.y
        );
        assert!(s.vel.y < 0.0);
    }

    #[test]
    fn a_banked_airplane_turns_like_an_airplane() {
        let t = world();
        let mut s = airborne(55.0);
        // Hold about 45 degrees of bank with a proportional aileron, like a player watching the horizon.
        let bank = 45f32.to_radians();
        hold_bank(&mut s, bank, 0.15, 0.55, 3.0, &t);
        let measured = deg(bank_of(s.rot));
        assert!(measured > 35.0 && measured < 55.0, "bank {measured}");

        let h0 = heading_of(s.rot);
        let r = hold_bank(&mut s, bank, 0.15, 0.55, 5.0, &t);
        let mut dh = heading_of(s.rot) - h0;
        if dh < 0.0 {
            dh += core::f32::consts::TAU;
        }
        let rate = dh / 5.0;
        // Ideal coordinated turn: omega = g * tan(bank) / v.
        let ideal = G * bank.tan() / r.speed;
        assert!(
            rate > ideal * 0.6 && rate < ideal * 1.5,
            "turn rate {rate} rad/s vs ideal {ideal}"
        );
        // A coordinated turn should not be side-slipping much.
        assert!(deg(r.beta).abs() < 8.0, "slipping: {} deg", deg(r.beta));
    }

    #[test]
    fn rolling_out_on_level_wings_stops_the_turn() {
        let t = world();
        let mut s = airborne(55.0);
        hold_bank(&mut s, 45f32.to_radians(), 0.15, 0.5, 4.0, &t);
        assert!(bank_of(s.rot).abs() > 0.4);
        // Neutral stick: dihedral and roll damping level the wings on their own.
        simulate(
            &mut s,
            &Controls {
                throttle: 0.5,
                ..Default::default()
            },
            &t,
            6.0,
            1.0 / 30.0,
        );
        assert!(
            deg(bank_of(s.rot)).abs() < 15.0,
            "did not self-level: {}",
            deg(bank_of(s.rot))
        );
    }

    #[test]
    fn full_throttle_takes_off_from_the_runway() {
        let t = world();
        let mut s = PlaneState::spawn(
            Vec3::new(
                0.0,
                t.surface_height(0.0, -400.0) + AIRCRAFT.gear_height,
                -400.0,
            ),
            0.0,
            0.0,
        );
        s.grounded = true;
        s.vel = Vec3::new(0.0, 0.0, 1.0);
        let c = Controls {
            throttle: 1.0,
            elevator: 0.75,
            ..Default::default()
        };
        let mut airborne_at = None;
        for i in 0..(30 * 40) {
            let (_, out) = step_plane(&mut s, &c, &t, 1.0 / 30.0, &AIRCRAFT);
            if out.took_off {
                airborne_at = Some(i as f32 / 30.0);
                break;
            }
        }
        let t_rot = airborne_at.expect("never rotated off the ground");
        assert!(t_rot < 20.0, "rotation took {t_rot} s");
        assert!(s.pos.y > RUNWAY_Y_TEST);
    }

    const RUNWAY_Y_TEST: f32 = crate::terrain::RUNWAY_Y;

    #[test]
    fn gentle_touchdown_on_the_runway_is_a_landing() {
        let t = world();
        let mut s = PlaneState::spawn(
            Vec3::new(0.0, RUNWAY_Y_TEST + AIRCRAFT.gear_height + 12.0, -300.0),
            0.0,
            36.0,
        );
        s.vel = Vec3::new(0.0, -1.0, 36.0);
        let c = Controls {
            throttle: 0.35,
            elevator: 0.05,
            ..Default::default()
        };
        let mut landed = false;
        for _ in 0..(30 * 30) {
            let (_, out) = step_plane(&mut s, &c, &t, 1.0 / 30.0, &AIRCRAFT);
            if out.crash.is_some() {
                panic!("expected a landing, got crash {:?}", out.crash);
            }
            if out.landed {
                landed = true;
                break;
            }
        }
        assert!(landed, "never touched down");
        assert!(s.grounded);
        assert!((s.pos.y - AIRCRAFT.gear_height - RUNWAY_Y_TEST).abs() < 1.0);
    }

    #[test]
    fn flying_into_a_hill_crashes() {
        let t = world();
        // Find a point with high terrain and aim straight at it, low and fast.
        let (tx, tz) = (1400.0f32, 900.0f32);
        let hill = t.surface_height(tx, tz);
        assert!(hill > 60.0, "no hill at that spot: {hill}");
        let aim = Vec3::new(tx, hill, tz) - Vec3::new(tx - 500.0, hill + 8.0, tz - 500.0);
        let mut s = PlaneState::spawn(
            Vec3::new(tx - 500.0, hill + 8.0, tz - 500.0),
            aim.x.atan2(aim.z),
            80.0,
        );
        s.vel = aim.normalize() * 80.0;
        let c = Controls {
            throttle: 0.8,
            ..Default::default()
        };
        let mut crashed = None;
        for _ in 0..(30 * 20) {
            let (_, out) = step_plane(&mut s, &c, &t, 1.0 / 30.0, &AIRCRAFT);
            if out.crash.is_some() {
                crashed = out.crash;
                break;
            }
        }
        assert!(
            crashed.is_some(),
            "flew through a mountain without crashing"
        );
    }

    #[test]
    fn ditching_in_the_sea_crashes() {
        let t = world();
        let sea = t.surface_height(2350.0, 0.0);
        assert!(
            sea < crate::terrain::SEA_LEVEL,
            "no sea at that spot: {sea}"
        );
        let mut s = PlaneState::spawn(Vec3::new(2350.0, sea + 15.0, 0.0), 0.0, 45.0);
        s.vel = Vec3::new(0.0, -5.0, 45.0);
        let c = Controls {
            throttle: 0.15,
            ..Default::default()
        };
        let mut crashed = None;
        for _ in 0..(30 * 30) {
            let (_, out) = step_plane(&mut s, &c, &t, 1.0 / 30.0, &AIRCRAFT);
            if let Some(crash) = out.crash {
                crashed = Some(crash);
                break;
            }
        }
        assert_eq!(crashed, Some(Crash::Water));
    }

    #[test]
    fn controls_are_clamped() {
        let c = Controls {
            throttle: 5.0,
            aileron: -9.0,
            elevator: 9.0,
            rudder: 3.0,
            boost: true,
            trigger: true,
            brake: true,
        }
        .clamped();
        assert_eq!(c.throttle, 1.0);
        assert_eq!(c.aileron, -1.0);
        assert_eq!(c.elevator, 1.0);
        assert_eq!(c.rudder, 1.0);
    }

    #[test]
    fn lift_and_stall_speed_are_physical() {
        let weight = AIRCRAFT.mass * G;
        // At the stall speed, max lift coefficient just supports the airplane.
        let v_stall = (2.0 * weight / (AIRCRAFT.rho * AIRCRAFT.wing_area * AIRCRAFT.cl_max)).sqrt();
        let cl = lift_coefficient(AIRCRAFT.alpha_stall, &AIRCRAFT);
        let lift = 0.5 * AIRCRAFT.rho * v_stall * v_stall * AIRCRAFT.wing_area * cl;
        assert!(
            (lift - weight).abs() / weight < 0.03,
            "lift {lift} weight {weight}"
        );
        // Cruise lift at the trimmed angle of attack supports it too, at a sane speed.
        let cl_c = lift_coefficient(AIRCRAFT.alpha_trim, &AIRCRAFT);
        let v_cruise = (2.0 * weight / (AIRCRAFT.rho * AIRCRAFT.wing_area * cl_c)).sqrt();
        assert!(
            v_cruise > v_stall && v_cruise < 60.0,
            "cruise {v_cruise} stall {v_stall}"
        );
    }
}
