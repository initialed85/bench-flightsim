//! The race course: a closed circuit of rings the airplane flies through in order.
//! Gate positions come from the seed, so every client builds the identical circuit.

use glam::Vec3;

use crate::terrain::Terrain;

pub const N_GATES: usize = 10;
pub const GATE_RADIUS: f32 = 22.0;
/// Minimum clearance a gate is raised to above the terrain it sits over.
pub const GATE_MIN_AGL: f32 = 55.0;

#[derive(Clone, Copy, Debug)]
pub struct Gate {
    pub pos: Vec3,
    /// Direction of travel through the gate: the gate's hole axis.
    pub normal: Vec3,
}

pub fn gates(terrain: &Terrain) -> Vec<Gate> {
    let n = N_GATES as f32;
    let tau = core::f32::consts::TAU;

    let mut pos: Vec<Vec3> = Vec::with_capacity(N_GATES);
    for i in 0..N_GATES {
        // Start gate due north of the airfield so a north-bound spawn is on course.
        let a = i as f32 * tau / n + core::f32::consts::FRAC_PI_2;
        let r = 820.0 + 330.0 * (3.0 * a + 1.2).sin() + 120.0 * (5.0 * a).sin();
        let x = r * a.cos();
        let z = r * a.sin();
        let path_alt = 190.0 + 80.0 * (2.0 * a + 0.7).sin() + 45.0 * (3.0 * a).cos();
        let ground = terrain.surface_height(x, z);
        let y = path_alt.max(ground + GATE_MIN_AGL);
        pos.push(Vec3::new(x, y, z));
    }

    let mut out = Vec::with_capacity(N_GATES);
    for i in 0..N_GATES {
        let prev = pos[(i + N_GATES - 1) % N_GATES];
        let normal = (pos[i] - prev).normalize();
        out.push(Gate { pos: pos[i], normal });
    }
    out
}

/// Did the segment `prev -> now` pass through the gate opening, in the right direction?
pub fn crossed_gate(prev: Vec3, now: Vec3, gate: &Gate, radius: f32) -> bool {
    let n = gate.normal;
    let d0 = (prev - gate.pos).dot(n);
    let d1 = (now - gate.pos).dot(n);
    // Must cross the gate plane from behind it to in front of it.
    if d0 >= 0.0 || d1 < 0.0 {
        return false;
    }
    let denom = d1 - d0;
    if denom <= 1e-6 {
        return false;
    }
    let t = -d0 / denom;
    if !(0.0..=1.0).contains(&t) {
        return false;
    }
    let hit = prev + (now - prev) * t;
    // At the crossing point the offset from the centre is already perpendicular to
    // the gate normal, so its length is the miss distance.
    (hit - gate.pos).length() <= radius
}

/// Points awarded for a gate: a bonus for arriving fast.
pub fn gate_points(speed: f32, lap: u32) -> i32 {
    let speed_bonus = (speed.max(0.0) / 12.0) as i32; // ~5 pts at 60 m/s
    100 + speed_bonus + (lap as i32).min(5) * 20
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::SEED;

    fn circuit() -> (Terrain, Vec<Gate>) {
        let t = Terrain::generate(SEED);
        let g = gates(&t);
        (t, g)
    }

    #[test]
    fn circuit_is_deterministic_and_sane() {
        let (t, a) = circuit();
        let b = gates(&t);
        assert_eq!(a.len(), N_GATES);
        for (x, y) in a.iter().zip(b.iter()) {
            assert_eq!(x.pos.to_array(), y.pos.to_array());
        }
        for (i, g) in a.iter().enumerate() {
            let ground = t.surface_height(g.pos.x, g.pos.z);
            assert!(g.pos.y >= ground + GATE_MIN_AGL - 1.0, "gate {i} is in the ground");
            assert!(g.pos.y < 500.0, "gate {i} is too high: {}", g.pos.y);
            assert!(g.pos.length() < 1500.0, "gate {i} is off the island: {:?}", g.pos);
            assert!((g.normal.length() - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn gates_are_spread_out() {
        let (_, g) = circuit();
        for i in 0..N_GATES {
            let next = g[(i + 1) % N_GATES];
            let d = (next.pos - g[i].pos).length();
            assert!(d > 250.0, "gates {i} and {} are {d} m apart", (i + 1) % N_GATES);
            assert!(d < 1400.0, "leg {i} is {d} m, too far");
        }
    }

    #[test]
    fn flying_straight_through_counts_once() {
        let (_, g) = circuit();
        let gate = g[0];
        let prev = gate.pos - gate.normal * 30.0;
        let now = gate.pos + gate.normal * 5.0;
        assert!(crossed_gate(prev, now, &gate, GATE_RADIUS));
        // Approaching but not yet through: nothing.
        assert!(!crossed_gate(prev, gate.pos - gate.normal * 2.0, &gate, GATE_RADIUS));
        // Going backwards through it does not count.
        assert!(!crossed_gate(now, prev, &gate, GATE_RADIUS));
    }

    #[test]
    fn missing_the_opening_does_not_count() {
        let (_, g) = circuit();
        let gate = g[3];
        let side = if gate.normal.abs().x > 0.5 { Vec3::Y } else { Vec3::X };
        let offset = side * (GATE_RADIUS + 8.0);
        let prev = gate.pos - gate.normal * 20.0 + offset;
        let now = gate.pos + gate.normal * 20.0 + offset;
        assert!(!crossed_gate(prev, now, &gate, GATE_RADIUS));
        // Same line, but through the hole.
        assert!(crossed_gate(
            gate.pos - gate.normal * 20.0,
            gate.pos + gate.normal * 20.0,
            &gate,
            GATE_RADIUS
        ));
    }

    #[test]
    fn a_fast_pass_still_registers() {
        let (_, g) = circuit();
        let gate = g[1];
        // 120 m/s at 30 Hz moves 4 m per step; the segment test is not a point test.
        let prev = gate.pos - gate.normal * 3.0;
        let now = gate.pos + gate.normal * 3.0;
        assert!(crossed_gate(prev, now, &gate, GATE_RADIUS));
    }

    #[test]
    fn faster_arrival_scores_more() {
        assert!(gate_points(90.0, 0) > gate_points(40.0, 0));
        assert!(gate_points(50.0, 3) > gate_points(50.0, 0));
    }
}
