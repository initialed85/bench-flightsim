//! Deterministic terrain. Both the server and the clients generate the identical
//! height grid from the seed, so the world is transferred as a seed, and the
//! surface a plane collides with on the server is the surface a client renders.
//!
//! The grid is split into triangles along the (i,j) -> (i+1,j+1) diagonal, and
//! `surface_height` uses the same two triangles, so collision and rendering agree
//! exactly instead of differing by a bilinear saddle.

use glam::Vec3;

/// World spans `-WORLD_HALF..WORLD_HALF` on X and Z.
pub const WORLD_HALF: f32 = 2400.0;
pub const GRID_STEP: f32 = 20.0;
pub const GRID_N: usize = 241;

pub const SEA_LEVEL: f32 = 0.0;
/// Airfield plateau height. The runway is flat at this height.
pub const RUNWAY_Y: f32 = 18.0;
/// Painted runway half width (the slab itself).
pub const RUNWAY_HALF_WIDTH: f32 = 20.0;
pub const RUNWAY_HALF_LENGTH: f32 = 450.0;
/// Width of the flat apron the terrain is blended to.
const FLAT_HALF_WIDTH: f32 = 46.0;
const FLAT_HALF_LENGTH: f32 = 470.0;
/// Distance over which the apron blends into the natural terrain.
const APRON_BLEND: f32 = 150.0;

pub const SEED: i64 = 0x5EED_C0DE_F119_8542;

pub fn hash2(x: i64, y: i64, seed: i64) -> i64 {
    let mut h = (x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ (y as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F)
        ^ (seed as u64).wrapping_mul(0x1656_67B1_9E37_79F9);
    h ^= h >> 33;
    h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
    h ^= h >> 33;
    h as i64
}

/// Deterministic hash to `[0, 1)`. Uses integer shifts only: no platform maths.
pub fn hash01(x: i64, y: i64, seed: i64) -> f32 {
    let h = hash2(x, y, seed) as u64;
    ((h >> 11) as f32) * (1.0 / (1u64 << 53) as f32)
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    if b == a {
        return if x >= b { 1.0 } else { 0.0 };
    }
    let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Smooth value noise on an integer lattice.
fn vnoise(x: f32, z: f32, seed: i64) -> f32 {
    let xi = x.floor();
    let zi = z.floor();
    let fx = x - xi;
    let fz = z - zi;
    let u = fx * fx * (3.0 - 2.0 * fx);
    let v = fz * fz * (3.0 - 2.0 * fz);
    let (ix, iz) = (xi as i64, zi as i64);
    let a = hash01(ix, iz, seed);
    let b = hash01(ix + 1, iz, seed);
    let c = hash01(ix, iz + 1, seed);
    let d = hash01(ix + 1, iz + 1, seed);
    lerp(lerp(a, b, u), lerp(c, d, u), v)
}

/// Fractal value noise, roughly `0..1`.
pub fn fbm(x: f32, z: f32, seed: i64, octaves: usize) -> f32 {
    let mut sum = 0.0f32;
    let mut amp = 1.0f32;
    let mut freq = 1.0 / 260.0;
    let mut norm = 0.0f32;
    for i in 0..octaves {
        sum += amp
            * vnoise(
                x * freq,
                z * freq,
                seed.wrapping_add((i as i64).wrapping_mul(1013)),
            );
        norm += amp;
        amp *= 0.5;
        freq *= 2.17;
    }
    sum / norm
}

/// Ridged noise: sharp crests, roughly `0..1`.
fn ridged(x: f32, z: f32, seed: i64, octaves: usize) -> f32 {
    let mut sum = 0.0f32;
    let mut amp = 1.0f32;
    let mut freq = 1.0 / 190.0;
    let mut norm = 0.0f32;
    for i in 0..octaves {
        let n = vnoise(
            x * freq + 41.0,
            z * freq - 17.0,
            seed.wrapping_add((i as i64).wrapping_mul(733)),
        );
        let r = 1.0 - (2.0 * n - 1.0).abs();
        sum += amp * r * r;
        norm += amp;
        amp *= 0.5;
        freq *= 2.03;
    }
    sum / norm
}

/// 1 inside the runway apron, 0 outside it.
fn apron(x: f32, z: f32) -> f32 {
    let dx = (x.abs() - FLAT_HALF_WIDTH).max(0.0);
    let dz = (z.abs() - FLAT_HALF_LENGTH).max(0.0);
    let d = (dx * dx + dz * dz).sqrt();
    1.0 - smoothstep(0.0, APRON_BLEND, d)
}

pub fn is_runway(x: f32, z: f32) -> bool {
    x.abs() <= RUNWAY_HALF_WIDTH && z.abs() <= RUNWAY_HALF_LENGTH
}

/// Natural (pre-apron) terrain height. `Terrain::generate` and tests both use it.
pub fn height_raw(x: f32, z: f32, seed: i64) -> f32 {
    let d = (x * x + z * z).sqrt();
    let rolling = fbm(x, z, seed, 5);
    let crest = ridged(x, z, seed ^ 0x51AB_C0DE, 4);

    // An island: land rises inland, drops to the sea floor at the edges.
    let island = 1.0 - smoothstep(1500.0, 2300.0, d);
    // Mountains only away from the airfield so the field stays flyable.
    let mountain = smoothstep(300.0, 1000.0, d);
    // Lakes are carved only away from the field, so the runway is never flooded.
    let lake = (0.36 - rolling).max(0.0) * smoothstep(420.0, 900.0, d);

    let land = 26.0 + rolling * 105.0 + crest * 265.0 * mountain;
    let h = -16.0 + island * (land + 16.0) - lake * 110.0;
    h.clamp(-30.0, 460.0)
}

pub struct Terrain {
    pub seed: i64,
    /// Row-major height grid, `h[j * GRID_N + i]` at x = -WORLD_HALF + i*GRID_STEP.
    pub h: Vec<f32>,
}

impl Terrain {
    pub fn generate(seed: i64) -> Terrain {
        let mut h = vec![0.0f32; GRID_N * GRID_N];
        for j in 0..GRID_N {
            for i in 0..GRID_N {
                let x = -WORLD_HALF + i as f32 * GRID_STEP;
                let z = -WORLD_HALF + j as f32 * GRID_STEP;
                let natural = height_raw(x, z, seed);
                let a = apron(x, z);
                h[j * GRID_N + i] = lerp(natural, RUNWAY_Y, a);
            }
        }
        Terrain { seed, h }
    }

    fn cell(&self, v: f32) -> usize {
        let c = ((v + WORLD_HALF) / GRID_STEP).floor();
        (c.clamp(0.0, (GRID_N - 2) as f32)) as usize
    }

    fn node(&self, i: usize, j: usize) -> f32 {
        self.h[j * GRID_N + i]
    }

    /// Height of the rendered surface at (x, z): the same two triangles per cell
    /// the client meshes, so a plane touches exactly what you see.
    pub fn surface_height(&self, x: f32, z: f32) -> f32 {
        let i = self.cell(x);
        let j = self.cell(z);
        let fx = ((x + WORLD_HALF) / GRID_STEP) - i as f32;
        let fz = ((z + WORLD_HALF) / GRID_STEP) - j as f32;
        let h00 = self.node(i, j);
        let h10 = self.node(i + 1, j);
        let h01 = self.node(i, j + 1);
        let h11 = self.node(i + 1, j + 1);

        if fx >= fz {
            // Triangle (0,0) (1,0) (1,1)
            h00 * (1.0 - fx) + h10 * (fx - fz) + h11 * fz
        } else {
            // Triangle (0,0) (1,1) (0,1)
            h00 * (1.0 - fz) + h11 * fx + h01 * (fz - fx)
        }
    }

    /// Surface normal from central differences, for shading.
    pub fn normal(&self, x: f32, z: f32) -> Vec3 {
        let i = self.cell(x).min(GRID_N - 2).max(1);
        let j = self.cell(z).min(GRID_N - 2).max(1);
        let dx = (self.node(i + 1, j) - self.node(i - 1, j)) / (2.0 * GRID_STEP);
        let dz = (self.node(i, j + 1) - self.node(i, j - 1)) / (2.0 * GRID_STEP);
        Vec3::new(-dx, 1.0, -dz).normalize()
    }

    /// Vertex colour: seabed, sand, grass, dry grass, rock, snow.
    pub fn color(&self, x: f32, z: f32) -> [f32; 3] {
        let h = self.surface_height(x, z);
        let n = self.normal(x, z);
        let slope = 1.0 - n.y;

        if h < -1.0 {
            // Seabed, shallower is brighter.
            let t = smoothstep(-26.0, 0.0, h);
            [
                lerp(0.07, 0.20, t),
                lerp(0.16, 0.35, t),
                lerp(0.22, 0.40, t),
            ]
        } else if h < 4.0 {
            [0.78, 0.72, 0.52]
        } else if h > 235.0 {
            [0.93, 0.95, 0.97]
        } else if slope > 0.42 && h > 90.0 {
            [0.52, 0.52, 0.55]
        } else {
            let patch = fbm(x * 1.7, z * 1.7, self.seed ^ 0xBEEF_1, 2);
            let dry = smoothstep(0.58, 0.78, patch);
            let alt = smoothstep(150.0, 220.0, h);
            let g = [
                lerp(0.27, 0.20, alt),
                lerp(0.55, 0.46, alt),
                lerp(0.24, 0.22, alt),
            ];
            [
                lerp(g[0], 0.58, dry),
                lerp(g[1], 0.54, dry),
                lerp(g[2], 0.34, dry),
            ]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_twice_is_identical() {
        let a = Terrain::generate(SEED);
        let b = Terrain::generate(SEED);
        assert_eq!(a.h, b.h);
        assert_eq!(a.h.len(), GRID_N * GRID_N);
    }

    #[test]
    fn grid_nodes_equal_the_surface_function() {
        let t = Terrain::generate(SEED);
        for k in 0..40 {
            let i = 40 + k * 4;
            let j = 60 + k * 3;
            let x = -WORLD_HALF + i as f32 * GRID_STEP;
            let z = -WORLD_HALF + j as f32 * GRID_STEP;
            // Exactly on a node, both triangles return the node height.
            assert!((t.surface_height(x, z) - t.node(i, j)).abs() < 1e-4);
        }
    }

    #[test]
    fn triangles_agree_across_the_diagonal() {
        let t = Terrain::generate(SEED);
        let i = 100;
        let j = 120;
        for s in 0..10 {
            let f = s as f32 / 10.0;
            let x = -WORLD_HALF + (i as f32 + f) * GRID_STEP;
            let z = -WORLD_HALF + (j as f32 + f) * GRID_STEP;
            let above = t.surface_height(x - 0.01, z - 0.01);
            let here = t.surface_height(x, z);
            let below = t.surface_height(x + 0.01, z + 0.01);
            // Crossing the diagonal must not jump.
            assert!((above - here).abs() < 0.05, "{above} {here} {below}");
            assert!((below - here).abs() < 0.05);
        }
    }

    #[test]
    fn runway_is_flat_and_above_sea() {
        let t = Terrain::generate(SEED);
        for z in (-400..400).step_by(50) {
            for x in (-18..18).step_by(6) {
                let h = t.surface_height(x as f32, z as f32);
                assert!((h - RUNWAY_Y).abs() < 0.01, "runway point {x} {z} = {h}");
            }
        }
        assert!(RUNWAY_Y > SEA_LEVEL);
    }

    #[test]
    fn island_shelves_into_the_sea() {
        let t = Terrain::generate(SEED);
        assert!(
            height_raw(0.0, 0.0, SEED) > RUNWAY_Y,
            "the field should be carved out of higher ground"
        );
        assert!(
            height_raw(0.0, 0.0, SEED) < 120.0,
            "no mountains under the field: {}",
            height_raw(0.0, 0.0, SEED)
        );
        assert!(t.surface_height(2350.0, 0.0) < SEA_LEVEL);
        assert!(t.surface_height(0.0, -2350.0) < SEA_LEVEL);
    }

    #[test]
    fn highest_terrain_is_flyable_and_varied() {
        let t = Terrain::generate(SEED);
        let max = t.h.iter().copied().fold(0.0f32, f32::max);
        let min = t.h.iter().copied().fold(0.0f32, f32::min);
        assert!(max > 160.0, "need mountains, got {max}");
        assert!(min < SEA_LEVEL, "need water, got {min}");
    }

    #[test]
    fn normals_point_up() {
        let t = Terrain::generate(SEED);
        let n = t.normal(0.0, 0.0);
        assert!(n.y > 0.99);
    }
}
