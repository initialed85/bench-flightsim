//! The static world is built from the same deterministic terrain the server simulated,
//! so what you see and what the physics uses are the same surface.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::PrimitiveTopology;
use bevy::prelude::*;
use flight_common::{
    gates, is_runway, quat_align_y, PlaneState, Terrain, GATE_RADIUS, GRID_N, GRID_STEP, WORLD_HALF,
};

use crate::Game;

/// Display grid: 40 m cells, half the resolution of the physics grid.
const STRIDE: usize = 2;
const TAU: f32 = std::f32::consts::TAU;

#[derive(Resource)]
pub struct Art {
    pub plane: Handle<Mesh>,
    pub ring: Handle<Mesh>,
    pub own: Handle<StandardMaterial>,
    pub wreck: Handle<StandardMaterial>,
    pub other: Vec<Handle<StandardMaterial>>,
    pub gate: Handle<StandardMaterial>,
}

pub fn build(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    game: Res<Game>,
) {
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: 0.85,
            near: 0.3,
            far: 9000.0,
            ..default()
        }),
        Transform::from_xyz(0.0, 260.0, -520.0),
    ));

    // Sun plus a weaker fill from the opposite side so the terrain is never black.
    commands.spawn((
        DirectionalLight {
            illuminance: 30000.0,
            color: Color::srgb(1.0, 0.96, 0.88),
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 0.8, -1.0, 0.0)),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 6000.0,
            color: Color::srgb(0.75, 0.83, 1.0),
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, -2.3, 0.5, 0.0)),
    ));

    commands.spawn((
        Mesh3d(meshes.add(terrain_mesh(&game.terrain))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 1.0, 1.0),
            double_sided: true,
            cull_mode: None,
            perceptual_roughness: 0.92,
            ..default()
        })),
    ));

    // Sea: one big translucent plate at mean sea level.
    commands.spawn((
        Mesh3d(meshes.add(Rectangle::new(WORLD_HALF * 6.0, WORLD_HALF * 6.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.10, 0.26, 0.42, 0.82),
            alpha_mode: AlphaMode::Blend,
            double_sided: true,
            cull_mode: None,
            perceptual_roughness: 0.25,
            metallic: 0.15,
            ..default()
        })),
        Transform::from_xyz(0.0, 0.0, 0.0).with_scale(Vec3::new(1.0, 1.0, 1.0)),
    ));

    let ring = meshes.add(ring_mesh(GATE_RADIUS));
    let gate_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.95, 0.72, 0.16),
        double_sided: true,
        cull_mode: None,
        ..default()
    });
    for gate in gates(&game.terrain) {
        commands.spawn((
            Mesh3d(ring.clone()),
            MeshMaterial3d(gate_mat.clone()),
            Transform {
                rotation: quat_align_y(gate.normal),
                translation: gate.pos,
                ..default()
            },
        ));
    }

    commands.insert_resource(Art {
        plane: meshes.add(plane_mesh()),
        ring,
        own: materials.add(StandardMaterial {
            base_color: Color::srgb(0.98, 0.80, 0.15),
            double_sided: true,
            cull_mode: None,
            perceptual_roughness: 0.5,
            ..default()
        }),
        wreck: materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.12, 0.12),
            double_sided: true,
            cull_mode: None,
            ..default()
        }),
        other: [
            [0.85, 0.20, 0.22],
            [0.20, 0.55, 0.85],
            [0.30, 0.75, 0.40],
            [0.75, 0.45, 0.85],
            [0.85, 0.55, 0.20],
            [0.55, 0.60, 0.68],
        ]
        .iter()
        .map(|c| {
            materials.add(StandardMaterial {
                base_color: Color::srgb(c[0], c[1], c[2]),
                double_sided: true,
                cull_mode: None,
                perceptual_roughness: 0.5,
                ..default()
            })
        })
        .collect(),
        gate: gate_mat,
    });
}

fn terrain_mesh(terrain: &Terrain) -> Mesh {
    // Lattice of display nodes, always including the outer edge.
    let mut lat: Vec<usize> = (0..GRID_N).step_by(STRIDE).collect();
    if *lat.last().unwrap() != GRID_N - 1 {
        lat.push(GRID_N - 1);
    }
    let m = lat.len();

    let mut pos: Vec<[f32; 3]> = Vec::with_capacity(m * m);
    let mut nrm: Vec<[f32; 3]> = Vec::with_capacity(m * m);
    let mut col: Vec<[f32; 4]> = Vec::with_capacity(m * m);

    for &j in &lat {
        for &i in &lat {
            let x = -WORLD_HALF + i as f32 * GRID_STEP;
            let z = -WORLD_HALF + j as f32 * GRID_STEP;
            let mut h = terrain.h[j * GRID_N + i];
            let mut c = terrain.color(x, z);
            if is_runway(x, z) {
                // Asphalt laid exactly on the surface the physics uses.
                h += 0.06;
                c = [0.22, 0.23, 0.25];
            }
            let n = terrain.normal(x, z);
            pos.push([x, h, z]);
            nrm.push([n.x, n.y, n.z]);
            col.push([c[0], c[1], c[2], 1.0]);
        }
    }

    let mut idx: Vec<u32> = Vec::with_capacity((m - 1) * (m - 1) * 6);
    for cj in 0..m - 1 {
        for ci in 0..m - 1 {
            let a = (cj * m + ci) as u32;
            let b = a + 1;
            let c = a + m as u32 + 1;
            let d = a + m as u32;
            idx.extend_from_slice(&[a, b, c, a, c, d]);
        }
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, nrm);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
    mesh.insert_indices(bevy::mesh::Indices::U32(idx));
    mesh
}

/// Flat annulus in the local XZ plane: `quat_align_y` puts its hole along the gate normal.
fn ring_mesh(radius: f32) -> Mesh {
    const SEG: usize = 32;
    let stations: [f32; 2] = [radius - 1.4, radius + 1.4];

    let mut pos: Vec<[f32; 3]> = Vec::new();
    let mut nrm: Vec<[f32; 3]> = Vec::new();
    let mut col: Vec<[f32; 4]> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();

    for (band, r) in stations.iter().enumerate() {
        for s in 0..=SEG {
            let a = s as f32 / SEG as f32 * TAU;
            pos.push([r * a.cos(), 0.0, r * a.sin()]);
            nrm.push([0.0, if band == 0 { -1.0 } else { 1.0 }, 0.0]);
            col.push([1.0, 1.0, 1.0, 1.0]);
        }
    }
    for s in 0..SEG as u32 {
        let a = s;
        let b = s + 1;
        let c = s + SEG as u32 + 1;
        let d = s + SEG as u32 + 2;
        idx.extend_from_slice(&[a, b, c, a, c, d]);
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, nrm);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
    mesh.insert_indices(bevy::mesh::Indices::U32(idx));
    mesh
}

fn add_box(
    pos: &mut Vec<[f32; 3]>,
    nrm: &mut Vec<[f32; 3]>,
    col: &mut Vec<[f32; 4]>,
    idx: &mut Vec<u32>,
    lo: [f32; 3],
    hi: [f32; 3],
    tint: [f32; 4],
) {
    const FACES: [([f32; 3], [[f32; 3]; 4]); 6] = [
        (
            [1.0, 0.0, 0.0],
            [[1., 0., 0.], [1., 0., 1.], [1., 1., 1.], [1., 1., 0.]],
        ),
        (
            [-1.0, 0.0, 0.0],
            [[0., 0., 0.], [0., 1., 0.], [0., 1., 1.], [0., 0., 1.]],
        ),
        (
            [0.0, 1.0, 0.0],
            [[0., 1., 0.], [0., 1., 1.], [1., 1., 1.], [1., 1., 0.]],
        ),
        (
            [0.0, -1.0, 0.0],
            [[0., 0., 0.], [1., 0., 0.], [1., 0., 1.], [0., 0., 1.]],
        ),
        (
            [0.0, 0.0, 1.0],
            [[0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]],
        ),
        (
            [0.0, 0.0, -1.0],
            [[0., 0., 0.], [0., 1., 0.], [1., 1., 0.], [1., 0., 0.]],
        ),
    ];
    for (n, corners) in FACES {
        let base = pos.len() as u32;
        for c in corners {
            pos.push([
                if c[0] > 0.5 { hi[0] } else { lo[0] },
                if c[1] > 0.5 { hi[1] } else { lo[1] },
                if c[2] > 0.5 { hi[2] } else { lo[2] },
            ]);
            nrm.push(n);
            col.push(tint);
        }
        idx.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
}

/// A fixed-wing shape whose nose points along local -Z, matching `PlaneState`'s frame.
fn plane_mesh() -> Mesh {
    let mut pos: Vec<[f32; 3]> = Vec::new();
    let mut nrm: Vec<[f32; 3]> = Vec::new();
    let mut col: Vec<[f32; 4]> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();

    let white = [0.92, 0.93, 0.95, 1.0];
    let dark = [0.30, 0.32, 0.36, 1.0];

    add_box(
        &mut pos,
        &mut nrm,
        &mut col,
        &mut idx,
        [-0.45, -0.40, -2.9],
        [0.45, 0.48, 1.7],
        white,
    );
    add_box(
        &mut pos,
        &mut nrm,
        &mut col,
        &mut idx,
        [-0.22, -0.22, -4.0],
        [0.22, 0.22, -2.9],
        dark,
    );
    add_box(
        &mut pos,
        &mut nrm,
        &mut col,
        &mut idx,
        [-4.9, -0.10, -1.5],
        [4.9, 0.10, -0.1],
        white,
    );
    add_box(
        &mut pos,
        &mut nrm,
        &mut col,
        &mut idx,
        [-1.8, -0.07, 1.0],
        [1.8, 0.07, 1.9],
        white,
    );
    add_box(
        &mut pos,
        &mut nrm,
        &mut col,
        &mut idx,
        [-0.07, 0.40, 1.0],
        [0.07, 1.6, 1.9],
        dark,
    );

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, pos);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, nrm);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, col);
    mesh.insert_indices(bevy::mesh::Indices::U32(idx));
    mesh
}

fn blend(a: &mut PlaneState, b: &PlaneState, k: f32) {
    a.pos = a.pos.lerp(b.pos, k);
    a.vel = a.vel.lerp(b.vel, k);
    a.rot = a.rot.slerp(b.rot, k);
    a.body = a.body.lerp(b.body, k);
    a.grounded = b.grounded;
}

pub fn sync_planes(
    mut game: ResMut<Game>,
    art: Res<Art>,
    mut commands: Commands,
    mut view: Query<(&mut Transform, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let ids: Vec<u64> = game.want.keys().copied().collect();
    for id in ids {
        let target = match game.want.get(&id) {
            Some(t) => *t,
            None => continue,
        };
        let mut shown = game.shown.get(&id).copied().unwrap_or(target);
        blend(&mut shown, &target, 0.4);
        game.shown.insert(id, shown);

        let dead = game.crashed.get(&id).copied().unwrap_or(false);
        let mine = id == game.me;
        let material = if dead {
            art.wreck.clone()
        } else if mine {
            art.own.clone()
        } else {
            art.other[(id as usize) % art.other.len()].clone()
        };

        if let Some(entity) = game.entities.get(&id).copied() {
            match view.get_mut(entity) {
                Ok((mut t, mut mat)) => {
                    t.translation = shown.pos;
                    t.rotation = shown.rot;
                    if mat.0 != material {
                        *mat = MeshMaterial3d(material);
                    }
                }
                Err(_) => {
                    let e = commands
                        .spawn((
                            Mesh3d(art.plane.clone()),
                            MeshMaterial3d(material),
                            Transform {
                                rotation: shown.rot,
                                translation: shown.pos,
                                ..default()
                            },
                        ))
                        .id();
                    game.entities.insert(id, e);
                }
            }
        } else {
            let e = commands
                .spawn((
                    Mesh3d(art.plane.clone()),
                    MeshMaterial3d(material),
                    Transform {
                        rotation: shown.rot,
                        translation: shown.pos,
                        ..default()
                    },
                ))
                .id();
            game.entities.insert(id, e);
        }
    }

    let stale: Vec<(u64, Entity)> = game
        .entities
        .iter()
        .filter(|(id, _)| !game.want.contains_key(id))
        .map(|(id, e)| (*id, *e))
        .collect();
    for (id, entity) in stale {
        game.entities.remove(&id);
        game.shown.remove(&id);
        game.want.remove(&id);
        game.names.remove(&id);
        game.crashed.remove(&id);
        commands.entity(entity).despawn();
    }
}

pub fn chase_camera(game: Res<Game>, mut cams: Query<&mut Transform, With<Camera3d>>) {
    let Some(st) = game.local.as_ref() else {
        return;
    };
    let Ok(mut cam) = cams.single_mut() else {
        return;
    };
    cam.translation = st.pos + st.rot * Vec3::new(0.0, 2.4, 12.0);
    cam.rotation = st.rot;
}
