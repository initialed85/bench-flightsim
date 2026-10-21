//! Client side: a Bevy scene driven by server snapshots plus a local prediction of
//! the same deterministic flight model, so the controls feel direct while the server
//! stays authoritative.

use std::collections::HashMap;

use bevy::prelude::*;
use bevy::window::WindowResolution;
use flight_common::{
    Aircraft, Controls, Crash, GameEvent, PlaneInfo, PlaneState, Readout, Terrain, AIRCRAFT, ClientMsg,
    SEED, state_from_info, step_plane,
};

pub mod hud;
pub mod net;
pub mod scene;

/// Fixed step, identical to the server's so the two integrations agree.
pub const DT: f32 = 1.0 / 30.0;

#[derive(Resource)]
pub struct Game {
    pub terrain: Terrain,
    pub me: u64,
    pub name: String,
    pub connected: bool,
    pub status: String,
    pub board: String,
    pub log: Vec<String>,
    /// Power lever, moved by the keys rather than snapped.
    pub throttle: f32,
    pub controls: Controls,
    /// Local prediction of this client's airplane.
    pub local: Option<PlaneState>,
    /// What the server says about this client's airplane.
    pub authority: Option<PlaneState>,
    pub readout: Readout,
    pub accrue: f32,
    pub tick: u64,
    pub sim_time: f64,
    pub ping_ms: f32,
    pub last_send: f64,
    pub last_ping: f64,
    pub stamp: f64,
    /// Rendered state per airplane id, and what the server last said.
    pub entities: HashMap<u64, Entity>,
    pub shown: HashMap<u64, PlaneState>,
    pub want: HashMap<u64, PlaneState>,
    pub names: HashMap<u64, String>,
    pub crashed: HashMap<u64, bool>,
}

impl Default for Game {
    fn default() -> Self {
        Game {
            terrain: Terrain::generate(SEED),
            me: 0,
            name: "pilot".into(),
            connected: false,
            status: "connecting".into(),
            board: String::new(),
            log: Vec::new(),
            throttle: 0.7,
            controls: Controls::default(),
            local: None,
            authority: None,
            readout: Readout::default(),
            accrue: 0.0,
            tick: 0,
            sim_time: 0.0,
            ping_ms: 0.0,
            last_send: 0.0,
            last_ping: 0.0,
            stamp: 0.0,
            entities: HashMap::new(),
            shown: HashMap::new(),
            want: HashMap::new(),
            names: HashMap::new(),
            crashed: HashMap::new(),
        }
    }
}

impl Game {
    pub fn push_event(&mut self, text: String) {
        self.log.push(text);
        if self.log.len() > 8 {
            self.log.remove(0);
        }
    }

    pub fn apply_snapshot(&mut self, tick: u64, time: f64, planes: &[PlaneInfo], events: &[GameEvent]) {
        self.tick = tick;
        self.sim_time = time;
        self.connected = true;
        for p in planes {
            let st = state_from_info(p);
            self.want.insert(p.id, st);
            self.names.insert(p.id, p.name.clone());
            self.crashed.insert(p.id, !p.alive);
            if p.id == self.me {
                self.authority = Some(st);
                if self.local.is_none() {
                    self.local = Some(st);
                }
            }
        }
        for e in events {
            let line = match e {
                GameEvent::Gate { name, gate, lap, points, total, .. } => {
                    format!("{name} gate {gate} lap {lap} +{points} ({total})")
                }
                GameEvent::Crash { name, reason, .. } => format!("{name} crashed: {reason}"),
                GameEvent::Kill { name, victim_name, points, .. } => {
                    format!("{name} shot down {victim_name} +{points}")
                }
                GameEvent::Damage { victim_name, hp, .. } => format!("{victim_name} hit, hp {hp:.0}"),
                GameEvent::Land { name, points, on_runway, .. } => {
                    format!("{name} landed {} +{points}", if *on_runway { "on the runway" } else { "in the field" })
                }
                GameEvent::Takeoff { name, .. } => format!("{name} took off"),
                GameEvent::Join { name, .. } => format!("{name} joined"),
                GameEvent::Leave { name, .. } => format!("{name} left"),
            };
            self.push_event(line);
        }
        let mut rows: Vec<&PlaneInfo> = planes.iter().collect();
        rows.sort_by(|a, b| b.score.cmp(&a.score).then(a.name.cmp(&b.name)));
        self.board = rows
            .iter()
            .map(|p| {
                format!(
                    "{:<16} {:>5}  gate {:>2}/10  lap {}  hp {:>3.0}  {} ms{}",
                    p.name,
                    p.score,
                    p.gate,
                    p.lap,
                    p.hp,
                    p.ping_ms.round(),
                    if p.alive { "" } else { "  [down]" }
                )
            })
            .collect::<Vec<_>>()
            .join("\n");
    }

    pub fn apply_welcome(&mut self, id: u64, name: String, you: &PlaneInfo, message: String) {
        self.me = id;
        self.name = name;
        self.local = Some(state_from_info(you));
        self.authority = Some(state_from_info(you));
        self.status = message;
    }
}

pub fn run() {
    let mut app = App::new();
    let (w, h) = hud::viewport();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "bench-flightsim".into(),
            canvas: Some("#game".to_string()),
            resolution: WindowResolution::new(w, h),
            prevent_default_event_handling: false,
            ..default()
        }),
        ..default()
    }));
    app.init_resource::<Game>();
    app.add_systems(Startup, scene::build);
    app.add_systems(Update, (net::connect, net::poll, hud::resize).chain());
    app.add_systems(Update, (read_input, predict, net::send_input, net::keep_alive).chain());
    app.add_systems(Update, (reconcile, scene::sync_planes, scene::chase_camera, hud::draw).chain());
    app.run();
}

/// Keys to control surfaces. Up arrow pulls (nose up), down pushes.
fn read_input(keys: Res<ButtonInput<KeyCode>>, time: Res<Time>, mut game: ResMut<Game>) {
    let dt = time.delta_secs();
    if keys.pressed(KeyCode::KeyW) {
        game.throttle = (game.throttle + 0.6 * dt).min(1.0);
    }
    if keys.pressed(KeyCode::KeyS) {
        game.throttle = (game.throttle - 0.9 * dt).max(0.0);
    }
    let axis = |plus: KeyCode, minus: KeyCode| {
        let p = keys.pressed(plus) as i8;
        let m = keys.pressed(minus) as i8;
        (p - m) as f32
    };
    game.controls = Controls {
        aileron: axis(KeyCode::KeyD, KeyCode::KeyA),
        elevator: axis(KeyCode::ArrowUp, KeyCode::ArrowDown),
        rudder: axis(KeyCode::ArrowRight, KeyCode::ArrowLeft),
        throttle: game.throttle,
        boost: keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight),
        trigger: keys.pressed(KeyCode::Space),
        brake: keys.pressed(KeyCode::KeyB),
    }
    .clamped();
    if keys.just_pressed(KeyCode::KeyR) {
        net::send(&ClientMsg::Respawn);
    }
}

/// Step the shared model locally at the server's rate so the airplane answers instantly.
fn predict(mut game: ResMut<Game>, time: Res<Time>) {
    let Some(mut state) = game.local.take() else { return };
    game.accrue += time.delta_secs();
    let mut steps = 0;
    while game.accrue >= DT && steps < 6 {
        game.accrue -= DT;
        steps += 1;
        let (r, out) = step_plane(&mut state, &game.controls, &game.terrain, DT, &AIRCRAFT);
        game.readout = r;
        match out.crash {
            Some(Crash::Terrain) => game.status = "down: hit the ground".into(),
            Some(Crash::Water) => game.status = "down: hit the sea".into(),
            None => {
                if out.landed {
                    game.status = "on the ground".into();
                } else if out.took_off {
                    game.status = "airborne".into();
                }
            }
        }
    }
    game.local = Some(state);
}

/// Pull the prediction back onto the authority: blend small error, snap big error.
fn reconcile(mut game: ResMut<Game>) {
    let Some(auth) = game.authority.clone() else { return };
    let Some(local) = game.local.as_mut() else {
        game.local = Some(auth);
        return;
    };
    let gap = local.pos.distance(auth.pos);
    if gap > 120.0 {
        *local = auth;
        return;
    }
    let k = 0.18_f32;
    local.pos = local.pos.lerp(auth.pos, k);
    local.vel = local.vel.lerp(auth.vel, k);
    local.rot = local.rot.slerp(auth.rot, k);
    local.body = local.body.lerp(auth.body, k);
    local.grounded = auth.grounded;
}

#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

/// Browsers call this on module load (see web/index.html).
#[cfg_attr(target_arch = "wasm32", wasm_bindgen(start))]
pub fn start() {
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();
    run();
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
pub fn send_chat(text: &str) {
    net::send(&ClientMsg::Chat { text: text.to_string() });
}

/// Re-exported so `scene` can name the aircraft type without importing it twice.
pub type Plane = Aircraft;
