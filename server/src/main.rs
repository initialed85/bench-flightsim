//! Authoritative flight-sim server: one port (default 23456) serves the browser
//! page over HTTP and the game transport over WebSocket.
//!
//! One 30 Hz simulation tick owns every airplane, the circuit and gunfire. Clients
//! push control surfaces at display rate; the tick samples whatever the stick was at
//! that moment. Clients predict the same physics locally (flight-common), so the wire
//! only carries authoritative state.

use std::collections::HashMap;
use std::fs;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures_util::{SinkExt, StreamExt};
use tokio::io::AsyncWriteExt;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc::UnboundedSender;
use tokio_tungstenite::{accept_async, tungstenite::Message};

use flight_common::{
    crossed_gate, encode, gate_points, gates, is_runway, pos_array, step_plane, Bullet, ClientMsg,
    Controls, Crash, GameEvent, Gate, PlaneInfo, PlaneState, ServerMsg, Terrain, AIRCRAFT,
    GATE_RADIUS, GRID_N, GRID_STEP, PROTOCOL_VERSION, RUNWAY_Y, SEED,
};

use glam::Vec3;

const DEFAULT_PORT: u16 = 23456;
const SIM_HZ: f32 = 30.0;
const SIM_DT: f32 = 1.0 / SIM_HZ;
const MAX_PLAYERS: usize = 16;
const MAX_NAME: usize = 24;

const MAX_HP: f32 = 100.0;
const RESPAWN_SECONDS: f32 = 4.0;
const BULLET_SPEED: f32 = 480.0;
const BULLET_TTL: f32 = 1.4;
const BULLET_COOLDOWN: f32 = 0.3;
const BULLET_DAMAGE: f32 = 34.0;
const HIT_RADIUS: f32 = 6.0;

const SCORE_KILL: i32 = 150;
const SCORE_HIT: i32 = 25;
const SCORE_LAP: i32 = 300;
const SCORE_RUNWAY_LAND: i32 = 60;
const SCORE_FIELD_LAND: i32 = 15;
const SCORE_TAKEOFF: i32 = 10;
const PENALTY_CRASH: i32 = 120;
/// A touchdown only counts after this much airtime, so landings cannot be farmed.
const MIN_AIRBORNE_TIME: f32 = 8.0;

pub struct Player {
    pub id: u64,
    pub name: String,
    pub state: PlaneState,
    pub controls: Controls,
    pub hp: f32,
    pub score: i32,
    pub gate: usize,
    pub lap: u32,
    pub alive: bool,
    pub respawn_in: f32,
    pub last_fire: f32,
    pub airborne_for: f32,
    pub crash_streak: u32,
    pub gates_done: u32,
    pub ping: f32,
    pub slot: u32,
}

pub struct LiveBullet {
    pub wire: Bullet,
    pub ttl: f32,
}

pub struct World {
    pub terrain: Terrain,
    pub gates: Vec<Gate>,
    pub players: HashMap<u64, Player>,
    pub bullets: Vec<LiveBullet>,
    pub events: Vec<GameEvent>,
    pub conns: HashMap<u64, UnboundedSender<Message>>,
    pub next_id: u64,
    pub next_bullet: u64,
    pub tick: u64,
    pub time: f32,
}

impl World {
    pub fn new(seed: i64) -> World {
        let terrain = Terrain::generate(seed);
        let gates = gates(&terrain);
        println!("[sim] seed {seed:#x}: {GRID_N}x{GRID_N} height grid at {GRID_STEP} m steps");
        println!(
            "[sim] runway {} m long at y {RUNWAY_Y} m, {} gates:",
            2 * flight_common::RUNWAY_HALF_LENGTH as i32,
            gates.len()
        );
        for (i, g) in gates.iter().enumerate() {
            println!(
                "[sim]   gate {i:>2}  x{:>7.0} y{:>6.0} z{:>7.0}   ground {:>6.1}   leg {:>6.0} m",
                g.pos.x,
                g.pos.y,
                g.pos.z,
                terrain.surface_height(g.pos.x, g.pos.z),
                g.pos.distance(gates[(i + 1) % gates.len()].pos),
            );
        }
        World {
            terrain,
            gates,
            players: HashMap::new(),
            bullets: Vec::new(),
            events: Vec::new(),
            conns: HashMap::new(),
            next_id: 1,
            next_bullet: 1,
            tick: 0,
            time: 0.0,
        }
    }

    /// Spawn slot: trail behind the threshold heading north into gate 0.
    fn slot_pos(&self, slot: u32) -> Vec3 {
        let lane = (slot % 4) as f32 - 1.5;
        let trail = (slot / 4) as f32;
        let x = lane * 90.0;
        let z = -640.0 - trail * 170.0;
        let ground = self.terrain.surface_height(x, z);
        Vec3::new(x, 400.0f32.max(ground + 220.0), z)
    }

    fn fresh_state(&self, slot: u32) -> PlaneState {
        PlaneState::spawn(self.slot_pos(slot), 0.0, 48.0)
    }

    pub fn join(&mut self, fallback_name: &str) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let slot = (id - 1) as u32;
        let player = Player {
            id,
            name: fallback_name.to_string(),
            state: self.fresh_state(slot),
            controls: Controls::default(),
            hp: MAX_HP,
            score: 0,
            gate: 0,
            lap: 0,
            alive: true,
            respawn_in: 0.0,
            last_fire: 0.0,
            airborne_for: 0.0,
            crash_streak: 0,
            gates_done: 0,
            ping: 0.0,
            slot,
        };
        let name = player.name.clone();
        self.players.insert(id, player);
        self.events.push(GameEvent::Join {
            id,
            name: name.clone(),
        });
        println!("[net] + {id} {name}");
        id
    }

    pub fn leave(&mut self, id: u64) {
        self.conns.remove(&id);
        if let Some(p) = self.players.remove(&id) {
            let name = p.name.clone();
            self.events.push(GameEvent::Leave {
                id,
                name: name.clone(),
            });
            println!("[net] - {id} {name}");
        }
    }

    pub fn set_name(&mut self, id: u64, name: &str) {
        let clean = sanitize_name(name);
        if let Some(p) = self.players.get_mut(&id) {
            if p.name != clean {
                let old = std::mem::replace(&mut p.name, clean.clone());
                self.events.push(GameEvent::Join {
                    id,
                    name: clean.clone(),
                });
                println!("[net] {id} renamed {old} -> {clean}");
            }
        }
    }

    pub fn apply_input(&mut self, id: u64, controls: Controls, ping: f32) {
        if let Some(p) = self.players.get_mut(&id) {
            p.controls = controls.clamped();
            if ping.is_finite() {
                p.ping = ping.clamp(0.0, 9999.0);
            }
        }
    }

    pub fn request_respawn(&mut self, id: u64) {
        if let Some(p) = self.players.get_mut(&id) {
            p.respawn_in = p.respawn_in.min(0.2);
        }
    }

    fn respawn(&mut self, id: u64) {
        let slot = match self.players.get(&id) {
            Some(p) => p.slot,
            None => return,
        };
        let fresh = self.fresh_state(slot);
        if let Some(p) = self.players.get_mut(&id) {
            p.state = fresh;
            p.hp = MAX_HP;
            p.alive = true;
            p.respawn_in = 0.0;
            p.controls = Controls::default();
            p.airborne_for = 0.0;
        }
    }

    pub fn step(&mut self, dt: f32) {
        self.tick += 1;
        self.time += dt;

        let gates = self.gates.clone();
        let mut respawn_now: Vec<u64> = Vec::new();

        for player in self.players.values_mut() {
            if !player.alive {
                player.respawn_in -= dt;
                if player.respawn_in <= 0.0 {
                    respawn_now.push(player.id);
                }
                continue;
            }

            let controls = player.controls;
            let firing = controls.trigger && (self.time - player.last_fire) >= BULLET_COOLDOWN;
            if firing {
                player.last_fire = self.time;
            }

            let prev_pos = player.state.pos;
            let (readout, outcome) =
                step_plane(&mut player.state, &controls, &self.terrain, dt, &AIRCRAFT);

            if !player.state.grounded {
                player.airborne_for += dt;
            }

            if outcome.took_off {
                player.score += SCORE_TAKEOFF;
                self.events.push(GameEvent::Takeoff {
                    id: player.id,
                    name: player.name.clone(),
                });
            }

            if outcome.landed {
                let on_runway = is_runway(player.state.pos.x, player.state.pos.z);
                let counts = player.airborne_for >= MIN_AIRBORNE_TIME;
                player.airborne_for = 0.0;
                if counts {
                    let points = if on_runway {
                        SCORE_RUNWAY_LAND
                    } else {
                        SCORE_FIELD_LAND
                    };
                    player.score += points;
                    self.events.push(GameEvent::Land {
                        id: player.id,
                        name: player.name.clone(),
                        points,
                        on_runway,
                    });
                }
            }

            // The circuit: gates must be crossed in order, so there is no shortcut.
            if let Some(gate) = gates.get(player.gate) {
                if crossed_gate(prev_pos, player.state.pos, gate, GATE_RADIUS) {
                    let points = gate_points(readout.speed, player.lap);
                    player.score += points;
                    player.gates_done += 1;
                    let crossed = player.gate as u32;
                    player.gate = (player.gate + 1) % gates.len();
                    if player.gate == 0 {
                        player.lap += 1;
                        player.score += SCORE_LAP;
                    }
                    self.events.push(GameEvent::Gate {
                        id: player.id,
                        name: player.name.clone(),
                        gate: crossed,
                        lap: player.lap,
                        points,
                        total: player.score,
                    });
                }
            }

            if firing {
                let fwd = flight_common::forward_of(player.state.rot);
                let origin = player.state.pos + fwd * 4.0;
                self.bullets.push(LiveBullet {
                    wire: Bullet {
                        id: self.next_bullet,
                        owner: player.id,
                        pos: pos_array(origin),
                        vel: pos_array(player.state.vel + fwd * BULLET_SPEED),
                    },
                    ttl: BULLET_TTL,
                });
                self.next_bullet += 1;
            }

            match outcome.crash {
                Some(reason) => {
                    player.alive = false;
                    player.respawn_in = RESPAWN_SECONDS;
                    player.score -= PENALTY_CRASH;
                    player.crash_streak += 1;
                    let label = match reason {
                        Crash::Water => "ditched in the sea",
                        Crash::Terrain if player.crash_streak >= 3 => {
                            "terrain (repeated: reduce throttle, watch the terrain)"
                        }
                        Crash::Terrain => "terrain",
                    };
                    self.events.push(GameEvent::Crash {
                        id: player.id,
                        name: player.name.clone(),
                        reason: label.to_string(),
                        pos: pos_array(player.state.pos),
                    });
                    println!(
                        "[sim] crash {label}: {} at {:?}",
                        player.name, player.state.pos
                    );
                }
                None => {
                    if player.crash_streak > 0 && player.airborne_for > 25.0 {
                        player.crash_streak = 0;
                    }
                }
            }
        }

        for id in respawn_now {
            self.respawn(id);
        }

        self.step_bullets(dt);
    }

    fn step_bullets(&mut self, dt: f32) {
        let mut hits: Vec<(u64, u64)> = Vec::new();

        for bullet in self.bullets.iter_mut() {
            if bullet.ttl <= 0.0 {
                continue;
            }
            bullet.ttl -= dt;

            let from = Vec3::from_array(bullet.wire.pos);
            let to = from + Vec3::from_array(bullet.wire.vel) * dt;
            bullet.wire.pos = pos_array(to);

            // A round travels ~16 m per tick, so terrain and pilots are tested against
            // the segment, not the endpoint.
            let mid = from.lerp(to, 0.5);
            if to.y - 1.5 < self.terrain.surface_height(to.x, to.z)
                || mid.y < self.terrain.surface_height(mid.x, mid.z)
            {
                bullet.ttl = 0.0;
                continue;
            }

            for player in self.players.values() {
                if !player.alive || player.id == bullet.wire.owner {
                    continue;
                }
                if distance_point_segment(player.state.pos, from, to) < HIT_RADIUS {
                    hits.push((bullet.wire.owner, player.id));
                    bullet.ttl = 0.0;
                    break;
                }
            }
        }

        self.bullets.retain(|b| b.ttl > 0.0);

        for (shooter, victim) in hits {
            let (victim_name, destroyed, hp, pos) = match self.players.get_mut(&victim) {
                Some(p) => {
                    p.hp -= BULLET_DAMAGE;
                    let destroyed = p.hp <= 0.0;
                    if destroyed {
                        p.alive = false;
                        p.hp = 0.0;
                        p.respawn_in = RESPAWN_SECONDS;
                    }
                    (p.name.clone(), destroyed, p.hp, p.state.pos)
                }
                None => continue,
            };

            self.events.push(GameEvent::Damage {
                id: shooter,
                victim,
                victim_name: victim_name.clone(),
                hp,
                pos: pos_array(pos),
            });

            if destroyed {
                if let Some(killer) = self.players.get_mut(&shooter) {
                    killer.score += SCORE_KILL;
                    self.events.push(GameEvent::Kill {
                        id: shooter,
                        name: killer.name.clone(),
                        victim,
                        victim_name,
                        points: SCORE_KILL,
                        total: killer.score,
                        pos: pos_array(pos),
                    });
                }
            } else if let Some(killer) = self.players.get_mut(&shooter) {
                killer.score += SCORE_HIT;
            }
        }
    }

    pub fn info(&self, p: &Player) -> PlaneInfo {
        PlaneInfo {
            id: p.id,
            name: p.name.clone(),
            pos: pos_array(p.state.pos),
            vel: pos_array(p.state.vel),
            rot: p.state.rot.to_array(),
            controls: p.controls,
            hp: p.hp,
            score: p.score,
            gate: p.gate as u32,
            lap: p.lap,
            alive: p.alive,
            respawn_in: p.respawn_in.max(0.0),
            ping_ms: p.ping,
        }
    }

    pub fn infos(&self) -> Vec<PlaneInfo> {
        let mut rows: Vec<PlaneInfo> = self.players.values().map(|p| self.info(p)).collect();
        rows.sort_by(|a, b| b.score.cmp(&a.score).then(a.id.cmp(&b.id)));
        rows
    }

    /// Build the next snapshot and drain the event queue.
    pub fn snapshot(&mut self) -> ServerMsg {
        ServerMsg::Snapshot {
            tick: self.tick,
            time: self.time as f64,
            planes: self.infos(),
            bullets: self.bullets.iter().map(|b| b.wire.clone()).collect(),
            events: std::mem::take(&mut self.events),
        }
    }

    pub fn broadcast(&mut self, msg: &Message) {
        let dead: Vec<u64> = self
            .conns
            .iter()
            .filter(|(_, tx)| tx.send(msg.clone()).is_err())
            .map(|(id, _)| *id)
            .collect();
        for id in dead {
            self.conns.remove(&id);
        }
    }

    pub fn standings(&self) -> String {
        let mut rows: Vec<&Player> = self.players.values().collect();
        rows.sort_by(|a, b| b.score.cmp(&a.score).then(a.id.cmp(&b.id)));
        rows.iter()
            .map(|p| {
                format!(
                    "{name} {score:+} g{gate}/{n} lap{lap} {speed:>4.0}km/h {alt:>4.0}m hp{hp:>3.0}{down}",
                    name = p.name,
                    score = p.score,
                    gate = p.gate,
                    n = self.gates.len(),
                    lap = p.lap,
                    speed = p.state.vel.length() * 3.6,
                    alt = p.state.pos.y,
                    hp = p.hp,
                    down = if p.alive { "" } else { " DOWN" },
                )
            })
            .collect::<Vec<_>>()
            .join("  |  ")
    }
}

fn distance_point_segment(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let len2 = ab.length_squared();
    if len2 < 1e-6 {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

fn sanitize_name(name: &str) -> String {
    let cleaned: String = name
        .chars()
        .filter(|c| c.is_alphanumeric() || matches!(c, '-' | '_' | '.' | ' '))
        .take(MAX_NAME)
        .collect();
    let cleaned = cleaned.trim();
    if cleaned.is_empty() {
        "pilot".to_string()
    } else {
        cleaned.to_string()
    }
}

// ------------------------------------------------------------------ transport

const MIME: &[(&str, &str)] = &[
    ("html", "text/html; charset=utf-8"),
    ("js", "text/javascript; charset=utf-8"),
    ("mjs", "text/javascript; charset=utf-8"),
    ("wasm", "application/wasm"),
    ("css", "text/css; charset=utf-8"),
    ("json", "application/json"),
    ("png", "image/png"),
    ("svg", "image/svg+xml"),
    ("ico", "image/x-icon"),
];

fn mime_for(path: &str) -> &'static str {
    let ext = path.rsplit('.').next().unwrap_or("");
    MIME.iter()
        .find(|(e, _)| *e == ext)
        .unwrap_or(&("", "application/octet-stream"))
        .1
}

fn static_path(root: &Path, request_path: &str) -> Option<PathBuf> {
    let path = request_path
        .split('?')
        .next()
        .unwrap_or("/")
        .split('#')
        .next()
        .unwrap_or("/");
    let rel = if path == "/" || path.is_empty() {
        "index.html"
    } else {
        path.trim_start_matches('/')
    };
    let candidate = Path::new(rel);
    if candidate.components().any(|c| {
        matches!(
            c,
            std::path::Component::ParentDir | std::path::Component::RootDir
        )
    }) {
        return None;
    }
    Some(root.join(candidate))
}

/// Read the request head without consuming it, so one port can route both
/// WebSocket upgrades and static files.
async fn peek_head(socket: &mut TcpStream) -> std::io::Result<String> {
    let mut buf = [0u8; 8192];
    let mut got = 0usize;
    for attempt in 0..4 {
        match socket.peek(&mut buf).await {
            Ok(n) if n > got => got = n,
            _ => break,
        }
        if buf[..got].windows(4).any(|w| w == b"\r\n\r\n") {
            break;
        }
        if attempt < 3 {
            tokio::time::sleep(Duration::from_millis(6)).await;
        }
    }
    Ok(String::from_utf8_lossy(&buf[..got]).to_string())
}

async fn respond(
    socket: &mut TcpStream,
    status: u16,
    content_type: &str,
    body: &[u8],
    extra_headers: &str,
) -> std::io::Result<()> {
    let reason = match status {
        200 => "OK",
        400 => "Bad Request",
        404 => "Not Found",
        405 => "Method Not Allowed",
        503 => "Service Unavailable",
        _ => "Error",
    };
    let head = format!(
        "HTTP/1.1 {status} {reason}\r\ncontent-type: {content_type}\r\ncontent-length: {}\r\n{extra_headers}connection: close\r\n\r\n",
        body.len()
    );
    socket.write_all(head.as_bytes()).await?;
    socket.write_all(body).await?;
    socket.flush().await?;
    Ok(())
}

async fn serve_file(
    socket: &mut TcpStream,
    root: &Path,
    request_path: &str,
    accept_gzip: bool,
) -> std::io::Result<()> {
    let file = match static_path(root, request_path) {
        Some(p) => p,
        None => {
            return respond(socket, 400, "text/plain; charset=utf-8", b"bad request", "").await;
        }
    };

    // The build script leaves a .gz next to the wasm blob; serve it when the browser
    // will accept it, which cuts the first-load transfer by roughly two thirds.
    if accept_gzip {
        let gz = file.with_extension(format!(
            "{}.gz",
            file.extension().and_then(|e| e.to_str()).unwrap_or("")
        ));
        if let Ok(body) = fs::read(&gz) {
            return respond(
                socket,
                200,
                mime_for(&file.to_string_lossy()),
                &body,
                "content-encoding: gzip\r\ncache-control: no-store\r\n",
            )
            .await;
        }
    }

    match fs::read(&file) {
        Ok(body) => {
            respond(
                socket,
                200,
                mime_for(&file.to_string_lossy()),
                &body,
                "cache-control: no-store\r\n",
            )
            .await
        }
        Err(e) => {
            let msg = format!("not found: {request_path} ({e})");
            respond(socket, 404, "text/plain; charset=utf-8", msg.as_bytes(), "").await
        }
    }
}

async fn handle_connection(
    mut socket: TcpStream,
    addr: SocketAddr,
    state: Arc<Mutex<World>>,
    web_root: PathBuf,
) -> std::io::Result<()> {
    socket.set_nodelay(true)?;
    let head = peek_head(&mut socket).await?;
    let mut lines = head.lines();
    let request_line = lines.next().unwrap_or("").trim().to_string();
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or("GET");
    let path = parts.next().unwrap_or("/");

    let headers: HashMap<String, String> = lines
        .filter_map(|l| l.split_once(':'))
        .map(|(k, v)| (k.trim().to_lowercase(), v.trim().to_string()))
        .collect();

    let route = path.split('?').next().unwrap_or("/");

    if route == "/ws" || route.starts_with("/ws?") {
        // Leave the stream untouched: accept_async re-reads the upgrade request.
        return run_socket(socket, addr, state).await;
    }

    if method != "GET" && method != "HEAD" {
        return respond(
            &mut socket,
            405,
            "text/plain; charset=utf-8",
            b"method not allowed",
            "",
        )
        .await;
    }

    if route == "/health" {
        let (tick, time, pilots, seed) = {
            let w = state.lock().unwrap();
            (w.tick, w.time, w.players.len(), w.terrain.seed)
        };
        let body = format!(
            "{{\"ok\":true,\"tick\":{tick},\"sim_time\":{time:.2},\"pilots\":{pilots},\"seed\":\"{seed:#x}\",\"sim_hz\":{SIM_HZ}}}"
        );
        return respond(
            &mut socket,
            200,
            "application/json",
            body.as_bytes(),
            "cache-control: no-store\r\n",
        )
        .await;
    }

    let gzip = headers
        .get("accept-encoding")
        .map(|v| v.contains("gzip"))
        .unwrap_or(false);
    serve_file(&mut socket, &web_root, path, gzip).await
}

async fn run_socket(
    socket: TcpStream,
    addr: SocketAddr,
    state: Arc<Mutex<World>>,
) -> std::io::Result<()> {
    let ws = match accept_async(socket).await {
        Ok(ws) => ws,
        Err(e) => {
            eprintln!("[net] {addr}: websocket handshake failed: {e}");
            return Ok(());
        }
    };
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<Message>();
    let (mut sink, mut stream) = ws.split();
    let mut joined: Option<u64> = None;
    tokio::spawn(async move {
        while let Some(frame) = rx.recv().await {
            if sink.send(frame).await.is_err() {
                break;
            }
        }
    });

    let welcome = {
        let mut world = state.lock().unwrap();
        if world.players.len() >= MAX_PLAYERS {
            drop(world);
            None
        } else {
            let id = world.join(&format!("pilot-{}", addr.port()));
            joined = Some(id);
            world.conns.insert(id, tx.clone());

            let player = world.players.get(&id).expect("just joined");
            let you = world.info(player);
            let others: Vec<PlaneInfo> = world.infos().into_iter().filter(|p| p.id != id).collect();
            Some(Message::text(encode(&ServerMsg::Welcome {
                version: PROTOCOL_VERSION,
                id,
                name: player.name.clone(),
                seed: world.terrain.seed,
                tick: world.tick,
                you,
                planes: others,
                message: format!(
                    "{} gates, fly them in order; runway runs north through the origin",
                    world.gates.len()
                ),
            })))
        }
    };
    let welcome = match welcome {
        Some(msg) => msg,
        None => {
            tx.send(Message::text(
                "{\"t\":\"error\",\"message\":\"server full\"}",
            ))
            .ok();
            tokio::time::sleep(Duration::from_millis(120)).await;
            return Ok(());
        }
    };
    tx.send(welcome).ok();

    let id = match joined {
        Some(id) => id,
        None => return Ok(()),
    };

    while let Some(inbound) = stream.next().await {
        match inbound {
            Ok(Message::Text(text)) => handle_client_text(&state, id, &tx, &text),
            Ok(Message::Binary(bytes)) => {
                if let Ok(text) = String::from_utf8(bytes.to_vec()) {
                    handle_client_text(&state, id, &tx, &text);
                }
            }
            Ok(Message::Ping(p)) => {
                tx.send(Message::Pong(p)).ok();
            }
            Ok(Message::Close(_)) | Err(_) => break,
            Ok(_) => {}
        }
    }

    state.lock().unwrap().leave(id);
    Ok(())
}

fn handle_client_text(
    state: &Arc<Mutex<World>>,
    id: u64,
    tx: &UnboundedSender<Message>,
    text: &str,
) {
    let msg: ClientMsg = match flight_common::decode(text) {
        Ok(m) => m,
        Err(e) => {
            tx.send(Message::text(encode(&ServerMsg::Error {
                message: format!("bad message: {e}"),
            })))
            .ok();
            return;
        }
    };

    let mut world = state.lock().unwrap();
    match msg {
        ClientMsg::Hello { name } => world.set_name(id, &name),
        ClientMsg::Input { controls, ping } => world.apply_input(id, controls, ping),
        ClientMsg::Ping { stamp } => {
            tx.send(Message::text(encode(&ServerMsg::Pong { stamp })))
                .ok();
        }
        ClientMsg::Respawn => world.request_respawn(id),
        ClientMsg::Chat { text } => {
            let name = world
                .players
                .get(&id)
                .map(|p| p.name.clone())
                .unwrap_or_else(|| "pilot".to_string());
            let trimmed: String = text.chars().take(200).collect();
            world.broadcast(&Message::text(encode(&ServerMsg::Chat {
                name,
                text: trimmed,
            })));
        }
    }
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let port: u16 = std::env::var("FLIGHT_PORT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_PORT);
    let seed: i64 = std::env::var("FLIGHT_SEED")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(SEED);
    let web_root = std::env::var("FLIGHT_WEB")
        .ok()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("web"));

    if !web_root.join("index.html").exists() {
        eprintln!(
            "[net] warning: {} missing; run scripts/build.sh before opening the page",
            web_root.join("index.html").display()
        );
    }

    let state = Arc::new(Mutex::new(World::new(seed)));

    {
        let state = state.clone();
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(Duration::from_secs_f64(1.0 / SIM_HZ as f64));
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            let started = Instant::now();
            let mut _printed = 0u64;
            loop {
                ticker.tick().await;
                let msg = {
                    let mut world = state.lock().unwrap();
                    world.step(SIM_DT);
                    let snapshot = encode(&world.snapshot());
                    if world.tick % 150 == 0 {
                        println!(
                            "[sim] tick {} {:.1} ms/tick {:.1} MB wire/s {:>2} pilots",
                            world.tick,
                            started.elapsed().as_secs_f64() * 1000.0 / world.tick as f64,
                            snapshot.len() as f64 * 60.0 / 1e6,
                            world.players.len()
                        );
                        println!("[sim] {}", world.standings());
                    }
                    Message::text(snapshot)
                };
                state.lock().unwrap().broadcast(&msg);
            }
        });
    }

    let listener = TcpListener::bind(("0.0.0.0", port)).await?;
    let local = listener.local_addr()?;
    println!("[net] http://{local}  (web root {})", web_root.display());
    println!("[net] ws://{local}/ws   sim {SIM_HZ} Hz   ctrl-c to stop");

    while let Ok((socket, addr)) = listener.accept().await {
        let state = state.clone();
        let web_root = web_root.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(socket, addr, state, web_root).await {
                eprintln!("[net] {addr}: {e}");
            }
        });
    }

    Ok(())
}
