//! Wire protocol. JSON frames over the WebSocket, one message per frame, so a
//! browser can read them straight off the wire and `scripts/sniff.py` can print them.

use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use crate::flight::{Controls, PlaneState};

pub const PROTOCOL_VERSION: u8 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneInfo {
    pub id: u64,
    pub name: String,
    pub pos: [f32; 3],
    pub vel: [f32; 3],
    /// xyzw quaternion.
    pub rot: [f32; 4],
    /// Echoed back so the client can animate the prop and check what the server applied.
    pub controls: Controls,
    pub hp: f32,
    pub score: i32,
    /// Next gate index this airplane must cross.
    pub gate: u32,
    pub lap: u32,
    pub alive: bool,
    /// Seconds until it is back in the air; 0.0 while alive.
    pub respawn_in: f32,
    pub ping_ms: f32,
}

impl PlaneInfo {
    pub fn pos(&self) -> Vec3 {
        Vec3::from_array(self.pos)
    }
    pub fn vel(&self) -> Vec3 {
        Vec3::from_array(self.vel)
    }
    pub fn rot(&self) -> Quat {
        Quat::from_array(self.rot)
    }
}

pub fn pos_array(v: Vec3) -> [f32; 3] {
    [v.x, v.y, v.z]
}

pub fn quat_array(q: Quat) -> [f32; 4] {
    q.to_array()
}

pub fn state_from_info(info: &PlaneInfo) -> PlaneState {
    PlaneState {
        pos: info.pos(),
        vel: info.vel(),
        rot: info.rot(),
        body: glam::Vec3::ZERO,
        grounded: false,
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Bullet {
    pub id: u64,
    pub owner: u64,
    pub pos: [f32; 3],
    pub vel: [f32; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "e", rename_all = "snake_case")]
pub enum GameEvent {
    Gate {
        id: u64,
        name: String,
        gate: u32,
        lap: u32,
        points: i32,
        total: i32,
    },
    Crash {
        id: u64,
        name: String,
        reason: String,
        pos: [f32; 3],
    },
    Kill {
        id: u64,
        name: String,
        victim: u64,
        victim_name: String,
        points: i32,
        total: i32,
        pos: [f32; 3],
    },
    Damage {
        id: u64,
        victim: u64,
        victim_name: String,
        hp: f32,
        pos: [f32; 3],
    },
    Land {
        id: u64,
        name: String,
        points: i32,
        on_runway: bool,
    },
    Takeoff {
        id: u64,
        name: String,
    },
    Join {
        id: u64,
        name: String,
    },
    Leave {
        id: u64,
        name: String,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "t", rename_all = "snake_case")]
pub enum ServerMsg {
    Welcome {
        version: u8,
        id: u64,
        name: String,
        seed: i64,
        tick: u64,
        you: PlaneInfo,
        planes: Vec<PlaneInfo>,
        message: String,
    },
    Snapshot {
        tick: u64,
        time: f64,
        planes: Vec<PlaneInfo>,
        bullets: Vec<Bullet>,
        events: Vec<GameEvent>,
    },
    Chat {
        name: String,
        text: String,
    },
    Pong {
        stamp: f64,
    },
    Error {
        message: String,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "t", rename_all = "snake_case")]
pub enum ClientMsg {
    Hello {
        name: String,
    },
    Input {
        controls: Controls,
        /// Client-measured round trip in milliseconds, so the server roster can show
        /// link quality without a second protocol.
        ping: f32,
    },
    Ping {
        stamp: f64,
    },
    Respawn,
    Chat {
        text: String,
    },
}

pub fn encode<T: Serialize>(msg: &T) -> String {
    serde_json::to_string(msg)
        .unwrap_or_else(|e| format!("{{\"t\":\"error\",\"message\":\"{e}\"}}"))
}

pub fn decode<T: for<'de> Deserialize<'de>>(text: &str) -> Result<T, serde_json::Error> {
    serde_json::from_str(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> PlaneInfo {
        PlaneInfo {
            id: 7,
            name: "pilot".into(),
            pos: pos_array(Vec3::new(1.0, 2.0, 3.0)),
            vel: pos_array(Vec3::new(0.0, 0.0, 40.0)),
            rot: quat_array(Quat::IDENTITY),
            controls: Controls {
                throttle: 0.5,
                aileron: 0.0,
                elevator: 0.1,
                rudder: 0.0,
                boost: false,
                trigger: false,
                brake: false,
            },
            hp: 100.0,
            score: 300,
            gate: 3,
            lap: 1,
            alive: true,
            respawn_in: 0.0,
            ping_ms: 12.5,
        }
    }

    #[test]
    fn plane_info_round_trips() {
        let p = sample();
        let json = encode(&p);
        let back: PlaneInfo = decode(&json).unwrap();
        assert_eq!(back.id, p.id);
        assert_eq!(back.pos, p.pos);
        assert_eq!(back.rot, p.rot);
        assert_eq!(back.score, p.score);
        assert!((back.controls.throttle - 0.5).abs() < 1e-6);
    }

    #[test]
    fn client_messages_tage_by_type() {
        let json = encode(&ClientMsg::Hello { name: "ed".into() });
        assert!(json.starts_with("{\"t\":\"hello\""), "{json}");
        let back: ClientMsg = decode(&json).unwrap();
        assert!(matches!(back, ClientMsg::Hello { name } if name == "ed"));

        let json = encode(&ClientMsg::Input {
            controls: Controls {
                throttle: 1.0,
                aileron: -0.5,
                elevator: 0.25,
                rudder: 0.0,
                boost: true,
                trigger: false,
                brake: false,
            },
            ping: 42.0,
        });
        let back: ClientMsg = decode(&json).unwrap();
        match back {
            ClientMsg::Input { controls, ping } => {
                assert_eq!(ping, 42.0);
                assert_eq!(controls.throttle, 1.0);
                assert_eq!(controls.aileron, -0.5);
                assert!(controls.boost);
            }
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn snapshots_are_compact_enough() {
        let msg = ServerMsg::Snapshot {
            tick: 1,
            time: 0.0,
            planes: (0..16).map(|i| PlaneInfo { id: i, ..sample() }).collect(),
            bullets: (0..24)
                .map(|i| Bullet {
                    id: i,
                    owner: i,
                    pos: [0.0; 3],
                    vel: [0.0; 3],
                })
                .collect(),
            events: vec![GameEvent::Gate {
                id: 1,
                name: "a".into(),
                gate: 2,
                lap: 1,
                points: 120,
                total: 900,
            }],
        };
        let json = encode(&msg);
        // 16 airplanes and a full bullet field must still fit a 30 Hz budget.
        assert!(json.len() < 24_000, "snapshot is {} bytes", json.len());
    }

    #[test]
    fn state_rebuilds_from_the_wire() {
        let p = sample();
        let s = state_from_info(&p);
        assert_eq!(s.pos, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(s.vel, Vec3::new(0.0, 0.0, 40.0));
        assert_eq!(s.rot, Quat::IDENTITY);
    }
}
