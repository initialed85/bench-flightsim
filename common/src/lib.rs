//! Shared, deterministic game core for bench-flightsim.
//!
//! Everything here is compiled into both the authoritative server and the wasm
//! client: the same terrain, the same circuit, the same flight model. That is what
//! lets the client predict locally and the server stay the tie-breaker.

pub mod course;
pub mod flight;
pub mod protocol;
pub mod terrain;

pub use course::{crossed_gate, gate_points, gates, Gate, GATE_RADIUS, N_GATES};
pub use flight::{
    bank_of, deg, forward_from_heading, forward_of, heading_of, lift_coefficient, pitch_of,
    quat_align_y, quat_from_forward, right_of, step_plane, up_of, Aircraft, Controls, Crash,
    Outcome, PlaneState, Readout, AIRCRAFT, G,
};
pub use protocol::{
    decode, encode, pos_array, quat_array, state_from_info, Bullet, ClientMsg, GameEvent, PlaneInfo,
    ServerMsg, PROTOCOL_VERSION,
};
pub use terrain::{
    height_raw, is_runway, Terrain, GRID_N, GRID_STEP, RUNWAY_Y, RUNWAY_HALF_LENGTH,
    RUNWAY_HALF_WIDTH, SEA_LEVEL, SEED, WORLD_HALF,
};
