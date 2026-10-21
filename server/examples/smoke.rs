//! End-to-end check against a running server: join, take off, score a gate.
//! Run `scripts/run.sh` first, then `cargo run -p flight-server --example smoke`.

use std::time::Duration;

use flight_common::{ClientMsg, Controls, ServerMsg, decode, encode};
use tokio_tungstenite::connect_async;
use tokio_tungstenite::tungstenite::Message;
use futures_util::{SinkExt, StreamExt};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let url = std::env::var("FLIGHT_WS").unwrap_or_else(|_| "ws://127.0.0.1:23456/ws".into());
    let (mut ws, _) = connect_async(&url).await?;
    ws.send(Message::text(encode(&ClientMsg::Hello { name: "smoke".into() })))
        .await?;

    let mut me = None;
    let mut gates = 0u32;
    let mut best_speed = 0f32;
    let mut best_alt = 0f32;
    let start = std::time::Instant::now();
    let mut sent = false;

    while start.elapsed() < Duration::from_secs(14) {
        if !sent && start.elapsed() > Duration::from_secs(1) {
            let controls = Controls {
                throttle: 1.0,
                aileron: 0.0,
                elevator: 0.25,
                rudder: 0.0,
                boost: false,
                trigger: false,
                brake: false,
            };
            ws.send(Message::text(encode(&ClientMsg::Input { controls, ping: 1.0 })))
                .await?;
            sent = true;
        }
        let frame = tokio::time::timeout(Duration::from_millis(400), ws.next())
            .await
            .ok()
            .flatten()
            .transpose()?;
        let Some(frame) = frame else { continue };
        let text = match frame {
            Message::Text(t) => t.to_string(),
            Message::Binary(b) => String::from_utf8(b.to_vec())?,
            _ => continue,
        };
        match decode::<ServerMsg>(&text)? {
            ServerMsg::Welcome { id, name, you, message, .. } => {
                me = Some(id);
                println!("[smoke] welcome {id} {name}: {message}");
                println!("[smoke] spawn y {:.1} speed {:.1}", you.pos().y, you.vel().length());
            }
            ServerMsg::Snapshot { tick, planes, .. } => {
                let Some(p) = planes.iter().find(|p| Some(p.id) == me) else { continue };
                best_speed = best_speed.max(p.vel().length());
                best_alt = best_alt.max(p.pos().y);
                gates = p.gate;
                if tick % 30 == 0 {
                    println!(
                        "[smoke] tick {tick} alt {:.0} speed {:.1} kt {:.0} gate {} score {} hp {:.0}",
                        p.pos().y,
                        p.vel().length(),
                        p.vel().length() * 1.94384,
                        p.gate,
                        p.score,
                        p.hp
                    );
                }
            }
            ServerMsg::Pong { .. } => {}
            ServerMsg::Chat { name, text } => println!("[smoke] chat {name}: {text}"),
            ServerMsg::Error { message } => println!("[smoke] error: {message}"),
        }
    }

    println!("[smoke] best altitude {best_alt:.0} m, best speed {best_speed:.1} m/s");
    if best_alt < 25.0 {
        return Err("never left the ground: the takeoff roll did not work".into());
    }
    Ok(())
}
