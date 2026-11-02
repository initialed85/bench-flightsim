//! WebSocket plumbing. A browser gets a `web_sys` socket; a native build gets a
//! blocking `tungstenite` reader thread. Both feed the same inbox/outbox queues so
//! the rest of the client does not care which platform it is on.

use std::collections::VecDeque;
use std::sync::{Mutex, OnceLock};

use bevy::prelude::*;
use flight_common::{decode, encode, ClientMsg, ServerMsg};

use crate::Game;

fn inbox() -> &'static Mutex<VecDeque<ServerMsg>> {
    static Q: OnceLock<Mutex<VecDeque<ServerMsg>>> = OnceLock::new();
    Q.get_or_init(Default::default)
}

fn outbox() -> &'static Mutex<VecDeque<String>> {
    static Q: OnceLock<Mutex<VecDeque<String>>> = OnceLock::new();
    Q.get_or_init(Default::default)
}

fn flag() -> &'static Mutex<bool> {
    static F: OnceLock<Mutex<bool>> = OnceLock::new();
    F.get_or_init(Default::default)
}

fn started() -> &'static Mutex<bool> {
    static S: OnceLock<Mutex<bool>> = OnceLock::new();
    S.get_or_init(Default::default)
}

fn set_connected(on: bool) {
    if let Ok(mut f) = flag().lock() {
        *f = on;
    }
}

pub fn is_connected() -> bool {
    flag().lock().map(|f| *f).unwrap_or(false)
}

pub fn send(msg: &ClientMsg) {
    if let Ok(mut q) = outbox().lock() {
        q.push_back(encode(msg));
    }
}

fn push_text(text: &str) {
    match decode::<ServerMsg>(text) {
        Ok(msg) => {
            if let Ok(mut q) = inbox().lock() {
                q.push_back(msg);
            }
        }
        Err(e) => bevy::log::warn!("[net] bad frame: {e}"),
    }
}

fn drain_inbox() -> Vec<ServerMsg> {
    inbox()
        .lock()
        .map(|mut q| q.drain(..).collect())
        .unwrap_or_default()
}

fn next_out() -> Option<String> {
    outbox().lock().ok().and_then(|mut q| q.pop_front())
}

pub fn now_ms() -> f64 {
    #[cfg(target_arch = "wasm32")]
    let t = js_sys::Date::now();
    #[cfg(not(target_arch = "wasm32"))]
    let t = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64() * 1000.0)
        .unwrap_or(0.0);
    t
}

/// Same origin as the page, which is also the port the server listens on.
pub fn server_url() -> String {
    #[cfg(target_arch = "wasm32")]
    let url = {
        let proto = web_sys::window()
            .and_then(|w| w.location().protocol().ok())
            .map(|p| if p == "https:" { "wss" } else { "ws" })
            .unwrap_or("ws");
        let host = web_sys::window()
            .and_then(|w| w.location().host().ok())
            .filter(|h| !h.is_empty())
            .unwrap_or_else(|| "127.0.0.1:23456".to_string());
        format!("{proto}://{host}/ws")
    };
    #[cfg(not(target_arch = "wasm32"))]
    let url = std::env::var("FLIGHT_WS").unwrap_or_else(|_| "ws://127.0.0.1:23456/ws".into());
    url
}

fn client_name() -> String {
    #[cfg(not(target_arch = "wasm32"))]
    let name = std::env::var("USER").unwrap_or_else(|_| "pilot".into());
    #[cfg(target_arch = "wasm32")]
    let name = "pilot".to_string();
    name
}

#[cfg(target_arch = "wasm32")]
mod transport {
    use std::sync::OnceLock;

    use wasm_bindgen::closure::Closure;
    use wasm_bindgen::JsCast;
    use web_sys::{MessageEvent, WebSocket};

    use super::{next_out, push_text, set_connected};
    use crate::ClientMsg;
    use flight_common::encode;

    fn socket() -> &'static std::sync::Mutex<Option<WebSocket>> {
        static S: OnceLock<std::sync::Mutex<Option<WebSocket>>> = OnceLock::new();
        S.get_or_init(Default::default)
    }

    pub fn open(url: &str, name: &str) -> bool {
        let Ok(ws) = WebSocket::new(url) else {
            bevy::log::error!("[net] could not open {url}");
            return false;
        };

        let on_message = Closure::wrap(Box::new(|e: MessageEvent| {
            if let Some(text) = e.data().as_string() {
                push_text(&text);
            }
        }) as Box<dyn FnMut(MessageEvent)>);
        ws.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
        on_message.forget();

        let hello = encode(&ClientMsg::Hello {
            name: name.to_string(),
        });
        let on_open = Closure::wrap(Box::new(move |_: web_sys::Event| {
            set_connected(true);
            if let Some(ws) = socket().lock().ok().and_then(|s| s.clone()) {
                ws.send_with_str(&hello).ok();
            }
        }) as Box<dyn FnMut(web_sys::Event)>);
        ws.set_onopen(Some(on_open.as_ref().unchecked_ref()));
        on_open.forget();

        let on_close = Closure::wrap(Box::new(|_: web_sys::Event| {
            set_connected(false);
        }) as Box<dyn FnMut(web_sys::Event)>);
        ws.set_onclose(Some(on_close.as_ref().unchecked_ref()));
        on_close.forget();

        *socket().lock().unwrap_or_else(|e| e.into_inner()) = Some(ws);
        true
    }

    pub fn pump() {
        let Some(ws) = socket().lock().ok().and_then(|s| s.clone()) else {
            while next_out().is_some() {}
            return;
        };
        while let Some(text) = next_out() {
            ws.send_with_str(&text).ok();
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod transport {
    use std::time::Duration;

    use super::{next_out, push_text, set_connected};
    use crate::ClientMsg;
    use flight_common::encode;

    pub fn open(url: &str, name: &str) -> bool {
        let url = url.to_string();
        let hello = encode(&ClientMsg::Hello {
            name: name.to_string(),
        });
        std::thread::spawn(move || loop {
            match tungstenite::connect(&url) {
                Ok((mut socket, _)) => {
                    set_connected(true);
                    socket.send(tungstenite::Message::text(hello.clone())).ok();
                    // The server talks at 30 Hz, so a blocking read still leaves room
                    // to flush queued inputs a few dozen times a second.
                    loop {
                        while let Some(text) = next_out() {
                            if socket.send(tungstenite::Message::text(text)).is_err() {
                                break;
                            }
                        }
                        match socket.read() {
                            Ok(msg) => {
                                if let Ok(text) = msg.into_text() {
                                    push_text(&text);
                                }
                            }
                            Err(e) => {
                                bevy::log::warn!("[net] read closed: {e}");
                                break;
                            }
                        }
                    }
                    set_connected(false);
                }
                Err(e) => {
                    set_connected(false);
                    bevy::log::warn!("[net] connect {url}: {e}");
                    std::thread::sleep(Duration::from_secs(2));
                }
            }
        });
        true
    }

    pub fn pump() {}
}

pub fn connect(mut game: ResMut<Game>) {
    let first = {
        let mut s = started().lock().unwrap_or_else(|e| e.into_inner());
        let first = !*s;
        *s = true;
        first
    };
    if first {
        transport::open(&server_url(), &client_name());
    }
    game.connected = is_connected();
}

pub fn poll(mut game: ResMut<Game>) {
    transport::pump();
    for msg in drain_inbox() {
        match msg {
            ServerMsg::Welcome {
                id,
                name,
                you,
                message,
                ..
            } => {
                game.apply_welcome(id, name, &you, message);
            }
            ServerMsg::Snapshot {
                tick,
                time,
                planes,
                events,
                ..
            } => {
                game.apply_snapshot(tick, time, &planes, &events);
            }
            ServerMsg::Pong { stamp } => {
                game.ping_ms = (now_ms() - stamp).max(0.0) as f32;
            }
            ServerMsg::Chat { name, text } => game.push_event(format!("{name}: {text}")),
            ServerMsg::Error { message } => game.status = format!("server: {message}"),
        }
    }
}

pub fn send_input(time: Res<Time>, mut game: ResMut<Game>) {
    let now = time.elapsed_secs() as f64;
    if !game.connected || now - game.last_send < 1.0 / 20.0 {
        return;
    }
    game.last_send = now;
    send(&ClientMsg::Input {
        controls: game.controls,
        ping: game.ping_ms,
    });
}

pub fn keep_alive(time: Res<Time>, mut game: ResMut<Game>) {
    let now = time.elapsed_secs() as f64;
    if !game.connected || now - game.last_ping < 0.5 {
        return;
    }
    game.last_ping = now;
    game.stamp = now_ms();
    send(&ClientMsg::Ping { stamp: game.stamp });
}
