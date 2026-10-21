//! The HUD is DOM, not geometry: index.html declares the elements and this writes
//! text into them. A native build prints the same lines to stdout instead.

use bevy::prelude::*;
use bevy::window::PrimaryWindow;

use crate::Game;

pub fn viewport() -> (u32, u32) {
    #[cfg(target_arch = "wasm32")]
    let size = web_sys::window().map(|win| {
        let width = win
            .inner_width()
            .ok()
            .and_then(|v| v.as_f64())
            .unwrap_or(1280.0);
        let height = win
            .inner_height()
            .ok()
            .and_then(|v| v.as_f64())
            .unwrap_or(720.0);
        let dpr = win.device_pixel_ratio().clamp(1.0, 2.0);
        ((width * dpr) as u32, (height * dpr) as u32)
    });
    #[cfg(target_arch = "wasm32")]
    let size = size.unwrap_or((1280, 720));
    #[cfg(not(target_arch = "wasm32"))]
    let size = (1280_u32, 720_u32);
    size
}

/// Match the drawing buffer to the canvas box so the view is not blurry on HiDPI.
pub fn resize(mut windows: Query<&mut Window, With<PrimaryWindow>>) {
    #[cfg(target_arch = "wasm32")]
    {
        let Some(mut window) = windows.iter_mut().next() else {
            return;
        };
        let (w, h) = viewport();
        if window.resolution.physical_width() != w || window.resolution.physical_height() != h {
            window.resolution =
                bevy::window::WindowResolution::new(w, h).with_scale_factor_override(1.0);
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let _ = &mut windows;
    }
}

#[cfg(target_arch = "wasm32")]
fn set_text(id: &str, text: &str) {
    let Some(doc) = web_sys::window().and_then(|w| w.document()) else {
        return;
    };
    let Some(el) = doc.get_element_by_id(id) else {
        return;
    };
    if el.text_content().as_deref() != Some(text) {
        el.set_text_content(Some(text));
    }
}

#[cfg(target_arch = "wasm32")]
pub fn draw(game: Res<Game>) {
    let r = &game.readout;
    set_text(
            "flight",
            &format!(
                "{:>5.0} kt   {:>5} ft   hdg {:>3.0}°   bank {:>3.0}°   pitch {:>3.0}°   {:+.1} g   {} ms",
                r.speed * 1.94384,
                (game.local.as_ref().map(|p| p.pos.y).unwrap_or(0.0) * 3.28084).round(),
                r.heading.to_degrees(),
                r.bank.to_degrees(),
                r.pitch.to_degrees(),
                r.g_load,
                game.ping_ms.round(),
            ),
        );
    set_text(
        "warn",
        &if r.stalled {
            "STALL".to_string()
        } else if r.speed < 22.0 {
            "below flying speed".to_string()
        } else {
            String::new()
        },
    );
    set_text("status", &game.status);
    set_text("board", &game.board);
    set_text(
        "log",
        &game
            .log
            .iter()
            .rev()
            .cloned()
            .collect::<Vec<_>>()
            .join("\n"),
    );
}

#[cfg(not(target_arch = "wasm32"))]
pub fn draw(game: Res<Game>, mut since: Local<f32>) {
    *since += 1.0 / 60.0;
    if *since < 1.0 {
        return;
    }
    *since = 0.0;
    let r = &game.readout;
    println!(
            "{} | {:>4.0} kt alt {:>5.0} m hdg {:>3.0} bank {:>3.0} pitch {:>3.0} cl {:+.2} g {:+.1} {}",
            game.status,
            r.speed * 1.94384,
            game.local.as_ref().map(|p| p.pos.y).unwrap_or(0.0),
            r.heading.to_degrees(),
            r.bank.to_degrees(),
            r.pitch.to_degrees(),
            r.cl,
            r.g_load,
            if r.stalled { "STALL" } else { "" }
        );
    println!("{}", game.board);
}
