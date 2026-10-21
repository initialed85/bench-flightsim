# bench-flightsim

A fixed-wing flight sim benchmark: deterministic Rust physics shared by an
authoritative WebSocket server and a Bevy wasm client, all on one port (23456).

## Layout

- `common/` — terrain height field, flight model (lift/drag/thrust/stall), race gates, protocol.
  Compiled into both server and client, which is what lets the client predict locally.
- `server/` — 30 Hz authoritative tick, static file server, `/ws` upgrade, one port.
- `client/` — Bevy 0.19 scene, local prediction plus snapshot reconciliation, DOM HUD.
- `web/` — `index.html` plus the generated `flight_client.js` / `flight_client_bg.wasm`.

## Run it

Prerequisites: `cargo`, `wasm-bindgen` (0.2.129), optional `wasm-opt`, the `wasm32-unknown-unknown` target.

```sh
scripts/build.sh          # wasm client -> web/ (also writes .wasm.gz for the server)
scripts/run.sh            # serves web/ and runs the sim on http://127.0.0.1:23456
```

Open http://127.0.0.1:23456 in a browser. `scripts/dev.sh` rebuilds the wasm on change.

## Keys

`W`/`S` throttle, `A`/`D` roll, `↑`/`↓` elevator (pull/push), `←`/`→` rudder,
`Shift` boost, `Space` guns, `B` wheel brake, `R` respawn. Fly the orange gates in
order; landing on the runway scores. Below the stall speed the wing stops working.

## Checks

```sh
scripts/test.sh                                   # unit tests, clippy, wasm type check
cargo run -p flight-server --example smoke        # needs a running server: joins, flies, reports
curl -s localhost:23456/health                    # tick, sim time, pilots, seed
```

Success looks like: 34 common tests pass, the wasm bundle lands in `web/`, `/health`
reports `sim_hz: 30`, and the smoke example reports a climb (`best altitude` well above
the ground) after sending full throttle.

### Check the browser client without a local browser

No Playwright browser is installed here; use Edward's headless Chromium over CDP
(`home-ops/SERVICES.md`), which is also how the page gets checked on a LAN IP:

```sh
T=$(curl -s -X PUT "https://chromium.dev.initialed85.cc/json/new")
W=$(echo "$T" | grep -o '"webSocketDebuggerUrl":"[^"]*' | cut -d'"' -f3)
ID=$(echo "$T" | grep -o '"id":"[^"]*' | head -1 | cut -d'"' -f3)
URL=http://192.168.11.128:23456/
python3 -c 'import json,sys; print(json.dumps({"id":1,"method":"Page.navigate","params":{"url":sys.argv[1]}}))' "$URL" | websocat -t "$W"
# read the HUD back out of the live page
python3 -c 'import json; print(json.dumps({"id":2,"method":"Runtime.evaluate","params":{"expression":"document.getElementById("flight").textContent","returnByValue":True}}))' | websocat -t "$W"
curl -s -X PUT "https://chromium.dev.initialed85.cc/json/close/$ID" > /dev/null
```

Success looks like `[net] + 1 pilot-…` in the server log and a flight readout such as
`71 kt 2836 ft hdg 360 deg bank 0 deg pitch 13 deg`. A `[net] …: Connection reset by peer`
line right afterwards is the headless tab being torn down, not a fault.

## Notes

- Coordinate system: `+Z` north, `-X` east, `+Y` up; heading is compass (clockwise from north).
  The airplane mesh points along local `-Z` so `PlaneState`'s rotation drops straight into `Transform`.
- Client sends inputs at 20 Hz and steps the same model at 30 Hz locally; snapshots nudge it
  back (blend for small error, snap past 120 m) so the server stays the tie-breaker.
- Server config: `FLIGHT_PORT`, `FLIGHT_SEED`, `FLIGHT_WEB`, `FLIGHT_WS` (client).
