#!/usr/bin/env bash
# Serve the browser client and run the authoritative sim on one port (23456).
# Open http://127.0.0.1:23456 after scripts/build.sh has produced web/.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f web/flight_client_bg.wasm ]]; then
    echo "web/flight_client_bg.wasm missing; run scripts/build.sh first" >&2
    exit 1
fi
exec cargo run -q -p flight-server
