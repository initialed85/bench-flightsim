#!/usr/bin/env bash
# Iteration loop: rebuild the wasm client on change, keep the server serving it.
# The server itself is not hot-reloaded; restart scripts/run.sh after server edits.
set -euo pipefail
cd "$(dirname "$0")/.."

scripts/build.sh --debug

if command -v entr >/dev/null; then
    trap 'exit 0' INT TERM
    find client/src common/src web/index.html | entr -c scripts/build.sh --debug &
    ENTR_PID=$!
    trap 'kill $ENTR_PID 2>/dev/null || true; exit 0' INT TERM
    FLIGHT_WEB=web cargo run -q -p flight-server
else
    echo "install entr for the watch loop; running the server once instead" >&2
    cargo run -q -p flight-server
fi
