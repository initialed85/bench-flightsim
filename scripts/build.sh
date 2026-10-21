#!/usr/bin/env bash
# Compile the client for wasm and emit the JS glue into web/.
# Default is the optimized build; pass --debug for a fast iteration build.
set -euo pipefail
cd "$(dirname "$0")/.."

profile=release
args=(--release)
if [[ "${1:-}" == "--debug" ]]; then
    profile=debug
    args=()
fi

cargo build --target wasm32-unknown-unknown -p flight-client "${args[@]}"

wasm-bindgen --target web --out-dir web "target/wasm32-unknown-unknown/$profile/flight_client.wasm"

if [[ "$profile" == release ]] && command -v wasm-opt >/dev/null; then
    wasm-opt -Oz --enable-bulk-memory --enable-sign-ext --enable-simd \
        --enable-nontrapping-float-to-int --enable-mutable-globals \
        web/flight_client_bg.wasm -o web/flight_client_bg.wasm
fi

# The server hands out flight_client_bg.wasm.gz when the browser accepts gzip.
gzip -9 -k -f web/flight_client_bg.wasm

echo "wrote web/flight_client.js"
echo "wrote web/flight_client_bg.wasm (.gz) $(du -h web/flight_client_bg.wasm | cut -f1)"
