#!/usr/bin/env bash
# Unit tests for the shared core and the server, plus a wasm client type check.
set -euo pipefail
cd "$(dirname "$0")/.."
cargo test --workspace "${@:-}"
cargo clippy -p flight-common -p flight-server -p flight-client --all-targets -- -D warnings
cargo check -p flight-client --target wasm32-unknown-unknown
