#!/bin/sh
# Builds the Intersection Anchors plug-in.
# Needs the wasm32-unknown-unknown target: `rustup target add wasm32-unknown-unknown`.
set -eu
cd "$(dirname "$0")"
cargo build --release --target wasm32-unknown-unknown
out=target/wasm32-unknown-unknown/release/vectorcraft_plugin_intersect.wasm
echo "built $out ($(wc -c < "$out") bytes)"
