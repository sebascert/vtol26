#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
COMPILE_COMMANDS="$BUILD_DIR/compile_commands.json"

FLAGS=(
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
)

cmake \
    -S "$PROJECT_DIR" \
    -B "$BUILD_DIR" \
    "${FLAGS[@]}"

cmake \
    --build "$BUILD_DIR" \
    --parallel

cp "$COMPILE_COMMANDS" "$PROJECT_DIR/compile_commands.json"
