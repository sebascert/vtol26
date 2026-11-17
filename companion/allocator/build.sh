#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

cmake -S "$PROJECT_DIR" -B "$PROJECT_DIR/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "$@"
cmake --build "$PROJECT_DIR/build" --parallel 2
