#!/usr/bin/env bash
set -euo pipefail

MODEL_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$MODEL_DIR"

if ! command -v pdflatex >/dev/null 2>&1; then
    printf '%s\n' 'Missing pdflatex: install a LaTeX distribution with the packages listed in vtol_model.tex.' >&2
    exit 1
fi

mkdir -p build
for _ in 1 2; do
    if ! pdflatex -interaction=nonstopmode -halt-on-error -file-line-error \
        -no-shell-escape -output-directory=build vtol_model.tex >build/pdflatex.stdout 2>&1; then
        cat build/pdflatex.stdout >&2
        exit 1
    fi
done

printf 'Built %s/build/vtol_model.pdf\n' "$MODEL_DIR"
