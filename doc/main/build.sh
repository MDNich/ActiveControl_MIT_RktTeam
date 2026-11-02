#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p .build
latexmk -pdf -interaction=nonstopmode -halt-on-error -file-line-error -outdir=.build main.tex
cp .build/main.pdf main.pdf
