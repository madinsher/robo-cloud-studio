#!/bin/bash
# Regenerate the Python ground truth of the MURKA agromarca article experiments (see README.md).
# Usage (WSL): bash gt_run.sh "<MURKA_Simulation>/simulation_ws/src/agromarca/murca_garden_ws/src" <out_dir> [hash_seed]
# Works on a COPY of murca_experiment + murca_metrics in a temporary directory (never writes next to the sources).
set -euo pipefail
SRC="$1"; OUT="$2"; HS="${3:-0}"
WD=$(mktemp -d)
cp -r "$SRC/murca_experiment" "$SRC/murca_metrics" "$WD/"
find "$WD" -name __pycache__ -prune -exec rm -rf {} +
( cd "$WD" && PYTHONHASHSEED="$HS" PYTHONPATH="murca_metrics:murca_experiment" \
    python3 -m murca_experiment.article_experiments all --runs 20 --output-dir out )
mkdir -p "$OUT"; for f in "$WD"/out/*.json; do gzip -9 -c "$f" > "$OUT/$(basename "$f").gz"; done
python3 -c "import sys,numpy,scipy; print(sys.version.split()[0], numpy.__version__, scipy.__version__)"
rm -rf "$WD"
