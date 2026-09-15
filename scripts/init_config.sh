#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $# != 1 ]]; then echo "Usage: bash scripts/init_config.sh /absolute/config-directory" >&2; exit 2; fi
mkdir -p -- "$1"
cp -n src/fr3_dual_bolt_cell/config/hardware.example.yaml "$1/hardware.yaml"
cp -n src/fr3_bolt_inspection_cell/config/real_feedback.example.yaml "$1/real_feedback.yaml"
cp -n src/fr3_bolt_inspection_cell/config/tracker.example.yaml "$1/tracker.yaml"
for file in arms scene inspection; do cp -n "src/fr3_bolt_inspection_cell/config/$file.yaml" "$1/$file.yaml"; done
echo "Created configuration templates. Replace placeholder transforms/endpoints/thresholds with measured values."
