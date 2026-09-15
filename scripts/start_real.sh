#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: bash scripts/start_real.sh /absolute/config-directory [--execute]" >&2; exit 2
fi
DEPLOY_CONFIG="$(realpath "$1")"
EXECUTE=false
if [[ "${2:-}" == --execute ]]; then EXECUTE=true
elif [[ $# == 2 ]]; then echo "Unknown option: $2" >&2; exit 2; fi
for file in hardware.yaml real_feedback.yaml arms.yaml scene.yaml inspection.yaml; do
  test -f "$DEPLOY_CONFIG/$file" || { echo "Missing $DEPLOY_CONFIG/$file" >&2; exit 1; }
done
source /opt/ros/humble/setup.bash
source install/setup.bash
exec ros2 launch fr3_bolt_inspection_cell bringup.launch.py mode:=real \
  hardware:="$DEPLOY_CONFIG/hardware.yaml" real_feedback:="$DEPLOY_CONFIG/real_feedback.yaml" \
  arms:="$DEPLOY_CONFIG/arms.yaml" scene:="$DEPLOY_CONFIG/scene.yaml" inspection:="$DEPLOY_CONFIG/inspection.yaml" \
  enable_execution:="$EXECUTE"
