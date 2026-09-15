#!/usr/bin/env bash
# ROS 2 setup files use variables that may be unset; nounset would abort while sourcing them.
set -eo pipefail

# One-command launcher for the FR3 + HKV panel.
# Usage:
#   bash start_panel.sh                 # mock robot + MoveIt + RViz + UI
#   bash start_panel.sh --real          # real bringup + UI (requires real.yaml)
#   bash start_panel.sh --ui-only       # UI only, no robot backend

MODE="mock"
for arg in "$@"; do
  case "$arg" in
    --mock) MODE="mock" ;;
    --real) MODE="real" ;;
    --ui-only) MODE="ui-only" ;;
    -h|--help) sed -n '1,14p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg" >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${FR3_PANEL_WS:-}" ]]; then
  WS_ROOT="$(cd -- "$FR3_PANEL_WS" && pwd)"
elif [[ -f "$(cd -- "$SCRIPT_DIR/../../.." && pwd)/install/setup.bash" || -f "$(cd -- "$SCRIPT_DIR/../../.." && pwd)/install_single/setup.bash" ]]; then
  # Source checkout: <ws>/src/fr3_control_panel/scripts/start_panel.sh
  WS_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
elif [[ -f "$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)/install/setup.bash" || -f "$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)/install_single/setup.bash" ]]; then
  # Installed data file: <ws>/install/fr3_control_panel/share/fr3_control_panel/scripts/...
  WS_ROOT="$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)"
else
  WS_ROOT=""
fi

source_if_exists() {
  if [[ -f "$1" ]]; then
    # shellcheck disable=SC1090
    source "$1"
  else
    echo "缺少环境文件: $1" >&2
    exit 1
  fi
}

source_if_exists /opt/ros/humble/setup.bash
if [[ -n "$WS_ROOT" ]]; then
  if [[ -f "$WS_ROOT/install_single/setup.bash" ]]; then
    source_if_exists "$WS_ROOT/install_single/setup.bash"
  else
    source_if_exists "$WS_ROOT/install/setup.bash"
  fi
else
  echo "未找到工作空间 install/setup.bash；请先 source ROS 工作空间，或设置 FR3_PANEL_WS。" >&2
  exit 1
fi

if [[ "$MODE" == "ui-only" ]]; then
  exec ros2 run fr3_control_panel panel --demo
fi

if [[ "$MODE" == "mock" ]]; then
  exec ros2 launch fr3_control_panel single_arm.launch.py mode:=mock enable_execution:=true
fi

REAL_CONFIG="${FR3_REAL_CONFIG:-$HOME/fr3_config/real.yaml}"
CELL_CONFIG="${FR3_CELL_CONFIG:-$(ros2 pkg prefix fr3_real_bringup)/share/fr3_real_bringup/config/cell.yaml}"
SERIAL_PORT="${HKV_SERIAL_PORT:-/dev/ttyACM0}"
if [[ ! -f "$REAL_CONFIG" ]]; then
  echo "缺少真实设备配置: $REAL_CONFIG" >&2
  echo "请先复制 real.example.yaml 并完成现场验收，或使用 --mock。" >&2
  exit 1
fi

exec ros2 launch fr3_control_panel single_arm.launch.py \
  mode:=real enable_execution:=true \
  real_config:="$REAL_CONFIG" cell:="$CELL_CONFIG" \
  serial_port:="$SERIAL_PORT" robot_ip:="${FR3_ROBOT_IP:-192.168.58.2}"

