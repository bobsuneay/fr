#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Requires Ubuntu 22.04 with ROS 2 Humble installed" >&2; exit 1
fi
if [[ "$(uname -m)" != x86_64 ]]; then
  echo "Bundled Fairino SDK is x86_64. Obtain a matching SDK before building on ARM." >&2; exit 1
fi
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select fr3_bolt_inspection_cell fr3_dual_bolt_cell fr3_control_panel --event-handlers console_direct+
# Run hardware regression tests, separately from the legacy vendor style linters.
colcon test --packages-select ros2_hkv_gripper --ctest-args -R '^hkv_protocol$' --event-handlers console_direct+
colcon test --packages-select fairino_hardware_v3_9_7 --ctest-args -R '^feedback_watchdog$' --event-handlers console_direct+
colcon test-result --verbose
