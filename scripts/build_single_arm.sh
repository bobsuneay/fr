#!/usr/bin/env bash
# Only single-arm UI/model and FR3/HKV drivers. No Gazebo or inspection packages.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
if [[ "$(uname -m)" != x86_64 ]]; then
  echo "Bundled FR3 SDK requires Linux x86_64" >&2
  exit 1
fi
packages=(fr3_control_panel fr3_real_bringup fairino_hardware_v3_9_7 fairino_msgs ros2_hkv_gripper)
paths=()
for package in "${packages[@]}"; do paths+=("src/$package"); done
rosdep install --from-paths "${paths[@]}" --ignore-src -r -y
# Separate output trees avoid sourcing the old simulation workspace by accident.
colcon --log-base log_single build --base-paths "${paths[@]}" \
  --build-base build_single --install-base install_single --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo "Next: source install_single/setup.bash"
echo "ros2 launch fr3_control_panel single_arm.launch.py mode:=mock enable_execution:=true"
