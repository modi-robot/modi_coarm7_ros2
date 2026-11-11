#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source_root="${SOURCE_ROOT:-/source}"
sdk_root="${SDK_ROOT:-/sdk}"
work_root="${WORK_ROOT:-/work}"
mkdir -p "$work_root" "$HOME" "${ROS_LOG_DIR:-$HOME/.ros/log}"
python3 "$source_root/tools/check_public_boundary.py" "$source_root"
python3 "$source_root/tools/audit_sdk.py" --sdk-root "$sdk_root" --report "$work_root/sdk_audit.json"
export CMAKE_PREFIX_PATH="$sdk_root${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
cd "$work_root"
colcon --log-base "$work_root/log" build --base-paths "$source_root" \
  --build-base "$work_root/build" --install-base "$work_root/install" \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCOARM7_ENABLE_SDK_EXTENSIONS=OFF \
  "-Dcanevo_sdk_x86_DIR=$sdk_root/lib/cmake/canevo_sdk_x86"
source "$work_root/install/setup.bash"
colcon --log-base "$work_root/test_log" test --base-paths "$source_root" --build-base "$work_root/build" \
  --packages-select coarm7_hardware coarm7_planning coarm7_controllers coarm7_bringup coarm7_diagnostics
colcon test-result --test-result-base "$work_root/build" --verbose
export ROS_DOMAIN_ID=161 ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER CYCLONEDDS_URI
python3 "$source_root/tools/clean_environment/run_fake_checks.py" --output "$work_root/integration"
