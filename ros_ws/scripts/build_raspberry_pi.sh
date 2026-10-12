#!/usr/bin/env bash
# Run after sourcing the ROS 2 distribution installed on the Raspberry Pi.
set -eo pipefail
robot_ws_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_VERSION:-}" != 2 || -z "${ROS_DISTRO:-}" ]]; then
  echo 'Please source /opt/ros/<your_distro>/setup.bash first.' >&2
  exit 1
fi
robot_jobs="${ROBOT_BUILD_JOBS:-2}"
robot_sdk_prefix="$robot_ws_dir/local/elite_sdk"
cmake -S "$robot_ws_dir/../Elite_Robots_CS_SDK" \
  -B "$robot_ws_dir/build_elite_sdk" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$robot_sdk_prefix" \
  -DELITE_COMPILE_TESTS=OFF -DELITE_COMPILE_EXAMPLES=OFF -DELITE_COMPILE_DOC=OFF
cmake --build "$robot_ws_dir/build_elite_sdk" --parallel "$robot_jobs"
cmake --install "$robot_ws_dir/build_elite_sdk"
export CMAKE_PREFIX_PATH="$robot_sdk_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$robot_sdk_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL="$robot_jobs"
cd "$robot_ws_dir"
colcon build --base-paths src --executor sequential --packages-select \
  eli_common_interface eli_dashboard_interface eli_cs_controllers \
  eli_cs_robot_description eli_cs_robot_driver eli_cs_robot_moveit_config \
  eli_cs_robot_serial eli_cs_robot_usr masc_chassis_can_sdk \
  masc_chassis_msgs masc_chassis_ros_driver robot_task_control \
  hik_camera_ros2_driver camer_diver masc_chassis_user_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DMASC_CHASSIS_CAN_SDK_BUILD_TESTS=OFF \
    -DMASC_CHASSIS_CAN_SDK_BUILD_EXAMPLES=OFF
printf '%s\n' 'Build finished. Source scripts/raspberry_pi_env.bash before launching.'
